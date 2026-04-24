from fastapi import FastAPI, HTTPException, Query
from fastapi.responses import Response, FileResponse, StreamingResponse
from pydantic import BaseModel

import hashlib
import pathlib
import select
import struct
import subprocess
import time
from typing import Iterator, Optional

# ============================================================
# Android Termux + Ubuntu(proot) + Piper-plus 用
# ESP32 (StackChan Minimal) 互換:
#   GET  /tts.wav?text=...&length_scale=1.0&language=ja
#   POST /tts {"text":"...", "length_scale":1.0, "language":"ja"}
#
# 比較用 endpoint:
#   GET  /tts_stream.wav?text=...&length_scale=1.0&language=ja
#   -> 完成済みWAVを FileResponse で返す
#
# 実験用 endpoint:
#   GET  /tts_live.wav?text=...&length_scale=1.0&language=ja
#   -> C++版 piper の --output-raw --streaming を使い、
#      サーバー側で unknown-size WAV header を付けて逐次返す
# ============================================================

PIPER_EXE = "/root/piper/piperplus/piper/bin/piper"
MODEL_NAME = "tsukuyomi"

CACHE_DIR = pathlib.Path("/root/piper/cache")
CACHE_DIR.mkdir(parents=True, exist_ok=True)

LENGTH_SCALE_MIN = 0.5
LENGTH_SCALE_MAX = 2.0
LENGTH_SCALE_DEFAULT = 1.0

LANGUAGE_ALIASES = {
    "ja": "ja",
    "ja-jp": "ja",
    "jp": "ja",
    "en": "en",
    "en-us": "en",
    "en-gb": "en",
    "zh": "zh",
    "zh-cn": "zh",
    "zh-tw": "zh",
}
LANGUAGE_DEFAULT = "ja"

# live 経路用パラメータ
LIVE_CHUNK_BYTES = 4096
LIVE_FIRST_CHUNK_TIMEOUT_SEC = 15.0

# Tsukuyomi の想定フォーマット
WAV_SAMPLE_RATE = 22050
WAV_BITS_PER_SAMPLE = 16
WAV_CHANNELS = 1

app = FastAPI(title="Piper-plus TTS Server (Android)", version="1.2.0-streaming-exp")


def _log(msg: str) -> None:
    print(f"[tts_server] {time.strftime('%Y-%m-%d %H:%M:%S')} {msg}", flush=True)


def _clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def _normalize_language(language: Optional[str]) -> str:
    if not language:
        return LANGUAGE_DEFAULT
    key = language.strip().lower()
    if not key:
        return LANGUAGE_DEFAULT
    return LANGUAGE_ALIASES.get(key, key)


def _cache_key(text: str, length_scale: float, language: Optional[str]) -> str:
    lang = _normalize_language(language)
    src = f"{text}\n{length_scale:.4f}\n{lang}"
    return hashlib.sha256(src.encode("utf-8")).hexdigest()


def _build_common_cmd(length_scale: float, language: Optional[str]) -> list[str]:
    ls = _clamp(length_scale, LENGTH_SCALE_MIN, LENGTH_SCALE_MAX)
    lang = _normalize_language(language)
    return [
        PIPER_EXE,
        "-q",
        "--model", MODEL_NAME,
        "--length_scale", f"{ls:.4f}",
        "--language", lang,
    ]


def _synthesize(text: str, length_scale: float, language: Optional[str] = None) -> pathlib.Path:
    cmd = _build_common_cmd(length_scale, language)

    ls = _clamp(length_scale, LENGTH_SCALE_MIN, LENGTH_SCALE_MAX)
    lang = _normalize_language(language)
    out_path = CACHE_DIR / f"{_cache_key(text, ls, lang)}.wav"

    if out_path.exists() and out_path.stat().st_size > 44:
        return out_path

    cmd = cmd + ["--output_file", str(out_path)]

    try:
        result = subprocess.run(
            cmd,
            input=text.encode("utf-8"),
            capture_output=True,
            timeout=30,
            check=False,
        )
    except FileNotFoundError:
        raise HTTPException(status_code=500, detail=f"piper not found: {PIPER_EXE}")
    except subprocess.TimeoutExpired:
        raise HTTPException(status_code=504, detail="piper timed out")

    if result.returncode != 0:
        stderr = result.stderr.decode("utf-8", errors="replace")
        stdout = result.stdout.decode("utf-8", errors="replace")
        raise HTTPException(
            status_code=500,
            detail=f"piper error\nstdout={stdout}\nstderr={stderr}"
        )

    if not out_path.exists() or out_path.stat().st_size <= 44:
        raise HTTPException(status_code=500, detail="WAV output is empty")

    return out_path


def _wav_header_unknown_size(
    sample_rate: int = WAV_SAMPLE_RATE,
    bits_per_sample: int = WAV_BITS_PER_SAMPLE,
    channels: int = WAV_CHANNELS,
) -> bytes:
    byte_rate = sample_rate * channels * bits_per_sample // 8
    block_align = channels * bits_per_sample // 8

    # RIFFサイズとdataサイズは長さ不定のため 0xFFFFFFFF
    return b"".join([
        b"RIFF",
        struct.pack("<I", 0xFFFFFFFF),
        b"WAVE",
        b"fmt ",
        struct.pack("<I", 16),          # PCM fmt chunk size
        struct.pack("<H", 1),           # PCM format
        struct.pack("<H", channels),
        struct.pack("<I", sample_rate),
        struct.pack("<I", byte_rate),
        struct.pack("<H", block_align),
        struct.pack("<H", bits_per_sample),
        b"data",
        struct.pack("<I", 0xFFFFFFFF),
    ])


def _start_live_piper(text: str, length_scale: float, language: Optional[str]) -> tuple[subprocess.Popen, bytes]:
    cmd = _build_common_cmd(length_scale, language) + ["--output-raw", "--streaming"]

    try:
        proc = subprocess.Popen(
            cmd,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            bufsize=0,
        )
    except FileNotFoundError:
        raise HTTPException(status_code=500, detail=f"piper not found: {PIPER_EXE}")

    assert proc.stdin is not None
    assert proc.stdout is not None
    assert proc.stderr is not None

    try:
        proc.stdin.write(text.encode("utf-8"))
        proc.stdin.close()
    except Exception:
        if proc.poll() is None:
            proc.kill()
        raise HTTPException(status_code=500, detail="failed to send text to piper")

    ready, _, _ = select.select([proc.stdout], [], [], LIVE_FIRST_CHUNK_TIMEOUT_SEC)

    if not ready:
        if proc.poll() is None:
            proc.terminate()
            try:
                proc.wait(timeout=2)
            except subprocess.TimeoutExpired:
                proc.kill()
        stderr = proc.stderr.read().decode("utf-8", errors="replace")
        raise HTTPException(
            status_code=504,
            detail=f"live stream timed out before first PCM chunk\nstderr={stderr}"
        )

    first_chunk = proc.stdout.read(LIVE_CHUNK_BYTES)

    if not first_chunk:
        rc = proc.wait(timeout=5)
        stderr = proc.stderr.read().decode("utf-8", errors="replace")
        raise HTTPException(
            status_code=500,
            detail=f"live stream produced no audio (rc={rc})\nstderr={stderr}"
        )

    return proc, first_chunk


def _iter_live_wav(proc: subprocess.Popen, first_chunk: bytes) -> Iterator[bytes]:
    assert proc.stdout is not None
    assert proc.stderr is not None

    total_bytes = 0
    chunk_count = 0
    t0 = time.perf_counter()

    try:
        header = _wav_header_unknown_size()
        yield header

        total_bytes += len(first_chunk)
        chunk_count += 1
        _log(f"/tts_live.wav first chunk: bytes={len(first_chunk)}")
        yield first_chunk

        while True:
            chunk = proc.stdout.read(LIVE_CHUNK_BYTES)
            if not chunk:
                break
            total_bytes += len(chunk)
            chunk_count += 1
            yield chunk

        rc = proc.wait(timeout=10)
        stderr = proc.stderr.read().decode("utf-8", errors="replace").strip()
        elapsed_ms = int((time.perf_counter() - t0) * 1000)

        if rc != 0:
            _log(
                f"/tts_live.wav piper error: rc={rc}, chunks={chunk_count}, "
                f"bytes={total_bytes}, elapsed_ms={elapsed_ms}, stderr={stderr}"
            )
        else:
            _log(
                f"/tts_live.wav done: rc={rc}, chunks={chunk_count}, "
                f"bytes={total_bytes}, elapsed_ms={elapsed_ms}"
            )
            if stderr:
                _log(f"/tts_live.wav stderr: {stderr}")

    except GeneratorExit:
        _log("/tts_live.wav client disconnected")
        raise
    finally:
        if proc.poll() is None:
            proc.terminate()
            try:
                proc.wait(timeout=2)
            except subprocess.TimeoutExpired:
                proc.kill()


@app.get("/health")
def health():
    return {
        "status": "ok",
        "piper": PIPER_EXE,
        "model": MODEL_NAME,
        "cache_dir": str(CACHE_DIR),
        "default_length_scale": LENGTH_SCALE_DEFAULT,
        "default_language": LANGUAGE_DEFAULT,
        "supported_language_aliases": sorted(LANGUAGE_ALIASES.keys()),
        "live_endpoint": "/tts_live.wav",
        "live_chunk_bytes": LIVE_CHUNK_BYTES,
        "live_first_chunk_timeout_sec": LIVE_FIRST_CHUNK_TIMEOUT_SEC,
        "wav_sample_rate": WAV_SAMPLE_RATE,
        "wav_bits_per_sample": WAV_BITS_PER_SAMPLE,
        "wav_channels": WAV_CHANNELS,
    }


class TTSRequest(BaseModel):
    text: str
    length_scale: Optional[float] = LENGTH_SCALE_DEFAULT
    language: Optional[str] = LANGUAGE_DEFAULT


@app.post("/tts")
async def tts_post(req: TTSRequest):
    if not req.text.strip():
        raise HTTPException(status_code=400, detail="text is empty")

    out_path = _synthesize(
        req.text,
        req.length_scale if req.length_scale is not None else LENGTH_SCALE_DEFAULT,
        req.language,
    )

    wav_bytes = out_path.read_bytes()

    return Response(
        content=wav_bytes,
        media_type="audio/wav",
        headers={
            "Content-Disposition": "inline; filename=tts.wav",
            "Content-Length": str(len(wav_bytes)),
        },
    )


@app.get("/tts.wav")
async def tts_get(
    text: str = Query(..., description="読み上げテキスト（URLエンコード済み）"),
    length_scale: float = Query(LENGTH_SCALE_DEFAULT, description="話速 (0.5〜2.0、1.0が標準)"),
    language: Optional[str] = Query(LANGUAGE_DEFAULT, description="言語指定: ja / en / zh"),
):
    if not text.strip():
        raise HTTPException(status_code=400, detail="text is empty")

    out_path = _synthesize(text, length_scale, language)
    wav_bytes = out_path.read_bytes()

    return Response(
        content=wav_bytes,
        media_type="audio/wav",
        headers={
            "Content-Disposition": "inline; filename=tts.wav",
            "Content-Length": str(len(wav_bytes)),
        },
    )


@app.get("/tts_stream.wav")
async def tts_stream_get(
    text: str = Query(..., description="読み上げテキスト（URLエンコード済み）"),
    length_scale: float = Query(LENGTH_SCALE_DEFAULT, description="話速 (0.5〜2.0、1.0が標準)"),
    language: Optional[str] = Query(LANGUAGE_DEFAULT, description="言語指定: ja / en / zh"),
):
    if not text.strip():
        raise HTTPException(status_code=400, detail="text is empty")

    out_path = _synthesize(text, length_scale, language)

    return FileResponse(
        path=str(out_path),
        media_type="audio/wav",
        filename="tts_stream.wav",
    )


@app.get("/tts_live.wav")
async def tts_live_get(
    text: str = Query(..., description="読み上げテキスト（URLエンコード済み）"),
    length_scale: float = Query(LENGTH_SCALE_DEFAULT, description="話速 (0.5〜2.0、1.0が標準)"),
    language: Optional[str] = Query(LANGUAGE_DEFAULT, description="言語指定: ja / en / zh"),
):
    if not text.strip():
        raise HTTPException(status_code=400, detail="text is empty")

    t0 = time.perf_counter()
    proc, first_chunk = _start_live_piper(text, length_scale, language)
    first_wait_ms = int((time.perf_counter() - t0) * 1000)

    _log(
        f"/tts_live.wav start ok: first_wait_ms={first_wait_ms}, "
        f"first_chunk_bytes={len(first_chunk)}, language={_normalize_language(language)}, "
        f"length_scale={_clamp(length_scale, LENGTH_SCALE_MIN, LENGTH_SCALE_MAX):.4f}"
    )

    return StreamingResponse(
        _iter_live_wav(proc, first_chunk),
        media_type="audio/wav",
        headers={
            "Content-Disposition": "inline; filename=tts_live.wav",
            "Cache-Control": "no-store",
            "X-TTS-Mode": "live-streaming",
        },
    )