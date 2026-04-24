#pragma once

#include <WiFiClient.h>
#include <M5Unified.h>
#include "AudioOutputM5Speaker.h"

// ============================================================
// WAVパーサー内部構造体
// ============================================================
struct WavInfo {
    uint16_t channels    = 0;
    uint32_t sampleRate  = 0;
    uint16_t bitsPerSamp = 0;
    uint32_t dataSize    = 0;  // 0 = unknown (chunked等)
    bool     valid       = false;
};

struct HttpBodyState {
    bool   chunked        = false;
    size_t chunkRemaining = 0;
    bool   finished       = false;
};

static inline uint16_t _readU16LE(const uint8_t* p) {
    return (uint16_t)(p[0] | (p[1] << 8));
}
static inline uint32_t _readU32LE(const uint8_t* p) {
    return (uint32_t)(p[0] | (p[1] << 8) | (p[2] << 16) | (p[3] << 24));
}

// ============================================================
// HTTPヘッダ読み飛ばし
// ============================================================
static int skipHttpHeader(WiFiClient& client, HttpBodyState& body, uint32_t timeoutMs = 5000) {
    int statusCode = -1;
    bool firstLine = true;
    uint32_t t0 = millis();

    body.chunked = false;
    body.chunkRemaining = 0;
    body.finished = false;

    while (millis() - t0 < timeoutMs) {
        if (!client.connected() && !client.available()) break;
        if (!client.available()) { delay(5); continue; }

        String line = client.readStringUntil('\n');

        if (firstLine) {
            firstLine = false;
            int sp = line.indexOf(' ');
            if (sp > 0 && line.length() >= sp + 4) {
                statusCode = line.substring(sp + 1, sp + 4).toInt();
            }
            continue;
        }

        line.trim();
        if (line.length() == 0) break;

        String lower = line;
        lower.toLowerCase();
        if (lower.startsWith("transfer-encoding:") && lower.indexOf("chunked") >= 0) {
            body.chunked = true;
        }
    }
    return statusCode;
}

static bool readBytes(WiFiClient& client, HttpBodyState& body, uint8_t* buf, size_t len, uint32_t timeoutMs = 3000) {
    size_t got = 0;
    uint32_t t0 = millis();

    auto readLine = [&](String& outLine, uint32_t lineTimeoutMs) -> bool {
        outLine = "";
        uint32_t lt0 = millis();
        while (millis() - lt0 < lineTimeoutMs) {
            if (!client.connected() && !client.available()) break;
            if (!client.available()) { delay(1); continue; }
            char c = (char)client.read();
            if (c == '\r') continue;
            if (c == '\n') return true;
            outLine += c;
        }
        return false;
    };

    while (got < len) {
        if (millis() - t0 > timeoutMs) {
            Serial.printf("[HTTP] readBytes timeout (got %u / %u)\n", (unsigned)got, (unsigned)len);
            return false;
        }

        if (body.finished) return false;

        if (!body.chunked) {
            if (!client.connected() && !client.available()) break;
            if (!client.available()) { delay(2); continue; }
            int n = client.read(buf + got, len - got);
            if (n > 0) { got += (size_t)n; t0 = millis(); }
            continue;
        }

        if (body.chunkRemaining == 0) {
            String sizeLine;
            if (!readLine(sizeLine, timeoutMs)) {
                Serial.println("[HTTP] failed to read chunk size line");
                return false;
            }

            sizeLine.trim();
            int semi = sizeLine.indexOf(';');
            if (semi >= 0) sizeLine = sizeLine.substring(0, semi);

            body.chunkRemaining = (size_t)strtoul(sizeLine.c_str(), nullptr, 16);

            if (body.chunkRemaining == 0) {
                while (true) {
                    String trailer;
                    if (!readLine(trailer, timeoutMs)) break;
                    trailer.trim();
                    if (trailer.length() == 0) break;
                }
                body.finished = true;
                return false;
            }
        }

        while (body.chunkRemaining > 0 && got < len) {
            if (!client.connected() && !client.available()) break;
            if (!client.available()) { delay(1); continue; }

            size_t want = len - got;
            if (want > body.chunkRemaining) want = body.chunkRemaining;

            int n = client.read(buf + got, want);
            if (n > 0) {
                got += (size_t)n;
                body.chunkRemaining -= (size_t)n;
                t0 = millis();
            }
        }

        if (body.chunkRemaining == 0) {
            uint8_t crlf[2];
            size_t crlfGot = 0;
            uint32_t ct0 = millis();
            while (crlfGot < 2 && millis() - ct0 < timeoutMs) {
                if (!client.connected() && !client.available()) break;
                if (!client.available()) { delay(1); continue; }
                int n = client.read(crlf + crlfGot, 2 - crlfGot);
                if (n > 0) crlfGot += (size_t)n;
            }
            if (crlfGot < 2) {
                Serial.println("[HTTP] failed to consume chunk CRLF");
                return false;
            }
        }
    }
    return (got == len);
}


static inline String piperLanguageFromLangCode() {
    String code = LANG_CODE;
    code.toLowerCase();
    if (code.startsWith("en")) return "en";
    if (code.startsWith("zh")) return "zh";
    return "ja";
}


// ===== TTS比較テスト用フラグ =====
// true  : /tts_live.wav
// false : /tts_stream.wav
 static bool g_use_live_tts = true;
// static bool g_use_live_tts = false;

// リクエストごとの timing ログ識別用
static uint32_t g_piper_req_seq = 0;

static inline const char* piperTtsPath() {
    return g_use_live_tts ? "/tts_live.wav" : "/tts_stream.wav";
}

// static inline const char* piperTtsPath() {
//     return "/tts_live.wav";   // live実験時。安定版へ戻す場合は "/tts_stream.wav"
// }



static inline bool shouldCancelTtsNow(bool allow_cancel) {
    if (g_tts_cancel_requested.load() || g_estop_active) {
        return true;
    }
    if (!allow_cancel) return false;
    M5.update();
    if (M5.BtnA.isHolding() || M5.BtnA.wasHold()) {
        g_tts_cancel_requested.store(true);
        return true;
    }
    return false;
}

static bool parseWavHeader(WiFiClient& client, HttpBodyState& body, WavInfo& info) {
    uint8_t riff[12];
    if (!readBytes(client, body, riff, 12)) { Serial.println("[WAV] Failed to read RIFF header"); return false; }
    if (riff[0]!='R'||riff[1]!='I'||riff[2]!='F'||riff[3]!='F') { Serial.println("[WAV] Not a RIFF file"); return false; }
    if (riff[8]!='W'||riff[9]!='A'||riff[10]!='V'||riff[11]!='E') { Serial.println("[WAV] Not a WAVE file"); return false; }

    bool fmtFound  = false;
    bool dataFound = false;
    const int MAX_CHUNKS = 16;

    for (int ci = 0; ci < MAX_CHUNKS && !dataFound; ci++) {
        uint8_t hdr[8];
        if (!readBytes(client, body, hdr, 8)) { Serial.println("[WAV] Failed to read chunk header"); return false; }
        char chunkId[5] = {(char)hdr[0],(char)hdr[1],(char)hdr[2],(char)hdr[3],0};
        uint32_t chunkSize = _readU32LE(hdr + 4);

        Serial.printf("[WAV] chunk='%s' size=%u\n", chunkId, (unsigned)chunkSize);

        if (memcmp(chunkId, "fmt ", 4) == 0) {
            if (chunkSize < 16) { Serial.println("[WAV] fmt chunk too small"); return false; }
            uint8_t fmt[16];
            if (!readBytes(client, body, fmt, 16)) return false;

            uint16_t audioFormat = _readU16LE(fmt + 0);  // 1=PCM
            info.channels    = _readU16LE(fmt + 2);
            info.sampleRate  = _readU32LE(fmt + 4);
            info.bitsPerSamp = _readU16LE(fmt + 14);

            Serial.printf("[WAV] fmt: format=%u ch=%u rate=%u bits=%u\n",
                (unsigned)audioFormat, (unsigned)info.channels, (unsigned)info.sampleRate, (unsigned)info.bitsPerSamp);

            if (audioFormat != 1) { Serial.println("[WAV] Non-PCM format not supported"); return false; }
            fmtFound = true;

            uint32_t extra = chunkSize - 16;
            if ((chunkSize & 1) != 0) extra++;
            while (extra > 0) { uint8_t tmp; if (!readBytes(client, body, &tmp, 1)) break; extra--; }
        }
        else if (memcmp(chunkId, "data", 4) == 0) {
            info.dataSize = chunkSize;
            dataFound = true;
        }
        else {
            uint32_t skip = chunkSize;
            if ((chunkSize & 1) != 0) skip++;
            const size_t TMP = 64;
            uint8_t tmp[TMP];
            while (skip > 0) {
                size_t n = (skip > TMP) ? TMP : skip;
                if (!readBytes(client, body, tmp, n)) break;
                skip -= (uint32_t)n;
            }
        }
    }

    if (!fmtFound)  { Serial.println("[WAV] fmt chunk not found");  return false; }
    if (!dataFound) { Serial.println("[WAV] data chunk not found"); return false; }

    info.valid = true;
    return true;
}


// static void streamPcmToSpeaker(WiFiClient& client,
//                                HttpBodyState& body,
//                                const WavInfo& info,
//                                uint32_t timeoutMs,
//                                bool allow_cancel)
// {
//     // prebuffer: データが溜まるまで少し待つ（ただし長くは待たない）
//     // プリフェッチ済みの場合、TCPバッファにデータがあるはずなので即座に開始できる。
//     // データがまだない場合（同期再生時）は最大500msだけ待つ。
//     const size_t SPK_PREBUFFER_SIZE = 4096;
//     uint32_t preT0 = millis();
//     if (!body.chunked) {
//         while ((size_t)client.available() < SPK_PREBUFFER_SIZE) {
//             if (!client.connected()) break;
//             if (millis() - preT0 > 500) break;  // 最大500msで切り上げ
//             delay(2);
//         }
//         Serial.printf("[WAV] prebuffer=%u\n", (unsigned)client.available());
//     } else {
//         // chunked時は client.available() が本文の実データ量と一致しないため、短い待ちだけ行う
//         while (!client.available()) {
//             if (!client.connected()) break;
//             if (millis() - preT0 > 100) break;
//             delay(2);
//         }
//         Serial.printf("[WAV] prebuffer=chunked avail=%u\n", (unsigned)client.available());
//     }

//     const size_t OUT_SAMPLES = 256;
//     const size_t MAX_IN_FRAME_BYTES = 4;
//     const size_t INBUF_BYTES = OUT_SAMPLES * MAX_IN_FRAME_BYTES;
//     const size_t IN_FRAME_BYTES = (info.bitsPerSamp / 8) * info.channels;

//     static uint8_t inbuf[INBUF_BYTES];
//     static int16_t outBuf[8][OUT_SAMPLES];
//     static uint8_t bufIndex = 0;
//     bufIndex = 0;

//     uint32_t remaining = info.dataSize;
//     bool unknownSize   = (remaining == 0 || remaining == 0xFFFFFFFF);

//     uint32_t t0 = millis();
//     uint32_t lastActivity = millis();

//     while (true) {
//         if (millis() - t0 > timeoutMs) {
//             Serial.println("[WAV] stream timeout");
//             break;
//         }

//         if (allow_cancel) {
//             M5.update();
//             if (M5.BtnA.wasHold()) {
//                 Serial.println("[WAV] cancelled by button hold");
//                 M5.Speaker.stop();
//                 break;
//             }
//         }

//         if (!unknownSize && remaining == 0) break;

//         size_t wantBytes = OUT_SAMPLES * IN_FRAME_BYTES;
//         if (!unknownSize && wantBytes > remaining) {
//             wantBytes = remaining;
//         }
//         if (wantBytes < IN_FRAME_BYTES) break;

//         size_t toRead = wantBytes;

//         if (!body.chunked) {
//             uint32_t waitT0 = millis();
//             while ((size_t)client.available() < wantBytes) {
//                 if (!client.connected()) break;
//                 if (millis() - waitT0 > 2000) break;
//                 delay(1);
//             }

//             toRead = client.available();
//             if (toRead > wantBytes) toRead = wantBytes;

//             if (toRead < IN_FRAME_BYTES) {
//                 if (!client.connected()) break;
//                 if (millis() - lastActivity > 3000) {
//                     Serial.println("[WAV] no data 3s, stopping");
//                     break;
//                 }
//                 delay(1);
//                 continue;
//             }
//         }

//         if (!readBytes(client, body, inbuf, toRead, 3000)) {
//             if (body.finished) break;
//             if (!client.connected()) break;
//             if (millis() - lastActivity > 3000) {
//                 Serial.println("[WAV] no data 3s, stopping");
//                 break;
//             }
//             delay(1);
//             continue;
//         }

//         int n = (int)toRead;
//         lastActivity = millis();
//         if (!unknownSize) remaining -= (uint32_t)n;

//         while (M5.Speaker.isPlaying(m5spk_virtual_channel) == 2) {
//             delay(1);
//             if (allow_cancel) {
//                 M5.update();
//                 if (M5.BtnA.wasHold()) {
//                     Serial.println("[WAV] cancelled while waiting queue");
//                     M5.Speaker.stop();
//                     return;
//                 }
//             }
//         }

//         int16_t* out = outBuf[bufIndex];
//         bufIndex = (bufIndex + 1) % 8;

//         size_t outCount = 0;
//         for (int i = 0; i + (int)IN_FRAME_BYTES <= n && outCount < OUT_SAMPLES; i += (int)IN_FRAME_BYTES) {
//             int16_t mono = 0;

//             if (info.bitsPerSamp == 16) {
//                 int16_t L = (int16_t)_readU16LE(inbuf + i);
//                 int16_t R = (info.channels >= 2) ? (int16_t)_readU16LE(inbuf + i + 2) : L;
//                 mono = (info.channels >= 2) ? (int16_t)(((int32_t)L + (int32_t)R) / 2) : L;
//             } else if (info.bitsPerSamp == 8) {
//                 int16_t L = ((int16_t)inbuf[i] - 128) << 8;
//                 int16_t R = (info.channels >= 2) ? (((int16_t)inbuf[i + 1] - 128) << 8) : L;
//                 mono = (info.channels >= 2) ? (int16_t)(((int32_t)L + (int32_t)R) / 2) : L;
//             } else {
//                 continue;
//             }

//             out[outCount++] = mono;
//         }

//         if (outCount == 0) continue;

//         M5.Speaker.playRaw(
//             out,
//             outCount,
//             info.sampleRate,
//             false,
//             1,
//             m5spk_virtual_channel,
//             false
//         );
//     }

//     // drain: スピーカーキューの再生完了を待つ（stop()は呼ばない）
//     uint32_t drainT0 = millis();
//     while (M5.Speaker.isPlaying(m5spk_virtual_channel) != 0) {
//         if (millis() - drainT0 > 3000) {
//             Serial.println("[WAV] drain timeout");
//             break;
//         }
//         delay(1);
//     }
//     Serial.println("[WAV] stream done");
// }
static void streamPcmToSpeaker(WiFiClient& client,
                               HttpBodyState& body,
                               const WavInfo& info,
                               uint32_t timeoutMs,
                               bool allow_cancel,
                               uint32_t reqId,
                               const char* path,
                               uint32_t tReqStart,
                               uint32_t tConnect,
                               uint32_t tHeader,
                               uint32_t tWav)
{
    bool firstPlayLogged = false;

    // prebuffer: データが溜まるまで少し待つ（ただし長くは待たない）
    const size_t SPK_PREBUFFER_SIZE = 4096;
    uint32_t preT0 = millis();
    if (!body.chunked) {
        while ((size_t)client.available() < SPK_PREBUFFER_SIZE) {
            if (shouldCancelTtsNow(allow_cancel)) {
                Serial.println("[WAV] cancelled during prebuffer");
                M5.Speaker.stop();
                client.stop();
                body.finished = true;
                return;
            }
            if (!client.connected()) break;
            if (millis() - preT0 > 500) break;
            delay(2);
        }
        Serial.printf("[WAV] prebuffer=%u\n", (unsigned)client.available());
    } else {
        while (!client.available()) {
            if (shouldCancelTtsNow(allow_cancel)) {
                Serial.println("[WAV] cancelled during prebuffer");
                M5.Speaker.stop();
                client.stop();
                body.finished = true;
                return;
            }
            if (!client.connected()) break;
            if (millis() - preT0 > 100) break;
            delay(2);
        }
        Serial.printf("[WAV] prebuffer=chunked avail=%u\n", (unsigned)client.available());
    }

    const size_t OUT_SAMPLES = 256;
    const size_t MAX_IN_FRAME_BYTES = 4;
    const size_t INBUF_BYTES = OUT_SAMPLES * MAX_IN_FRAME_BYTES;
    const size_t IN_FRAME_BYTES = (info.bitsPerSamp / 8) * info.channels;

    static uint8_t inbuf[INBUF_BYTES];
    static int16_t outBuf[8][OUT_SAMPLES];
    static uint8_t bufIndex = 0;
    bufIndex = 0;

    uint32_t remaining = info.dataSize;
    bool unknownSize   = (remaining == 0 || remaining == 0xFFFFFFFF);

    uint32_t t0 = millis();
    uint32_t lastActivity = millis();

    while (true) {
        if (millis() - t0 > timeoutMs) {
            Serial.println("[WAV] stream timeout");
            break;
        }

        if (shouldCancelTtsNow(allow_cancel)) {
            Serial.println("[WAV] cancelled by button hold");
            M5.Speaker.stop();
            client.stop();
            body.finished = true;
            return;
        }

        if (!unknownSize && remaining == 0) break;

        size_t wantBytes = OUT_SAMPLES * IN_FRAME_BYTES;
        if (!unknownSize && wantBytes > remaining) {
            wantBytes = remaining;
        }
        if (wantBytes < IN_FRAME_BYTES) break;

        size_t toRead = wantBytes;

        if (!body.chunked) {
            uint32_t waitT0 = millis();
            while ((size_t)client.available() < wantBytes) {
                if (!client.connected()) break;
                if (millis() - waitT0 > 2000) break;
                delay(1);
            }

            toRead = client.available();
            if (toRead > wantBytes) toRead = wantBytes;

            if (toRead < IN_FRAME_BYTES) {
                if (!client.connected()) break;
                if (millis() - lastActivity > 3000) {
                    Serial.println("[WAV] no data 3s, stopping");
                    break;
                }
                delay(1);
                continue;
            }
        }

        if (!readBytes(client, body, inbuf, toRead, 3000)) {
            if (body.finished) break;
            if (!client.connected()) break;
            if (millis() - lastActivity > 3000) {
                Serial.println("[WAV] no data 3s, stopping");
                break;
            }
            delay(1);
            continue;
        }

        int n = (int)toRead;
        lastActivity = millis();
        if (!unknownSize) remaining -= (uint32_t)n;

        while (M5.Speaker.isPlaying(m5spk_virtual_channel) == 2) {
            delay(1);
            if (shouldCancelTtsNow(allow_cancel)) {
                Serial.println("[WAV] cancelled while waiting queue");
                M5.Speaker.stop();
                client.stop();
                body.finished = true;
                return;
            }
        }

        int16_t* out = outBuf[bufIndex];
        bufIndex = (bufIndex + 1) % 8;

        size_t outCount = 0;
        for (int i = 0; i + (int)IN_FRAME_BYTES <= n && outCount < OUT_SAMPLES; i += (int)IN_FRAME_BYTES) {
            int16_t mono = 0;

            if (info.bitsPerSamp == 16) {
                int16_t L = (int16_t)_readU16LE(inbuf + i);
                int16_t R = (info.channels >= 2) ? (int16_t)_readU16LE(inbuf + i + 2) : L;
                mono = (info.channels >= 2) ? (int16_t)(((int32_t)L + (int32_t)R) / 2) : L;
            } else if (info.bitsPerSamp == 8) {
                int16_t L = ((int16_t)inbuf[i] - 128) << 8;
                int16_t R = (info.channels >= 2) ? (((int16_t)inbuf[i + 1] - 128) << 8) : L;
                mono = (info.channels >= 2) ? (int16_t)(((int32_t)L + (int32_t)R) / 2) : L;
            } else {
                continue;
            }

            out[outCount++] = mono;
        }

        if (outCount == 0) continue;

        // ★ 最初の playRaw 直前だけ1回ログを出す
        if (!firstPlayLogged) {
            const uint32_t tFirstPlay = millis();
            Serial.printf(
                "[PiperTiming#%lu] FIRST_PLAY path=%s connect=%lu hdr=%lu wav=%lu wait=%lu to_first_play=%lu chunked=%d avail=%u samples=%u\n",
                (unsigned long)reqId,
                path ? path : "?",
                (unsigned long)(tConnect - tReqStart),
                (unsigned long)(tHeader - tConnect),
                (unsigned long)(tWav - tHeader),
                (unsigned long)(tFirstPlay - tWav),
                (unsigned long)(tFirstPlay - tReqStart),
                body.chunked ? 1 : 0,
                (unsigned)client.available(),
                (unsigned)outCount
            );
            firstPlayLogged = true;
        }

        M5.Speaker.playRaw(
            out,
            outCount,
            info.sampleRate,
            false,
            1,
            m5spk_virtual_channel,
            false
        );
    }

    uint32_t drainT0 = millis();
    while (M5.Speaker.isPlaying(m5spk_virtual_channel) != 0) {
        if (millis() - drainT0 > 3000) {
            Serial.println("[WAV] drain timeout");
            break;
        }
        delay(1);
    }
    Serial.println("[WAV] stream done");
}

// ============================================================
// プリフェッチ機構（ダブルバッファ方式）
// ============================================================
// WiFiClientは内部でソケットを共有参照するため、プリフェッチ中のクライアントと
// 再生中のクライアントを同時に持つにはスロットを分ける必要がある。
// slot[0] と slot[1] を交互に使う。

struct PiperPrefetchSlot {
    WiFiClient    client;
    HttpBodyState body;
    WavInfo       info;
    bool          ready   = false;
    bool       failed  = false;
    bool       running = false;
    String     text;
};

static PiperPrefetchSlot g_pfSlot[2];
static int g_pfNextSlot = 0;  // 次にプリフェッチで使うスロット

// プリフェッチタスク本体（Core 0 で実行）
struct PrefetchArg { int slot; };
static PrefetchArg g_pfArg;

static void piperPrefetchTaskFunc(void* arg) {
    int slotIdx = ((PrefetchArg*)arg)->slot;
    PiperPrefetchSlot& pf = g_pfSlot[slotIdx];

    String t = pf.text;
    t.trim();
    t.replace("\r", "");
    while (t.startsWith("\n")) t.remove(0, 1);
    t.replace("\n", "");

    Serial.printf("[Prefetch@%d] start: %s\n", slotIdx, t.c_str());

    pf.client.setTimeout(3000);
    if (!pf.client.connect(PIPER_TTS_IP.c_str(), PIPER_TTS_PORT)) {
        Serial.printf("[Prefetch@%d] connect failed\n", slotIdx);
        pf.failed = true;
        pf.running = false;
        vTaskDelete(nullptr);
        return;
    }

    const String piperLang = piperLanguageFromLangCode();

    // pf.client.print(String("GET ") + "/tts.wav?text=" + urlEncodeUTF8(t) +
    //                  "&length_scale=" + String(PIPER_TTS_LENGTH_SCALE, 2) +
    //                  "&language=" + piperLang +
    //                  " HTTP/1.1\r\n" +
    //                  "Host: " + PIPER_TTS_IP + ":" + String(PIPER_TTS_PORT) + "\r\n" +
    //                  "Connection: close\r\n\r\n");
    pf.client.print(String("GET ") + piperTtsPath() + String("?text=") + urlEncodeUTF8(t) +
                    "&length_scale=" + String(PIPER_TTS_LENGTH_SCALE, 2) +
                    "&language=" + piperLang +
                    " HTTP/1.1\r\n" +
                    "Host: " + PIPER_TTS_IP + ":" + String(PIPER_TTS_PORT) + "\r\n" +
                    "Connection: close\r\n\r\n");    

    HttpBodyState body;
    int httpCode = skipHttpHeader(pf.client, body);
    Serial.printf("[Prefetch@%d] HTTP status: %d\n", slotIdx, httpCode);
    if (httpCode != 200) {
        pf.client.stop();
        pf.failed = true;
        pf.running = false;
        vTaskDelete(nullptr);
        return;
    }

    WavInfo info;
    if (!parseWavHeader(pf.client, body, info)) {
        Serial.printf("[Prefetch@%d] WAV parse failed\n", slotIdx);
        pf.client.stop();
        pf.failed = true;
        pf.running = false;
        vTaskDelete(nullptr);
        return;
    }

    Serial.printf("[Prefetch@%d] WAV OK: %uHz %ubit %uch dataSize=%u\n",
        slotIdx, (unsigned)info.sampleRate, (unsigned)info.bitsPerSamp,
        (unsigned)info.channels, (unsigned)info.dataSize);

    pf.body  = body;
    pf.info  = info;
    pf.ready = true;
    pf.running = false;
    vTaskDelete(nullptr);
}

// プリフェッチ開始: 指定スロットで次チャンクの合成を開始
static int piper_prefetch_start(const String& text) {
    int slot = g_pfNextSlot;
    PiperPrefetchSlot& pf = g_pfSlot[slot];

    // 前回の残りをクリーンアップ
    if (pf.client.connected()) pf.client.stop();
    pf.ready   = false;
    pf.failed  = false;
    pf.running = true;
    pf.text    = text;
    pf.body    = HttpBodyState();
    pf.info    = WavInfo();

    g_pfArg.slot = slot;
    g_pfNextSlot = (slot + 1) % 2;  // 次回は別スロット

    xTaskCreatePinnedToCore(
        piperPrefetchTaskFunc,
        "piperPF",
        4096,
        &g_pfArg,
        1,
        nullptr,
        0
    );

    return slot;
}

// プリフェッチ完了待ち
static bool piper_prefetch_wait(int slot, uint32_t timeoutMs = 15000) {
    PiperPrefetchSlot& pf = g_pfSlot[slot];
    uint32_t t0 = millis();
    while (pf.running) {
        if (millis() - t0 > timeoutMs) {
            Serial.printf("[Prefetch@%d] wait timeout\n", slot);
            return false;
        }
        delay(5);
    }
    return pf.ready;
}

// ============================================================
// speak_piper_http(): 通常の同期再生（プリフェッチなし）
// ============================================================
// static bool speak_piper_http(const String& text, bool allow_cancel) {
//     ensureSpeakerActive();

//     String t = text;
//     t.trim();
//     t.replace("\r", "");
//     while (t.startsWith("\n")) t.remove(0, 1);
//     t.replace("\n", "");
//     if (t.length() == 0) return true;
//     if (t.length() <= 2) return true;

//     M5.Speaker.setVolume(PiperPlus_voice_volume);    

//     const String piperLang = piperLanguageFromLangCode();

//     // const String url = String("http://") + PIPER_TTS_IP + ":" + String(PIPER_TTS_PORT) +
//     //                    "/tts.wav?text=" + urlEncodeUTF8(t) +
//     //                    "&length_scale=" + String(PIPER_TTS_LENGTH_SCALE, 2) +
//     //                    "&language=" + piperLang;
//     const String url = String("http://") + PIPER_TTS_IP + ":" + String(PIPER_TTS_PORT) +
//                     String(piperTtsPath()) + "?text=" + urlEncodeUTF8(t) +
//                     "&length_scale=" + String(PIPER_TTS_LENGTH_SCALE, 2) +
//                     "&language=" + piperLang;

//     Serial.print("[Piper] GET "); Serial.println(url);

//     WiFiClient client;
//     client.setTimeout(3000);

//     if (!client.connect(PIPER_TTS_IP.c_str(), PIPER_TTS_PORT)) {
//         Serial.println("[Piper] connect failed");
//         return false;
//     }

//     // client.print(String("GET ") + "/tts.wav?text=" + urlEncodeUTF8(t) +
//     //              "&length_scale=" + String(PIPER_TTS_LENGTH_SCALE, 2) +
//     //              "&language=" + piperLang +
//     //              " HTTP/1.1\r\n" +
//     //              "Host: " + PIPER_TTS_IP + ":" + String(PIPER_TTS_PORT) + "\r\n" +
//     //              "Connection: close\r\n\r\n");
//     client.print(String("GET ") + String(piperTtsPath()) + "?text=" + urlEncodeUTF8(t) +
//                 "&length_scale=" + String(PIPER_TTS_LENGTH_SCALE, 2) +
//                 "&language=" + piperLang +
//                 " HTTP/1.1\r\n" +
//                 "Host: " + PIPER_TTS_IP + ":" + String(PIPER_TTS_PORT) + "\r\n" +
//                 "Connection: close\r\n\r\n");    

//     HttpBodyState body;
//     int httpCode = skipHttpHeader(client, body);
//     Serial.printf("[Piper] HTTP status: %d\n", httpCode);
//     if (httpCode != 200) {
//         Serial.printf("[Piper] Unexpected HTTP status: %d\n", httpCode);
//         client.stop();
//         return false;
//     }

//     WavInfo info;
//     if (!parseWavHeader(client, body, info)) {
//         Serial.println("[Piper] WAV parse failed");
//         client.stop();
//         return false;
//     }
//     Serial.printf("[Piper] WAV OK: %uHz %ubit %uch dataSize=%u\n",
//         (unsigned)info.sampleRate, (unsigned)info.bitsPerSamp, (unsigned)info.channels, (unsigned)info.dataSize);

//     g_piper_tts_playing = true;

//     const uint32_t STREAM_TIMEOUT_MS = 15000;
//     streamPcmToSpeaker(client, body, info, STREAM_TIMEOUT_MS, allow_cancel);
//     client.stop();

//     return true;
// }
static bool speak_piper_http(const String& text, bool allow_cancel) {
    ensureSpeakerActive();

    String t = text;
    t.trim();
    t.replace("\r", "");
    while (t.startsWith("\n")) t.remove(0, 1);
    t.replace("\n", "");
    if (t.length() == 0) return true;
    if (t.length() <= 2) return true;

    M5.Speaker.setVolume(PiperPlus_voice_volume);

    const String piperLang = piperLanguageFromLangCode();
    const char* path = piperTtsPath();

    const String url = String("http://") + PIPER_TTS_IP + ":" + String(PIPER_TTS_PORT) +
                       String(path) + "?text=" + urlEncodeUTF8(t) +
                       "&length_scale=" + String(PIPER_TTS_LENGTH_SCALE, 2) +
                       "&language=" + piperLang;

    const uint32_t reqId = ++g_piper_req_seq;
    const uint32_t t0 = millis();

    Serial.printf("[PiperTiming#%lu] START path=%s text='%s'\n",
                  (unsigned long)reqId, path, t.c_str());
    Serial.print("[Piper] GET "); Serial.println(url);

    WiFiClient client;
    client.setTimeout(3000);

    if (!client.connect(PIPER_TTS_IP.c_str(), PIPER_TTS_PORT)) {
        Serial.printf("[PiperTiming#%lu] connect failed after %lu ms\n",
                      (unsigned long)reqId,
                      (unsigned long)(millis() - t0));
        Serial.println("[Piper] connect failed");
        return false;
    }
    const uint32_t tConnect = millis();

    client.print(String("GET ") + String(path) + "?text=" + urlEncodeUTF8(t) +
                 "&length_scale=" + String(PIPER_TTS_LENGTH_SCALE, 2) +
                 "&language=" + piperLang +
                 " HTTP/1.1\r\n" +
                 "Host: " + PIPER_TTS_IP + ":" + String(PIPER_TTS_PORT) + "\r\n" +
                 "Connection: close\r\n\r\n");

    HttpBodyState body;
    int httpCode = skipHttpHeader(client, body);
    const uint32_t tHeader = millis();

    Serial.printf("[Piper] HTTP status: %d\n", httpCode);
    if (httpCode != 200) {
        Serial.printf("[PiperTiming#%lu] HTTP status=%d connect=%lu hdr=%lu total=%lu\n",
                      (unsigned long)reqId,
                      httpCode,
                      (unsigned long)(tConnect - t0),
                      (unsigned long)(tHeader - tConnect),
                      (unsigned long)(tHeader - t0));
        Serial.printf("[Piper] Unexpected HTTP status: %d\n", httpCode);
        client.stop();
        return false;
    }

    WavInfo info;
    if (!parseWavHeader(client, body, info)) {
        const uint32_t tFail = millis();
        Serial.printf("[PiperTiming#%lu] WAV parse failed connect=%lu hdr=%lu wav=%lu total=%lu\n",
                      (unsigned long)reqId,
                      (unsigned long)(tConnect - t0),
                      (unsigned long)(tHeader - tConnect),
                      (unsigned long)(tFail - tHeader),
                      (unsigned long)(tFail - t0));
        Serial.println("[Piper] WAV parse failed");
        client.stop();
        return false;
    }
    const uint32_t tWav = millis();

    Serial.printf("[Piper] WAV OK: %uHz %ubit %uch dataSize=%u\n",
                  (unsigned)info.sampleRate,
                  (unsigned)info.bitsPerSamp,
                  (unsigned)info.channels,
                  (unsigned)info.dataSize);

    g_piper_tts_playing = true;

    const uint32_t STREAM_TIMEOUT_MS = 15000;

    // streamPcmToSpeaker(client, body, info, STREAM_TIMEOUT_MS, allow_cancel);
    streamPcmToSpeaker(
        client,
        body,
        info,
        STREAM_TIMEOUT_MS,
        allow_cancel,
        reqId,
        path,
        t0,
        tConnect,
        tHeader,
        tWav
    );

    const uint32_t tDone = millis();

    client.stop();

    Serial.printf("[PiperTiming#%lu] DONE path=%s connect=%lu hdr=%lu wav=%lu play=%lu total=%lu\n",
                  (unsigned long)reqId,
                  path,
                  (unsigned long)(tConnect - t0),
                  (unsigned long)(tHeader - tConnect),
                  (unsigned long)(tWav - tHeader),
                  (unsigned long)(tDone - tWav),
                  (unsigned long)(tDone - t0));

    return true;
}