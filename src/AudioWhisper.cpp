#include <M5Unified.h>
#include <Wire.h>
#include "AudioWhisper.h"
#include <cmath>
#include <algorithm>
#include <cstring>

// ===== Recording constants =====
// constexpr size_t record_number     = 400;   // 400 frames
constexpr size_t record_number     = 540;   // 540 frames
constexpr size_t record_length     = 150;   // samples per frame
constexpr size_t record_size       = record_number * record_length;
constexpr size_t record_samplerate = 16000;
constexpr int    headerSize        = 44;

// ===== VAD static defaults =====
// Step 3c: 実運用で安定した値を正式な既定値として確定。
// これらは main.cpp 側の applyCurrentVadConfigToAudioWhisper() から
// 録音直前に上書きされるが、ここで既定値を整えておくことで、
// 単体テスト時や上書きされなかった場合のフォールバックが意味のある値になる。
float AudioWhisper::vad_threshold         = 180.0f;  // 旧800 → 実運用値180
int   AudioWhisper::min_speech_frames     = 4;       // 旧10 → 実運用値4 (日本語の立ち上がり吸収)
int   AudioWhisper::max_silence_frames    = 40;      // 約0.375秒 (息継ぎ吸収)
int   AudioWhisper::pre_speech_frames     = 15;      // 約141ms遡行 (内部定数、変更不可)
int   AudioWhisper::post_speech_frames    = 24;      // 約0.225秒 (内部定数、変更不可)
// max_silence + post_speech = 64フレーム ≈ 0.60秒
int   AudioWhisper::max_recording_frames  = 540;     // 約5.06秒 (record_number と整合済み)


// ===== VAD動的閾値の内部状態 =====
// 録音セッション間の汚染を防ぐため、関数内staticからファイルスコープへ外出し。
// ResetVADState() で初期化される。
static float s_noise_level = 0.0f;
static float s_adaptive_threshold = 0.0f;  // 互換のため保持(現実装では未使用)
static int   s_noise_update_counter = 0;
static int   s_debug_frame = 0;  // デバッグログ用カウンタ
static int   s_frame_count = 0;   // 初期フレーム無視用カウンタ
static constexpr bool kVadDebug = false;  // Step 3d: 通常運用では詳細VADログを出さない

// ===== VAD setter =====
// 設計方針:
//   ユーザーが録音直前に調整するのは threshold / min_speech / max_silence の3つのみ。
//   pre_speech_frames / post_speech_frames は内部定数として静的初期値で固定する。
//   - pre_speech_frames はコンストラクタで pre_speech_buffer のサイズ確定に使うため、
//     後から変更するとバッファオーバーランの危険がある。
//   - post_speech_frames は会話末尾の保険として 24 フレームで固定。
//   この方針により、ユーザーは threshold を主値として安全に調整できる。
void AudioWhisper::SetVADParameters(float threshold,
                                    int min_speech,
                                    int max_silence)
{
    vad_threshold      = threshold;
    min_speech_frames  = min_speech;
    max_silence_frames = max_silence;
}


// ===== VAD内部状態リセット =====
// Record() 冒頭で呼び、録音セッション間での状態汚染を防ぐ。
void AudioWhisper::ResetVADState() {
    s_noise_level          = 0.0f;
    s_adaptive_threshold   = 0.0f;
    s_noise_update_counter = 0;
    s_debug_frame          = 0;
    s_frame_count          = 0;
    Serial.println("[VAD] state reset");
}

AudioWhisper::AudioWhisper() {
    const size_t bytes = record_size * sizeof(int16_t) + headerSize;
    record_buffer = static_cast<byte*>(::heap_caps_malloc(bytes, MALLOC_CAP_8BIT));
    if (!record_buffer) {
        Serial.println("AudioWhisper: ERROR heap_caps_malloc(record_buffer) failed");
        return;
    }
    ::memset(record_buffer, 0, bytes);

    // VAD用循環バッファ
    const size_t pre_bytes = (size_t)pre_speech_frames * record_length * sizeof(int16_t);
    pre_speech_buffer = static_cast<int16_t*>(::heap_caps_malloc(pre_bytes, MALLOC_CAP_8BIT));
    if (!pre_speech_buffer) {
        Serial.println("AudioWhisper: ERROR heap_caps_malloc(pre_speech_buffer) failed");
        return;
    }
    ::memset(pre_speech_buffer, 0, pre_bytes);

    actual_recorded_frames = 0;
    Serial.println("AudioWhisper: VAD機能付きで初期化完了");
}

AudioWhisper::~AudioWhisper() {
    if (record_buffer) {
        free(record_buffer);
        record_buffer = nullptr;
    }
    if (pre_speech_buffer) {
        free(pre_speech_buffer);
        pre_speech_buffer = nullptr;
    }
}

size_t AudioWhisper::GetSize() const {
    // WAV header + actual frames
    return actual_recorded_frames * record_length * sizeof(int16_t) + headerSize;
}

float AudioWhisper::calculateRMS(const int16_t* data, size_t length) {
    float sum = 0.0f;
    for (size_t i = 0; i < length; i++) {
        float s = static_cast<float>(data[i]);
        sum += s * s;
    }
    return std::sqrt(sum / (float)length);
}


bool AudioWhisper::isVoiceActivity(const int16_t* data, size_t length) {
    float rms = calculateRMS(data, length);

    // 録音開始直後の I2S 残留/TTS 残響を無視する。
    // 最初の5フレームは音声判定もノイズ推定も行わず、強制的に無音扱い。
    s_frame_count++;
    if (s_frame_count <= 5) {
        if (kVadDebug && s_debug_frame < 20) {
            Serial.printf("[VAD_DBG] frame=%d rms=%.1f noise=%.1f effective=%.1f voice=NO (ignore warmup)\n",
                          s_debug_frame, rms, s_noise_level, vad_threshold * 1.3f);
            s_debug_frame++;
        }
        return false;
    }

    // ===== 論点1=B実装 =====
    // 方針:
    //   - vad_threshold を「主値」として尊重する
    //   - 動的推定は「補助役」とし、vad_threshold を大きく押し上げない
    //   - 実効閾値は最大でも vad_threshold × 1.3 に制限

    // 最初の10フレームでノイズレベル推定
    if (s_noise_update_counter < 10) {
        s_noise_level = (s_noise_level * s_noise_update_counter + rms)
                        / (s_noise_update_counter + 1);
        s_noise_update_counter++;
    } else if (rms < vad_threshold) {
        // 発話なし区間だけで緩やかに更新(発話中はノイズ推定を汚染しない)
        s_noise_level = s_noise_level * 0.99f + rms * 0.01f;
    }

    // 実効閾値の決定
    // 基本は vad_threshold。ノイズフロアが非常に高い場合のみ、
    // 最大 vad_threshold × 1.3 までの押し上げを許可。
    float threshold_effective = vad_threshold;
    float noise_suggested = s_noise_level * 1.5f;  // 旧実装の3.0から緩和
    if (noise_suggested > vad_threshold) {
        threshold_effective = std::min(noise_suggested, vad_threshold * 1.3f);
    }

    const bool is_voice = rms > threshold_effective;

    // Step 3d: 通常運用では詳細VADログを出さない
    if (kVadDebug && s_debug_frame < 20) {
        Serial.printf("[VAD_DBG] frame=%d rms=%.1f noise=%.1f effective=%.1f voice=%s\n",
                      s_debug_frame, rms, s_noise_level, threshold_effective,
                      is_voice ? "YES" : "NO");
        s_debug_frame++;
    }

    return is_voice;
}

int16_t* AudioWhisper::MakeHeader(byte* header, size_t actual_frames) {
    const auto wavDataSize = actual_frames * record_length * 2;  // int16 mono
    header[0] = 'R'; header[1] = 'I'; header[2] = 'F'; header[3] = 'F';
    unsigned int fileSizeMinus8 = wavDataSize + headerSize - 8;
    header[4] = (byte)(fileSizeMinus8 & 0xFF);
    header[5] = (byte)((fileSizeMinus8 >> 8) & 0xFF);
    header[6] = (byte)((fileSizeMinus8 >> 16) & 0xFF);
    header[7] = (byte)((fileSizeMinus8 >> 24) & 0xFF);
    header[8] = 'W'; header[9] = 'A'; header[10] = 'V'; header[11] = 'E';
    header[12] = 'f'; header[13] = 'm'; header[14] = 't'; header[15] = ' ';
    header[16] = 0x10; header[17] = 0x00; header[18] = 0x00; header[19] = 0x00;
    header[20] = 0x01; header[21] = 0x00; header[22] = 0x01; header[23] = 0x00;
    header[24] = 0x80; header[25] = 0x3E; header[26] = 0x00; header[27] = 0x00; // 16000
    header[28] = 0x00; header[29] = 0x7D; header[30] = 0x00; header[31] = 0x00; // byte rate 32000
    header[32] = 0x02; header[33] = 0x00; header[34] = 0x10; header[35] = 0x00; // block align 2, bits 16
    header[36] = 'd'; header[37] = 'a'; header[38] = 't'; header[39] = 'a';
    header[40] = (byte)(wavDataSize & 0xFF);
    header[41] = (byte)((wavDataSize >> 8) & 0xFF);
    header[42] = (byte)((wavDataSize >> 16) & 0xFF);
    header[43] = (byte)((wavDataSize >> 24) & 0xFF);
    return (int16_t*)&header[headerSize];
}

void AudioWhisper::Record() {
    Serial.println("VAD録音開始準備中...");

    // VAD内部状態を初期化(セッション間汚染防止)
    ResetVADState();

    if (!record_buffer || !pre_speech_buffer) {
        Serial.println("ERROR: AudioWhisper buffers not allocated");
        actual_recorded_frames = 0;
        return;
    }

    // ★★★ 公式パターン準拠 (ATOMIC ECHO BASE + AtomS3R, M5Unified >= 0.2.8):
    //   Mic と Speaker は同じ ES8311 codec を共有するため同時使用不可。
    //   Mic.begin() の前に必ず Speaker の再生完了を待ち、Speaker.end() で明示停止する。
    //   これを怠ると Mic.begin() 後の Mic.record() で I2S/codec ハンドル不整合により
    //   Double Exception (EXCVADDR=0xffffffd0) を発生する。
    //   参照: https://docs.m5stack.com/en/arduino/atom_echos3r/mic
    {
        // Speaker 再生完了を待機（安全のため最大 500ms）
        uint32_t wait_start = millis();
        while (M5.Speaker.isPlaying() && (millis() - wait_start) < 500) {
            delay(1);
        }
        uint32_t waited_ms = millis() - wait_start;
        Serial.printf("[AW::Record] Speaker.isPlaying wait=%ums\n", (unsigned)waited_ms);
        M5.Speaker.end();
        delay(50);  // codec 側の切り替えを確実にするための短い待機
    }

    // Mic config
    auto cfg = M5.Mic.config();
    cfg.noise_filter_level = (cfg.noise_filter_level + 8) & 255;
    cfg.sample_rate = record_samplerate;
    M5.Mic.config(cfg);

    if (!M5.Mic.begin()) {
        Serial.println("ERROR: マイク初期化失敗 (VAD録音)");
        actual_recorded_frames = 0;
        return;
    }
    Serial.println("マイク初期化成功 (VAD録音)");

    enum VADState {
        WAITING_FOR_SPEECH,
        RECORDING_SPEECH,
        POST_SPEECH_SILENCE
    };

    VADState vad_state = WAITING_FOR_SPEECH;
    int speech_frames = 0;
    int silence_frames = 0;
    int total_frames = 0;
    int pre_speech_index = 0;
    actual_recorded_frames = 0;

    auto* wavData = MakeHeader(record_buffer, 0);

    Serial.println("VAD録音開始... お話しください");

    unsigned long start_time = millis();
    const unsigned long max_wait_time = 10000;

    // 簡略テスト（再初期化なし）
    // Serial.println("マイク動作確認中...");
    // for (int t = 0; t < 3; t++) {
    //     int16_t test_data[record_length];
    //     if (M5.Mic.record(test_data, record_length, record_samplerate)) {
    //         float test_rms = calculateRMS(test_data, record_length);
    //         Serial.printf("テストフレーム %d: RMS=%.2f\n", t, test_rms);
    //     }
    //     delay(50);
    // }

    // マイクI2Sウォームアップ: 初期化直後のノイズバーストを食い潰す
    // 3フレーム(約30ms分)を連続読み捨て。delayは入れずに最小時間化。
    // 旧実装(テスト3回+delay(50)×3 = 約200ms) より大幅短縮しつつ、
    // I2Sハードウェアの不安定な初期フレームを吸収する。
    for (int i = 0; i < 3; i++) {
        int16_t warmup_data[record_length];
        M5.Mic.record(warmup_data, record_length, record_samplerate);
    }    

    Serial.println("VAD録音ループ開始");

    while (total_frames < max_recording_frames) {
        if (vad_state == WAITING_FOR_SPEECH && (millis() - start_time) > max_wait_time) {
            Serial.println("TIMEOUT: 音声が検出されませんでした");
            break;
        }

        int16_t frame_data[record_length];
        bool ok = M5.Mic.record(frame_data, record_length, record_samplerate);
        if (!ok) {
            delay(10);
            continue;
        }

        bool voice = isVoiceActivity(frame_data, record_length);

        if (total_frames % 20 == 0) {
            float rms = calculateRMS(frame_data, record_length);
            Serial.printf("Frame %d: RMS=%.2f, Voice=%s\n", total_frames, rms, voice ? "YES" : "NO");
        }

        switch (vad_state) {
            case WAITING_FOR_SPEECH: {
                // 循環バッファに保存
                memcpy(&pre_speech_buffer[(pre_speech_index % pre_speech_frames) * record_length],
                       frame_data, record_length * sizeof(int16_t));
                pre_speech_index++;

                if (voice) {
                    speech_frames++;
                    if (speech_frames >= min_speech_frames) {
                        vad_state = RECORDING_SPEECH;
                        Serial.println("SPEECH DETECTED: 録音開始");

                        int pre_frames_to_copy = std::min(pre_speech_frames, pre_speech_index);
                        int start_index = std::max(0, pre_speech_index - pre_frames_to_copy);

                        for (int i = 0; i < pre_frames_to_copy; i++) {
                            if (actual_recorded_frames < record_number) {
                                int src = (start_index + i) % pre_speech_frames;
                                memcpy(&wavData[actual_recorded_frames * record_length],
                                       &pre_speech_buffer[src * record_length],
                                       record_length * sizeof(int16_t));
                                actual_recorded_frames++;
                            }
                        }

                        if (actual_recorded_frames < record_number) {
                            memcpy(&wavData[actual_recorded_frames * record_length],
                                   frame_data, record_length * sizeof(int16_t));
                            actual_recorded_frames++;
                        }

                        speech_frames = 0;
                        silence_frames = 0;
                    }
                } else {
                    speech_frames = 0;
                }
            } break;

            case RECORDING_SPEECH: {
                if (actual_recorded_frames < record_number) {
                    memcpy(&wavData[actual_recorded_frames * record_length],
                           frame_data, record_length * sizeof(int16_t));
                    actual_recorded_frames++;
                }

                if (voice) {
                    silence_frames = 0;
                } else {
                    silence_frames++;
                    if (silence_frames >= max_silence_frames) {
                        vad_state = POST_SPEECH_SILENCE;
                        Serial.println("SPEECH ENDED: 後処理中");
                    }
                }
            } break;

            case POST_SPEECH_SILENCE: {
                if (actual_recorded_frames < record_number) {
                    memcpy(&wavData[actual_recorded_frames * record_length],
                           frame_data, record_length * sizeof(int16_t));
                    actual_recorded_frames++;
                }

                silence_frames++;
                if (silence_frames >= (max_silence_frames + post_speech_frames)) {
                    Serial.printf("RECORDING COMPLETED: %d フレーム\n", (int)actual_recorded_frames);
                    goto recording_finished;
                }

                if (voice) {
                    vad_state = RECORDING_SPEECH;
                    silence_frames = 0;
                    Serial.println("SPEECH RESUMED: 発話再開");
                }
            } break;
        }

        total_frames++;

        if (actual_recorded_frames >= record_number) {
            Serial.println("WARNING: バッファ満杯、録音停止");
            break;
        }
    }

recording_finished:
    // ★★★ 公式パターン準拠: Mic.end() 前に録音完了を確実に待つ
    //   record() が返ってきた直後も内部 I2S DMA がまだ最後のフレームを処理中のことがある。
    //   isRecording() が false になるまで待ってから end() することで、
    //   DMA 処理中の driver uninstall による内部状態破壊を防ぐ。
    {
        uint32_t wait_start = millis();
        while (M5.Mic.isRecording() && (millis() - wait_start) < 500) {
            delay(1);
        }
        uint32_t waited_ms = millis() - wait_start;
        Serial.printf("[AW::Record] Mic.isRecording wait=%ums\n", (unsigned)waited_ms);
    }
    M5.Mic.end();
    delay(150);

    Serial.printf("録音終了: total_frames=%d, recorded_frames=%d\n", total_frames, (int)actual_recorded_frames);

    if (actual_recorded_frames > 0) {
        MakeHeader(record_buffer, actual_recorded_frames);
        float duration = (float)actual_recorded_frames * record_length / record_samplerate;
        Serial.printf("STATS: %.2f秒, %d フレーム, %zu bytes\n", duration, (int)actual_recorded_frames, GetSize());
    }
}

void AudioWhisper::RecordFixed() {
    Serial.println("固定時間録音モード（5秒）");

    if (!record_buffer) {
        Serial.println("ERROR: AudioWhisper record_buffer not allocated");
        actual_recorded_frames = 0;
        return;
    }

    M5.Speaker.end();
    delay(150);

    auto cfg = M5.Mic.config();
    cfg.noise_filter_level = (cfg.noise_filter_level + 8) & 255;
    cfg.sample_rate = record_samplerate;
    M5.Mic.config(cfg);

    // ★ retry削除：1回だけ
    Serial.println("マイク初期化中...");
    if (!M5.Mic.begin()) {
        Serial.println("ERROR: マイク初期化失敗（固定録音）");
        actual_recorded_frames = 0;
        return;
    }
    Serial.println("マイク初期化成功（固定録音）");

    // 簡略テスト（再初期化なし）
    Serial.println("マイクテスト開始...");
    int16_t test_buffer[record_length];
    bool test_success = false;
    for (int i = 0; i < 2; i++) {
        if (M5.Mic.record(test_buffer, record_length, record_samplerate)) {
            float rms = calculateRMS(test_buffer, record_length);
            Serial.printf("テスト %d: RMS=%.2f\n", i, rms);
            if (rms > 5.0f) { test_success = true; break; }
        }
        delay(80);
    }
    if (!test_success) {
        Serial.println("WARNING: マイクテストで低レベル検出");
    }

    actual_recorded_frames = record_number;
    auto* wavData = MakeHeader(record_buffer, actual_recorded_frames);

    Serial.println("固定時間録音開始... マイクに向かって話してください...");


    // --- warm-up: discard a few frames to stabilize I2S/DMIC (prevents all-zero frames) ---
    for (int i = 0; i < 12; i++) {
        int16_t dummy[record_length];
        M5.Mic.record(dummy, record_length, record_samplerate);
        delay(10);
    }
    int successful_frames = 0;
    float total_rms = 0.0f;
    int16_t overall_max = 0, overall_min = 0;

    for (int idx = 0; idx < (int)record_number; idx++) {
        auto data = &wavData[idx * record_length];

        if (M5.Mic.record(data, record_length, record_samplerate)) {
            successful_frames++;
            float frame_rms = calculateRMS(data, record_length);
            // If frame looks like near-silence due to timing/glitch, retry once
            if (frame_rms < 1.0f) {
                int16_t tmp[record_length];
                if (M5.Mic.record(tmp, record_length, record_samplerate)) {
                    memcpy(data, tmp, record_length * sizeof(int16_t));
                    frame_rms = calculateRMS(data, record_length);
                }
            total_rms += frame_rms;
            }
            for (size_t i = 0; i < record_length; i++) {
                if (data[i] > overall_max) overall_max = data[i];
                if (data[i] < overall_min) overall_min = data[i];
            }

            if (idx % 80 == 0) {
                Serial.printf("録音進捗: %d/%d, RMS=%.2f\n", idx, (int)record_number, frame_rms);
            }
        } else {
            memset(data, 0, record_length * sizeof(int16_t));
        }
    }

    M5.Mic.end();
    delay(150);

    Serial.println("固定時間録音完了");

    const float average_rms = (successful_frames > 0) ? (total_rms / successful_frames) : 0.0f;
    Serial.printf("=== 録音統計 ===\n");
    Serial.printf("成功フレーム: %d/%d\n", successful_frames, (int)record_number);
    Serial.printf("平均RMS: %.2f\n", average_rms);
    Serial.printf("音声レベル: Min=%d, Max=%d\n", overall_min, overall_max);
    Serial.printf("録音時間: %.2f秒\n", (float)actual_recorded_frames * record_length / record_samplerate);
    Serial.printf("ファイルサイズ: %zu bytes\n", GetSize());

    if (average_rms < 10.0f) {
        Serial.println("WARNING: 音声レベルが非常に低いです。マイクの接続を確認してください。");
    } else {
        Serial.println("音声レベルは正常範囲内です。");
    }

    Serial.print("音声データサンプル (hex): ");
    for (int i = 0; i < 16 && i < (int)record_length; i++) {
        Serial.printf("%04X ", (uint16_t)wavData[i]);
    }
    Serial.println();
}

void AudioWhisper::PrintVADStatus() const {
    Serial.println("=== VAD設定状況 ===");
    Serial.printf("閾値: %.1f\n", vad_threshold);
    Serial.printf("最小発話フレーム: %d\n", min_speech_frames);
    Serial.printf("最大無音フレーム: %d\n", max_silence_frames);
    Serial.printf("前後保持フレーム: %d, %d\n", pre_speech_frames, post_speech_frames);
    Serial.printf("最大録音フレーム: %d\n", max_recording_frames);
    Serial.println("==================");
}
