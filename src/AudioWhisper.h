// ===== AudioWhisper.h (Plan A: begin retry削除 / I2Cログ抑制) =====
#ifndef _AUDIOWHISPER_H
#define _AUDIOWHISPER_H

#include <Arduino.h>  // byte, size_t, int16_t

class AudioWhisper {
private:
    byte*    record_buffer     = nullptr;
    int16_t* pre_speech_buffer = nullptr;   // 発話前データ保持用バッファ
    size_t   actual_recorded_frames = 0;    // 実際に録音したフレーム数

    // VAD設定（外部から変更可能）
    static float vad_threshold;
    static int   min_speech_frames;
    static int   max_silence_frames;
    static int   pre_speech_frames;
    static int   post_speech_frames;
    static int   max_recording_frames;

    // VAD関連のプライベートメソッド
    float    calculateRMS(const int16_t* data, size_t length);
    bool     isVoiceActivity(const int16_t* data, size_t length);
    int16_t* MakeHeader(byte* header, size_t actual_frames);

public:
    AudioWhisper();
    ~AudioWhisper();

    const byte* GetBuffer() const { return record_buffer; }
    size_t GetSize() const;
    size_t GetRecordedFrames() const { return actual_recorded_frames; }

    void Record();       // VAD機能付き録音
    void RecordFixed();  // 固定時間録音（約5秒）

    // VAD設定関数
    // 注意：pre_speech_frames を変える場合は、AudioWhisper インスタンス生成前に呼ぶこと
    // static void SetVADParameters(float threshold = 800.0f,
    //                              int min_speech = 10,
    //                              int max_silence = 20,
    //                              int pre_frames = 5,
    //                              int post_frames = 10);

    // VAD設定関数
    // 注意：
    //   - pre_speech_frames / post_speech_frames は内部定数扱い。
    //     コンストラクタで pre_speech_buffer のサイズを確定するため、
    //     後から変更すると不整合・バッファオーバーランの危険がある。
    //   - 変更する場合は AudioWhisper.cpp 内の静的初期値を直接編集すること。
    //   - ユーザーが調整するのは threshold を主値とする3パラメータのみ。
    static void SetVADParameters(float threshold = 180.0f,
                                 int min_speech = 4,
                                 int max_silence = 40);

    // VAD内部状態リセット
    // 録音セッション間の状態汚染を防ぐため、Record() 冒頭で呼ぶこと
    static void ResetVADState();
                                 
    // デバッグ用
    void PrintVADStatus() const;
};

#endif // _AUDIOWHISPER_H
