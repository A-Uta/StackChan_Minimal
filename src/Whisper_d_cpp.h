#ifndef _Whisper_H
#define _Whisper_H
#include "AudioWhisper.h"
#include <WiFi.h>

class Whisper {
  WiFiClient client;
  String server;
  uint16_t port_;           // ★追加: 接続先ポートをメンバで保持
  String path_;             // ★追加: APIパスをメンバで保持
public:
  Whisper(const char* server_ip, uint16_t port = 8081, const char* path = "/inference");  // ★path引数追加
  ~Whisper();
  // language: "ja" / "en" / "zh" など Whisper の言語コード（省略時 "ja"）
  // detect_language: true にするとサーバー側で自動判定（language より優先）
  String Transcribe(AudioWhisper* audio,
                    const char* language = "ja",
                    bool detect_language = false);
};

#endif // _Whisper_H
