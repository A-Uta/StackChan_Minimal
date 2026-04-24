/***
 - GitHub StackChan_Minimal

 <For AI server tools>
 - Speech-to-Text
 1. whisper.cpp
    https://github.com/ggml-org/whisper.cpp

 - Local LLM (OpenAI Compatibility API)
 1. llama.cpp
    https://github.com/ggml-org/llama.cpp
 2. LM Studio
    https://lmstudio.ai/
 3. ollama
    https://github.com/ollama/ollama

 - Text-to-Speech
 1. piper-plus
    https://github.com/ayutaz/piper-plus

***/

#include <Wire.h>
#include <Arduino.h>
#include <M5Unified.h>
#include <nvs.h>          // Add for Web Setting
#include <Avatar.h>       // Add for M5 avatar
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <WiFi.h>            // Wi-Fi control (explicit include)
#include <DNSServer.h>       // For http://m5stack/ in AP mode
#include <ESPmDNS.h>         // For http://m5stack.local/ fallback
#include <ArduinoJson.h>
#include "AudioWhisper.h"
#include "Whisper_d_cpp.h"
#include <deque>
#include <atomic>
#include <ESP32WebServer.h> // Add for Web Setting
#include <Preferences.h>    // Add for nvs management
// ===== Sensors =====
#include <MAX3010x.h>       // MAX30100 optical pulse sensor driver (MIT)

#include <mbedtls/base64.h>
#include <memory>               // std::unique_ptr 用

// ---- External I2C for sensors ----
// ATOM S3R Unit/Grove port uses GPIO2 as SDA and GPIO1 as SCL.
// Sensor access is centralized through I2C_UNIT.
static TwoWire& I2C_UNIT = Wire;

// 前方宣言（定義はsetup()より前にあるが、関数より前に呼ぶ箇所があるため）
static void initSpeakerOnce();
static void initUnitI2COnce();
static void ensureSpeakerActive();
static void markSpeakerEndedByAudioWhisper();
static bool speak_piper_http(const String& text, bool allow_cancel);
static bool speak_piper_http_chunked(const String& text, bool allow_cancel);
static std::deque<String> splitForPiperTTS(const String& text, size_t maxChars = 60);
// Phase 3-A: 会話×モーション制御ヘルパー（handle_api_estop等から呼ばれるため前方宣言）
static void motion_on_conversation_start();
static void motion_on_conversation_end();
static void motion_force_stop();
static void ttsTaskFunc(void* arg);
static bool enqueueTtsJob(const String& text, uint8_t expr, bool allow_cancel, bool is_final);
static void requestAvatarUpdate(const String& text, uint8_t expr);
static void applyPendingAvatarUpdate();
static void clearPendingTtsQueue();
// speak_piper_http is defined later (Piper TTS over HTTP)
static constexpr int I2C_UNIT_SDA = 2;
static constexpr int I2C_UNIT_SCL = 1;
static constexpr uint32_t I2C_UNIT_FREQ = 100000;

// nvs
Preferences preferences;

// ===== STT over HTTP (whisper.cpp server) =====
static String WHISPER_SERVER_IP   = "192.168.11.2";   // Whisper.cpp サーバIP (Webメニューで変更可)
static uint16_t WHISPER_SERVER_PORT = 8081;           // Whisper.cpp サーバポート (Webメニューで変更可)
static String WHISPER_SERVER_PATH  = "/inference";    // ★追加: Whisper API パス（NVS保存対象）

// ===== TTS over HTTP (Piper on Termux) =====
static String PIPER_TTS_IP = "192.168.11.2";   // Termux device IP (Webメニューで変更可)
static uint16_t PIPER_TTS_PORT = 5000;         // FastAPI/uvicorn port (Webメニューで変更可)
static float PIPER_TTS_GAIN = 0.25f;           // AudioOutput gain (0.0-1.0)
static float PIPER_TTS_LENGTH_SCALE = 1.00f;   // ★追加: 話速 (0.5=速い / 1.0=標準 / 2.0=遅い)
static std::atomic<bool> g_piper_tts_playing{false}

;

// ===== TTS Task (PR1): Core 0 に再生処理を分離 =====
struct TtsJob {
  char*   text;         // ヒープ確保、ttsTask 側で free する
  bool    allow_cancel;
  bool    is_final;
  uint8_t expression;   // Expression enum を uint8_t で格納
};

static QueueHandle_t       g_ttsQueue         = nullptr;
static TaskHandle_t        g_ttsTaskHandle    = nullptr;
static std::atomic<int>    g_tts_pending_jobs{0};
static std::atomic<bool>   g_tts_cancel_requested{false};

// ===== PR2-C: Avatar update request (loop() 側で実反映) =====
static std::atomic<uint8_t> g_avatar_expr_pending{0};  // 0 = Neutral
static std::atomic<bool>    g_avatar_update_requested{false};
static SemaphoreHandle_t    g_avatar_text_mutex = nullptr;
static String               g_avatar_text_pending = "";

// ===== Piper/TTS test flags (temporary) =====
static bool g_test_piper_direct_mode = false;   // true: start_talking()でLLMを通さず固定文を直接Piper再生
static bool g_test_disable_history   = true;    // true: exec_chatGPT()で会話履歴を使わず今回の質問だけ送る
static bool g_test_run_direct_once   = false;   // true: setup()完了後に固定文Piperテストを1回だけ自動実行

// (legacy UI vars) voice volume/speed
uint8_t PiperPlus_voice_volume = 50;            // 会話音量(Default:50)
uint8_t PiperPlus_voice_speed_legacy = 100;     // 廃止予定: 旧UI互換用（Piper-plusでは未使用）

// ===== VAD calibration async request =====
static volatile bool  g_vad_calibration_requested = false;
static volatile bool  g_vad_calibration_done = false;
static volatile bool  g_vad_calibration_success = false;
static String         g_vad_calibration_result = "";

// ===== リレー（他個体への会話受け渡し）用 =====
String   g_relay_origin_question = "";   // 最初のユーザ質問（ホップ不変）
String   g_relay_previous_answer = "";   // 直前個体の回答（ホップごとに上書き）
int      g_relay_hop             = 0;    // 現在のホップ数（起点受信前=0, 起点→B送信時点で1）
String   g_relay_origin          = "";   // 起点個体のIP
bool     g_is_relay_request      = false; // 現在処理中の会話が relay 経由かどうか

// リレー関連定数
static const int RELAY_MAX_HOPS = 5;
static const int RELAY_HTTP_TIMEOUT_MS = 3000;
static const int RELAY_RETRY_WAIT_MS = 2000;

/// Web or SSL server
ESP32WebServer server(80);  // Add for Web server setting

// ===== Wi-Fi config portal hostname (AP mode) =====
static const char* WIFI_CONFIG_HOST = "m5stack";  // Access: http://m5stack/ (and m5stack.local)
static DNSServer dnsServer;
static bool g_apPortalRunning = false;

using namespace m5avatar;   // Add for M5 avatar
Avatar avatar;              // Add for M5 avatar

/// set M5Speaker virtual channel (0-7)
// static constexpr uint8_t m5spk_virtual_channel = 0;
static constexpr uint8_t m5spk_virtual_channel = 1;
// static AudioOutputM5Speaker out(&M5.Speaker, m5spk_virtual_channel);

String LANG_CODE = "ja-jp";

// ===== Language helpers =====
static String normalizeLangCode(const String& code) {
  String lc = code;
  lc.trim();
  lc.toLowerCase();
  lc.replace("_", "-");

  if (lc.startsWith("zh")) return "zh-CN";
  if (lc.startsWith("en")) return "en-US";
  return "ja-jp";
}

// LANG_CODE ("ja-jp" / "en-US" / "zh-CN" 等) を
// Whisper.cpp server の language フィールド用短縮コードに変換する。
// 未知の値は日本語既定（サーバー起動時の -l ja と同じ）。
static String whisperLanguageFromLangCode(const String& code) {
  String lc = code;
  lc.trim();
  lc.toLowerCase();
  lc.replace("_", "-");

  if (lc.startsWith("ja")) return "ja";
  if (lc.startsWith("en")) return "en";
  if (lc.startsWith("zh")) return "zh";

  return "ja";  // 不明時は日本語既定
}

static const char* languageChangedMessage(const String& code) {
  if (code == "zh-CN") return "角色语音已更改";
  if (code == "en-US") return "Character voice changed";
  return "キャラクター音声が、変更されました";
}

// ===== Display rotation =====
static uint8_t g_display_rotation = 0;   // 0,1,2,3
static volatile bool g_display_rotation_apply_requested = false;
static uint8_t g_display_rotation_pending = 0;

static uint8_t clampDisplayRotation(int rot) {
  if (rot < 0) return 0;
  if (rot > 3) return 3;
  return (uint8_t)rot;
}

static void applyDisplayRotation(uint8_t rot, bool redraw = true) {
  g_display_rotation = clampDisplayRotation(rot);

  // 実際の回転変更だけに限定し、ここでは追加描画を行わない
  // （HTTPハンドラや他タスクと表示キューが衝突しないようにする）
  M5.Display.setRotation(g_display_rotation);

  (void)redraw;  // 互換のため残す
  Serial.printf("[Display] rotation=%u", g_display_rotation);
}


// ===== LLM prompt helpers (LANG_CODE に応じて指示文を切り替え) =====
static String replyInstruction200() {
  String code = LANG_CODE;
  code.toLowerCase();
  if (code.startsWith("en"))
    return "Please answer in short, natural English within 200 characters. "
           "Do not include character counts, parenthetical notes, bullet points, "
           "or numbered lists. Question: ";
  if (code.startsWith("zh"))
    return "请用简短自然的中文回答，控制在200字以内。"
           "不要输出字数统计、括号补充、项目符号或编号列表。问题：";
  return "回答は200文字以内の短い口調でお願いします。"
         "文字数カウント、括弧付き補足、箇条書き番号は出さず、"
         "自然な短い文章で答えてください。質問：";
}

static String replyInstruction100() {
  String code = LANG_CODE;
  code.toLowerCase();
  if (code.startsWith("en"))
    return "Rule: Answer briefly in English within 100 characters. Question: ";
  if (code.startsWith("zh"))
    return "规则：用100字以内的中文简短回答。问题：";
  return "ルール：100文字以内で短く回答。質問：";
}

static String imageDescriptionPrompt() {
  String code = LANG_CODE;
  code.toLowerCase();
  if (code.startsWith("en"))
    return "Please describe this image clearly in English.";
  if (code.startsWith("zh"))
    return "请用中文简明地描述这张图片。";
  return "この画像を日本語でわかりやすく説明してください。";
}

// 前方宣言: handle_character_voice_set() から呼ばれるが、定義は後方にあるため
void setlang_messege();

String message_help_you = ""; // Add for Global language
String message_You_said = ""; // Add for Global language
String message_tinking = "";  // Add for Global language
String message_error = "";    // Add for Global language
String message_cant_hear = ""; // Add for Global language
String message_dont_understand = ""; // Add for Global language

/// 保存する質問と回答の最大数
const int MAX_HISTORY = 2; // Adjust for Atom Echo
/// 過去の質問と回答を保存するデータ構造
std::deque<String> chatHistory;
///---------------------------------------------
String OPENAI_API_KEY = "dummy";     // ローカルLLMのため、dummyを設定
String VOICEVOX_API_KEY = "dummy";   // TTS互換用に dummy を設定
String STT_API_KEY = "";
String TTS_SPEAKER = "&speaker=";
String TTS_PARMS = TTS_SPEAKER;
String speech_text_buffer = "";

// Last full LLM answer (captured from streaming handler) for conversation history
static String g_last_llm_answer = "";

String Speech_Recognition = "";   // 音声認識した内容

/// キャラクターの音声構造
class CHARACTER_VOICE {
  public:
    uint16_t normal, happy, sasa_yaki;
};

/// 選択:キャラクター
String CHARACTER = "";
uint8_t character;

/// 選択:LLM モデル名
String MODEL_VER = "";
uint8_t model_ver;
String LLM_SERVER_IP = "";   // ローカルLLM
String LLM_MODEL_NAME = "";  // ローカルLLM
String NEXT_SPEACH_IP = "";  // ★おしゃべり転送
uint16_t LLM_SERVER_PORT = 0; // ローカルLLM ポート（0=未設定→デフォルト値を使用）

/// 入力:Webからのテキスト入力
String TEXTAREA = "";

// 画像付きWeb入力用（OpenAI互換VLM対応: LM Studio / llama.cpp）
String WEB_IMAGE_DATA_URL = "";
bool   WEB_HAS_IMAGE = false;

#define UART_TX_PIN 2         // PIN for TX (Grove of AtomS3)
#define UART_RX_PIN 1         // PIN for RX (Grove of AtomS3)
#define UART_PORT 1           // use UART1
#define UART_BAUD_RATE 115200 // Speed

// StaticJsonDocument<768> chat_doc; // 旧: サイズ不足でチャット履歴が4ターン以上でnullになる問題あり
DynamicJsonDocument chat_doc(8192); // ヒープ確保で長い会話でもcontent:nullが発生しない
// ローカルLLM - ollama
String json_ChatString = "{\"model\": \"gemma2:2b\",\"stream\": true,\"messages\": [{\"role\": \"user\", \"content\": \"""\"}]}";

String Role_JSON = "";
String InitBuffer = "";

// JSON文字列用エスケープ（" / \ / 改行 等でJSONが壊れるのを防ぐ）
static String jsonEscapeForJsonString(const String& in) {
  String out;
  out.reserve(in.length() + 16);
  for (size_t i = 0; i < in.length(); i++) {
    char c = in[i];
    switch (c) {
      case '\\': out += "\\\\"; break;
      case '\"': out += "\\\""; break;
      case '\n': out += "\\n"; break;
      case '\r': out += "\\r"; break;
      case '\t': out += "\\t"; break;
      default:
        if ((uint8_t)c < 0x20) { /* skip */ }
        else out += c;
        break;
    }
  }
  return out;
}

// ===== Vision用 補助関数 =====

// 区切りタグで本文を切り出す
static bool extractTaggedBlock(const String& body, const String& startTag, const String& endTag, String& out) {
  int s = body.indexOf(startTag);
  int e = body.indexOf(endTag);
  if (s < 0 || e < 0 || e <= s) {
    out = "";
    return false;
  }
  s += startTag.length();
  out = body.substring(s, e);
  out.trim();
  return true;
}

// POST本文をテキスト部と画像部に分離（後方互換あり）
static void parseTextChatBody(const String& body, String& text, String& imageDataUrl) {
  if (body.startsWith("__TEXT_START__")) {
    extractTaggedBlock(body, "__TEXT_START__", "__TEXT_END__", text);
    extractTaggedBlock(body, "__IMAGE_START__", "__IMAGE_END__", imageDataUrl);
  } else {
    // 後方互換: 従来のテキストのみ送信
    text = body;
    imageDataUrl = "";
  }
}

// ===== OpenAI互換バックエンド判定 / デフォルトポート =====
// LM Studio と llama.cpp は共に OpenAI 互換 /v1/chat/completions を使用する。
// Vision (VLM) 経路でも同じ JSON 形式・SSE パーサを共有できるため、
// 分岐条件をこのヘルパーに集約する。
static bool isOpenAICompatVisionBackend() {
  return MODEL_VER == "LMStudio-LLM" || MODEL_VER == "llamacpp-LLM";
}

// ポート決定を1箇所に集約（0=未設定時のデフォルト値）
static uint16_t getDefaultLlmPort() {
  if (LLM_SERVER_PORT != 0) return LLM_SERVER_PORT;
  if (MODEL_VER == "llamacpp-LLM") return 8080;
  if (MODEL_VER == "LMStudio-LLM") return 52626;
  if (MODEL_VER == "ollama-LLM")   return 11434;
  return 8080;
}

static String buildOpenAIVisionChatJson(const String& userText, const String& imageDataUrl) {
  String prompt = userText;
  if (prompt.length() == 0) {
    prompt = imageDescriptionPrompt();
  }

  String safeModel = jsonEscapeForJsonString(LLM_MODEL_NAME);
  String safeText  = jsonEscapeForJsonString(
    replyInstruction200() + prompt
  );
  String safeImage = jsonEscapeForJsonString(imageDataUrl);

  String json;
  json.reserve(safeText.length() + safeImage.length() + 640);

  json += "{\"model\":\"";
  json += safeModel;
  json += "\",\"stream\":true,\"messages\":[{\"role\":\"user\",\"content\":[";
  json += "{\"type\":\"text\",\"text\":\"";
  json += safeText;
  json += "\"},";
  json += "{\"type\":\"image_url\",\"image_url\":{\"url\":\"";
  json += safeImage;
  json += "\"}}";
  json += "]}],\"chat_template_kwargs\":{\"enable_thinking\":false}}";

  return json;
}

// UTF-8文字数で安全に切り詰める（String.substringはバイト単位なので、日本語が途中で壊れないようにする）
static String truncateUtf8Chars(const String& s, size_t maxChars) {
  const uint8_t* p = (const uint8_t*)s.c_str();
  size_t i = 0;
  size_t chars = 0;
  while (p[i] && chars < maxChars) {
    uint8_t c = p[i];
    size_t step = 1;
    if      ((c & 0x80) == 0x00) step = 1;        // 1-byte
    else if ((c & 0xE0) == 0xC0) step = 2;        // 2-byte
    else if ((c & 0xF0) == 0xE0) step = 3;        // 3-byte
    else if ((c & 0xF8) == 0xF0) step = 4;        // 4-byte
    else step = 1;

    // 途中で終端なら止める
    if (!p[i + step - 1]) break;

    i += step;
    chars++;
  }
  return s.substring(0, i);
}

// グローバル変数として追加
// struct VADConfig {
//     float threshold = 800.0f;
//     int min_speech = 10;
//     int max_silence = 20;
//     int pre_frames = 5;
//     int post_frames = 10;
//     bool enabled = false;  // デフォルトは無効（展示会用）
//     String current_mode = "standard";  // 現在のモード
//     bool calibrated = false;  // キャリブレーション状態
// } global_vad_config;

// グローバル変数として追加
// Step 4c: pre_frames / post_frames / current_mode / calibrated を削除。
//   - pre_frames / post_frames は AudioWhisper 側の内部定数化済み(Step 3b)
//   - current_mode は mode切替機能の削除(Step 4b)に伴い不要
//   - calibrated は設計上「現在のthresholdだけが真実」方針に基づき不要
// 注意:
//   既定値(800 / 10 / 20 / false)は敢えて旧値のまま維持する。
//   理由: 真の設定源は NVS。構造体既定値はフォールバックのみの役割。
//   NVS が読めない/壊れている等の異常時に、旧値が出ることで異常検知できる(fail loud)。
//   実運用では ensureVadNvsKeys() とマイグレーション処理で 180/4/40/true に補正済み。
struct VADConfig {
    float threshold = 800.0f;       // NVS未設定時のフォールバック(異常検知用)
    int min_speech = 10;            // 同上
    int max_silence = 20;           // 同上
    bool enabled = false;           // 同上(マイグレーションで true に補正される)
} global_vad_config;

// ===== Control Plane: Phase 1 状態フラグ =====
static volatile bool  g_estop_active  = false;   // estop中はpublish/actionを拒否
static volatile bool  g_audio_busy    = false;   // 録音・TTS中はtrueで排他
static String         g_last_error    = "";       // 直近エラー文字列
// blob: 録音WAVを最新1件だけ保持（PSRAM節約）
static uint8_t*       g_blob_buf      = nullptr;  // PSRAMまたはheap
static size_t         g_blob_size     = 0;
static uint32_t       g_blob_timestamp = 0;       // millis() at record time
static const uint32_t BLOB_TTL_MS     = 30000;    // 30秒でTTL失効
static const size_t   BLOB_MAX_BYTES  = 512 * 1024; // 512KB上限
// 非同期録音リクエスト（HTTPハンドラ→loop()へ委譲）
static volatile bool  g_record_requested = false;  // trueならloop()で録音実行
static volatile bool  g_record_done      = false;  // 録音完了フラグ
static String         g_record_result_json = "";    // 録音結果JSON（成功/失敗）

// ===== Phase 3-A: 会話×モーション連動フラグ =====
static bool     g_demo_motion_enabled   = true;   // 会話連動モーションのON/OFF
static bool     g_talk_motion_active    = false;  // 会話中モーション追跡中か
static uint32_t g_talk_motion_start_ms  = 0;      // 会話モーション開始時刻
static bool     g_talk_motion_nod_done  = false;  // 最初のnodが完了したか

// ============================================================
// Phase 2: Servo Motion Layer
// ============================================================
// ビルドフラグ ENABLE_SERVO で有効化。未定義時はモーション関連コードは空になる。
// サーボはGroveポート(GPIO2/GPIO1)を使うため、I2Cセンサーと排他。
// platformio.ini に -DENABLE_SERVO を追加して有効化すること。
#ifdef ENABLE_SERVO

// ===== Servo driver (Arduino-ESP32 Core v2 LEDC API — espressif32@6.7.0) =====
// Core v2: ledcSetup(ch, freq, res) / ledcAttachPin(pin, ch) / ledcDetachPin(pin) / ledcWrite(ch, duty)
//
// ★ 停止時は ledcDetachPin + end() でPWM信号を完全に切断し、サーボを脱力させる。
//    再開時は begin() で再初期化する。ledcAttach 直後のグリッチによる跳ねは
//    MOTION_WAKE_CENTER（起床動作）で演出として吸収する。

class ServoLEDC {
public:
  bool begin(uint8_t pin,
             uint16_t min_us = 500,
             uint16_t max_us = 2500,
             uint16_t freq_hz = 50,
             uint8_t resolution_bits = 14,
             int initial_angle = 90) {
    _pin = pin;
    _min_us = min_us;
    _max_us = max_us;
    _freq = freq_hz;
    _res = resolution_bits;
    _maxDuty = (1UL << resolution_bits) - 1;
    _period_us = 1000000UL / freq_hz;

    // Core v2: チャンネル = ピン番号をそのまま使う（0-15の範囲で）
    _channel = _pin;
    ledcSetup(_channel, _freq, _res);
    // ★ ピン接続前に正しいdutyをセットしてグリッチを防ぐ
    {
      int a = constrain(initial_angle, 0, 180);
      uint32_t us = map(a, 0, 180, _min_us, _max_us);
      us = constrain((int)us, (int)_min_us, (int)_max_us);
      uint32_t duty = (uint64_t)us * _maxDuty / _period_us;
      ledcWrite(_channel, duty);
    }
    ledcAttachPin(_pin, _channel);
    _lastAngle = initial_angle;
    _attached = true;
    return true;
  }

  void write(int angle) {
    if (!_attached) return;
    angle = constrain(angle, 0, 180);
    _lastAngle = angle;
    uint32_t us = map(angle, 0, 180, _min_us, _max_us);
    writeMicroseconds(us);
  }

  void writeMicroseconds(uint32_t us) {
    if (!_attached) return;
    us = constrain((int)us, (int)_min_us, (int)_max_us);
    uint32_t duty = (uint64_t)us * _maxDuty / _period_us;
    ledcWrite(_channel, duty);
  }

  void end() {
    if (_attached) {
      // ★ detach前にduty=0（常時LOW）にしてパルスを止める。
      //    ledcDetachPin の瞬間にパルス途中で切れるのを防ぐ。
      ledcWrite(_channel, 0);
      delayMicroseconds(100);   // 1PWM周期(20ms)は不要、短い待ちで十分
      ledcDetachPin(_pin);
      _attached = false;
    }
  }

  bool isAttached() const { return _attached; }
  int lastAngle() const { return _lastAngle; }

private:
  uint8_t  _pin = 255;
  uint8_t  _channel = 0;
  uint16_t _min_us = 500;
  uint16_t _max_us = 2500;
  uint16_t _freq = 50;
  uint8_t  _res = 14;
  uint32_t _maxDuty = 0;
  uint32_t _period_us = 20000;
  bool     _attached = false;
  int      _lastAngle = 90;

  void _writeAngleDuty(int angle) {
    int a = constrain(angle, 0, 180);
    uint32_t us = map(a, 0, 180, _min_us, _max_us);
    us = constrain((int)us, (int)_min_us, (int)_max_us);
    uint32_t duty = (uint64_t)us * _maxDuty / _period_us;
    ledcWrite(_channel, duty);
  }
};

// ===== Motion enum (Phase 2: 限定プリセットのみ) =====
enum Motion : uint8_t {
  MOTION_INIT = 0,    // 初期化（A/B=90度で停止、detach）
  MOTION_WAKE_CENTER, // ★ 再開直後のウォームアップ（起床動作）
  MOTION_NOD,         // うんうん（YESYES）
  MOTION_SHAKE,       // イヤイヤ（NONO）
  MOTION_IDLE,        // キョロキョロ（SCAN）
  MOTION_COUNT
};

static const char* motionName(Motion m) {
  switch (m) {
    case MOTION_INIT:         return "init";
    case MOTION_WAKE_CENTER:  return "wake_center";
    case MOTION_NOD:          return "nod";
    case MOTION_SHAKE:        return "shake";
    case MOTION_IDLE:         return "idle";
    default:                  return "unknown";
  }
}

// ===== AtomS3R Grove pins (shared with I2C sensors — mutually exclusive) =====
static constexpr int SERVO_A_PIN = 2;  // Yellow = G2 (上下)
static constexpr int SERVO_B_PIN = 1;  // White  = G1 (左右)

// ===== Servo angle limits =====
static constexpr int A_CENTER = 90;
static constexpr int A_HIGH   = 130;
static constexpr int A_LOW    =   0;
static constexpr int B_CENTER = 90;
static constexpr int B_HIGH   = 180;
static constexpr int B_LOW    =   0;
static constexpr int SAFE_MARGIN_DEG = 5;
static constexpr int A_MIN = A_LOW  + SAFE_MARGIN_DEG;
static constexpr int A_MAX = A_HIGH - SAFE_MARGIN_DEG;
static constexpr int B_MIN = B_LOW  + SAFE_MARGIN_DEG;
static constexpr int B_MAX = B_HIGH - SAFE_MARGIN_DEG;
static inline int clampA(int a) { return constrain(a, A_MIN, A_MAX); }
static inline int clampB(int b) { return constrain(b, B_MIN, B_MAX); }

// ===== Servo instances & state =====
static ServoLEDC servoA;
static ServoLEDC servoB;
static bool servosAttached = false;

static Motion g_motion          = MOTION_INIT;
static uint32_t g_motionStartMs = 0;
static int aCmd = A_CENTER;
static int bCmd = B_CENTER;

// Transition state (Return to Center)
static Motion   g_pendingMotion        = MOTION_INIT;
static bool     g_transitioningToCenter = false;
static uint32_t g_transitionStartMs    = 0;
static int      g_transitionStartA     = A_CENTER;
static int      g_transitionStartB     = B_CENTER;
static uint32_t g_transitionDurationMs = 2000;  // ★ 遷移ごとに切り替え（初期値=SERVO_RETURN_MS相当）

// Timing
static uint32_t nextServoComputeMs = 0;
static uint32_t nextServoAWriteMs  = 0;
static uint32_t nextServoBWriteMs  = 0;
static constexpr int    SERVO_STEP_MS        = 20;
static constexpr uint32_t SERVO_RETURN_MS    = 2000;  // センター復帰にかける時間（停止時: ゆっくり）
static constexpr uint32_t SERVO_WAKE_RETURN_MS = 150; // ★ 開始時のセンター遷移（最短）
static constexpr uint32_t SERVO_B_OFFSET_MS  = 60;
static constexpr uint32_t SERVO_MOVE_DURATION_MS = 3000;
static constexpr uint32_t SERVO_HOLD_MS      = 50;
static constexpr uint32_t SERVO_MOTION_BASE_MS = 800;
static constexpr uint32_t SERVO_WAKE_SETTLE_MS = 350;  // ★ 再開ウォームアップの静止時間（一拍置いてからNOD）

// Speed / acceleration limits（安全優先: 倒れ防止のため控えめに設定）
static constexpr float A_MAX_SPEED_DPS  = 150.0f;   // 220→150 上下速度を抑制
static constexpr float B_MAX_SPEED_DPS  =  80.0f;   // 120→80  左右速度を抑制（倒れやすい方向）
static constexpr float A_MAX_ACCEL_DPS2 = 4000.0f;   // 8000→4000 反転衝撃を半減
static constexpr float B_MAX_ACCEL_DPS2 = 1500.0f;   // 2500→1500 左右の加速を抑制

static float aFilt = (float)A_CENTER;
static float bFilt = (float)B_CENTER;
static float aVel  = 0.0f;
static float bVel  = 0.0f;

// Write deadband
static int aLastWritten = A_CENTER;
static int bLastWritten = B_CENTER;
static constexpr int WRITE_DEADBAND_DEG = 2;

// ★ INIT到達後にdetachをスケジュールするタイマー
static uint32_t g_initDetachScheduledMs = 0;

// ===== Utility functions =====
static float servo_easeInOutCubic(float x) {
  if (x < 0.5f) return 4.0f * x * x * x;
  float u = -2.0f * x + 2.0f;
  return 1.0f - (u * u * u) / 2.0f;
}
static float servo_fract(float x) { return x - floorf(x); }
static inline float servo_lerp(float a, float b, float t) { return a + (b - a) * t; }

// ===== accelTrack: acceleration-limited tracking filter =====
static inline float accelTrack(float current, float target, float &vel,
                               float vmax, float amax, float dt) {
  float dist = target - current;
  float dist_abs = fabsf(dist);
  if (dist_abs < 1.0f && fabsf(vel) < 20.0f) { vel = 0.0f; return target; }

  float v_brake = sqrtf(2.0f * amax * dist_abs) * 0.8f;
  static constexpr float NEAR_ZONE = 7.0f;
  static constexpr float NEAR_MIN  = 15.0f;
  float v_eff = vmax;
  if (dist_abs < NEAR_ZONE) {
    v_eff = NEAR_MIN + (vmax - NEAR_MIN) * (dist_abs / NEAR_ZONE);
  }
  float v_des = copysignf(fminf(v_eff, v_brake), dist);

  float dv = v_des - vel;
  float dv_max = amax * dt;
  if (dv >  dv_max) dv =  dv_max;
  if (dv < -dv_max) dv = -dv_max;
  vel += dv;
  if (vel >  v_eff) vel =  v_eff;
  if (vel < -v_eff) vel = -v_eff;

  current += vel * dt;
  return current;
}

// ===== computeMotionAngles: generate target angles for each motion =====
static void computeMotionAngles(Motion m, float t_sec, int &outA, int &outB) {
  if (m == MOTION_INIT) { outA = A_CENTER; outB = B_CENTER; return; }

  float a = (float)A_CENTER;
  float b = (float)B_CENTER;
  const float A_AMP = (A_MAX - A_MIN) * 0.35f;
  const float B_AMP = (B_MAX - B_MIN) * 0.50f;

  switch (m) {
    case MOTION_WAKE_CENTER: {  // ★ 再開ウォームアップ（センター固定）
      outA = A_CENTER;
      outB = B_CENTER;
      return;
    }
    case MOTION_NOD: {  // うんうん (YESYES)
      float w = 2.0f * (float)M_PI * 1.6f;
      a = (float)A_CENTER + 0.55f * A_AMP * sinf(w * t_sec);
      b = (float)B_CENTER + 0.12f * B_AMP * sinf(w * t_sec + 0.7f);
      break;
    }
    case MOTION_SHAKE: {  // イヤイヤ (NONO)
      float w = 2.0f * (float)M_PI * 2.5f;
      b = (float)B_CENTER + 0.40f * B_AMP * sinf(w * t_sec);
      a = (float)A_CENTER - 0.15f * A_AMP + 0.05f * A_AMP * sinf(w * 0.5f * t_sec);
      break;
    }
    case MOTION_IDLE: {  // キョロキョロ (SCAN)
      static constexpr float B_RANGE_SCAN_DEG = 34.0f;
      static constexpr float SCAN_ENTRY_HOLD_S = 0.30f;
      if (t_sec < SCAN_ENTRY_HOLD_S) { outA = A_CENTER; outB = B_CENTER; return; }
      float t_scan = t_sec - SCAN_ENTRY_HOLD_S;

      float move_s = SERVO_MOVE_DURATION_MS / 1000.0f;
      float hold_s = SERVO_HOLD_MS / 1000.0f;
      float cycle = move_s + 2.0f * hold_s;
      float hold_r = hold_s / cycle;
      float move_r = (move_s * 0.5f) / cycle;
      float start_u = hold_r + move_r * 0.5f;
      float u = servo_fract(t_scan / cycle + start_u);
      float x;

      if (u < hold_r) {
        x = -1.0f;
      } else if (u < hold_r + move_r) {
        float t = (u - hold_r) / move_r;
        t = servo_easeInOutCubic(t);
        x = servo_lerp(-1.0f, 1.0f, t);
      } else if (u < hold_r + move_r + hold_r) {
        x = 1.0f;
      } else {
        float t = (u - (hold_r + move_r + hold_r)) / move_r;
        t = servo_easeInOutCubic(t);
        x = servo_lerp(1.0f, -1.0f, t);
      }
      b = (float)B_CENTER + B_RANGE_SCAN_DEG * x;
      a = (float)A_CENTER + 0.25f * (A_MAX - A_CENTER) * fabsf(x);
      break;
    }
    default: break;
  }
  outA = clampA((int)roundf(a));
  outB = clampB((int)roundf(b));
}

// ===== servoResetFilterState: reset filter on motion transition =====
static void servoResetFilterState() {
  aFilt = (float)aCmd;
  bFilt = (float)bCmd;
  aVel = 0.0f;
  bVel = 0.0f;
}

// ===== servoAttach / servoDetach =====
// ★ Core v2: end() で完全 detach、begin() で再初期化する方式。
//    再開時の跳ねは MOTION_WAKE_CENTER で演出として吸収する。
//    begin() 内で初期角のdutyが1回だけ書かれる。直後の二重writeは
//    初回捕捉を強めるだけなので省略する。
static void servoAttach() {
  if (!servosAttached) {
    servoA.begin(SERVO_A_PIN, 900, 2100, 50, 14, A_CENTER);
    servoB.begin(SERVO_B_PIN, 900, 2100, 50, 14, B_CENTER);
    // ★ 直後の write(CENTER) は省略（begin内の1回だけで十分）
    servosAttached = true;
    aLastWritten = A_CENTER;
    bLastWritten = B_CENTER;
    Serial.println("[Servo] attached (begin only)");
  }
}

static void servoDetach() {
  if (servosAttached) {
    servoA.end();
    servoB.end();
    servosAttached = false;
    Serial.println("[Servo] detached (end, servo free)");
  }
}

// ===== servoRequestMotion: called from API or internal to start a motion =====
static void servoRequestMotion(Motion m) {
  if (g_estop_active && m != MOTION_INIT) {
    Serial.println("[Servo] motion blocked by estop");
    return;
  }

  g_pendingMotion = m;

  // ★ 新モーション要求時はdetachスケジュールをキャンセル
  g_initDetachScheduledMs = 0;

  // Resume if suspended
  if (m != MOTION_INIT) {
    servoAttach();
  }

  // Start return-to-center transition
  g_transitioningToCenter = true;
  g_transitionStartMs = millis();
  g_transitionStartA  = aCmd;
  g_transitionStartB  = bCmd;
  // ★ 開始時（WAKE_CENTER）は短い遷移、停止時（INIT）はゆっくり遷移
  g_transitionDurationMs = (m == MOTION_WAKE_CENTER) ? SERVO_WAKE_RETURN_MS : SERVO_RETURN_MS;
  nextServoComputeMs = 0;
  nextServoAWriteMs  = 0;
  nextServoBWriteMs  = 0;
  servoResetFilterState();

  Serial.printf("[Servo] motion requested -> %s\n", motionName(m));
}

// ===== servoEstopHalt: called when estop is activated =====
static void servoEstopHalt() {
  // Return to center, then detach
  servoRequestMotion(MOTION_INIT);
  Serial.println("[Servo] estop -> INIT");
}

// ===== servoTick: called from loop() every iteration =====
static void servoTick() {
  // ★ INIT到達後の遅延detach処理（サーボ脱力）
  if (g_initDetachScheduledMs != 0 && millis() >= g_initDetachScheduledMs) {
    g_initDetachScheduledMs = 0;
    servoDetach();
    Serial.println("[Servo] INIT: detached (servo free)");
  }

  // INIT中かつ遷移中でなければ何もしない
  if (g_motion == MOTION_INIT && !g_transitioningToCenter) return;

  uint32_t now = millis();

  // 初回タイミング初期化
  if (nextServoComputeMs == 0) {
    nextServoComputeMs = now;
    nextServoAWriteMs  = now;
    nextServoBWriteMs  = now + (g_transitioningToCenter ? 0 : SERVO_B_OFFSET_MS);
  }

  // 角度計算
  if ((int32_t)(now - nextServoComputeMs) >= 0) {
    nextServoComputeMs = now + SERVO_STEP_MS;

    if (g_transitioningToCenter) {
      float u = (now - g_transitionStartMs) / (float)g_transitionDurationMs;
      if (u >= 1.0f) {
        // センター到達
        aCmd = A_CENTER;
        bCmd = B_CENTER;
        g_transitioningToCenter = false;
        servoResetFilterState();

        // モーション確定
        g_motion = g_pendingMotion;
        g_motionStartMs = now;
        Serial.printf("[Servo] motion started -> %s\n", motionName(g_motion));

        // ★ INITならセンターに保持後、50ms待ってから detach（脱力）
        if (g_motion == MOTION_INIT) {
          servoA.write(A_CENTER);
          servoB.write(B_CENTER);
          g_initDetachScheduledMs = millis() + 50;
          Serial.println("[Servo] INIT: will detach after 50ms settle");
        }

        nextServoComputeMs = 0;
        nextServoAWriteMs  = 0;
        nextServoBWriteMs  = 0;
        return;
      }

      float e = servo_easeInOutCubic(constrain(u, 0.0f, 1.0f));
      aCmd = clampA((int)roundf(servo_lerp((float)g_transitionStartA, (float)A_CENTER, e)));
      bCmd = clampB((int)roundf(servo_lerp((float)g_transitionStartB, (float)B_CENTER, e)));
      aFilt = (float)aCmd;
      bFilt = (float)bCmd;
      aVel = 0.0f;
      bVel = 0.0f;
    } else {
      // 通常モーション計算
      float t_sec = (now - g_motionStartMs) / 1000.0f;

      // ★ WAKE_CENTER 終了後に NOD へ自動遷移
      if (g_motion == MOTION_WAKE_CENTER) {
        if ((now - g_motionStartMs) >= SERVO_WAKE_SETTLE_MS) {
          g_motion = MOTION_NOD;
          g_motionStartMs = now;
          servoResetFilterState();
          Serial.println("[Servo] wake complete -> nod");
        }
      }

      float timeScale = (float)SERVO_MOTION_BASE_MS / (float)SERVO_MOVE_DURATION_MS;
      float t_motion = t_sec * timeScale;

      int aDeg, bDeg;
      computeMotionAngles(g_motion, t_motion, aDeg, bDeg);

      const float dt = SERVO_STEP_MS / 1000.0f;
      aFilt = accelTrack(aFilt, (float)aDeg, aVel, A_MAX_SPEED_DPS, A_MAX_ACCEL_DPS2, dt);
      bFilt = accelTrack(bFilt, (float)bDeg, bVel, B_MAX_SPEED_DPS, B_MAX_ACCEL_DPS2, dt);

      aCmd = clampA((int)lroundf(aFilt));
      bCmd = clampB((int)lroundf(bFilt));
    }
  }

  // Servo A write (with deadband)
  if ((int32_t)(now - nextServoAWriteMs) >= 0) {
    nextServoAWriteMs = now + SERVO_STEP_MS;
    if (abs(aCmd - aLastWritten) >= WRITE_DEADBAND_DEG) {
      servoA.write(aCmd);
      aLastWritten = aCmd;
    }
  }

  // Servo B write (with deadband)
  if ((int32_t)(now - nextServoBWriteMs) >= 0) {
    nextServoBWriteMs = now + SERVO_STEP_MS;
    if (abs(bCmd - bLastWritten) >= WRITE_DEADBAND_DEG) {
      servoB.write(bCmd);
      bLastWritten = bCmd;
    }
  }
}

// ===== servoTask: FreeRTOS task running servoTick() on Core 0 =====
// start_talking() は loop() をブロックするため、servoTick() を独立タスクで回す。
// Core 0 で動かすことで Core 1 (loop) の長時間ブロックに影響されない。
static void servoTaskFunc(void* /*arg*/) {
  for (;;) {
    servoTick();
    vTaskDelay(pdMS_TO_TICKS(10));  // 10ms周期で呼ぶ（servoTick内部の20ms制御はそのまま有効）
  }
}

#else
// ===== ENABLE_SERVO not defined: stub functions =====
static void servoAttach() {}
static void servoDetach() {}
static void servoRequestMotion(int) {}
static void servoEstopHalt() {}
static void servoTick() {}
static const char* motionName(int) { return "n/a"; }
#endif // ENABLE_SERVO

// ===== シンプルなクリック回数判定 =====
static int g_clickCount = 0;
static uint32_t g_firstClickMs = 0;
static const uint32_t MULTI_CLICK_WINDOW_MS = 1000;  // 1秒以内の連続クリック判定


// I2C addresses
constexpr uint8_t ADDR_HEART_MAX30100 = 0x57; // MAX30100 (HEART)

// -----------------------------------------------------------------------------
// MAX30100 pulse / SpO2 reference helper
//
// Provides a small application-level wrapper around the MAX30100 sensor driver.
// Heart rate and SpO2 values are uncalibrated reference values for demo use.
// -----------------------------------------------------------------------------
class PulseOximeterCompat {
public:
  using BeatCallback = void (*)();

  PulseOximeterCompat()
  : sensor_(ADDR_HEART_MAX30100, I2C_UNIT) {
  }

  bool begin() {
    reset_();

    if (!sensor_.begin()) {
      return false;
    }

    return true;
  }

  void setOnBeatDetectedCallback(BeatCallback cb) {
    beat_cb_ = cb;
  }

  void update() {
    auto sample = sensor_.readSample(kReadTimeoutMs);
    if (!sample.valid) {
      return;
    }

    const float red = (float)sample.red;
    const float ir  = (float)sample.ir;

    // 指が置かれていない、または反射値が小さすぎる場合は無効扱い
    if (red <= kFingerThreshold || ir <= kFingerThreshold) {
      reset_();
      return;
    }

    // 指を置いた直後の初期化
    if (!finger_detected_) {
      finger_detected_ = true;
      maxValue_ = red;
      minValue_ = red;
      threshold_ = red;
      lastValue_ = red;
      lastHeartbeatMs_ = 0;
      hr_ = 0.0f;
      spo2_ = 0.0f;
      stat_red_.reset();
      stat_ir_.reset();
      return;
    }

    // SpO2参考値用の統計
    stat_red_.process(red);
    stat_ir_.process(ir);

    // 現在の波形レンジ
    const float range = maxValue_ - minValue_;

    // 動的しきい値
    if (range >= kMinDiff) {
      const float targetThreshold = minValue_ + range * kRThreshold;
      threshold_ = threshold_ * (1.0f - kThrRate) + targetThreshold * kThrRate;
    } else {
      threshold_ = red;
    }

    // しきい値を下から上にまたいだら拍動候補
    const bool risingCross =
      (lastValue_ < threshold_) &&
      (red >= threshold_) &&
      (range >= kMinDiff);

    const uint32_t now = millis();

    if (risingCross) {
      if (lastHeartbeatMs_ != 0) {
        const uint32_t deltaMs = now - lastHeartbeatMs_;

        // 30〜240 bpm 相当だけ採用
        if (deltaMs >= 250 && deltaMs <= 2000) {
          const float bpm = 60000.0f / (float)deltaMs;

          // 少しだけ平滑化
          if (hr_ <= 0.0f) {
            hr_ = bpm;
          } else {
            hr_ = hr_ * 0.7f + bpm * 0.3f;
          }

          spo2_ = computeSpo2_();

          if (beat_cb_) {
            beat_cb_();
          }

          // 次の1拍ぶんの統計へ
          stat_red_.reset();
          stat_ir_.reset();
        }
      }

      lastHeartbeatMs_ = now;
    }

    lastValue_ = red;

    // 最大・最小をゆっくり追従させる
    if (red > maxValue_) {
      maxValue_ = red;
    } else {
      maxValue_ -= (maxValue_ - red) * kDecayRate;
    }

    if (red < minValue_) {
      minValue_ = red;
    } else {
      minValue_ += (red - minValue_) * kDecayRate;
    }
  }

  float getHeartRate() const {
    return hr_;
  }

  float getSpO2() const {
    return spo2_;
  }

private:
  struct MinMaxAvg {
    float min_ = 0.0f;
    float max_ = 0.0f;
    double sum_ = 0.0;
    uint16_t n_ = 0;

    void reset() {
      min_ = 0.0f;
      max_ = 0.0f;
      sum_ = 0.0;
      n_ = 0;
    }

    void process(float v) {
      if (n_ == 0) {
        min_ = v;
        max_ = v;
      } else {
        if (v < min_) min_ = v;
        if (v > max_) max_ = v;
      }
      sum_ += v;
      n_++;
    }

    float minimum() const {
      return n_ ? min_ : 0.0f;
    }

    float maximum() const {
      return n_ ? max_ : 0.0f;
    }

    float average() const {
      return n_ ? (float)(sum_ / (double)n_) : 0.0f;
    }
  };

  void reset_() {
    finger_detected_ = false;
    maxValue_ = 0.0f;
    minValue_ = 0.0f;
    threshold_ = 0.0f;
    lastValue_ = 0.0f;
    lastHeartbeatMs_ = 0;
    hr_ = 0.0f;
    spo2_ = 0.0f;
    stat_red_.reset();
    stat_ir_.reset();
  }

  float computeSpo2_() const {
    const float dc_red = stat_red_.average();
    const float dc_ir  = stat_ir_.average();
    const float ac_red = stat_red_.maximum() - stat_red_.minimum();
    const float ac_ir  = stat_ir_.maximum() - stat_ir_.minimum();

    if (!(dc_red > 0.0f && dc_ir > 0.0f && ac_red > 0.0f && ac_ir > 0.0f)) {
      return 0.0f;
    }

    const float rred = ac_red / dc_red;
    const float rir  = ac_ir  / dc_ir;

    if (!(rir > 0.0f)) {
      return 0.0f;
    }

    const float r = rred / rir;

    // 参考値。医療用途ではなく、展示・デモ用の目安。
    const float a = 1.5958422f;
    const float b = -34.6596622f;
    const float c = 112.6898759f;

    float spo2 = a * r * r + b * r + c;

    if (spo2 < 0.0f) spo2 = 0.0f;
    if (spo2 > 100.0f) spo2 = 100.0f;

    return spo2;
  }

private:
  MAX30100 sensor_;
  BeatCallback beat_cb_ = nullptr;

  static constexpr uint32_t kReadTimeoutMs = 20;

  // 実機でHRが出ない場合、まずここを 5000 → 2000 → 1000 に下げて確認
  static constexpr float kFingerThreshold = 10000.0f;

  // 簡易拍動検出用
  static constexpr float kRThreshold = 0.7f;
  static constexpr float kDecayRate  = 0.02f;
  static constexpr float kThrRate    = 0.05f;
  static constexpr float kMinDiff    = 50.0f;

  bool     finger_detected_ = false;
  float    maxValue_ = 0.0f;
  float    minValue_ = 0.0f;
  float    threshold_ = 0.0f;
  float    lastValue_ = 0.0f;
  uint32_t lastHeartbeatMs_ = 0;

  float hr_ = 0.0f;
  float spo2_ = 0.0f;

  MinMaxAvg stat_red_;
  MinMaxAvg stat_ir_;
};

static PulseOximeterCompat pox;

enum class AppMode : uint8_t { kNone, kHeart };
static AppMode active = AppMode::kNone;
static bool hasHeart  = false;

// === HR=0が続いたら通知するための状態 ===
static uint32_t hrZeroStartMs = 0;
static bool     hrZeroRunning = false;
static bool     hrZeroBeeped  = false;

static constexpr uint32_t HR_ZERO_TIMEOUT_MS = 5000; // 5秒
static constexpr float    HR_ZERO_THRESH     = 1.0f; // <=1.0 を「ゼロ」扱い

// ---- 心拍ビープ用（HR→トーン変換 & コールバック） ----
struct ToneSpec { uint16_t freq; uint16_t dur_ms; };

// 5段階（必要に応じて周波数はお好みで）
inline ToneSpec hrToTone(float hr) {
  if (!(hr >= 30.f && hr <= 240.f)) return {0, 0};
  if (hr <  60.f)  return {1100, 80};   // 低い
  if (hr <  90.f)  return {1600, 80};
  if (hr < 120.f)  return {2100, 80};   // 普通
  if (hr < 160.f)  return {2600, 80};
                   return {3100, 80};   // 高い
}

static void onBeatDetected() {
  const float hr = pox.getHeartRate();
  ToneSpec t = hrToTone(hr);
  if (t.freq) {
    M5.Speaker.tone(t.freq, t.dur_ms, 0, false);
    Serial.printf("Beep HR=%.1f bpm\n", hr);
  }
}


// -----------------------------------------------------------------------------
// I2C helpers
// -----------------------------------------------------------------------------
bool i2cPing(uint8_t addr) {
  I2C_UNIT.beginTransmission(addr);
  return (I2C_UNIT.endTransmission() == 0);
}

bool i2cReadReg(uint8_t addr, uint8_t reg, uint8_t &val) {
  I2C_UNIT.beginTransmission(addr);
  I2C_UNIT.write(reg);
  if (I2C_UNIT.endTransmission(false) != 0) return false; // repeated start
  if (I2C_UNIT.requestFrom(addr, (uint8_t)1) != 1) return false;
  val = I2C_UNIT.read();

  return true;
}

static void scanI2CBus(TwoWire& bus, const char* name) {
  Serial.printf("[I2C] scan %s start\n", name);

  int found = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    bus.beginTransmission(addr);
    uint8_t err = bus.endTransmission();

    if (err == 0) {
      Serial.printf("[I2C] %s found: 0x%02X\n", name, addr);
      found++;
    }
    delay(2);
  }

  Serial.printf("[I2C] scan %s total=%d\n", name, found);
}


void detectSensors() {
  hasHeart = i2cPing(ADDR_HEART_MAX30100);
  if (hasHeart) active = AppMode::kHeart;
  else          active = AppMode::kNone;
}

// === 3. 関数宣言 ===
String SpeechToText(bool isGoogle);
void handle_vad_test_recording();
void handle_vad_set_threshold();
// 削除済み(Step 4b): handle_vad_test_fixed

// === Wi-Fi 設定用 関数宣言 ===
void startWifiConfigPortal();
void handle_wifi_config();
void handle_wifi_scan();
void handle_wifi_save();

// ===== VAD設定を AudioWhisper へ適用する共通関数 =====
// この関数は録音開始の直前に呼ぶこと。
// global_vad_config の内容を AudioWhisper の静的パラメータへ反映する。
// 注意: pre_frames / post_frames は AudioWhisper 側の内部定数として固定されており、
//       ここから渡しても無視される(Step 3b 以降)。
//       VADConfig の pre_frames / post_frames メンバの削除は Step 4 で実施予定。
static void applyCurrentVadConfigToAudioWhisper() {
    AudioWhisper::SetVADParameters(
        global_vad_config.threshold,
        global_vad_config.min_speech,
        global_vad_config.max_silence
    );
    Serial.printf("[VAD] config applied: threshold=%.1f, min_speech=%d, max_silence=%d\n",
                  global_vad_config.threshold,
                  global_vad_config.min_speech,
                  global_vad_config.max_silence);
}

// ===== 録音入口の共通関数 =====
// 第2引数 force_fixed=true のときのみ固定録音。
// 通常はVAD録音(global_vad_config.enabled が true のとき)。
// 録音実行前に applyCurrentVadConfigToAudioWhisper() を必ず呼ぶ。
static void runAudioRecording(AudioWhisper& audio, bool force_fixed = false) {
    // 録音前の共通前処理
    markSpeakerEndedByAudioWhisper();

    // 現在のVAD設定を録音コアへ反映
    applyCurrentVadConfigToAudioWhisper();

    // 録音方式の分岐
    const bool use_vad = global_vad_config.enabled && !force_fixed;
    if (use_vad) {
        Serial.println("[VAD] VAD録音を実行");
        audio.Record();
    } else {
        Serial.println("[VAD] 固定時間録音を実行");
        audio.RecordFixed();
    }
}

// === Function: Import HTML Files  ===
#define EMBED_TEXT(section, relpath, symbol) \
  extern const char symbol[]; \
  extern const char symbol##_end[]; \
  asm( \
    ".section " #section "\n" \
    ".balign 4\n" \
    ".global " #symbol "\n" \
    ".global " #symbol "_end\n" \
    #symbol ":\n" \
    ".incbin \"" relpath "\"\n" \
    ".byte 0\n" \
    #symbol "_end:\n" \
    ".previous\n" \
  )
#define EMBED_TEXT_LEN(symbol) ((size_t)(symbol##_end - symbol - 1))

// === HTML Files in incbin folder ===
EMBED_TEXT(.rodata, "incbin/html/head.html", HEAD);
EMBED_TEXT(.rodata, "incbin/html/wifi_config.html", WIFI_CONFIG_HTML);
EMBED_TEXT(.rodata, "incbin/html/character_voice.html", CHARACTER_VOICE_HTML);
EMBED_TEXT(.rodata, "incbin/html/piper_plus_voice.html", PIPER_PLUS_VOICE_HTML);
EMBED_TEXT(.rodata, "incbin/html/model.html", MODEL_HTML);
EMBED_TEXT(.rodata, "incbin/html/text_chat.html", TEXT_CHAT_HTML);
EMBED_TEXT(.rodata, "incbin/html/vad_calibration.html", VAD_CALIBRATION_HTML);
    
static void applyPendingAvatarUpdate() {
  if (!g_avatar_update_requested.load()) return;
  g_avatar_update_requested.store(false);

  Expression expr = (Expression)g_avatar_expr_pending.load();
  avatar.setExpression(expr);

  if (g_avatar_text_mutex && xSemaphoreTake(g_avatar_text_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    avatar.setSpeechText(g_avatar_text_pending.c_str());
    xSemaphoreGive(g_avatar_text_mutex);
  }
}

static void clearPendingTtsQueue() {
  if (!g_ttsQueue) return;

  TtsJob dropped;
  int droppedCount = 0;
  while (xQueueReceive(g_ttsQueue, &dropped, 0) == pdTRUE) {
    if (dropped.text) free(dropped.text);
    g_tts_pending_jobs.fetch_sub(1);
    droppedCount++;
  }
  if (droppedCount > 0) {
    Serial.printf("[TTS] cleared %d queued jobs", droppedCount);
  }
}


void handleRoot() {
  // ★ここを先頭に追加
  Serial.printf("[HTTP] Host=%s URI=%s\n",
                server.hostHeader().c_str(),
                server.uri().c_str());

  String message = "";
  message += "<h1>設定メニュー</h1>";
  message += "\n<ul>";
  message += "\n  <li><a href='wifi_config'>Wi-Fi設定</a></li>";   // ★追加  
  message += "\n  <li><a href='piper_plus_voice'>Piper-plus音声設定</a></li>";
  message += "\n  <li><a href='vad_calibration'>VAD音声設定</a></li>";  // 新規追加
  message += "\n  <li><a href='model_ver'>対話型AIモデルの設定</a></li>";
  message += "\n  <li><a href='text_chat'>テキストで会話</a></li>";
  message += "\n</ul>";
  server.send(200, "text/html", String(HEAD) + String("<body>") + message + String("</body>"));
}

// 新しいハンドラー関数群（main.cppに追加）
void handle_vad_calibration() {
    server.send(200, "text/html", VAD_CALIBRATION_HTML);
}

void handle_vad_status() {
    String status = "";
    status += "=== VAD設定状況 ===\n";
    status += "VAD録音: 有効\n";  // 設計方針により常時有効
    status += "音声検出閾値: " + String(global_vad_config.threshold, 1) + "\n";
    status += "最小発話時間: " + String(global_vad_config.min_speech) + "フレーム\n";
    status += "最大無音時間: " + String(global_vad_config.max_silence) + "フレーム\n";
    // pre/post フレームは内部定数化済み(Step 3b)
    // mode / calibrated は状態として不要と判断(Step 4a)
    
    server.send(200, "text/plain", status);
}

void handle_vad_set_threshold() {
    if (server.method() != HTTP_POST) {
        server.send(405, "text/plain", "Method Not Allowed");
        return;
    }

    if (!server.hasArg("threshold")) {
        server.send(400, "text/plain", "missing threshold");
        return;
    }

    float th = server.arg("threshold").toFloat();

    // UI側の想定範囲に制限
    if (th < 100.0f) th = 100.0f;
    if (th > 300.0f) th = 300.0f;

    global_vad_config.threshold = th;

    preferences.begin("my-app", false);
    preferences.putFloat("vad_threshold", th);
    preferences.end();

    Serial.printf("[VAD] threshold updated via web: %.1f\n", th);
    server.send(200, "text/plain", "ok");
}

void handle_vad_calibration_exec() {
    if (server.method() != HTTP_POST) {
        server.send(405, "text/plain", "Method Not Allowed");
        return;
    }

    if (g_vad_calibration_requested) {
        server.send(409, "text/plain", "Calibration already running");
        return;
    }

    Serial.println("Web経由でVADキャリブレーション要求を受信");
    g_vad_calibration_requested = true;
    g_vad_calibration_done = false;
    g_vad_calibration_success = false;
    g_vad_calibration_result = "";

    server.send(200, "text/plain", "Calibration started");
}


void handle_vad_test_recording() {
    if (server.method() != HTTP_POST) return;
    
    Serial.println("VAD録音テスト開始");
    avatar.setSpeechText("テスト録音中…");
    
    // テスト録音実行
    String ret = SpeechToText(false);
    
    if (ret != "") {
        String result_msg = "テスト成功: " + ret;
        avatar.setSpeechText(result_msg.c_str());
        Serial.println("テスト結果: " + ret);
        server.send(200, "text/plain", "Test completed: " + ret);
    } else {
        avatar.setSpeechText("テスト失敗");
        Serial.println("テスト失敗");
        server.send(500, "text/plain", "Test failed");
    }
    
    delay(3000);
    avatar.setSpeechText("");
}

static void runVadCalibration() {
    Serial.println("[VAD] calibration worker start");

    // Speaker→Mic（I2S競合を避ける）
    M5.Speaker.end();
    markSpeakerEndedByAudioWhisper();
    yield();
    delay(150);
    yield();

    if (!M5.Mic.begin()) {
        Serial.println("[VAD] calibration failed: mic begin");
        g_vad_calibration_success = false;
        g_vad_calibration_result = "Calibration begin failed";
        ensureSpeakerActive();
        g_vad_calibration_done = true;
        return;
    }

    yield();
    delay(100);
    yield();

    float noise_samples[30];
    int valid_samples = 0;

    for (int i = 0; i < 30; i++) {
        int16_t audio_buffer[128];

        if (M5.Mic.record(audio_buffer, 128, 16000)) {
            float sum = 0.0f;
            for (int j = 0; j < 128; j++) {
                float v = (float)audio_buffer[j];
                sum += v * v;
            }
            float rms = sqrt(sum / 128.0f);

            if (rms > 0.0f && rms < 5000.0f) {
                noise_samples[valid_samples++] = rms;
            }
        }

        yield();
        delay(100);
        yield();
    }

    M5.Mic.end();
    yield();
    delay(150);
    yield();

    if (valid_samples > 10) {

        // ノイズサンプルを昇順ソートして中央値を使う
        for (int i = 0; i < valid_samples - 1; i++) {
            for (int j = i + 1; j < valid_samples; j++) {
                if (noise_samples[j] < noise_samples[i]) {
                    float tmp = noise_samples[i];
                    noise_samples[i] = noise_samples[j];
                    noise_samples[j] = tmp;
                }
            }
        }

        float noise_median = 0.0f;
        if (valid_samples % 2 == 0) {
            noise_median = (noise_samples[valid_samples / 2 - 1] +
                            noise_samples[valid_samples / 2]) * 0.5f;
        } else {
            noise_median = noise_samples[valid_samples / 2];
        }

        // 平均×3 では強すぎるため、中央値×1.4 に変更
        float new_threshold = noise_median * 1.4f;

        // 実運用域に制限
        if (new_threshold < 120.0f) new_threshold = 120.0f;
        if (new_threshold > 220.0f) new_threshold = 220.0f;

        global_vad_config.threshold = new_threshold;

        preferences.begin("my-app", false);
        preferences.putFloat("vad_threshold", global_vad_config.threshold);
        preferences.end();

        Serial.printf("[VAD] calibration done: noise_median=%.1f, threshold=%.1f\n",
                      noise_median, global_vad_config.threshold);

        g_vad_calibration_success = true;
        g_vad_calibration_result = "Calibration completed";
    } else {
        Serial.println("[VAD] calibration failed: insufficient samples");
        g_vad_calibration_success = false;
        g_vad_calibration_result = "Calibration failed";
    }

    ensureSpeakerActive();
    g_vad_calibration_done = true;
}


// ============================================================
// Control Plane API v1  (Phase 1)
// ============================================================

// ---- 共通ユーティリティ ----

// JSON文字列値を安全にエスケープしてダブルクォートで囲む
static String apiJsonStr(const String& v) {
  return "\"" + jsonEscapeForJsonString(v) + "\"";
}

// 現在の表情名を文字列で返す（M5Avatar は enum なのでテーブル変換）
static String currentExpressionName() {
  // avatar.getExpression() が返す enum → 文字列
  // ※ コンパイルエラーが出る場合はこの関数ごと削除して state から expression フィールドを "n/a" 固定にする
  switch (avatar.getExpression()) {
    case Expression::Neutral:  return "neutral";
    case Expression::Happy:    return "happy";
    case Expression::Sad:      return "sad";
    case Expression::Doubt:    return "doubt";
    case Expression::Sleepy:   return "sleepy";
    case Expression::Angry:    return "angry";
    default:                   return "unknown";
  }
}

// ---- GET /api/v1/health ----
void handle_api_health() {
  server.send(200, "application/json",
    "{\"status\":\"ok\","
    "\"uptime_ms\":" + String(millis()) + ","
    "\"estop\":" + String(g_estop_active ? "true" : "false") +
    "}");
}

// ---- GET /api/v1/state ----
void handle_api_state() {
  String wifiStatus = (WiFi.status() == WL_CONNECTED)
    ? WiFi.localIP().toString() : "disconnected";

  String json = "{";
  json += "\"estop\":"       + String(g_estop_active ? "true" : "false") + ",";
  json += "\"audio_busy\":"  + String(g_audio_busy   ? "true" : "false") + ",";
  json += "\"tts_playing\":" + String(g_piper_tts_playing ? "true" : "false") + ",";
  json += "\"wifi\":"        + apiJsonStr(wifiStatus) + ",";
  json += "\"vad_enabled\":" + String(global_vad_config.enabled ? "true" : "false") + ",";
  json += "\"expression\":"  + apiJsonStr(currentExpressionName()) + ",";
  json += "\"whisper_ip\":"  + apiJsonStr(WHISPER_SERVER_IP) + ",";
  json += "\"whisper_port\":" + String(WHISPER_SERVER_PORT) + ",";
  json += "\"whisper_path\":" + apiJsonStr(WHISPER_SERVER_PATH) + ",";
  json += "\"piper_ip\":"    + apiJsonStr(PIPER_TTS_IP) + ",";
  json += "\"piper_port\":"  + String(PIPER_TTS_PORT) + ",";
  json += "\"llm_ip\":"      + apiJsonStr(LLM_SERVER_IP) + ",";
  json += "\"llm_model\":"   + apiJsonStr(LLM_MODEL_NAME) + ",";
#ifdef ENABLE_SERVO
  json += "\"motion\":"      + apiJsonStr(String(motionName(g_motion))) + ",";
  json += "\"servo_attached\":" + String(servosAttached ? "true" : "false") + ",";
  json += "\"servo_suspended\":" + String(!servoA.isAttached() ? "true" : "false") + ",";
#endif
  json += "\"last_error\":"  + apiJsonStr(g_last_error);
  json += "}";
  server.send(200, "application/json", json);
}

// ---- GET /api/v1/capabilities ----
void handle_api_capabilities() {
  server.send(200, "application/json",
    "{\"publish\":[\"face/cmd\",\"motion/cmd\"],"
    "\"service\":[\"config.get\",\"config.set\"],"
    "\"action\":[\"audio.record\"],"
    "\"estop\":true,"
    "\"tts_engine\":\"piper\","
    "\"stt_engine\":\"whisper\","
#ifdef ENABLE_SERVO
    "\"motion\":[\"nod\",\"shake\",\"idle\",\"init\"],"
    "\"servo_enabled\":true,"
#else
    "\"motion\":[],"
    "\"servo_enabled\":false,"
#endif
    "\"sensors\":[\"bme688\",\"max30100\"]"
    "}");
}

// ---- POST /api/v1/publish ----
// Body: {"topic":"face/cmd","payload":{"expression":"happy"}}
void handle_api_publish() {
  if (server.method() != HTTP_POST) {
    server.send(405, "application/json", "{\"error\":\"method_not_allowed\"}");
    return;
  }
  if (g_estop_active) {
    server.send(503, "application/json", "{\"error\":\"estop_active\"}");
    return;
  }

  String body = server.arg("plain");
  DynamicJsonDocument doc(512);
  if (deserializeJson(doc, body)) {
    server.send(400, "application/json", "{\"error\":\"invalid_json\"}");
    return;
  }

  const char* topic = doc["topic"] | "";

  if (strcmp(topic, "face/cmd") == 0) {
    const char* expr = doc["payload"]["expression"] | "neutral";
    if      (strcmp(expr, "happy")   == 0) avatar.setExpression(Expression::Happy);
    else if (strcmp(expr, "sad")     == 0) avatar.setExpression(Expression::Sad);
    else if (strcmp(expr, "doubt")   == 0) avatar.setExpression(Expression::Doubt);
    else if (strcmp(expr, "sleepy")  == 0) avatar.setExpression(Expression::Sleepy);
    else if (strcmp(expr, "angry")   == 0) avatar.setExpression(Expression::Angry);
    else                                    avatar.setExpression(Expression::Neutral);

    const char* text = doc["payload"]["text"] | "";
    if (strlen(text) > 0) avatar.setSpeechText(text);

    server.send(200, "application/json", "{\"ok\":true}");

  } else if (strcmp(topic, "motion/cmd") == 0) {
#ifdef ENABLE_SERVO
    const char* motion_name = doc["payload"]["motion"] | "";
    if (strcmp(motion_name, "nod") == 0) {
      servoRequestMotion(MOTION_NOD);
    } else if (strcmp(motion_name, "shake") == 0) {
      servoRequestMotion(MOTION_SHAKE);
    } else if (strcmp(motion_name, "idle") == 0) {
      servoRequestMotion(MOTION_IDLE);
    } else if (strcmp(motion_name, "init") == 0 || strcmp(motion_name, "stop") == 0) {
      servoRequestMotion(MOTION_INIT);
    } else {
      server.send(400, "application/json", "{\"error\":\"unknown_motion\"}");
      return;
    }
    server.send(200, "application/json", "{\"ok\":true}");
#else
    server.send(200, "application/json",
      "{\"ok\":true,\"note\":\"servo not enabled (build with -DENABLE_SERVO)\"}");
#endif

  } else {
    server.send(400, "application/json", "{\"error\":\"unknown_topic\"}");
  }
}

// ---- POST /api/v1/service ----
// Body: {"service":"config.get"} または {"service":"config.set","params":{...}}
void handle_api_service() {
  if (server.method() != HTTP_POST) {
    server.send(405, "application/json", "{\"error\":\"method_not_allowed\"}");
    return;
  }

  String body = server.arg("plain");
  DynamicJsonDocument doc(1024);
  if (deserializeJson(doc, body)) {
    server.send(400, "application/json", "{\"error\":\"invalid_json\"}");
    return;
  }

  const char* svc = doc["service"] | "";

  // --- config.get ---
  if (strcmp(svc, "config.get") == 0) {
    String json = "{";
    json += "\"whisper_ip\":"     + apiJsonStr(WHISPER_SERVER_IP) + ",";
    json += "\"whisper_port\":"   + String(WHISPER_SERVER_PORT) + ",";
    json += "\"whisper_path\":"   + apiJsonStr(WHISPER_SERVER_PATH) + ",";
    json += "\"piper_ip\":"       + apiJsonStr(PIPER_TTS_IP) + ",";
    json += "\"piper_port\":"     + String(PIPER_TTS_PORT) + ",";
    json += "\"piper_length_scale\":" + String(PIPER_TTS_LENGTH_SCALE, 2) + ",";
    json += "\"llm_server_ip\":"  + apiJsonStr(LLM_SERVER_IP) + ",";
    json += "\"llm_model\":"      + apiJsonStr(LLM_MODEL_NAME);
    json += "}";
    server.send(200, "application/json", json);
    return;
  }

  // --- config.set ---
  if (strcmp(svc, "config.set") == 0) {
    if (!doc.containsKey("params")) {
      server.send(400, "application/json", "{\"error\":\"missing_params\"}");
      return;
    }
    JsonObject p = doc["params"];
    bool changed = false;

    if (p.containsKey("whisper_ip")) {
      WHISPER_SERVER_IP = p["whisper_ip"].as<String>();
      changed = true;
    }
    if (p.containsKey("whisper_port")) {
      // ★ バリデーション: 1〜65535
      int port = p["whisper_port"].as<int>();
      if (port < 1 || port > 65535) {
        server.send(400, "application/json", "{\"error\":\"whisper_port out of range (1-65535)\"}");
        return;
      }
      WHISPER_SERVER_PORT = (uint16_t)port;
      changed = true;
    }
    if (p.containsKey("whisper_path")) {
      String path = p["whisper_path"].as<String>();
      if (!path.startsWith("/")) {
        server.send(400, "application/json", "{\"error\":\"whisper_path must start with /\"}");
        return;
      }
      WHISPER_SERVER_PATH = path;
      changed = true;
    }
    if (p.containsKey("piper_ip")) {
      PIPER_TTS_IP = p["piper_ip"].as<String>();
      changed = true;
    }
    if (p.containsKey("piper_port")) {
      // ★ バリデーション: 1〜65535
      int port = p["piper_port"].as<int>();
      if (port < 1 || port > 65535) {
        server.send(400, "application/json", "{\"error\":\"piper_port out of range (1-65535)\"}");
        return;
      }
      PIPER_TTS_PORT = (uint16_t)port;
      changed = true;
    }
    if (p.containsKey("piper_length_scale")) {
      // ★ バリデーション: 0.5〜2.0（UIの handle_piper_plus_voice_set() と同じ範囲）
      float ls = p["piper_length_scale"].as<float>();
      if (ls < 0.5f || ls > 2.0f) {
        server.send(400, "application/json", "{\"error\":\"piper_length_scale out of range (0.5-2.0)\"}");
        return;
      }
      PIPER_TTS_LENGTH_SCALE = ls;
      changed = true;
    }
    if (p.containsKey("llm_server_ip")) {
      LLM_SERVER_IP = p["llm_server_ip"].as<String>();
      changed = true;
    }
    if (p.containsKey("llm_model")) {
      LLM_MODEL_NAME = p["llm_model"].as<String>();
      changed = true;
    }

    if (changed) {
      // NVS 保存（既存の "my-app" namespace に統一）
      // ★ キー名は現行ソースに合わせる
      preferences.begin("my-app", false);
      preferences.putString("whisper_ip",    WHISPER_SERVER_IP);
      preferences.putUShort("whisper_port",  WHISPER_SERVER_PORT);
      preferences.putString("whisper_path",  WHISPER_SERVER_PATH);
      preferences.putString("piper_tts_ip",  PIPER_TTS_IP);
      preferences.putUShort("piper_tts_port", PIPER_TTS_PORT);
      preferences.putFloat("piper_ls",       PIPER_TTS_LENGTH_SCALE);
      preferences.putString("llm_server_ip", LLM_SERVER_IP);
      preferences.putString("llm_model_name", LLM_MODEL_NAME);
      preferences.end();
    }

    server.send(200, "application/json", "{\"ok\":true}");
    return;
  }

  server.send(400, "application/json", "{\"error\":\"unknown_service\"}");
}

// ---- POST /api/v1/action ----
// Body: {"action":"audio.record"}
// 録音はHTTPハンドラ内で同期実行するとタイムアウトでクラッシュするため、
// 202 Accepted を即返しして loop() 側で非同期実行する。
// 結果は GET /api/v1/action/result でポーリング取得する。
void handle_api_action() {
  if (server.method() != HTTP_POST) {
    server.send(405, "application/json", "{\"error\":\"method_not_allowed\"}");
    return;
  }
  if (g_estop_active) {
    server.send(503, "application/json", "{\"error\":\"estop_active\"}");
    return;
  }
  if (g_audio_busy || g_record_requested) {
    server.send(409, "application/json", "{\"error\":\"audio_busy\"}");
    return;
  }

  String body = server.arg("plain");
  DynamicJsonDocument doc(256);
  if (deserializeJson(doc, body)) {
    server.send(400, "application/json", "{\"error\":\"invalid_json\"}");
    return;
  }

  const char* action = doc["action"] | "";

  if (strcmp(action, "audio.record") == 0) {
    // 非同期録音をリクエスト
    g_record_done = false;
    g_record_result_json = "";
    g_record_requested = true;

    server.send(202, "application/json",
      "{\"ok\":true,\"status\":\"recording\",\"hint\":\"poll GET /api/v1/action/result\"}");
    return;
  }

  server.send(400, "application/json", "{\"error\":\"unknown_action\"}");
}

// ---- GET /api/v1/action/result ----
// 非同期録音の結果をポーリングで取得する
void handle_api_action_result() {
  if (g_record_requested && !g_record_done) {
    // まだ録音中
    server.send(202, "application/json", "{\"status\":\"recording\"}");
    return;
  }
  if (g_record_done) {
    // 録音完了 → 結果を返してフラグリセット
    String result = g_record_result_json;
    g_record_done = false;
    g_record_result_json = "";
    // result の中身で成功/失敗を判断（errorキーの有無）
    int httpCode = result.indexOf("\"error\"") >= 0 ? 500 : 200;
    server.send(httpCode, "application/json", result);
    return;
  }
  // リクエストもされていない
  server.send(404, "application/json", "{\"error\":\"no_pending_action\"}");
}

// loop() から呼ばれる非同期録音実行関数
static void executeAsyncRecord() {
  g_audio_busy = true;

  // 既存 blob を破棄
  if (g_blob_buf) {
    free(g_blob_buf);
    g_blob_buf  = nullptr;
    g_blob_size = 0;
  }

  // Speaker→Mic 切り替え（SpeechToTextと同じ手順）
  markSpeakerEndedByAudioWhisper();

  // 録音実行
  AudioWhisper* audio = new AudioWhisper();
  runAudioRecording(*audio);  

  size_t sz = audio->GetSize();
  if (sz <= 44 + 500) {
    delete audio;
    g_audio_busy = false;
    ensureSpeakerActive();
    g_last_error = "record_too_short";
    g_record_result_json = "{\"error\":\"record_too_short\"}";
    g_record_done = true;
    g_record_requested = false;
    return;
  }
  if (sz > BLOB_MAX_BYTES) {
    delete audio;
    g_audio_busy = false;
    ensureSpeakerActive();
    g_last_error = "record_too_large";
    g_record_result_json = "{\"error\":\"record_too_large\"}";
    g_record_done = true;
    g_record_requested = false;
    return;
  }

  // PSRAM 優先でコピー確保
  g_blob_buf = (uint8_t*)ps_malloc(sz);
  if (!g_blob_buf) g_blob_buf = (uint8_t*)malloc(sz);
  if (!g_blob_buf) {
    delete audio;
    g_audio_busy = false;
    ensureSpeakerActive();
    g_last_error = "malloc_failed";
    g_record_result_json = "{\"error\":\"malloc_failed\"}";
    g_record_done = true;
    g_record_requested = false;
    return;
  }

  memcpy(g_blob_buf, audio->GetBuffer(), sz);
  g_blob_size      = sz;
  g_blob_timestamp = millis();
  delete audio;

  g_audio_busy = false;
  ensureSpeakerActive();

  // 成功結果JSON
  String blob_id = String(g_blob_timestamp);
  g_record_result_json = "{\"ok\":true,\"blob_id\":" + apiJsonStr(blob_id) +
    ",\"size\":" + String(sz) + "}";
  g_record_done = true;
  g_record_requested = false;
}

// ---- GET /api/v1/blob?id=<blob_id> ----
// ※ 仕様: クエリパラメータ ?id= 形式（パスパラメータではない）
void handle_api_blob() {
  // TTL チェック
  if (!g_blob_buf || g_blob_size == 0) {
    server.send(404, "application/json", "{\"error\":\"no_blob\"}");
    return;
  }
  if ((millis() - g_blob_timestamp) > BLOB_TTL_MS) {
    free(g_blob_buf);
    g_blob_buf  = nullptr;
    g_blob_size = 0;
    server.send(410, "application/json", "{\"error\":\"blob_expired\"}");
    return;
  }

  // id 一致チェック（省略時はスキップ）
  if (server.hasArg("id")) {
    String req_id = server.arg("id");
    String cur_id = String(g_blob_timestamp);
    if (req_id != cur_id) {
      server.send(404, "application/json", "{\"error\":\"blob_id_mismatch\"}");
      return;
    }
  }

  server.send(200, "audio/wav",
    String((const char*)g_blob_buf, g_blob_size));
}

// ---- POST /api/v1/estop ----
void handle_api_estop() {
  if (server.method() != HTTP_POST) {
    server.send(405, "application/json", "{\"error\":\"method_not_allowed\"}");
    return;
  }
  g_estop_active = true;

  // TTS を即停止
  M5.Speaker.stop();
  g_piper_tts_playing = false;

  // 会話連動モーションを強制停止
  motion_force_stop();

  // サーボをセンター復帰→detach
  servoEstopHalt();

  Serial.println("[ESTOP] activated");
  server.send(200, "application/json", "{\"ok\":true,\"estop\":true}");
}

// ---- POST /api/v1/estop/clear ----
void handle_api_estop_clear() {
  if (server.method() != HTTP_POST) {
    server.send(405, "application/json", "{\"error\":\"method_not_allowed\"}");
    return;
  }
  g_estop_active = false;
  Serial.println("[ESTOP] cleared");
  server.send(200, "application/json", "{\"ok\":true,\"estop\":false}");
}


void handleNotFound(){
  // ★ここを一番先頭（returnより前）に
  Serial.printf("[HTTP] Host=%s URI=%s\n",
                server.hostHeader().c_str(),
                server.uri().c_str());

  // （以下、既存のAPモード時リダイレクト等）
  if (g_apPortalRunning) {
    // /api/ はリダイレクト除外（APモード中でも Control Plane API を生かす）
    if (server.uri().startsWith("/api/")) {
      server.send(503, "application/json",
                  "{\"error\":\"wifi_not_connected\",\"hint\":\"connect to local WiFi first\"}");
      return;
    }
    server.sendHeader("Location", String("/"), true);
    server.send(302, "text/plain", "");
    return;
  }

  String message = "File Not Found\n\n";
  message += "URI: ";
  message += server.uri();
  message += "\nMethod: ";
  message += (server.method() == HTTP_GET)?"GET":"POST";
  message += "\nArguments: ";
  message += server.args();
  message += "\n";
  for (uint8_t i=0; i<server.args(); i++){
    message += " " + server.argName(i) + ": " + server.arg(i) + "\n";
  }
  server.send(404, "text/html", String(HEAD) + String("<body>") + message + String("</body>"));
}

// ===== Wi-Fi 設定用ハンドラ =====
// 設定画面本体
void handle_wifi_config() {
  // 既存設定を読み込んでプレースホルダを埋める
  preferences.begin("wifi", true);
  String currentSsid = preferences.getString("ssid", "");
  preferences.end();

  String page = WIFI_CONFIG_HTML;
  page.replace("{{CURRENT_SSID}}", currentSsid);
  server.send(200, "text/html", page);
}

// AP一覧スキャン
void handle_wifi_scan() {
  // STA+APモードでもスキャンできるようにしておく
  if (WiFi.getMode() != WIFI_AP_STA && WiFi.getMode() != WIFI_STA) {
    WiFi.mode(WIFI_AP_STA);
  }

  int n = WiFi.scanNetworks();
  String json = "[";

  for (int i = 0; i < n; ++i) {
    if (i > 0) json += ",";
    String ssid = WiFi.SSID(i);
    ssid.replace("\"", "\\\""); // JSON用に " をエスケープ

    json += "{";
    json += "\"ssid\":\"" + ssid + "\"";
    json += ",\"rssi\":" + String(WiFi.RSSI(i));
    // 暗号化なしなら open=true
    bool isOpen = (WiFi.encryptionType(i) == WIFI_AUTH_OPEN);
    json += ",\"open\":" + String(isOpen ? "true" : "false");
    json += "}";
  }
  json += "]";

  WiFi.scanDelete();
  server.send(200, "application/json", json);
}

// SSID / パスワード保存
void handle_wifi_save() {
  if (server.method() != HTTP_POST) {
    server.send(405, "text/plain", "POSTのみ対応しています");
    return;
  }

  String ssid = server.arg("ssid");
  String pass = server.arg("password");

  if (ssid.length() == 0) {
    server.send(400, "text/plain", "SSIDが空です");
    return;
  }

  preferences.begin("wifi", false);
  preferences.putString("ssid", ssid);
  preferences.putString("pass", pass);
  preferences.end();

  Serial.println("[WiFi] 新しい設定を保存しました:");
  Serial.print("  SSID: "); Serial.println(ssid);

  server.send(200, "text/plain",
              "Wi-Fi設定を保存しました。数秒後に再起動します。");

  // 応答を返す時間を少し取ってから再起動
  delay(500);
  ESP.restart();
}

// 接続できなかったときに APモードを立ち上げる
void startWifiConfigPortal() {

  if (g_apPortalRunning) return;  // already running

  // 末尾3バイトを使って簡易なユニークSSIDを作る
  uint32_t chipId = (uint32_t)(ESP.getEfuseMac() & 0xFFFFFF);
  String apSsid = "STACKCHAN-" + String(chipId, HEX);
  apSsid.toUpperCase();

  WiFi.disconnect(true, true);
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP(apSsid.c_str());

  IPAddress apIP = WiFi.softAPIP();

  // DNS: resolve any hostname (including "m5stack") to the AP IP so users can open http://m5stack/
  dnsServer.start(53, "*", apIP);
  // mDNS: provide http://m5stack.local/ as a fallback (some devices prefer .local)
  if (MDNS.begin(WIFI_CONFIG_HOST)) {
    MDNS.addService("http", "tcp", 80);
  } else {
    Serial.println("[mDNS] failed to start");
  }
  g_apPortalRunning = true;

  Serial.println("==== Wi-Fi 設定モード(AP) ====");
  Serial.print("  SSID: "); Serial.println(apSsid);
  Serial.print("  IP  : "); Serial.println(apIP);

  Serial.print("  URL : http://"); Serial.print(WIFI_CONFIG_HOST); Serial.println("/");

  M5.Display.println("WiFi Config AP");
  M5.Display.println(apSsid);
  // M5.Display.println(apIP);

  M5.Display.print("http://");
  M5.Display.print(WIFI_CONFIG_HOST);
  M5.Display.println("/");

}

void handle_piper_plus_voice() {
  /// ファイルを読み込み、クライアントに送信する
  server.send(200, "text/html", PIPER_PLUS_VOICE_HTML);
}

// ★追加: 現在の音声設定を JSON で返す（piper_plus_voice.html のスライダー初期化用）
void handle_piper_plus_voice_get() {
  String json = "{";
  json += "\"voice_volume\":" + String(PiperPlus_voice_volume) + ",";
  json += "\"piper_tts_length_scale\":" + String(PIPER_TTS_LENGTH_SCALE, 2) + ",";
  json += "\"lang_code\":" + apiJsonStr(LANG_CODE) + ",";
  // json += "\"character\":" + apiJsonStr(LANG_CODE.substring(0, 2) + "-" + CHARACTER);
  json += "\"character\":" + apiJsonStr(LANG_CODE.substring(0, 2) + "-" + CHARACTER) + ",";
  json += "\"display_rotation\":" + String(g_display_rotation);
  json += "}";
  server.send(200, "application/json", json);
}

void handle_piper_plus_voice_set() {
  /// POST以外は拒否
  if (server.method() != HTTP_POST) {
    return;
  }

  bool langChanged = false;
  bool rotationChanged = false;

  if (server.hasArg("display_rotation") && server.arg("display_rotation").length() > 0) {
    uint8_t nextRotation = clampDisplayRotation(server.arg("display_rotation").toInt());
    rotationChanged = (nextRotation != g_display_rotation);
    g_display_rotation = nextRotation;
    g_display_rotation_pending = nextRotation;
    g_display_rotation_apply_requested = true;
  }

  if (server.hasArg("voice_volume") && server.arg("voice_volume").length() > 0) {
    PiperPlus_voice_volume = (uint8_t)server.arg("voice_volume").toInt();
  }

  if (server.hasArg("piper_tts_length_scale") && server.arg("piper_tts_length_scale").length() > 0) {
    float ls = server.arg("piper_tts_length_scale").toFloat();
    if (ls < 0.5f) ls = 0.5f;
    if (ls > 2.0f) ls = 2.0f;
    PIPER_TTS_LENGTH_SCALE = ls;
  }

  if (server.hasArg("character") && server.arg("character").length() > 0) {
    String argCharacter = server.arg("character");
    if (argCharacter.length() < 5 || argCharacter.charAt(2) != '-') {
      server.send(400, "text/plain", "Invalid character parameter");
      return;
    }

    String nextLangCode = normalizeLangCode(argCharacter.substring(0, 2));
    String nextCharacter = argCharacter.substring(3, 5);
    langChanged = (nextLangCode != LANG_CODE) || (nextCharacter != CHARACTER);
    LANG_CODE = nextLangCode;
    CHARACTER = nextCharacter;
    setlang_messege();
  }

  Serial.printf("PiperPlus_voice_volume: %u\n", PiperPlus_voice_volume);
  Serial.printf("PIPER_TTS_LENGTH_SCALE: %.2f\n", PIPER_TTS_LENGTH_SCALE);
  Serial.print("LANG_CODE:");  Serial.println(LANG_CODE);
  Serial.print("CHARACTER:");  Serial.println(CHARACTER);
  Serial.printf("DISPLAY_ROTATION: %u\n", g_display_rotation);

  // NVSに保存（キー名は15文字以内）
  preferences.begin("my-app", false);
  preferences.putUChar("voice_volume", PiperPlus_voice_volume);
  preferences.putFloat("piper_ls",     PIPER_TTS_LENGTH_SCALE);
  preferences.putString("lang_code",   LANG_CODE);
  preferences.putString("character",   CHARACTER);
  preferences.putUChar("disp_rot",     g_display_rotation);
  preferences.end();

  // HTTPハンドラ内では Avatar / Display の追加描画を行わない
  // （表示系の内部queue競合による assert 回避）
  String response = "OK";
  if (rotationChanged) {
    response = "OK_ROTATION_PENDING";
  } else if (langChanged) {
    response = "OK_LANG_CHANGED";
  }
  server.send(200, "text/plain", response);
}


void handle_character_voice() {
  /// ファイルを読み込み、クライアントに送信する
  server.send(200, "text/html", CHARACTER_VOICE_HTML);
}
void handle_character_voice_set() {
  /// POST以外は拒否
  if (server.method() != HTTP_POST) {
    return;
  }

  String argCharacter = server.arg("character");
  if (argCharacter.length() < 5 || argCharacter.charAt(2) != '-') {
    server.send(400, "text/plain", "Invalid character parameter");
    return;
  }

  /// language
  String _LANG_CODE = argCharacter.substring(0, 2);
  Serial.print("_LANG_CODE:");  Serial.println(_LANG_CODE);
  LANG_CODE = normalizeLangCode(_LANG_CODE);
  Serial.print("LANG_CODE:");  Serial.println(LANG_CODE);

  /// character
  String character = argCharacter.substring(3, 5);
  CHARACTER = character;
  Serial.print("CHARACTER:"); Serial.println(character);

  // PreferencesへLANG_CODE/CHARACTERを保存（旧nvs_open("setting")保存は廃止）
  preferences.begin("my-app", false);
  preferences.putString("lang_code", LANG_CODE);
  preferences.putString("character", CHARACTER);
  preferences.end();

  setlang_messege();

  // neopixelWrite(LED_PIN, 0, BRIGHTNESS, 0);  // LED:Green
  avatar.setExpression(Expression::Happy);
  avatar.setSpeechText(languageChangedMessage(LANG_CODE));
  server.send(200, "text/plain", String("OK"));
  delay(3000);
  avatar.setExpression(Expression::Neutral);
  avatar.setSpeechText("");
  // neopixelWrite(LED_PIN, 0, 0, 0);          // LED:Off / black
}

void handle_model() {
  /// ファイルを読み込み、クライアントに送信する
  server.send(200, "text/html", MODEL_HTML);
}
void handle_model_get() {
  // 現在の設定をJSONで返す（ページ表示時にフォームへ自動反映）
  String env = "3";  // ollama-LLM
  if (MODEL_VER == "LMStudio-LLM") env = "2";
  else if (MODEL_VER == "llamacpp-LLM") env = "1";  

  String json = "{";
  json += "\"model_ver\":\"" + env + "\",";
  json += "\"llm_server_ip\":\"" + LLM_SERVER_IP + "\",";
  json += "\"llm_model_name\":\"" + LLM_MODEL_NAME + "\",";
  json += "\"openai_api_key\":\"" + OPENAI_API_KEY + "\",";
  json += "\"next_speach_ip\":\"" + NEXT_SPEACH_IP + "\",";
  json += "\"llm_server_port\":" + String(LLM_SERVER_PORT) + ",";
  // ★追加: Whisper/Piper 接続先
  json += "\"whisper_server_ip\":\"" + WHISPER_SERVER_IP + "\",";
  json += "\"whisper_server_port\":" + String(WHISPER_SERVER_PORT) + ",";
  json += "\"piper_tts_ip\":\"" + PIPER_TTS_IP + "\",";
  json += "\"piper_tts_port\":" + String(PIPER_TTS_PORT) + ",";
  json += "\"piper_tts_length_scale\":" + String(PIPER_TTS_LENGTH_SCALE, 2);  // ★追加
  json += "}";
  server.send(200, "application/json", json);
}

void handle_llm_models_get() {
  // /llm_models_get は UI 選択を必ずクエリで渡す前提にする
  if (!server.hasArg("model_ver") && !server.hasArg("llm_server_ip") && !server.hasArg("llm_server_port")) {
    server.send(400, "application/json",
                "{\"error\":\"missing_query_params\",\"used_url\":\"\"}");
    return;
  }
  //「どのURIで呼ばれたか」をログに出す
  Serial.printf("[LLM_LIST] request_uri=%s\n", server.uri().c_str());

  // ---- Allow overriding settings via query params (no need to press 保存 before fetch) ----
  String eff_model_ver = MODEL_VER;        // "ollama-LLM" / "LMStudio-LLM" / "llamacpp-LLM"
  String eff_ip        = LLM_SERVER_IP;    // default: saved value
  uint16_t eff_port    = LLM_SERVER_PORT;  // 0 = unset => use defaults

  // model_ver override: "1"=llama.cpp, "2"=LMStudio, "3"=ollama
  if (server.hasArg("model_ver")) {
    String v = server.arg("model_ver");
    if      (v == "1") eff_model_ver = "llamacpp-LLM";
    else if (v == "2") eff_model_ver = "LMStudio-LLM";
    else if (v == "3") eff_model_ver = "ollama-LLM";
  }

  // ip override
  if (server.hasArg("llm_server_ip") && server.arg("llm_server_ip").length() > 0) {
    eff_ip = server.arg("llm_server_ip");
  }

  // port override (0 allowed => treat as unset)
  if (server.hasArg("llm_server_port")) {
    String p = server.arg("llm_server_port");
    if (p.length() > 0) eff_port = (uint16_t)p.toInt();
  }

  if (eff_ip.length() == 0) {
    server.send(400, "application/json",
                "{\"error\":\"llm_server_ip_empty\",\"used_url\":\"\"}");
    return;
  }

  // ---- Decide port ----
  uint16_t llm_port = eff_port;
  if (llm_port == 0) {
    if      (eff_model_ver == "ollama-LLM")   llm_port = 11434;
    else if (eff_model_ver == "LMStudio-LLM") llm_port = 52626;
    else if (eff_model_ver == "llamacpp-LLM") llm_port = 8080;
    else                                      llm_port = 8080;
  }

  // ---- Build URL ----
  String path = (eff_model_ver == "ollama-LLM") ? "/api/tags" : "/v1/models";
  String url  = "http://" + eff_ip + ":" + String(llm_port) + path;

  Serial.printf("[LLM_LIST] model_ver=%s ip=%s port=%u path=%s\n",
                eff_model_ver.c_str(), eff_ip.c_str(), (unsigned)llm_port, path.c_str());
  Serial.printf("[LLM_LIST] url=%s\n", url.c_str());

  // ---- Fetch ----
  HTTPClient http;
  WiFiClient client;
  http.setTimeout(5000);
  http.setReuse(false);

  auto doGetOnce = [&](String& outBody)->int {
    if (!http.begin(client, url)) return -1000;
    http.addHeader("Connection", "close");
    int code = http.GET();
    if (code > 0) outBody = http.getString();
    http.end();
    return code;
  };

  String body;
  int httpCode = doGetOnce(body);
  if (httpCode <= 0) {
    delay(200);  // ★軽く待って再試行
    httpCode = doGetOnce(body);
  }

  if (httpCode <= 0) {
    server.send(502, "application/json",
                String("{\"error\":\"llm_http_failed\",\"used_url\":\"") + url + "\"}");
    return;
  }

  if (httpCode != 200) {
    // server responded but not OK
    server.send(502, "application/json",
                String("{\"error\":\"llm_http_status\",\"http\":") + httpCode +
                ",\"used_url\":\"" + url + "\"}");
    return;
  }

  // ---- Parse & return models[] (simple string scan to keep it lightweight) ----
  String out = String("{\"used_url\":\"") + url + "\",\"models\":[";
  bool first = true;

  auto addModel = [&](String name) {
    if (name.length() == 0) return;

    // escape minimal JSON
    name.replace("\\", "\\\\");
    name.replace("\"", "\\\"");

    String needle = "\"" + name + "\"";
    if (out.indexOf(needle) >= 0) return;  // dedupe

    if (!first) out += ",";
    out += needle;
    first = false;
  };

  if (eff_model_ver == "ollama-LLM") {
    // /api/tags: {"models":[{"name":"..."}...]}
    int pos = 0;
    while (true) {
      int key = body.indexOf("\"name\"", pos);
      if (key < 0) break;
      int q1 = body.indexOf("\"", key + 6);
      if (q1 < 0) break;
      int q2 = body.indexOf("\"", q1 + 1);
      if (q2 < 0) break;
      addModel(body.substring(q1 + 1, q2));
      pos = q2 + 1;
    }
  } else {
    // /v1/models: {"data":[{"id":"..."}...]}
    int pos = 0;
    while (true) {
      int key = body.indexOf("\"id\"", pos);
      if (key < 0) break;
      int q1 = body.indexOf("\"", key + 4);
      if (q1 < 0) break;
      int q2 = body.indexOf("\"", q1 + 1);
      if (q2 < 0) break;
      addModel(body.substring(q1 + 1, q2));
      pos = q2 + 1;
    }
  }

  out += "]}";
  server.send(200, "application/json", out);
}

void handle_model_set() {
  /// POST以外は拒否
  if (server.method() != HTTP_POST) {
    return;
  }

  // 現在のグローバル値を初期値として保持（送られてきた項目だけ上書き）
  String new_model_ver  = MODEL_VER;
  String new_ip         = LLM_SERVER_IP;
  String new_model_name = LLM_MODEL_NAME;
  String new_openai_key = OPENAI_API_KEY;  
  String new_next_ip    = NEXT_SPEACH_IP;
  uint16_t new_port     = LLM_SERVER_PORT;
  // ★追加
  String   new_whisper_ip   = WHISPER_SERVER_IP;
  uint16_t new_whisper_port = WHISPER_SERVER_PORT;
  String   new_piper_ip     = PIPER_TTS_IP;
  uint16_t new_piper_port   = PIPER_TTS_PORT;
  float    new_piper_ls     = PIPER_TTS_LENGTH_SCALE;  // ★追加: length_scale

  // model_ver（選択必須）

  if (server.hasArg("model_ver")) {
    String v = server.arg("model_ver");
    if (v == "1") new_model_ver = "llamacpp-LLM";
    else if (v == "2") new_model_ver = "LMStudio-LLM";
    else new_model_ver = "ollama-LLM";
  }


  // 各項目：送られてきた＆空でない場合のみ上書き（空文字で既存値が消えるのを防ぐ）
  if (server.hasArg("llm_server_ip") && server.arg("llm_server_ip").length() > 0)
    new_ip = server.arg("llm_server_ip");

  if (server.hasArg("llm_model_name") && server.arg("llm_model_name").length() > 0)
    new_model_name = server.arg("llm_model_name");

  if (server.hasArg("openai_api_key") && server.arg("openai_api_key").length() > 0)
    new_openai_key = server.arg("openai_api_key");   // ★追加

  // if (server.hasArg("next_speach_ip") && server.arg("next_speach_ip").length() > 0)
  //   new_next_ip = server.arg("next_speach_ip");
  if (server.hasArg("next_speach_ip")) {
    String v = server.arg("next_speach_ip");
    v.trim();
    new_next_ip = v;  // 空文字なら「消去したい」という意味でそのまま採用
  }
  
  // Port: 値"0"を送った場合はデフォルトリセット扱い
  if (server.hasArg("llm_server_port")) {
    String pstr = server.arg("llm_server_port");
    if (pstr.length() > 0) new_port = (uint16_t)pstr.toInt();  // "0"=デフォルトリセット
  }

  // ★追加: Whisper
  if (server.hasArg("whisper_server_ip") && server.arg("whisper_server_ip").length() > 0)
    new_whisper_ip = server.arg("whisper_server_ip");
  if (server.hasArg("whisper_server_port")) {
    String pstr = server.arg("whisper_server_port");
    if (pstr.length() > 0) new_whisper_port = (uint16_t)pstr.toInt();
  }

  // ★追加: Piper
  if (server.hasArg("piper_tts_ip") && server.arg("piper_tts_ip").length() > 0)
    new_piper_ip = server.arg("piper_tts_ip");
  if (server.hasArg("piper_tts_port")) {
    String pstr = server.arg("piper_tts_port");
    if (pstr.length() > 0) new_piper_port = (uint16_t)pstr.toInt();
  }
  // ★追加: length_scale（0.5〜2.0 の範囲に clamp）
  if (server.hasArg("piper_tts_length_scale")) {
    String lstr = server.arg("piper_tts_length_scale");
    if (lstr.length() > 0) {
      float ls = lstr.toFloat();
      if (ls < 0.5f) ls = 0.5f;
      if (ls > 2.0f) ls = 2.0f;
      new_piper_ls = ls;
    }
  }

  // グローバル変数に反映
  MODEL_VER       = new_model_ver;
  LLM_SERVER_IP   = new_ip;
  LLM_MODEL_NAME  = new_model_name;
  OPENAI_API_KEY  = new_openai_key;
  NEXT_SPEACH_IP  = new_next_ip;
  LLM_SERVER_PORT = new_port;
  // ★追加
  WHISPER_SERVER_IP   = new_whisper_ip;
  WHISPER_SERVER_PORT = new_whisper_port;
  PIPER_TTS_IP        = new_piper_ip;
  PIPER_TTS_PORT      = new_piper_port;
  PIPER_TTS_LENGTH_SCALE = new_piper_ls;  // ★追加

  Serial.print("MODEL_VER: ");      Serial.println(MODEL_VER);
  Serial.print("LLM_SERVER_IP: ");  Serial.println(LLM_SERVER_IP);
  Serial.print("LLM_MODEL_NAME: "); Serial.println(LLM_MODEL_NAME);
  Serial.print("LLM_SERVER_PORT: ");Serial.println(LLM_SERVER_PORT);
  if (NEXT_SPEACH_IP.length() > 5) {
    Serial.print("NEXT_SPEACH_IP: "); Serial.println(NEXT_SPEACH_IP);
  } else {
    Serial.print("NEXT_SPEACH_IP: "); Serial.println("N/A");
  }

  // NVSに確定値を保存
  preferences.begin("my-app", false);
  preferences.putString("model_ver",       MODEL_VER);
  preferences.putString("llm_server_ip",   LLM_SERVER_IP);
  preferences.putString("llm_model_name",  LLM_MODEL_NAME);
  preferences.putString("openai_key",      OPENAI_API_KEY);
  // preferences.putString("next_speach_ip",  NEXT_SPEACH_IP);
  if (NEXT_SPEACH_IP.length() == 0) {
    preferences.remove("next_speach_ip");
  } else {
    preferences.putString("next_speach_ip", NEXT_SPEACH_IP);
  }  
  preferences.putUShort("llm_server_port", LLM_SERVER_PORT);
  // ★追加（キー名はNVS 15文字制限に合わせて短縮）
  preferences.putString("whisper_ip",    WHISPER_SERVER_IP);    // whisper_server_ip → whisper_ip
  preferences.putUShort("whisper_port",  WHISPER_SERVER_PORT);  // whisper_server_port → whisper_port
  preferences.putString("piper_tts_ip",        PIPER_TTS_IP);
  preferences.putUShort("piper_tts_port",      PIPER_TTS_PORT);
  preferences.putFloat("piper_ls",       PIPER_TTS_LENGTH_SCALE); // piper_tts_length_scale → piper_ls
  preferences.end();

  // neopixelWrite(LED_PIN, 0, BRIGHTNESS, 0);  // LED:Green
  avatar.setExpression(Expression::Happy);
  avatar.setSpeechText("モデルが、変更されました");
  server.send(200, "text/plain", String("OK"));
  delay(3000);
  avatar.setExpression(Expression::Neutral);
  avatar.setSpeechText("");
  // neopixelWrite(LED_PIN, 0, 0, 0);          // LED:Off / black
}

// 顔文字除外処理
String removeEmojis(String input) {
  String result = "";
  int i = 0;
  
  while (i < input.length()) {
    // UTF-8の先頭バイトを取得
    uint8_t byte1 = input[i];
    
    // ASCII文字の場合（0x00-0x7F）
    if (byte1 <= 0x7F) {
      result += input[i];
      i++;
    } 
    // 2バイト文字の場合（0xC0-0xDF）
    else if ((byte1 & 0xE0) == 0xC0) {
      // 一般的な2バイト文字は保持（多くの非ラテン文字）
      if (i + 1 < input.length()) {
        result += input.substring(i, i + 2);
      }
      i += 2;
    } 
    // 3バイト文字の場合（0xE0-0xEF）
    else if ((byte1 & 0xF0) == 0xE0) {
      // VS16(U+FE0F) や ZWJ(U+200D) は絵文字の合成に使われるので削除
      if (i + 2 < input.length()) {
        uint8_t b2 = (uint8_t)input[i + 1];
        uint8_t b3 = (uint8_t)input[i + 2];
        // VS16: EF B8 8F, ZWJ: E2 80 8D
        bool isVS16 = (byte1 == 0xEF && b2 == 0xB8 && b3 == 0x8F);
        bool isZWJ  = (byte1 == 0xE2 && b2 == 0x80 && b3 == 0x8D);
        if (!isVS16 && !isZWJ) {
          result += input.substring(i, i + 3);
        }
      }
      i += 3;
    } 
    // 4バイト文字の場合（0xF0-0xF7）- 多くの絵文字はここに含まれる
    else if ((byte1 & 0xF8) == 0xF0) {
      // 4バイト文字（絵文字を含む）をスキップ
      i += 4;
    } 
    // その他の無効なUTF-8シーケンス
    else {
      i++;
    }
  }

  return result;
}

static String sanitizeForPiper(String s) {
  s.trim();
  s.replace("\r", "");
  s.replace("\n", " ");
  s.replace("\t", " ");

  // thinkingタグ除去（Qwen系対策）
  s.replace("<think>", "");
  s.replace("</think>", "");

  while (s.indexOf("  ") >= 0) s.replace("  ", " ");

  // Piperで500になりやすい記号・絵文字を除去
  s.replace("☀", "");
  s.replace("✨", "");
  s.replace("♪", "");
  s.replace("😊", "");
  s.replace("😀", "");
  s.replace("😂", "");
  s.replace("⭐", "");
  s.replace("🌞", "");
  s.replace("🙂", "");
  s.replace("😉", "");
  s.replace("🥰", "");
  s.replace("❤", "");
  s.replace("❤️", "");
  s.replace("～", "");
  s.replace("〜", "");

  while (s.indexOf("  ") >= 0) s.replace("  ", " ");
  s.trim();

  // 記号だけ・タグだけになった場合を避ける
  if (s.length() == 0 ||
      s == "。" || s == "、" ||
      s == "!" || s == "！" ||
      s == "?" || s == "？") {
    return "";
  }
  return s;
}

// ============================================================
// Piper TTS 文分割・チャンク再生（雑音対策込み）
// ============================================================
static bool isPiperOnlyPunctuation(String s) {
  s = sanitizeForPiper(s);
  s.trim();
  if (s.length() == 0) return true;

  const char* tokens[] = {
    "。", "、", "!", "！", "?", "？", "「", "」", "『", "』",
    "（", "）", "(", ")", "【", "】", "［", "］", ":", "：",
    "・", "…", "ー", "-", " ", "　"
  };
  for (const char* t : tokens) s.replace(t, "");
  s.trim();
  return s.length() == 0;
}

// LLM応答が閉じ括弧なしで終わった場合に補完する
// 例: "伏見稲荷大社（千本鳥居が絶景。" → "伏見稲荷大社（千本鳥居が絶景。）"
static String fixUnbalancedJapaneseParens(String s) {
  // 全角丸括弧のバランス補正（UTF-8: （=0xEF 0xBC 0x88, ）=0xEF 0xBC 0x89）
  int open = 0;
  const uint8_t* p = (const uint8_t*)s.c_str();
  size_t len = s.length();
  for (size_t i = 0; i + 2 < len; i++) {
    if (p[i] == 0xEF && p[i+1] == 0xBC && p[i+2] == 0x88) { open++; i += 2; }
    else if (p[i] == 0xEF && p[i+1] == 0xBC && p[i+2] == 0x89) { if (open > 0) open--; i += 2; }
  }
  while (open-- > 0) s += "）";
  return s;
}

// LLMが末尾に付けることがある "(199)" のような数字括弧チャンクを除外する
// ※全角括弧はUTF-8マルチバイトのため、まずASCII括弧のみ対象とする
static bool isPiperMetaChunk(const String& s) {
  if (s.length() < 3) return false;
  if (s[0] != '(') return false;
  if (s[s.length() - 1] != ')') return false;
  for (int i = 1; i < (int)s.length() - 1; i++) {
    if (s[i] < '0' || s[i] > '9') return false;
  }
  return true;
}

static bool startsWithPiperClosingPunct(const String& s) {
  return s.startsWith("」") || s.startsWith("』") || s.startsWith("）") ||
         s.startsWith(")") || s.startsWith("】") || s.startsWith("］") ||
         s.startsWith("!") || s.startsWith("！") || s.startsWith("?") ||
         s.startsWith("？") || s.startsWith("。") || s.startsWith("、");
}

static size_t leadingUtf8TokenBytes(const String& s) {
  if (s.length() <= 0) return 0;
  const uint8_t c = (uint8_t)s[0];
  if      ((c & 0x80) == 0x00) return 1;
  else if ((c & 0xE0) == 0xC0) return 2;
  else if ((c & 0xF0) == 0xE0) return 3;
  else if ((c & 0xF8) == 0xF0) return 4;
  return 1;
}

static void absorbLeadingClosingPunct(std::deque<String>& chunks) {
  if (chunks.size() < 2) return;
  for (size_t i = 1; i < chunks.size(); ++i) {
    chunks[i].trim();
    while (chunks[i].length() > 0 && startsWithPiperClosingPunct(chunks[i])) {
      size_t n = leadingUtf8TokenBytes(chunks[i]);
      String tok = chunks[i].substring(0, n);
      chunks[i - 1] += tok;
      chunks[i] = chunks[i].substring(n);
      chunks[i].trim();
    }
  }
}

static void mergeTinyPiperChunks(std::deque<String>& chunks, size_t minChars = 6) {
  if (chunks.size() < 2) return;

  // パス1: 開き括弧で始まる短チャンクを後続へ結合
  // 例: 「元気だ！ / 僕も元気で... → 「元気だ！僕も元気で...
  for (size_t i = 0; i + 1 < chunks.size(); ) {
    const String& cur = chunks[i];
    bool startsWithOpen = cur.startsWith("「") || cur.startsWith("『") ||
                          cur.startsWith("（") || cur.startsWith("(");
    String truncated = truncateUtf8Chars(cur, 8);
    bool isShort = (truncated.length() == cur.length());
    if (startsWithOpen && isShort) {
      chunks[i + 1] = cur + chunks[i + 1];
      chunks.erase(chunks.begin() + i);
      // i はそのまま（結合後のチャンクを再評価しない）
    } else {
      i++;
    }
  }

  // パス2: 既存の tiny chunk 結合ロジック
  std::deque<String> merged;
  for (auto chunk : chunks) {
    chunk = sanitizeForPiper(chunk);
    chunk.trim();
    if (chunk.length() == 0) continue;

    bool tiny = truncateUtf8Chars(chunk, minChars).length() == chunk.length();
    bool mergeIntoPrev = false;
    if (!merged.empty()) {
      if (isPiperOnlyPunctuation(chunk)) mergeIntoPrev = true;
      else if (tiny && (startsWithPiperClosingPunct(chunk) || chunk[0] == ' ' || isdigit((unsigned char)chunk[0]))) mergeIntoPrev = true;
    }

    if (mergeIntoPrev) {
      merged.back() += chunk;
    } else {
      merged.push_back(chunk);
    }
  }
  chunks.swap(merged);
}

// 指定バイト位置より前で最後に現れる区切り記号位置を探す
// 戻り値はバイトインデックス（見つからなければ -1）
static int findLastTokenBefore(const String& s, int beforeByte) {
  auto findLastAscii = [&](char ch)->int {
    int found = -1;
    for (int i = 0; i < beforeByte && i < (int)s.length(); i++) {
      if (s[i] == ch) found = i + 1;
    }
    return found;
  };
  auto findLastUtf8 = [&](uint8_t b1, uint8_t b2, uint8_t b3)->int {
    int found = -1;
    for (int i = 0; i + 2 < beforeByte && i + 2 < (int)s.length(); i++) {
      if ((uint8_t)s[i] == b1 && (uint8_t)s[i+1] == b2 && (uint8_t)s[i+2] == b3) {
        found = i + 3;
      }
    }
    return found;
  };

  int found = findLastUtf8(0xEF, 0xBC, 0x9A); // ：
  if (found > 0) return found;
  found = findLastAscii(':');
  if (found > 0) return found;
  found = findLastUtf8(0xEF, 0xBC, 0x89); // ）
  if (found > 0) return found;
  found = findLastAscii(')');
  if (found > 0) return found;
  found = findLastUtf8(0xE3, 0x80, 0x81); // 、
  if (found > 0) return found;
  found = findLastAscii(',');
  if (found > 0) return found;
  found = findLastAscii(' ');
  if (found > 0) return found;
  return -1;
}

// 1文を maxChars 文字以内のチャンクに分割して deque へ追加する
static void pushPiperChunk(std::deque<String>& out, const String& sentence, size_t maxChars) {
  String rest = sanitizeForPiper(sentence);
  rest.trim();
  while (rest.length() > 0) {
    String head = truncateUtf8Chars(rest, maxChars);
    if (head.length() == rest.length()) {
      out.push_back(rest);
      break;
    }

    int cut = findLastTokenBefore(head, (int)head.length());
    if (cut > 0) {
      String left = rest.substring(0, cut);
      left = sanitizeForPiper(left);
      left.trim();
      if (left.length() > 0) out.push_back(left);
      rest = rest.substring(cut);
    } else {
      const int rawLen = head.length();   // sanitize前のバイト長を保存
      String speakable = sanitizeForPiper(head);
      speakable.trim();
      if (speakable.length() > 0) out.push_back(speakable);
      rest = rest.substring(rawLen);      // 元のバイト長で進める
    }
    rest = sanitizeForPiper(rest);
    rest.trim();
  }
}

// 全文を文単位（→必要なら maxChars 文字以内に再分割）した deque を返す
static std::deque<String> splitForPiperTTS(const String& text, size_t maxChars) {
  std::deque<String> result;

  String s = sanitizeForPiper(text);
  if (s.length() == 0) return result;

  String work = s;

  // 中黒リスト対策（固有名詞中の・は壊さず、見出し・文末直後のみ分割）
  work.replace("：・", "：\n・");
  work.replace(":・", ":\n・");
  work.replace("）・", "）\n・");
  work.replace("。・", "。\n・");
  work.replace("！・", "！\n・");
  work.replace("？・", "？\n・");

  // 後置き補足（※）の前で分割（「）※」「。※」のみ。無条件分割は誤爆リスクあり）
  work.replace("）※", "）\n※");
  work.replace("。※", "。\n※");

  // 箇条書き番号の前で改行（文末記号直後のパターンを最優先で処理）
  // 例: "...です。1. XXX" → "...です。\n1. XXX"
  work.replace("。1. ", "。\n1. ");
  work.replace("。2. ", "。\n2. ");
  work.replace("。3. ", "。\n3. ");
  work.replace("。4. ", "。\n4. ");
  work.replace("！1. ", "！\n1. ");
  work.replace("！2. ", "！\n2. ");
  work.replace("！3. ", "！\n3. ");
  work.replace("！4. ", "！\n4. ");
  work.replace("？1. ", "？\n1. ");
  work.replace("？2. ", "？\n2. ");
  work.replace("？3. ", "？\n3. ");
  work.replace("？4. ", "？\n4. ");
  work.replace("）1. ", "）\n1. ");
  work.replace("）2. ", "）\n2. ");
  work.replace("）3. ", "）\n3. ");
  work.replace("）4. ", "）\n4. ");
  // sanitize後でも残る "1. " 形式を改行（残余パターン）
  work.replace("1. ", "\n1. ");
  work.replace("2. ", "\n2. ");
  work.replace("3. ", "\n3. ");
  work.replace("4. ", "\n4. ");

  // 列挙の手前で改行（日本語LLMがよく返す半角リスト用）
  work.replace(" 1.", "\n1.");
  work.replace(" 2.", "\n2.");
  work.replace(" 3.", "\n3.");
  work.replace(" 4.", "\n4.");
  work.replace(" 1)", "\n1)");
  work.replace(" 2)", "\n2)");
  work.replace(" 3)", "\n3)");
  work.replace(" 4)", "\n4)");

  // 文末で改行。コロンは pushPiperChunk 側だけで扱う
  work.replace("。", "。\n");
  work.replace("！", "！\n");
  work.replace("？", "？\n");
  work.replace("!", "!\n");
  work.replace("?", "?\n");
  // 英語ピリオドの改行化（番号付きリスト "1." "2." 等を壊さないよう、
  // ピリオド直前が数字の場合はスキップする）
  {
    String tmp;
    tmp.reserve(work.length() + 32);
    for (int i = 0; i < (int)work.length(); i++) {
      tmp += work[i];
      if (work[i] == '.') {
        // 直前が数字なら番号付きリストの可能性が高いのでスキップ
        bool prevIsDigit = (i > 0 && work[i - 1] >= '0' && work[i - 1] <= '9');
        if (!prevIsDigit) {
          tmp += '\n';
        }
      }
    }
    work = tmp;
  }
  while (work.indexOf("\n\n") >= 0) work.replace("\n\n", "\n");

  int start = 0;
  while (start < (int)work.length()) {
    int nl = work.indexOf('\n', start);
    String line;
    if (nl < 0) {
      line = work.substring(start);
      start = work.length();
    } else {
      line = work.substring(start, nl);
      start = nl + 1;
    }
    line = sanitizeForPiper(line);
    line.trim();
    if (line.length() == 0) continue;
    if (isPiperOnlyPunctuation(line)) continue;
    line = fixUnbalancedJapaneseParens(line);  // 閉じ括弧欠落を補完

    String truncated = truncateUtf8Chars(line, maxChars);
    if (truncated.length() == line.length()) {
      result.push_back(line);
    } else {
      pushPiperChunk(result, line, maxChars);
    }
  }

  absorbLeadingClosingPunct(result);
  mergeTinyPiperChunks(result, 6);

  std::deque<String> filtered;
  for (auto chunk : result) {
    chunk = sanitizeForPiper(chunk);
    chunk.trim();
    if (chunk.length() == 0) continue;
    if (isPiperOnlyPunctuation(chunk)) continue;
    filtered.push_back(chunk);
  }
  return filtered;
}

// ============================================================
// PR1/PR2-C: TTS Task (Core 0 で再生処理を実行)
// Avatar は直接描画せず、loop() 側へ更新要求だけを出す
// ============================================================
static void requestAvatarUpdate(const String& text, uint8_t expr) {
  if (g_avatar_text_mutex && xSemaphoreTake(g_avatar_text_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    g_avatar_text_pending = text;
    xSemaphoreGive(g_avatar_text_mutex);
  }
  g_avatar_expr_pending.store(expr);
  g_avatar_update_requested.store(true);
}

static bool enqueueTtsJob(const String& text, uint8_t expr, bool allow_cancel, bool is_final) {
  if (!g_ttsQueue) return false;

  TtsJob job;
  job.text         = nullptr;
  job.allow_cancel = allow_cancel;
  job.is_final     = is_final;
  job.expression   = expr;

  if (text.length() > 0) {
    size_t n = text.length() + 1;
    job.text = (char*)malloc(n);
    if (!job.text) {
      Serial.println("[TTS] enqueue: malloc failed");
      return false;
    }
    memcpy(job.text, text.c_str(), n);
  }

  g_tts_pending_jobs.fetch_add(1);

  if (xQueueSend(g_ttsQueue, &job, 0) != pdTRUE) {
    Serial.println("[TTS] queue full, dropping job");
    g_tts_pending_jobs.fetch_sub(1);
    if (job.text) free(job.text);
    return false;
  }
  return true;
}

static void ttsTaskFunc(void* /*arg*/) {
  for (;;) {
    TtsJob job;
    if (xQueueReceive(g_ttsQueue, &job, portMAX_DELAY) != pdTRUE) continue;

    if (g_tts_cancel_requested.load()) {
      Serial.println("[TTS] drop job due to cancel");
      if (job.text) free(job.text);
      g_tts_pending_jobs.fetch_sub(1);
      continue;
    }

    if (job.text && strlen(job.text) > 2) {
      requestAvatarUpdate(String(job.text), job.expression);
      Serial.printf("[TTS] playing: %s\n", job.text);
      bool ok = speak_piper_http(String(job.text), job.allow_cancel);
      if (!ok && g_tts_cancel_requested.load()) {
        Serial.println("[TTS] current job cancelled");
      }
    }

    // speak_piper_http() 側では false に戻していないため、ここで戻す
    g_piper_tts_playing.store(false);

    if (job.is_final) {
      requestAvatarUpdate("", (uint8_t)Expression::Neutral);
    }

    if (job.text) free(job.text);
    g_tts_pending_jobs.fetch_sub(1);

    Serial.printf("[TTS] job done, pending=%d\n", g_tts_pending_jobs.load());
  }
}

static bool speak_piper_http_chunked(const String& text, bool allow_cancel) {
  auto chunks = splitForPiperTTS(text);
  Serial.printf("[PIPER] chunk count = %d\n", (int)chunks.size());
  if (chunks.empty()) return true;

  int idx = 0;
  int total = (int)chunks.size();
  for (auto& part : chunks) {
    part = sanitizeForPiper(part);
    part.trim();
    if (part.length() == 0 || isPiperOnlyPunctuation(part) || isPiperMetaChunk(part)) continue;

    M5.update();
    if (allow_cancel && M5.BtnA.wasHold()) {
      Serial.printf("[PIPER] chunk %d cancelled before start\n", idx + 1);
      avatar.setExpression(Expression::Neutral);
      avatar.setSpeechText("");
      return false;
    }

    Serial.printf("[PIPER] chunk %d/%d: %s\n", idx + 1, total, part.c_str());
    avatar.setExpression(Expression::Happy);
    avatar.setSpeechText(part.c_str());

    uint32_t t0 = millis();
    bool ok = speak_piper_http(part, allow_cancel);
    uint32_t elapsed = millis() - t0;

    Serial.printf("[PIPER] chunk %d/%d done ok=%d elapsed=%lu ms\n",
                  idx + 1, total, ok ? 1 : 0, (unsigned long)elapsed);

    if (!ok) {
      Serial.printf("[PIPER] chunk %d failed/cancelled\n", idx + 1);
      avatar.setExpression(Expression::Neutral);
      avatar.setSpeechText("");
      return false;
    }
    idx++;
    delay(40);
  }
  return true;
}


// ===== Base64 エンコード/デコード（UTF-8 文字列用）=====
static String relayBase64Encode(const String& src) {
  if (src.length() == 0) return "";
  size_t outLen = 0;
  size_t bufSize = 4 * ((src.length() + 2) / 3) + 1;
  std::unique_ptr<unsigned char[]> buf(new unsigned char[bufSize]);
  int rc = mbedtls_base64_encode(
    buf.get(), bufSize, &outLen,
    (const unsigned char*)src.c_str(), src.length()
  );
  if (rc != 0) {
    Serial.printf("[relay] base64_encode failed: %d\n", rc);
    return "";
  }
  buf[outLen] = '\0';
  return String((char*)buf.get());
}

static String relayBase64Decode(const String& src) {
  if (src.length() == 0) return "";
  size_t outLen = 0;
  size_t bufSize = src.length();
  std::unique_ptr<unsigned char[]> buf(new unsigned char[bufSize + 1]);
  int rc = mbedtls_base64_decode(
    buf.get(), bufSize, &outLen,
    (const unsigned char*)src.c_str(), src.length()
  );
  if (rc != 0) {
    Serial.printf("[relay] base64_decode failed: %d\n", rc);
    return "";
  }
  buf[outLen] = '\0';
  return String((char*)buf.get());
}

// ===== 転送除外判定（エラー文字列を弾く）=====
static bool isNonRelayableAnswer(const String& answer) {
  if (answer.length() == 0) return true;
  if (answer.startsWith("接続エラー")) return true;
  if (answer.startsWith("通信エラー")) return true;
  if (answer.startsWith("応答を受信できませんでした")) return true;
  if (answer.startsWith("応答の解析に失敗しました")) return true;
  if (answer.startsWith("HTTPエラー")) return true;
  if (answer.startsWith("画像対応エラー")) return true;
  if (answer.indexOf("APIキーを設定してください") >= 0) return true;
  if (answer == "Error" || answer == "Error1") return true;
  return false;
}

// ===== 次個体向け完成プロンプトの組み立て =====
static String buildRelayPromptForNext(const String& originQuestion,
                                      const String& previousAnswer,
                                      const String& myModelName) {
  String p;
  p.reserve(originQuestion.length() + previousAnswer.length() + 256);
  p += "これは複数LLMで同じ質問に順番に答えるリレーです。\n";
  p += "【元の質問】\n";
  p += originQuestion;
  p += "\n\n";
  if (previousAnswer.length() > 0) {
    p += "【直前の個体(";
    p += (myModelName.length() > 0 ? myModelName : String("unknown"));
    p += ")の回答】\n";
    p += previousAnswer;
    p += "\n\n";
  }
  p += "【あなたへの依頼】\n";
  p += "上記の元の質問に対して、あなた自身の視点から100文字程度で自然に日本語で答えてください。";
  p += "直前の回答には必ずしも同意や反論をする必要はありません。参考情報として扱い、あなたの答えを出してください。";
  return p;
}

void runText_chatTominimal(String IP, String textData) {
  String myIp = WiFi.localIP().toString();
  if (IP == myIp) {
    Serial.println("[relay] skip: target is self");
    return;
  }
  if (g_relay_origin.length() > 0 && IP == g_relay_origin) {
    Serial.println("[relay] skip: target is relay_origin");
    return;
  }

  int nextHop = g_relay_hop + 1;
  if (nextHop > RELAY_MAX_HOPS) {
    Serial.printf("[relay] skip: hop limit exceeded (next=%d)\n", nextHop);
    return;
  }

  String hdrOriginQ = relayBase64Encode(g_relay_origin_question);
  String hdrPrevA   = relayBase64Encode(g_relay_previous_answer);
  String hdrOrigin  = (g_relay_origin.length() > 0) ? g_relay_origin : myIp;

  auto doPost = [&]() -> int {
    WiFiClient client;
    HTTPClient http;
    http.setTimeout(RELAY_HTTP_TIMEOUT_MS);
    http.setReuse(false);
    http.begin(client, "http://" + IP + "/text_chat_set");
    http.addHeader("Content-Type", "text/plain;charset=UTF-8");
    http.addHeader("X-Origin-Question", hdrOriginQ);
    http.addHeader("X-Previous-Answer", hdrPrevA);
    http.addHeader("X-Relay-Hop", String(nextHop));
    http.addHeader("X-Relay-Origin", hdrOrigin);
    int code = http.POST(textData);
    http.end();
    return code;
  };

  int httpResponseCode = doPost();
  if (httpResponseCode == 200) {
    Serial.printf("[relay] 送信成功 hop=%d to=%s\n", nextHop, IP.c_str());
    return;
  }

  bool shouldRetry = (httpResponseCode == 409) || (httpResponseCode < 0);
  if (!shouldRetry) {
    Serial.printf("[relay] 送信失敗 (リトライ対象外): %d\n", httpResponseCode);
    return;
  }

  Serial.printf("[relay] 1回目失敗 (%d), %dms 待機後リトライ\n",
                httpResponseCode, RELAY_RETRY_WAIT_MS);
  delay(RELAY_RETRY_WAIT_MS);
  httpResponseCode = doPost();
  if (httpResponseCode == 200) {
    Serial.printf("[relay] リトライ成功 hop=%d to=%s\n", nextHop, IP.c_str());
  } else {
    Serial.printf("[relay] リトライ失敗: %d（諦め）\n", httpResponseCode);
  }
}

bool init_chat_doc(const char *data)
{
  DeserializationError error = deserializeJson(chat_doc, data);
  if (error) {
    Serial.println("DeserializationError");
    return false;
  }
  String json_str; //= JSON.stringify(chat_doc);
  serializeJsonPretty(chat_doc, json_str);  // 文字列をシリアルポートに出力する
//  Serial.println(json_str);
    return true;
}

String https_post_json(const char* url, const char* json_string, const char* root_ca) {
  String payload = "";
  WiFiClientSecure *client = new WiFiClientSecure;
  if(client) {
    client -> setCACert(root_ca);
    {
      // Add a scoping block for HTTPClient https to make sure it is destroyed before WiFiClientSecure *client is 
      HTTPClient https;
      https.setTimeout( 65000 ); 

      //debug
      Serial.print("json_string:"); Serial.println(json_string);
  
      Serial.print("[HTTPS] begin...\n");
      if (https.begin(*client, url)) {  // HTTPS
        Serial.print("[HTTPS] POST...\n");
        // start connection and send HTTP header
        https.addHeader("Content-Type", "application/json");
        https.addHeader("Authorization", String("Bearer ") + OPENAI_API_KEY);
        int httpCode = https.POST((uint8_t *)json_string, strlen(json_string));
  
        // httpCode will be negative on error
        if (httpCode > 0) {
          // HTTP header has been send and Server response header has been handled
          Serial.printf("[HTTPS] POST... code: %d\n", httpCode);
  
          // file found at server
          if (httpCode == HTTP_CODE_OK || httpCode == HTTP_CODE_MOVED_PERMANENTLY) {
            payload = https.getString();
            Serial.println("//////////////1");
            Serial.println(payload);
            Serial.println("//////////////1");
          }
        } else {
          Serial.printf("[HTTPS] POST... failed, error: %s\n", https.errorToString(httpCode).c_str());
        }  
        https.end();
      } else {
        Serial.printf("[HTTPS] Unable to connect\n");
      }
      // End extra scoping block
    }  
    delete client;
  } else {
    Serial.println("Unable to create client");
  }
  return payload;
}

String http_post_json_ollama(String url, const char* json_string) {
  (void)json_string;  // 現状未使用（互換のため残す）

  String final_response = "";

  // ---- 質問文 ----
  const String q_rule = replyInstruction100();
  const String q_content = Speech_Recognition;  // start_talking() でセット済み
  const String user_text = q_rule + q_content;

  // ---- JSONエスケープ（グローバル関数を利用）----
  const String escaped_text = jsonEscapeForJsonString(user_text);

  // ★重要: AI.Local は stream:true でも逐次チャンクを流さず単発JSONになりやすいので stream:false 固定
  // options: num_predict で暴走を抑え、repeat_penalty で同文連呼を抑制
  String json =
    String("{\"model\":\"") + LLM_MODEL_NAME + "\","
    "\"stream\":false,"
    "\"messages\":[{\"role\":\"user\",\"content\":\"" + escaped_text + "\"}],"
    "\"options\":{\"num_predict\":120,\"repeat_penalty\":1.15}}";

  Serial.print("url: ");  Serial.println(url);
  Serial.print("json: "); Serial.println(json);

  WiFiClient client;
  HTTPClient http;
  http.setTimeout(65000);
  http.setReuse(false);  // keep-alive再利用を避ける（ハング回避）

  Serial.println("[Local_LLM-ollama] begin...");
  if (!http.begin(client, url)) {
    Serial.println("Failed HTTPClient begin!");
    final_response = "接続エラー";
    goto END_OLLAMA;
  }

  http.addHeader("Content-Type", "application/json");
  http.addHeader("Connection", "close");  // 念のため明示

  {
    int httpCode = http.POST(json);
    if (httpCode > 0) {
      Serial.printf("[Local_LLM-ollama] POST... code: %d\n", httpCode);

      if (httpCode == HTTP_CODE_OK || httpCode == HTTP_CODE_MOVED_PERMANENTLY) {
        // ★単発JSONを一括受信
        String body = http.getString();
        Serial.println("//// OLLAMA BODY ////");
        Serial.println(body);
        Serial.println("/////////////////////");

        DynamicJsonDocument doc(8192);
        DeserializationError err = deserializeJson(doc, body);

        if (err) {
          Serial.printf("JSON解析エラー(ollama body): %s\n", err.c_str());
          final_response = "応答の解析に失敗しました";
        } else {
          const char* c = doc["message"]["content"] | "";
          final_response = removeEmojis(String(c));
          final_response.trim();

          // Piper-plus/TTSの安全上限（保険）
          if (final_response.length() > 300) {
            final_response = final_response.substring(0, 300) + "…";
          }
        }
      } else {
        final_response = "HTTPエラー: " + String(httpCode);
      }
    } else {
      Serial.printf("[Local_LLM-ollama] POST... failed, error: %s\n",
                    http.errorToString(httpCode).c_str());
      final_response = "通信エラー";
    }
  }

  http.end();

END_OLLAMA:
  if (final_response.length() == 0) {
    final_response = "応答を受信できませんでした";
  }

  Serial.printf("最終応答(ollama): %s\n", final_response.c_str());

  // ---- 会話履歴用（exec_chatGPT() が参照）----
  g_last_llm_answer = final_response;

  // ---- ここで喋る（start_talking() 側の二重発話を避ける設計を維持）----
  {
    if (final_response.length() > 0) {
      final_response = sanitizeForPiper(final_response);
      if (final_response.length() > 0) {
        avatar.setExpression(Expression::Happy);
        avatar.setSpeechText(final_response.c_str());
        speak_piper_http_chunked(final_response, true);
      }
    }    
  }

  // ★重要: chatGpt() 側で二重に喋らせないためのダミーJSON（contentは空）
  return "{\"model\":\"" + LLM_MODEL_NAME + "\",\"message\":{\"role\":\"assistant\",\"content\":\"\"}}";
}


String http_post_json_openai_stream(String url, const char* json_string) {
  String payload = "";         // sentence buffer（現在蓄積中の文）
  String final_response = "";  // full text (for history)
  String json = "";

  bool tts_cancel = false;

  // Use caller-provided JSON (keep parity across servers)
  if (json_string && strlen(json_string) > 0) {
    json = String(json_string);
  } else {
    json = String("{\"model\": \"") + LLM_MODEL_NAME + "\", \"stream\": true, \"messages\":[{\"role\":\"user\",\"content\":\"\"}]}";
  }

  // Normalize to stream:true for OpenAI-compatible SSE
  json.replace("\"stream\":false", "\"stream\":true");
  json.replace("\"stream\": false", "\"stream\": true");

  Serial.print("url: ");  Serial.println(url);
  Serial.print("json: "); Serial.println(json);

  WiFiClient client;
  HTTPClient http;
  http.setTimeout(65000);
  http.setReuse(false);

  Serial.print("[Local_LLM-FLM] begin...");
  if (!http.begin(client, url)) {
    Serial.println("Failed HTTPClient begin!");
    return "接続エラー";
  }

  http.addHeader("Content-Type", "application/json");
  if (MODEL_VER == "llamacpp-LLM") {
    http.addHeader("Authorization", String("Bearer ") + OPENAI_API_KEY);
  }

  uint32_t t_request = millis();

  int httpCode = http.POST(json);

  if (httpCode > 0) {
    Serial.printf("[Local_LLM-FLM] POST... code: %d", httpCode);

    if (httpCode == HTTP_CODE_OK || httpCode == HTTP_CODE_MOVED_PERMANENTLY) {
      WiFiClient* stream = http.getStreamPtr();
      stream->setTimeout(5000);  // readStringUntilの無限ブロック防止
      DynamicJsonDocument doc(2000);

      bool first_token_captured = false;
      uint32_t t_first_token = 0;      

      // ===== Phase 1: SSE受信を完全に終えてからTTS再生 =====
      // TTS再生（数秒）の同期ブロック中にSSE受信が止まり、
      // TCP受信バッファ（数KB）が溢れて後続トークンが失われる問題の根本対策。
      // LLM応答は通常1-3秒で完了するため、全文受信後の一括再生でも
      // 体感遅延はわずか（最初の発話開始が1-3秒遅れる程度）。

      bool tts_started = false;
      int sentence_count = 0;

      ensureSpeakerActive();
      M5.Speaker.setVolume(PiperPlus_voice_volume);

      // SSEアイドルタイムアウト
      const uint32_t SSE_IDLE_TIMEOUT_MS = 8000;
      uint32_t lastSseActivityMs = millis();

      // 文キュー: SSE受信完了後にまとめて再生する
      std::deque<String> sentenceQueue;

      while (http.connected() || stream->available()) {
        if (!stream->available()) {
          if (!http.connected()) break;

          if (millis() - lastSseActivityMs > SSE_IDLE_TIMEOUT_MS) {
            Serial.printf("[SSE] idle timeout (%lu ms), breaking\n",
                          (unsigned long)(millis() - lastSseActivityMs));
            break;
          }

          delay(10);
          continue;
        }

        lastSseActivityMs = millis();

        String line = stream->readStringUntil('\n');

        
        line.trim();
        if (line.length() == 0) continue;

        // chunked transfer encoding のサイズ行をスキップ
        {
          bool isChunkSize = (line.length() > 0 && line.length() <= 8);
          if (isChunkSize) {
            for (unsigned int ci = 0; ci < line.length(); ci++) {
              char c = line.charAt(ci);
              if (!((c >= '0' && c <= '9') || (c >= 'a' && c <= 'f') || (c >= 'A' && c <= 'F'))) {
                isChunkSize = false;
                break;
              }
            }
          }
          if (isChunkSize) {
            if (line == "0") {
              Serial.println("[SSE] chunked terminal '0' -> break");
              break;
            }
            continue;
          }
        }

        // 行頭にchunkedサイズが付着した場合の対策
        if (!line.startsWith("data:")) {
          int dataPos = line.indexOf("data:");
          if (dataPos > 0) {
            line = line.substring(dataPos);
          } else {
            continue;
          }
        }

        Serial.print("data: "); Serial.println(line);

        String dataLine = line.substring(5);
        dataLine.trim();
        if (dataLine.startsWith(" ")) dataLine.remove(0, 1);
        dataLine.trim();

        if (dataLine == "[DONE]") {
          Serial.println("FastFlowLM stream DONE marker");
          break;
        }

        DeserializationError error = deserializeJson(doc, dataLine);
        if (error) {
          Serial.printf("JSON解析エラー(FLM): %s\n", error.c_str());
          doc.clear();
          continue;
        }

        const char* chunk_content = doc["choices"][0]["delta"]["content"];
        const char* finish_reason  = doc["choices"][0]["finish_reason"];

        if (chunk_content && strlen(chunk_content) > 0) {
          lastSseActivityMs = millis();

          String cleanString = removeEmojis(String(chunk_content));
          cleanString.replace("\r", "");
          cleanString.replace("\n", "");
          cleanString.replace("<think>", "");
          cleanString.replace("</think>", "");

          // ★ 最初の本文トークン到着を記録
          if (!first_token_captured && cleanString.length() > 0) {
            t_first_token = millis();
            first_token_captured = true;
            Serial.printf("[PERF] first_token_ms=%lu\n",
                          (unsigned long)(t_first_token - t_request));
          }          

          payload        += cleanString;
          final_response += cleanString;

          Serial.printf("受信データ(FLM): %s", cleanString.c_str());

          // 文境界検出
          int lastBound = -1;
          for (int i = payload.length() - 1; i >= 0; i--) {
            if (i + 2 < (int)payload.length()) {
              uint8_t b0 = (uint8_t)payload[i];
              uint8_t b1 = (uint8_t)payload[i+1];
              uint8_t b2 = (uint8_t)payload[i+2];
              if ((b0==0xE3 && b1==0x80 && b2==0x82) ||
                  (b0==0xEF && b1==0xBC && b2==0x81) ||
                  (b0==0xEF && b1==0xBC && b2==0x9F)) {
                lastBound = i + 3;
                break;
              }
            }
            if (payload[i] == '.' || payload[i] == '?' || payload[i] == '!') {
              lastBound = i + 1;
              break;
            }
          }

          if (lastBound > 0) {
            String sentence = sanitizeForPiper(payload.substring(0, lastBound));
            sentence.trim();
            payload = payload.substring(lastBound);

            if (sentence.length() > 2) {
              // 全ての文をキューに蓄積（SSE完了後にまとめて再生）
              sentenceQueue.push_back(sentence);
              Serial.printf("[StreamTTS] queued: %s\n", sentence.c_str());
            }
          }
        }

        if (finish_reason && strcmp(finish_reason, "stop") == 0) {
          Serial.println("FastFlowLM finish_reason == stop");
          doc.clear();
          break;
        }

        doc.clear();
      }

      // ★観測ログ: SSEループ脱出理由の記録
      Serial.printf("[SSE] loop exited. queued=%d, final_response_len=%d, payload_remaining=%d\n",
                    (int)sentenceQueue.size(), (int)final_response.length(), (int)payload.length());

      if (!first_token_captured) {
        Serial.println("[PERF] first_token_ms=NOT_CAPTURED");
      }
      Serial.printf("[PERF] total_ms=%lu\n",
                    (unsigned long)(millis() - t_request));

      // ===== Phase 2 (PR2-C): sentenceQueue + tail をまとめて enqueue、最後だけ is_final=true =====
      g_tts_cancel_requested.store(false);

      String tail;
      bool has_tail = false;
      if (!tts_cancel && payload.length() > 0) {
        tail = sanitizeForPiper(payload);
        tail.trim();
        has_tail = (tail.length() > 2);
      } else {
        Serial.printf("[StreamTTS] no tail (cancel=%d, payload_len=%d)", (int)tts_cancel, (int)payload.length());
      }

      int total = sentenceQueue.size() + (has_tail ? 1 : 0);
      int enqueued = 0;
      int idx = 0;

      for (const String& s : sentenceQueue) {
        idx++;
        bool is_last = (!has_tail && idx == total);
        if (enqueueTtsJob(s, (uint8_t)Expression::Happy, true, is_last)) {
          enqueued++;
        }
      }

      if (has_tail) {
        if (enqueueTtsJob(tail, (uint8_t)Expression::Happy, true, true)) {
          enqueued++;
          Serial.printf("[StreamTTS] tail enqueued: %s", tail.c_str());
        }
      } else if (!tts_cancel && payload.length() > 0) {
        Serial.println("[StreamTTS] tail skipped (too short or empty)");
      }

      if (enqueued == 0) {
        if (enqueueTtsJob("", (uint8_t)Expression::Neutral, false, true)) {
          enqueued++;
          Serial.println("[StreamTTS] empty final marker enqueued");
        }
      }

      Serial.printf("[StreamTTS] enqueued %d jobs, waiting for completion", enqueued);

      // ===== バッチ完了待ち（カウンタ方式、ポーリング形式）=====
      const uint32_t BATCH_TIMEOUT_MS = 70000;
      uint32_t wait_start = millis();
      while (g_tts_pending_jobs.load() > 0) {
        M5.update();
        applyPendingAvatarUpdate();
        if (M5.BtnA.wasHold()) {
          Serial.println("[StreamTTS] cancelled by button");
          g_tts_cancel_requested.store(true);
          M5.Speaker.stop();
          tts_cancel = true;
          requestAvatarUpdate("", (uint8_t)Expression::Neutral);
        }
        if (millis() - wait_start > BATCH_TIMEOUT_MS) {
          Serial.println("[StreamTTS] WARNING: batch wait timed out");
          g_tts_cancel_requested.store(true);
          M5.Speaker.stop();
          requestAvatarUpdate("", (uint8_t)Expression::Neutral);
          break;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
      }

      Serial.printf("[StreamTTS] batch completed, pending=%d", g_tts_pending_jobs.load());

      // drain remaining bytes
      {
        unsigned long t0 = millis();
        while ((http.connected() || stream->available()) && (millis() - t0) < 2000) {
          while (stream->available()) { stream->read(); }
          delay(1);
        }
      }

      // クリーンアップ：Avatar は ttsTask → loop() 経由で反映済み
      applyPendingAvatarUpdate();
      Serial.println("[SSE] cleanup: done");
    } else {
      // HTTP 200/301 以外（400, 500 等）
      Serial.printf("[OpenAI-stream] HTTP error: %d\n", httpCode);

      // VLM 非対応サーバに画像を送った場合の典型的エラー（500 / 400）
      if (WEB_HAS_IMAGE && (httpCode == 500 || httpCode == 400)) {
        Serial.println("[Vision] サーバがVLM(画像)に対応していない可能性があります。");
        Serial.println("[Vision] llama.cpp の場合: --mmproj オプション付きで起動してください。");
        final_response = "画像対応エラー。サーバがVLMモードで起動されていない可能性があります";
      } else {
        final_response = "HTTPエラー: " + String(httpCode);
      }
    }
  } else {
    Serial.printf("[Local_LLM-FLM] POST... failed, error: %s",
                  http.errorToString(httpCode).c_str());
    final_response = "通信エラー";
  }

  http.end();

  if (final_response.length() == 0) {
    final_response = "応答を受信できませんでした";
  }

  Serial.printf("最終応答(FLM): [%d chars] %s\n", (int)final_response.length(), final_response.c_str());

  // Store for history (thinkタグは会話履歴にも残さない)
  final_response.replace("<think>", "");
  final_response.replace("</think>", "");
  final_response.trim();
  g_last_llm_answer = final_response;

  // dummy JSON to prevent double speaking
  // ★ content は空にする。chatGpt() が非空 response を返すと
  //    start_talking() Step 6 で二重発話になるため。
  //    会話履歴は g_last_llm_answer 経由で exec_chatGPT() が保存する。
  String dummy =
    "{\"id\":\"flm-stream\",\"object\":\"chat.completion\","
    "\"model\":\"" + LLM_MODEL_NAME + "\","
    "\"choices\":[{\"index\":0,"
      "\"message\":{\"role\":\"assistant\",\"content\":\"\"}"
    "}]}";

  return dummy;
}



String chatGpt(String json_string) {
  String response = "";
  Serial.print("chatGpt = "); Serial.println(json_string);
  avatar.setExpression(Expression::Doubt);
  avatar.setSpeechText(message_tinking.c_str());

  /// LLMの接続先切り替え
  String ret = "";
  // ポート決定：getDefaultLlmPort() に統一
  uint16_t llm_port = getDefaultLlmPort();

  if (MODEL_VER == "ollama-LLM") {
    Serial.println("MODEL_VER1: ollama-LLM");
    String LLM_Server = "http://" + LLM_SERVER_IP + ":" + String(llm_port) + "/api/chat";
    ret = http_post_json_ollama(LLM_Server, json_string.c_str());

  } else if (MODEL_VER == "LMStudio-LLM") {
    Serial.println("MODEL_VER1: LMStudio/FastFlowLM-LLM");
    // ★ ポートは変数 llm_port を使用（未設定時デフォルト 52626）
    String LLM_Server = "http://" + LLM_SERVER_IP + ":" + String(llm_port) + "/v1/chat/completions";
    ret = http_post_json_openai_stream(LLM_Server, json_string.c_str());

  } else if (MODEL_VER == "llamacpp-LLM") {
    Serial.println("MODEL_VER1: llama.cpp-LLM");
    String LLM_Server = "http://" + LLM_SERVER_IP + ":" + String(llm_port) + "/v1/chat/completions";
    ret = http_post_json_openai_stream(LLM_Server, json_string.c_str());

  } else {  // ChatGPT
    Serial.println("MODEL_VER1: ChatGPT");
    // ret = https_post_json("https://api.openai.com/v1/chat/completions", json_string.c_str(), root_ca_openai);
  }

  // PR2-C.1:
  // OpenAI互換ストリーミング経路では、発話中テキストの表示/クリアは
  // ttsTask -> pending要求 -> applyPendingAvatarUpdate() 側に委譲する。
  // ここで即クリアすると、会話中に画面表示される前に消えてしまう。
  if (!(MODEL_VER == "LMStudio-LLM" || MODEL_VER == "llamacpp-LLM")) {
    avatar.setExpression(Expression::Neutral);
    avatar.setSpeechText("");
  }

  Serial.print("ret: "); Serial.println(ret); //ここで2回表示している。

  if(ret != ""){
    DynamicJsonDocument doc(2000);  // Adjust for Atom Echo
    DeserializationError error = deserializeJson(doc, ret.c_str());
    if (error) {
      Serial.print(F("deserializeJson() failed: "));
      Serial.println(error.f_str());
      avatar.setExpression(Expression::Sad);
      // avatar.setSpeechText("Error");
      // response = "Error";
      avatar.setSpeechText("Error1");
      response = "Error1";

      // delay(1000);
      delay(700); // Adjust for Atom Echo
      avatar.setSpeechText("");
      avatar.setExpression(Expression::Neutral);
    }else{

      if (MODEL_VER == "ollama-LLM") {
        Serial.println("MODEL_VER2: ollama-LLM");
        const char* data = doc["message"]["content"];
        Serial.println(data);
        response = String(data);

      } else if (MODEL_VER == "LMStudio-LLM" || MODEL_VER == "llamacpp-LLM") {
        Serial.println("MODEL_VER2: OpenAI-compatible (LMStudio/FastFlowLM/llama.cpp)");
        JsonObject choices_0 = doc["choices"][0];
        const char* choices_0_message_content = choices_0["message"]["content"];
        response = String(choices_0_message_content);

      } else {  // ChatGPT
        Serial.println("MODEL_VER2: ChatGPT");
        const char* data = doc["choices"][0]["message"]["content"];
        response = String(data);
      }      

      std::replace(response.begin(),response.end(),'\n',' ');
    }
  } else {
    avatar.setExpression(Expression::Sad);
    avatar.setSpeechText(message_dont_understand.c_str());
    // avatar.setSpeechText("わかりません");
    response = message_dont_understand.c_str();
    delay(700); // Adjust for Atom Echo
    avatar.setSpeechText("");
    avatar.setExpression(Expression::Neutral);
  }
  return response;
}

String exec_chatGPT(String text) {
  static String response = "";

  // ★ InitBuffer（stream:false などの分岐）を廃止し、毎回 chat_doc をゼロから組み立てる
  chat_doc.clear();
  chat_doc["model"]  = LLM_MODEL_NAME;
  // chat_doc["stream"] = true;  // OpenAI互換SSEで統一（比較しやすくする）

  // ★ollama(AI.Local)は stream:false で安定させる。OpenAI互換SSEは他の実装で使用。
  chat_doc["stream"] = (MODEL_VER == "ollama-LLM") ? false : true;
  // ★前回の回答が残って会話履歴に混ざらないように毎回クリア
  g_last_llm_answer = "";

  JsonArray messages = chat_doc.createNestedArray("messages");

  if (g_test_disable_history) {
    // ===== テスト用: 今回の質問だけ送る =====
    JsonObject msg = messages.createNestedObject();
    msg["role"] = "user";
    msg["content"] = replyInstruction200() + text;
  } else {
    /// 質問をチャット履歴に追加
    chatHistory.push_back(text);

    /// チャット履歴が最大数を超えた場合、古い質問と回答を削除
    if (chatHistory.size() > MAX_HISTORY * 2) {
      chatHistory.pop_front();
      chatHistory.pop_front();
    }

    for (int i = 0; i < chatHistory.size(); i++) {
      JsonArray messages = chat_doc["messages"];
      JsonObject systemMessage1 = messages.createNestedObject();
      if (i % 2 == 0) {
        systemMessage1["role"] = "user";
        if (i == (int)chatHistory.size() - 1) {
          systemMessage1["content"] = replyInstruction200() + chatHistory[i];
        } else {
          systemMessage1["content"] = chatHistory[i];
        }
      } else {
        systemMessage1["role"] = "assistant";
        systemMessage1["content"] = chatHistory[i];
      }
    }
  }


  String json_string;
  serializeJson(chat_doc, json_string);
  response = chatGpt(json_string);

  if (!g_test_disable_history) {
    if (g_last_llm_answer.length() > 0) {
      chatHistory.push_back(g_last_llm_answer);
    } else {
      chatHistory.push_back(response);
    }
  }

  serializeJsonPretty(chat_doc, json_string);
  Serial.println("====================3");
  Serial.println(json_string);
  Serial.println("====================3");

  return response;
}


String SpeechToText(bool isGoogle) {
    Serial.println("\r\n音声録音開始\r\n");

    String ret = "";
    if (isGoogle) {
        Serial.println("Google音声認識は未対応です");
    } else {
        Serial.printf("[HEAP][STT] before new AudioWhisper: free=%u largest=%u\n",
                      (unsigned)ESP.getFreeHeap(),
                      (unsigned)ESP.getMaxAllocHeap());
        AudioWhisper* audio = new AudioWhisper();
        Serial.printf("[HEAP][STT] after  new AudioWhisper: free=%u  audio=%p\n",
                      (unsigned)ESP.getFreeHeap(), audio);

        // 録音実行(設定適用は runAudioRecording 内で行う)
        runAudioRecording(*audio);
        
        Serial.printf("[HEAP][STT] after  runAudioRecording: free=%u largest=%u\n",
                      (unsigned)ESP.getFreeHeap(),
                      (unsigned)ESP.getMaxAllocHeap());

        // 録音結果の確認
        size_t recorded_size = audio->GetSize();
        if (recorded_size <= 44 + 500) {  // ヘッダー + 最小音声データ
            Serial.println("ERROR: 録音データが不足しています");
            avatar.setSpeechText("音声が検出されませんでした");
            delete audio;
            return "";
        }
        
        Serial.printf("SUCCESS: 録音完了 %zu bytes\n", recorded_size);
        
        // 認識処理開始
        avatar.setSpeechText("認識中…");
        Serial.println("音声認識開始");
        
        Serial.printf("[HEAP][STT] before new Whisper client: free=%u largest=%u\n",
                      (unsigned)ESP.getFreeHeap(),
                      (unsigned)ESP.getMaxAllocHeap());
        // ★バグ修正: LLM_SERVER_IP → WHISPER_SERVER_IP/PORT を使う
        Whisper* cloudSpeechClient = new Whisper(
            WHISPER_SERVER_IP.c_str(),
            WHISPER_SERVER_PORT,
            WHISPER_SERVER_PATH.c_str()   // ★ path を渡す（これで config.set が実際に効く）
        );
        Serial.printf("[HEAP][STT] after  new Whisper client: free=%u  client=%p\n",
                      (unsigned)ESP.getFreeHeap(), cloudSpeechClient);
        // LANG_CODE ("ja-jp" / "en-US" / "zh-CN" 等) から Whisper 用短縮コード ("ja"/"en"/"zh") を導出し、
        // リクエストごとに STT 言語を切り替える。detect_language は通常 false 固定。
        String sttLanguage = whisperLanguageFromLangCode(LANG_CODE);
        Serial.printf("[STT] LANG_CODE=%s -> whisper language=%s\n",
                      LANG_CODE.c_str(), sttLanguage.c_str());
        Serial.printf("[HEAP][STT] before Transcribe: free=%u\n", (unsigned)ESP.getFreeHeap());
        ret = cloudSpeechClient->Transcribe(audio, sttLanguage.c_str(), false);
        Serial.printf("[HEAP][STT] after  Transcribe: free=%u\n", (unsigned)ESP.getFreeHeap());
        delete cloudSpeechClient;
        delete audio;
        Serial.printf("[HEAP][STT] after  delete: free=%u largest=%u\n",
                      (unsigned)ESP.getFreeHeap(),
                      (unsigned)ESP.getMaxAllocHeap());
    }

    /* 文字列修正 */
    ret.replace("\r\n", "");
    ret.replace("\n", "");   
    ret.replace("\r", "");

    return ret;
}


void setlang_messege()
{
  if (LANG_CODE == "zh-CN") {
    M5.Display.setFont(&fonts::efontCN_12); // Adjust for SSD1306 (Connect info)
    avatar.setSpeechFont(&fonts::efontCN_12);  // Adjust for SSD1306
    message_help_you = "我可以帮你吗？";
    message_You_said = "知道了:";
    message_tinking = "我在思考";
    message_cant_hear = "我听不到";
    message_dont_understand = "我不明白";

  } else if (LANG_CODE == "en-US") {
    message_help_you = "May I help you?";
    message_You_said = "You said:";
    message_tinking = "Thinking...";
    message_cant_hear = "I can't hear you.";
    message_dont_understand = "I don't understand.";

  } else {  // LANG_CODE = "ja-jp";
    // M5.Display.setFont(&fonts::lgfxJapanGothic_12); // Adjust for SSD1306 (Connect info)
    // avatar.setSpeechFont(&fonts::lgfxJapanGothic_12);  // Adjust for SSD1306
    // M5.Display.setFont(&fonts::lgfxJapanMinchoP_12); // Adjust for AtomS3
    // avatar.setSpeechFont(&fonts::lgfxJapanMinchoP_12);  // Adjust for AtomS3
    // M5.Display.setFont(&fonts::lgfxJapanMincho_12); // Adjust for SSD1306 (Connect info)
    M5.Display.setFont(&fonts::lgfxJapanMincho_12); // Adjust for SSD1306 (Connect info)
    avatar.setSpeechFont(&fonts::lgfxJapanMincho_12);  // Adjust for SSD1306
    
    message_help_you = "御用でしょうか？";
    message_You_said = "認識:";
    message_tinking = "考え中…";
    message_cant_hear = "聞き取れませんでした";
    message_dont_understand = "わかりません";

  }

}

// -----------------------------------------------------------------------------
// Piper TTS over HTTP: GET /tts.wav?text=...
// -----------------------------------------------------------------------------
static String urlEncodeUTF8(const String& s) {
  String out;
  out.reserve(s.length() * 3);
  const uint8_t* p = (const uint8_t*)s.c_str();
  for (size_t i = 0; i < s.length(); i++) {
    uint8_t c = p[i];
    // unreserved: ALPHA / DIGIT / "-" / "." / "_" / "~"
    bool unreserved =
      (c >= 'A' && c <= 'Z') ||
      (c >= 'a' && c <= 'z') ||
      (c >= '0' && c <= '9') ||
      (c == '-' || c == '.' || c == '_' || c == '~');
    if (unreserved) out += (char)c;
    else {
      char buf[4];
      snprintf(buf, sizeof(buf), "%%%02X", (unsigned)c);
      out += buf;
    }
  }
  return out;
}

static void runPiperDirectTest() {
  Serial.println("===== Piper Direct Test Start =====");

  avatar.setExpression(Expression::Happy);

  speak_piper_http_chunked("こんにちは。", true);
  delay(1500);

  speak_piper_http_chunked("おはようございます。", true);
  delay(1500);

  speak_piper_http_chunked("今日はいい天気ですね。", true);
  delay(1500);

  avatar.setSpeechText("");
  avatar.setExpression(Expression::Neutral);

  Serial.println("===== Piper Direct Test End =====");
}

/// Called when a metadata event occurs (i.e. an ID3 tag, an ICY block, etc.
void MDCallback(void *cbData, const char *type, bool isUnicode, const char *string)
{
  const char *ptr = reinterpret_cast<const char *>(cbData);
  (void) isUnicode; // Punt this ball for now
  /// Note that the type and string may be in PROGMEM, so copy them to RAM for printf
  char s1[32], s2[64];
  strncpy_P(s1, type, sizeof(s1));
  s1[sizeof(s1)-1]=0;
  strncpy_P(s2, string, sizeof(s2));
  s2[sizeof(s2)-1]=0;
  Serial.printf("METADATA(%s) '%s' = '%s'\n", ptr, s1, s2);
  Serial.flush();
}

/// Called when there's a warning or error (like a buffer underflow or decode hiccup)
void StatusCallback(void *cbData, int code, const char *string)
{
  const char *ptr = reinterpret_cast<const char *>(cbData);
  /// Note that the string may be in PROGMEM, so copy it to RAM for printf
  char s1[64];
  strncpy_P(s1, string, sizeof(s1));
  s1[sizeof(s1)-1]=0;
  Serial.printf("STATUS(%s) '%d' = '%s'\n", ptr, code, s1);
  Serial.flush();
}

void lipSync(void *args)  // Add for M5 avatar
{
  float gazeX, gazeY;
  int level = 0;
  DriveContext *ctx = (DriveContext *)args;
  Avatar *avatar = ctx->getAvatar();
  for (;;)
  {
    if (g_piper_tts_playing) {
      // Simple mouth animation while TTS is playing
      static bool flip = false;
      flip = !flip;
      avatar->setMouthOpenRatio(flip ? 0.6f : 0.15f);
    } else {
      avatar->setMouthOpenRatio(0.0f);
    }
    delay(50);
  }
}

void Wifi_setup() {
  Serial.println("Connect:WiFi");
  M5.Display.println("Connect:WiFi");

  WiFi.disconnect(true, true);
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_STA);

  // NVS から SSID / パスワードを読み込み
  String ssid = "";
  String pass = "";
  preferences.begin("wifi", true);
  ssid = preferences.getString("ssid", "");
  pass = preferences.getString("pass", "");
  preferences.end();

  bool connected = false;

  if (ssid.length() > 0) {
    Serial.print("[WiFi] Try SSID: ");
    Serial.println(ssid);

    WiFi.begin(ssid.c_str(), pass.c_str());

    uint32_t start = millis();
    while (WiFi.status() != WL_CONNECTED &&
           (millis() - start) < 15000) {  // 15秒だけ待つ
      Serial.print(".");
      M5.Display.print(".");
      delay(500);
    }
    Serial.println("");
    M5.Display.println("");

    if (WiFi.status() == WL_CONNECTED) {
      // If we were in AP config mode, stop DNS/mDNS
      if (g_apPortalRunning) {
        dnsServer.stop();
        MDNS.end();
        g_apPortalRunning = false;
      }

      connected = true;
      Serial.print("[WiFi] Connected: ");
      Serial.println(WiFi.localIP());
    }
  }

  if (!connected) {
    Serial.println("[WiFi] 未接続。設定用APモードに入ります。");
    startWifiConfigPortal();
  }
}


// ============================================================
// Phase 3-A: 会話×モーション制御ヘルパー
// ============================================================

static void motion_on_conversation_start() {
  if (!g_demo_motion_enabled) return;
  if (g_estop_active) return;
  if (g_audio_busy) return;
#ifdef ENABLE_SERVO
  g_talk_motion_active   = true;
  g_talk_motion_start_ms = millis();
  g_talk_motion_nod_done = false;
  servoRequestMotion(MOTION_WAKE_CENTER);   // ★ まずウォームアップ → 自動でNODへ
  Serial.println("[TalkMotion] start -> WAKE_CENTER");
#endif
}

static void motion_on_conversation_end() {
#ifdef ENABLE_SERVO
  if (!g_talk_motion_active) return;
  g_talk_motion_active   = false;
  g_talk_motion_nod_done = false;
  servoRequestMotion(MOTION_INIT);  // 終了時: init
  Serial.println("[TalkMotion] end -> INIT");
#endif
}

static void motion_force_stop() {
#ifdef ENABLE_SERVO
  g_talk_motion_active   = false;
  g_talk_motion_nod_done = false;
  servoRequestMotion(MOTION_INIT);
  Serial.println("[TalkMotion] force stop -> INIT");
#endif
}

/// 会話ロジック
// start_talking関数の修正版（main.cppの該当部分を置き換え）
void start_talking() {
    // ===== 安全ガード: estop / audio_busy =====
    if (g_estop_active) {
        Serial.println("[estop] start_talking blocked");
        return;
    }
    if (g_audio_busy) {
        Serial.println("[audio_busy] start_talking skipped");
        return;
    }

    // ===== relay 状態の安全初期化 =====
    // exec_chatGPT() を通らない早期分岐でも前回の回答が残らないようにする。
    g_last_llm_answer = "";

    // 音声入力経路は handle_text_chat_set() を経由しないため、必ず非relayとして扱う。
    if (TEXTAREA == "") {
      g_is_relay_request = false;
      g_relay_origin_question = "";
      g_relay_previous_answer = "";
      g_relay_hop = 0;
      g_relay_origin = "";
    }    

M5.Speaker.setVolume(PiperPlus_voice_volume);
ensureSpeakerActive();

if (TEXTAREA != "") {
    // テキスト入力経路  
    M5.Speaker.tone(2000, 100, 0, true);
    M5.Speaker.tone(1000, 100, 0, false);
    delay(220);
    M5.Speaker.stop();
    delay(300);
} else {
    // マイク入力経路
    M5.Speaker.tone(2000, 80, 0, true);
    delay(100);
    M5.Speaker.stop();
    delay(50);  
    // Serial.println("[Audio] start beep skipped for mic path");
}

// end()後に再開（initSpeakerOnceはスキップされるため直接呼ぶ）
    
    avatar.setExpression(Expression::Happy);
    avatar.setSpeechText(message_help_you.c_str());
    String ret;

    CHARACTER_VOICE Chr_Vic;
    // キャラクター設定
    if (CHARACTER == "02") {
        Chr_Vic.normal = 2; Chr_Vic.happy = 0; Chr_Vic.sasa_yaki = 36;
    } else if (CHARACTER == "13") {
        Chr_Vic.normal = 13; Chr_Vic.happy = 83; Chr_Vic.sasa_yaki = 86;
    } else if (CHARACTER == "23") {
        Chr_Vic.normal = 23; Chr_Vic.happy = 24; Chr_Vic.sasa_yaki = 25;
    } else {
        Chr_Vic.normal = 3; Chr_Vic.happy = 1; Chr_Vic.sasa_yaki = 22;
    }
    TTS_PARMS = TTS_SPEAKER + String(Chr_Vic.normal);
    
    if (TEXTAREA == "") {
        // ▼ マイク入力時は常にローカル Whisper STT を使う
        //    （Google STT は使わない前提のミニマル構成）
        Serial.println("Whisper STT開始 (local)");
        markSpeakerEndedByAudioWhisper();
        ret = SpeechToText(false);  // VAD録音実行（Piper-plus制御なし）
        Serial.print("Whisper STT完了 / result = ");
        Serial.println(ret);
        // Phase 3-A: 音声入力はSTT完了（認識結果が出た直後）にモーション開始
        if (ret != "") {
            motion_on_conversation_start();
        }
    } else {
        // Web UI などからテキストが直接入力されている場合
        // Phase 3-A: テキスト入力は即座にモーション開始
        motion_on_conversation_start();
        ret = TEXTAREA;
    }

    // **Step 4**: 録音完了後の結果音制御（main.cppで統一管理）
    if (TEXTAREA == "") {
        if (ret != "") {
            // 録音成功音
            Serial.println("録音成功音再生");
            delay(100);
            ensureSpeakerActive();
            M5.Speaker.tone(2000, 100, 0, true);
            M5.Speaker.tone(1500, 100, 0, false);
            delay(200);
            M5.Speaker.stop();
            delay(150);
        } else {
            // 録音失敗音
            Serial.println("録音失敗音再生");
            delay(100);
            ensureSpeakerActive();
            M5.Speaker.tone(500, 300);
            delay(320);
            M5.Speaker.stop();
            delay(150);
        }
    }

TEXTAREA = "";
    String You_said = message_You_said + ret;
    Speech_Recognition = ret;

Serial.println("point01");

// **Step 5**: LLM応答と音声合成
if (ret != "") {
    Serial.println("point03");

    // ===== テスト用: LLMを通さず固定文を直接Piperへ =====
    if (g_test_piper_direct_mode) {
        Serial.println("[TEST] direct Piper mode");
        String testText = sanitizeForPiper(ret);
        if (testText.length() == 0) {
            testText = "こんにちは。";
        }

        avatar.setSpeechText(testText.c_str());
        avatar.setExpression(Expression::Happy);
        Serial.println("Piper direct playback");
        speak_piper_http_chunked(testText, true);

        Serial.println("point08");
        delay(2000);
        avatar.setSpeechText("");
        avatar.setExpression(Expression::Neutral);
        motion_on_conversation_end();  // Phase 3-A: 早期return前に終了
        return;
    }

    String response = "";
    if (OPENAI_API_KEY == "") {
        response = "初めに、URL：" + WiFi.localIP().toString() + "をブラウザに入力し、APIキーを設定してください";
    } else if (isOpenAICompatVisionBackend() && WEB_HAS_IMAGE) {
        // ===== OpenAI互換VLMバックエンド + 画像あり → vision JSON を手組みして送る =====
        // LM Studio / llama.cpp 共通経路
        Serial.println("[Vision] buildOpenAIVisionChatJson start");
        String visionJson = buildOpenAIVisionChatJson(ret, WEB_IMAGE_DATA_URL);
        uint16_t llm_port = getDefaultLlmPort();
        String url = "http://" + LLM_SERVER_IP + ":" + String(llm_port) + "/v1/chat/completions";
        Serial.printf("[Vision] url=%s\n", url.c_str());

        String streamResult = http_post_json_openai_stream(url, visionJson.c_str());
        // http_post_json_openai_stream は内部でストリーミングTTSを完了させている。
        // 正常時は response を空にして Step6 の二重発話を防ぐ。
        // エラー時のみメッセージを伝搬して喋らせる。
        if (streamResult.startsWith("画像対応エラー") ||
            streamResult.startsWith("HTTPエラー") ||
            streamResult.startsWith("通信エラー") ||
            streamResult.startsWith("接続エラー")) {
          response = streamResult;
          Serial.printf("[Vision] error response: %s\n", response.c_str());
        } else {
          response = "";
        }

        // 使い終わったら即クリア（次回の通常会話に引きずらない）
        WEB_IMAGE_DATA_URL = "";
        WEB_HAS_IMAGE = false;
    } else {
        response = exec_chatGPT(ret);
    }

    // **Step 6**: Speak if response is non-empty (streaming modes already spoke)
    if (response.length() > 0 && response != "Error" && response != "Error1") {
        response = sanitizeForPiper(response);
        if (response.length() > 0) {
            avatar.setSpeechText(response.c_str());
            avatar.setExpression(Expression::Happy);
            Serial.println("Piper TTS playback");
            speak_piper_http_chunked(response, true);
        } else {
            Serial.println("TTS skipped after sanitize");
        }
    } else {
        Serial.println("TTS skipped (already spoken or error)");
    }

    Serial.println("point08");
    motion_on_conversation_end();   // Phase 3-A: TTS完了直後に停止（delay前）
    delay(2000);
    avatar.setSpeechText("");
    avatar.setExpression(Expression::Neutral);
}

Serial.println("point09");    


    // ===== リレー転送 =====
    do {
      if (ret == "") {
        Serial.println("[relay] skip: ret empty");
        break;
      }
      if (isNonRelayableAnswer(g_last_llm_answer)) {
        Serial.println("[relay] skip: non-relayable answer");
        break;
      }
      if (NEXT_SPEACH_IP.length() <= 5) {
        Serial.println("[relay] 終着点: 転送なし");
        break;
      }

      if (!g_is_relay_request) {
        g_relay_origin_question = ret;
        g_relay_origin = WiFi.localIP().toString();
        g_relay_hop = 0;
      }

      g_relay_previous_answer = g_last_llm_answer;

      String nextPrompt = buildRelayPromptForNext(
        g_relay_origin_question,
        g_relay_previous_answer,
        LLM_MODEL_NAME
      );

      Serial.printf("[relay] 転送開始: next=%s hop=%d\n",
                    NEXT_SPEACH_IP.c_str(), g_relay_hop + 1);
      runText_chatTominimal(NEXT_SPEACH_IP, nextPrompt);
    } while (false);

    // relay 状態の後片付け（次回の入力に引きずらない）
    g_is_relay_request = false;
    g_relay_origin_question = "";
    g_relay_previous_answer = "";
    g_relay_hop = 0;
    g_relay_origin = "";

    TTS_PARMS = TTS_SPEAKER + String(Chr_Vic.normal);
}


void handle_text_chat() {
  /// ファイルを読み込み、クライアントに送信する
  server.send(200, "text/html", TEXT_CHAT_HTML);
}
void handle_text_chat_set() {
  if (server.method() != HTTP_POST) {
    server.send(405, "text/plain", "Method Not Allowed");
    return;
  }

  // ===== 安全ガード: estop / audio_busy =====
  if (g_estop_active) {
    server.send(503, "application/json", "{\"error\":\"estop_active\"}");
    return;
  }
  if (g_audio_busy) {
    server.send(409, "application/json", "{\"error\":\"audio_busy\"}");
    return;
  }

  String body = server.arg("plain");
  String text;
  String imageDataUrl;

  parseTextChatBody(body, text, imageDataUrl);

  TEXTAREA = text;
  WEB_IMAGE_DATA_URL = imageDataUrl;
  WEB_HAS_IMAGE = imageDataUrl.startsWith("data:image/");

  // AtomS3R 保護: data URL が大きすぎる場合は拒否（90KB ≒ 512px JPEG 目安）
  // if (WEB_HAS_IMAGE && WEB_IMAGE_DATA_URL.length() > 90000) {
  //   WEB_IMAGE_DATA_URL = "";
  //   WEB_HAS_IMAGE = false;
  //   server.send(413, "text/plain", "Image too large");
  //   return;
  // }
  // AtomS3R 保護: data URL が大きすぎる場合は拒否
  // 512px / JPEG quality 0.65 を通しやすいように上限を少し緩める
  if (WEB_HAS_IMAGE && WEB_IMAGE_DATA_URL.length() > 140000) {
    WEB_IMAGE_DATA_URL = "";
    WEB_HAS_IMAGE = false;
    server.send(413, "text/plain", "Image too large");
    return;
  }
  Serial.printf("[Vision] imageDataUrl length=%u\n", WEB_IMAGE_DATA_URL.length());
  Serial.printf("[Vision] imageDataUrl head=%.48s\n", WEB_IMAGE_DATA_URL.c_str());

  Serial.print("TEXTAREA: "); Serial.println(TEXTAREA);
  Serial.print("WEB_HAS_IMAGE: "); Serial.println(WEB_HAS_IMAGE ? "true" : "false");

  // ===== リレー受信判定とメタ情報取得 =====
  String hdrOriginQ = server.hasHeader("X-Origin-Question")
                      ? server.header("X-Origin-Question") : "";

  bool relayHeaderValid = false;
  String decodedOriginQ = "";
  if (hdrOriginQ.length() > 0) {
    decodedOriginQ = relayBase64Decode(hdrOriginQ);
    if (decodedOriginQ.length() > 0) {
      relayHeaderValid = true;
    } else {
      Serial.println("[relay] X-Origin-Question のデコード失敗 → 通常入力扱いにフォールバック");
    }
  }

  if (relayHeaderValid) {
    g_is_relay_request = true;
    g_relay_origin_question = decodedOriginQ;

    String hdrPrevA = server.hasHeader("X-Previous-Answer")
                      ? server.header("X-Previous-Answer") : "";
    g_relay_previous_answer = (hdrPrevA.length() > 0) ? relayBase64Decode(hdrPrevA) : "";

    String hdrHop = server.hasHeader("X-Relay-Hop")
                    ? server.header("X-Relay-Hop") : "0";
    g_relay_hop = hdrHop.toInt();

    g_relay_origin = server.hasHeader("X-Relay-Origin")
                    ? server.header("X-Relay-Origin") : "";

    Serial.printf("[relay] 受信: hop=%d origin=%s origin_q_len=%u prev_a_len=%u\n",
      g_relay_hop, g_relay_origin.c_str(),
      g_relay_origin_question.length(), g_relay_previous_answer.length());
  } else {
    g_is_relay_request = false;
    g_relay_origin_question = "";
    g_relay_previous_answer = "";
    g_relay_hop = 0;
    g_relay_origin = "";
  }

  server.send(200, "text/plain", "OK");

  /// おしゃべり開始
  start_talking();
}


// ===== ボタン処理（改良版：シンプルなクリック回数判定）=====
static void resetButtonState() {
  g_clickCount = 0;
  g_firstClickMs = 0;
  Serial.println("[BTN] Button state reset");
}

// -----------------------------------------------------------------------------
// Sensor several-shots measurement handlers (count-based termination)
// -----------------------------------------------------------------------------
// -- Sensor : 心拍数・酸素濃度モニタ
void runHeartMeasurement() {
  Serial.println("[HEART] init...");
  if (!pox.begin()) {
    Serial.println("[HEART] begin failed");
    // ★ 失敗時にも前回の文字が残らないようにクリア/通知
    avatar.setExpression(Expression::Sad);
    avatar.setSpeechText(" HEART: init failed");
    return;
  }
  pox.setOnBeatDetectedCallback(onBeatDetected);
  hrZeroRunning = false;
  hrZeroBeeped  = false;

  // ★ 計測開始時に必ず上書き（前回値を消す）
  avatar.setExpression(Expression::Happy);
  avatar.setSpeechText(" HEART Check -Start");

  // 認識 - 開始音
  ensureSpeakerActive();
  M5.Speaker.tone(2000, 100, 0, true);
  M5.Speaker.tone(1000, 100, 0, false);
  // delay(1000); //: なぜかセンサーが反応しなくなるので省略

  // ---------------------------------------------------------------------------
  // 計測パラメータ
  //
  // check_count:
  //   平均に採用する「有効HRサンプル」の目標数。
  //
  // HEART_WARMUP_VALID_SKIP:
  //   MAX30100 は指検出 〜 最初の Beat までの AC/DC が安定しないため、
  //   最初の N 件の有効HRサンプルは平均に入れない。
  //   実機ログで最初の1件だけ SpO2=76% のような外れ値が出ていた対策。
  //
  // SpO2 は参考値扱い（展示・デモ用途）。医療用途ではない。
  //
  // 宣言を先に置き、開始ログは printf で定数と連動させる（文言ズレ防止）。
  // ---------------------------------------------------------------------------
  uint16_t check_count = 8;   // 調査回数（平均採用する有効HRサンプル数）
  static constexpr int HEART_WARMUP_VALID_SKIP = 2;  // ウォームアップで捨てる有効HRサンプル数
  int      warmup_valid_seen = 0;
  uint16_t valid_count = 0;
  uint32_t lastLog = 0;

  Serial.printf(
    "[HEART] measuring until %u valid HR samples (30..240 bpm), skipping first %d as warmup...\n",
    (unsigned)check_count,
    HEART_WARMUP_VALID_SKIP
  );

  // 平均算出（既存どおり）
  double   sum_hr = 0.0, sum_spo2 = 0.0;
  uint16_t n_hr   = 0,   n_spo2   = 0;

  while (valid_count < check_count) {  // 調査回数分繰り返し
    M5.update();
    pox.update();

    uint32_t now = millis();
    if (now - lastLog >= 1000) {
      float hr = pox.getHeartRate();
      float s  = pox.getSpO2();
      Serial.printf("[HEART] HR=%.1f bpm, SpO2=%.1f%%\n", hr, s);
      lastLog = now;

      // ★ 有効でなくても毎秒プレースホルダ/最新値を表示して“前回の文字”を上書き
      {
        char msg[32];
        if (hr >= 30.f && hr <= 240.f) {
          snprintf(msg, sizeof(msg), " HR %.1f bpm", hr);
        } else {
          snprintf(msg, sizeof(msg), " HR --.- bpm");
        }
        avatar.setExpression(Expression::Neutral);
        avatar.setSpeechText(msg);
      }

      // 有効値ならカウントアップ
      if (hr >= 30.f && hr <= 240.f) {
        if (warmup_valid_seen < HEART_WARMUP_VALID_SKIP) {
          // ウォームアップ期間：平均には入れない
          warmup_valid_seen++;
          Serial.printf("[HEART] warmup skip %d/%d (HR=%.1f SpO2=%.1f)\n",
                        warmup_valid_seen, HEART_WARMUP_VALID_SKIP, hr, s);
          avatar.setExpression(Expression::Neutral);
        } else {
          valid_count++;

          // 平均用に加算
          sum_hr += hr; n_hr++;

          // SpO2 は参考値のため、90〜100% に入ったもののみ平均採用。
          // （指検出直後の低SpO2外れ値を除外するため条件を厳しくしている）
          if (s >= 90.0f && s <= 100.0f) {
            sum_spo2 += s; n_spo2++;
          } else if (s > 0.0f) {
            Serial.printf("[HEART] SpO2 out of accept range, skipped (s=%.1f)\n", s);
          }

          avatar.setExpression(Expression::Doubt);
          // ※ setSpeechText は上で毎秒更新済みなので、ここはログのみでもOK
          Serial.printf("[HEART] valid %u/%u\n", (unsigned)valid_count, (unsigned)check_count);
        }
      }

      // HR=0が5秒続いたら1回ビープ（既存ロジック）
      bool isZero = !(hr > HR_ZERO_THRESH);
      if (isZero) {
        if (!hrZeroRunning) { hrZeroRunning = true; hrZeroStartMs = now; hrZeroBeeped = false; }
        else if (!hrZeroBeeped && (now - hrZeroStartMs >= HR_ZERO_TIMEOUT_MS)) {
          M5.Speaker.tone(2000, 100, 0, false);
          Serial.println("Beep (5s HR=0)");
          hrZeroBeeped = true;
        }
      } else {
        hrZeroRunning = false;
        hrZeroBeeped  = false;
      }
    }
    delay(2);
  }

  // 平均の計算と表示
  // SpO2 は有効サンプル 0 件なら NAN として扱う（以降の表示・プロンプトで分岐）
  auto avgOrNan = [](double sum, uint16_t n)->double { return n ? (sum / n) : NAN; };
  const double avg_hr   = avgOrNan(sum_hr,   n_hr);
  const double avg_spo2 = avgOrNan(sum_spo2, n_spo2);

  if (isnan(avg_spo2)) {
    Serial.printf("[HEART][avg] HR=%.1f bpm, SpO2(ref)=--.-%% (no valid SpO2 sample)\n", avg_hr);
  } else {
    Serial.printf("[HEART][avg] HR=%.1f bpm, SpO2(ref)=%.1f%%\n", avg_hr, avg_spo2);
  }

  // 平均をAvatarにも一度表示（SpO2 は参考値として (ref) 付き、NANなら --.-）
  {
    avatar.setExpression(Expression::Happy);
    char msg[48];
    if (isnan(avg_spo2)) {
      snprintf(msg, sizeof(msg), " Avg HR %.1f bpm, SpO2(ref)=--.-%%", avg_hr);
    } else {
      snprintf(msg, sizeof(msg), " Avg HR %.1f bpm, SpO2(ref)=%.1f%%", avg_hr, avg_spo2);
    }
    avatar.setSpeechText(msg);
  }
  // 認識-終了音
  ensureSpeakerActive();
  M5.Speaker.tone(2000, 100, 0, true);
  M5.Speaker.tone(1500, 100, 0, false); 
  delay(2500);

  // センシング値で、どんな状態かをLLMに質問する
  // ------------------------------------------------------------------
  // プロンプト方針:
  //   - これは展示・デモ用の「参考値モニタ」であり医療機器ではない
  //   - LLMに医療判断（診断・受診推奨・異常指摘）をさせない
  //   - SpO2 は「参考値」と明記
  //   - SpO2 が取れていない場合は数値を含めない分岐
  // ------------------------------------------------------------------
  {
    // SpO2 パート（NAN なら数値は出さず測定不安定と伝える）
    String spo2_part;
    if (isnan(avg_spo2)) {
      spo2_part = "SpO2参考値: 測定が不安定のため今回は算出なし。";
    } else {
      spo2_part = "平均SpO2参考値: " + String(avg_spo2, 1) + " %。";
    }

    String prompt =
      String("これは展示・デモ用の心拍/SpO2参考値モニタの結果です。")
      + "医療機器ではないため、診断・受診推奨・体調判断はしないでください。"
      + "『正常』『異常』『通院』『受診』『治療』などの医療的判断に関する語は使わないでください。"
      + "平均心拍: " + String(avg_hr, 1) + " bpm。"
      + spo2_part
      + "デモのナレーションとして、測定できた数値を軽く紹介してください。"
      + "日本語100字以内でお願いします。";

    // LLMに意見を問合せ
    // doPostChatStream(prompt);  // LLMに送信（後で）
    TEXTAREA = prompt;
    Serial.print("TEXTAREA: "); Serial.println(TEXTAREA);
    /// おしゃべり開始
    start_talking();
  }
  
  Serial.println("[HEART] done.\n");
}

// I2C Unit の初期化を1回だけ行うガード関数
// MAX30100ライブラリは標準Wireを使うため、UnitポートもWireに統一する
static void initUnitI2COnce() {
  static bool inited = false;
  if (inited) return;
  inited = true;

  // 以前の状態を一度解放してから、Unitポート(GPIO2/GPIO1)に割り当てる
  I2C_UNIT.end();
  delay(10);

  bool ok = I2C_UNIT.begin(I2C_UNIT_SDA, I2C_UNIT_SCL, I2C_UNIT_FREQ);

  Serial.printf("[I2C] Unit I2C initialized on Wire: SDA=GPIO%d, SCL=GPIO%d, FREQ=%u, result=%s\n",
                I2C_UNIT_SDA,
                I2C_UNIT_SCL,
                I2C_UNIT_FREQ,
                ok ? "OK" : "NG");
}

// M5.Speaker.begin() を1回だけ呼ぶガード関数
// Piper-plus の再初期化ループで毎回 Speaker.begin() が呼ばれると
// M5 ライブラリが Wire(38/39) を再初期化し、I2C多重初期化警告が出るため
static void initSpeakerOnce() {
  static bool inited = false;
  if (inited) return;
  inited = true;
  M5.Speaker.begin();  // 実際のSpeaker初期化（1回だけ実行）
}

// ===== Audio / I2S state helpers (Plan A) =====
static bool g_speaker_active = false;  // “現在Speakerが使える状態”をmain側で管理

static void ensureSpeakerActive() {
  initSpeakerOnce();

  if (!g_speaker_active) {
    M5.Speaker.begin();
    g_speaker_active = true;
    Serial.println("[Audio] Speaker re-begin (ensureSpeakerActive)");
  }
}


// “録音に入る前にSpeakerがendされる前提”なので、main側でフラグを落としておく
static void markSpeakerEndedByAudioWhisper() {
  g_speaker_active = false;
}

// VAD NVSキーが未作成の場合にデフォルト値でキーを作成する(初回起動時のNOT_FOUNDログ抑制)
// Step 3c: VAD既定値を実運用値に格上げ
//   - vad_threshold:   800 → 180  (実運用で安定した検出閾値)
//   - vad_min_speech:  10  → 4    (日本語の立ち上がり吸収)
//   - vad_max_silence: 20  → 40   (息継ぎ吸収、post_speech と合算で約0.6秒)
//   - vad_enabled:     false → true (VAD常時有効化)
// Step 4c: vad_mode / vad_calibrated キー作成を廃止(機能削除に伴い不要)
//          既存デバイスの該当キーは残存するが、読まなければ実害なし。
static void ensureVadNvsKeys() {
  preferences.begin("my-app", false);  // 書き込みモード
  bool created = false;
  if (!preferences.isKey("vad_threshold"))   { preferences.putFloat("vad_threshold",   180.0f);   created = true; }
  if (!preferences.isKey("vad_min_speech"))  { preferences.putInt("vad_min_speech",    4);        created = true; }
  if (!preferences.isKey("vad_max_silence")) { preferences.putInt("vad_max_silence",   40);       created = true; }
  if (!preferences.isKey("vad_enabled"))     { preferences.putBool("vad_enabled",      true);     created = true; }
  if (!preferences.isKey("lang_code"))       { preferences.putString("lang_code",      "ja-jp");  created = true; }
  if (!preferences.isKey("character"))       { preferences.putString("character",      "00");     created = true; }
  if (!preferences.isKey("disp_rot"))        { preferences.putUChar("disp_rot", 0); created = true; }
  // ★デバッグログ(問題解消後に削除可)
  Serial.printf("[VAD][NVS] ensureVadNvsKeys: created=%s threshold=%d\n",
    created ? "YES" : "NO",
    preferences.isKey("vad_threshold") ? 1 : 0);
  preferences.end();
}

// ============================================================
// PiperStreamPlayer.h のインクルード
// ここより上で以下がすべて定義済みであること:
//   ensureSpeakerActive(), urlEncodeUTF8()
//   PIPER_TTS_IP, PIPER_TTS_PORT, PiperPlus_voice_volume
//   g_piper_tts_playing, m5spk_virtual_channel, avatar
// ============================================================
#include "PiperStreamPlayer.h"

void setup()
{
  auto mem0 = esp_get_free_heap_size(); // check memory size
  auto cfg = M5.config();

  // Avoid initializing ATOM S3R built-in IMU/RTC (I2C on GPIO45/GPIO0),
  // so we can dedicate I2C_UNIT (Wire1) to the Unit/Port (GPIO2/GPIO1).
  cfg.internal_imu = false;
  cfg.internal_rtc = false;
  /* for ATOMIC ECHO BASE */
  cfg.external_speaker.atomic_echo    = true;  // default=false. use ATOMIC ECHO BASE

  auto mem1 = esp_get_free_heap_size(); // check memory size
  Serial.begin(115200);
  M5.begin(cfg);
  // M5.Display.setRotation(1);  // 回転:右90 for Atom Echo 

  // ★ VAD NVSキーを最序盤で作成（getFloat等より前に実行することでNOT_FOUNDログを抑制）
  ensureVadNvsKeys();

  { /// custom setting
    auto spk_cfg = M5.Speaker.config();
    spk_cfg.sample_rate   = 48000;
    spk_cfg.dma_buf_len   = 512;
    spk_cfg.dma_buf_count = 12;
    spk_cfg.task_priority = 3;
    spk_cfg.task_pinned_core = APP_CPU_NUM;
    M5.Speaker.config(spk_cfg);
  }

  initSpeakerOnce();    // M5.Speaker.begin() を1回だけ実行
  g_speaker_active = true;   // ★Plan A: 初期状態ではSpeakerは使える
  /// set master volume (0~255)
  // M5.Speaker.setVolume(150);  // Adjust for Atom Echo (DO NOT set over 150. This echo Speaker will be broken.)
  M5.Speaker.setVolume(50);  // Adjust for ATOMIC ECHO BASE
  // Wire.begin(2, 1);  // I2C SDA=G2(黄), SCL=G1(白)
  // Ensure the Unit I2C bus is not already running on different pins
#ifdef ENABLE_SERVO
  // ★ ENABLE_SERVO時はGPIO2/GPIO1をサーボ専用にするため、I2C初期化をスキップ
  Serial.println("[I2C] Unit I2C skipped (ENABLE_SERVO: GPIO2/GPIO1 reserved for servo)");
#else
  initUnitI2COnce();  // Wire(GPIO2/GPIO1) の初期化を1回だけ実行
#endif


  // Mic config は1回だけ（beginはしない：Plan A）
  {
    auto mic_cfg = M5.Mic.config();
    mic_cfg.sample_rate = 16000;
    mic_cfg.noise_filter_level = 8;
    mic_cfg.task_pinned_core = APP_CPU_NUM;
    M5.Mic.config(mic_cfg);
    Serial.println("[Mic] config applied in setup (no begin)");
  }

  // Preferencesを初期化（"my-app"という名前空間で開く）
  preferences.begin("my-app", true);  // false=読み書きモード、true=読み取り専用モード
  // Model選択設定を読み込み
  MODEL_VER       = preferences.getString("model_ver",       MODEL_VER);
  LLM_SERVER_IP   = preferences.getString("llm_server_ip",   LLM_SERVER_IP);
  LLM_MODEL_NAME  = preferences.getString("llm_model_name",  LLM_MODEL_NAME);
  OPENAI_API_KEY  = preferences.getString("openai_key",      OPENAI_API_KEY);   // ★追加
  NEXT_SPEACH_IP  = preferences.getString("next_speach_ip",  NEXT_SPEACH_IP);   // ★追加
  LANG_CODE       = normalizeLangCode(preferences.getString("lang_code", LANG_CODE));
  CHARACTER       = preferences.getString("character", CHARACTER);
  LLM_SERVER_PORT = preferences.getUShort("llm_server_port", 0);                // ★追加
  // ★追加: Whisper/Piper 接続先をNVSから復元（なければデフォルト値を維持）
  WHISPER_SERVER_IP   = preferences.getString("whisper_ip",   WHISPER_SERVER_IP);    // whisper_ip
  WHISPER_SERVER_PORT = preferences.getUShort("whisper_port", WHISPER_SERVER_PORT);  // whisper_port
  WHISPER_SERVER_PATH = preferences.getString("whisper_path", WHISPER_SERVER_PATH);  // ★追加: whisper_path
  PIPER_TTS_IP        = preferences.getString("piper_tts_ip",        PIPER_TTS_IP);
  PIPER_TTS_PORT      = preferences.getUShort("piper_tts_port",      PIPER_TTS_PORT);
  PIPER_TTS_LENGTH_SCALE = preferences.getFloat("piper_ls", PIPER_TTS_LENGTH_SCALE);  // piper_ls
  // ★追加: 音声設定の復元（保存形式変更: putString→putUChar のため getUChar で読む）
  PiperPlus_voice_volume = preferences.getUChar("voice_volume", PiperPlus_voice_volume);
  g_display_rotation = preferences.getUChar("disp_rot", 0);
  g_display_rotation = clampDisplayRotation(g_display_rotation);
  // Close the Preferences
  preferences.end();

  /// 画面回転の反映
  applyDisplayRotation(g_display_rotation, false);  

  /// ネットワークに接続
  Wifi_setup(); // Add for Web Setting
  M5.Display.fillScreen(BLACK);
  M5.Display.setCursor(0, 0);
  M5.Display.println("Connect OK-WiFi");

  //PSRAM Initialisation
  // if (psramInit()) {
  //   Serial.println("\nThe PSRAM is correctly initialized");
  // } else {
  //   Serial.println("\nPSRAM does not work");
  // }
  // log_d("Total heap: %d", ESP.getHeapSize());
  // log_d("Free heap: %d", ESP.getFreeHeap());
  // log_d("Total PSRAM: %d", ESP.getPsramSize());
  // log_d("Free PSRAM: %d", ESP.getFreePsram());


  auto mem2 = esp_get_free_heap_size(); // check memory size

  server.on("/", handleRoot);
  server.on("/inline", [](){
    server.send(200, "text/plain", "this works as well");
  });

  /// And as regular external functions:
  server.on("/character_voice", handle_character_voice);
  server.on("/character_voice_set", HTTP_POST, handle_character_voice_set);

  server.on("/piper_plus_voice",     handle_piper_plus_voice);
  server.on("/piper_plus_voice_get", HTTP_GET,  handle_piper_plus_voice_get);
  server.on("/piper_plus_voice_set", HTTP_POST, handle_piper_plus_voice_set);
  server.on("/model_ver", handle_model);
  server.on("/model_ver_get", HTTP_GET, handle_model_get);
  server.on("/llm_models_get", HTTP_GET, handle_llm_models_get);
  server.on("/model_ver_set", HTTP_POST, handle_model_set);
  server.on("/text_chat", handle_text_chat);
  server.on("/text_chat_set", HTTP_POST, handle_text_chat_set);

  // VAD関連のルート(Step 4b 後、必要最小限に縮退)
  server.on("/vad_calibration", handle_vad_calibration);                        // HTML配信(保守用)
  server.on("/vad_status", handle_vad_status);                                  // ステータス確認
  server.on("/vad_set_threshold", HTTP_POST, handle_vad_set_threshold);
  server.on("/vad_calibration_exec", HTTP_POST, handle_vad_calibration_exec);   // キャリブ実行
  server.on("/vad_test_recording", HTTP_POST, handle_vad_test_recording);       // 録音テスト
  // 削除済み(Step 4b): /vad_set_mode, /vad_enable, /vad_disable, /vad_test_fixed
  // 新設予定(Step 5):  /vad_set_threshold


  // ★ Wi-Fi 設定用ハンドラを追加
  server.on("/wifi_config", handle_wifi_config);
  server.on("/wifi_scan", handle_wifi_scan);
  server.on("/wifi_save", HTTP_POST, handle_wifi_save);

  // ===== Control Plane API v1 =====
  server.on("/api/v1/health",          HTTP_GET,  handle_api_health);
  server.on("/api/v1/state",           HTTP_GET,  handle_api_state);
  server.on("/api/v1/capabilities",    HTTP_GET,  handle_api_capabilities);
  server.on("/api/v1/publish",         HTTP_POST, handle_api_publish);
  server.on("/api/v1/service",         HTTP_POST, handle_api_service);
  server.on("/api/v1/action",          HTTP_POST, handle_api_action);
  server.on("/api/v1/action/result",   HTTP_GET,  handle_api_action_result);
  server.on("/api/v1/blob",            HTTP_GET,  handle_api_blob);
  server.on("/api/v1/estop",           HTTP_POST, handle_api_estop);
  server.on("/api/v1/estop/clear",     HTTP_POST, handle_api_estop_clear);

  server.onNotFound(handleNotFound);

  init_chat_doc(json_ChatString.c_str());

  const char* relayHeaders[] = {
    "X-Origin-Question",
    "X-Previous-Answer",
    "X-Relay-Hop",
    "X-Relay-Origin"
  };
  server.collectHeaders(relayHeaders, sizeof(relayHeaders) / sizeof(relayHeaders[0]));

  server.begin(); // Add for Web Setting
  // Serial.println("Setting URL:"); M5.Display.println("Setting URL:");   // 設定URL
  // Serial.print(WiFi.localIP()); M5.Display.print(WiFi.localIP());

  // 1) ローカルネットワークにちゃんとつながっている場合（従来どおり）
  if (WiFi.getMode() == WIFI_STA && WiFi.status() == WL_CONNECTED) {
    Serial.println("Setting URL:");
    M5.Display.println("Setting URL:");   // 設定URL
    
    String ipStr = WiFi.localIP().toString();
    Serial.println(ipStr);
    M5.Display.println(ipStr);

  // 2) ローカルネットワークに未接続で、設定用APモードになっている場合
  } else {
    // APのSSIDを STACKCHAN-XXXX 形式で生成
    uint32_t chipId = (uint32_t)(ESP.getEfuseMac() & 0xFFFFFF);
    String apSsid = "STACKCHAN-" + String(chipId, HEX);
    apSsid.toUpperCase();   // 大文字の方がそれっぽいのでそのまま

    // SoftAP側のIPアドレス（通常は 192.168.4.1）
    IPAddress apIP = WiFi.softAPIP();
    String apIPStr = apIP.toString();

    Serial.println("WiFi Config Mode");
    M5.Display.println("WiFi Config Mode");

    // 画面1行目：SSID案内
    String line1 = "SSID: " + apSsid;
    Serial.println(line1);
    M5.Display.println(line1);

    // 画面2行目：アクセスするIP（192.168.4.1）
    // String line2 = "Access: http://" + apIPStr;
    String line2 = String("Access: http://") + WIFI_CONFIG_HOST + "/";

    Serial.println(line2);
    M5.Display.println(line2);
  }

  // 表示をしばらく残す
  delay(6000);

  /* Avatarの顔位置・サイズ */
  avatar.setScale(.45);               // Adjust for AtomS3
  avatar.setPosition(-72,-100);       // Adjust for AtomS3
  avatar.init();                      // Add for M5 avatar
  avatar.addTask(lipSync, "lipSync"); // Add for M5 avatar
  setlang_messege();  // Set message based on language

  auto mem3 = esp_get_free_heap_size();
  printf("Heap available : %u : %u : %u : %u\n", mem0, mem1, mem2, mem3);

// VAD設定の読み込み
  // Step 3c: 既定値を実運用値(threshold=180, min_speech=4, max_silence=40, enabled=true)に格上げ
  // Step 4c: current_mode / calibrated は構造体から削除済みのため読み込まない
  preferences.begin("my-app", true);
  global_vad_config.threshold = preferences.getFloat("vad_threshold", 180.0f);
  global_vad_config.min_speech = preferences.getInt("vad_min_speech", 4);
  global_vad_config.max_silence = preferences.getInt("vad_max_silence", 40);
  global_vad_config.enabled = preferences.getBool("vad_enabled", true);
  preferences.end();

  // ===== VAD旧NVS値の自動マイグレーション =====
  bool vad_migrated = false;
  if (global_vad_config.threshold >= 500.0f) {
    Serial.printf("[VAD][NVS] migrate threshold %.1f -> 180.0\n",
                  global_vad_config.threshold);
    global_vad_config.threshold = 180.0f;
    vad_migrated = true;
  }
  if (global_vad_config.min_speech >= 10) {
    Serial.printf("[VAD][NVS] migrate min_speech %d -> 4\n",
                  global_vad_config.min_speech);
    global_vad_config.min_speech = 4;
    vad_migrated = true;
  }
  if (global_vad_config.max_silence <= 20) {
    Serial.printf("[VAD][NVS] migrate max_silence %d -> 40\n",
                  global_vad_config.max_silence);
    global_vad_config.max_silence = 40;
    vad_migrated = true;
  }
  if (!global_vad_config.enabled) {
    Serial.println("[VAD][NVS] migrate enabled false -> true");
    global_vad_config.enabled = true;
    vad_migrated = true;
  }

  if (vad_migrated) {
    preferences.begin("my-app", false);
    preferences.putFloat("vad_threshold", global_vad_config.threshold);
    preferences.putInt("vad_min_speech", global_vad_config.min_speech);
    preferences.putInt("vad_max_silence", global_vad_config.max_silence);
    preferences.putBool("vad_enabled", global_vad_config.enabled);
    preferences.end();
  }

  // ===== VAD強制ON(設計方針: VAD常時有効) =====
  // 既定値は true に格上げ済みだが、旧バージョンから持ち越したNVS保存値が
  // false の可能性があるため、互換マイグレーションとして強制 true 上書きを残す。
  // (Step 4 で handle_vad_enable/disable が削除されるため、
  //  この上書きが実質的なVAD有効化の担保となる)
  if (!global_vad_config.enabled) {
    Serial.println("[VAD] NVS has enabled=false, forcing to true (design policy)");
    global_vad_config.enabled = true;
    preferences.begin("my-app", false);
    preferences.putBool("vad_enabled", true);
    preferences.end();
  }


  // ===== Phase 2: Servo initialization =====
#ifdef ENABLE_SERVO
  {
    bool okA = servoA.begin(SERVO_A_PIN, 900, 2100, 50, 14, A_CENTER);
    bool okB = servoB.begin(SERVO_B_PIN, 900, 2100, 50, 14, B_CENTER);
    Serial.printf("[Servo] A(GPIO%d)=%s, B(GPIO%d)=%s\n",
                  SERVO_A_PIN, okA ? "OK" : "NG",
                  SERVO_B_PIN, okB ? "OK" : "NG");
    servosAttached = true;

    // ★ begin() 内で CENTER のdutyは書き込み済み。追加writeは不要。
    //    サーボが物理的にセンターへ到達するまで待つ。
    Serial.println("[Servo] waiting for center position...");
    delay(2000);

    // ★ Core v2: センター到達後に end() で完全 detach して待機。
    //    モーション要求時に begin() で再初期化される。
    servoDetach();

    g_motion = MOTION_INIT;
    g_motionStartMs = millis();
    aCmd = A_CENTER; bCmd = B_CENTER;
    aFilt = (float)A_CENTER; bFilt = (float)B_CENTER;
    Serial.println("[Servo] Boot: INIT (centered, then detached)");

    // Phase 3-A: サーボ制御タスクを Core 0 で起動
    // loop()（Core 1）がブロックされても servoTick() が止まらないようにする
    xTaskCreatePinnedToCore(
      servoTaskFunc,   // タスク関数
      "servoTask",     // タスク名
      2048,            // スタックサイズ（bytes）
      nullptr,         // 引数
      2,               // 優先度（lipSyncタスクより高め）
      nullptr,         // タスクハンドル不要
      0                // Core 0 に固定
    );
    Serial.println("[Servo] servoTask started on Core 0");
  }
#endif

  // ===== PR2-C: Avatar mutex 初期化 =====
  g_avatar_text_mutex = xSemaphoreCreateMutex();
  if (!g_avatar_text_mutex) {
    Serial.println("[TTS] g_avatar_text_mutex creation failed");
  }

  // ===== PR1: TTS Task 起動 =====
  // ENABLE_SERVO の有無に関わらず起動する（servoTask と独立）
  g_ttsQueue = xQueueCreate(8, sizeof(TtsJob));
  if (!g_ttsQueue) {
    Serial.println("[TTS] xQueueCreate failed");
  } else {
    xTaskCreatePinnedToCore(
      ttsTaskFunc,
      "ttsTask",
      8192,             // speak_piper_http の内部スタックを考慮して大きめ
      nullptr,
      1,                // 優先度は低め
      &g_ttsTaskHandle,
      0                 // Core 0 に固定
    );
    Serial.println("[TTS] ttsTask started on Core 0");
  }

  Serial.printf("VAD設定読み込み: 閾値=%.1f, 最小発話=%d, 最大無音=%d, 有効=%s\n",
                global_vad_config.threshold,
                global_vad_config.min_speech,
                global_vad_config.max_silence,
                global_vad_config.enabled ? "Yes" : "No");  

  // ===== テスト用: 起動後に固定文Piperテストを1回だけ実行 =====
  if (g_test_run_direct_once) {
    delay(1000);
    runPiperDirectTest();
  }                  

}


void loop()
{
  M5.update();

  if (g_display_rotation_apply_requested) {
    g_display_rotation_apply_requested = false;
    applyDisplayRotation(g_display_rotation_pending, false);
  }

  if (g_vad_calibration_requested) {
      g_vad_calibration_requested = false;
      runVadCalibration();
  }

  // ===== PR2-C.1: Avatar 更新要求の反映 =====
  applyPendingAvatarUpdate();

  // DNS を処理（AP設定中だけ）
  if (g_apPortalRunning) {
    dnsServer.processNextRequest();
  }

  // 現在の値を取得（Wi-Fi接続状況でメッセージを切り替え）
  String response2;

  if (WiFi.status() == WL_CONNECTED) {
    // ◆ ローカルネットワークに接続済みのとき
    //    → 従来どおり Setting URL + Model を案内
    response2  = "Setting URL:" + WiFi.localIP().toString();
    response2 += " Model:" + LLM_MODEL_NAME;

  } else {
    // ◆ 未接続で、設定用APモードのとき
    //    → 「STACKCHAN-XXXX と 192.168.4.1 に接続してね」と案内
    uint32_t chipId = (uint32_t)(ESP.getEfuseMac() & 0xFFFFFF);
    String apSsid = "STACKCHAN-" + String(chipId, HEX);
    apSsid.toUpperCase();

    IPAddress apIP = WiFi.softAPIP();  // 通常 192.168.4.1

    response2  = "Connect to SSID: ";
    response2 += apSsid;
    response2 += " and ";
    response2 += apIP.toString();      // => "Connect to SSID: STACKCHAN-XXXX and 192.168.4.1"
  }
  String vad_info = "VAD: " + String(global_vad_config.enabled ? "ON" : "OFF") + 
                    " 閾値:" + String(global_vad_config.threshold, 0);

  // ボタンクリック回数制御
  if (M5.BtnA.wasPressed()) {
    uint32_t now = millis();
    
    // 初回クリックまたはタイムアウト後のクリック
    if (g_clickCount == 0 || (now - g_firstClickMs) > MULTI_CLICK_WINDOW_MS) {
      g_clickCount = 1;
      g_firstClickMs = now;
      Serial.println("[BTN] First click detected, waiting for more...");
    } else {
      // 連続クリック
      g_clickCount++;
      Serial.printf("[BTN] Click #%d detected\n", g_clickCount);
    }
  }

  // タイムアウト判定と実行
  if (g_clickCount > 0 && (millis() - g_firstClickMs) > MULTI_CLICK_WINDOW_MS) {
    switch (g_clickCount) {
      case 1: /* おしゃべり or Wi-Fi案内 */

        // ローカルWi-Fiに接続済みなら、従来どおり会話開始
        if (WiFi.status() == WL_CONNECTED) {
          setlang_messege();  // 言語別メッセージ切替
          start_talking();    // おしゃべり開始

        } else {
          // ★ 未接続の場合は会話をキャンセルし、
          //    Wi-Fi設定用の案内だけを表示する

          // 念のためAPモードを起動（すでに起動済みなら何もしない程度の処理）
          startWifiConfigPortal();

          // SSID: STACKCHAN-XXXX を生成
          uint32_t chipId = (uint32_t)(ESP.getEfuseMac() & 0xFFFFFF);
          String apSsid = "STACKCHAN-" + String(chipId, HEX);
          apSsid.toUpperCase();

          // SoftAP側IP（通常 192.168.4.1）
          IPAddress apIP = WiFi.softAPIP();

          // 表示メッセージ
          String msg = "Connect to SSID: ";
          msg += apSsid;
          msg += " and ";
          msg += apIP.toString();   // → "Connect to SSID: STACKCHAN-XXXX and 192.168.4.1"

          Serial.println("[BTN] 1 click: Wi-Fi config guide");
          Serial.println(msg);

          // 軽くビープ（好みで省略してOK）
          // initSpeakerOnce();
          // // Wi-Fiなし時のビープ
          // M5.Speaker.tone(1200, 100);
          ensureSpeakerActive();
          M5.Speaker.tone(500, 100);          
          delay(100);
          M5.Speaker.stop();
          M5.Speaker.stop();
          delay(100);
          // end()後に再開

          // アバター表示だけして会話は始めない
          avatar.setExpression(Expression::Happy);
          avatar.setSpeechText(msg.c_str());
          delay(6000);  // 少し長めに表示してあげる
          avatar.setSpeechText("");
          avatar.setExpression(Expression::Neutral);
        }

        break;

      case 2: /* 接続センサー実行 */
        Serial.println("[BTN] 2 click: Sensor");
        Serial.println("scanning I2C...");

        // ★一時診断用：detectSensors() の前に、I2C上に見えているアドレスを確認
        // scanI2CBus(I2C_UNIT, "Unit Wire");

        detectSensors();
        Serial.printf("[Detect] HEART:%d\n", hasHeart);

        if (hasHeart) runHeartMeasurement();

        if (!hasHeart) {
          Serial.println("[Detect] No known I2C sensor found.");
          ensureSpeakerActive();
          M5.Speaker.tone(800, 120, 0, false);
        }

        Serial.println("[Trigger] Sequence finished.");        

        break;
      case 3: /* 現在情報の取得 */
        ensureSpeakerActive();
        M5.Speaker.tone(500, 100);
        delay(100);
        M5.Speaker.stop();
        delay(100);

        avatar.setExpression(Expression::Happy);
        avatar.setSpeechText(response2.c_str());
        delay(7000);
        avatar.setSpeechText("");
        avatar.setExpression(Expression::Neutral);

        // String vad_info = "VAD: " + String(global_vad_config.enabled ? "ON" : "OFF") + 
        //                 " 閾値:" + String(global_vad_config.threshold, 0);

        Serial.println("=== VAD情報 ===");
        Serial.printf("状態: %s\n", global_vad_config.enabled ? "有効" : "無効");
        Serial.printf("閾値: %.1f\n", global_vad_config.threshold);

        break;
      default:
        Serial.println("default");
        break;
    }
    
    // リセット
    resetButtonState();
  }

  // ★展示会向け：Wi-Fi自己回復（10秒に1回だけ）
  {
    static uint32_t last = 0;
    if (millis() - last > 10000) {
      last = millis();
      if (WiFi.getMode() == WIFI_STA && WiFi.status() != WL_CONNECTED) {
        Serial.println("[WiFi] reconnect");
        WiFi.reconnect();
      }
    }
  }

  // ===== Phase 3-A: 会話中モーション状態遷移（wake → nod → idle → init）=====
#ifdef ENABLE_SERVO
  if (g_talk_motion_active) {
    // estop または録音中なら強制停止
    if (g_estop_active || g_audio_busy) {
      motion_force_stop();
    } else {
      uint32_t elapsed = millis() - g_talk_motion_start_ms;

      // ★ WAKE遷移(短) + WAKE静止 + NOD 700ms 経過後に idle へ切り替える
      uint32_t nod_visible_after = SERVO_WAKE_RETURN_MS + SERVO_WAKE_SETTLE_MS + 700;
      if (!g_talk_motion_nod_done && elapsed > nod_visible_after) {
        g_talk_motion_nod_done = true;
        servoRequestMotion(MOTION_IDLE);
        Serial.println("[TalkMotion] NOD done -> IDLE");
      }

      // TTS が終わっていて、すでに idle へ入っているなら終了
      if (!g_piper_tts_playing && g_talk_motion_nod_done) {
        motion_on_conversation_end();
      }
    }
  }
#endif

  // ===== Control Plane: 非同期録音の実行 =====
  if (g_record_requested && !g_record_done) {
    Serial.println("[API] async record start");
    executeAsyncRecord();
    Serial.println("[API] async record done");
  }

  // Web設定処理
  server.handleClient();  // Add for Web Setting

}