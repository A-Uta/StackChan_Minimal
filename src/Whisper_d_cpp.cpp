#include <ArduinoJson.h>
#include "Whisper_d_cpp.h"



// パラメータ設定
const char* temperature = "0.0";
const char* temperature_inc = "0.2";
const char* response_format = "json";
// マルチパートフォームのバウンダリ
const char* boundary = "ArduinoFormBoundary";


// ★変更: port引数追加。connect()はTranscribe()側で行う（失敗状態を引きずらないため）
Whisper::Whisper(const char* server_ip, uint16_t port, const char* path)
  : client(), server(server_ip), port_(port), path_(path) {
  client.setTimeout(10000);
  Serial.printf("- Whisper server: %s:%u%s\n", server_ip, (unsigned)port, path);
}

Whisper::~Whisper() {
  client.stop();
}

String parseJsonResponse(String jsonString) {
  // JSONの始まりと終わりを検出
  int jsonStart = jsonString.indexOf('{');
  int jsonEnd = jsonString.lastIndexOf('}');
  
  if (jsonStart == -1 || jsonEnd == -1 || jsonStart >= jsonEnd) {
    Serial.println("有効なJSONデータが見つかりませんでした");
    Serial.println("生のレスポンスデータ:");
    Serial.println(jsonString);
    return "";
  }
  
  // 実際のJSON部分のみを抽出
  String actualJson = jsonString.substring(jsonStart, jsonEnd + 1);
  
  // JSONのパース（ArduinoJsonライブラリが必要）
  DynamicJsonDocument doc(4096);  // サイズを大きめに設定
  DeserializationError error = deserializeJson(doc, actualJson);
  
  if (error) {
    Serial.print("JSONのパースに失敗しました: ");
    Serial.println(error.c_str());
    return "";
  }
  
  // textフィールドだけを抽出して表示と返却
  if (doc.containsKey("text")) {
    String text = doc["text"].as<String>();
    Serial.println("\n--- 応答テキスト ---");
    Serial.println(text);
    return text;
  } else {
    Serial.println("JSONに 'text' フィールドが見つかりませんでした");
    Serial.println("JSON全体:");
    serializeJsonPretty(doc, Serial);
    return "";
  }
}

// language: "ja" / "en" / "zh" など（空文字列の場合はサーバー起動時の既定値を使用）
// detect_language: true の場合は language より優先して自動判定モードになる
String Whisper::Transcribe(AudioWhisper* audio,
                           const char* language,
                           bool detect_language) {
  // 既存接続があれば切断してから再接続（リトライ耐性）
  if (client.connected()) client.stop();

  Serial.printf("[Whisper] connecting to %s:%u\n", server.c_str(), (unsigned)port_);
  if (!client.connect(server.c_str(), port_)) {
    Serial.println("[Whisper] Connection failed!");
    return "エラーです。Whisperサーバーに接続できませんでした";
  }
  Serial.println("[Whisper] Connection success!");

  char boundary[64] = "------------------------";
  for (auto i = 0; i < 2; ++i) {
    ltoa(random(0x7fffffff), boundary + strlen(boundary), 16);
  }

  Serial.println("\n--- 録音音声データの送信 ---");
  
  // ヘッダーとフッターの文字列を構築
  String startBoundary = "--" + String(boundary) + "\r\n";
  String endBoundary = "\r\n--" + String(boundary) + "--\r\n";
    
  // ファイル部分のヘッダー
  String fileHeader = 
    "Content-Disposition: form-data; name=\"file\"; filename=\"speak.wav\"\r\n"
    "Content-Type: application/octet-stream\r\n\r\n";
  
  // ---------------------------------------------------------------
  // language フィールドの決定
  //   detect_language=true → "auto" を送り、サーバー側で自動判定
  //   language が空でない  → その言語コードをそのまま送る
  //   両方とも無効        → フィールドを省略（サーバー起動時の -l 既定値を使用）
  // ---------------------------------------------------------------
  String effectiveLang = "";
  if (detect_language) {
    effectiveLang = "auto";
  } else if (language && language[0] != '\0') {
    effectiveLang = String(language);
  }
  Serial.printf("[Whisper] STT language field: \"%s\" (detect_language=%s)\n",
                effectiveLang.c_str(), detect_language ? "true" : "false");

  // 他のフォームフィールド
  String otherFields = 
    startBoundary +
    "Content-Disposition: form-data; name=\"temperature\"\r\n\r\n" +
    String(temperature) + "\r\n" +
    startBoundary +
    "Content-Disposition: form-data; name=\"temperature_inc\"\r\n\r\n" +
    String(temperature_inc) + "\r\n" +
    startBoundary +
    "Content-Disposition: form-data; name=\"response_format\"\r\n\r\n" +
    String(response_format) + "\r\n";

  // language フィールドを追加（有効な値がある場合のみ）
  if (effectiveLang.length() > 0) {
    otherFields +=
      startBoundary +
      "Content-Disposition: form-data; name=\"language\"\r\n\r\n" +
      effectiveLang + "\r\n";
  }

  // file フィールドの先頭バウンダリ
  otherFields += startBoundary;

  // 全体のコンテンツの長さを計算
  size_t contentLength = 
    otherFields.length() +
    fileHeader.length() +
    audio->GetSize() +
    endBoundary.length();

  Serial.println("POSTリクエストを送信中...");
  
  // HTTPヘッダーの送信
  client.print("POST ");
  client.print(path_.c_str());
  client.println(" HTTP/1.1");
  client.print("Host: ");
  client.println(server + ":" + String(port_));
  client.println("User-Agent: ArduinoWiFi");
  client.print("Content-Type: multipart/form-data; boundary=");
  client.println(boundary);
  client.print("Content-Length: ");
  client.println(contentLength);
  client.println("Connection: close");
  client.println();
  
  // 他のフォームフィールドを先に送信
  client.print(otherFields);
  
  // ファイルヘッダーを送信
  client.print(fileHeader);

  // ファイルデータを送信
  Serial.println("ファイルデータを送信中...");
  
  auto ptr = audio->GetBuffer();
  auto remainings = audio->GetSize();
  while (remainings > 0) {
    // auto sz = (remainings > 512) ? 512 : remainings;   // Org
    auto sz = (remainings > 1024) ? 1024 : remainings; // OK
    // auto sz = (remainings > 2048) ? 2048 : remainings;
    client.write(ptr, sz);
    client.flush();
    remainings -= sz;
    ptr += sz;
  }
  client.flush();  
  
  Serial.print("ファイル送信完了: ");
  
  // 最後の境界を送信
  client.print(endBoundary);
  
  // レスポンスを待機
  Serial.println("サーバーからのレスポンスを待機中...");
  unsigned long startTime = millis();
  // const unsigned long maxWaitTime = 60000; // 最大待機時間: 60秒（長めに設定）
  // const unsigned long maxWaitTime = 30000; // 最大待機時間: 30秒（長めに設定）
  const unsigned long maxWaitTime = 15000; // 最大待機時間: 15秒（長めに設定）
  bool receivedResponse = false;
  String jsonResponse = "";
  bool isBody = false;
  bool emptyLineFound = false;
  bool headerReceived = false;
  int statusCode = 0;
  
  // デバッグ用 - 受信データを16進数で表示する関数
  auto printHex = [](const String& str) {
    Serial.print("HEX: ");
    for (int i = 0; i < str.length(); i++) {
      char c = str.charAt(i);
      Serial.print((int)c, HEX);
      Serial.print(" ");
    }
    Serial.println();
  };
  
  // データバッファがない場合の小さな待機時間
  const unsigned long noDataWaitTime = 100;
  unsigned long lastActivity = millis();
  
  // ボディデータ読み取りのための待機カウンター
  int bodyWaitCounter = 0;
  const int maxBodyWaitCycles = 100; // 最大待機サイクル数
  
  while (client.connected() || client.available()) {
    // 現在の時間を取得
    unsigned long currentTime = millis();
    
    // 最大待機時間を超えた場合はループを抜ける
    if (currentTime - startTime >= maxWaitTime) {
      Serial.println("タイムアウト: 最大待機時間(60秒)を超えました");
      break;
    }
    
    // データがない場合
    if (!client.available()) {
      // ヘッダーを受信済みかつボディ読み取り待機中の場合
      if (isBody && emptyLineFound && bodyWaitCounter < maxBodyWaitCycles) {
        delay(50); // ボディデータを待つためにより長く待機
        bodyWaitCounter++;
        
        // 待機状況を定期的に表示
        if (bodyWaitCounter % 20 == 0) {
          Serial.print("ボディデータ待機中... ");
          Serial.print(bodyWaitCounter);
          Serial.print("/");
          Serial.println(maxBodyWaitCycles);
        }
        continue;
      }
      
      // 一般的なデータ待機
      if (currentTime - lastActivity >= noDataWaitTime) {
        // ヘッダーを受信したがボディデータがまだない場合、もう少し待機
        if (headerReceived && !isBody) {
          // delay(500);  //org
          // delay(250);  //ok
          delay(100);
          Serial.println("ヘッダー受信済み、ボディ待機中...");
          continue;
        }
        
        // 応答の終了を確認（一定時間データが来ない場合）
        if (receivedResponse && currentTime - lastActivity >= 5000) {
          if (jsonResponse.length() > 0) {
            Serial.println("レスポンス完了（データ終了を検出）");
            break;
          } else {
            Serial.println("警告: ヘッダーは受信しましたが、ボディデータがありません");
            Serial.print("ステータスコード: ");
            Serial.println(statusCode);
            
            // ボディデータがない場合でも、ステータスコードが200なら成功とみなす
            if (statusCode == 200) {
              Serial.println("ステータスコード200を受信しましたが、ボディデータはありません");
              break;
            }
            
            // もう少し待機を続ける
            // delay(1000); //org
            delay(100);
            continue;
          }
        }
      }
      delay(10); // CPU負荷軽減のための短い待機
      continue;
    }
    
    // データが利用可能な場合、最後のアクティビティ時間を更新
    lastActivity = currentTime;
    receivedResponse = true;
    
    if (!isBody) {
      // ヘッダー処理モード
      String line = client.readStringUntil('\n');
      
      // 行の内容を表示（デバッグ用）
      Serial.print("受信: [");
      Serial.print(line.length());
      Serial.print("] ");
      Serial.println(line);
      
      // HTTPステータスコードの抽出
      if (line.startsWith("HTTP/1.")) {
        headerReceived = true;
        int spacePos = line.indexOf(' ');
        if (spacePos > 0) {
          statusCode = line.substring(spacePos + 1, spacePos + 4).toInt();
          Serial.print("HTTPステータスコード: ");
          Serial.println(statusCode);
        }
      }
      
      // 空行を検出 - HTTPヘッダーの終わり
      if (line.length() == 0 || (line.length() == 1 && line[0] == '\r')) {
        Serial.println("*** ヘッダー終了、ボディ開始 ***");
        isBody = true;
        emptyLineFound = true;
        
        // ボディデータが到着するまで少し待機
        // delay(1000); //org
        delay(100);
      } else {
        // ヘッダー行を処理
        Serial.print("< ");
        Serial.println(line);
      }
    } else {
      // ボディ処理モード - ここでJSONレスポンスを収集
      
      // ボディの読み取り方法を変更 - 一度に全部読み取る
      if (emptyLineFound) {
        // 最初のボディデータ読み取り時に全て取得を試みる
        emptyLineFound = false;
        
        Serial.println("ボディデータ読み取り開始...");
        
        // バッファに十分なデータが溜まるまで少し待つ
        // delay(1000); //org
        delay(100);
        
        // 利用可能なデータを全て読み取る
        while (client.available()) {
          jsonResponse += (char)client.read();
        }
        
        Serial.print("ボディデータ長: ");
        Serial.println(jsonResponse.length());
        
        if (jsonResponse.length() > 0) {
          // 先頭部分だけ表示（デバッグ用）
          int previewLength = min(100, (int)jsonResponse.length());
          Serial.print("ボディプレビュー: ");
          Serial.println(jsonResponse.substring(0, previewLength));
          
          if (jsonResponse[0] == '{' || jsonResponse[0] == '[') {
            Serial.println("JSONフォーマットを検出しました");
          } else {
            Serial.println("警告: JSONフォーマットではない可能性があります");
          }
        } else {
          Serial.println("警告: 初回ボディデータ読み取りが空です。データをさらに待機します...");
          // ボディデータがない場合は、emptyLineFoundをリセットしない
          emptyLineFound = true;
        }
      } else {
        // 追加データがあれば読み取る
        String additionalData = "";
        while (client.available()) {
          additionalData += (char)client.read();
        }
        
        if (additionalData.length() > 0) {
          jsonResponse += additionalData;
          Serial.print("追加データを受信: ");
          Serial.print(additionalData.length());
          Serial.print(" バイト (合計: ");
          Serial.print(jsonResponse.length());
          Serial.println(" バイト)");
        }
      }
    }
  }
  
  if (!receivedResponse) {
    Serial.println("指定したWhisperサーバーからの応答がありませんでした");
    return "エラーです。指定したWhisperサーバーからの応答がありませんでした";
  } else if (jsonResponse.length() > 0) {
    Serial.println("\n--- JSONレスポンス処理開始 ---");
    
    // JSONをパース
    String responseText = parseJsonResponse(jsonResponse);
    responseText.trim();  // 改行を除去
    
    // 戻り値を使った追加処理をここに記述できます
    if (responseText.length() > 0) {
      Serial.println("応答テキストを正常に取得しました");
    } else {
      Serial.println("応答テキストを取得できませんでした");
    }
    
    return responseText;
  }
  
  client.stop();
  Serial.println("接続終了");
  return "接続終了";
}