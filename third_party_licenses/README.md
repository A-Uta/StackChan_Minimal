# Third-party licenses

This directory contains license notices for third-party components used by StackChan Minimal.

StackChan Minimal のアプリケーション本体のソースコードは、リポジトリ直下の `LICENSE` に記載された Apache License 2.0 で公開しています。
ただし、PlatformIO でビルドされるファームウェアバイナリには、Arduino ESP32 Core や各種ライブラリなど、Apache License 2.0 以外のサードパーティコンポーネントが含まれる、またはリンクされる場合があります。

This notice is provided to clarify the licenses of third-party components that may be included in, linked with, or required to build firmware binaries distributed through WebInstaller or other binary distribution methods.

## Build configuration covered by this notice

The current firmware build is based on PlatformIO with the following configuration:

```ini
platform = espressif32@6.7.0
board = m5stack-atoms3
framework = arduino
```

Because `framework = arduino` is used, the firmware is built with Arduino ESP32 Core.

## Included or build-time third-party components

| Component | License | Purpose / role | Source / note |
|---|---|---|---|
| Arduino ESP32 Core | GNU LGPL v2.1 or later / LGPL-2.1-or-later | Arduino framework and ESP32 board support used by PlatformIO builds | https://github.com/espressif/arduino-esp32 |
| ESP-IDF components | Apache License 2.0 and third-party component licenses | Low-level ESP32 SDK components used underneath Arduino ESP32 Core | https://docs.espressif.com/projects/esp-idf/en/stable/esp32/COPYRIGHT.html |
| M5Unified | MIT License | M5Stack hardware abstraction, display, buttons, speaker, etc. | https://github.com/m5stack/M5Unified |
| M5UnitUnified | MIT License | Unified M5Stack Unit manager used for optional units | https://github.com/m5stack/M5UnitUnified |
| M5Unit-GESTURE | MIT License | Optional M5Stack Unit Gesture / PAJ7620U2 support | https://github.com/m5stack/M5Unit-GESTURE |
| ArduinoJson | MIT License | JSON parsing and serialization | https://github.com/bblanchon/ArduinoJson |
| ESP32WebServer | LGPL-2.1, depending on the exact package resolved by PlatformIO | Web configuration server used by the firmware | Check the exact package resolved in `.pio/libdeps/` or PlatformIO registry |
| MAX3010x Sensor Library | MIT License | MAX30100 optical pulse sensor driver | https://github.com/devxplained/MAX3010x-Sensor-Library |
| M5Stack-Avatar | See `lib/M5Stack-Avatar/LICENSE.txt` | Avatar face rendering | Bundled local library |

## License texts included in this directory

- `LGPL-2.1.txt` — GNU Lesser General Public License v2.1 text, provided for Arduino ESP32 Core and other LGPL-2.1 components.
- `Arduino-ESP32-Core-NOTICE.txt` — notice for Arduino ESP32 Core usage.
- `MAX3010x-Sensor-Library-LICENSE.txt` — MIT License text for MAX3010x Sensor Library.

Some bundled libraries may keep their original license file in their own directory, for example:

- `lib/M5Stack-Avatar/LICENSE.txt`

## Notes for firmware binaries and WebInstaller distribution

Firmware binaries distributed through WebInstaller are built artifacts. They may include or link with third-party components listed above.

The StackChan Minimal application source code remains licensed under Apache License 2.0, but that does not change the licenses of third-party components used to build the firmware.

Users who receive firmware binaries should also have access to:

- the corresponding StackChan Minimal source code,
- the PlatformIO build configuration,
- this third-party license notice,
- the applicable third-party license texts or upstream license locations.

## Optional external tools not included in this repository

StackChan Minimal can be configured to connect to external AI servers and tools such as:

- whisper.cpp
- llama.cpp
- LM Studio
- Ollama
- piper-plus
- VOICEVOX-compatible adapters

These external tools, servers, models, voice libraries, and installers are not included in this repository unless explicitly stated otherwise. Please install them separately from their official distribution sources and follow each project's license and terms.

VOICEVOX, VOICEVOX Engine, VOICEVOX Core, voice models, voice libraries, and VOICEVOX installers are not included in this repository.

If you use VOICEVOX-generated audio, please install VOICEVOX separately from its official distribution source and follow the VOICEVOX software terms and each voice library's terms.

Credit examples:

- VOICEVOX:ずんだもん
- VOICEVOX:四国めたん

StackChan Minimal is not affiliated with or endorsed by VOICEVOX or the rights holders of the voice libraries.

## 日本語メモ

- StackChan Minimal の自作アプリケーションコードは Apache License 2.0 です。
- WebInstaller で配布する `.bin` ファームウェアは、Arduino ESP32 Core などのサードパーティコンポーネントを含む、またはリンクしたビルド成果物です。
- そのため、リポジトリ全体を「Apache 2.0 だけ」と説明するのではなく、「本体コードは Apache 2.0、依存コンポーネントは各ライセンスに従う」と説明します。
- Arduino ESP32 Core を使うビルドでは、LGPL-2.1 系の表示とライセンス本文へのアクセスを用意します。
- PlatformIO がビルド時に取得するライブラリは、`platformio.ini` および `.pio/libdeps/` で実際に解決されたパッケージを確認してください。

This file is a license notice summary, not legal advice.
