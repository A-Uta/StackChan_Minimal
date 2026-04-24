# StackChan Minimal

StackChan Minimal is a small AI companion robot project for M5Stack AtomS3R.

It connects an ESP32-based body to local AI services such as speech recognition, local LLMs, and text-to-speech servers.

This project is intended for demonstration, learning, and Maker activities.

## Features

- Wi-Fi configuration portal
- Local speech recognition via whisper.cpp
- Local LLM chat via OpenAI-compatible APIs
  - llama.cpp
  - LM Studio
  - Ollama
- Text-to-speech via piper-plus
- MAX30100 heart-rate / SpO2 reference value demo
- Optional servo motion

## Hardware

- M5Stack AtomS3R
- Atomic Voice Base / compatible audio base
- MAX30100 sensor module
- Optional servo setup

## AI server tools

This firmware does not include AI models or AI server binaries.

You need to run compatible local services separately:

- whisper.cpp for speech-to-text
- llama.cpp, LM Studio, or Ollama for local LLM chat
- piper-plus for text-to-speech

## Sensors

The current public version includes MAX30100 heart-rate / SpO2 reference value support.

ENV-Pro / BME688 environmental sensor support is not included in this public release.

## HR / SpO2 reference values

StackChan Minimal can read heart-rate and SpO2-like reference values from a MAX30100 sensor module.

These values are for demonstration and educational purposes only.

This project is not a medical device and must not be used for diagnosis, treatment, health monitoring, or healthcare decisions.

The SpO2 value is an uncalibrated reference value based on optical sensor readings. It may vary depending on finger placement, ambient light, sensor module differences, and motion.

## 日本語での注意

心拍数およびSpO2参考値は、展示・デモ・学習用途の目安です。

本プロジェクトは医療機器ではありません。診断、治療、健康管理、受診判断には使用できません。

SpO2参考値は未校正の光学センサー値から算出しており、指の置き方、周囲光、センサーモジュールの個体差、動きによって大きく変動することがあります。

現在の公開版には、ENV-Pro / BME688 環境センサー機能は含めていません。

## License

This project is licensed under the Apache License 2.0.

Third-party library licenses are listed in `third_party_licenses/`.