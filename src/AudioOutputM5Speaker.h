#pragma once
#include <M5Unified.h>

class AudioOutputM5Speaker
{
public:
  AudioOutputM5Speaker(m5::Speaker_Class* m5sound, uint8_t virtual_sound_channel = 0)
  : _m5sound(m5sound), _virtual_ch(virtual_sound_channel) {}

  bool begin(void) { return true; }

  void SetGain(float gain) {
    if (gain < 0.0f) gain = 0.0f;
    if (gain > 2.0f) gain = 2.0f;
    _gain = gain;
  }

  bool ConsumeSample(int16_t sample[2])
  {
    if (_tri_buffer_index < tri_buf_size)
    {
      int32_t v = (int32_t)((float)sample[0] * _gain);
      if (v >  32767) v =  32767;
      if (v < -32768) v = -32768;

      // monoで1サンプル格納
      _tri_buffer[_tri_index][_tri_buffer_index] = (int16_t)v;
      _tri_buffer_index += 1;
      return true;
    }

    flush();
    return false;
  }

  void flush(void)
  {
    if (_tri_buffer_index)
    {
      uint32_t sr = M5.Speaker.config().sample_rate;
      if (sr == 0) sr = 16000;

      // ★monoとして再生（stereo=false）
      // length は “monoサンプル数”
      _m5sound->playRaw(_tri_buffer[_tri_index], _tri_buffer_index, sr,
                        /*stereo=*/false, /*repeat=*/1, _virtual_ch);

      _tri_index = (_tri_index < 2) ? (_tri_index + 1) : 0;
      _tri_buffer_index = 0;
      ++_update_count;
    }
  }

  bool stop(void)
  {
    flush();
    _m5sound->stop(_virtual_ch);
    for (size_t i = 0; i < 3; ++i) {
      memset(_tri_buffer[i], 0, tri_buf_size * sizeof(int16_t));
    }
    ++_update_count;
    return true;
  }

private:
  m5::Speaker_Class* _m5sound;
  uint8_t _virtual_ch;
  float _gain = 1.0f;

  static constexpr size_t tri_buf_size = 640;
  int16_t _tri_buffer[3][tri_buf_size];
  size_t _tri_buffer_index = 0;
  size_t _tri_index = 0;
  size_t _update_count = 0;
};