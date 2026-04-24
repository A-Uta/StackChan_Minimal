// Copyright (c) Shinya Ishikawa. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full
// license information.

#ifndef BALLOON_H_
#define BALLOON_H_
#define LGFX_USE_V1
#include <M5Unified.h>
#include "DrawContext.h"
#include "Drawable.h"

#ifndef ARDUINO
#include <string>
typedef std::string String;
#endif  // ARDUINO

const int16_t TEXT_HEIGHT = 8;
// const int16_t TEXT_SIZE = 2;
const int16_t TEXT_SIZE = 4; // Adjust for SSD1306
const int16_t MIN_WIDTH = 40;
// const int cx = 240;
const int cx = 210; // Adjust for SSD1306
// const int cy = 220;
const int cy = 210; // Adjust for SSD1306
static int tid = 0; // Add for scroll

namespace m5avatar {
class Balloon final : public Drawable {
 public:
  // constructor
  Balloon() = default;
  ~Balloon() = default;
  Balloon(const Balloon &other) = default;
  Balloon &operator=(const Balloon &other) = default;
  void draw(M5Canvas *spi, BoundingRect rect,
            DrawContext *drawContext) override {
    String text = drawContext->getspeechText();
    const lgfx::IFont *font = drawContext->getSpeechFont();
    if (text.length() == 0) {
      return;
    }
    ColorPalette* cp = drawContext->getColorPalette();
    uint16_t primaryColor = cp->get(COLOR_BALLOON_FOREGROUND);
    uint16_t backgroundColor = cp->get(COLOR_BALLOON_BACKGROUND);
    M5.Lcd.setTextSize(TEXT_SIZE);
    M5.Lcd.setTextDatum(MC_DATUM);
    spi->setTextSize(TEXT_SIZE);
//    spi->setTextColor(primaryColor, backgroundColor);
    spi->setTextColor(backgroundColor, primaryColor); // Change for scroll
    spi->setTextDatum(MC_DATUM);
    M5.Lcd.setFont(font);
    int textWidth = M5.Lcd.textWidth(text.c_str());
    int textHeight = TEXT_HEIGHT * TEXT_SIZE;
// Add for scroll
    int textLength = text.length();
// Comment for scroll
//    spi->fillEllipse(cx - 20, cy,textWidth + 2, textHeight * 2 + 2,
//                     primaryColor);
//    spi->fillTriangle(cx - 62, cy - 42, cx - 8, cy - 10, cx - 41, cy - 8,
//                      primaryColor);
//    spi->fillEllipse(cx - 20, cy, textWidth, textHeight * 2,
//                     backgroundColor);
//    spi->fillTriangle(cx - 60, cy - 40, cx - 10, cy - 10, cx - 40, cy - 10,
//                      backgroundColor);
//    spi->drawString(text, cx - textWidth / 6 - 15, cy, font);  // Continue printing from new x position

// Add for scroll
    static int wait = 0;
    if (textWidth < spi->width()){
      spi->drawString(text, cx - textWidth / 6 - 15, cy, font);  // Continue printing from new x position
      tid = 0;
      wait = 0;
    } else {
      spi->setTextDatum(ML_DATUM);
      spi->drawString(&text[tid], 0, cy, font);  // Continue printing from new x position
      if (--wait < 0)
      {
        if(text[tid] < 0x80){
          tid++;
        }else if(text[tid] < 0xe0) {
          tid += 2;
        }else if(text[tid] >= 0xe0) {
          tid += 3;
        }
        if (tid >= textLength) { tid = 0; }
        if(tid == 0){
          // wait = 50;
          wait = 10; // Adjust for SSD1306
        }else{
          // wait = 10;
          wait = 1; // Adjust for SSD1306
        }
      }
    }

  }
};

}  // namespace m5avatar

#endif  // BALLOON_H_
