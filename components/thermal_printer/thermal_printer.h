#pragma once

#include "esphome/core/helpers.h"

#include "esphome/components/display/display_buffer.h"
#include "esphome/components/uart/uart.h"

#include <cinttypes>
#include <queue>
#include <vector>

namespace esphome {
namespace thermal_printer {

enum BarcodeType {
  UPC_A = 0x41,
  UPC_E,
  EAN13,
  EAN8,
  CODE39,
  ITF,
  CODABAR,
  CODE93,
  CODE128,
};

class ThermalPrinterDisplay : public display::DisplayBuffer, public uart::UARTDevice {
 public:
  void setup() override;
  void loop() override;
  void update() override;

  void begin();
  void timeoutSet(unsigned long x);
  void wake();
  void reset();
  void setHeatConfig(uint8_t dots = 11, uint8_t time = 120, uint8_t interval = 40);
  void setDefault();
  void online();
  void justify(char value);
  void inverseOff();
  void doubleHeightOn();
  void doubleHeightOff();
  void doubleWidthOn();
  void doubleWidthOff();
  void strikeOn();
  void strikeOff();
  void boldOn();
  void boldOff();
  void setLineHeight(int val);
  void underlineOff();
  void setBarcodeHeight(uint8_t val = 50);
  void setSize(char value);
  void setCharset(uint8_t val = 0);
  void setCodePage(uint8_t val = 0);
  void feed(uint8_t x);
  void timeoutWait();
  void write_to_device_();

  size_t write(uint8_t c);

  // Display buffer
  int get_width_internal() override { return 8 * 58; };  // 58mm, 8 dots per mm
  int get_height_internal() override { return this->height_; };

  void set_height(int height) { this->height_ = height; }

  display::DisplayType get_display_type() override { return display::DisplayType::DISPLAY_TYPE_BINARY; }

  void print_text(std::string text, uint8_t font_size = 0);
  void new_line(uint8_t lines);
  void print_qrcode(std::string data);

  void print_barcode(std::string barcode, BarcodeType type);

 protected:
  void draw_absolute_pixel_internal(int x, int y, Color color) override;
  size_t get_buffer_length_() { return size_t(this->get_width_internal()) * size_t(this->get_height_internal()) / 8; }
  void queue_data_(std::vector<uint8_t> data);
  void queue_data_(const uint8_t *data, size_t size);
  void init_();

  std::queue<std::vector<uint8_t>> queue_{};
  int height_{0};

 private:
  uint8_t printMode,
      prevByte,       // Last character issued to printer
      column,         // Last horizontal column printed
      maxColumn,      // Page width (output 'wraps' at this point)
      charHeight,     // Height of characters, in 'dots'
      lineSpacing,    // Inter-line spacing (not line height), in dots
      barcodeHeight,  // Barcode height in dots, not including text
      maxChunkHeight,
      dtrPin;                // DTR handshaking pin (experimental)
  uint16_t firmware;         // Firmware version
  bool dtrEnabled;           // True if DTR pin set & printer initialized
  unsigned long resumeTime,  // Wait until micros() exceeds this before sending byte
      dotPrintTime,          // Time to print a single dot line, in microseconds
      dotFeedTime;           // Time to feed a single dot line, in microseconds
  void setPrintMode(uint8_t mask), unsetPrintMode(uint8_t mask), writePrintMode(), adjustCharValues(uint8_t printMode);
};

template<typename... Ts>
class ThermalPrinterPrintTextAction : public Action<Ts...>, public Parented<ThermalPrinterDisplay> {
 public:
  TEMPLATABLE_VALUE(std::string, text)
  TEMPLATABLE_VALUE(uint8_t, font_size)

  void play(Ts... x) override { this->parent_->print_text(this->text_.value(x...), this->font_size_.value(x...)); }
};

}  // namespace thermal_printer
}  // namespace esphome
