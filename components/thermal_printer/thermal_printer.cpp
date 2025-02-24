// Inspired by Adafruit Thermal Printer library https://github.com/adafruit/Adafruit-Thermal-Printer-Library
// check TODOs, some are hardcoded for now.
#include "thermal_printer.h"

#include <cinttypes>

namespace esphome {
namespace thermal_printer {

static const char *const TAG = "thermal_printer";

// Though most of these printers are factory configured for 19200 baud
// operation, a few rare specimens instead work at 9600.  If so, change
// this constant.  This will NOT make printing slower!  The physical
// print and feed mechanisms are the bottleneck, not the port speed.
#define BAUDRATE 9600  // 19200 //!< How many bits per second the serial port should transfer //TODO

// ASCII codes used by some of the printer config commands:
#define ASCII_TAB '\t'  //!< Horizontal tab
#define ASCII_LF '\n'   //!< Line feed
#define ASCII_FF '\f'   //!< Form feed
#define ASCII_CR '\r'   //!< Carriage return
#define ASCII_DC2 18    //!< Device control 2
#define ASCII_ESC 27    //!< Escape
#define ASCII_FS 28     //!< Field separator
#define ASCII_GS 29     //!< Group separator

// Because there's no flow control between the printer and Arduino,
// special care must be taken to avoid overrunning the printer's buffer.
// Serial output is throttled based on serial speed as well as an estimate
// of the device's print and feed rates (relatively slow, being bound to
// moving parts and physical reality).  After an operation is issued to
// the printer (e.g. bitmap print), a timeout is set before which any
// other printer operations will be suspended.  This is generally more
// efficient than using delay() in that it allows the parent code to
// continue with other duties (e.g. receiving or decoding an image)
// while the printer physically completes the task.

/*!
 * Number of microseconds to issue one byte to the printer.  11 bits
 * (not 8) to accommodate idle, start and stop bits.  Idle time might
 * be unnecessary, but erring on side of caution here.
 */
#define BYTE_TIME (((11L * 1000000L) + (BAUDRATE / 2)) / BAUDRATE)

#define text_size 'S'   // Letter size - default is S, options are S for Small, M for Medium, L for Large
#define row_spacing 24  // Spacing between rows - default is 24, values range from minimum of 24 and maximum of 64

#define dtr 255
#define version 268

static const uint8_t SLEEP_OFF_CMD[] = {ASCII_ESC, '8', 0, 0};  // Sleep off (important!)
static const uint8_t INIT_CMD[] = {ASCII_ESC, '@'};             // Init command

static const uint8_t TAB_STOP_CMD[] = {ASCII_ESC, 'D'};       // Set tab stops...
static const uint8_t TAB_STOP_CMD_4_COLS[] = {4, 8, 12, 16};  // every four columns
static const uint8_t TAB_STOP_CMD_STOP[] = {20, 24, 28, 0};   // 0 marks end-of-list.

static const uint8_t PRINT_SETTINGS_CMD[] = {ASCII_ESC, '7'};  // Esc 7 (print settings)

static const uint8_t ONLINE_CMD[] = {ASCII_ESC, '=', 1};
static const uint8_t UNDERLINE_OFF_CMD[] = {ASCII_ESC, '-', 0};
static const uint8_t INVERSE_OFF_CMD[] = {ASCII_GS, 'B', 0};

// === Character commands ===
#define FONT_MASK (1 << 0)  //!< Select character font A or B
#define INVERSE_MASK \
  (1 << 1)                           //!< Turn on/off white/black reverse printing mode. Not in 2.6.8
                                     //!< firmware (see inverseOn())
#define UPDOWN_MASK (1 << 2)         //!< Turn on/off upside-down printing mode
#define BOLD_MASK (1 << 3)           //!< Turn on/off bold printing mode
#define DOUBLE_HEIGHT_MASK (1 << 4)  //!< Turn on/off double-height printing mode
#define DOUBLE_WIDTH_MASK (1 << 5)   //!< Turn on/off double-width printing mode
#define STRIKE_MASK (1 << 6)         //!< Turn on/off deleteline mode

/* stuff from jesse's m5stack_printer component */
/*static const uint8_t ESC = 0x1B;
static const uint8_t GS = 0x1D;

static const uint8_t INIT_PRINTER_CMD[] = {ESC, 0x40};
static const uint8_t BAUD_RATE_9600_CMD[] = {ESC, '#', '#', 'S', 'B', 'D', 'R', 0x80, 0x25, 0x00, 0x00};
static const uint8_t BAUD_RATE_115200_CMD[] = {ESC, '#', '#', 'S', 'B', 'D', 'R', 0x00, 0xC2, 0x01, 0x00};

static const uint8_t FONT_SIZE_CMD[] = {GS, '!'};
static const uint8_t FONT_SIZE_RESET_CMD[] = {ESC, 0x14};

static const uint8_t QR_CODE_SET_CMD[] = {GS, 0x28, 0x6B, 0x00, 0x00, 0x31, 0x50, 0x30};
static const uint8_t QR_CODE_PRINT_CMD[] = {GS, 0x28, 0x6B, 0x03, 0x00, 0x31, 0x51, 0x30, 0x00};

static const uint8_t BARCODE_ENABLE_CMD[] = {GS, 0x45, 0x43, 0x01};
static const uint8_t BARCODE_DISABLE_CMD[] = {GS, 0x45, 0x43, 0x00};
static const uint8_t BARCODE_PRINT_CMD[] = {GS, 0x6B};

static const uint8_t BYTES_PER_LOOP = 120;
*/

// setup()
void ThermalPrinterDisplay::setup() {
  this->init_internal_(this->get_buffer_length_());

  this->begin();

  this->setDefault();
  this->setSize(text_size);
  this->setLineHeight(row_spacing);

  this->feed(1);

  // old stuff from jesse's component
  //  this->write_array(BAUD_RATE_115200_CMD, sizeof(BAUD_RATE_115200_CMD));
  //  delay(10);
  //  this->parent_->set_baud_rate(115200);
  //  this->parent_->load_settings();
  //  delay(10);

  // this->write_array(INIT_PRINTER_CMD, sizeof(INIT_PRINTER_CMD));
}

// maps to Adafruit_Thermal::begin()
void ThermalPrinterDisplay::begin() {
  firmware = version;
  // The printer can't start receiving data immediately upon power up --
  // it needs a moment to cold boot and initialize.  Allow at least 1/2
  // sec of uptime before printer can receive data.
  this->timeoutSet(500000L);

  this->wake();
  this->reset();

  this->setHeatConfig();

  // Enable DTR pin if requested
  if (dtr < 255) {
    // pinMode(dtrPin, INPUT_PULLUP); //TODO: implement pinMode
    // this->write_byte(ASCII_GS, 'a', (1 << 5));
    dtrEnabled = true;
  }

  dotPrintTime = 30000;  // See comments near top of file for
  dotFeedTime = 2100;    // an explanation of these values.
  maxChunkHeight = 255;
}

// This method sets the estimated completion time for a just-issued task.
void ThermalPrinterDisplay::timeoutSet(unsigned long x) {
  if (!dtrEnabled)
    resumeTime = micros() + x;
}

// Wake the printer from a low-energy state.
void ThermalPrinterDisplay::wake() {
  timeoutSet(0);          // Reset timeout counter
  this->write_byte(255);  // Wake
  if (firmware >= 264) {
    delay(50);
    // Sleep off (important!)
    this->write_array(SLEEP_OFF_CMD, sizeof(SLEEP_OFF_CMD));
  } else {
    // Datasheet recommends a 50 mS delay before issuing further commands,
    // but in practice this alone isn't sufficient (e.g. text size/style
    // commands may still be misinterpreted on wake).  A slightly longer
    // delay, interspersed with NUL chars (no-ops) seems to help.
    for (uint8_t i = 0; i < 10; i++) {
      this->write_byte(0);
      timeoutSet(10000L);
    }
  }
}

// Reset printer to default state.
void ThermalPrinterDisplay::reset() {
  // Init command
  this->write_array(INIT_CMD, sizeof(INIT_CMD));
  prevByte = '\n';  // Treat as if prior line is blank
  column = 0;
  maxColumn = 32;
  charHeight = 24;
  lineSpacing = 6;
  barcodeHeight = 50;

  if (firmware >= 264) {
    // Configure tab stops on recent printers
    this->write_array(TAB_STOP_CMD, sizeof(TAB_STOP_CMD));                // Set tab stops...
    this->write_array(TAB_STOP_CMD_4_COLS, sizeof(TAB_STOP_CMD_4_COLS));  // ...every 4 columns,
    this->write_array(TAB_STOP_CMD_STOP, sizeof(TAB_STOP_CMD_STOP));      // 0 marks end-of-list.
  }
}

// ESC 7 n1 n2 n3 Setting Control Parameter Command
// n1 = "max heating dots" 0-255 -- max number of thermal print head
//      elements that will fire simultaneously.  Units = 8 dots (minus 1).
//      Printer default is 7 (64 dots, or 1/6 of 384-dot width), this code
//      sets it to 11 (96 dots, or 1/4 of width).
// n2 = "heating time" 3-255 -- duration that heating dots are fired.
//      Units = 10 us.  Printer default is 80 (800 us), this code sets it
//      to value passed (default 120, or 1.2 ms -- a little longer than
//      the default because we've increased the max heating dots).
// n3 = "heating interval" 0-255 -- recovery time between groups of
//      heating dots on line; possibly a function of power supply.
//      Units = 10 us.  Printer default is 2 (20 us), this code sets it
//      to 40 (throttled back due to 2A supply).
// More heating dots = more peak current, but faster printing speed.
// More heating time = darker print, but slower printing speed and
// possibly paper 'stiction'.  More heating interval = clearer print,
// but slower printing speed.
void ThermalPrinterDisplay::setHeatConfig(uint8_t dots, uint8_t time, uint8_t interval) {
  this->write_array(PRINT_SETTINGS_CMD, sizeof(PRINT_SETTINGS_CMD));  // Esc 7 (print settings)
  this->write_byte(dots);
  this->write_byte(time);
  this->write_byte(interval);  // Heating dots, heat time, heat interval
}

// Reset text formatting parameters.
void ThermalPrinterDisplay::setDefault() {
  this->online();
  this->justify('L');
  this->inverseOff();
  this->doubleHeightOff();
  this->setLineHeight(30);
  this->boldOff();
  this->underlineOff();
  this->setBarcodeHeight(50);
  this->setSize('s');
  this->setCharset();
  this->setCodePage();
}

// Take the printer back online. Subsequent print commands will be obeyed.
void ThermalPrinterDisplay::online() { this->write_array(ONLINE_CMD, sizeof(ONLINE_CMD)); }

void ThermalPrinterDisplay::justify(char value) {
  uint8_t pos = 0;

  switch (toupper(value)) {
    case 'L':
      pos = 0;
      break;
    case 'C':
      pos = 1;
      break;
    case 'R':
      pos = 2;
      break;
  }

  uint8_t justify_arr[] = {ASCII_ESC, 'a', pos};
  this->write_array(justify_arr, sizeof(justify_arr));
}

void ThermalPrinterDisplay::inverseOff() {
  if (firmware >= 268) {
    this->write_array(INVERSE_OFF_CMD, sizeof(INVERSE_OFF_CMD));
  } else {
    unsetPrintMode(INVERSE_MASK);
  }
}

void ThermalPrinterDisplay::doubleHeightOn() { setPrintMode(DOUBLE_HEIGHT_MASK); }

void ThermalPrinterDisplay::doubleHeightOff() { unsetPrintMode(DOUBLE_HEIGHT_MASK); }

void ThermalPrinterDisplay::doubleWidthOn() { setPrintMode(DOUBLE_WIDTH_MASK); }

void ThermalPrinterDisplay::doubleWidthOff() { unsetPrintMode(DOUBLE_WIDTH_MASK); }

void ThermalPrinterDisplay::strikeOn() { setPrintMode(STRIKE_MASK); }

void ThermalPrinterDisplay::strikeOff() { unsetPrintMode(STRIKE_MASK); }

void ThermalPrinterDisplay::boldOn() { setPrintMode(BOLD_MASK); }

void ThermalPrinterDisplay::boldOff() { unsetPrintMode(BOLD_MASK); }

void ThermalPrinterDisplay::underlineOff() { this->write_array(UNDERLINE_OFF_CMD, sizeof(UNDERLINE_OFF_CMD)); }

void ThermalPrinterDisplay::setLineHeight(int val) {
  if (val < 24)
    val = 24;
  lineSpacing = val - 24;

  // The printer doesn't take into account the current text height
  // when setting line height, making this more akin to inter-line
  // spacing.  Default line spacing is 30 (char height of 24, line
  // spacing of 6).
  uint8_t line_height_arr[] = {ASCII_ESC, '3', (uint8_t) val};
  this->write_array(line_height_arr, sizeof(line_height_arr));
}

void ThermalPrinterDisplay::setBarcodeHeight(uint8_t val) {  // Default is 50
  if (val < 1)
    val = 1;
  barcodeHeight = val;
  uint8_t barcode_height_arr[] = {ASCII_GS, 'h', val};
  this->write_array(barcode_height_arr, sizeof(barcode_height_arr));
}

void ThermalPrinterDisplay::setSize(char value) {
  uint8_t size;

  switch (toupper(value)) {
    default:  // Small: standard width and height
      // size = 0x00;
      // charHeight = 24;
      // maxColumn = 32;
      doubleWidthOff();
      doubleHeightOff();
      break;
    case 'M':  // Medium: double height
      // size = 0x01;
      // charHeight = 48;
      // maxColumn = 32;
      doubleHeightOn();
      doubleWidthOff();
      break;
    case 'L':  // Large: double width and height
      // size = 0x11;
      // charHeight = 48;
      // maxColumn = 16;
      doubleHeightOn();
      doubleWidthOn();
      break;
  }

  // writeBytes(ASCII_GS, '!', size);
  // prevByte = '\n'; // Setting the size adds a linefeed
}

// Alters some chars in ASCII 0x23-0x7E range; see datasheet
void ThermalPrinterDisplay::setCharset(uint8_t val) {
  if (val > 15)
    val = 15;
  uint8_t charset_arr[] = {ASCII_ESC, 'R', val};
  this->write_array(charset_arr, sizeof(charset_arr));
}

// Selects alt symbols for 'upper' ASCII values 0x80-0xFF
void ThermalPrinterDisplay::setCodePage(uint8_t val) {
  if (val > 47)
    val = 47;
  uint8_t codepage_arr[] = {ASCII_ESC, 't', val};
  this->write_array(codepage_arr, sizeof(codepage_arr));
}

// Feeds by the specified number of lines
void ThermalPrinterDisplay::feed(uint8_t x) {
  if (firmware >= 264) {
    uint8_t feed_arr[] = {ASCII_ESC, 'd', x};
    this->write_array(feed_arr, sizeof(feed_arr));
    timeoutSet(dotFeedTime * charHeight);
    prevByte = '\n';
    column = 0;
  } else {
    while (x--)
      write('\n');  // Feed manually; old firmware feeds excess lines
  }
}
void ThermalPrinterDisplay::setPrintMode(uint8_t mask) {
  printMode |= mask;
  writePrintMode();
  adjustCharValues(printMode);
  // charHeight = (printMode & DOUBLE_HEIGHT_MASK) ? 48 : 24;
  // maxColumn = (printMode & DOUBLE_WIDTH_MASK) ? 16 : 32;
}

void ThermalPrinterDisplay::unsetPrintMode(uint8_t mask) {
  printMode &= ~mask;
  writePrintMode();
  adjustCharValues(printMode);
  // charHeight = (printMode & DOUBLE_HEIGHT_MASK) ? 48 : 24;
  // maxColumn = (printMode & DOUBLE_WIDTH_MASK) ? 16 : 32;
}

void ThermalPrinterDisplay::writePrintMode() {
  uint8_t printModeArr[] = {ASCII_ESC, '!', printMode};
  this->write_array(printModeArr, sizeof(printModeArr));
}

// The underlying method for all high-level printing (e.g. println()).
// The inherited Print class handles the rest!
size_t ThermalPrinterDisplay::write(uint8_t c) {
  if (c != 13) {  // Strip carriage returns
    timeoutWait();
    char str[2] = {static_cast<char>(c), '\0'};
    this->write_str(str);
    unsigned long d = BYTE_TIME;
    if ((c == '\n') || (column == maxColumn)) {                               // If newline or wrap
      d += (prevByte == '\n') ? ((charHeight + lineSpacing) * dotFeedTime) :  // Feed line
               ((charHeight * dotPrintTime) + (lineSpacing * dotFeedTime));   // Text line
      column = 0;
      c = '\n';  // Treat wrap as newline on next pass
    } else {
      column++;
    }
    timeoutSet(d);
    prevByte = c;
  }

  return 1;
}

// This function waits (if necessary) for the prior task to complete.
void ThermalPrinterDisplay::timeoutWait() {
  if (dtrEnabled) {
    /*while (digitalRead(dtrPin) == HIGH) {
      yield();
    };*/
  } else {
    while ((long) (micros() - resumeTime) < 0L) {
      yield();
    };  // (syntax is rollover-proof)
  }
}

void ThermalPrinterDisplay::adjustCharValues(uint8_t printMode) {
  uint8_t charWidth;
  if (printMode & FONT_MASK) {
    // FontB
    charHeight = 17;
    charWidth = 9;
  } else {
    // FontA
    charHeight = 24;
    charWidth = 12;
  }
  // Double Width Mode
  if (printMode & DOUBLE_WIDTH_MASK) {
    maxColumn /= 2;
    charWidth *= 2;
  }
  // Double Height Mode
  if (printMode & DOUBLE_HEIGHT_MASK) {
    charHeight *= 2;
  }
  maxColumn = (384 / charWidth);
}

//---stuff from Jesse's m5stack_printer component
void ThermalPrinterDisplay::print_text(std::string text, uint8_t font_size) {
  this->init_();
  /*font_size = clamp<uint8_t>(font_size, 0, 7);
  this->write_array(FONT_SIZE_CMD, sizeof(FONT_SIZE_CMD));
  this->write_byte(font_size | (font_size << 4));*/

  for (char c : text) {
    this->write(static_cast<uint8_t>(c));
  }

  /*this->write_array(FONT_SIZE_RESET_CMD, sizeof(FONT_SIZE_RESET_CMD));*/
}

void ThermalPrinterDisplay::new_line(uint8_t lines) {
  for (uint8_t i = 0; i < lines; i++) {
    this->write_byte('\n');
  }
}

void ThermalPrinterDisplay::print_qrcode(std::string data) {
  this->init_();

  /*size_t len;
  uint8_t len_low, len_high;
  len = data.length() + 3;
  len_low = len & 0xFF;
  len_high = len >> 8;

  uint8_t qr_code_cmd[sizeof(QR_CODE_SET_CMD)];
  memcpy(qr_code_cmd, QR_CODE_SET_CMD, sizeof(QR_CODE_SET_CMD));
  qr_code_cmd[3] = len_low;
  qr_code_cmd[4] = len_high;
  this->write_array(qr_code_cmd, sizeof(qr_code_cmd));
  this->write_str(data.c_str());
  this->write_byte(0x00);

  this->write_array(QR_CODE_PRINT_CMD, sizeof(QR_CODE_PRINT_CMD));
  */
}

void ThermalPrinterDisplay::print_barcode(std::string barcode, BarcodeType type) {
  this->init_();

  /*this->write_array(BARCODE_ENABLE_CMD, sizeof(BARCODE_ENABLE_CMD));

  this->write_array(BARCODE_PRINT_CMD, sizeof(BARCODE_PRINT_CMD));
  this->write_byte(type);
  this->write_byte(barcode.length());
  this->write_str(barcode.c_str());
  this->write_byte(0x00);

  this->write_array(BARCODE_DISABLE_CMD, sizeof(BARCODE_DISABLE_CMD));*/
}

void ThermalPrinterDisplay::queue_data_(std::vector<uint8_t> data) {
  /*for (size_t i = 0; i < data.size(); i += BYTES_PER_LOOP) {
    std::vector<uint8_t> chunk(data.begin() + i, data.begin() + std::min(i + BYTES_PER_LOOP, data.size()));
    this->queue_.push(chunk);
  }*/
}
void ThermalPrinterDisplay::queue_data_(const uint8_t *data, size_t size) {
  /*for (size_t i = 0; i < size; i += BYTES_PER_LOOP) {
    size_t chunk_size = std::min(i + BYTES_PER_LOOP, size) - i;
    std::vector<uint8_t> chunk(data + i, data + i + chunk_size);
    this->queue_.push(chunk);
  }*/
}

void ThermalPrinterDisplay::loop() {
  if (this->queue_.empty()) {
    return;
  }

  std::vector<uint8_t> data = this->queue_.front();
  this->queue_.pop();
  this->write_array(data.data(), data.size());
}

static uint16_t count = 0;

void ThermalPrinterDisplay::update() {
  this->do_update_();
  this->write_to_device_();
  ESP_LOGD(TAG, "count: %d;", count);
  count = 0;
}

void ThermalPrinterDisplay::write_to_device_() {
  if (this->buffer_ == nullptr) {
    return;
  }

  uint8_t header[] = {0x1D, 0x76, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00};

  uint16_t width = this->get_width() / 8;
  uint16_t height = this->get_height();

  header[3] = 0;  // Mode
  header[4] = width & 0xFF;
  header[5] = (width >> 8) & 0xFF;
  header[6] = height & 0xFF;
  header[7] = (height >> 8) & 0xFF;

  this->queue_data_(header, sizeof(header));
  this->queue_data_(this->buffer_, this->get_buffer_length_());
}

void ThermalPrinterDisplay::draw_absolute_pixel_internal(int x, int y, Color color) {
  if (this->buffer_ == nullptr) {
    ESP_LOGW(TAG, "Buffer is null");
    return;
  }
  if (x < 0 || y < 0 || x >= this->get_width_internal() || y >= this->get_height_internal()) {
    ESP_LOGW(TAG, "Invalid pixel: x=%d, y=%d", x, y);
    return;
  }
  uint8_t width = this->get_width_internal() / 8;
  uint16_t index = x / 8 + y * width;
  uint8_t bit = x % 8;
  if (color.is_on()) {
    this->buffer_[index] |= 1 << (7 - bit);
  } else {
    this->buffer_[index] &= ~(1 << (7 - bit));
  }
  count++;
}

}  // namespace thermal_printer
}  // namespace esphome
