#pragma once

#if __has_include_next(<BluetoothSerial.h>)
#  include_next <BluetoothSerial.h>
#else
#  include <Arduino.h>

// Stub BluetoothSerial for boards without Bluetooth support (e.g. ESP32-S2).
class BluetoothSerial : public Stream {
 public:
  bool begin(const String &) { return false; }
  int available() override { return 0; }
  int read() override { return -1; }
  int peek() override { return -1; }
  void flush() override {}
  size_t write(uint8_t) override { return 0; }
  using Print::write;
};
#endif
