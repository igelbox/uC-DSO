#pragma once

#include <HardwareSerial.h>

struct SerIO {
  SerIO() {}
  SerIO(SerIO &from) { from.hollow = true; }
  ~SerIO() {
    if (!hollow)
      Serial.println();
  }
  SerIO &LV() noexcept { return *this; }

private:
  bool hollow = false;
};
#define Serio SerIO().LV()

SerIO &operator<<(SerIO &serio, void *value) {
  Serial.printf("%p", value);
  return serio;
}

SerIO &operator<<(SerIO &serio, const std::string &value) {
  Serial.print(value.c_str());
  return serio;
}

template <typename T> SerIO &operator<<(SerIO &serio, const T &printable) {
  Serial.print(printable);
  return serio;
}
