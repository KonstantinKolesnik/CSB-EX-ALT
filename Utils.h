#pragma once

#ifndef UTILS_H
#define UTILS_H

#include "Arduino.h"

inline String getChipId() {
#ifdef ARDUINO_ARCH_ESP32
  uint8_t mac[6] = {0};
  esp_efuse_mac_get_default(mac);

  const size_t mac_size = sizeof(mac);
  char result[2 * mac_size + 1];
  
  char *ptr = result;
  for (size_t i = 0; i < mac_size; i++) {
      ptr += sprintf(ptr, "%02X", mac[i]);
  }

  return String(result);
#endif // ARDUINO_ARCH_ESP32

  return "";
}

#endif