#pragma once

#include <Arduino.h>

#define DCC_PIN 17          // Output to your H-Bridge/Booster
#define ENABLE_PIN  18      // To Motor Shield ENABLE/EN
#define RAILCOM_RX_PIN 47   // Connected to 6N137 Pin 6

void setupDCC();
