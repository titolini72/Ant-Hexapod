/**
 * @file LEDManager.hpp
 * @author Pierre-Yves Mordret <titolini72@gmail.com>
 * @brief Defines the LEDManager class for controlling LED patterns.
 * @version 1.0
 * @date 2026-01-02
 * @copyright Copyright (c) 2026 Pierre-Yves Mordret
 */

#ifndef LEDMANAGER_H
#define LEDMANAGER_H

#include <Arduino.h>
#include <initializer_list>

#define MODE_PATTERN_INFO 1
#define MODE_PATTERN_MAX (20 * MODE_PATTERN_INFO)
#define DELAY 10

#define BLINK_RUNNING { \
  100, 100, 100, 100, 0 \
}

#define BLINK_SOS { \
  20, 5, 20, 5, 20, 5, /* S -- */ \
  5, 5, 5, 5, 5, 5, /* O ... */ \
  20, 5, 20, 5, 20, 5, /* S -- */ \
  100, 0 \
}

#define BLINK_ERROR { \
  5, 5, 5, 5, /* .. */ \
  20, 5, 20, 5, /* -- */ \
  5, 5, 5, 5, /* .. */ \
  100, 0 \
}

class LEDManager {

public:
  LEDManager(unsigned char pin);

  void loop();
  void setMode(const std::initializer_list<unsigned int> &modePattern);
  void toggle(int On, int Off);

private:
  unsigned char _pin;
  unsigned int _modePattern[MODE_PATTERN_MAX + 1] = {};
  unsigned char _modeStep = 0;
  unsigned char _modeCounter = 0;
  unsigned long _now = 0;
};

#endif // LEDMANAGER_H
