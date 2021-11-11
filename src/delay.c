// *************************************************************************
//
// Copyright (c) 2021 Andrei Gramakov. All rights reserved.
//
// This file is licensed under the terms of the MIT license.
// For a copy, see: https://opensource.org/licenses/MIT
//
// site:    https://agramakov.me
// e-mail:  mail@agramakov.me
//
// *************************************************************************

#include "delay.h"

void delay(uint32_t ms) {
  (ms == 0U) ? (vTaskDelay(1)) : (vTaskDelay(to_ticks(ms)));
}
