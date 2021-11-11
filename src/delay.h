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

#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

    void delay(uint32_t ms);
    inline TickType_t to_ticks(uint32_t ms) { return ms / portTICK_PERIOD_MS; };
    inline uint32_t to_ms(TickType_t ticks) { return ticks * portTICK_PERIOD_MS; };

#ifdef __cplusplus
}
#endif
