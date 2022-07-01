// *************************************************************************
//
// Copyright (c) 2022 Andrei Gramakov. All rights reserved.
//
// This file is licensed under the terms of the MIT license.
// For a copy, see: https://opensource.org/licenses/MIT
//
// site:    https://agramakov.me
// e-mail:  mail@agramakov.me
//
// *************************************************************************

#pragma once

#include "esp_err.h"
#include "esp_log.h"

#define CHECK(func, error_msg)         \
    do {                               \
        esp_err_t res = (func);        \
        if (res != ESP_OK) {           \
            ESP_LOGE("%s", error_msg); \
        }                              \
    } while (0)
