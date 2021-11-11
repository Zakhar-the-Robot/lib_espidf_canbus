// *************************************************************************
//
// Copyright (c) 2021 Andrei Gramakov. All rights reserved.
//
// site:    https://agramakov.me
// e-mail:  mail@agramakov.me
//
// *************************************************************************

#pragma once

#include "esp_log.h"
#include "esp_err.h"

#define CHECK(func, error_msg)         \
    do                                 \
    {                                  \
        esp_err_t res = (func);        \
        if (res != ESP_OK)             \
        {                              \
            ESP_LOGE("%s", error_msg); \
        }                              \
    } while (0)
