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

#include "canbus.hpp"
#include "delay.h" // from canbus lib
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "millis.h" // from canbus lib
#include <stdio.h>
#include <stdlib.h>

#define TAG "first"
#define TX_GPIO_NUM static_cast<gpio_num_t>(21)
#define RX_GPIO_NUM static_cast<gpio_num_t>(22)
#define DATA_PERIOD_MS 2000
#define DEVICE_ID_2ND 0x2
#define DEVICE_ID_3RD 0x3

static uint8_t cmd[] = { 0xA0, 2, 3, 4, 5, 6, 7, 8 };

void RxCallback(CanBus* dev, twai_message_t& rMsg)
{
    ESP_LOGI(TAG, "[<] Message 7...0: \tID:%03x - %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x", rMsg.identifier,rMsg.data[7], rMsg.data[6],
             rMsg.data[5], rMsg.data[4], rMsg.data[3], rMsg.data[2], rMsg.data[1], rMsg.data[0]);
}

extern "C" void app_main(void)
{
    // esp_log_level_set("CAN", ESP_LOG_DEBUG);

    devCanBus.Start(1U, TX_GPIO_NUM, RX_GPIO_NUM);
    esp_err_t res;
    devCanBus.SetCallbackRx(RxCallback);
    while (devCanBus.IsStarted()) {
        res = devCanBus.SendCmdMsg(DEVICE_ID_2ND, cmd[7]++, cmd[6]++, cmd[5]++, cmd[4]++, cmd[3]++, cmd[2]++,
                                   cmd[1]++, cmd[0]++);
        ESP_LOGW(TAG, "[-> Cmd: target_dev_id:0x%x, %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x -> res:0x%x",
                 DEVICE_ID_2ND, cmd[7], cmd[6], cmd[5], cmd[4], cmd[3], cmd[2], cmd[1], cmd[0], res);
        delay(DATA_PERIOD_MS);

        res = devCanBus.SendCmdMsg(DEVICE_ID_3RD, cmd[7]++, cmd[6]++, cmd[5]++, cmd[4]++, cmd[3]++, cmd[2]++,
                                   cmd[1]++, cmd[0]++);;
        ESP_LOGI(TAG, "[-> Cmd: target_dev_id:0x%x, %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x -> res:0x%x",
                 DEVICE_ID_3RD, cmd[7], cmd[6], cmd[5], cmd[4], cmd[3], cmd[2], cmd[1], cmd[0], res);
        delay(DATA_PERIOD_MS);
    }
}
