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

#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"
#include "canbus.hpp"
#include "millis.h" // from canbus lib
#include "delay.h"  // from canbus lib

#define TAG "first"
#define TX_GPIO_NUM static_cast<gpio_num_t>(21)
#define RX_GPIO_NUM static_cast<gpio_num_t>(22)
#define DATA_PERIOD_MS 5000
#define ID_2ND_STORE 0x0c5
#define ID_MASTER_RQ 0x0A5

static twai_message_t msg_to_store = {.identifier = ID_2ND_STORE,
                                    .data_length_code = 8,
                                    .data = {1, 2, 3, 4, 5, 6, 7, 8}};
static twai_message_t msg_for_response = {};

extern "C" void app_main(void)
{
    devCanBus.Start(1U, TX_GPIO_NUM, RX_GPIO_NUM);
    esp_err_t res;
    while (devCanBus.IsStarted())
    {
        ESP_LOGI(TAG,"-------------------------------------------------------");
        res = devCanBus.Send(msg_to_store);
        ESP_LOGI(TAG, "Sent data to store: id:0x%x, %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x -> res:0x%x",
                 msg_to_store.identifier,
                 msg_to_store.data[0],
                 msg_to_store.data[1],
                 msg_to_store.data[2],
                 msg_to_store.data[3],
                 msg_to_store.data[4],
                 msg_to_store.data[5],
                 msg_to_store.data[6],
                 msg_to_store.data[7],
                 res);
        msg_to_store.data[0]++;

        ESP_LOGI(TAG,"-------------------------------------------------------");
        ESP_LOGI(TAG, "Request data, 2nd aware of (id:0x%02x)", ID_MASTER_RQ);
        res = devCanBus.Request(ID_MASTER_RQ, msg_for_response, 100);
        if (res == ESP_OK)
        {
            ESP_LOGI(TAG, "Got requested data (id:0x%x, %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x)",
                     msg_for_response.identifier,
                     msg_for_response.data[0],
                     msg_for_response.data[1],
                     msg_for_response.data[2],
                     msg_for_response.data[3],
                     msg_for_response.data[4],
                     msg_for_response.data[5],
                     msg_for_response.data[6],
                     msg_for_response.data[7]);
        }
        else
        {
            ESP_LOGE(TAG, "Error! (res:0x%x)", res);
        }
        delay(DATA_PERIOD_MS);

        ESP_LOGI(TAG,"-------------------------------------------------------");
        ESP_LOGW(TAG, "Request data, 2nd not aware of (id:0x%02x)", ID_MASTER_RQ+1);
        res = devCanBus.Request(ID_MASTER_RQ + 1, msg_for_response, 100);
        if (res == ESP_OK)
        {
            ESP_LOGI(TAG, "Got requested data (id:0x%x, %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x)",
                     msg_for_response.identifier,
                     msg_for_response.data[0],
                     msg_for_response.data[1],
                     msg_for_response.data[2],
                     msg_for_response.data[3],
                     msg_for_response.data[4],
                     msg_for_response.data[5],
                     msg_for_response.data[6],
                     msg_for_response.data[7]);
        }
        else
        {
            ESP_LOGE(TAG, "Error! (res:0x%x)", res);
        }

        delay(DATA_PERIOD_MS);
    }
}
