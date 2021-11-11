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
#include "delay.h"

#define TAG "second"
#define TX_GPIO_NUM static_cast<gpio_num_t>(21)
#define RX_GPIO_NUM static_cast<gpio_num_t>(22)
#define DATA_PERIOD_MS 1500
#define ID_MASTER_DATA 0x0A4

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "-------------------------------------------------------");
    ESP_LOGI(TAG, "CAN start...");
    devCanBus.Start(TX_GPIO_NUM, RX_GPIO_NUM);


    ESP_LOGI(TAG, "-------------------------------------------------------");
    ESP_LOGI(TAG, "Setting up the periodic TX...");
    uint8_t tx_source = 0xb;
    CanBusDataPointers_t tx_data = {};
    tx_data.d0 = &tx_source;
    devCanBus.AddTxDescriptor(0xb5, tx_data);
    devCanBus.SetTxPeriod(3000);

    ESP_LOGI(TAG, "-------------------------------------------------------");
    ESP_LOGI(TAG, "Setting up the Store on receiveing...");
    uint8_t data_storage[8] = {1, 2, 3, 4, 5, 6, 7, 8};
    CanBusDataPointers_t store_data = {};
    store_data.d0 = &data_storage[0];
    store_data.d1 = &data_storage[1];
    store_data.d2 = &data_storage[2];
    store_data.d3 = &data_storage[3];
    store_data.d4 = &data_storage[4];
    store_data.d5 = &data_storage[5];
    store_data.d6 = &data_storage[6];
    store_data.d7 = &data_storage[7];
    devCanBus.AddStoreDescriptor(0xc5, store_data);
    
    ESP_LOGI(TAG, "-------------------------------------------------------");
    ESP_LOGI(TAG, "Setting up the response...");
    CanBusDataPointers_t resp_data = {};
    resp_data.d0 = &tx_source;
    resp_data.d2 = &data_storage[0];
    resp_data.d3 = &data_storage[1];
    resp_data.d4 = &data_storage[2];
    devCanBus.AddResponseDescriptor(0xa5, resp_data);



    while (devCanBus.IsStarted())
    {
        ESP_LOGI(TAG, "Storage: \t %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x",
                 data_storage[0],
                 data_storage[1],
                 data_storage[2],
                 data_storage[3],
                 data_storage[4],
                 data_storage[5],
                 data_storage[6],
                 data_storage[7]);
        tx_source++;
        delay(100);
    }
}
