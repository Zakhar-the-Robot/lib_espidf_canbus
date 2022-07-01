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
#include "delay.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>
#include <stdlib.h>

#define TAG "second"
#define TX_GPIO_NUM static_cast<gpio_num_t>(21)
#define RX_GPIO_NUM static_cast<gpio_num_t>(22)
#define DATA_PERIOD_MS 1500
#define ID_MASTER_DATA 0x0A4

static uint8_t data_storage[8] = { 1, 2, 3, 4, 5, 6, 7, 8 };
static uint8_t counter = 0;

void RxCallback(CanBus* dev, twai_message_t& rMsg)
{
    ESP_LOGI(TAG, "[<] Message 7...0: \tID:%03x - %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x", rMsg.identifier,
             rMsg.data[7], rMsg.data[6], rMsg.data[5], rMsg.data[4], rMsg.data[3], rMsg.data[2], rMsg.data[1],
             rMsg.data[0]);
}

void TxCallback(CanBus* dev, twai_message_t& rMsg)
{
    ESP_LOGI(TAG,
             "[>] Tx Message before callback 7...0: \tID:%03x - %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x",
             rMsg.identifier, rMsg.data[7], rMsg.data[6], rMsg.data[5], rMsg.data[4], rMsg.data[3],
             rMsg.data[2], rMsg.data[1], rMsg.data[0]);
    rMsg.data[0] += 0x12;
    rMsg.data[2] = counter++;
    ESP_LOGI(TAG, "[-> Tx Message to send         7...0: \tID:%03x - %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x",
             rMsg.identifier, rMsg.data[7], rMsg.data[6], rMsg.data[5], rMsg.data[4], rMsg.data[3],
             rMsg.data[2], rMsg.data[1], rMsg.data[0]);
}

void RxCmdCallback(CanBus* dev, twai_message_t& rMsg)
{
    ESP_LOGW(TAG, "[!] Command 7...0: \tID:%03x - %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x", rMsg.identifier,
             rMsg.data[7], rMsg.data[6], rMsg.data[5], rMsg.data[4], rMsg.data[3], rMsg.data[2], rMsg.data[1],
             rMsg.data[0]);
}

extern "C" void app_main(void)
{

    // esp_log_level_set("CAN", ESP_LOG_DEBUG);

    ESP_LOGI(TAG, "-------------------------------------------------------");
    ESP_LOGI(TAG, "CAN start...");
    devCanBus.Start(2U, TX_GPIO_NUM, RX_GPIO_NUM);

    ESP_LOGI(TAG, "-------------------------------------------------------");
    ESP_LOGI(TAG, "Setting up the periodic TX...");
    CanBusDataPointers_t tx_data
        = { data_storage, data_storage + 1, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr };
    devCanBus.AddTxDescriptor(0xa, tx_data);
    devCanBus.AddTxDescriptor(0x1, tx_data);
    devCanBus.SetTxPeriod(3000);
    devCanBus.SetCallbackTx(TxCallback);

    ESP_LOGI(TAG, "-------------------------------------------------------");
    ESP_LOGI(TAG, "Setting up the Store on receiving...");
    devCanBus.SetCallbackRxData(RxCallback);
    devCanBus.SetCallbackRxCmd(RxCmdCallback);

    while (1) {
        delay(0U);
    }
}
