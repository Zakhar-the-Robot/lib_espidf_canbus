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
#include "canbus_version.hpp"
#include "delay.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "millis.h"
#include <utility>

#define TAG "CAN"

void CanBus::TxTask(void* arg)
{
    ESP_LOGI(TAG, "TX task is started");
    CanBus* dev = reinterpret_cast<CanBus*>(arg);
    while (dev->m_flagStarted) {
        for (auto it = dev->m_TxDescriptor.begin(); it != dev->m_TxDescriptor.end(); it++) {
            twai_message_t msg = {};
            msg.identifier = it->first | dev->m_address;
            SetMsgFromPointerData(it->second, msg);
            dev->CallbackTx(msg);
            dev->SendMsg(msg);
        }
        delay(dev->m_TxPeriod_ms);
    }
}

void CanBus::RxTask(void* arg)
{
    ESP_LOGI(TAG, "RX task is started");
    CanBus*        dev = reinterpret_cast<CanBus*>(arg);
    twai_message_t rx_msg = {};
    esp_err_t      res;
    // While the device is active
    while (dev->m_flagStarted) {
        rx_msg = {}; // clear the structure
        res = dev->ReceiveMsg(rx_msg, portMAX_DELAY);
        if (res != ESP_OK) { // On error - try again
            continue;
        }
        ESP_LOGD(TAG, "RX message!");
        dev->CallbackRxData(rx_msg);
        bool is_cmd = (rx_msg.identifier & 0xF) == 0xF; // message type is CMD
        bool is_target = ((rx_msg.identifier >> 4) & 0xF) == (dev->m_address >> 8); // is it for me?
        if (is_cmd && is_target) {
            ESP_LOGD(TAG, "RX command!");
            dev->CallbackRxCmd(rx_msg);
        }
        delay(0U); // for task switching
    }
}

void CanBus::PresenceTask(void* arg)
{
    ESP_LOGI(TAG, "Presence task is started");
    CanBus* dev = reinterpret_cast<CanBus*>(arg);

    while (dev->m_flagStarted) {
        dev->SendMsg(dev->m_presenceMsg);
        delay(CAN_PRESENCE_PERIOD_MS); // for task switching
    }
}

std::pair<const CanBusId_t, CanBusDataPointers_t>*
CanBus::GetDescriptorPair(const std::map<CanBusId_t, CanBusDataPointers_t>& rDesc, CanBusId_t Id)
{
    auto search = rDesc.find(Id);
    if (search != rDesc.end()) // if found, fill and send
    {
        return const_cast<std::pair<const CanBusId_t, CanBusDataPointers_t>*>(&(*search));
    } else {
        return nullptr;
    }
}

void CanBus::SetMsgFromPointerData(const CanBusDataPointers_t& rPointers, twai_message_t& rMsg)
{
    for (int i = 0; i < 8; i++) {
        if (rPointers.array[i] != nullptr) {
            rMsg.data[i] = *rPointers.array[i];
            rMsg.data_length_code = i + 1;
        }
    }
}

void CanBus::SetPointerDataFromMsg(twai_message_t& rMsg, const CanBusDataPointers_t& rPointers)
{
    for (int i = 0; i < 8; i++) {
        if (rPointers.array[i] != nullptr) {
            *rPointers.array[i] = rMsg.data[i];
        } else {
            ESP_LOGE(TAG, "Not configured pointer for data %d! Data is lost!", i);
        }
    }
}

esp_err_t CanBus::Start(uint8_t address_byte2, gpio_num_t txPin, gpio_num_t rxPin)
{
    m_address = (address_byte2 & 0x7) << 8;
    m_flagStarted = true;
    m_txPin = txPin;
    m_rxPin = rxPin;
    const twai_timing_config_t  t_config = TWAI_TIMING_CONFIG_125KBITS();
    const twai_filter_config_t  f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    const twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(m_txPin, m_rxPin, TWAI_MODE_NORMAL);

    // build a presence message
    m_presenceMsg.identifier = m_address;
    m_presenceMsg.data_length_code = 3;
    m_presenceMsg.data[0] = QCAN_VERSION_MAJOR;
    m_presenceMsg.data[1] = QCAN_VERSION_MINOR;
    m_presenceMsg.data[2] = QCAN_VERSION_PATCH;

    // Install TWAI driver
    ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
    ESP_LOGI(TAG, "Driver installed");

    ESP_ERROR_CHECK(twai_start());
    ESP_LOGI(TAG, "Driver started");

    xTaskCreatePinnedToCore(TxTask, "canbus_tx", 4096, this, TX_TASK_PRIO, NULL, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(RxTask, "canbus_rx", 4096, this, RX_TASK_PRIO, NULL, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(PresenceTask, "canbus_presence", 4096, this, RX_TASK_PRIO, NULL, tskNO_AFFINITY);

    return ESP_OK;
}

esp_err_t CanBus::Stop()
{
    m_flagStarted = false;
    ESP_ERROR_CHECK(twai_stop());
    ESP_LOGI(TAG, "Driver stopped");

    // Uninstall TWAI driver
    ESP_ERROR_CHECK(twai_driver_uninstall());
    ESP_LOGI(TAG, "Driver uninstalled");
    return ESP_OK;
}

esp_err_t CanBus::SendMsg(uint32_t id, uint8_t data7, uint8_t data6, uint8_t data5, uint8_t data4,
                          uint8_t data3, uint8_t data2, uint8_t data1, uint8_t data0, uint32_t timeout_ms)
{
    twai_message_t msg = {};
    msg.data[0] = data0;
    msg.data[1] = data1;
    msg.data[2] = data2;
    msg.data[3] = data3;
    msg.data[4] = data4;
    msg.data[5] = data5;
    msg.data[6] = data6;
    msg.data[7] = data7;
    for (int i = 0; i < 8; i++) {
        msg.data_length_code = (msg.data[i] == 0U) ? msg.data_length_code : (msg.data_length_code + 1);
    }
    msg.identifier = id;

    return SendMsg(msg, timeout_ms);
}

esp_err_t CanBus::SendMsg(twai_message_t& rMsg, uint32_t timeout_ms)
{
    ESP_LOGD(TAG, "[x]---> id:%x flags:0x%08x data:%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x", rMsg.identifier,
             rMsg.flags, rMsg.data[0], rMsg.data[1], rMsg.data[2], rMsg.data[3], rMsg.data[4], rMsg.data[5],
             rMsg.data[6], rMsg.data[7]);
    TickType_t timeout_ticks = (timeout_ms == TIMEOUT_FOREVER) ? portMAX_DELAY : to_ticks(timeout_ms);
    return twai_transmit(&rMsg, timeout_ticks);
}

esp_err_t CanBus::ReceiveMsg(twai_message_t& rMsg, uint32_t timeout_ms)
{
    ESP_LOGV(TAG, "[ ]<--- ?");
    TickType_t timeout_ticks = (timeout_ms == TIMEOUT_FOREVER) ? portMAX_DELAY : to_ticks(timeout_ms);
    esp_err_t  res = twai_receive(&rMsg, timeout_ticks);
    if (res == ESP_OK) {
        ESP_LOGD(TAG, "[x]<--- id:%x flags:0x%08x data:%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x",
                 rMsg.identifier, rMsg.flags, rMsg.data[0], rMsg.data[1], rMsg.data[2], rMsg.data[3],
                 rMsg.data[4], rMsg.data[5], rMsg.data[6], rMsg.data[7]);
    } else {
        ESP_LOGE(TAG, "[ ]<--- Error 0x%08x!\n", res);
    }
    return res;
}

bool CanBus::IsTimeUp(uint64_t Start_ms, uint64_t Timeout_ms)
{
    uint64_t fromStart_ms
        = (Timeout_ms == TIMEOUT_FOREVER) ? static_cast<uint64_t>(0) : (millis() - Start_ms);
    // If timeout_ms == TIMEOUT_FOREVER, fromStart_ms = 0 -> always false
    return fromStart_ms > Timeout_ms;
}

esp_err_t CanBus::AddTxDescriptor(uint8_t data_id, CanBusDataPointers_t& rData)
{
    m_TxDescriptor[data_id] = rData;
    return ESP_OK;
}

void CanBus::SetTxPeriod(uint64_t period_ms) { m_TxPeriod_ms = period_ms; }

bool CanBus::IsStarted() { return m_flagStarted; }

CanBus::CanBus()
    : m_TxPeriod_ms(static_cast<uint64_t>(100U))
    , m_presenceMsg {}
    , m_pCallbackRxData(nullptr)
    , m_pCallbackTx(nullptr)
    // TODO: add all attributes to initialize all of them explicitly
    {

    };

CanBus::~CanBus() { Stop(); }

esp_err_t CanBus::SendDataMsg(uint8_t data_id, uint8_t data7, uint8_t data6, uint8_t data5, uint8_t data4,
                              uint8_t data3, uint8_t data2, uint8_t data1, uint8_t data0, uint32_t timeout_ms)
{
    return SendMsg((m_address | data_id), data7, data6, data5, data4, data3, data2, data1, data0, timeout_ms);
}

esp_err_t CanBus::SendDataMsg(uint8_t data_id, const uint8_t (&data)[8], uint32_t timeout_ms)
{
    return SendMsg((m_address | data_id), data[0], data[1], data[2], data[3], data[4], data[5], data[6],
                   data[7], timeout_ms);
}

esp_err_t CanBus::SendCmdMsg(uint8_t target_dev_id, uint8_t data7, uint8_t data6, uint8_t data5,
                             uint8_t data4, uint8_t data3, uint8_t data2, uint8_t data1, uint8_t data0,
                             uint32_t timeout_ms)
{

    uint32_t id = m_address | (target_dev_id << 4U) | 0xF; // src | target | 0xF
    return SendMsg(id, data7, data6, data5, data4, data3, data2, data1, data0, timeout_ms);
}

esp_err_t CanBus::SendCmdMsg(uint8_t target_dev_id, const uint8_t (&data)[8], uint32_t timeout_ms)
{
    uint32_t id = m_address | (target_dev_id << 4U) | 0xF; // src | target | 0xF
    return SendMsg(id, data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7], timeout_ms);
}

CanBus devCanBus;
