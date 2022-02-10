// *************************************************************************
//
// Copyright (c) 2021 Andrei Gramakov. All rights reserved.
//
// site:    https://agramakov.me
// e-mail:  mail@agramakov.me
//
// *************************************************************************

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "delay.h"
#include "millis.h"
#include "canbus.hpp"
#include <utility>

#define TAG "CAN"
#define LOG_LOCAL_LEVEL ESP_LOG_DEBUG

#define CAN_PRESENCE_PERIOD_MS 1000U
#define RX_TASK_PRIO 8
#define TX_TASK_PRIO 9

void CanBus::TxTask(void *arg)
{
    ESP_LOGI(TAG, "TX task is started");
    CanBus *dev = reinterpret_cast<CanBus *>(arg);
    while (dev->m_flagStarted)
    {
        for (auto it = dev->m_TxDescriptor.begin(); it != dev->m_TxDescriptor.end(); it++)
        {
            twai_message_t msg = {};
            msg.identifier = it->first;
            SetMsgFromPointerData(it->second, msg);
            dev->Send(msg);
        }

        delay(dev->m_TxPeriod_ms);
    }
}

void CanBus::RxTask(void *arg)
{
    ESP_LOGI(TAG, "RX task is started");
    CanBus *dev = reinterpret_cast<CanBus *>(arg);
    twai_message_t rx_msg = {};
    esp_err_t res;
    // While the device is active
    while (dev->m_flagStarted)
    {
        rx_msg = {}; // clear the structure
        res = dev->Receive(rx_msg, portMAX_DELAY);
        if (res != ESP_OK)
        {
            // On error - try again
            continue;
        }

        if (dev->IsMessageRequest(rx_msg))
        {
            ESP_LOGD(TAG, "Request!");
            dev->RxTaskRequestHandler(rx_msg);
        }
        else // message is data
        {
            ESP_LOGD(TAG, "Data!");
            if (dev->IsMessageRequested(rx_msg))
            {
                ESP_LOGD(TAG, "Response - requested data 0x%x!", dev->GetRequestedId());
                dev->m_requestedMsg = rx_msg;
                dev->m_requestResponsed = true;
            }
            else // regular data
            {
                ESP_LOGD(TAG, "Check if to store...");
                auto pair = GetDescriptorPair(dev->m_StoreDescriptor, rx_msg.identifier);
                if (pair != nullptr) // if found, fill and send
                {
                    ESP_LOGD(TAG, "Message should be stored (0x%02x)!", pair->first);
                    SetPointerDataFromMsg(rx_msg, pair->second);
                }
                else
                {
                    ESP_LOGD(TAG, "No, useless data!");
                }
            }
        }
        delay(0U); // for task switching
    }
}

void CanBus::PresenceTask(void *arg)
{
    ESP_LOGI(TAG, "RX task is started");
    CanBus *dev = reinterpret_cast<CanBus *>(arg);

    while (dev->m_flagStarted)
    {
        dev->Send(dev->m_presenceMsg);
        delay(CAN_PRESENCE_PERIOD_MS); // for task switching
    }
}

std::pair<const CanBusId_t, CanBusDataPointers_t> *CanBus::GetDescriptorPair(const std::map<CanBusId_t, CanBusDataPointers_t> &rDesc, CanBusId_t Id)
{
    auto search = rDesc.find(Id);
    if (search != rDesc.end()) // if found, fill and send
    {
        return const_cast<std::pair<const CanBusId_t, CanBusDataPointers_t> *>(&(*search));
    }
    else
    {
        return nullptr;
    }
}

void CanBus::RxTaskRequestHandler(const twai_message_t &rMsg)
{
    auto pair = GetDescriptorPair(m_ResponseDescriptor, rMsg.identifier);
    if (pair != nullptr) // if found, fill and send
    {
        ESP_LOGD(TAG, "Found response!");
        CanBusDataPointers_t resp = pair->second;
        static twai_message_t data_message = {};
        data_message.identifier = rMsg.identifier;
        SetMsgFromPointerData(resp, data_message);
        Send(data_message);
    }
    else // if not - just ignore
    {
        ESP_LOGD(TAG, "Unknown ID 0x%02x", rMsg.identifier);
    }
}

void CanBus::SetMsgFromPointerData(const CanBusDataPointers_t &rPointers, twai_message_t &rMsg)
{
    for (int i = 0; i < 8; i++)
    {
        if (rPointers.array[i] != nullptr)
        {
            rMsg.data[i] = *rPointers.array[i];
            rMsg.data_length_code = i + 1;
        }
    }
}

void CanBus::SetPointerDataFromMsg(twai_message_t &rMsg, const CanBusDataPointers_t &rPointers)
{
    for (int i = 0; i < 8; i++)
    {
        if (rPointers.array[i] != nullptr)
        {
            *rPointers.array[i] = rMsg.data[i];
        }
        else
        {
            ESP_LOGE(TAG, "Not configured pointer for data %d! Data is lost!", i);
        }
    }
}

bool CanBus::IsMessageRequest(const twai_message_t &rMsg)
{
    return (rMsg.rtr == 1U);
};

bool CanBus::IsMessageRequested(const twai_message_t &rMsg)
{
    return (GetRequestedId() == rMsg.identifier);
};

esp_err_t CanBus::Start(uint8_t address, gpio_num_t TxPin, gpio_num_t RxPin)
{
    m_address = address;
    m_flagStarted = true;
    m_TxPin = TxPin;
    m_RxPin = RxPin;
    const twai_timing_config_t t_config = TWAI_TIMING_CONFIG_125KBITS();
    const twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    const twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(m_TxPin, m_RxPin, TWAI_MODE_NORMAL);

    // build a presence message
    m_presenceMsg.identifier = m_address << 8;

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

esp_err_t CanBus::Send(const twai_message_t &rMsg, uint32_t Timeout_ms)
{
    ESP_LOGD(TAG, "[x]---> id:%x flags:0x%08x data:%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x",
             rMsg.identifier,
             rMsg.flags,
             rMsg.data[0],
             rMsg.data[1],
             rMsg.data[2],
             rMsg.data[3],
             rMsg.data[4],
             rMsg.data[5],
             rMsg.data[6],
             rMsg.data[7]);
    TickType_t timeout_ticks = (Timeout_ms == TIMEOUT_FOREVER) ? portMAX_DELAY : to_ticks(Timeout_ms);
    return twai_transmit(&rMsg, timeout_ticks);
}

esp_err_t CanBus::Receive(twai_message_t &rMsg, uint32_t Timeout_ms)
{
    ESP_LOGV(TAG, "[ ]<--- ?");
    TickType_t timeout_ticks = (Timeout_ms == TIMEOUT_FOREVER) ? portMAX_DELAY : to_ticks(Timeout_ms);
    esp_err_t res = twai_receive(&rMsg, timeout_ticks);
    if (res == ESP_OK)
    {
        ESP_LOGD(TAG, "[x]<--- id:%x flags:0x%08x data:%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x",
                 rMsg.identifier,
                 rMsg.flags,
                 rMsg.data[0],
                 rMsg.data[1],
                 rMsg.data[2],
                 rMsg.data[3],
                 rMsg.data[4],
                 rMsg.data[5],
                 rMsg.data[6],
                 rMsg.data[7]);
    }
    else
    {
        ESP_LOGE(TAG, "[ ]<--- Error 0x%08x!\n", res);
    }
    return res;
}

esp_err_t CanBus::StartRequest(uint32_t Id, uint32_t Timeout_ms)
{
    if (m_requestInProgress)
    {
        ESP_LOGE(TAG, "Another request is already in progress");
        return ESP_ERR_INVALID_STATE;
    }
    m_requestedId = Id;
    twai_message_t request_msg = {.rtr = 1,
                                  .identifier = Id,
                                  .data_length_code = 0,
                                  .data = {}};
    m_requestInProgress = true;
    ESP_LOGD(TAG, "+ Request 0x%02x is opened", Id);

    esp_err_t res = Send(request_msg, Timeout_ms);
    if (res != ESP_OK)
    {
        ESP_LOGE(TAG, "Cannot send the request");
        FinishRequest(Id);
    }
    else
    {
        ESP_LOGD(TAG, "> Request is sent");
    }
    return res;
}

void CanBus::FinishRequest(uint32_t Id)
{
    m_requestInProgress = false;
    m_requestResponsed = false;
    m_requestedMsg = {};
    ESP_LOGD(TAG, "- Request 0x%02x is closed", Id);
}

bool CanBus::IsTimeUp(uint64_t Start_ms, uint64_t Timeout_ms)
{
    uint64_t fromStart_ms = (Timeout_ms == TIMEOUT_FOREVER) ? static_cast<uint64_t>(0) : (millis() - Start_ms);
    // If timeout_ms == TIMEOUT_FOREVER, fromStart_ms = 0 -> always false
    return fromStart_ms > Timeout_ms;
}

esp_err_t CanBus::Request(uint32_t Id, twai_message_t &rMsg, uint32_t Timeout_ms)
{
    uint64_t start_ms = millis();

    if (m_requestInProgress)
    {
        ESP_LOGE(TAG, "Another request is already in progress");
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t res = StartRequest(Id, Timeout_ms);

    ESP_LOGD(TAG, "? Request is sent");
    while (!IsTimeUp(start_ms, Timeout_ms))
    {
        if (m_requestResponsed)
        {
            ESP_LOGD(TAG, "v Request 0x%02x got a response!", Id);
            rMsg = m_requestedMsg;
            break;
        }
        delay(0U);
    }
    res = m_requestResponsed ? ESP_OK : ESP_ERR_TIMEOUT;
    FinishRequest(Id);
    return res;
}

esp_err_t CanBus::AddResponseDescriptor(CanBusId_t Id, CanBusDataPointers_t &rData)
{
    m_ResponseDescriptor[Id] = rData;
    return ESP_OK;
}

esp_err_t CanBus::AddTxDescriptor(CanBusId_t Id, CanBusDataPointers_t &rData)
{
    m_TxDescriptor[Id] = rData;
    return ESP_OK;
}

esp_err_t CanBus::AddStoreDescriptor(CanBusId_t Id, CanBusDataPointers_t &rData)
{
    m_StoreDescriptor[Id] = rData;
    return ESP_OK;
}

void CanBus::SetTxPeriod(uint64_t Period_ms)
{
    m_TxPeriod_ms = Period_ms;
}

bool CanBus::IsStarted()
{
    return m_flagStarted;
}

CanBus::CanBus() : m_TxPeriod_ms(static_cast<uint64_t>(100U)),
                   m_presenceMsg{} {

                   };

CanBus::~CanBus()
{
    Stop();
}

CanBus devCanBus;
