// *************************************************************************
//
// Copyright (c) 2021 Andrei Gramakov. All rights reserved.
//
// site:    https://agramakov.me
// e-mail:  mail@agramakov.me
//
// *************************************************************************

#pragma once

#include <map>
#include "driver/twai.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "hal/gpio_types.h"

typedef union CanBusResponseData
{
    struct
    {
        uint8_t *d0;
        uint8_t *d1;
        uint8_t *d2;
        uint8_t *d3;
        uint8_t *d4;
        uint8_t *d5;
        uint8_t *d6;
        uint8_t *d7;
    };
    uint8_t *array[8];
} CanBusDataPointers_t;

typedef uint32_t CanBusId_t;

class CanBus
{
private:
    bool m_flagStarted;
    gpio_num_t m_TxPin;
    gpio_num_t m_RxPin;
    uint64_t m_TxPeriod_ms;

    std::map<CanBusId_t, CanBusDataPointers_t> m_StoreDescriptor; // where to store received data
    std::map<CanBusId_t, CanBusDataPointers_t> m_TxDescriptor;    // what data to send periodically

    std::map<CanBusId_t, CanBusDataPointers_t> m_ResponseDescriptor; // what to send in
    uint32_t m_requestedId;
    bool m_requestInProgress;
    bool m_requestResponsed;
    twai_message_t m_requestedMsg;

    uint32_t GetRequestedId() { return m_requestInProgress ? m_requestedId : 0xFFFFFFFF; };
    static std::pair<const CanBusId_t, CanBusDataPointers_t> *GetDescriptorPair(const std::map<CanBusId_t, CanBusDataPointers_t> &rDesc, CanBusId_t Id);
    esp_err_t StartRequest(uint32_t Id, uint32_t Timeout_ms);
    static void SetMsgFromPointerData(const CanBusDataPointers_t &rPointers, twai_message_t &rMsg);
    static void SetPointerDataFromMsg(twai_message_t &rMsg, const CanBusDataPointers_t &rPointers);
    void FinishRequest(uint32_t Id);

    static bool IsMessageRequest(const twai_message_t &rMsg);
    bool IsMessageRequested(const twai_message_t &rMsg);
    bool IsTimeUp(uint64_t Start_ms, uint64_t Timeout_ms);

    static void TxTask(void *arg);
    static void RxTask(void *arg);
    void RxTaskRequestHandler(const twai_message_t &rMsg);

public:
    static const uint32_t TIMEOUT_FOREVER = 0xFFFFFFFF;

    CanBus();
    ~CanBus();

    esp_err_t Start(gpio_num_t TxPin, gpio_num_t RxPin);
    esp_err_t Stop();
    esp_err_t Send(const twai_message_t &Msg, uint32_t Timeout_ms = TIMEOUT_FOREVER);
    esp_err_t Receive(twai_message_t &rMsg, uint32_t Timeout_ms = TIMEOUT_FOREVER);
    esp_err_t Request(uint32_t Id, twai_message_t &rMsg, uint32_t Timeout_ms = TIMEOUT_FOREVER);
    esp_err_t AddResponseDescriptor(CanBusId_t Id, CanBusDataPointers_t &rData);
    esp_err_t AddTxDescriptor(CanBusId_t Id, CanBusDataPointers_t &rData);
    esp_err_t AddStoreDescriptor(CanBusId_t Id, CanBusDataPointers_t &rData);
    void SetTxPeriod(uint64_t Period_ms);
    bool IsStarted();
};

extern CanBus devCanBus;
