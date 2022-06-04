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

// FORWARD REFERENCES
class CanBus;

typedef uint32_t CanBusId_t;
typedef void (*CanBusCallback_t)(CanBus *, const twai_message_t & rMsg);

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


class CanBus
{
private:
    /*******************/
    /* Private Methods */
    /*******************/
    uint32_t GetRequestedId() { return m_requestInProgress ? m_requestedId : 0xFFFFFFFF; };
    static std::pair<const CanBusId_t, CanBusDataPointers_t> *GetDescriptorPair(
        const std::map<CanBusId_t, 
        CanBusDataPointers_t> &rDesc, 
        CanBusId_t Id);
    esp_err_t StartRequest(uint32_t Id, uint32_t Timeout_ms);
    static void SetMsgFromPointerData(const CanBusDataPointers_t &rPointers, twai_message_t &rMsg);
    static void SetPointerDataFromMsg(twai_message_t &rMsg, const CanBusDataPointers_t &rPointers);
    void FinishRequest(uint32_t Id);
    // Checks
    static bool IsMessageRequest(const twai_message_t &rMsg);
    bool IsMessageRequested(const twai_message_t &rMsg);
    bool IsTimeUp(uint64_t Start_ms, uint64_t Timeout_ms);
    // Tasks
    static void TxTask(void *arg);
    static void RxTask(void *arg);
    static void PresenceTask(void *arg);
    void HandlerRxTaskRequest(const twai_message_t &rMsg);
    void CallbackRxData(twai_message_t &rMsg);
    void CallbackRxRequext(twai_message_t &rMsg);
    void CallbackTx(twai_message_t &rMsg);

    /**********************/
    /* Private Attributes */
    /**********************/
    uint8_t m_address;
    bool m_flagStarted;  // the object is started
    gpio_num_t m_TxPin;
    gpio_num_t m_RxPin;
    uint64_t m_TxPeriod_ms;
    twai_message_t m_presenceMsg;
    std::map<CanBusId_t, CanBusDataPointers_t> m_StoreDescriptor; // where to store received data
    std::map<CanBusId_t, CanBusDataPointers_t> m_TxDescriptor;    // what data to send periodically
    std::map<CanBusId_t, CanBusDataPointers_t> m_ResponseDescriptor; // what to send in
    uint32_t m_requestedId;
    bool m_requestInProgress;
    bool m_requestResponsed;
    twai_message_t m_requestedMsg;
    CanBusCallback_t m_pCallbackRxData;
    CanBusCallback_t m_pCallbackTx;
    CanBusCallback_t m_pCallbackRxRequest;


public:
    static const uint32_t TIMEOUT_FOREVER = 0xFFFFFFFFU;
    static const uint32_t CAN_PRESENCE_PERIOD_MS = 1000U;
    static const uint32_t RX_TASK_PRIO = 8U;
    static const uint32_t TX_TASK_PRIO = 9U;

    CanBus();
    ~CanBus();

    esp_err_t Start(uint8_t address, gpio_num_t TxPin, gpio_num_t RxPin);
    esp_err_t Stop();
    esp_err_t Send(const twai_message_t &Msg, uint32_t Timeout_ms = TIMEOUT_FOREVER);
    esp_err_t Receive(twai_message_t &rMsg, uint32_t Timeout_ms = TIMEOUT_FOREVER);
    esp_err_t Request(uint32_t Id, twai_message_t &rMsg, uint32_t Timeout_ms = TIMEOUT_FOREVER);
    esp_err_t AddResponseDescriptor(CanBusId_t Id, CanBusDataPointers_t &rData);
    esp_err_t AddTxDescriptor(CanBusId_t Id, CanBusDataPointers_t &rData);
    esp_err_t AddStoreDescriptor(CanBusId_t Id, CanBusDataPointers_t &rData);
    void SetTxPeriod(uint64_t Period_ms);
    void SetCallbackRxData(CanBusCallback_t callback);
    void SetCallbackRxRequext(CanBusCallback_t callback);
    void SetCallbackTx(CanBusCallback_t callback);
    bool IsStarted();
};

inline void CanBus::SetCallbackRxData(CanBusCallback_t callback)
{
    m_pCallbackRxData = callback;
}

inline void CanBus::SetCallbackRxRequext(CanBusCallback_t callback)
{
    m_pCallbackRxRequest = callback;
}

inline void CanBus::SetCallbackTx(CanBusCallback_t callback)
{
    m_pCallbackTx = callback;
}

inline void CanBus::CallbackRxData(twai_message_t& rMsg)
{
    if (m_pCallbackRxData != nullptr) {
        m_pCallbackRxData(this, rMsg);
    }
}

inline void CanBus::CallbackRxRequext(twai_message_t& rMsg)
{
    if (m_pCallbackRxRequest != nullptr) {
        m_pCallbackRxRequest(this, rMsg);
    }
}

inline void CanBus::CallbackTx(twai_message_t& rMsg)
{
    if (m_pCallbackTx != nullptr) {
        m_pCallbackTx(this, rMsg);
    }
}

extern CanBus devCanBus;
