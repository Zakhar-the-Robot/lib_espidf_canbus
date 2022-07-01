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

#include "driver/twai.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "hal/gpio_types.h"
#include <map>

// FORWARD REFERENCES
class CanBus;

typedef uint32_t CanBusId_t;
typedef void (*CanBusCallback_t)(CanBus*, twai_message_t& rMsg);

typedef union CanBusResponseData {
    struct {
        uint8_t* d0;
        uint8_t* d1;
        uint8_t* d2;
        uint8_t* d3;
        uint8_t* d4;
        uint8_t* d5;
        uint8_t* d6;
        uint8_t* d7;
    };
    uint8_t* array[8];
} CanBusDataPointers_t;

class CanBus {
private:
    /********************/
    /* Private Constans */
    /********************/
    static const uint32_t RX_TASK_PRIO = 8U;
    static const uint32_t TX_TASK_PRIO = 9U;

    /*******************/
    /* Private Methods */
    /*******************/
    static void TxTask(void* arg);
    static void RxTask(void* arg);
    static void PresenceTask(void* arg);

    static std::pair<const CanBusId_t, CanBusDataPointers_t>*
                GetDescriptorPair(const std::map<CanBusId_t, CanBusDataPointers_t>& rDesc, CanBusId_t Id);
    static void SetMsgFromPointerData(const CanBusDataPointers_t& rPointers, twai_message_t& rMsg);
    static void SetPointerDataFromMsg(twai_message_t& rMsg, const CanBusDataPointers_t& rPointers);
    static bool IsTimeUp(uint64_t Start_ms, uint64_t Timeout_ms);
    esp_err_t   ReceiveMsg(twai_message_t& rMsg, uint32_t timeout_ms = TIMEOUT_FOREVER);
    esp_err_t   SendMsg(twai_message_t& rMsg, uint32_t timeout_ms = TIMEOUT_FOREVER);
    esp_err_t   SendMsg(uint32_t id, uint8_t data7 = 0x00U, uint8_t data6 = 0x00U, uint8_t data5 = 0x00U,
                        uint8_t data4 = 0x00U, uint8_t data3 = 0x00U, uint8_t data2 = 0x00U,
                        uint8_t data1 = 0x00U, uint8_t data0 = 0x00U, uint32_t timeout_ms = TIMEOUT_FOREVER);

    void CallbackRxData(twai_message_t& rMsg);
    void CallbackRxCmd(twai_message_t& rMsg);
    void CallbackTx(twai_message_t& rMsg);

    /**********************/
    /* Private Attributes */
    /**********************/
    uint32_t                                   m_address;
    bool                                       m_flagStarted; // the object is started
    gpio_num_t                                 m_txPin;
    gpio_num_t                                 m_rxPin;
    uint64_t                                   m_TxPeriod_ms;
    twai_message_t                             m_presenceMsg;
    std::map<CanBusId_t, CanBusDataPointers_t> m_TxDescriptor; // what data to send periodically
    uint32_t                                   m_requestedId;
    bool                                       m_requestInProgress;
    bool                                       m_requestResponsed;
    twai_message_t                             m_requestedMsg;
    CanBusCallback_t                           m_pCallbackRxData;
    CanBusCallback_t                           m_pCallbackRxCmd;
    CanBusCallback_t                           m_pCallbackTx;

public:
    static const uint32_t TIMEOUT_FOREVER = 0xFFFFFFFFU;
    static const uint32_t CAN_PRESENCE_PERIOD_MS = 1000U;

    CanBus();
    ~CanBus();

    /**
     * @brief Start CAN bus
     *
     * @param address_byte1 lower bits are vary according to the protocol
     * @param txPin
     * @param rxPin
     * @return esp_err_t ESP Error code
     */
    esp_err_t Start(uint8_t address_byte1, gpio_num_t txPin, gpio_num_t rxPin);

    /**
     * @brief Return the state of the CAN
     *
     * @return true Started
     * @return false Stopped
     */
    bool IsStarted();

    /**
     * @brief Stop CAN bus
     *
     * @return esp_err_t ESP Error code
     */
    esp_err_t Stop();

    /**
     * @brief Send a Data message
     *
     * @param data_id Defines last 8 bits of the message address. The first byte
     *                is a device ID.
     * @param data7 Message byte 7
     * @param data6 Message byte 6
     * @param data5 Message byte 5
     * @param data4 Message byte 4
     * @param data3 Message byte 3
     * @param data2 Message byte 2
     * @param data1 Message byte 1
     * @param data0 Message byte 0
     * @param timeout_ms Timeout
     * @return esp_err_t ESP Error code
     */
    esp_err_t SendDataMsg(uint8_t data_id, uint8_t data7 = 0x00U, uint8_t data6 = 0x00U,
                          uint8_t data5 = 0x00U, uint8_t data4 = 0x00U, uint8_t data3 = 0x00U,
                          uint8_t data2 = 0x00U, uint8_t data1 = 0x00U, uint8_t data0 = 0x00U,
                          uint32_t timeout_ms = TIMEOUT_FOREVER);

    /**
     * @brief Send a Data message
     *
     * @param data_id Defines last 8 bits of the message address. The first byte
     *                is a device ID.
     * @param data A data array  with size of 8
     * @param timeout_ms Timeout
     * @return esp_err_t ESP Error code
     */
    esp_err_t SendDataMsg(uint8_t data_id, const uint8_t (&data)[8], uint32_t timeout_ms = TIMEOUT_FOREVER);

    /**
     * @brief Send a Command message
     *
     * @param target_dev_id ID of the target device
     * @param data7 Message byte 7
     * @param data6 Message byte 6
     * @param data5 Message byte 5
     * @param data4 Message byte 4
     * @param data3 Message byte 3
     * @param data2 Message byte 2
     * @param data1 Message byte 1
     * @param data0 Message byte 0
     * @param timeout_ms Timeout
     * @return esp_err_t ESP Error code
     *
     */
    esp_err_t SendCmdMsg(uint8_t target_dev_id, uint8_t data7 = 0x00U, uint8_t data6 = 0x00U,
                         uint8_t data5 = 0x00U, uint8_t data4 = 0x00U, uint8_t data3 = 0x00U,
                         uint8_t data2 = 0x00U, uint8_t data1 = 0x00U, uint8_t data0 = 0x00U,
                         uint32_t timeout_ms = TIMEOUT_FOREVER);

    /**
     * @brief Send a Data message
     *
     * @param target_dev_id ID of the target device
     * @param data A data array  with size of 8
     * @param timeout_ms Timeout
     * @return esp_err_t ESP Error code
     */
    esp_err_t SendCmdMsg(uint8_t  target_dev_id, const uint8_t (&data)[8],
                         uint32_t timeout_ms = TIMEOUT_FOREVER);

    /**
     * @brief Add data location for periodic transaction
     *
     * @param data_id Data ID
     * @param rData Data location descriptor
     * @return esp_err_t ESP Error code
     */
    esp_err_t AddTxDescriptor(uint8_t data_id, CanBusDataPointers_t& rData);

    /**
     * @brief Set the Tx period
     *
     * @param Period_ms
     */
    void SetTxPeriod(uint64_t period_ms);

    void SetCallbackRxData(CanBusCallback_t callback); // TODO rename to SetCallbackRxMsg
    void SetCallbackRxCmd(CanBusCallback_t callback);
    void SetCallbackTx(CanBusCallback_t callback);
};

inline void CanBus::SetCallbackRxData(CanBusCallback_t callback) { m_pCallbackRxData = callback; }

inline void CanBus::SetCallbackTx(CanBusCallback_t callback) { m_pCallbackTx = callback; }

inline void CanBus::SetCallbackRxCmd(CanBusCallback_t callback) { m_pCallbackRxCmd = callback; }

inline void CanBus::CallbackRxData(twai_message_t& rMsg)
{
    if (m_pCallbackRxData != nullptr) {
        m_pCallbackRxData(this, rMsg);
    }
}

inline void CanBus::CallbackRxCmd(twai_message_t& rMsg)
{
    if (m_pCallbackRxCmd != nullptr) {
        m_pCallbackRxCmd(this, rMsg);
    }
}

inline void CanBus::CallbackTx(twai_message_t& rMsg)
{
    if (m_pCallbackTx != nullptr) {
        m_pCallbackTx(this, rMsg);
    }
}

extern CanBus devCanBus;
