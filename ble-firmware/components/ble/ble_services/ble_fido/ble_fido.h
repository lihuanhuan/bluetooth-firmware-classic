 /**
 * Copyright (c) 2012 - 2019, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/**@file
 *
 * @defgroup ble_fido FIDO Service
 * @{
 * @ingroup  ble_sdk_srv
 * @brief    FIDO Service implementation.
 *
 * @details The FIDO Service is a simple GATT-based service with TX and RX characteristics.
 *          Data received from the peer is passed to the application, and the data received
 *          from the application of this service is sent to the peer as Handle Value
 *          Notifications. This module demonstrates how to implement a custom GATT-based
 *          service and characteristics using the SoftDevice. The service
 *          is used by the application to send and receive ASCII text strings to and from the
 *          peer.
 *
 * @note    The application must register this module as BLE event observer using the
 *          NRF_SDH_BLE_OBSERVER macro. Example:
 *          @code
 *              ble_fido_t instance;
 *              NRF_SDH_BLE_OBSERVER(anything, BLE_FIDO_BLE_OBSERVER_PRIO,
 *                                   ble_fido_on_ble_evt, &instance);
 *          @endcode
 */
#ifndef BLE_FIDO_H__
#define BLE_FIDO_H__

#include <stdint.h>
#include <stdbool.h>
#include "sdk_config.h"
#include "ble.h"
#include "ble_srv_common.h"
#include "nrf_sdh_ble.h"
#include "ble_link_ctx_manager.h"

#ifdef __cplusplus
extern "C" {
#endif

#define BLE_FIDO_BLE_OBSERVER_PRIO 2

/**@brief   Macro for defining a ble_fido instance.
 *
 * @param     _name            Name of the instance.
 * @param[in] _fido_max_clients Maximum number of NUS clients connected at a time.
 * @hideinitializer
 */
#define BLE_FIDO_DEF(_name, _fido_max_clients)                      \
    BLE_LINK_CTX_MANAGER_DEF(CONCAT_2(_name, _link_ctx_storage),    \
                             (_fido_max_clients),                   \
                             sizeof(ble_fido_client_context_t));    \
    static ble_fido_t _name =                                       \
    {                                                               \
        .p_link_ctx_storage = &CONCAT_2(_name, _link_ctx_storage)   \
    };                                                              \
    NRF_SDH_BLE_OBSERVER(_name ## _obs,                             \
                         BLE_FIDO_BLE_OBSERVER_PRIO,                 \
                         ble_fido_on_ble_evt,                       \
                         &_name)

#define BLE_UUID_FIDO_SERVICE 0xFFFD /**< The UUID of the FIDO Service. */
#define BLE_UUID_FIDO_CONTROL_POINT 0xFFF1 /**< The UUID of the FIDO Control Point. */
#define BLE_UUID_FIDO_STATUS 0xFFF2 /**< The UUID of the FIDO Status. */
#define BLE_UUID_FIDO_CONTROL_POINT_LENGTH 0xFFF3 /**< The UUID of the FIDO Control Point Length. */
#define BLE_UUID_FIDO_SERVICE_REVISION_BITFIELD 0xFFF4 /**< The UUID of the FIDO Service Revision Bitfield. */
#define BLE_UUID_FIDO_SERVICE_REVISION 0x2A28 /**< The UUID of the FIDO Service Revision. */

#define OPCODE_LENGTH        1
#define HANDLE_LENGTH        2

/**@brief   Maximum length of data (in bytes) that can be transmitted to the peer by the FIDO service module. */
#if defined(NRF_SDH_BLE_GATT_MAX_MTU_SIZE) && (NRF_SDH_BLE_GATT_MAX_MTU_SIZE != 0)
    #define BLE_FIDO_MAX_DATA_LEN (NRF_SDH_BLE_GATT_MAX_MTU_SIZE - OPCODE_LENGTH - HANDLE_LENGTH)
#else
    #define BLE_FIDO_MAX_DATA_LEN (BLE_GATT_MTU_SIZE_DEFAULT - OPCODE_LENGTH - HANDLE_LENGTH)
    #warning NRF_SDH_BLE_GATT_MAX_MTU_SIZE is not defined.
#endif


/**@brief   FIDO Service event types. */
typedef enum
{
    BLE_FIDO_EVT_RX_DATA,      /**< Data received. */
    BLE_FIDO_EVT_TX_RDY,       /**< Service is ready to accept new data to be transmitted. */
    BLE_FIDO_EVT_COMM_STARTED, /**< Notification has been enabled. */
    BLE_FIDO_EVT_COMM_STOPPED, /**< Notification has been disabled. */
} ble_fido_evt_type_t;


/* Forward declaration of the ble_fido_t type. */
typedef struct ble_fido_s ble_fido_t;


/**@brief   FIDO Service @ref BLE_FIDO_EVT_RX_DATA event data.
 *
 * @details This structure is passed to an event when @ref BLE_FIDO_EVT_RX_DATA occurs.
 */
typedef struct
{
    uint8_t const * p_data; /**< A pointer to the buffer with received data. */
    uint16_t        length; /**< Length of received data. */
} ble_fido_evt_rx_data_t;


/**@brief FIDO Service client context structure.
 *
 * @details This structure contains state context related to hosts.
 */
typedef struct
{
    bool is_notification_enabled; /**< Variable to indicate if the peer has enabled notification of the RX characteristic.*/
} ble_fido_client_context_t;


/**@brief   FIDO Service event structure.
 *
 * @details This structure is passed to an event coming from service.
 */
typedef struct
{
    ble_fido_evt_type_t         type;        /**< Event type. */
    ble_fido_t                * p_fido;       /**< A pointer to the instance. */
    uint16_t                   conn_handle; /**< Connection handle. */
    ble_fido_client_context_t * p_link_ctx;  /**< A pointer to the link context. */
    union
    {
        ble_fido_evt_rx_data_t rx_data; /**< @ref BLE_FIDO_EVT_RX_DATA event data. */
    } params;
} ble_fido_evt_t;


/**@brief FIDO Service event handler type. */
typedef void (* ble_fido_data_handler_t) (ble_fido_evt_t * p_evt);


/**@brief   FIDO Service initialization structure.
 *
 * @details This structure contains the initialization information for the service. The application
 * must fill this structure and pass it to the service using the @ref ble_fido_init
 *          function.
 */
typedef struct
{
    ble_fido_data_handler_t data_handler; /**< Event handler to be called for handling received data. */
} ble_fido_init_t;


/**@brief   FIDO Service structure.
 *
 * @details This structure contains status information related to the service.
 */
struct ble_fido_s
{
    uint8_t                         uuid_type;          /**< UUID type for FIDO Service Base UUID. */
    uint16_t                        service_handle;     /**< Handle of FIDO Service (as provided by the SoftDevice). */
    ble_gatts_char_handles_t        control_point_handles;
    ble_gatts_char_handles_t        status_handles;
    ble_gatts_char_handles_t        control_point_length_handles;
    ble_gatts_char_handles_t        service_revision_bitfield_handles;
    ble_gatts_char_handles_t        service_revision_handles;
    blcm_link_ctx_storage_t * const p_link_ctx_storage; /**< Pointer to link context storage with handles of all current connections and its context. */
    ble_fido_data_handler_t          data_handler;       /**< Event handler to be called for handling received data. */
};


/**@brief   Function for initializing the FIDO Service.
 *
 * @param[out] p_fido      FIDO Service structure. This structure must be supplied
 *                        by the application. It is initialized by this function and will
 *                        later be used to identify this particular service instance.
 * @param[in] p_fido_init  Information needed to initialize the service.
 *
 * @retval NRF_SUCCESS If the service was successfully initialized. Otherwise, an error code is returned.
 * @retval NRF_ERROR_NULL If either of the pointers p_fido or p_fido_init is NULL.
 */
uint32_t ble_fido_init(ble_fido_t * p_fido, ble_fido_init_t const * p_fido_init);


/**@brief   Function for handling the FIDO Service's BLE events.
 *
 * @details The FIDO Service expects the application to call this function each time an
 * event is received from the SoftDevice. This function processes the event if it
 * is relevant and calls the FIDO Service event handler of the
 * application if necessary.
 *
 * @param[in] p_ble_evt     Event received from the SoftDevice.
 * @param[in] p_context     FIDO Service structure.
 */
void ble_fido_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context);


/**@brief   Function for sending a data to the peer.
 *
 * @details This function sends the input string as an RX characteristic notification to the
 *          peer.
 *
 * @param[in]     p_fido       Pointer to the FIDO Service structure.
 * @param[in]     p_data      String to be sent.
 * @param[in,out] p_length    Pointer Length of the string. Amount of sent bytes.
 * @param[in]     conn_handle Connection Handle of the destination client.
 *
 * @retval NRF_SUCCESS If the string was sent successfully. Otherwise, an error code is returned.
 */
uint32_t ble_fido_data_send(ble_fido_t * p_fido,
                           uint8_t   * p_data,
                           uint16_t  * p_length,
                           uint16_t    conn_handle);


#ifdef __cplusplus
}
#endif

#endif // BLE_FIDO_H__

/** @} */
