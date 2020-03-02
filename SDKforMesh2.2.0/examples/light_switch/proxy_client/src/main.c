/* Copyright (c) 2010 - 2018, Nordic Semiconductor ASA
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
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
 */

#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include <string.h>

/* HAL */
#include "boards.h"
#include "simple_hal.h"
#include "app_timer.h"
#include "nrf_delay.h"

/* Core */
#include "app_uart.h"
#include "nrf_uart.h"
#include "nrf_mesh_configure.h"
#include "nrf_mesh.h"
#include "mesh_stack.h"
#include "device_state_manager.h"
#include "access_config.h"
#include "sdk_config.h"
#include "mesh_adv.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_soc.h"
#include "ble_conn_params.h"
#include "ble_hci.h"
#include "proxy.h"

#include "rssi_filter.h"
/* Provisioning and configuration */
#include "mesh_provisionee.h"
#include "mesh_app_utils.h"
#include "mesh_softdevice_init.h"

/* Models */
#include "generic_onoff_client.h"
#include "app_onoff.h"

/* Logging and RTT */
#include "log.h"
#include "rtt_input.h"

/* Example specific includes */
#include "app_config.h"
#include "nrf_mesh_config_examples.h"
#include "light_switch_example_common.h"
#include "example_common.h"

/*  User Include File */
#include "localization.h"

#define APP_STATE_OFF                (0)
#define APP_STATE_ON                 (1)

#define APP_UNACK_MSG_REPEAT_COUNT   (2)

#define DEVICE_NAME                     "Mesh Client Location"
#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(150,  UNIT_1_25_MS)           /**< Minimum acceptable connection interval. */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(250,  UNIT_1_25_MS)           /**< Maximum acceptable connection interval. */
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds). */
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(100)                        /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called. */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(2000)                       /**< Time between each call to sd_ble_gap_conn_param_update after the first call. */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                          /**< Number of attempts before giving up the connection parameter negotiation. */

#define UART_TX_BUF_SIZE 256                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE 256                         /**< UART RX buffer size. */
#define UART_HWFC APP_UART_FLOW_CONTROL_DISABLED
#define XY_STRING_MAX_LENGTH 7

APP_TIMER_DEF(sending);
APP_TIMER_DEF(rssiGET);
APP_TIMER_DEF(estimating);
APP_TIMER_DEF(uart_receive_timeout);

uint8_t led;
bool sendFlag= false;
Position position;
uint8_t uart_receive;
uint8_t server_string[10];

void uart_send(const char* str, uint32_t delay);
void printPosition(Position *p);
uint32_t ServerStringCheck(uint8_t *str);
void uart_receive_timeout_handler();

uint8_t server_flag = 0;
uint32_t server_str_count = 0;

static void gap_params_init(void);
static void conn_params_init(void);

static void on_sd_evt(uint32_t sd_evt, void * p_context)
{
    (void) nrf_mesh_on_sd_evt(sd_evt);
}

NRF_SDH_SOC_OBSERVER(mesh_observer, NRF_SDH_BLE_STACK_OBSERVER_PRIO, on_sd_evt, NULL);

static generic_onoff_client_t m_clients[CLIENT_MODEL_INSTANCE_COUNT];


static bool m_device_provisioned;

/* Forward declaration */
static void app_gen_onoff_client_publish_interval_cb(access_model_handle_t handle, void * p_self);
static void app_generic_onoff_client_status_cb(const generic_onoff_client_t * p_self,
                                               const access_message_rx_meta_t * p_meta,
                                               const generic_onoff_status_params_t * p_in);
static void app_rssiGET_client_status_cb(const generic_onoff_client_t * p_self,
                                               const access_message_rx_meta_t * p_meta,
                                               const rssiGET_status_params_t * p_in);
static void app_gen_onoff_client_transaction_status_cb(access_model_handle_t model_handle,
                                                       void * p_args,
                                                       access_reliable_status_t status);

const generic_onoff_client_callbacks_t client_cbs =
{
    .onoff_status_cb = app_generic_onoff_client_status_cb,
    .rssiCB = app_rssiGET_client_status_cb,
    .ack_transaction_status_cb = app_gen_onoff_client_transaction_status_cb,
    .periodic_publish_cb = app_gen_onoff_client_publish_interval_cb
};

static void provisioning_complete_cb(void)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Successfully provisioned\n");

    /* Restores the application parameters after switching from the Provisioning service to the Proxy  */
    gap_params_init();
    conn_params_init();

    dsm_local_unicast_address_t node_address;
    dsm_local_unicast_addresses_get(&node_address);
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Node Address: 0x%04x \n", node_address.address_start);

    hal_led_mask_set(LEDS_MASK, LED_MASK_STATE_OFF);
    hal_led_blink_ms(LEDS_MASK, LED_BLINK_INTERVAL_MS, LED_BLINK_CNT_PROV);
}

/* This callback is called periodically if model is configured for periodic publishing */
static void app_gen_onoff_client_publish_interval_cb(access_model_handle_t handle, void * p_self)
{
     __LOG(LOG_SRC_APP, LOG_LEVEL_WARN, "Publish desired message here.\n");
}

/* Acknowledged transaction status callback, if acknowledged transfer fails, application can
* determine suitable course of action (e.g. re-initiate previous transaction) by using this
* callback.
*/
static void app_gen_onoff_client_transaction_status_cb(access_model_handle_t model_handle,
                                                       void * p_args,
                                                       access_reliable_status_t status)
{
    switch(status)
    {
        case ACCESS_RELIABLE_TRANSFER_SUCCESS:
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Acknowledged transfer success.\n");
            break;

        case ACCESS_RELIABLE_TRANSFER_TIMEOUT:
            hal_led_blink_ms(LEDS_MASK, LED_BLINK_SHORT_INTERVAL_MS, LED_BLINK_CNT_NO_REPLY);
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Acknowledged transfer timeout.\n");
            break;

        case ACCESS_RELIABLE_TRANSFER_CANCELLED:
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Acknowledged transfer cancelled.\n");
            break;

        default:
            ERROR_CHECK(NRF_ERROR_INTERNAL);
            break;
    }
}

/* Generic OnOff client model interface: Process the received status message in this callback */
static void app_generic_onoff_client_status_cb(const generic_onoff_client_t * p_self,
                                               const access_message_rx_meta_t * p_meta,
                                               const generic_onoff_status_params_t * p_in)
{
    if (p_in->remaining_time_ms > 0)
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "OnOff server: 0x%04x, Present OnOff: %d, Target OnOff: %d, Remaining Time: %d ms\n",
              p_meta->src.value, p_in->present_on_off, p_in->target_on_off, p_in->remaining_time_ms);
    }
    else
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "OnOff server: 0x%04x, Present OnOff: %d\n",
              p_meta->src.value, p_in->present_on_off);
    }

     __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "RSSI: %d\n", p_meta->p_core_metadata->params.scanner.rssi);

}
void rssiGET_timer()
{
        rssiGET_set_params_t rssiGET_params;
        model_transition_t transition_params;
        transition_params.delay_ms = APP_CONFIG_ONOFF_DELAY_MS;
        transition_params.transition_time_ms = APP_CONFIG_ONOFF_TRANSITION_TIME_MS;
        uint8_t tid = 0;
    
        rssiGET_params.reverse = 0;
        rssiGET_params.tid = tid++;
        (void)rssiGET_client_set_unack(&m_clients[1], &rssiGET_params, &transition_params, 0); 
}

static void app_rssiGET_client_status_cb(const generic_onoff_client_t * p_self,
                                               const access_message_rx_meta_t * p_meta,
                                               const rssiGET_status_params_t * p_in)
{
    
    static uint8_t nSample[APP_LOCALIZE_NODE_NUMBER] = {0};
    static uint8_t enough_sample_node_count = 0;
    static int32_t rssi_vector[APP_LOCALIZE_NODE_NUMBER][APP_RSSI_SAMPLE_NUMBER];
    static uint32_t *localize_node_address_buffer;
    static bool turn = true;
    static localize_node_info nodeInfo[APP_LOCALIZE_NODE_NUMBER];
    // allocate new address buffer
    if (turn) 
    {
        //avoid multiple interrupt when code is execute
        turn = false;
        if (localize_node_address_buffer == NULL) 
        {
            localize_node_address_buffer = (int32_t*)calloc(APP_LOCALIZE_NODE_NUMBER, sizeof(int32_t));
        }
       
        // 
        if (p_in->LocalizeNodeAddress != 1 && !is_address_in_buffer(localize_node_address_buffer, APP_LOCALIZE_NODE_NUMBER, p_in->LocalizeNodeAddress))
        {
            add_address_to_buffer(localize_node_address_buffer, APP_LOCALIZE_NODE_NUMBER, p_in->LocalizeNodeAddress);
        }
    
        int32_t rssi = p_meta->p_core_metadata->params.scanner.rssi;


        for (uint8_t i = 0; i < APP_LOCALIZE_NODE_NUMBER; i++)
        {
            // index i trong buffer = index cua vector
            if (p_in->LocalizeNodeAddress == localize_node_address_buffer[i])
            {
                // if vector is not enough
                if (nSample[i] < APP_RSSI_SAMPLE_NUMBER) { 
                    rssi_vector[i][nSample[i]] = rssi;
                    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "RSSI GET FROM %d : %d ---- %d vector ----- Sample: %d\n",
                            localize_node_address_buffer[i], rssi_vector[i][nSample[i]], i, nSample[i]+1);   
                    nSample[i]++;

                    if (nSample[i] >= APP_RSSI_SAMPLE_NUMBER) 
                    {
                        //printRssiVector(rssi_vector[i], p_in->LocalizeNodeAddress);
                        enough_sample_node_count++;

                        nodeInfo[i].node_address = localize_node_address_buffer[i];
                        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Address %d is FULL : %d\n", nodeInfo[i].node_address, i);
                    }
                    break;
                }
            }
        }
    
        if (enough_sample_node_count >= APP_LOCALIZE_NODE_NUMBER) 
        {
            APP_ERROR_CHECK(app_timer_stop(rssiGET));  
            for (uint8_t i = 0; i < APP_LOCALIZE_NODE_NUMBER; i++)
            {
                free(localize_node_address_buffer);
                localize_node_address_buffer = NULL;
                nodeInfo[i].distance = printRssiVector(rssi_vector[i], nodeInfo[i].node_address);
                assign_position(&nodeInfo[i]);
                                        nSample[i] = 0;

                __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Distance to %d: %d\n", nodeInfo[i].node_address,  nodeInfo[i].distance);    
            }
            position = getPosition(nodeInfo);
            for (uint8_t i = 0; i < APP_LOCALIZE_NODE_NUMBER; i++)
            {
                nodeInfo[i].node_address = 0;    
            }
            enough_sample_node_count = 0;
            sendFlag = true;
        }

        turn = true;
    }
}

static void handler()
{
    static uint8_t a = 0; 
    a ^= 1;
    nrf_gpio_pin_write(led,a);

    if (&m_clients[0] != NULL)
    {
        uint32_t status = NRF_SUCCESS;
        generic_onoff_set_params_t set_params;
        model_transition_t transition_params;
        static uint8_t tid = 0;

        set_params.on_off = a;
        set_params.tid = tid++;
        transition_params.delay_ms = APP_CONFIG_ONOFF_DELAY_MS;
        transition_params.transition_time_ms = APP_CONFIG_ONOFF_TRANSITION_TIME_MS;

        status = generic_onoff_client_set_unack(&m_clients[0], &set_params,
                                            &transition_params, APP_UNACK_MSG_REPEAT_COUNT);
    }

}

static void auto_estimate_timer()
{
    APP_ERROR_CHECK(app_timer_start(rssiGET, APP_TIMER_TICKS(SAMPLE_RATE), NULL)); 
}

static void auto_estimate() {
    static bool estimate_on = false;
    estimate_on ^= 1;
    if (estimate_on) {
        APP_ERROR_CHECK(app_timer_start(estimating, APP_TIMER_TICKS(5500), NULL)); 
        
    }
    else {
                    APP_ERROR_CHECK(app_timer_stop(estimating)); 

    }
}

static void node_reset(void)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "----- Node reset  -----\n");
    hal_led_blink_ms(LEDS_MASK, LED_BLINK_INTERVAL_MS, LED_BLINK_CNT_RESET);
    /* This function may return if there are ongoing flash operations. */
    mesh_stack_device_reset();
}

static void config_server_evt_cb(const config_server_evt_t * p_evt)
{
    if (p_evt->type == CONFIG_SERVER_EVT_NODE_RESET)
    {
        node_reset();
    }
}

static void button_event_handler(uint32_t button_number)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Button %u pressed\n", button_number);

    APP_ERROR_CHECK(app_timer_stop(sending));  
      
    uint32_t delay_ms;
    uint32_t status = NRF_SUCCESS;
    generic_onoff_set_params_t set_params;
    location_set_params_t location_params;
    model_transition_t transition_params;
    static uint8_t tid = 0;

    /* Button 1: ON, Button 2: Off, Client[0]
     * Button 2: ON, Button 3: Off, Client[1]
     */

    set_params.tid = tid++;
    transition_params.delay_ms = APP_CONFIG_ONOFF_DELAY_MS;
    transition_params.transition_time_ms = APP_CONFIG_ONOFF_TRANSITION_TIME_MS;

    switch(button_number)
    {
        case 0:
            delay_ms = 500; 
            led = BSP_LED_0;
            set_params.on_off = BSP_LED_0;
            status = generic_onoff_client_set(&m_clients[0], &set_params,
                                            &transition_params);
            break;
        case 2:
            /*location_params.val = 2500;
            location_params.tid++;
            status = location_client_set(&m_clients[2], &location_params, &transition_params, APP_UNACK_MSG_REPEAT_COUNT);
            break;*/              
        case 1:
        auto_estimate();
            //APP_ERROR_CHECK(app_timer_start(rssiGET, APP_TIMER_TICKS(SAMPLE_RATE), NULL)); 
            break;
        case 3:
            set_params.on_off = BSP_LED_3;
            hal_led_pin_set(BSP_LED_3,1);
            status = generic_onoff_client_set(&m_clients[0], &set_params,
                                            &transition_params);
            APP_ERROR_CHECK(app_timer_stop(sending)); 
            break;

    }  

    
    if (button_number == 0) {
        APP_ERROR_CHECK(app_timer_start(sending, APP_TIMER_TICKS(delay_ms), NULL)); 
        hal_led_pin_set(BSP_LED_3,0);
    }
   // __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Sending msg: ONOFF SET %d\n", set_params.on_off);

//    switch (button_number)
//    {
//        case 0:
//        case 1:
//            /* Demonstrate acknowledged transaction, using 1st client model instance */
//            /* In this examples, users will not be blocked if the model is busy */
//            (void)access_model_reliable_cancel(m_clients[0].model_handle);
//            status = generic_onoff_client_set(&m_clients[0], &set_params, &transition_params);
//            hal_led_pin_set(BSP_LED_0, set_params.on_off);
//            break;
//
//        case 2:
//        case 3:
//            /* Demonstrate un-acknowledged transaction, using 2nd client model instance */
//            status = generic_onoff_client_set_unack(&m_clients[1], &set_params,
//                                                    &transition_params, APP_UNACK_MSG_REPEAT_COUNT);
//            hal_led_pin_set(BSP_LED_1, set_params.on_off);
//            break;
//      }
    
    switch (status)
    {
        case NRF_SUCCESS:
            break;

        case NRF_ERROR_NO_MEM:
        case NRF_ERROR_BUSY:
        case NRF_ERROR_INVALID_STATE:
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Client %u cannot send\n", button_number);
            hal_led_blink_ms(LEDS_MASK, LED_BLINK_SHORT_INTERVAL_MS, LED_BLINK_CNT_NO_REPLY);
            break;

        case NRF_ERROR_INVALID_PARAM:
            /* Publication not enabled for this client. One (or more) of the following is wrong:
             * - An application key is missing, or there is no application key bound to the model
             * - The client does not have its publication state set
             *
             * It is the provisioner that adds an application key, binds it to the model and sets
             * the model's publication state.
             */
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Publication not configured for client %u\n", button_number);
            break;

        default:
            ERROR_CHECK(status);
            break;
    }
}

static void rtt_input_handler(int key)
{
    if (key >= '0' && key <= '3')
    {
        uint32_t button_number = key - '0';
        button_event_handler(button_number);
    }
}

static void models_init_cb(void)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Initializing and adding models\n");

    for (uint32_t i = 0; i < CLIENT_MODEL_INSTANCE_COUNT; ++i)
    {
        m_clients[i].settings.p_callbacks = &client_cbs;
        m_clients[i].settings.timeout = 0;
        m_clients[i].settings.force_segmented = APP_CONFIG_FORCE_SEGMENTATION;
        m_clients[i].settings.transmic_size = APP_CONFIG_MIC_SIZE;

        ERROR_CHECK(generic_onoff_client_init(&m_clients[i], i + 1));
    }
    

}

static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(p_evt->conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
    else if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_SUCCEEDED)
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Successfully updated connection parameters\n");
    }
}

static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

static void mesh_init(void)
{
    uint8_t dev_uuid[NRF_MESH_UUID_SIZE];
    uint8_t node_uuid_prefix[NODE_UUID_PREFIX_LEN] = CLIENT_NODE_UUID_PREFIX;

    ERROR_CHECK(mesh_app_uuid_gen(dev_uuid, node_uuid_prefix, NODE_UUID_PREFIX_LEN));
    mesh_stack_init_params_t init_params =
    {
        .core.irq_priority       = NRF_MESH_IRQ_PRIORITY_LOWEST,
        .core.lfclksrc           = DEV_BOARD_LF_CLK_CFG,
        .core.p_uuid             = dev_uuid,
        .models.models_init_cb   = models_init_cb,
        .models.config_server_cb = config_server_evt_cb
    };
    ERROR_CHECK(mesh_stack_init(&init_params, &m_device_provisioned));
}

static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *) DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);
}

static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;
    ble_gap_conn_params_t  gap_conn_params;

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));
    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);

    memset(&cp_init, 0, sizeof(cp_init));
    cp_init.p_conn_params                  = &gap_conn_params;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}

//
//  UART
//

void uart_error_handle(app_uart_evt_t * p_event)
{         


    if (p_event->evt_type == APP_UART_COMMUNICATION_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_communication);
    }
    else if (p_event->evt_type == APP_UART_FIFO_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_code);
    }

    if (p_event->evt_type == APP_UART_DATA) 
    {
        app_uart_get(&uart_receive);
        if (server_flag == 1) {
            server_string[server_str_count] = uart_receive;
            server_str_count++;
        }
        if (uart_receive == '@') {
            server_flag = 1;
            APP_ERROR_CHECK(app_timer_start(uart_receive_timeout, APP_TIMER_TICKS(500), NULL)); 
        }
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "----- BLE Mesh Light Switch Client Demo -----\n");

    }
}

void inituart() {
uint32_t err_code;

    const app_uart_comm_params_t comm_params =
      {
          RX_PIN_NUMBER,
          TX_PIN_NUMBER,
          RTS_PIN_NUMBER,
          CTS_PIN_NUMBER,
          UART_HWFC,
          false,
#if defined (UART_PRESENT)
          NRF_UART_BAUDRATE_9600
#else
          NRF_UARTE_BAUDRATE_115200
#endif
      };

    APP_UART_FIFO_INIT(&comm_params,
                         UART_RX_BUF_SIZE,
                         UART_TX_BUF_SIZE,
                         uart_error_handle,
                         APP_IRQ_PRIORITY_LOWEST,
                         err_code);

    APP_ERROR_CHECK(err_code); 

}

static void initialize(void)
{
    __LOG_INIT(LOG_SRC_APP | LOG_SRC_ACCESS, LOG_LEVEL_INFO, LOG_CALLBACK_DEFAULT);
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "----- BLE Mesh Light Switch Client Demo -----\n");

    ERROR_CHECK(app_timer_init());
    APP_ERROR_CHECK(app_timer_create(&sending, APP_TIMER_MODE_REPEATED, handler));
    APP_ERROR_CHECK(app_timer_create(&rssiGET, APP_TIMER_MODE_REPEATED, rssiGET_timer));
    APP_ERROR_CHECK(app_timer_create(&estimating, APP_TIMER_MODE_REPEATED, auto_estimate_timer));
    APP_ERROR_CHECK(app_timer_create(&uart_receive_timeout, APP_TIMER_MODE_REPEATED, uart_receive_timeout_handler));

    hal_leds_init();
    
    inituart();

#if BUTTON_BOARD
    ERROR_CHECK(hal_buttons_init(button_event_handler));
#endif
    uint32_t err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);
#if defined S140 // todo remove that after S140 priority fixing
    softdevice_irq_priority_checker();
#endif

    uint32_t ram_start = 0;
    /* Set the default configuration (as defined through sdk_config.h). */
    err_code = nrf_sdh_ble_default_cfg_set(MESH_SOFTDEVICE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    gap_params_init();
    conn_params_init();

    mesh_init();
    bearer_rssi_filtering_set(-100);
}

static void start(void)
{
    rtt_input_enable(rtt_input_handler, RTT_INPUT_POLL_PERIOD_MS);
    ERROR_CHECK(mesh_stack_start());

    if (!m_device_provisioned)
    {
        static const uint8_t static_auth_data[NRF_MESH_KEY_SIZE] = STATIC_AUTH_DATA;
        mesh_provisionee_start_params_t prov_start_params =
        {
            .p_static_data    = static_auth_data,
            .prov_complete_cb = provisioning_complete_cb,
            .p_device_uri = NULL
        };
        ERROR_CHECK(mesh_provisionee_prov_start(&prov_start_params));
    }

    const uint8_t *p_uuid = nrf_mesh_configure_device_uuid_get();
    __LOG_XB(LOG_SRC_APP, LOG_LEVEL_INFO, "Device UUID ", p_uuid, NRF_MESH_UUID_SIZE);

    hal_led_mask_set(LEDS_MASK, LED_MASK_STATE_OFF);
    hal_led_blink_ms(LEDS_MASK, LED_BLINK_INTERVAL_MS, LED_BLINK_CNT_START);
}

int main(void)
{
    initialize();
    execution_start(start);
    uart_send("AT\r\n",1000);
    
    for (;;)
    {
        (void)sd_app_evt_wait();
        if (sendFlag) {
            printPosition(&position);
            sendFlag = false;
        }
    }
}

void uart_send(const char* str, uint32_t delay) 
{
    while (*str != 0) 
    {
        while(app_uart_put(*str) != NRF_SUCCESS);
        str++;
    }
    nrf_delay_ms(delay);
}

void printPosition(Position *p) 
{
    char str[7] = {0};
    uint8_t length;
    
    if (p != NULL) {
      uart_send("AT+CIPSEND=7\r\n",500);
        
      sprintf(str, "%d,%d", p->x, p->y);
      
      for (uint32_t i = 0; i < 7; i++) {
          if (str[i] == 0) str[i] = 32;
      }

      uart_send(str, 500);
    }

}

uint32_t ServerStringCheck(uint8_t *str) 
{
    if(str[2] == 'P' && str[2] == 'D' && str[3] == ',') 
    {
        return 1;
    }
    return 0;
}

void uart_receive_timeout_handler()
{
    int8_t LED_index = 0;
    int8_t LED_command;
    APP_ERROR_CHECK(app_timer_stop(uart_receive_timeout));
    
    for (uint8_t i = 0; i < server_str_count - 1; i++) {
        if (server_string[i] != '-') {
            if (server_string[i+1] == '-') {
                LED_index += server_string[i] - 48;
            }
            else {
                LED_index += (server_string[i] - 48)*10;
            }
        }
        else {
            LED_command = server_string[i+1] - 48;
        }
    }
    
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "%s\n", server_string);
    server_str_count = 0;
    server_flag = 0;
    if (LED_index > 2)
    {
        generic_onoff_set_params_t set_params;
        model_transition_t transition_params;
        static uint8_t tid = 0;

        set_params.on_off = LED_command;
        set_params.tid = tid++;
        transition_params.delay_ms = APP_CONFIG_ONOFF_DELAY_MS;
        transition_params.transition_time_ms = APP_CONFIG_ONOFF_TRANSITION_TIME_MS;

        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "LED Index:%d\n", LED_index);
         __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Command %d\n", LED_command);


        generic_onoff_client_set_unack(&m_clients[LED_index], &set_params,
                                            &transition_params, APP_UNACK_MSG_REPEAT_COUNT);
                                            nrf_gpio_pin_write(BSP_LED_1, 0);
    }
}