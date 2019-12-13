#include "uart_application.h"
/**************************************************
******************* Definition ********************
**************************************************/
#define UART_HWFC APP_UART_FLOW_CONTROL_DISABLED

/**************************************************
************* Global Variables ********************
**************************************************/
uint8_t uart_receive;
uint8_t server_string[15];
uint8_t server_str_count = 0;
uint8_t server_flag = 0;

uint8_t deviceID = 0;
uint8_t serverCommand = 0;
const uint8_t stringStart = 8;

APP_TIMER_DEF(uart_receive_timeout);
void uart_receive_timeout_handler();

static generic_onoff_client_t *m_clients_mirror;

uint32_t ServerStringCheck(uint8_t *str);
uint8_t StringFind(uint8_t *str, uint8_t *target);
/**************************************************
******************* Function **********************
**************************************************/

/******* uart_handler() ********/
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
    else {
        app_uart_get(&uart_receive);
        if (server_flag == 1) {
            server_string[server_str_count] = uart_receive;
            server_str_count++;
        }
        if (uart_receive == '@') {
            server_flag = 1;
            APP_ERROR_CHECK(app_timer_start(uart_receive_timeout, APP_TIMER_TICKS(100), NULL)); 
                        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "String Server Started\n");
        }
    }
}

/******* application_uart_init() ********/
void application_uart_init(generic_onoff_client_t *m_clients)
{
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
          NRF_UART_BAUDRATE_115200
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
    m_clients_mirror = m_clients;
    APP_ERROR_CHECK(app_timer_create(&uart_receive_timeout, APP_TIMER_MODE_SINGLE_SHOT, uart_receive_timeout_handler));
}

/******* uart_timeout() ********/
void uart_receive_timeout_handler()
{
    int8_t LED_index = 0;
    int8_t LED_command;
    
        LED_index += (server_string[0] - 48) * 10;
        LED_index +=  server_string[1] - 48;

        LED_command =  server_string[3] - 48;
    
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "%s\n", server_string);
        server_str_count = 0;
        server_flag = 0;
        if (LED_index > 0)
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

            (void)access_model_reliable_cancel(m_clients_mirror[LED_index].model_handle);
            (void)generic_onoff_client_set(&m_clients_mirror[LED_index], &set_params, &transition_params);    
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

uint8_t StringFind(uint8_t *str, uint8_t *target)
{
    uint8_t found = 0;
    uint8_t i = 1;
    uint8_t j = 1;
    while(str[i] != 0) {
        if (str[i - 1] == target[j - 1] && str[i] == target[j]) {
            found = 1;
            j++;
            if (target[j] == 0) {
                break;
            }
        }
        else {
            found = 0;
            j = 0;
        }
        i++;
    }
    return found;
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

