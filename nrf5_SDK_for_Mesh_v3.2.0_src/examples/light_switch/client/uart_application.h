#ifndef  _APPLICATION_UART_H_
#define _APPLICATION_UART_H_

/* HAL */
#include "boards.h"
#include "simple_hal.h"
#include "app_timer.h"
#include "nrf_delay.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "app_uart.h"
#include "app_error.h"
#include "nrf_delay.h"
#include "nrf.h"
#if defined (UART_PRESENT)
#include "nrf_uart.h"
#endif
#if defined (UARTE_PRESENT)
#include "nrf_uarte.h"
#endif

#define MAX_TEST_DATA_BYTES     (15U)                /**< max number of test bytes to be used for tx and rx. */
#define UART_TX_BUF_SIZE 256                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE 256                         /**< UART RX buffer size. */

#include "generic_onoff_client.h"
#include "log.h"

void application_uart_init();


#endif /* _APPLICATION_UART_H_*/