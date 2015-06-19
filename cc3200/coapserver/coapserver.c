/*
 * This file is part of the Micro Python project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2015 Daniel Campora
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include <stdint.h>

#include "py/mpconfig.h"
#include MICROPY_HAL_H
#include "py/obj.h"
#include "coapserver.h"
#include "simplelink.h"
#include "modwlan.h"
#include "debug.h"
#include "mpexception.h"
#include "serverstask.h"
#include "genhdr/mpversion.h"

#include "picocoap/coap.h"

/******************************************************************************
 DEFINE PRIVATE CONSTANTS
 ******************************************************************************/
#define COAP_PORT                         5683

#define COAP_RX_BUFFER_SIZE               256
#define COAP_MAX_CLIENTS                  1
#define COAP_TX_RETRIES_MAX               25
#define COAP_WAIT_TIME_MS                 5
#define COAP_LOGIN_RETRIES_MAX            3
#define COAP_TIMEOUT_MS                   300000        // 5 minutes
#define COAP_CYCLE_TIME_MS                (SERVERS_CYCLE_TIME_MS * 2)

/******************************************************************************
 DEFINE PRIVATE TYPES
 ******************************************************************************/
typedef enum {
    E_COAP_RESULT_OK = 0,
    E_COAP_RESULT_AGAIN,
    E_COAP_RESULT_FAILED
} coap_result_t;

typedef enum {
    E_COAP_STE_DISABLED = 0,
    E_COAP_STE_START,
    E_COAP_STE_LISTEN,
    E_COAP_STE_CONNECTED,
    E_COAP_STE_LOGGED_IN
} coap_state_t;

typedef enum {
    E_COAP_STE_SUB_WELCOME,
    E_COAP_STE_SUB_SND_USER_OPTIONS,
    E_COAP_STE_SUB_REQ_USER,
    E_COAP_STE_SUB_GET_USER,
    E_COAP_STE_SUB_REQ_PASSWORD,
    E_COAP_STE_SUB_SND_PASSWORD_OPTIONS,
    E_COAP_STE_SUB_GET_PASSWORD,
    E_COAP_STE_SUB_INVALID_LOGGIN,
    E_COAP_STE_SUB_SND_REPL_OPTIONS,
    E_COAP_STE_SUB_LOGGIN_SUCCESS
} coap_connected_substate_t;

typedef union {
    coap_connected_substate_t connected;
} coap_substate_t;

typedef struct {
    uint8_t             *rxBuffer;
    uint32_t            timeout;
    coap_state_t      state;
    coap_substate_t   substate;
    int16_t             sd;
    int16_t             n_sd;

    // rxRindex and rxWindex must be uint8_t and COAP_RX_BUFFER_SIZE == 256
    uint8_t             rxWindex;
    uint8_t             rxRindex;

    uint8_t             txRetries;
    uint8_t             logginRetries;
    bool                enabled;
    bool                credentialsValid;
} coap_data_t;

/******************************************************************************
 DECLARE PRIVATE DATA
 ******************************************************************************/
static coap_data_t coap_data;
static const char* coap_welcome_msg       = "Micro Python " MICROPY_GIT_TAG " on " MICROPY_BUILD_DATE "; " MICROPY_HW_BOARD_NAME " with " MICROPY_HW_MCU_NAME "\r\n";
static const char* coap_request_user      = "Login as:";
static const char* coap_request_password  = "Password:";
static const char* coap_invalid_loggin    = "\r\nInvalid credentials, try again.\r\n";
static const char* coap_loggin_success    = "\r\nLogin succeeded!\r\nType \"help()\" for more information.\r\n";
static const uint8_t coap_options_user[]  = // IAC   WONT ECHO IAC   WONT SUPPRESS_GO_AHEAD IAC  WILL LINEMODE
                                               { 255,  252,   1, 255,  252,       3,          255, 251,   34 };
static const uint8_t coap_options_pass[]  = // IAC   WILL ECHO IAC   WONT SUPPRESS_GO_AHEAD IAC  WILL LINEMODE
                                               { 255,  251,   1, 255,  252,       3,          255, 251,   34 };
static const uint8_t coap_options_repl[]  = // IAC   WILL ECHO IAC   WILL SUPPRESS_GO_AHEAD IAC  WONT LINEMODE
                                               { 255,  251,   1, 255,  251,       3,          255, 252,   34 };

/******************************************************************************
 DECLARE PRIVATE FUNCTIONS
 ******************************************************************************/
static void coap_wait_for_enabled (void);
static bool coap_create_socket (void);
static void coap_wait_for_connection (void);
static void coap_send_and_proceed (void *data, _i16 Len, coap_connected_substate_t next_state);
static coap_result_t coap_send_non_blocking (void *data, _i16 Len);
static coap_result_t coap_recv_text_non_blocking (void *buff, _i16 Maxlen, _i16 *rxLen);
static void coap_process (void);
static int coap_process_credential (char *credential, _i16 rxLen);
static void coap_parse_input (uint8_t *str, int16_t *len);
static bool coap_send_with_retries (int16_t sd, const void *pBuf, int16_t len);
static void coap_reset (void);
static void coap_reset_buffer (void);

/******************************************************************************
 DEFINE PUBLIC FUNCTIONS
 ******************************************************************************/
void coap_init (void) {
    // Allocate memory for the receive buffer (from the RTOS heap)
    ASSERT ((coap_data.rxBuffer = mem_Malloc(COAP_RX_BUFFER_SIZE)) != NULL);
    coap_data.state = E_COAP_STE_DISABLED;
}

void coap_run (void) {
    _i16 rxLen;
    switch (coap_data.state) {
        case E_COAP_STE_DISABLED:
            coap_wait_for_enabled();
            break;
        case E_COAP_STE_START:
            if (coap_create_socket()) {
                coap_data.state = E_COAP_STE_CONNECTED;
            }
            break;
        case E_COAP_STE_CONNECTED:
            coap_recv_text_non_blocking(coap_data.rxBuffer, COAP_RX_BUFFER_SIZE, &rxLen);
            coap_send_and_proceed((void *)coap_request_user, strlen(coap_request_user), E_COAP_STE_SUB_GET_USER);
            break;

        default:
            break;
    }

    if (coap_data.state >= E_COAP_STE_CONNECTED) {
        if (coap_data.timeout++ > (COAP_TIMEOUT_MS / COAP_CYCLE_TIME_MS)) {
            coap_reset();
        }
    }
}

void coap_tx_strn (const char *str, int len) {
    if (len > 0 && coap_data.n_sd > 0) {
        coap_send_with_retries(coap_data.n_sd, str, len);
    }
}

void coap_tx_strn_cooked (const char *str, uint len) {
    int32_t nslen = 0;
    const char *_str = str;

    for (int i = 0; i < len; i++) {
        if (str[i] == '\n') {
            coap_send_with_retries(coap_data.n_sd, _str, nslen);
            coap_send_with_retries(coap_data.n_sd, "\r\n", 2);
            _str += nslen + 1;
            nslen = 0;
        }
        else {
            nslen++;
        }
    }
    if (_str < str + len) {
        coap_send_with_retries(coap_data.n_sd, _str, nslen);
    }
}

bool coap_rx_any (void) {
    return (coap_data.n_sd > 0) ? ((coap_data.rxRindex != coap_data.rxWindex) &&
           (coap_data.state == E_COAP_STE_LOGGED_IN)) : false;
}

int coap_rx_char (void) {
    int rx_char = -1;
    if (coap_data.rxRindex != coap_data.rxWindex) {
        // rxRindex must be uint8_t and COAP_RX_BUFFER_SIZE == 256 so that it wraps around automatically
        rx_char = (int)coap_data.rxBuffer[coap_data.rxRindex++];
    }
    return rx_char;
}

void coap_enable (void) {
    coap_data.enabled = true;
}

void coap_disable (void) {
    coap_reset();
    coap_data.enabled = false;
    coap_data.state = E_COAP_STE_DISABLED;
}

bool coap_is_enabled (void) {
    return coap_data.enabled;
}

bool coap_is_active (void) {
    return (coap_data.state == E_COAP_STE_LOGGED_IN);
}

/******************************************************************************
 DEFINE PRIVATE FUNCTIONS
 ******************************************************************************/
static void coap_wait_for_enabled (void) {
    // Init coap's data
    coap_data.n_sd = -1;
    coap_data.sd   = -1;

    // Check if the coap service has been enabled
    if (coap_data.enabled) {
        coap_data.state = E_COAP_STE_START;
    }
}

static bool coap_create_socket (void) {
    SlSockNonblocking_t nonBlockingOption;
    sockaddr_in         sServerAddress;
    _i16 result;

    // Open a socket for coap
    ASSERT ((coap_data.sd = sl_Socket(AF_INET, SOCK_STREAM, IPPROTO_UDP)) > 0);
    if (coap_data.sd > 0) {
        // Enable non-blocking mode
        nonBlockingOption.NonblockingEnabled = 1;
        ASSERT (sl_SetSockOpt(coap_data.sd, SOL_SOCKET, SL_SO_NONBLOCKING, &nonBlockingOption, sizeof(nonBlockingOption)) == SL_SOC_OK);

        // Bind the socket to a port number
        sServerAddress.sin_family = AF_INET;
        sServerAddress.sin_addr.s_addr = INADDR_ANY;
        sServerAddress.sin_port = htons(COAP_PORT);

        ASSERT (sl_Bind(coap_data.sd, (const SlSockAddr_t *)&sServerAddress, sizeof(sServerAddress)) == SL_SOC_OK);

		return true;
    }

    return false;
}

static void coap_send_and_proceed (void *data, _i16 Len, coap_connected_substate_t next_state) {
    if (E_COAP_RESULT_OK == coap_send_non_blocking(data, Len)) {
        coap_data.substate.connected = next_state;
    }
}

static coap_result_t coap_send_non_blocking (void *data, _i16 Len) {
    int16_t result = sl_Send(coap_data.n_sd, data, Len, 0);

    if (result > 0) {
        coap_data.txRetries = 0;
        return E_COAP_RESULT_OK;
    }
    else if ((COAP_TX_RETRIES_MAX >= ++coap_data.txRetries) && (result == SL_EAGAIN)) {
        return E_COAP_RESULT_AGAIN;
    }
    else {
        // error
        coap_reset();
        return E_COAP_RESULT_FAILED;
    }
}

static coap_result_t coap_recv_text_non_blocking (void *buff, _i16 Maxlen, _i16 *rxLen) {
    *rxLen = sl_RecvFrom(coap_data.sd, buff, Maxlen, 0, (SlSockAddr_t *)&sClientAddress, (SlSocklen_t *)&in_addrSize);
    // if there's data received, parse it
    if (*rxLen > 0) {
        coap_data.timeout = 0;
        coap_parse_input (buff, rxLen);
        if (*rxLen > 0) {
            return E_COAP_RESULT_OK;
        }
    }
    else if (*rxLen != SL_EAGAIN) {
        // error
        coap_reset();
        return E_COAP_RESULT_FAILED;
    }
    return E_COAP_RESULT_AGAIN;
}

static void coap_process (void) {
    _i16 rxLen;
    _i16 maxLen = (coap_data.rxWindex >= coap_data.rxRindex) ? (COAP_RX_BUFFER_SIZE - coap_data.rxWindex) :
                                                                   ((coap_data.rxRindex - coap_data.rxWindex) - 1);
    // to avoid an overrrun
    maxLen = (coap_data.rxRindex == 0) ? (maxLen - 1) : maxLen;

    if (maxLen > 0) {
        if (E_COAP_RESULT_OK == coap_recv_text_non_blocking(&coap_data.rxBuffer[coap_data.rxWindex], maxLen, &rxLen)) {
            // rxWindex must be uint8_t and COAP_RX_BUFFER_SIZE == 256 so that it wraps around automatically
            coap_data.rxWindex = coap_data.rxWindex + rxLen;
        }
    }
}

static int coap_process_credential (char *credential, _i16 rxLen) {
    coap_data.rxWindex += rxLen;
    if (coap_data.rxWindex >= SERVERS_USER_PASS_LEN_MAX) {
        coap_data.rxWindex = SERVERS_USER_PASS_LEN_MAX;
    }

    uint8_t *p = coap_data.rxBuffer + SERVERS_USER_PASS_LEN_MAX;
    // if a '\r' is found, or the length exceeds the max username length
    if ((p = memchr(coap_data.rxBuffer, '\r', coap_data.rxWindex)) || (coap_data.rxWindex >= SERVERS_USER_PASS_LEN_MAX)) {
        uint8_t len = p - coap_data.rxBuffer;

        coap_data.rxWindex = 0;
        if ((len > 0) && (memcmp(credential, coap_data.rxBuffer, MAX(len, strlen(credential))) == 0)) {
            return 1;
        }
        return -1;
    }
    return 0;
}

static void coap_parse_input (uint8_t *str, int16_t *len) {
    int16_t b_len = *len;
    uint8_t *b_str = str;

     coap_pdu msg_recv = {b_str, b_len, COAP_RX_BUFFER_SIZE};
	 
	 if(coap_validate_pkt(&msg_recv) == CE_NONE)
	 {
		 
	 
}

static bool coap_send_with_retries (int16_t sd, const void *pBuf, int16_t len) {
    int32_t retries = 0;
    // abort sending if we happen to be within interrupt context
    if ((HAL_NVIC_INT_CTRL_REG & HAL_VECTACTIVE_MASK) == 0) {
        do {
            _i16 result = sl_Send(sd, pBuf, len, 0);
            if (result > 0) {
                return true;
            }
            else if (SL_EAGAIN != result) {
                return false;
            }
            HAL_Delay (COAP_WAIT_TIME_MS);
        } while (++retries <= COAP_TX_RETRIES_MAX);
    }
    return false;
}

static void coap_reset (void) {
    // close the connection and start all over again
    servers_close_socket(&coap_data.n_sd);
    servers_close_socket(&coap_data.sd);
    coap_data.state = E_COAP_STE_START;
}

static void coap_reset_buffer (void) {
    // erase any characters present in the current line
    memset (coap_data.rxBuffer, '\b', COAP_RX_BUFFER_SIZE / 2);
    coap_data.rxWindex = COAP_RX_BUFFER_SIZE / 2;
    // fake an "enter" key pressed to display the prompt
    coap_data.rxBuffer[coap_data.rxWindex++] = '\r';
}

