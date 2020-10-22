/*!
    \file  gd32fr_wifi.c
    \brief the header file of the machine

    \version 2020-09-16, V1.0.0, firmware for GD32F4xx
*/

/*
    Copyright (c) 2020, VKing.

    All rights reserved.

    Redistribution and use in source and binary forms, with or without modification, 
are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this 
       list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright notice, 
       this list of conditions and the following disclaimer in the documentation 
       and/or other materials provided with the distribution.
    3. Neither the name of the copyright holder nor the names of its contributors 
       may be used to endorse or promote products derived from this software without 
       specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. 
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT 
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR 
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY 
OF SUCH DAMAGE.
*/

#include "gd32fr_wifi.h"
#include "gd32f4xx_usart.h"
#include "FreeRTOS.h"
#include "gd32fr_global.h"

#ifdef WIFI_TASK

#define TXD_PIN (GPIO_PIN_11)
#define RXD_PIN (GPIO_PIN_11) 
uint8_t txATAPCMD[]          = "\n\rAT+CWJAP=<ssid>,<pwd>[,<bssid>][,<pci_en>][,<reconn_interval>][,<listen_interval>][,<scan_mode>]\n\r";
uint8_t txATHostCMD[]        = "\n\rAT+CWJAP=<ssid>,<pwd>[,<bssid>][,<pci_en>][,<reconn_interval>][,<listen_interval>][,<scan_mode>]\n\r";
uint8_t txATCWAUTOCONNCMD[]  = "\n\rAT+CWAUTOCONN=1\n\r";
//uint8_t txATHttpClient[]     = "\n\rAT+HTTPCLIENT=<opt>,<content-type>,[<url>],[<host>],[<path>],<transport_type>,[<data>][,"http_req_header"][,"http_req_header"][...] \n\r";


#define ARRAYNUM(arr_nanme)      (uint32_t)(sizeof(arr_nanme) / sizeof(*(arr_nanme)))
#define TRANSMIT_SIZE            (ARRAYNUM(txATAPCMD) - 1)

uint8_t rxbuffer[32];
static const int RX_BUF_SIZE = 1024; 
__IO uint8_t txcount         = 0; 
__IO uint16_t rxcount        = 0; 
uint8_t tx_size              = TRANSMIT_SIZE;
uint8_t rx_size              = 32;

void wifi_task(void){ 

    printf("wifi task");
    wifi_rcu_init();
    wifi_gpio_init();
    SET_WIFI_POWER_ON

    init();
    xTaskCreate(rx_task, "uart_rx_task", 1024*2, NULL, WIFI_TASK_PRIO, NULL);
    xTaskCreate(tx_task, "uart_tx_task", 1024*2, NULL, WIFI_TASK_PRIO-1, NULL);

    while(1){ 
        //doing nothing now
    }
}
void wifi_rcu_init(void){
    rcu_periph_clock_enable(RCU_GPIOA);
}
void wifi_gpio_init(void){
    //UWB IRQ
    gpio_af_set(GPIOA, GPIO_AF_11, GPIO_PIN_11);
    gpio_mode_set(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, GPIO_PIN_11);
} 

void init(void) {
    /* USART interrupt configuration */
    nvic_irq_enable(USART2_IRQn, 0, 0);
 
    wifi_com_init(EVAL_COM2);
    /* configure COM1 */
    gd_eval_com_init(EVAL_COM2);
    /* enable USART TBE interrupt */  
    usart_interrupt_enable(USART2, USART_INT_TBE);
    usart_interrupt_enable(USART2, USART_INT_RBNE);
 
    #if 0
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    // We won't use a buffer for sending data.
    uart_driver_install(UART_NUM_1, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    #endif
}

int sendData(const char* logName, const char* data) {
    const int len = strlen(data);
    //const int txBytes = uart_write_bytes(UART_NUM_1, data, len);
    //ESP_LOGI(logName, "Wrote %d bytes", txBytes);
    //return txBytes;
	  return 0;
}

//static void tx_task(void *arg) {
static void tx_task(void) {
    static const char *TX_TASK_TAG = "TX_TASK";
    //esp_log_level_set(TX_TASK_TAG, ESP_LOG_INFO);
    while (1) {
        sendData(TX_TASK_TAG, "Hello world");
        vTaskDelay(2000 / portTICK_RATE_MS);
    }
}

static void rx_task(void) {
    static const char *RX_TASK_TAG = "RX_TASK";
    at_enum_status at_status;
    uint8_t* data = (uint8_t*) malloc(RX_BUF_SIZE+1);
    int rxBytes[100];
    switch(at_status) {
        case AP_SETTINGS_OK:  
            rxBytes = uart_read_bytes(USART2, data, RX_BUF_SIZE, 1000 / portTICK_RATE_MS);
        break;
        case AP_SETTINGS_FAIL:
            rxBytes = uart_read_bytes(USART2, data, RX_BUF_SIZE, 1000 / portTICK_RATE_MS);
        break;
        case CIP_UDP_CONNECT:
            rxBytes = uart_read_bytes(USART2, data, RX_BUF_SIZE, 1000 / portTICK_RATE_MS);
        break;
        case CIP_UDPDATA_RECEIVED:
            rxBytes = uart_read_bytes(USART2, data, RX_BUF_SIZE, 1000 / portTICK_RATE_MS);
        break;
        case CIP_UDPDATA_FAILD:
            rxBytes = uart_read_bytes(USART2, data, RX_BUF_SIZE, 1000 / portTICK_RATE_MS);
        break;
        default:
        break;
    }
        #if 0
        const int rxBytes = uart_read_bytes(UART_NUM_1, data, RX_BUF_SIZE, 1000 / portTICK_RATE_MS);
        const int rxBytes = uart_read_bytes(UART_NUM_1, data, RX_BUF_SIZE, 1000 / portTICK_RATE_MS);
        if (rxBytes > 0) {
            data[rxBytes] = 0;
            ESP_LOGI(RX_TASK_TAG, "Read %d bytes: '%s'", rxBytes, data);
            ESP_LOG_BUFFER_HEXDUMP(RX_TASK_TAG, data, rxBytes, ESP_LOG_INFO);
        }
        #endif
    }
    free(data);
}

void app_main(void){
    }

void wifi_com_init(uint32_t com){
    /* enable GPIO clock */
    uint32_t COM_ID = 0;
    if(EVAL_COM2 == com)
    {
        COM_ID = 0U;
    }

    rcu_periph_clock_enable(EVAL_COM2_GPIO_CLK);

    /* enable USART clock */
    rcu_periph_clock_enable(COM_CLK[COM_ID]);

    /* connect port to USARTx_Tx */
    gpio_af_set(EVAL_COM2_GPIO_PORT, EVAL_COM2_AF, COM_TX_PIN[COM_ID]);

    /* connect port to USARTx_Rx */
    gpio_af_set(EVAL_COM2_GPIO_PORT, EVAL_COM2_AF, COM_RX_PIN[COM_ID]);

    /* configure USART Tx as alternate function push-pull */
    gpio_mode_set(EVAL_COM2_GPIO_PORT, GPIO_MODE_AF, GPIO_PUPD_PULLUP,COM_TX_PIN[COM_ID]);
    gpio_output_options_set(EVAL_COM2_GPIO_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ,COM_TX_PIN[COM_ID]);

    /* configure USART Rx as alternate function push-pull */
    gpio_mode_set(EVAL_COM2_GPIO_PORT, GPIO_MODE_AF, GPIO_PUPD_PULLUP,COM_RX_PIN[COM_ID]);
    gpio_output_options_set(EVAL_COM2_GPIO_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ,COM_RX_PIN[COM_ID]);

    /* USART configure */
    usart_deinit(com);
    usart_baudrate_set(com,115200U);
    usart_receive_config(com, USART_RECEIVE_ENABLE);
    usart_transmit_config(com, USART_TRANSMIT_ENABLE);
    usart_enable(com);
}

/*!
    \brief      this function handles USART RBNE interrupt request and TBE interrupt request
    \param[in]  none
    \param[out] none
    \retval     none
*/
void USART2_IRQHandler(void) {
    if((RESET != usart_interrupt_flag_get(USART0, USART_INT_FLAG_RBNE)) && 
       (RESET != usart_flag_get(USART0, USART_FLAG_RBNE))){
        /* receive data */
        rxbuffer[rxcount++] = usart_data_receive(USART2);
        if(rxcount == rx_size){
            usart_interrupt_disable(USART2, USART_INT_RBNE);
        }
    }
    if((RESET != usart_flag_get(USART2, USART_FLAG_TBE)) && 
       (RESET != usart_interrupt_flag_get(USART2, USART_INT_FLAG_TBE))){
           
        /* transmit data */
        usart_data_transmit(USART2, txATAPCMD[txcount++]);
        if(txcount == tx_size){
            usart_interrupt_disable(USART2, USART_INT_TBE);
        }
    }
    printf("USART2 IRQ");

}
#endif 
