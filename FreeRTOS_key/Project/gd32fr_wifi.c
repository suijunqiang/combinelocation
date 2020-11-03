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
#include "systick.h"
#include "gd32f450i_eval.h"
#include "queue.h"

#ifdef WIFI_TASK

#define ARRAYNUM(arr_nanme)      (uint32_t)(sizeof(arr_nanme) / sizeof(*(arr_nanme)))
#define TRANSMIT_SIZE            (ARRAYNUM(txATAPCMD) - 1)
#define TXD_PIN (GPIO_PIN_11)
#define RXD_PIN (GPIO_PIN_11) 

static QueueHandle_t s_ATMsgQueue;

typedef enum
{
 MT_ATOK,         //AT cmd done
 MT_ATERROR,      //AT cmd done with error
 MT_ATBUSY,       //AT is busy with error
 MT_ATDATA,       //AT done with data
 MT_ATINIT,       //AT initiallzing
 MT_ATCMDTEST,    //AT test cmd
 MT_ATPAUSE,      //AT in idel status
 MT_ATAPSETTINGS, //AT settings AP info
 MT_ATCIPSTART,   //AT UDP/IP start
 MT_ATECHO0,      //AT echo off
 MT_ATCIPSEND,    //AT CIP udp send 
 MT_ATCIPMUX,     //AT udp single/mult
 MT_ATUDPDATA,    //AT udp data
 MT_ATNONE        //AT none status
}MainWifiTaskMsg;

uint8_t txATCMDTEST[]                    = "AT\r\n";
uint8_t txATAPCMD[]                      = "AT+CWJAP=\"SJQiPhone\",\"sjqjesus\"\r\n";
uint8_t txATHostCMD[]                    = "AT+CWJAP=<ssid>,<pwd>[,<bssid>][,<pci_en>][,<reconn_interval>][,<listen_interval>][,<scan_mode>]\r\n";
uint8_t txATCWAUTOCONNCMD[]              = "AT+CWAUTOCONN=1\r\n";
uint8_t txATCIPSTART[]                   = "AT+CIPSTART=\"UDP\",\"180.156.207.166\",3000\r\n";
uint8_t txATCIPSEND[]                    = "AT+CIPSEND=5\r\n";
uint8_t txATUDPDATA[]                    = "hello";
uint8_t txATCIPMUX[]                     = "AT+CIPMUX=0\r\n";
//AT results
uint8_t txATRESULTOK[]                   = "\r\nOK\r\n";
uint8_t txATRESULTWIFICONNECTED[]        = "WIFI CONNECTED\r\nWIFI GOT IP\r\n";
uint8_t txATRESULTWIFIGOTIP[]            = "WIFI GOT IP\r\n";
uint8_t txATRESULTWIFIDISCONNECTED[]     = "WIFI DISCONNECTED\r\n";
uint8_t txATRESULTUDPCONNECTED[]         = "CONNECT\r\n\r\nOK\r\n";

uint8_t rxbuffer[32];
static const int RX_BUF_SIZE = 1024; 
__IO uint8_t txcount         = 0; 
__IO uint16_t rxcount        = 0; 
uint8_t tx_size              = TRANSMIT_SIZE;
uint8_t rx_size              = 32;

void wifi_task(void){ 
    systick_config(); 

    SET_WIFI_POWER_OFF //as GD said 
    vTaskDelay(5000 / portTICK_RATE_MS);
    SET_WIFI_RESET
    vTaskDelay(5000 / portTICK_RATE_MS);
    gd_eval_com_init(USART2);
    #if 0
    gd_eval_com_init(USART2);
    wifi_uart_init();
    SET_WIFI_RESET
    SET_WIFI_SET
    vTaskDelay(5000 / portTICK_RATE_MS);
    SET_WIFI_RESET
    vTaskDelay(200 / portTICK_RATE_MS);
    #endif 
    xTaskCreate(tx_task, "uart_tx_task", 1024, NULL, WIFI_TASK_PRIO + 1, NULL);
    xTaskCreate(rx_task, "uart_rx_task", 1024*2, NULL, WIFI_TASK_PRIO + 1, NULL);
    //printf("wifi task"); 
}
void wifi_uart_init(void){
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_USART2); 
    //USART2 PA11 power on, PB0 reset, PB10 TX, PB11 RX
    //gpio_af_set(GPIOA, GPIO_AF_7, GPIO_PIN_11); 
    //gpio_af_set(GPIOB, GPIO_AF_7, GPIO_PIN_0);
    gpio_af_set(GPIOB, GPIO_AF_7, GPIO_PIN_10); 
    gpio_af_set(GPIOB, GPIO_AF_7, GPIO_PIN_11); 

    gpio_mode_set(GPIOA,GPIO_MODE_OUTPUT,GPIO_PUPD_NONE,GPIO_PIN_11);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ, GPIO_PIN_11); 
    //wifi reset
    gpio_mode_set(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, GPIO_PIN_0);
    //wifi gpio settings
    /* configure USART Tx as alternate function push-pull */
    gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE,GPIO_PIN_10);
    gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ,GPIO_PIN_10); 
    /* configure USART Rx as alternate function push-pull */
    gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE,GPIO_PIN_11);
    gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ,GPIO_PIN_11); 
    /* configure USART synchronous mode */
    //usart_synchronous_clock_enable(USART2);
    //usart_synchronous_clock_config(USART2, USART_CLEN_EN, USART_CPH_2CK, USART_CPL_HIGH); 
    /* USART configure */
    usart_deinit(USART2);
    usart_baudrate_set(USART2,115200U);
    usart_receive_config(USART2, USART_RECEIVE_ENABLE);
    usart_transmit_config(USART2, USART_TRANSMIT_ENABLE);
    usart_enable(USART2);
    /* enable USART TBE interrupt */  
    //usart_interrupt_enable(USART2, USART_INT_TBE);
    //usart_interrupt_enable(USART2, USART_INT_RBNE); 
} 
int sendData(const char* logName, const char* data) {
    int len = strlen(data);
    int txBytes = len;
    //const int txBytes =usart_data_transmit(USART2, data, len);
    //printf("ATE0\r\n");
    printf(data);
    #if 0
    while(len){
    /* enable GPIO clock */
        usart_data_transmit(USART2, (uint32_t) data++); 
        while(RESET == usart_flag_get(USART2, USART_FLAG_TBE)); 
        len--;
    }
    int i=32;
    int j=0;
    uint16_t tmp[32] = {0};
    while(i--){
            /* wait the byte is entirely received by USART2 */
        while(RESET == usart_flag_get(USART2, USART_FLAG_RBNE)){ }
        tmp[j++]=usart_data_receive(USART2);
    }
		
    //ESP_LOGI(logName, "Wrote %d bytes", txBytes);
    //return txBytes;
    #endif
	return txBytes;
}

static void tx_task(void) {
    static const char *TX_TASK_TAG = "TX_TASK";
    MainWifiTaskMsg event          = MT_ATNONE;
    MainWifiTaskMsg event_pri      = MT_ATNONE;
    s_ATMsgQueue                   = xQueueCreate(20, sizeof(MainWifiTaskMsg));

    printf("ATE0\r\n");
    vTaskDelay(1000 / portTICK_RATE_MS);
    while(1){
        switch(event){
            case MT_ATOK:
                switch(event_pri){
                    case MT_ATCMDTEST:
                        event     = MT_ATCIPMUX;
                        event_pri = event;
                        vTaskDelay(1000 / portTICK_RATE_MS);
                        sendData(TX_TASK_TAG,txATCIPMUX);
                        xQueueReceive(s_ATMsgQueue, &event, portMAX_DELAY);
                        break;
                    case MT_ATECHO0:
                        event     = MT_ATOK;
                        event_pri = event;
                        vTaskDelay(1000 / portTICK_RATE_MS);
                        sendData(TX_TASK_TAG,txATCMDTEST);
                        xQueueReceive(s_ATMsgQueue, &event, portMAX_DELAY);
                        break; 
                    case MT_ATCIPMUX:
                        event     = MT_ATAPSETTINGS;
                        event_pri = event;
                        vTaskDelay(1000 / portTICK_RATE_MS);
                        sendData(TX_TASK_TAG,txATAPCMD);
                        xQueueReceive(s_ATMsgQueue, &event, portMAX_DELAY);
                        break; 
                     case MT_ATCIPSTART:
                        event     = MT_ATCIPSEND;
                        event_pri = event;
                        vTaskDelay(5000 / portTICK_RATE_MS);
                        sendData(TX_TASK_TAG,txATCIPSEND);
                        xQueueReceive(s_ATMsgQueue, &event, portMAX_DELAY);
                        break; 
                      case MT_ATCIPSEND:
                        //event     = MT_ATUDPDATA;
                        event     = MT_ATCIPSTART;
                        event_pri = event;
                        vTaskDelay(1000 / portTICK_RATE_MS);
                        sendData(TX_TASK_TAG,txATUDPDATA);
                        xQueueReceive(s_ATMsgQueue, &event, portMAX_DELAY);
                        break; 
                     case MT_ATAPSETTINGS:
                        event     = MT_ATCIPSTART;
                        event_pri = event;
                        vTaskDelay(5000 / portTICK_RATE_MS);
                        sendData(TX_TASK_TAG,txATCIPSTART);
                        xQueueReceive(s_ATMsgQueue, &event, portMAX_DELAY);
                        break; 
 
                    case MT_ATINIT:
                    case MT_ATPAUSE:
                   case MT_ATERROR:
                    case MT_ATBUSY:
                    default:
                    break;

                }
            break;
            case MT_ATERROR:
            break;
            case MT_ATBUSY:
            break;
            case MT_ATNONE:
                event     = MT_ATCMDTEST;
                event_pri = event;
                vTaskDelay(1000 / portTICK_RATE_MS);
                sendData(TX_TASK_TAG,txATCMDTEST);
                xQueueReceive(s_ATMsgQueue, &event, portMAX_DELAY);
            break;
            case MT_ATDATA:
            break;
            case MT_ATINIT:
            case MT_ATPAUSE:
            default:
                event     = MT_ATNONE;
            break;
        }
        vTaskDelay(1000 / portTICK_RATE_MS);
    }
}

/*!
    \brief      USART receive data function
    \param[in]  usart_periph: USARTx(x=0,1,2,5)/UARTx(x=3,4,6,7)
    \param[out] none
    \retval     data of received
*/
#if 0
uint16_t usart_wifi_data_receive(uint32_t usart_periph)
{
    return (uint16_t)(GET_BITS(USART_DATA(usart_periph), 0U, 8U));
}
#endif

static void rx_task(void) {
    static const char *RX_TASK_TAG = "RX_TASK";
    at_enum_status at_status = AP_SETTINGS_OK;
    uint8_t* data = (uint8_t*) malloc(RX_BUF_SIZE+1);
    static uint8_t RXLEN = 100;
    uint8_t rxBytes[100] ={'\n'};
    uint8_t i = 0;    
    uint8_t j = 0;
    MainWifiTaskMsg event                          = MT_ATNONE;
    while(1){   
        while(RESET == usart_flag_get(USART2, USART_FLAG_RBNE));
        uint8_t tmp[1] = {0};
        rxBytes[i++] = usart_data_receive(USART2);
        if(strstr(rxBytes, txATRESULTUDPCONNECTED)){
            i=0;
            event = MT_ATOK;
            xQueueSend(s_ATMsgQueue, &event, 0); 
        }
        else if(strstr(rxBytes, txATRESULTOK)){
            i=0;
            event = MT_ATOK;
            xQueueSend(s_ATMsgQueue, &event, 0); 
        }
        //else if(strstr(rxBytes, txATRESULTWIFICONNECTED)||strstr(rxBytes, txATRESULTWIFIGOTIP)){
        else if(strstr(rxBytes, txATRESULTWIFICONNECTED)){
            i=0;
            xQueueSend(s_ATMsgQueue, &event, 0); 
        } 
        /*
        else if(strstr(rxBytes, txATCIPSTART)){
            i=0;
            xQueueSend(s_ATMsgQueue, &event, 0); 
        }
        */
        #if 0
        if(rxBytes[i-1]=='\0x0D'){ 
            j++;
            if(i==99){
                i=0;
                xQueueSend(s_ATMsgQueue, &event, 0); 
            }
            //if(j == 2 ){ 
            //    i = 0;
            //}

            //printf(rxBytes);
        }
 
        if(rxBytes[i] == 0x00){ 
            i = 0;
            //printf(rxBytes);
        }
        #endif
        #if 0
        do{                
            rxBytes[i++] = usart_data_receive(USART2);
            if(rxBytes[i] == '\n') {
                break;
            }
            if(i >= RXLEN){
                i = 0;
                printf("WIFI_LOG: you input is overlength\r\n");
                break;
            } 
            while(RESET == usart_flag_get(USART2, USART_FLAG_TBE));
        }while(1);
                        
        if(i < RXLEN) {
            //printf("you input is ");
					printf(rxBytes);
        }
        //i++;
        #endif

        switch(at_status) {
            case AP_SETTINGS_OK:  
                //rxBytes[0] = uart_wifi_data_receive(USART2, data, RX_BUF_SIZE, 1000 / portTICK_RATE_MS);
            break;
            case AP_SETTINGS_FAIL:
                //rxBytes[0] = uart_wifi_data_receive(USART2, data, RX_BUF_SIZE, 1000 / portTICK_RATE_MS);
            break;
            case CIP_UDP_CONNECT:
                //rxBytes[0] = uart_wifi_data_receive(USART2, data, RX_BUF_SIZE, 1000 / portTICK_RATE_MS);
            break;
            case CIP_UDPDATA_RECEIVED:
                //rxBytes[0] = uart_wifi_data_receive(USART2, data, RX_BUF_SIZE, 1000 / portTICK_RATE_MS);
            break;
            case CIP_UDPDATA_FAILD:
                //rxBytes[0] = uart_wifi_data_receive(USART2, data, RX_BUF_SIZE, 1000 / portTICK_RATE_MS);
            break;
            default:
            break;
        }
        #if 0
        const int rxBytes = uart_wifi_data_receive(UART_NUM_1, data, RX_BUF_SIZE, 1000 / portTICK_RATE_MS);
        const int rxBytes = uart_wifi_data_receive(UART_NUM_1, data, RX_BUF_SIZE, 1000 / portTICK_RATE_MS);
        if (rxBytes > 0) {
            data[rxBytes] = 0;
            ESP_LOGI(RX_TASK_TAG, "Read %d bytes: '%s'", rxBytes, data);
            ESP_LOG_BUFFER_HEXDUMP(RX_TASK_TAG, data, rxBytes, ESP_LOG_INFO);
        }
        #endif
        /* code */
    }
   free(data);
}

void app_main(void){
    }

/*!
    \brief      this function handles USART RBNE interrupt request and TBE interrupt request
    \param[in]  none
    \param[out] none
    \retval     none
*/
void USART2_IRQHandler(void) {
    if((RESET != usart_interrupt_flag_get(USART2, USART_INT_FLAG_RBNE)) && 
       (RESET != usart_flag_get(USART2, USART_FLAG_RBNE))){
        /* receive data */
        rxbuffer[rxcount++] = usart_data_receive(USART2);
        if(rxcount == rx_size){
            usart_interrupt_disable(USART2, USART_INT_RBNE);
        }
    }
    if((RESET != usart_flag_get(USART2, USART_FLAG_TBE)) && 
       (RESET != usart_interrupt_flag_get(USART2, USART_INT_FLAG_TBE))){
           
        #if 1
        /* transmit data */
        usart_data_transmit(USART2, txATAPCMD[txcount++]);
        if(txcount == tx_size){
            usart_interrupt_disable(USART2, USART_INT_TBE);
        }
        #endif
    }
    printf("USART2 IRQ\r\n");

}
/* retarget the C library printf function to the USART */
int fputc(int ch, FILE *f)
{
    usart_data_transmit(USART2, (uint8_t)ch);
    while(RESET == usart_flag_get(USART2, USART_FLAG_TBE));
    return ch;
}

int checkstr(char *s, char *t, int flag,int lenstr)
{
    char *q;
    for (; *(s + lenstr); s++) {
        if(flag){ //case not sensetive 
          for (q = t; (*s == *q||*s-32==*q||*s+32==*q) && *q; s++, q++) ; 
        }else{ //case sensitive 
            for (q = t; *s == *q && *q; s++, q++) ;
        } 
        if (!*q) {
            return 1;
        }
    }

    return 0;
} 

#endif 
