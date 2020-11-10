/*!
    \file  gd32fr_wifi.h
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

#ifndef GD32FR_WIFI_H
#define GD32FR_WIFI_H

//#include "gd32f450z_eval.h"
#include "gd32f450i_eval.h"
#include "gd32fr_global.h"
//#include "gd32f4xx_rcu.h"
#include "stdio.h"
#include "queue.h"

#ifdef WIFI_TASK //setting wifi devices

#define EVAL_COM2_TX_PIN                 GPIO_PIN_10
#define EVAL_COM2_RX_PIN                 GPIO_PIN_11

#define SET_WIFI_POWER_ON  gpio_bit_set(GPIOA, GPIO_PIN_11);
#define SET_WIFI_POWER_OFF gpio_bit_reset(GPIOA, GPIO_PIN_11);
#define SET_WIFI_RESET     gpio_bit_reset(GPIOB, GPIO_PIN_0);
#define SET_WIFI_SET       gpio_bit_set(GPIOB, GPIO_PIN_0);

//USART2
#define EVAL_COM2                        USART2
#define EVAL_COM2_CLK                    RCU_USART2 
#define EVAL_COM2_GPIO_PORT              GPIOB
#define EVAL_COM2_GPIO_CLK               RCU_GPIOB
#define EVAL_COM2_AF                     GPIO_AF_7

#define COMn                             1U

static rcu_periph_enum COM_CLK[COMn] = {EVAL_COM2_CLK};
static uint32_t COM_TX_PIN[COMn]     = {EVAL_COM2_TX_PIN};
static uint32_t COM_RX_PIN[COMn]     = {EVAL_COM2_RX_PIN};

typedef enum{
    AP_SETTINGS_OK       =2,
    AP_SETTINGS_FAIL     =3,
    CIP_UDP_CONNECT      =4,
    CIP_UDPDATA_RECEIVED =5,
    CIP_UDPDATA_FAILD    =6
}at_enum_status;

typedef enum{
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
 MT_ATUWBDATA,    //AT uwb data
 MT_WIFISENDOK,   //AT wifi able to send udp data out
 MT_ATNONE        //AT none status
}MainWifiTaskMsg;

static QueueHandle_t s_ATMsgQueue;

void wifi_task(void);
void wifi_gpio_init(void);
void wifi_rcu_init(void);
void eprintf(uint32_t *str);
void wifi_uart_init(void);
void tx_task(void);
void rx_task(void);
void wifi_com_init(uint32_t com);
uint16_t usart_wifi_data_receive(uint32_t usart_periph); 

#endif //WIFI_TASK 

#endif  //GD32FR_WIFI_H
