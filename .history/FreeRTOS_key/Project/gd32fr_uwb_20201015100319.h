/*!
    \file  gd32fr_uwb.h
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

#ifndef GD32FR_UWB_H
#define GD32FR_UWB_H

#include "gd32f450z_eval.h"
#include "gd32fr_global.h"
#include "kcm_driver.h"
#include "stdio.h"

//UWB NSS high is disable
#define SET_SPI1_NSS_HIGH          gpio_bit_set(GPIOA,GPIO_PIN_4);
#define SET_SPI0_NSS_HIGH          gpio_bit_set(GPIOA,GPIO_PIN_4);
//UWB NSS low is enable
#define SET_SPI1_NSS_LOW           gpio_bit_reset(GPIOA,GPIO_PIN_4);
#define SET_SPI0_NSS_LOW           gpio_bit_reset(GPIOA,GPIO_PIN_4);
//UWB power on
#define SET_SPI1_SWITCH_HIGH       gpio_bit_set(GPIOB,GPIO_PIN_15);
//UWB power off
#define SET_SPI1_SWITCH_LOW        gpio_bit_reset(GPIOB,GPIO_PIN_15);
//Set UWB 
#define SET_UWB_SET                gpio_bit_set(GPIOC,GPIO_PIN_5);
//Reset UWB 
#define SET_UWB_RESET              gpio_bit_reset(GPIOC,GPIO_PIN_5);

#ifdef JLINK //BUSY is PA9
    #define KCM_BUSY()                 gpio_input_bit_get(GPIOA, GPIO_PIN_9)
#else
    //BUSY pin is 14 not 13
    #define KCM_BUSY()                 gpio_input_bit_get(GPIOA, GPIO_PIN_14)
#endif

#define NOP 0x00 
#define NOP_NUM 2 
#define DOFFSET (1+NOP_NUM) 
#define REG_W 0x00
#define REG_R 0x80
#define DEVID_REGID 0x03
#define KCM2000_DEV_ID 0x2d9c0501
#define KCM2001_DEV_ID 0x2d9c0502
#define ARRAYSIZE         7
__IO uint32_t send_n = 0, receive_n = 0;
uint8_t spi0_send_array[ARRAYSIZE] = {0x83,0x00,0x00,0x00,0x00,0x00,0x00};

uint32_t dev_id = 0;
volatile BaseType_t g_KCM_SPI_Done = pdFAIL;

unsigned char* ptr_receive = NULL;
uint16_t tmp_buffer[1000] ={0};
int tmp_len = 0;
static void KCM_ReadWriteFunc(unsigned char* send, unsigned char* recv, unsigned int len);
//extern dev_status;
//get_dev_id test only
uint16_t send[5+NOP_NUM] = {DEVID_REGID|REG_R, NOP, NOP};

void uwb_cmd_init(void);
void uwb_spi_init(void);
void gpio_init(void);
#ifdef UWB_TASK //setting UWB devices
void uwb_task(void);
#endif


#endif  //GD32F450I_EVAL_H
