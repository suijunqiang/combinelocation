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

#ifndef GD32FR_UM4B0_H
#define GD32FR_UW4B0_H

#include "gd32f450z_eval.h"
#include "gd32fr_global.h"
#include "stdio.h"

#ifdef WIFI_TASK //setting wifi devices
#define SET_WIFI_POWER_ON  gpio_bit_set(GPIOA, GPIO_PIN_11);
#define SET_WIFI_POWER_OFF gpio_bit_set(GPIOA, GPIO_PIN_11);
void wifi_task(void);
void wifi_gpio_init(void);
void wifi_rcu_init(void);
void eprintf(uint32_t *str);
void init(void);
void tx_task(void);
void rx_task(void);
void wifi_com_inituint32_t com);
#endif


#endif  //GD32F450I_EVAL_H
