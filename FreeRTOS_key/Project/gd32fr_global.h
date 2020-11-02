/*!
    \file  gd32fr_global.h
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

#ifndef GD32FR_GLOBAL_H
#define GD32FR_GLOBAL_H
#include "FreeRTOS.h"
#include "task.h"

typedef enum{
    IDLE = 0,   /*idle this is nothing to do*/
    UWB  = 1,   /*UWB module is working*/ 
    UM   = 2,   /*Unicorecomm is working*/
    EC   = 3,   /*EC20 module is working*/
    WIFI = 4    /*Wifi module is working*/
}dev_status;

dev_status static run_status = IDLE;

#define JLINK
//#define UWB_TASK
#define WIFI_TASK
//#define UM4B0_TASK
//#define SPIIRQ
#define ENABLE_DMA

#define INIT_TASK_PRIO   ( tskIDLE_PRIORITY + 1 )
#define LED_TASK_PRIO    ( tskIDLE_PRIORITY + 2 )
#ifdef UWB_TASK
#define UWB_TASK_PRIO    ( tskIDLE_PRIORITY + 2 )
#endif
#ifdef UM4B0_TASK
#define UM4B0_TASK_PRIO    ( tskIDLE_PRIORITY + 2 )
#endif 
#ifdef WIFI_TASK
#define WIFI_TASK_PRIO    ( tskIDLE_PRIORITY + 2 )
#endif
 
#endif // GD32FR_GLOBAL_H
