/*!
    \file  main.c
    \brief UWB  
    
    \version 2020-10-14, V1.0.0, firmware for GD32F4xx platform
*/

/*
    Copyright (c) 2020, VKing Inc.

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

#include "gd32f4xx.h"
#include "gd32f450z_eval.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "gd32fr_uwb.h"
#include "gd32fr_global.h"
#include <stdio.h>

#define INIT_TASK_PRIO   ( tskIDLE_PRIORITY + 1 )
#define LED_TASK_PRIO    ( tskIDLE_PRIORITY + 2 )
#define UWB_TASK_PRIO    ( tskIDLE_PRIORITY + 2 )
 
/* Binary semaphore handle definition. */
SemaphoreHandle_t binary_semaphore;
//internal function declare
/*!
    \brief      main function
    \param[in]  none
    \param[out] none
    \retval     none
*/
int main(void)
{
    //verify the USART0
	//gd_eval_com_init(USART0);
    //usart_data_transmit(USART0, '\r');
    //printf("main function.\r\n");
    #ifdef UWB_TASK
    /* UWB task */
    xTaskCreate(uwb_task, "UWB_TASK", configMINIMAL_STACK_SIZE, NULL, UWB_TASK_PRIO, NULL);
    #endif
    #ifdef UM4B0_TASK
    /* UWB task */
    xTaskCreate(um4b0_task, "UM4B0_TASK", configMINIMAL_STACK_SIZE, NULL, UWB_TASK_PRIO, NULL);
    #endif
    
    /* start scheduler */
    vTaskStartScheduler();

    //Status machine for monitor all devices
    //IDLE: nothing to do
	while(1){
        switch(run_status){
            case IDLE:
                printf("IDLE!");
                //usart_data_transmit(USART0, '\r');
                //usart_data_transmit(USART0, '\n');
                vTaskDelay(500 / portTICK_RATE_MS);
            break;
            case UWB:
                printf("UWB!");
                vTaskDelay(500 / portTICK_RATE_MS);
            break;
           case EC:
                printf("EC!");
                vTaskDelay(500 / portTICK_RATE_MS);
            break;
           case UM:
                printf("UM!");
                vTaskDelay(500 / portTICK_RATE_MS);
            break;
            case WIFI:
                printf("WIFI!");
                vTaskDelay(500 / portTICK_RATE_MS);
            break;
           default:
            break;
        }
	}

}
