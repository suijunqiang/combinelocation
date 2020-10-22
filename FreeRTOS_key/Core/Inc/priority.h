
#ifndef __PRIORITY_H
#define __PRIORITY_H

#include "cmsis_os.h"

#define IRQ_RF_RxINT_Priority     0

#define IRQ_KCM_SPI_Priority      1
#define IRQ_ETH_SPI_Priority      1
#define IRQ_TIM3_Priority         1
#define IRQ_TIM6_Priority         6
#define IRQ_TIM7_Priority         6

#define IRQ_KCM_INT_Priority      7

#define IRQ_ETH_RxINT_Priority    8

#define IRQ_RF_UART4_Priority     10
#define IRQ_DBG_UART2_Priority    10

#define IRQ_ADC1_Priority         12

#define IRQ_TIM1_Priority         15

/*************************************************************************/


#define IWDG_TASK_Priority        2
#define UART2_TX_TASK_Priority    3
#define UART2_RX_TASK_Priority    4
#define UART4_RX_TASK_Priority    5
#define NET_RX_TASK_Priority      6
#define MAIN_TASK_Priority        7





#endif 


