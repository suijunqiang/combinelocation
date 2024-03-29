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

#include "gd32f4xx.h"
#include "gd32f450i_eval.h"
#include "gd32fr_global.h"
#include "kcm_driver.h"
#include "stdio.h"
#include "stdbool.h"
#include "gd32fr_wifi.h"

//UWB NSS high is disable
#define SET_SPI1_NSS_HIGH          gpio_bit_set(GPIOA,GPIO_PIN_4);
#define SET_SPI0_NSS_HIGH          gpio_bit_set(GPIOA,GPIO_PIN_4);
//UWB NSS low is enable
#define SET_SPI1_NSS_LOW           gpio_bit_reset(GPIOA,GPIO_PIN_4);
#define SET_SPI0_NSS_LOW           gpio_bit_reset(GPIOA,GPIO_PIN_4);
//UWB power on
#define SET_UWB_POWER_ON       gpio_bit_set(GPIOB,GPIO_PIN_15);
//UWB power off
#define SET_UWB_POWER_OFF        gpio_bit_reset(GPIOB,GPIO_PIN_15);
//Set UWB 
#define SET_UWB_SET                gpio_bit_set(GPIOC,GPIO_PIN_5);
//Reset UWB 
#define SET_UWB_RESET              gpio_bit_reset(GPIOC,GPIO_PIN_5);

#ifdef JLINK //BUSY is PB 13
    #define KCM_BUSY()                 gpio_input_bit_get(GPIOB, GPIO_PIN_13)     //Busy PB 13
#else
    //BUSY pin is 14 not 13
    #define KCM_BUSY()                 gpio_input_bit_get(GPIOA, GPIO_PIN_13)
#endif
#define KCM_INT_PIN_GET                   gpio_input_bit_get(GPIOB, GPIO_PIN_12) //Int PB 12
#define KCM_INT_PIN_RESET                 gpio_bit_reset(GPIOA, GPIO_PIN_10)

#define NOP 0x00 
#define NOP_NUM 2 
#define DOFFSET (1+NOP_NUM) 
#define REG_W 0x00
#define REG_R 0x80
#define DEVID_REGID 0x03
#define KCM2000_DEV_ID 0x2d9c0501
#define KCM2001_DEV_ID 0x2d9c0502
#define ARRAYSIZE         7
#define ID0BITS      0xFF
#define ID1BITS      0xFF << 8
#define ID2BITS      0xFF << 16
#define ID3BITS      0xFF << 24

__IO uint32_t send_n = 0, receive_n = 0;
uint8_t spi0_send_array[ARRAYSIZE] = {0x83,0x00,0x00,0x00,0x00,0x00,0x00};
uint8_t  ut[10] = {0}; 

uint32_t dev_id = 0;
static QueueHandle_t s_MTSKMsgQueue, s_UdpMsgQueue;
rtc_timestamp_struct rtc_timestamp; 
unsigned char* ptr_receive = NULL;
uint16_t tmp_buffer[1000] ={0};
int tmp_len = 0;
static void KCM_ReadWriteFunc(unsigned char* send, unsigned char* recv, unsigned int len);
uint8_t UWB_LOG[] = "UWB_LOG:";
bool static wifi_ready = false;
bool static wifi_flag  = false;

typedef enum
{
  MT_ACSyncInit,
  MT_ACSyncDeInit,
  MT_NetworkMsg,
  MT_nRFMsg,
  MT_GetRFWarnMsg,
  MT_PreWriteSbeacon,
  MT_PreWriteLbeacon,
  MT_ReportStatus,
  MT_KCMTxSyncMsg,
  MT_KCMInterrupt,
  MT_WIFIREADY
}MainTaskMsg;

//extern dev_status;
//get_dev_id test only
uint16_t send[5+NOP_NUM] = {DEVID_REGID|REG_R, NOP, NOP};

void uwb_cmd_init(void);
void uwb_spi_init(void);
void uwb_gpio_init(void);
void GPIO_EXTI_Callback(uint16_t);
#ifdef UWB_TASK //setting UWB devices
void uwb_task(void);
#endif
static void uwb_cmd_init(void){

    KCM_CONFIG kcm_config = 
    {
        .tx_preamble_len = KCM_PLEN_256,
        .rx_preamble_len = KCM_PLEN_128,
        .sfd_sequence = 2,
        .prf = KCM_PRF_1M,
        .pdu_len = 8,
    };

    /* get device id */
    dev_id = kcm_get_dev_id();

    /* dev id is correct */
    if(KCM2000_DEV_ID == dev_id){
        //printf("\n\rdev_id:%s\n\r", dev_id);
        //printf("\n\rUWB_LOG: dev_id:%x\n\r", dev_id);
        //printf("\n\rUWB_LOG: SPI device id: Read ID Fail!\n\r");

    }else{
        /* spi read id fail */
    }
    kcm_get_temperature();
    kcm_get_ver();

    kcm_set_debug_cmd_on(1); 
    kcm_initialize(&kcm_config); 
    kcm_set_rx_timeout(0x1BEBC200); 
    kcm_set_rx_continuously(pdTRUE); 
    //kcm_set_ant_mode(KCM_ANT_AOA);
    kcm_set_ant_mode(KCM_ANT_TDOA2);  
    kcm_set_rx_dsa(3); 
    kcm_set_da0(190);
    kcm_set_da1(190); 
}

void uwb_spi_init(void){
    spi_parameter_struct spi_init_struct; 
    /* SPI0 parameter config */
    spi_init_struct.trans_mode           = SPI_TRANSMODE_FULLDUPLEX;
    spi_init_struct.device_mode          = SPI_MASTER;
    spi_init_struct.frame_size           = SPI_FRAMESIZE_8BIT;
    spi_init_struct.clock_polarity_phase = SPI_CK_PL_LOW_PH_1EDGE;
    spi_init_struct.nss                  = SPI_NSS_SOFT;
    spi_init_struct.prescale             = SPI_PSC_16;
    spi_init_struct.endian               = SPI_ENDIAN_MSB;
    spi_init(SPI0, &spi_init_struct); 
    /* set crc polynomial */
    //spi_crc_polynomial_set(SPI0,7);
    #ifdef SPIIRQ
        //spi_i2s_interrupt_enable(SPI0, SPI_I2S_INT_TBE);
        spi_i2s_interrupt_enable(SPI0, SPI_I2S_INT_RBNE);
    #endif

    spi_enable(SPI0); 
}

#ifdef NO_INDEPENDENT_WIFI_TASK
void wifi_at_init(void){ 

    SET_WIFI_POWER_OFF //as GD said 
    vTaskDelay(5000 / portTICK_RATE_MS);
    SET_WIFI_RESET
    vTaskDelay(5000 / portTICK_RATE_MS);
    gd_eval_com_init(USART2); 
}
#endif //NO_INDEPENDENT_WIFI_TASK
void uwb_gpio_init(void){
    /* SPI5_CLK(PG13), SPI5_MISO(PG12), SPI5_MOSI(PG14),SPI5_IO2(PG10) and SPI5_IO3(PG11) GPIO pin configuration */
    gpio_af_set(GPIOA, GPIO_AF_5, GPIO_PIN_5 | GPIO_PIN_6 |GPIO_PIN_7);
    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_5 | GPIO_PIN_6 |GPIO_PIN_7);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ, GPIO_PIN_5 | GPIO_PIN_6 |GPIO_PIN_7);

    /* set SPI1_NSS as GPIO*/
    gpio_mode_set(GPIOA,GPIO_MODE_OUTPUT,GPIO_PUPD_NONE,GPIO_PIN_4);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ, GPIO_PIN_4);

	//UWB Power on
    gpio_mode_set(GPIOB,GPIO_MODE_OUTPUT,GPIO_PUPD_NONE,GPIO_PIN_15);
    gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ, GPIO_PIN_15);
	
	//UWB Reset
    gpio_mode_set(GPIOC,GPIO_MODE_OUTPUT,GPIO_PUPD_NONE,GPIO_PIN_5);
    gpio_output_options_set(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ, GPIO_PIN_5); 

    //USART0_RX
    gpio_af_set(GPIOA, GPIO_AF_10, GPIO_PIN_10);
    gpio_mode_set(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, GPIO_PIN_10); 

    //UWB Int
    gpio_af_set(GPIOB, GPIO_AF_12, GPIO_PIN_12);
    gpio_mode_set(GPIOB, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, GPIO_PIN_12);

    //gpio_af_set(GPIOB, GPIO_AF_13, GPIO_PIN_13);
    //gpio_mode_set(GPIOB, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, GPIO_PIN_13);
    //gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ, GPIO_PIN_10); 
    //GPIO_BC()
    //gpio_af_set(GPIOA, GPIO_AF_10, GPIO_PIN_10);
    //gpio_mode_set(GPIOA,GPIO_MODE_INPUT,GPIO_PUPD_PULLUP,GPIO_PIN_10); 


    #ifdef JLINK
        gpio_mode_set(GPIOA,GPIO_MODE_INPUT,GPIO_PUPD_PULLDOWN,GPIO_PIN_9); 
        gpio_mode_set(GPIOB,GPIO_MODE_INPUT,GPIO_PUPD_PULLDOWN,GPIO_PIN_13); 
        //gpio_mode_set(GPIOA,GPIO_MODE_INPUT,GPIO_PUPD_PULLDOWN,GPIO_PIN_10);
    #else 
        gpio_mode_set(GPIOA,GPIO_MODE_INPUT,GPIO_PUPD_PULLDOWN,GPIO_PIN_13); 
    #endif

    //settings as GD LED example
    SET_SPI1_NSS_HIGH
} 
#ifdef SPIIRQ
void irq_init(void){ 
    /* NVIC config */
    nvic_priority_group_set(NVIC_PRIGROUP_PRE1_SUB3);
    nvic_irq_enable(SPI0_IRQn,1,1); 
}
#endif //SPIIRQ
void uwb_exti_irq_init(void){
    /* NVIC config */
    nvic_irq_enable(EXTI10_15_IRQn, 2U, 0U); 
    syscfg_exti_line_config(EXTI_SOURCE_GPIOB, EXTI_SOURCE_PIN12);
    /* configure key EXTI line */
    exti_init(EXTI_12, EXTI_INTERRUPT, EXTI_TRIG_RISING);
    NVIC_SetPriorityGrouping(0);
    exti_interrupt_flag_clear(EXTI_12);
}

void uwb_rcu_init(void){
    rcu_periph_clock_enable(RCU_GPIOA);
	rcu_periph_clock_enable(RCU_GPIOB);
	rcu_periph_clock_enable(RCU_GPIOC);
    rcu_periph_clock_enable(RCU_SPI1);
    rcu_periph_clock_enable(RCU_SPI0);
    rcu_periph_clock_enable(RCU_SYSCFG);
    #ifdef ENABLE_DMA
    rcu_periph_clock_enable(RCU_DMA1);
    #endif 
}
void uwb_reset(void){
     SET_UWB_RESET
    vTaskDelay(100 / portTICK_RATE_MS);
     //vTaskDelay(5 / portTICK_RATE_MS);
     //SET_UWB_SET;
     //vTaskDelay(5 / portTICK_RATE_MS);
    for(uint32_t i = 0; i < 100; ++i){
        if(!KCM_BUSY())
        return;
        vTaskDelay(100 / portTICK_RATE_MS);
    }
    
    //printf("UWB_LOG: [KCM Init] BUSY_PIN Timeout.");  
}

void dma_send_receive(unsigned char *send, unsigned char *recv, unsigned int len){
    dma_single_data_parameter_struct dma_init_struct;

    /* SPI0 transmit dma config */
    dma_deinit(DMA1,DMA_CH3);
    dma_init_struct.periph_addr         = (uint32_t)&SPI_DATA(SPI0);
    dma_init_struct.memory0_addr        = (uint32_t)send;
    dma_init_struct.direction           = DMA_MEMORY_TO_PERIPH;
    dma_init_struct.periph_memory_width = DMA_PERIPH_WIDTH_8BIT;
    dma_init_struct.priority            = DMA_PRIORITY_HIGH;
    dma_init_struct.number              = len;
    dma_init_struct.periph_inc          = DMA_PERIPH_INCREASE_DISABLE;
    dma_init_struct.memory_inc          = DMA_MEMORY_INCREASE_ENABLE;
    dma_init_struct.circular_mode       = DMA_CIRCULAR_MODE_DISABLE;
    dma_single_data_mode_init(DMA1,DMA_CH3,&dma_init_struct);
    dma_channel_subperipheral_select(DMA1,DMA_CH3,DMA_SUBPERI3);

    /* SPI0 receive dma config */
    dma_deinit(DMA1,DMA_CH2);
    dma_init_struct.periph_addr  = (uint32_t)&SPI_DATA(SPI0);
    dma_init_struct.memory0_addr = (uint32_t)recv;
    dma_init_struct.direction    = DMA_PERIPH_TO_MEMORY;
    dma_init_struct.priority     = DMA_PRIORITY_HIGH;
    dma_single_data_mode_init(DMA1,DMA_CH2,&dma_init_struct);
    dma_channel_subperipheral_select(DMA1,DMA_CH2,DMA_SUBPERI3);
  
    dma_channel_enable(DMA1,DMA_CH3);
    dma_channel_enable(DMA1,DMA_CH2);

    //settings as GD LED example
    SET_SPI1_NSS_LOW
    vTaskDelay(500 / portTICK_RATE_MS);

    /* SPI DMA enable */ 
    spi_dma_enable(SPI0, SPI_DMA_TRANSMIT);
    spi_dma_enable(SPI0, SPI_DMA_RECEIVE);

    /* wait dma transmit complete */
    while(!dma_flag_get(DMA1,DMA_CH3,DMA_FLAG_FTF));
    while(!dma_flag_get(DMA1,DMA_CH2,DMA_FLAG_FTF));
    //settings as GD LED example
    SET_SPI1_NSS_HIGH
}
/*!
    \brief      toggle the led every 500ms
    \param[in]  none
    \param[out] none
    \retval     none
*/
void led_spark(void)
{
    static __IO uint32_t timingdelaylocal = 0U;

    if(timingdelaylocal){

        if(timingdelaylocal < 500U){
            gd_eval_led_on(LED1);
        }else{
            gd_eval_led_off(LED1);
        }

        timingdelaylocal--;
    }else{
        timingdelaylocal = 1000U;
    }
}

#if 0
/* retarget the C library printf function to the USART */
int fputc(int ch, FILE *f)
{
    usart_data_transmit(EVAL_COM0, (uint8_t)ch);
    while(RESET == usart_flag_get(EVAL_COM0, USART_FLAG_TBE));
    return ch;
}    
#endif

static void KCM_ReadWriteFunc(unsigned char* send, unsigned char* recv, unsigned int len){
  static uint32_t wait_cnt = 0;
  ptr_receive = recv;
  tmp_len = 0;

  /*
  wait_cnt = 2000;

  while(KCM_BUSY()){
    if(--wait_cnt==0)
    {
      printf("\n\rUWB_LOG: [KCM WR] BUSY_PIN Timeout.\n\r");
      return;
    }
  }
  */
  #ifdef ENABLE_DMA//send & receive by DMA
        wait_cnt = 2000;
        while(KCM_BUSY()){
            if(--wait_cnt==0)
            {
            //printf("\n\r UWB_LOG: [KCM WR] BUSY_PIN Timeout.\n\r");
            return;
            }
        }

        SET_SPI1_NSS_LOW;


    //SET_SPI1_NSS_HIGH
    dma_send_receive(send, recv, len);  
    //SET_SPI1_NSS_LOW
    //HAL_SPI_TransmitReceive_DMA(&spi_init_struct, send, recv, len);
  #else 

    //while(1)
    {
        wait_cnt = 2000;
        while(KCM_BUSY()){
            if(--wait_cnt==0)
            {
            printf("\n\rUWB_LOG: [KCM WR] BUSY_PIN Timeout.\n\r");
            return;
            }
        }

        SET_SPI1_NSS_LOW;

        #ifndef SPIIRQ
        uint8_t ut8 ;
        uint8_t  uint[100] = {0}; 
        __IO uint16_t i = 0;
        __IO uint16_t j = 0;
        #endif
        //for(send_n=0; send_n<len; send_n++){
        while(send_n < len) {
            while(RESET == spi_i2s_flag_get(SPI0, SPI_FLAG_TBE));
            spi_i2s_data_transmit(SPI0, send[send_n++]);

            #ifndef SPIIRQ
            while(RESET == spi_i2s_flag_get(SPI0, SPI_FLAG_RBNE));
            uint[j++] =  spi_i2s_data_receive(SPI0); 
            #endif

        } 
        while(RESET != spi_i2s_flag_get(SPI0, SPI_FLAG_TRANS));
        //while(RESET == spi_i2s_flag_get(SPI0, SPI_FLAG_RBNE)); 
        SET_SPI1_NSS_HIGH;
        send_n = 0;
    }
    
  #endif  // end of ENABLE_DMA
} 

#ifdef SPIIRQ
/*!
    \brief      this function handles SPI0 Handler 
    \param[in]  none
    \param[out] none
    \retval     none
*/
void SPI0_IRQHandler(void)
{
    /* received data */
    if(RESET != spi_i2s_interrupt_flag_get(SPI0,SPI_I2S_INT_RBNE)){
        //uint16_t ut16 = spi_i2s_data_receive(SPI0); 
        __IO uint16_t ut16 = spi_i2s_data_receive(SPI0); 
         tmp_buffer[tmp_len++] = ut16;
         0;
        // if(ptr_receive != NULL){
        //     ptr_receive++ = spi_i2s_data_receive(SPI0);
        // } 
    } 
     /* received data */
    if(RESET == spi_i2s_interrupt_flag_get(SPI0,SPI_I2S_INT_RBNE)){
        //uint16_t ut16 = spi_i2s_data_receive(SPI0); 
        __IO uint16_t ut16 = spi_i2s_data_receive(SPI0); 
        tmp_buffer[tmp_len++] = ut16;
        spi_i2s_interrupt_disable(SPI1, SPI_I2S_INT_TBE);
        spi_i2s_interrupt_disable(SPI1, SPI_I2S_INT_RBNE);
        0;
        // if(ptr_receive != NULL){
        //     ptr_receive++ = spi_i2s_data_receive(SPI0);
        // } 
    } 
    if(RESET != spi_i2s_interrupt_flag_get(SPI0,SPI_I2S_INT_TBE)){
        1;
    }
    if(RESET != spi_i2s_interrupt_flag_get(SPI0,SPI_I2S_INT_ERR)){
        2;
    }
}

#endif //end of SPIIRQ
#ifdef UWB_TASK
    static void HandleGoodFrame(void) {
    MainTaskMsg udp_event = MT_WIFIREADY;
    s_UdpMsgQueue = xQueueCreate(10, sizeof(MainTaskMsg));
    static uint8_t CQI[3],CQI2[3];
    static uint8_t tp[6];
    static int16_t diff_tp;
    static uint8_t pdu[8]; 
    bool aoa_enable = false;
    /*
			MainWifiTaskMsg pdu_event      = MT_ATUWBDATA;
      s_ATMsgQueue                   = xQueueCreate(10, sizeof(MainWifiTaskMsg));
		*/
    
    if(aoa_enable) {
        kcm_get_aoa_all(tp,&diff_tp,CQI,CQI2,pdu);
        //kcm_get_rx_pdu(pdu.Data);
        //kcm_get_rx_time_stamp(tp);
        //kcm_get_rx_difftime_quality(&diff_tp, CQI);
        diff_tp = diff_tp / 50;

        if(diff_tp > 127)
        diff_tp = 127;
        else if(diff_tp < -127)
        diff_tp = -127;
    }else{
        kcm_get_rx_pdu_time_quality(tp, CQI, pdu);
        //kcm_get_rx_pdu(pdu.Data);
        //kcm_get_rx_time_stamp(tp);
        //kcm_get_rx_quality(CQI);
    }
    /*
    if(!wifi_flag){
			
        vTaskSuspend(uwbHandle);
        wifi_flag = true;
    }
    if(wifi_flag){
        strcpy(g_udp, pdu);
        strcpy(g_udp, "  ");
        strcpy(g_udp, tp);

        printf("AT+CIPSEND=%d\r\n",strlen(g_udp));
        printf("%s", g_udp); 
    }
    */
    #ifdef WIFI_READY
    if(!wifi_ready){
        xQueueReceive(s_UdpMsgQueue, &udp_event, portMAX_DELAY);
        wifi_ready = true; 
    }
    if(wifi_ready){
        strcpy(g_udp, pdu);
        strcpy(g_udp, "  ");
        strcpy(g_udp, tp);

        printf("AT+CIPSEND=%d\r\n",strlen(g_udp));
        printf("%s", g_udp);
    }else{
        //nothing to do
        //printf("ZZZ");

    }
 
   free(s_UdpMsgQueue);
   #else //WIFI_READY
   /*
        if(g_udp_availble){
            strcpy(g_udp, pdu);
            strcpy(g_udp, "  ");
            strcpy(g_udp, tp);

            printf("AT+CIPSEND=%d\r\n",strlen(g_udp));
            printf("%s", g_udp); 
        }else{
            //do nothing here
        }
        */
    if(!g_uwb_wiat_wifi){
        //delay 1min to wait wifi ready, it should be changed in other way.
        //the time as long as wifi initial finish
        vTaskDelay(15000 / portTICK_RATE_MS);
        g_uwb_wiat_wifi = true;
    }else{
        vTaskDelay(2000 / portTICK_RATE_MS); 
    }

    //printf("AT+CIPSEND=%d\r\n",strlen(g_udp));
    //printf("%s", g_udp); 

    uint8_t id[4];
    id[0] = dev_id&ID0BITS; 
    id[1] = (dev_id&ID1BITS)>>8;
    id[2] = (dev_id&ID2BITS)>>16;
    id[3] = (dev_id&ID3BITS)>>24;

    printf("AT+CIPSEND=%d\r\n",19);

    printf("%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%d", \
        id[0],id[1],id[2],id[3],pdu[0],pdu[1],pdu[2],pdu[3],pdu[4],pdu[5],pdu[6],pdu[7],\
        tp[0],tp[1],tp[2],tp[3],tp[4],tp[5], diff_tp); 
        /*
    printf("%02x%02x%02x%02x%02x%02x%02x%02x  %02x%02x%02x%02x%02x%02x  %d", \
        pdu[0],pdu[1],pdu[2],pdu[3],pdu[4],pdu[5],pdu[6],pdu[7],\
        tp[0],tp[1],tp[2],tp[3],tp[4],tp[5], diff_tp); 
        */
  #endif  //WIFI_READY
    /*
    xQueueSend(s_ATMsgQueue, &pdu_event, 0); 
    
    
    printf("UWB_LOG: PDU:%02x%02x%02x%02x%02x%02x%02x%02x TP:%02x%02x%02x%02x%02x%02x DFTP:%d", \
        pdu[0],pdu[1],pdu[2],pdu[3],pdu[4],pdu[5],pdu[6],pdu[7],\
        tp[0],tp[1],tp[2],tp[3],tp[4],tp[5], diff_tp);
    */
        
        
}

static void HandleErrFrame(uint16_t status) {
    //printf("UWB_LOG:Error Occurred. Reg:[%04x]", status);
}

static void HandleTxDone(void){
    uint8_t tp[6];
    
    kcm_get_tx_time_stamp(tp);
    if(g_udp_availble){ 
        strcpy(g_udp, tp);

        printf("AT+CIPSEND=%d\r\n",strlen(g_udp));
        printf("%s", g_udp);
    }else{
        //nothing to do 
        //printf("ZZZ");
    }
 
    //printf("UWB_LOG:Tx Done.  TP:%02x%02x%02x%02x%02x%02x",tp[0],tp[1],tp[2],tp[3],tp[4],tp[5]); 
}

static void HandleTxFail(void){
    //printf("UWB_LOG:KCM Tx Fail.");
}
static void HandleKCMIRQ(void){
    uint16_t status = kcm_get_int_status();

    kcm_set_int_status(KCM_INT_ALL);
    
    if(status & KCM_INT_RX_DONE)
        HandleGoodFrame();
    else if(status & KCM_INT_TX_DONE)
        HandleTxDone();
    else if(status & (KCM_INT_CRC_ERR|KCM_INT_RX_TO|KCM_INT_LDE_ERR|KCM_INT_PANIC))
        HandleErrFrame(status);
    else if(status &KCM_INT_TX_FAIL)
        HandleTxFail();
    else
			1;
        //printf("UWB_LOG:Invalid status value. %04x", status);
}

void uwb_task(void){ 
    MainTaskMsg event;
    s_MTSKMsgQueue = xQueueCreate(20, sizeof(MainTaskMsg));
    uwb_spi_init();
    uwb_exti_irq_init();
    uwb_reset(); 
    kcm_set_spi_wr_func(KCM_ReadWriteFunc); 
    uwb_cmd_init(); 
    #ifdef NO_INDEPENDENT_WIFI_TASK
        wifi_at_init();
    #endif //NO_INDEPENDENT_WIFI_TASK
    bool kcm_tx_test = false; 
    if(kcm_tx_test == false) {
            kcm_set_rx_start(IMMEDIATE);
    }
    
    while(1) {
        uint8_t tp[6];
        if(kcm_tx_test) {
            //vTaskDelay(pdMS_TO_TICKS(100)); 
            uint8_t tx_pdu[8] = {1,2,3,4,5,6,7,8}; 
            //kcm_force_off_trx();
            kcm_set_int_status(KCM_INT_ALL);
            kcm_set_tx_pdu_fully(tx_pdu);    
            kcm_set_tx_start(IMMEDIATE);
        }
        if(!kcm_tx_test){
            kcm_set_int_status(KCM_INT_ALL);
            kcm_get_rx_time_stamp(tp);
        }
            
        xQueueReceive(s_MTSKMsgQueue, &event, portMAX_DELAY);
        switch (event) {
            case MT_KCMInterrupt:
                HandleKCMIRQ();
                break;
            default:
                //printf("UWB_LOG:Unknown MTSK Msg Type.");
                break;
        }
    }
}

#ifdef NO_INDEPENDENT_WIFI_TASK
static void rx_task(void) {
    at_enum_status at_status = AP_SETTINGS_OK;
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
    }
}
#endif //NO_INDEPENDENT_WIFI_TASK

void MTSK_AppendWorkMsgISR(MainTaskMsg msg) {
  static BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  xHigherPriorityTaskWoken = pdTRUE;
  xQueueSendFromISR(s_MTSKMsgQueue, &msg, &xHigherPriorityTaskWoken); 
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void EXTI10_15_IRQHandler(void){
    exti_interrupt_flag_clear(EXTI_12);
    GPIO_EXTI_Callback(GPIO_PIN_12); 
}
void GPIO_EXTI_Callback(uint16_t GPIO_Pin){ 
    switch(GPIO_Pin){
        case GPIO_PIN_12:
            if(KCM_INT_PIN_GET) {
                MTSK_AppendWorkMsgISR(MT_KCMInterrupt);	
            }
            break;
        default:
            break;

    }

}
#endif //UWB_TASK
#if 0
/* retarget the C library printf function to the USART */
int fputc(int ch, FILE *f)
{
    usart_data_transmit(EVAL_COM1, (uint8_t) ch);
    while (RESET == usart_flag_get(EVAL_COM1, USART_FLAG_TBE));
    return ch;
}
#endif
#endif  //GD32FR_UWB_H
