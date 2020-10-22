/*!
    \file  gd32fr_uwb.c
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

#include "gd32fr_uwb.h"
#include "stdbool.h"
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

    #if 0
    /* dev id is correct */
    if(KCM2000_DEV_ID == dev_id){

    }else{
        /* spi read id fail */
        //printf("\n\rdev_id:%s\n\r", dev_id);
        printf("\n\rdev_id:%x\n\r", dev_id);
        printf("\n\rSPI device id: Read ID Fail!\n\r");
    }
    printf("\n\rTempreture:%d\n\r",kcm_get_temperature());
    printf("\n\rVersion:%08d\n\r",kcm_get_ver());
    #endif //end of #if 0
    kcm_get_temperature();
    kcm_get_ver();

    kcm_set_debug_cmd_on(1); 
    kcm_initialize(&kcm_config); 
    kcm_set_rx_timeout(0x1BEBC200); 
    kcm_set_rx_continuously(TRUE); 
    //kcm_set_ant_mode(KCM_ANT_AOA);
    kcm_set_ant_mode(KCM_ANT_TDOA2);  
    kcm_set_rx_dsa(3); 
    kcm_set_da0(155);
    kcm_set_da1(155); 
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

void gpio_init(void){
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

    #ifdef JLINK
        gpio_mode_set(GPIOA,GPIO_MODE_INPUT,GPIO_PUPD_PULLDOWN,GPIO_PIN_9); 
        gpio_mode_set(GPIOA,GPIO_MODE_INPUT,GPIO_PUPD_PULLDOWN,GPIO_PIN_10);
    #else 
        gpio_mode_set(GPIOA,GPIO_MODE_INPUT,GPIO_PUPD_PULLDOWN,GPIO_PIN_13); 
        gpio_mode_set(GPIOA,GPIO_MODE_INPUT,GPIO_PUPD_PULLDOWN,GPIO_PIN_14); 
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

void rcu_init(void){
    rcu_periph_clock_enable(RCU_GPIOA);
	rcu_periph_clock_enable(RCU_GPIOB);
	rcu_periph_clock_enable(RCU_GPIOC);
    rcu_periph_clock_enable(RCU_SPI1);
    rcu_periph_clock_enable(RCU_SPI0);
    #ifdef ENABLE_DMA
    rcu_periph_clock_enable(RCU_DMA0);
    #endif 
}
void uwb_reset(void){
     SET_UWB_RESET
     vTaskDelay(5 / portTICK_RATE_MS);
     //delay_1ms(5);
     //SET_UWB_SET;
     //delay_1ms(5);
     vTaskDelay(5 / portTICK_RATE_MS);
    for(uint32_t i = 0; i < 100; ++i)
    {
        if(!KCM_BUSY())
        return;
        vTaskDelay(100 / portTICK_RATE_MS);
        //delay_1ms(100);
    }
    
    //printf("[KCM Init] BUSY_PIN Timeout.");  
}
/*!
    \brief      configure the DMA peripheral
    \param[in]  none
    \param[out] none
    \retval     none
*/
void dma_send_receive(unsigned char *send, unsigned char *recv, unsigned int len)
{
 dma_single_data_parameter_struct dma_init_struct;

    /* SPI0 transmit dma config */
    dma_deinit(DMA0,DMA_CH4);
    dma_init_struct.periph_addr         = (uint32_t)&SPI_DATA(SPI0);
    dma_init_struct.memory0_addr        = (uint32_t)send;
    dma_init_struct.direction           = DMA_MEMORY_TO_PERIPH;
    dma_init_struct.periph_memory_width = DMA_PERIPH_WIDTH_8BIT;
    dma_init_struct.priority            = DMA_PRIORITY_HIGH;
    dma_init_struct.number              = len;
    dma_init_struct.periph_inc          = DMA_PERIPH_INCREASE_DISABLE;
    dma_init_struct.memory_inc          = DMA_MEMORY_INCREASE_ENABLE;
    dma_init_struct.circular_mode       = DMA_CIRCULAR_MODE_DISABLE;
    dma_single_data_mode_init(DMA0,DMA_CH4,dma_init_struct);
    dma_channel_subperipheral_select(DMA0,DMA_CH4,DMA_SUBPERI0);

    /* SPI0 receive dma config */
    dma_deinit(DMA0,DMA_CH3);
    dma_init_struct.periph_addr  = (uint32_t)&SPI_DATA(SPI0);
    dma_init_struct.memory0_addr = (uint32_t)recv;
    dma_init_struct.direction    = DMA_PERIPH_TO_MEMORY;
    dma_init_struct.priority     = DMA_PRIORITY_HIGH;
    dma_single_data_mode_init(DMA0,DMA_CH3,dma_init_struct);
    dma_channel_subperipheral_select(DMA0,DMA_CH3,DMA_SUBPERI0);
		
    /* DMA channel enable */
    //SET_SPI1_NSS_HIGH
    dma_channel_enable(DMA0,DMA_CH4);
    dma_channel_enable(DMA0,DMA_CH3);

    //settings as GD LED example
    SET_SPI1_NSS_LOW
    vTaskDelay(500 / portTICK_RATE_MS);
    //delay_1ms(500);

    /* SPI DMA enable */
    spi_dma_enable(SPI0, SPI_DMA_TRANSMIT);
    spi_dma_enable(SPI0, SPI_DMA_RECEIVE);

    /* wait dma transmit complete */
    //while(!dma_flag_get(DMA0,DMA_CH4,DMA_FLAG_FTF));
    //while(!dma_flag_get(DMA0,DMA_CH3,DMA_FLAG_FTF));
    while(!dma_flag_get(DMA0,DMA_CH4,DMA_CHFCTL_RESET_VALUE));
    while(!dma_flag_get(DMA0,DMA_CH3,DMA_CHFCTL_RESET_VALUE));
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

void SPI2_IRQHandler(void)
{
	//usart_data_transmit(USART0, '2');
    //printf("SPI2 has IRQ!");
    //get dev id from UWB
    //this instruction has to be called in 
    //interrupt since we using spi interupt 
    //spi_i2s_data_transmit(SPI2, 0x80|0x03);
}

static void KCM_ReadWriteFunc(unsigned char* send, unsigned char* recv, unsigned int len)
{
  static uint32_t wait_cnt = 0;
  ptr_receive = recv;
  tmp_len = 0;

  /*
  wait_cnt = 2000;

  while(KCM_BUSY()){
    if(--wait_cnt==0)
    {
      printf("\n\r[KCM WR] BUSY_PIN Timeout.\n\r");
      return;
    }
  }
  */
  #ifdef ENABLE_DMA//send & receive by DMA

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
            printf("\n\r[KCM WR] BUSY_PIN Timeout.\n\r");
            return;
            }
        }

        SET_SPI1_NSS_LOW;
        //for(send_n=0; send_n<len; send_n++){
        while(send_n < len) {
            while(RESET == spi_i2s_flag_get(SPI0, SPI_FLAG_TBE));
            spi_i2s_data_transmit(SPI0, send[send_n++]);
        } 
        #ifndef SPIIRQ
            uint8_t ut8 ;
            unsigned  int uint32 = 0;

        for(__IO uint16_t i=0; i<7;i++){
            //while(RESET == spi_i2s_flag_get(SPI0, SPI_FLAG_TBE));
            switch(i){
                case 0:
                    uint32 |=  spi_i2s_data_receive(SPI0) << 28; 
                    break;
                case 1:
                    uint32 |=  spi_i2s_data_receive(SPI0) << 24; 
                    break;
                case 2:
                    uint32 |=  spi_i2s_data_receive(SPI0) << 20; 
                    break;
                case 3:
                    uint32 |=  spi_i2s_data_receive(SPI0) << 16; 
                    break;
                case 4:
                    uint32 |=  spi_i2s_data_receive(SPI0) << 12; 
                break;
                case 5:
                    uint32 |=  spi_i2s_data_receive(SPI0) << 8; 
                break;
                case 6:
                    uint32 |=  spi_i2s_data_receive(SPI0) << 4; 
                break;
                case 7:
                break;
                default:
                break;
            }
			int t = 1;
        }
        #endif  //SPIIRQ
        while(RESET != spi_i2s_flag_get(SPI0, SPI_FLAG_TRANS));
        while(RESET == spi_i2s_flag_get(SPI0, SPI_FLAG_RBNE)); 
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
    void uwb_task(void){
        #if 0
        systick_config(); 
        /* configure EVAL_COM1 */
        gd_eval_com_init(USART0);
        #endif
        
        #ifdef SPIIRQ
        irq_init();
        #endif
        rcu_init(); 
        gpio_init(); 
        SET_SPI1_SWITCH_HIGH; 
        vTaskDelay(2000 / portTICK_RATE_MS);
        uwb_spi_init();
        //NSS disable here 
        //it will be enable before dma settings
        //SET_SPI1_NSS_LOW
        uwb_reset();

        kcm_set_spi_wr_func(KCM_ReadWriteFunc); 
        uwb_cmd_init(); 
    }
#endif

/* retarget the C library printf function to the USART */
int fputc(int ch, FILE *f)
{
    usart_data_transmit(EVAL_COM1, (uint8_t) ch);
    while (RESET == usart_flag_get(EVAL_COM1, USART_FLAG_TBE));
    return ch;
}
