
/*! ----------------------------------------------------------------------------
 * @file    kcm_driver.c
 * @attention
 *
 * Copyright 2020 (c) Kunchen Ltd.
 *
 * All rights reserved.
 *
 */

#include "kcm_driver.h"
#include "kcm_regs.h"

#include <string.h>
#include <stdbool.h>

#define NOP 0x00

#define NOP_NUM 2

#define DOFFSET (1+NOP_NUM)

#define REG_W 0x00
#define REG_R 0x80

static unsigned char s_tx_pdu_length = 8;
static unsigned char s_rx_pdu_length = 8;

#define MAX_PDU_LENGTH 40

static KCM_WR_FUNC s_spi_wr_func;

#define ACC_MAX_LEN 6400+DOFFSET

unsigned int kcm_get_dev_id(void);

static unsigned char acc_send[ACC_MAX_LEN];
static unsigned char acc_recv[ACC_MAX_LEN];


void kcm_set_debug_cmd_on(unsigned char on_off)
{
  unsigned char send[2] = {DEBUG_EN_REGID|REG_W, on_off};
  unsigned char recv[2];

  s_spi_wr_func(send, recv, sizeof(send));
}

unsigned int kcm_get_ver(void)
{
  unsigned char send[5+NOP_NUM] = {VERSION_REGID|REG_R, NOP, NOP};
  unsigned char recv[5+NOP_NUM];
  unsigned int version = 0;
  
  s_spi_wr_func(send, recv, sizeof(send));

  version |= (unsigned int)recv[DOFFSET+0] << 24;
  version |= (unsigned int)recv[DOFFSET+1] << 16;
  version |= (unsigned int)recv[DOFFSET+2] << 8;
  version |= (unsigned int)recv[DOFFSET+3] << 0;

  return version;
}

void kcm_get_detailed_ver(unsigned int *fw_ver, unsigned int *hw_ver, unsigned int logic_ver[3])
{
  unsigned char send[21+NOP_NUM] = {DETAIL_VERSION_REGID|REG_R, NOP, NOP};
  unsigned char recv[21+NOP_NUM];
  
  s_spi_wr_func(send, recv, sizeof(send));

  *fw_ver = 0;
  *fw_ver |= (unsigned int)recv[DOFFSET+0] << 24;
  *fw_ver |= (unsigned int)recv[DOFFSET+1] << 16;
  *fw_ver |= (unsigned int)recv[DOFFSET+2] << 8;
  *fw_ver |= (unsigned int)recv[DOFFSET+3] << 0;

  *hw_ver = 0;
  *hw_ver |= (unsigned int)recv[DOFFSET+4] << 24;
  *hw_ver |= (unsigned int)recv[DOFFSET+5] << 16;
  *hw_ver |= (unsigned int)recv[DOFFSET+6] << 8;
  *hw_ver |= (unsigned int)recv[DOFFSET+7] << 0;

  logic_ver[0] = 0;
  logic_ver[0] |= (unsigned int)recv[DOFFSET+8] << 24;
  logic_ver[0] |= (unsigned int)recv[DOFFSET+9] << 16;
  logic_ver[0] |= (unsigned int)recv[DOFFSET+10] << 8;
  logic_ver[0] |= (unsigned int)recv[DOFFSET+11] << 0;

  logic_ver[1] = 0;
  logic_ver[1] |= (unsigned int)recv[DOFFSET+12] << 24;
  logic_ver[1] |= (unsigned int)recv[DOFFSET+13] << 16;
  logic_ver[1] |= (unsigned int)recv[DOFFSET+14] << 8;
  logic_ver[1] |= (unsigned int)recv[DOFFSET+15] << 0;

  logic_ver[2] = 0;
  logic_ver[2] |= (unsigned int)recv[DOFFSET+16] << 24;
  logic_ver[2] |= (unsigned int)recv[DOFFSET+17] << 16;
  logic_ver[2] |= (unsigned int)recv[DOFFSET+18] << 8;
  logic_ver[2] |= (unsigned int)recv[DOFFSET+19] << 0;
}

unsigned int kcm_get_dev_id(void)
{
  unsigned char send[5+NOP_NUM] = {DEVID_REGID|REG_R, NOP, NOP};
  unsigned char recv[5+NOP_NUM] = {0};
  unsigned int dev_id = 0;
 
  s_spi_wr_func(send, recv, sizeof(send));

  dev_id |= (unsigned int)recv[DOFFSET+0] << 24;
  dev_id |= (unsigned int)recv[DOFFSET+1] << 16;
  dev_id |= (unsigned int)recv[DOFFSET+2] << 8;
  dev_id |= (unsigned int)recv[DOFFSET+3] << 0;
  
  return dev_id;
}

char kcm_get_temperature(void)
{
  unsigned char send[2+NOP_NUM] = {TEMPERATURE_REGID|REG_R, NOP};
  unsigned char recv[2+NOP_NUM];
  
  s_spi_wr_func(send, recv, sizeof(send));
  
  return recv[DOFFSET];
}

void kcm_get_vcc(unsigned char vcc[6])
{
  unsigned char send[7+NOP_NUM] = {VOLTAGE_REGID|REG_R, NOP};
  unsigned char recv[7+NOP_NUM];
  
  s_spi_wr_func(send, recv, sizeof(send));
  
  memcpy(vcc, recv+DOFFSET, sizeof(vcc));
}

void kcm_set_tx_dsa(unsigned char dsa)
{
  unsigned char send[2] = {VGA_TX_DSA_REGID|REG_W, dsa};
  unsigned char recv[2];

  s_spi_wr_func(send, recv, sizeof(send));
}

unsigned char kcm_get_tx_dsa(void)
{
  unsigned char send[2+NOP_NUM] = {VGA_TX_DSA_REGID|REG_R, NOP};
  unsigned char recv[2+NOP_NUM];
  
  s_spi_wr_func(send, recv, sizeof(send));
  
  return recv[DOFFSET];
}

void kcm_set_rx_dsa(unsigned char dsa)
{
  unsigned char send[2] = {VGA_RX_DSA_REGID|REG_W, dsa};
  unsigned char recv[2];
  
  s_spi_wr_func(send, recv, sizeof(send));
}

unsigned char kcm_get_rx_dsa(void)
{
  unsigned char send[2+NOP_NUM] = {VGA_RX_DSA_REGID|REG_R, NOP};
  unsigned char recv[2+NOP_NUM];
  
  s_spi_wr_func(send, recv, sizeof(send));
  
  return recv[DOFFSET];
}

void kcm_set_da0(unsigned char da)
{
  unsigned char send[2] = {VGA_DA0_REGID|REG_W, da};
  unsigned char recv[2];
  
  s_spi_wr_func(send, recv, sizeof(send));
}


unsigned char kcm_get_da0(void)
{
  unsigned char send[2+NOP_NUM] = {VGA_DA0_REGID|REG_R, NOP};
  unsigned char recv[2+NOP_NUM];
  
  s_spi_wr_func(send, recv, sizeof(send));
  
  return recv[DOFFSET];
}

void kcm_set_da1(unsigned char da)
{
  unsigned char send[2] = {VGA_DA1_REGID|REG_W, da};
  unsigned char recv[2];
  
  s_spi_wr_func(send, recv, sizeof(send));
}

unsigned char kcm_get_da1(void)
{
  unsigned char send[2+NOP_NUM] = {VGA_DA1_REGID|REG_R, NOP};
  unsigned char recv[2+NOP_NUM];
  
  s_spi_wr_func(send, recv, sizeof(send));
  
  return recv[DOFFSET];
}

void kcm_set_tx_preamble_len(unsigned char len)
{
  unsigned char send[2] = {TX_PREAM_LEN_REGID|REG_W, len};
  unsigned char recv[2];
  
  s_spi_wr_func(send, recv, sizeof(send));
}

unsigned char kcm_get_tx_preamble_len(void)
{
  unsigned char send[2+NOP_NUM] = {TX_PREAM_LEN_REGID|REG_R, NOP};
  unsigned char recv[2+NOP_NUM];
  
  s_spi_wr_func(send, recv, sizeof(send));
  
  return recv[DOFFSET];
}

void kcm_set_rx_preamble_len(unsigned char len)
{
  unsigned char send[2] = {RX_PREAM_LEN_REGID|REG_W, len};
  unsigned char recv[2];
  
  s_spi_wr_func(send, recv, sizeof(send));
}

unsigned char kcm_get_rx_preamble_len(void)
{
  unsigned char send[2+NOP_NUM] = {RX_PREAM_LEN_REGID|REG_R, NOP};
  unsigned char recv[2+NOP_NUM];
  
  s_spi_wr_func(send, recv, sizeof(send));
  
  return recv[DOFFSET];
}

void kcm_set_tx_sfd(unsigned char sfd)
{
  unsigned char send[2] = {TX_SFD_REGID|REG_W, sfd};
  unsigned char recv[2];
  
  s_spi_wr_func(send, recv, sizeof(send));
}

unsigned char kcm_get_tx_sfd(void)
{
  unsigned char send[2+NOP_NUM] = {TX_SFD_REGID|REG_R, NOP};
  unsigned char recv[2+NOP_NUM];
  
  s_spi_wr_func(send, recv, sizeof(send));
  
  return recv[DOFFSET];
}

void kcm_set_rx_sfd(unsigned char sfd)
{
  unsigned char send[2] = {RX_SFD_REGID|REG_W, sfd};
  unsigned char recv[2];
  
  s_spi_wr_func(send, recv, sizeof(send));
}

unsigned char kcm_get_rx_sfd(void)
{
  unsigned char send[2+NOP_NUM] = {RX_SFD_REGID|REG_R, NOP};
  unsigned char recv[2+NOP_NUM];
  
  s_spi_wr_func(send, recv, sizeof(send));
  
  return recv[DOFFSET];
}

void kcm_set_prf(unsigned short prf)
{
  unsigned char send[3] = {PRF_REGID|REG_W};
  unsigned char recv[3];

  send[1] = (prf>>8)&0xFF;
  send[2] = (prf>>0)&0xFF;
  
  s_spi_wr_func(send, recv, sizeof(send));  
}

unsigned short kcm_get_prf(void)
{
  unsigned char send[3+NOP_NUM] = {PRF_REGID|REG_R, NOP};
  unsigned char recv[3+NOP_NUM];
  unsigned short prf = 0;
  
  s_spi_wr_func(send, recv, sizeof(send)); 
  
  prf |= recv[DOFFSET+0] << 8;
  prf |= recv[DOFFSET+1];
  
  return prf;
}

void kcm_set_tx_pdu_len(unsigned char len)
{
  unsigned char send[2] = {TX_PDU_LENGTH_REGID|REG_W, len};
  unsigned char recv[2];

  s_tx_pdu_length = len>MAX_PDU_LENGTH?MAX_PDU_LENGTH:len;
  
  s_spi_wr_func(send, recv, sizeof(send)); 
}

unsigned char kcm_get_tx_pdu_len(void)
{
  unsigned char send[2+NOP_NUM] = {TX_PDU_LENGTH_REGID|REG_R, NOP};
  unsigned char recv[2+NOP_NUM];
  
  s_spi_wr_func(send, recv, sizeof(send)); 

  return recv[DOFFSET];
}

void kcm_set_rx_pdu_len(unsigned char len)
{
  unsigned char send[2] = {RX_PDU_LENGTH_REGID|REG_W, len};
  unsigned char recv[2];

  s_rx_pdu_length = len>MAX_PDU_LENGTH?MAX_PDU_LENGTH:len;
  
  s_spi_wr_func(send, recv, sizeof(send)); 
}

unsigned char kcm_get_rx_pdu_len(void)
{
  unsigned char send[2+NOP_NUM] = {RX_PDU_LENGTH_REGID|REG_R, NOP};
  unsigned char recv[2+NOP_NUM];
  
  s_spi_wr_func(send, recv, sizeof(send)); 

  return recv[DOFFSET];
}

void kcm_set_int_status(unsigned short mask)
{
  unsigned char send[3] = {INT_STATUS_REGID|REG_W};
  unsigned char recv[3];

  send[1] = (mask>>8)&0xFF;
  send[2] = (mask>>0)&0xFF;
  
  s_spi_wr_func(send, recv, sizeof(send)); 
}
unsigned short kcm_get_int_status(void)
{
  unsigned char send[3+NOP_NUM] = {INT_STATUS_REGID|REG_R, NOP};
  unsigned char recv[3+NOP_NUM];
  unsigned short mask = 0;

  s_spi_wr_func(send, recv, sizeof(send));
  
  mask |= recv[DOFFSET+0] << 8;
  mask |= recv[DOFFSET+1];

  return mask;
}

void kcm_set_ant_mode(unsigned char mode)
{
  unsigned char send[2] = {PORT_MODE_REGID|REG_W, mode};
  unsigned char recv[2];
  
  s_spi_wr_func(send, recv, sizeof(send)); 
}

unsigned char kcm_get_ant_mode(void)
{
  unsigned char send[2+NOP_NUM] = {INT_STATUS_REGID|REG_R, NOP};
  unsigned char recv[2+NOP_NUM];
  
  s_spi_wr_func(send, recv, sizeof(send)); 

  return recv[DOFFSET];
}

void kcm_set_fec_code_len(unsigned char len)
{
  unsigned char send[2] = {FEC_CODE_LEN_REGID|REG_W, len};
  unsigned char recv[2];
  
  s_spi_wr_func(send, recv, sizeof(send)); 
}

unsigned char kcm_get_fec_code_len(void)
{
  unsigned char send[2+NOP_NUM] = {FEC_CODE_LEN_REGID|REG_R, NOP};
  unsigned char recv[2+NOP_NUM];
  
  s_spi_wr_func(send, recv, sizeof(send)); 

  return recv[DOFFSET];
}

void kcm_set_tx_pdu(unsigned char *pdu, unsigned char offset)
{
  unsigned char send[64] = {TX_PDU_DATA_REGID|REG_W, offset};
  unsigned char recv[64];

  memcpy(send+2, pdu, s_tx_pdu_length);

  s_spi_wr_func(send, recv, s_tx_pdu_length + 2);
}

void kcm_set_tx_delay_time(unsigned int delay_time)
{
  unsigned char send[5] = {TX_DELAY_REGID|REG_W};
  unsigned char recv[5];

  send[1] = (delay_time >> 24) & 0xFF;
  send[2] = (delay_time >> 16) & 0xFF;
  send[3] = (delay_time >> 8)  & 0xFF;
  send[4] = (delay_time >> 0)  & 0xFF;

  s_spi_wr_func(send, recv, sizeof(send)); 
}

void kcm_set_tx_start(unsigned char delay_type)
{
  unsigned char send[2] = {TX_START_REGID|REG_W, delay_type};
  unsigned char recv[2];

  s_spi_wr_func(send, recv, sizeof(send));   
}

void kcm_get_tx_time_stamp(unsigned char *time_stamp)
{
  unsigned char send[7+NOP_NUM] = {TX_TIMESTAMP_REGID|REG_R, NOP};
  unsigned char recv[7+NOP_NUM];

  s_spi_wr_func(send, recv, sizeof(send));   

  memcpy(time_stamp, recv+DOFFSET, 6);
}

void kcm_set_tx_pdu_fully(unsigned char *pdu)
{
  unsigned char send[64] = {TX_PDU_ALL_REGID|REG_W};
  unsigned char recv[64];

  memcpy(send+1, pdu, s_tx_pdu_length);

  s_spi_wr_func(send, recv, s_tx_pdu_length + 1);
}

void kcm_set_tx_pdu_with_offset_length(unsigned char addr, unsigned char len, unsigned char *pdu)
{
  unsigned char send[128] = {TX_PDU_OFST_REGID|REG_W, addr, len};
  unsigned char recv[128];

  memcpy(send+3, pdu, len);

  s_spi_wr_func(send, recv, len + 3);
}

void kcm_set_rx_start(unsigned int delay_time)
{
  unsigned char send[5] = {RX_START_REGID|REG_W};
  unsigned char recv[5];

  send[1] = (delay_time >> 24) & 0xFF;
  send[2] = (delay_time >> 16) & 0xFF;
  send[3] = (delay_time >> 8)  & 0xFF;
  send[4] = (delay_time >> 0)  & 0xFF;  

  s_spi_wr_func(send, recv, sizeof(send));  
}

void kcm_get_rx_pdu(unsigned char *pdu)
{
  unsigned char send[64] = {RX_PDU_DATA_REGID|REG_R, NOP};
  unsigned char recv[64];

  s_spi_wr_func(send, recv, s_rx_pdu_length + DOFFSET);  

  memcpy(pdu, recv+DOFFSET, s_rx_pdu_length);
}

void kcm_get_rx_time_stamp(unsigned char *time_stamp)
{
  unsigned char send[7+NOP_NUM] = {RX_TIMESTAMP_REGID|REG_R, NOP};
  unsigned char recv[7+NOP_NUM];

  s_spi_wr_func(send, recv, sizeof(send));

  memcpy(time_stamp, recv+DOFFSET, 6);
}

void kcm_get_rx_quality(unsigned char *quality)
{
  unsigned char send[4+NOP_NUM] = {RX_FQUAL_REGID|REG_R, NOP};
  unsigned char recv[4+NOP_NUM];

  s_spi_wr_func(send, recv, sizeof(send));

  memcpy(quality, recv+DOFFSET, 3);
}

void kcm_get_rx_pdu_time_quality(unsigned char *time_stamp, unsigned char *quality, unsigned char *pdu)
{
  unsigned char send[64] = {PUD_AND_FQUAL_REGID|REG_R, NOP};
  unsigned char recv[64];

  s_spi_wr_func(send, recv, 1+NOP_NUM+6+3+s_rx_pdu_length);

  memcpy(time_stamp, recv+DOFFSET, 6);
  
  memcpy(pdu, recv+DOFFSET+6, s_rx_pdu_length);

  memcpy(quality, recv+DOFFSET+6+s_rx_pdu_length, 3);
}

unsigned char kcm_get_noise_power(void)
{
  unsigned char send[2+NOP_NUM] = {NOISE_POWER_REGID|REG_R, NOP};
  unsigned char recv[2+NOP_NUM];

  s_spi_wr_func(send, recv, sizeof(send));

  return recv[DOFFSET];
}

unsigned short kcm_get_prejudge_val(void)
{
  unsigned char send[3+NOP_NUM] = {FP_INDEX_REGID|REG_R, NOP};
  unsigned char recv[3+NOP_NUM];
  unsigned short fp_index = 0;
  
  s_spi_wr_func(send, recv, sizeof(send));

  fp_index |= (unsigned short)recv[DOFFSET+0]<<8;
  fp_index |= (unsigned short)recv[DOFFSET+1];

  return fp_index;
}

void kcm_get_acc_data(unsigned char *buf, unsigned short len)
{
  if(len > ACC_MAX_LEN-3)
    return;

  acc_send[0] = ACC_DATA_REGID|REG_R;
  
  s_spi_wr_func(acc_send, acc_recv, DOFFSET+len);

  memcpy(buf, acc_recv+DOFFSET, len);
}

void kcm_get_rx_difftime_quality(short *diff_time, unsigned char *quality)
{
  unsigned char send[6+NOP_NUM] = {RX_TP_DIFF_REGID|REG_R,NOP};
  unsigned char recv[6+NOP_NUM];

  s_spi_wr_func(send, recv, sizeof(send));

  memcpy(quality, recv+DOFFSET, 3);

  *diff_time = 0;
  *diff_time |= (short)recv[DOFFSET+3+0] << 8;
  *diff_time |= (short)recv[DOFFSET+3+1] << 0;
}

void kcm_get_aoa_all(unsigned char *time_stamp, short *diff_time,unsigned char *quality, unsigned char *quality2, unsigned char *pdu)
{
  unsigned char send[128] = {RX_TP_DIFF_ALL_REGID|REG_R,NOP};
  unsigned char recv[128];

  s_spi_wr_func(send, recv, 1+NOP_NUM+6+2+3+3+s_rx_pdu_length);

  memcpy(time_stamp, recv+DOFFSET, 6);

  memcpy(pdu, recv+DOFFSET+6, s_rx_pdu_length);

  memcpy(quality, recv+DOFFSET+6+s_rx_pdu_length, 3);
  memcpy(quality2, recv+DOFFSET+6+s_rx_pdu_length+3, 3);

  *diff_time = 0;
  *diff_time |= (short)recv[DOFFSET+6+s_rx_pdu_length+3+3] << 8;
  *diff_time |= (short)recv[DOFFSET+6+s_rx_pdu_length+3+3+1] << 0;
}

void kcm_get_slave_ant_acc_data(unsigned char *buf, unsigned short len)
{
  if(len > ACC_MAX_LEN-3)
    return;

  acc_send[0] = SLAVE_ANT_ACC_REGID|REG_R;

  s_spi_wr_func(acc_send, acc_recv, DOFFSET+len);

  memcpy(buf, acc_recv+DOFFSET, len);
}

void kcm_get_all_acc_data(unsigned char *buf, unsigned short len)
{
  if(len > ACC_MAX_LEN-3)
    return;

  acc_send[0] = ALL_ACC__REGID|REG_R;

  s_spi_wr_func(acc_send, acc_recv, DOFFSET+len);

  memcpy(buf, acc_recv+DOFFSET, len);
}

void kcm_enter_sleep(void)
{
  unsigned char send[1] = {SLEEP_ENTER_REGID|REG_W};
  unsigned char recv[1];

  s_spi_wr_func(send, recv, sizeof(send));
}

void kcm_set_sleep_after_trx(unsigned char slee_after_trx_en)
{
  unsigned char send[2] = {SLEEP_AFTER_TRX|REG_W, slee_after_trx_en};
  unsigned char recv[2];

  s_spi_wr_func(send, recv, sizeof(send));
}

void kcm_force_off_trx(void)
{
  unsigned char send[1] = {FORCE_OFF_TRX|REG_W};
  unsigned char recv[1];

  s_spi_wr_func(send, recv, sizeof(send));
}

void kcm_set_rx_continuously(unsigned char enable)
{
  unsigned char send[2] = {RX_MODE_REGID|REG_W};
  unsigned char recv[2];

  send[1] = enable ? 0 : 1;
  
  s_spi_wr_func(send, recv, sizeof(send));
}

void kcm_set_acc_data_fully_ready(unsigned char type)
{
  unsigned char send[2] = {ACC_READY_REGID|REG_W};
  unsigned char recv[2];

  if(type == ACC_ALL)
    send[1] = ACC_ALL;
  else if(type == ACC_MASTER)
    send[1] = ACC_MASTER;
  else if(type == ACC_SLAVE)
    send[1] = ACC_SLAVE;
  else
    return;
    
  s_spi_wr_func(send, recv, sizeof(send));
}

void kcm_set_rx_timeout(unsigned int timeout)
{
  unsigned char send[5] = {RX_TIMEOUT_REGID|REG_W};
  unsigned char recv[5];

  send[1] = (timeout >> 24) & 0xFF;
  send[2] = (timeout >> 16) & 0xFF;
  send[3] = (timeout >> 8)  & 0xFF;
  send[4] = (timeout >> 0)  & 0xFF;  

  s_spi_wr_func(send, recv, sizeof(send));
}

void kcm_set_fimware(unsigned char *buf, unsigned int pos, unsigned int len)
{
  if(len > 512)
    return;

  unsigned char checksum = 0;
  
  acc_send[0] = FIRMWARE_REGID|REG_W;
  
  acc_send[1] = (pos >> 24) & 0xFF;
  acc_send[2] = (pos >> 16) & 0xFF;
  acc_send[3] = (pos >> 8) & 0xFF;
  acc_send[4] = (pos >> 0) & 0xFF;

  acc_send[5] = (len >> 24) & 0xFF;
  acc_send[6] = (len >> 16) & 0xFF;
  acc_send[7] = (len >> 8) & 0xFF;
  acc_send[8] = (len >> 0) & 0xFF;
  
  memcpy(acc_send + 9, buf, len);

  for(int i = 1; i < len + 9; ++i)
    checksum = checksum + acc_send[i];

  acc_send[len + 9] = checksum;

  s_spi_wr_func(acc_send, acc_recv, len + 10);
}

void kcm_set_fimwriteflash(void)
{
  unsigned char send[1] = {FLASH_WRT_REGID|REG_W};
  unsigned char recv[1];

  s_spi_wr_func(send, recv, sizeof(send));
}

unsigned char kcm_get_fimchecksum(void)
{
  unsigned char send[2+NOP_NUM] = {CHECK_FM_REGID|REG_R,NOP};
  unsigned char recv[2+NOP_NUM];

  s_spi_wr_func(send, recv, sizeof(send));

  return recv[DOFFSET];
}

void kcm_set_tpencryption(unsigned int enable)
{
  unsigned char send[2] = {KCM_ENCRYPTION_REGID|REG_W,NOP};
  unsigned char recv[2];

  send[1] = enable ? 1 : 0;
  
  s_spi_wr_func(send, recv, sizeof(send));
}

unsigned char kcm_get_tpencryption(void)
{
  unsigned char send[2+NOP_NUM] = {KCM_ENCRYPTION_REGID|REG_R,NOP};
  unsigned char recv[2+NOP_NUM];

  s_spi_wr_func(send, recv, sizeof(send));

  return recv[DOFFSET];
}

void kcm_initialize(KCM_CONFIG *config)
{
  kcm_set_tx_preamble_len(config->tx_preamble_len);
  kcm_set_rx_preamble_len(config->rx_preamble_len);
  kcm_set_tx_sfd(config->sfd_sequence);
  kcm_set_rx_sfd(config->sfd_sequence);
  kcm_set_prf(config->prf);
  kcm_set_tx_pdu_len(config->pdu_len);
  kcm_set_rx_pdu_len(config->pdu_len);
}

void kcm_set_spi_wr_func(KCM_WR_FUNC wr_ifs)
{
  s_spi_wr_func = wr_ifs;
}

















