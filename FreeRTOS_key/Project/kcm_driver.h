
/*! ----------------------------------------------------------------------------
 * @file    kcm_driver.h
 * @brief   Defines the version info for the KCM2000 device driver including its API
 *
 * @ API version: V1.0.1
 *
 * @attention
 *
 * Copyright 2020 (c) Kunchen Ltd.
 *
 * All rights reserved.
 *
 */

#ifndef _KCM_DRIVER_H_
#define _KCM_DRIVER_H_

#define KCM2000_DEV_ID 0x2d9c0501
#define KCM2001_DEV_ID 0x2d9c0502


typedef enum
{
  KCM_PLEN_32   = 32,
  KCM_PLEN_64   = 64,
  KCM_PLEN_128  = 128,
  KCM_PLEN_256  = 255
}KCM_PLEN;

typedef enum
{
  KCM_PRF_1M    = 1000,
  KCM_PRF_500K  = 500
}KCM_PRF;

typedef enum
{
  KCM_INT_RX_DONE = 0x0001,
  KCM_INT_CRC_ERR = 0x0002,
  KCM_INT_RX_TO   = 0x0004,
  KCM_INT_LDE_ERR = 0x0008,
  KCM_INT_PANIC   = 0x0010,
  KCM_INT_TX_DONE = 0x0100,
  KCM_INT_TX_FAIL = 0x0200,
  KCM_INT_ALL     = 0xFFFF
}KCM_INT;

typedef enum
{
  IMMEDIATE = 0,
  DELAY = 1
}KCM_TX_DELAY;

typedef enum
{
  KCM_ANT_AOA = 0,
  KCM_ANT_TDOA1 = 2,
  KCM_ANT_TDOA2 = 3
}KCM_ANT_MODE;

typedef enum
{
  ACC_ALL = 0,
  ACC_MASTER = 1,
  ACC_SLAVE = 2
}KCM_ACC_TYPE;

typedef struct
{
  unsigned int tx_preamble_len;
  unsigned int rx_preamble_len;
  unsigned int sfd_sequence;
  unsigned int prf;
  unsigned int pdu_len;
}KCM_CONFIG;


typedef void (*KCM_WR_FUNC)(unsigned char* send, unsigned char* recv, unsigned int len);


void kcm_set_debug_cmd_on(unsigned char on_off);
unsigned int kcm_get_ver(void);
void kcm_get_detailed_ver(unsigned int *fw_ver, unsigned int *hw_ver, unsigned int logic_ver[3]);
unsigned int kcm_get_dev_id(void);
char kcm_get_temperature(void);
void kcm_get_vcc(unsigned char vcc[6]);
void kcm_set_tx_dsa(unsigned char dsa);
unsigned char kcm_get_tx_dsa(void);
void kcm_set_rx_dsa(unsigned char dsa);
unsigned char kcm_get_rx_dsa(void);
void kcm_set_da0(unsigned char da);
unsigned char kcm_get_da0(void);
void kcm_set_da1(unsigned char da);
unsigned char kcm_get_da1(void);
void kcm_set_tx_preamble_len(unsigned char len);
unsigned char kcm_get_tx_preamble_len(void);
void kcm_set_rx_preamble_len(unsigned char len);
unsigned char kcm_get_rx_preamble_len(void);
void kcm_set_tx_sfd(unsigned char sfd);
unsigned char kcm_get_tx_sfd(void);
void kcm_set_rx_sfd(unsigned char sfd);
unsigned char kcm_get_rx_sfd(void);
void kcm_set_prf(unsigned short prf);
unsigned short kcm_get_prf(void);
void kcm_set_tx_pdu_len(unsigned char len);
unsigned char kcm_get_tx_pdu_len(void);
void kcm_set_rx_pdu_len(unsigned char len);
unsigned char kcm_get_rx_pdu_len(void);
void kcm_set_int_status(unsigned short mask);
unsigned short kcm_get_int_status(void);
void kcm_set_ant_mode(unsigned char mode);
unsigned char kcm_get_ant_mode(void);
void kcm_set_fec_code_len(unsigned char len);
unsigned char kcm_get_fec_code_len(void);
void kcm_set_tx_pdu(unsigned char *pdu, unsigned char offset);
void kcm_set_tx_delay_time(unsigned int delay_time);
void kcm_set_tx_start(unsigned char delay_type);
void kcm_get_tx_time_stamp(unsigned char *time_stamp);
void kcm_set_tx_pdu_fully(unsigned char *pdu);
void kcm_set_tx_pdu_with_offset_length(unsigned char addr, unsigned char len, unsigned char *pdu);
void kcm_set_rx_start(unsigned int delay_time);
void kcm_get_rx_pdu(unsigned char *pdu);
void kcm_get_rx_time_stamp(unsigned char *time_stamp);
void kcm_get_rx_quality(unsigned char *quality);
void kcm_get_rx_pdu_time_quality(unsigned char *time_stamp, unsigned char *quality, unsigned char *pdu);
unsigned char kcm_get_noise_power(void);
unsigned short kcm_get_prejudge_val(void);
void kcm_get_acc_data(unsigned char *buf, unsigned short len);
void kcm_get_rx_difftime_quality(short *diff_time, unsigned char *quality);
void kcm_get_aoa_all(unsigned char *time_stamp, short *diff_time,unsigned char *quality, unsigned char *quality2, unsigned char *pdu);
void kcm_get_slave_ant_acc_data(unsigned char *buf, unsigned short len);
void kcm_get_all_acc_data(unsigned char *buf, unsigned short len);
void kcm_enter_sleep(void);
void kcm_set_sleep_after_trx(unsigned char slee_after_trx_en);
void kcm_force_off_trx(void);
void kcm_set_rx_continuously(unsigned char enable);
void kcm_set_acc_data_fully_ready(unsigned char type);
void kcm_set_rx_timeout(unsigned int timeout);
void kcm_set_fimware(unsigned char *buf, unsigned int pos, unsigned int len);
void kcm_set_fimwriteflash(void);
unsigned char kcm_get_fimchecksum(void);
void kcm_set_tpencryption(unsigned int enable);
unsigned char kcm_get_tpencryption(void);




void kcm_initialize(KCM_CONFIG *config);
void kcm_set_spi_wr_func(KCM_WR_FUNC wr_ifs);



#endif



