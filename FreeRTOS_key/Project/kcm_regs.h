/*! ------------------------------------------------------------------------------------------------------------------
 * @file    kcm_regs.h
 * @brief   KCM200x Register Definitions
 *          This file supports assembler and C development for KCM200x enabled devices
 *
 * @attention
 *
 * Copyright 2013 (c) Kunchen Ltd.
 *
 * All rights reserved.
 *
 */

#ifndef _KCM_REGS_H_
#define _KCM_REGS_H_



#define DEBUG_EN_REGID        0x00  /* Enable Or Disable Debug Mode. WO */

#define VERSION_REGID         0x01  /* Device Version. RO */

#define DETAIL_VERSION_REGID  0x02  /* Detail Version. RO */

#define DEVID_REGID           0x03  /* Device ID. RO */

#define TEMPERATURE_REGID     0x04  /* Device Temperature. RO */

#define VOLTAGE_REGID         0x05  /* Device Voltage. RO */

#define VGA_TX_DSA_REGID      0x11  /* Tx DSA. RW */

#define VGA_RX_DSA_REGID      0x12  /* Rx DSA. RW */

#define VGA_DA0_REGID         0x13  /* DA0. RW */

#define VGA_DA1_REGID         0x14  /* DA1. RW */

#define TX_PREAM_LEN_REGID    0x15  /* TX Preamble Length. RW*/

#define TX_SFD_REGID          0x16  /* tx SFD(Start Frame Delimiter) Sequence. RW*/

#define PRF_REGID             0x17  /* PRF(Pulse Repetition Frequencies) RW*/

#define TX_PDU_LENGTH_REGID   0x18  /* TX PDU(Protocol Data Unit) Length. RW */

#define INT_STATUS_REGID      0x19  /* Interrupt Status. RW */

#define RX_PREAM_LEN_REGID    0x1a  /* RX Preamble Length. RW*/

#define PORT_MODE_REGID       0x1b  /* Antenna Mode. RW */

#define FEC_CODE_LEN_REGID    0x1c  /* fec code length. RW */

#define RX_SFD_REGID          0x1d  /* RX SFD. RW*/

#define RX_PDU_LENGTH_REGID   0x1e  /* RX PDU Length. RW*/

#define TX_PDU_DATA_REGID     0x21  /* PDU(Protocol Data Unit) Data. WO */

#define TX_DELAY_REGID        0x22  /* Tx Delay Time(delay_time * 5000 ps). WO */

#define TX_START_REGID        0x23  /* Tx Start. WO */

#define TX_TIMESTAMP_REGID    0x24  /* Tx TimeStamp. RO */

#define TX_PDU_ALL_REGID      0x25  /* Tx PDU Fully. */

#define TX_PDU_OFST_REGID     0x26  /* Tx PDU Fully. */

#define RX_START_REGID        0x31  /* Rx Start. WO */

#define RX_PDU_DATA_REGID     0x32  /* PDU(Protocol Data Unit) Data. RO */

#define RX_TIMESTAMP_REGID    0x33  /* Rx TimeStamp. RO */

#define RX_FQUAL_REGID        0x34  /* Rx Frame Quality Information. RO */

#define PUD_AND_FQUAL_REGID   0x35  /* PDU + Rx Frame Quality RO */

#define NOISE_POWER_REGID     0x36  /* Noise Power. RO */

#define FP_INDEX_REGID        0x37  /* First Path Index. RO */

#define ACC_DATA_REGID        0x38  /* Accumulated CIR(channel impulse response) Data Memory. RO */

#define RX_TP_DIFF_REGID      0x39  /* AOA TDOA. RO */

#define RX_TP_DIFF_ALL_REGID  0x3a  /* AOA All data. RO */

#define SLAVE_ANT_ACC_REGID   0x3b  /* Slave ANT acc. RO */

#define ALL_ACC__REGID        0x3c  /* AOA acc. RO */

#define SLEEP_ENTER_REGID     0x41  /* ENTER Sleep. WO */

#define SLEEP_AFTER_TRX       0x42  /* SLEEP AFTER Tx And Rx. WO */
			
#define FORCE_OFF_TRX         0x43  /* Force Off Tranxceiver. WO */

#define ACC_READY_REGID       0x44  /* Let KCM Prepare ACC Data. WO*/

#define RX_MODE_REGID         0x45  /* Set Rx Mode. WO */

#define RX_TIMEOUT_REGID      0x46  /* Set Rx Timeout. WO */

#define FIRMWARE_REGID        0x7a  /* Firmware Write. WO */

#define FLASH_WRT_REGID       0x7b  /* Write Firmware to flash. WO */

#define CHECK_FM_REGID        0x7c  /* Check Firmware Write. RO */

#define KCM_ENCRYPTION_REGID  0x7d  /* tp encryption */

#endif
