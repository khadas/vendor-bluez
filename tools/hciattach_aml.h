/*
 *
 *  Copyright (c) 2020-2021, The Linux Foundation. All rights reserved.
 *  Not a Contribution.
 *
 *  Copyright 2012 The Android Open Source Project
 *
 *  Licensed under the Apache License, Version 2.0 (the "License"); you
 *  may not use this file except in compliance with the License. You may
 *  obtain a copy of the License at
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or
 *  implied. See the License for the specific language governing
 *  permissions and limitations under the License.
 *
 */

#ifndef HW_AML_H
#define HW_AML_H

/* HCI Packet types */
#define HCI_COMMAND_PKT      0x01
#define HCI_ACLDATA_PKT      0x02
#define HCI_SCODATA_PKT      0x03
#define HCI_EVENT_PKT        0x04
#define HCI_CMD_IND            (1)
#define HCI_COMMAND_HDR_SIZE   (3)
#define MSM_ENABLE_FLOW_CTRL   16
#define MSM_DISABLE_FLOW_CTRL  17
/**** baud rates ****/
#define USERIAL_BAUD_300        0
#define USERIAL_BAUD_600        1
#define USERIAL_BAUD_1200       2
#define USERIAL_BAUD_2400       3
#define USERIAL_BAUD_9600       4
#define USERIAL_BAUD_19200      5
#define USERIAL_BAUD_57600      6
#define USERIAL_BAUD_115200     7
#define USERIAL_BAUD_230400     8
#define USERIAL_BAUD_460800     9
#define USERIAL_BAUD_921600     10
#define USERIAL_BAUD_1M         11
#define USERIAL_BAUD_1_5M       12
#define USERIAL_BAUD_2M         13
#define USERIAL_BAUD_3M         14
#define USERIAL_BAUD_4M         15
#define USERIAL_BAUD_AUTO       16
#define DELIM " =\n\t\r"
#define MAX_LINE_LEN 255

/*aml fw path*/
#define AML_BT_PATH "/etc/bluetooth/aml"
#define FW_VER_FILE AML_BT_PATH"/fw_version"

/**********bt fw&config**********/
#define AML_BT_CONFIG_RF_FILE   AML_BT_PATH"/aml_bt_rf.txt"
#define AML_W1_BT_FW_UART_FILE  AML_BT_PATH"/w1_bt_fw_uart.bin"
#define AML_W1U_BT_FW_UART_FILE AML_BT_PATH"/w1u_bt_fw_uart.bin"
#define AML_W1U_BT_FW_USB_FILE  AML_BT_PATH"/w1u_bt_fw_usb.bin"
#define AML_W2_BT_FW_UART_FILE  AML_BT_PATH"/w2_bt_fw_uart.bin"
#define AML_W2L_BT_FW_UART_FILE AML_BT_PATH"/w2l_bt_15p4_fw_uart.bin"


/**********a2dp mode cfg**********/
#define AML_A2DP_CFG_FILE AML_BT_PATH"/a2dp_mode_cfg.txt"
/**************DEFAULT BT MAC***************************/
#define NUIFYKEY_MAC "/sys/module/kernel/parameters/btmac"
/*aml bt module*/
#define W1_UART     0x01
#define W1U_UART    0x02
#define W1U_USB     0x03
#define W2_UART     0x04
#define W2L_UART    0x05
#define W3_UART     0x06


#ifndef FALSE
#define FALSE  0
#endif

#ifndef TRUE
#define TRUE   (!FALSE)
#endif


#define HCI_MAX_CMD_SIZE       260
#define HCI_MAX_EVENT_SIZE     260
/*opcode*/
#define HCI_RESET              0x0C03

typedef struct {
	int module_type;
	char *fw_file;
} vnd_fw_t;

typedef struct {
	int module_type;
	char *chipid;
} vnd_chip_t;

/* vendor serial control block */
typedef struct
{
	int fd;                     /* fd to Bluetooth device */
	struct termios termios;     /* serial terminal of BT port */
	char port_name[256];
} vnd_userial_cb_t;

typedef struct {
	unsigned short    opcode;
	unsigned char     plen;
} __attribute__ ((packed))  hci_command_hdr;

typedef int (*action_act)(const char * p_name, char * p_value);
typedef struct {
    const char *entry_name;
    action_act p_action;
} d_entry_t;



#endif//HW_AML_H

