/******************************************************************************
* File Name:   cy_atmodem.h
*
* Description: This file defines AT modem parameters
*
* Related Document: See README.md
*
*
*******************************************************************************
* Copyright 2020-2021, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/

#ifndef SOURCE_CY_ATMODEM_H_
#define SOURCE_CY_ATMODEM_H_

#include "atmodem_config.h"

#ifdef __cplusplus
extern "C" {
#endif


/*-- Public Definitions -------------------------------------------------*/

/* methods to power on/off modem */
#define PPP_SIMPLE_SWITCH_METHOD    1
#define PPP_POWER_STEP_METHOD       2
#define PPP_PULSE_SWITCH_METHOD     3


/* ----------------------------------------------------------------------*/
/* Parameters common to all modems */

#define AT_CMD_READY_CHECK          "AT\r\n"
#define AT_CMD_MFG                  "AT+CGMI\r"
#define AT_CMD_MODEL                "AT+CGMM\r"
#define AT_CMD_IMEI                 "AT+CGSN\r"
#define AT_CMD_ECHO_OFF             "ATE0\r"
#define AT_CMD_ECHO_ON              "ATE1\r"

#define AT_RSP_CSIM                 "+CSIM: "
#define AT_CMD_CSIM_START           "AT+CSIM=%u,\""
#define AT_CMD_CSIM_END             "\"\r\n"

#define AT_RSP_CME_ERROR            "+CME ERROR"
#define AT_RSP_ERROR                "ERROR"
#define AT_RSP_ERROR_END            "ERROR\r\n"
#define AT_RSP_OK                   "OK"
#define AT_RSP_OK_END               "OK\r\n"
#define AT_RSP_START                "\r\n"
#define AT_RSP_ABORTED              "ABORTED"

#define AT_CMD_PDP_CONTEXT_CID      1
#define AT_CMD_PDP_CONTEXT_TYPE     "IP"
#define AT_CMD_TEST_PDP_CONTEXT     "AT+CGDCONT=?\r"
#define AT_CMD_SET_PDP_CONTEXT      "AT+CGDCONT"
#define AT_CMD_SET_BAUD_RATE        "AT+IPR"
#define AT_CMD_QUERY_BAUD_RATE      "AT+IPR?\r"
#define AT_CMD_IMSI                 "AT+CIMI\r"

//#define AT_CMD_RESTORE_USER_SETTINGS "ATZ\r"
//#define AT_CMD_DEFINE_USER_SETTINGS "ATQ0 V1 E1 S0=0 &C1 &D2\r" // Quectel BG96 does not support +FCLASS=0 // SimCom7600 "ATQ0 V1 E1 S0=0 &C1 &D2 +FCLASS=0\r"
//#define AT_CMD_SAVE_USER_SETTINGS   "AT&W\r"

#define AT_CMD_SET_FLOW_CONTROL_NONE  "AT+IFC=0,0\r"
#define AT_CMD_OPERATOR_SELECTION   "AT+COPS?\r"
#define AT_CMD_OPERATOR_SELECTION_AUTO_MODE  "AT+COPS=0\r"
#define AT_CMD_QUERY_SIGNAL_QUALITY "AT+CSQ\r"

#define AT_CMD_DIAL                 "ATD*99***1#\r" //"ATD*99#\r"
#define AT_CMD_SWITCH_DATA_TO_CMD_MODE "+++"
#define AT_CMD_SWITCH_CMD_TO_DATA_MODE "ATO\r"

#define AT_CMD_QUERY_SIM_CARD_STATUS "AT+CPIN?\r"

#define AT_CMD_TEST_GSM_NETWORK     "AT+CREG=?\r"
#define AT_CMD_QUERY_GSM_NETWORK     "AT+CREG?\r"
#define AT_CMD_SET_GSM_NETWORK_PRESENTATION "AT+CREG=2\r"

#define AT_CMD_TEST_GPRS_NETWORK     "AT+CGREG=?\r"
#define AT_CMD_QUERY_GPRS_NETWORK    "AT+CGREG?\r"
#define AT_CMD_SET_GPRS_NETWORK_PRESENTATION  "AT+CGREG=2\r"

#define AT_CMD_TEST_EPS_NETWORK      "AT+CEREG=?\r"
#define AT_CMD_QUERY_EPS_NETWORK     "AT+CEREG?\r"
#define AT_CMD_SET_EPS_NETWORK_PRESENTATION  "AT+CEREG=2\r"

#define AT_CMD_TEST_PACKET_DOMAIN    "AT+CGATT=?\r"
#define AT_CMD_QUERY_PACKET_DOMAIN   "AT+CGATT?\r"
#define AT_CMD_ATTACH_PACKET_DOMAIN  "AT+CGATT=1\r"
#define AT_CMD_DETACH_PACKET_DOMAIN  "AT+CGATT=0\r"

#define AT_CMD_TEST_PDP_ADDRESS    "AT+CGPADDR=?\r"
#define AT_CMD_SHOW_PDP_ADDRESS   "AT+CGPADDR=1\r"

#define AT_CMD_SET_ERROR_MSG_FORMAT_VERBOSE    "AT+CMEE=2\r"
#define AT_CMD_SET_TA_RESPONSE_FORMAT_VERBOSE  "ATV1\r"

#if 0 // Quectel BG96 code
#define AT_CMD_SET_QCFG_BAND        "AT+QCFG=\"band\"\r"
#define AT_CMD_SET_QCFG_IOTOPMODE   "AT+QCFG=\"iotopmode\"\r"
#define AT_CMD_SET_QCFG_NWSCANSEQ   "AT+QCFG=\"nwscanseq\"\r"
#define AT_CMD_SET_QCFG_NWSCANMODE  "AT+QCFG=\"nwscanmode\"\r"
#endif

/* ----------------------------------------------------------------------*/
/* Parameters for SimCom SIM7600G-H and A7670E */
#if ((ATMODEM_HW == ATMODEM_HW_SIMCOM_7600G) || (ATMODEM_HW == ATMODEM_HW_SIMCOM_A7670E))

/* 1. modem maximum baud rate */
#define PPP_MAX_MODEM_BAUD_RATE     115200 //921600 (RT-thread add-profile failed) //3000000 (list profiles failed) // failed: 3686400, 3200000, // ok: 921600, 460800, 230400

/* 2. IO Reference Voltage pin */
#undef PPP_MODEM_IO_REF             // unused

/* 3. Power on/off pin */
#define PPP_MODEM_POWER_KEY         ATMODEM_HW_PIN_POWER_KEY

/* 4. UART RX pin */
#define PPP_MODEM_UART_RX           ATMODEM_HW_PIN_UART_RX

/* 5. UART TX pin */
#define PPP_MODEM_UART_TX           ATMODEM_HW_PIN_UART_TX

#if (ATMODEM_HW == ATMODEM_HW_SIMCOM_7600G)
  /* 6. method to power on/off modem */
  #define PPP_MODEM_POWER_METHOD      PPP_POWER_STEP_METHOD

  /* 7. 'Power on' logic level */
  #define PPP_MODEM_POWER_KEY_ON_LEVEL  0

  /* 8. 'Power off' logic level */
  #define PPP_MODEM_POWER_KEY_OFF_LEVEL 1

#elif (ATMODEM_HW == ATMODEM_HW_SIMCOM_A7670E)
  /* 6. method to power on/off modem */
  #define PPP_MODEM_POWER_METHOD      PPP_SIMPLE_SWITCH_METHOD

  /* 7. 'Power on' logic level */
  #define PPP_MODEM_POWER_KEY_ON_LEVEL  1

  /* 8. 'Power off' logic level */
  #define PPP_MODEM_POWER_KEY_OFF_LEVEL 0
#endif

/* 9. whether to send AT during wait_for_modem_ready */
#define PPP_SEND_AT_DURING_WAIT_FOR_MODEM_READY

/* 10. whether modem_can support eSIM LPA */
#define PPP_MODEM_CAN_SUPPORT_ESIM_LPA

/* 11. AT command to halt the PPP daemon running in the modem */
#define AT_CMD_HALT_PPP_DAEMON      "ATH"

/* 12. AT command to power off the modem */
#define AT_CMD_POWER_OFF_MODEM      "AT+CPOF"

/* 13. AT command to reset the modem */
#define AT_CMD_RESET                "AT+CFUN=1,1\r\n"  /* SIMCom7600 & 7000 */
//#define AT_CMD_RESET              "AT+CRESET\r\n"    /* SIMCom7600 only */

/* 14. while modem is starting up, the hint that indicates it's ready */
#define AT_RSP_READY                "PB DONE"

/* 15. AT command to query UE System Info */
#define AT_CMD_QUERY_UE_INFO        "AT+CPSI?\r"

/* 16. good pattern to look for in UE System Info response */
#define AT_RSP_UE_INFO_PATTERN_LTE    "LTE,Online"

/* 17. failure pattern to look for in UE System Info response */
#define AT_RSP_UE_INFO_PATTERN_FAILED "NO SERVICE,Online"

/* 18. AT command to start Global Positioning System (GPS) session */
#define AT_CMD_GPS_SESSION_START    "AT+CGPS=1,1\r"

/* 19. AT command to stop Global Positioning System (GPS) session */
#define AT_CMD_GPS_SESSION_STOP     "AT+CGPS=0\r"

/* 20. AT command to get Global Positioning System (GPS) info */
#define AT_CMD_GET_GPS_INFO         "AT+CGPSINFO\r"

/* 21. pattern to look for in GPS Info response */
#define AT_RSP_GET_GPS_INFO         "+CGPSINFO: "

/* 22. query the SIM card hotswap level */
#define AT_CMD_QUERY_HOTSWAP_LEVEL  "AT+UIMHOTSWAPLEVEL?"

/* 23. set the SIM card hotswap on */
#define AT_CMD_SET_HOTSWAP_ON       "AT+UIMHOTSWAPON=1"

/* 24. set DTR function mode */
#define AT_CMD_SET_DTR_FUNCTION_MODE_IGNORE    "AT&D0\r"

/* 25. set connect response format */
#define AT_CMD_SET_CONNECT_RESPONSE_FORMAT     "ATX0\r"


/* ----------------------------------------------------------------------*/
/* Parameters for Murata Type-1SC */
#elif (ATMODEM_HW == ATMODEM_HW_MURATA_1SC)

/* 1. modem maximum baud rate */
#define PPP_MAX_MODEM_BAUD_RATE     115200

/* 2. IO Reference Voltage pin */
#define PPP_MODEM_IO_REF            ATMODEM_HW_PIN_IO_REF

/* 3. Power on/off pin */
#define PPP_MODEM_POWER_KEY         ATMODEM_HW_PIN_POWER_KEY

/* 4. UART RX pin */
#define PPP_MODEM_UART_RX           ATMODEM_HW_PIN_UART_RX

/* 5. UART TX pin */
#define PPP_MODEM_UART_TX           ATMODEM_HW_PIN_UART_TX

/* 6. method to power on/off modem */
#define PPP_MODEM_POWER_METHOD      PPP_SIMPLE_SWITCH_METHOD

/* 7. 'Power on' logic level */
#define PPP_MODEM_POWER_KEY_ON_LEVEL  1

/* 8. 'Power off' logic level */
#define PPP_MODEM_POWER_KEY_OFF_LEVEL 0

/* 9. whether to send AT during wait_for_modem_ready */
#define PPP_SEND_AT_DURING_WAIT_FOR_MODEM_READY

/* 10. whether modem_can support eSIM LPA */
#undef PPP_MODEM_CAN_SUPPORT_ESIM_LPA   // unsupported

/* 11. AT command to halt the PPP daemon running in the modem */
#define AT_CMD_HALT_PPP_DAEMON      "AT%H"

/* 12. AT command to power off the modem */
#undef AT_CMD_POWER_OFF_MODEM           // unsupported

/* 13. AT command to reset the modem */
#undef AT_CMD_RESET                     // unsupported

/* 14. while modem is starting up, the hint that indicates it's ready */
#undef AT_RSP_READY                     // unsupported

/* 15. AT command to query UE System Info */
#undef AT_CMD_QUERY_UE_INFO             // unsupported

/* 16. good pattern to look for in UE System Info response */
#undef AT_RSP_UE_INFO_PATTERN_LTE       // unsupported

/* 17. failure pattern to look for in UE System Info response */
#undef AT_RSP_UE_INFO_PATTERN_FAILED    // unsupported

/* 18. AT command to start Global Positioning System (GPS) session */
#undef AT_CMD_GPS_SESSION_START         // unsupported

/* 19. AT command to stop Global Positioning System (GPS) session */
#undef AT_CMD_GPS_SESSION_STOP          // unsupported

/* 20. AT command to get Global Positioning System (GPS) info */
#undef AT_CMD_GET_GPS_INFO              // unsupported

/* 21. pattern to look for in GPS Info response */
#undef AT_RSP_GET_GPS_INFO              // unsupported

/* 22. query the SIM card hotswap level */
#undef AT_CMD_QUERY_HOTSWAP_LEVEL       // unsupported

/* 23. set the SIM card hotswap on */
#undef AT_CMD_SET_HOTSWAP_ON            // unsupported

/* 24. set DTR function mode */
#undef AT_CMD_SET_DTR_FUNCTION_MODE_IGNORE  // unsupported

/* 25. set connect response format */
#undef AT_CMD_SET_CONNECT_RESPONSE_FORMAT   // unsupported


/* ----------------------------------------------------------------------*/
/* Parameters for Quectel BG96 */
#elif (ATMODEM_HW == ATMODEM_HW_QUECTEL_BG96)

/* 1. modem maximum baud rate */
#define PPP_MAX_MODEM_BAUD_RATE     115200

/* 2. IO Reference Voltage pin */
#undef PPP_MODEM_IO_REF             // unused

/* 3. Power on/off pin */
#define PPP_MODEM_POWER_KEY         ATMODEM_HW_PIN_POWER_KEY

/* 4. UART RX pin */
#define PPP_MODEM_UART_RX           ATMODEM_HW_PIN_UART_RX

/* 5. UART TX pin */
#define PPP_MODEM_UART_TX           ATMODEM_HW_PIN_UART_TX

/* 6. method to power on/off modem */
#define PPP_MODEM_POWER_METHOD      PPP_SIMPLE_SWITCH_METHOD

/* 7. 'Power on' logic level */
#define PPP_MODEM_POWER_KEY_ON_LEVEL  1

/* 8. 'Power off' logic level */
#define PPP_MODEM_POWER_KEY_OFF_LEVEL 0

/* 9. whether to send AT during wait_for_modem_ready */
#define PPP_SEND_AT_DURING_WAIT_FOR_MODEM_READY

/* 10. whether modem_can support eSIM LPA */
#undef PPP_MODEM_CAN_SUPPORT_ESIM_LPA   // unsupported

/* 11. AT command to halt the PPP daemon running in the modem */
#define AT_CMD_HALT_PPP_DAEMON      "ATH"

/* 12. AT command to power off the modem */
#undef AT_CMD_POWER_OFF_MODEM           // unsupported

/* 13. AT command to reset the modem */
#undef AT_CMD_RESET                     // unsupported

/* 14. while modem is starting up, the hint that indicates it's ready */
#undef AT_RSP_READY                     // unsupported

/* 15. AT command to query UE System Info */
#undef AT_CMD_QUERY_UE_INFO             // unsupported

/* 16. good pattern to look for in UE System Info response */
#undef AT_RSP_UE_INFO_PATTERN_LTE       // unsupported

/* 17. failure pattern to look for in UE System Info response */
#undef AT_RSP_UE_INFO_PATTERN_FAILED    // unsupported

/* 18. AT command to start Global Positioning System (GPS) session */
#undef AT_CMD_GPS_SESSION_START         // unsupported

/* 19. AT command to stop Global Positioning System (GPS) session */
#undef AT_CMD_GPS_SESSION_STOP          // unsupported

/* 20. AT command to get Global Positioning System (GPS) info */
#undef AT_CMD_GET_GPS_INFO              // unsupported

/* 21. pattern to look for in GPS Info response */
#undef AT_RSP_GET_GPS_INFO              // unsupported

/* 22. query the SIM card hotswap level */
#undef AT_CMD_QUERY_HOTSWAP_LEVEL       // unsupported

/* 23. set the SIM card hotswap on */
#undef AT_CMD_SET_HOTSWAP_ON            // unsupported

/* 24. set DTR function mode */
#define AT_CMD_SET_DTR_FUNCTION_MODE_IGNORE    "AT&D0\r"

/* 25. set connect response format */
#define AT_CMD_SET_CONNECT_RESPONSE_FORMAT     "ATX0\r"


// 
/* ----------------------------------------------------------------------*/
/* Parameters for U-Blox LARA R280 */
#elif (ATMODEM_HW == ATMODEM_HW_UBLOX_LARA_R280)

/* 1. modem maximum baud rate */
#define PPP_MAX_MODEM_BAUD_RATE     115200

/* 2. IO Reference Voltage pin */
#undef PPP_MODEM_IO_REF             // unused

/* 3. Power on/off pin */
#define PPP_MODEM_POWER_KEY         ATMODEM_HW_PIN_POWER_KEY

/* 4. UART RX pin */
#define PPP_MODEM_UART_RX           ATMODEM_HW_PIN_UART_RX

/* 5. UART TX pin */
#define PPP_MODEM_UART_TX           ATMODEM_HW_PIN_UART_TX

/* 6. method to power on/off modem */
#define PPP_MODEM_POWER_METHOD      PPP_PULSE_SWITCH_METHOD

/* 7. 'Power on' logic level */
#define PPP_MODEM_POWER_KEY_ON_LEVEL  1

/* 8. 'Power off' logic level */
#define PPP_MODEM_POWER_KEY_OFF_LEVEL 0

/* 9. whether to send AT during wait_for_modem_ready */
#define PPP_SEND_AT_DURING_WAIT_FOR_MODEM_READY

/* 10. whether modem_can support eSIM LPA */
#undef PPP_MODEM_CAN_SUPPORT_ESIM_LPA   // unsupported

/* 11. AT command to halt the PPP daemon running in the modem */
#define AT_CMD_HALT_PPP_DAEMON      "ATH"

/* 12. AT command to power off the modem */
#define AT_CMD_POWER_OFF_MODEM      "AT+CPWROFF"

/* 13. AT command to reset the modem */
#define AT_CMD_RESET                "AT+CFUN=16"

/* 14. while modem is starting up, the hint that indicates it's ready */
#undef AT_RSP_READY                     // unsupported

/* 15. AT command to query UE System Info */
#undef AT_CMD_QUERY_UE_INFO             // unsupported

/* 16. good pattern to look for in UE System Info response */
#undef AT_RSP_UE_INFO_PATTERN_LTE       // unsupported

/* 17. failure pattern to look for in UE System Info response */
#undef AT_RSP_UE_INFO_PATTERN_FAILED    // unsupported

/* 18. AT command to start Global Positioning System (GPS) session */
#undef AT_CMD_GPS_SESSION_START         // unsupported

/* 19. AT command to stop Global Positioning System (GPS) session */
#undef AT_CMD_GPS_SESSION_STOP          // unsupported

/* 20. AT command to get Global Positioning System (GPS) info */
#undef AT_CMD_GET_GPS_INFO              // unsupported

/* 21. pattern to look for in GPS Info response */
#undef AT_RSP_GET_GPS_INFO              // unsupported

/* 22. query the SIM card hotswap level */
#undef AT_CMD_QUERY_HOTSWAP_LEVEL       // unsupported

/* 23. set the SIM card hotswap on */
#undef AT_CMD_SET_HOTSWAP_ON            // unsupported

/* 24. set DTR function mode */
#define AT_CMD_SET_DTR_FUNCTION_MODE_IGNORE    "AT&D0\r"

/* 25. set connect response format */
#define AT_CMD_SET_CONNECT_RESPONSE_FORMAT     "ATX0\r"

#endif /* ATMODEM_HW */


#ifdef __cplusplus
}
#endif

#endif /* SOURCE_CY_ATMODEM_H_*/
