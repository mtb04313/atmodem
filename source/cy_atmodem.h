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

#define AT_CMD_READY_CHECK          "AT\r"
#define AT_CMD_MFG                  "AT+CGMI\r"
#define AT_CMD_MODEL                "AT+CGMM\r"
#define AT_CMD_IMEI                 "AT+CGSN\r"
#define AT_CMD_VERSION              "AT+CGMR\r"
#define AT_CMD_CAPABILITIES         "AT+GCAP\r"

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
#define AT_RSP_START_OK             "\r\nOK"
#define AT_RSP_ABORTED              "ABORTED"

#define AT_CMD_PDP_CONTEXT_CID      1
#define AT_CMD_PDP_CONTEXT_TYPE     "IP"

#define AT_CMD_QUERY_PDP_CONTEXT    "AT+CGDCONT?\r"
#define AT_CMD_SET_PDP_CONTEXT      "AT+CGDCONT"
#define AT_CMD_SET_BAUD_RATE        "AT+IPR"
#define AT_CMD_QUERY_BAUD_RATE      "AT+IPR?\r"
#define AT_CMD_IMSI                 "AT+CIMI\r"

#define AT_CMD_GET_PHONE_FUNCTIONALITY       "AT+CFUN?\r"
#define AT_CMD_SET_PHONE_FUNCTIONALITY_FULL  "AT+CFUN=1\r"
#define AT_CMD_SET_PHONE_FUNCTIONALITY_MIN   "AT+CFUN=0\r"

//#define AT_CMD_RESTORE_USER_SETTINGS "ATZ\r"
//#define AT_CMD_DEFINE_USER_SETTINGS "ATQ0 V1 E1 S0=0 &C1 &D2\r" // Quectel BG96 does not support +FCLASS=0 // SimCom7600 "ATQ0 V1 E1 S0=0 &C1 &D2 +FCLASS=0\r"
//#define AT_CMD_SAVE_USER_SETTINGS   "AT&W\r"

#define AT_CMD_SET_FLOW_CONTROL_NONE  "AT+IFC=0,0\r"
#define AT_CMD_OPERATOR_SELECTION   "AT+COPS?\r"
#define AT_CMD_OPERATOR_SELECTION_AUTO_MODE  "AT+COPS=0\r"
#define AT_CMD_QUERY_SIGNAL_QUALITY "AT+CSQ\r"

//#define AT_CMD_DIAL                 "ATD*99***1#\r"
#define AT_CMD_DIAL                 "ATD*99#\r"

#define AT_CMD_SWITCH_DATA_TO_CMD_MODE "+++"
#define AT_CMD_SWITCH_CMD_TO_DATA_MODE "ATO\r"

#define AT_CMD_QUERY_SIM_CARD_STATUS "AT+CPIN?\r"

#define AT_CMD_TEST_PDP_ADDRESS    "AT+CGPADDR=?\r"
#define AT_CMD_SHOW_PDP_ADDRESS   "AT+CGPADDR=1\r"

#define AT_CMD_SET_ERROR_MSG_FORMAT_VERBOSE    "AT+CMEE=2\r"
#define AT_CMD_SET_TA_RESPONSE_FORMAT_VERBOSE  "ATV1\r"

#ifdef ATMODEM_HW_PIN_UART_RTS
#define PPP_MODEM_RTS               ATMODEM_HW_PIN_UART_RTS
#else
#undef PPP_MODEM_RTS
#endif

/* ----------------------------------------------------------------------*/
/* Parameters for SimCom SIM7600G-H and A7670E */
#if ((ATMODEM_HW == ATMODEM_HW_SIMCOM_7600G)   || \
     (ATMODEM_HW == ATMODEM_HW_SIMCOM_A7670E)  || \
     (ATMODEM_HW == ATMODEM_HW_SIMCOM_7000G))

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

#elif ((ATMODEM_HW == ATMODEM_HW_SIMCOM_A7670E)  || \
       (ATMODEM_HW == ATMODEM_HW_SIMCOM_7000G))
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
#define AT_CMD_HALT_PPP_DAEMON      "ATH\r"

/* 12. AT command to power off the modem */
#if (ATMODEM_HW == ATMODEM_HW_SIMCOM_7000G)
//#define AT_CMD_POWER_OFF_MODEM      "AT+CPOWD=1\r"
#undef AT_CMD_POWER_OFF_MODEM           // unsupported
#elif ((ATMODEM_HW == ATMODEM_HW_SIMCOM_A7670E)  || \
       (ATMODEM_HW == ATMODEM_HW_SIMCOM_7600G))
#define AT_CMD_POWER_OFF_MODEM      "AT+CPOF\r"
#endif

/* 13. AT command to reset the modem */
#define AT_CMD_RESET                "AT+CFUN=1,1\r"

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
#undef AT_CMD_QUERY_HOTSWAP_LEVEL

/* 23. set the SIM card hotswap on */
#undef AT_CMD_SET_HOTSWAP_ON

/* 24. set DTR function mode */
#define AT_CMD_SET_DTR_FUNCTION_MODE_IGNORE    "AT&D0\r"

/* 25. set connect response format */
#define AT_CMD_SET_CONNECT_RESPONSE_FORMAT     "ATX0\r"

/* 26. get SIM card profile ICCID */
#if (ATMODEM_HW == ATMODEM_HW_SIMCOM_7000G)
#define AT_CMD_ICCID                "AT+CCID\r"
#elif ((ATMODEM_HW == ATMODEM_HW_SIMCOM_A7670E)  || \
       (ATMODEM_HW == ATMODEM_HW_SIMCOM_7600G))
#define AT_CMD_ICCID                "AT+CICCID\r"
#endif

/* 27. activate PDP context */
#if (ATMODEM_HW == ATMODEM_HW_SIMCOM_7000G)
//#define AT_CMD_ACTIVATE_PDP_CONTEXT "AT+CGACT=1,1\r"
#undef AT_CMD_ACTIVATE_PDP_CONTEXT      // PPP fails to connect
#elif ((ATMODEM_HW == ATMODEM_HW_SIMCOM_A7670E)  || \
       (ATMODEM_HW == ATMODEM_HW_SIMCOM_7600G))
#undef AT_CMD_ACTIVATE_PDP_CONTEXT      // PPP fails to connect
#endif

/* 28. enable BIP */
#undef AT_CMD_ENABLE_BIP                // unsupported

/* 29. Quectel QCFG band */
#undef AT_CMD_SET_QCFG_BAND             // unsupported

/* 30. Quectel QCFG iotopmode */
#undef AT_CMD_SET_QCFG_IOTOPMODE        // unsupported

/* 31. Quectel QCFG nwscanseq */
#undef AT_CMD_SET_QCFG_NWSCANSEQ        // unsupported

/* 32. Quectel QCFG nwscanmode */
#undef AT_CMD_SET_QCFG_NWSCANMODE       // unsupported

/* 33. Modem Identity Info I */
#define AT_CMD_IDENT_0              "ATI\r"

/* 34. Modem Identity Info II */
#undef AT_CMD_IDENT_6                   // unsupported
#if (ATMODEM_HW == ATMODEM_HW_SIMCOM_A7670E)
#define AT_CMD_IDENT_6              "AT+SIMCOMATI\r"
#elif ((ATMODEM_HW == ATMODEM_HW_SIMCOM_7000G)  || \
       (ATMODEM_HW == ATMODEM_HW_SIMCOM_7600G))
#undef AT_CMD_IDENT_6                   // unsupported
#endif

/* 35. Modem Identity Info III */
#undef AT_CMD_IDENT_9                   // unsupported

/* 36. Check if PDP Context command is available */
#if (ATMODEM_HW == ATMODEM_HW_SIMCOM_7000G)
#undef AT_CMD_TEST_PDP_CONTEXT          // response is too verbose; fail to read OK
#elif ((ATMODEM_HW == ATMODEM_HW_SIMCOM_A7670E)  || \
       (ATMODEM_HW == ATMODEM_HW_SIMCOM_7600G))
#define AT_CMD_TEST_PDP_CONTEXT     "AT+CGDCONT=?\r"
#endif

/* 37. Enable SIM Toolkit */
#if (ATMODEM_HW == ATMODEM_HW_SIMCOM_A7670E)
#define AT_CMD_ENABLE_STK           "AT+ENSTK=1\r"
#elif ((ATMODEM_HW == ATMODEM_HW_SIMCOM_7000G)  || \
       (ATMODEM_HW == ATMODEM_HW_SIMCOM_7600G))
#undef AT_CMD_ENABLE_STK                // unsupported
#endif

/* 38. Test GSM (2G) Network */
#define AT_CMD_TEST_GSM_NETWORK     "AT+CREG=?\r"

/* 39. Query GSM (2G) Network */
#define AT_CMD_QUERY_GSM_NETWORK     "AT+CREG?\r"

/* 40. Set GSM (2G) Network Presentation Number */
#define AT_CMD_SET_GSM_NETWORK_PRESENTATION "AT+CREG=2\r"

/* 41. Test GPRS (3G) Network */
#define AT_CMD_TEST_GPRS_NETWORK     "AT+CGREG=?\r"

/* 42. Query GPRS (3G) Network */
#define AT_CMD_QUERY_GPRS_NETWORK    "AT+CGREG?\r"

/* 43. Set GPRS (3G) Network Presentation Number */
#define AT_CMD_SET_GPRS_NETWORK_PRESENTATION  "AT+CGREG=2\r"

/* 44. Test EPS (4G) Network */
#define AT_CMD_TEST_EPS_NETWORK      "AT+CEREG=?\r"

/* 45. Query EPS (4G) Network */
#define AT_CMD_QUERY_EPS_NETWORK     "AT+CEREG?\r"

/* 46. Set EPS (4G) Network Presentation Number */
#define AT_CMD_SET_EPS_NETWORK_PRESENTATION  "AT+CEREG=2\r"

/* 47. Test GPRS Service / Packet Domain State */
#define AT_CMD_TEST_PACKET_DOMAIN    "AT+CGATT=?\r"

/* 48. Query GPRS Service / Packet Domain State */
#define AT_CMD_QUERY_PACKET_DOMAIN   "AT+CGATT?\r"

/* 49. Attach to GPRS Service / Packet Domain */
#define AT_CMD_ATTACH_PACKET_DOMAIN  "AT+CGATT=1\r"

/* 50. Detach from GPRS Service / Packet Domain */
#define AT_CMD_DETACH_PACKET_DOMAIN  "AT+CGATT=0\r"


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
#define AT_CMD_HALT_PPP_DAEMON      "AT%H\r"

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

/* 26. get SIM card profile ICCID */
#define AT_CMD_ICCID                "AT%CCID\r"

/* 27. activate PDP context */
#undef AT_CMD_ACTIVATE_PDP_CONTEXT      // unsupported

/* 28. enable BIP */
#undef AT_CMD_ENABLE_BIP                // unsupported

/* 29. Quectel QCFG band */
#undef AT_CMD_SET_QCFG_BAND             // unsupported

/* 30. Quectel QCFG iotopmode */
#undef AT_CMD_SET_QCFG_IOTOPMODE        // unsupported

/* 31. Quectel QCFG nwscanseq */
#undef AT_CMD_SET_QCFG_NWSCANSEQ        // unsupported

/* 32. Quectel QCFG nwscanmode */
#undef AT_CMD_SET_QCFG_NWSCANMODE       // unsupported

/* 33. Modem Identity Info I */
#define AT_CMD_IDENT_0              "ATI\r"

/* 34. Modem Identity Info II */
#undef AT_CMD_IDENT_6                   // unsupported

/* 35. Modem Identity Info III */
#undef AT_CMD_IDENT_9                   // unsupported

/* 36. Check if PDP Context command is available */
#define AT_CMD_TEST_PDP_CONTEXT     "AT+CGDCONT=?\r"

/* 37. Enable SIM Toolkit */
#undef AT_CMD_ENABLE_STK                // unsupported

/* 38. Test GSM (2G) Network */
#define AT_CMD_TEST_GSM_NETWORK     "AT+CREG=?\r"

/* 39. Query GSM (2G) Network */
#define AT_CMD_QUERY_GSM_NETWORK     "AT+CREG?\r"

/* 40. Set GSM (2G) Network Presentation Number */
#define AT_CMD_SET_GSM_NETWORK_PRESENTATION "AT+CREG=2\r"

/* 41. Test GPRS (3G) Network */
#define AT_CMD_TEST_GPRS_NETWORK     "AT+CGREG=?\r"

/* 42. Query GPRS (3G) Network */
#define AT_CMD_QUERY_GPRS_NETWORK    "AT+CGREG?\r"

/* 43. Set GPRS (3G) Network Presentation Number */
#define AT_CMD_SET_GPRS_NETWORK_PRESENTATION  "AT+CGREG=2\r"

/* 44. Test EPS (4G) Network */
#define AT_CMD_TEST_EPS_NETWORK      "AT+CEREG=?\r"

/* 45. Query EPS (4G) Network */
#define AT_CMD_QUERY_EPS_NETWORK     "AT+CEREG?\r"

/* 46. Set EPS (4G) Network Presentation Number */
#define AT_CMD_SET_EPS_NETWORK_PRESENTATION  "AT+CEREG=2\r"

/* 47. Test GPRS Service / Packet Domain State */
#define AT_CMD_TEST_PACKET_DOMAIN    "AT+CGATT=?\r"

/* 48. Query GPRS Service / Packet Domain State */
#define AT_CMD_QUERY_PACKET_DOMAIN   "AT+CGATT?\r"

/* 49. Attach to GPRS Service / Packet Domain */
#define AT_CMD_ATTACH_PACKET_DOMAIN  "AT+CGATT=1\r"

/* 50. Detach from GPRS Service / Packet Domain */
#define AT_CMD_DETACH_PACKET_DOMAIN  "AT+CGATT=0\r"


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
#define AT_CMD_HALT_PPP_DAEMON      "ATH\r"

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

/* 26. get SIM card profile ICCID */
#define AT_CMD_ICCID                "AT+QCCID\r"

/* 27. activate PDP context */
#undef AT_CMD_ACTIVATE_PDP_CONTEXT      // unsupported

/* 28. enable BIP */
#define AT_CMD_ENABLE_BIP           "AT+QCFG=\"bip/auth\",1\r"

/* 29. Quectel QCFG band */
#define AT_CMD_SET_QCFG_BAND        "AT+QCFG=\"band\"\r"

/* 30. Quectel QCFG iotopmode */
#define AT_CMD_SET_QCFG_IOTOPMODE   "AT+QCFG=\"iotopmode\"\r"

/* 31. Quectel QCFG nwscanseq */
#define AT_CMD_SET_QCFG_NWSCANSEQ   "AT+QCFG=\"nwscanseq\"\r"

/* 32. Quectel QCFG nwscanmode */
#define AT_CMD_SET_QCFG_NWSCANMODE  "AT+QCFG=\"nwscanmode\"\r"

/* 33. Modem Identity Info I */
#define AT_CMD_IDENT_0              "ATI\r"

/* 34. Modem Identity Info II */
#undef AT_CMD_IDENT_6                   // unsupported

/* 35. Modem Identity Info III */
#undef AT_CMD_IDENT_9                   // unsupported

/* 36. Check if PDP Context command is available */
#define AT_CMD_TEST_PDP_CONTEXT     "AT+CGDCONT=?\r"

/* 37. Enable SIM Toolkit */
#undef AT_CMD_ENABLE_STK                // unsupported

/* 38. Test GSM (2G) Network */
#define AT_CMD_TEST_GSM_NETWORK     "AT+CREG=?\r"

/* 39. Query GSM (2G) Network */
#define AT_CMD_QUERY_GSM_NETWORK     "AT+CREG?\r"

/* 40. Set GSM (2G) Network Presentation Number */
#define AT_CMD_SET_GSM_NETWORK_PRESENTATION "AT+CREG=2\r"

/* 41. Test GPRS (3G) Network */
#define AT_CMD_TEST_GPRS_NETWORK     "AT+CGREG=?\r"

/* 42. Query GPRS (3G) Network */
#define AT_CMD_QUERY_GPRS_NETWORK    "AT+CGREG?\r"

/* 43. Set GPRS (3G) Network Presentation Number */
#define AT_CMD_SET_GPRS_NETWORK_PRESENTATION  "AT+CGREG=2\r"

/* 44. Test EPS (4G) Network */
#define AT_CMD_TEST_EPS_NETWORK      "AT+CEREG=?\r"

/* 45. Query EPS (4G) Network */
#define AT_CMD_QUERY_EPS_NETWORK     "AT+CEREG?\r"

/* 46. Set EPS (4G) Network Presentation Number */
#define AT_CMD_SET_EPS_NETWORK_PRESENTATION  "AT+CEREG=2\r"

/* 47. Test GPRS Service / Packet Domain State */
#define AT_CMD_TEST_PACKET_DOMAIN    "AT+CGATT=?\r"

/* 48. Query GPRS Service / Packet Domain State */
#define AT_CMD_QUERY_PACKET_DOMAIN   "AT+CGATT?\r"

/* 49. Attach to GPRS Service / Packet Domain */
#define AT_CMD_ATTACH_PACKET_DOMAIN  "AT+CGATT=1\r"

/* 50. Detach from GPRS Service / Packet Domain */
#define AT_CMD_DETACH_PACKET_DOMAIN  "AT+CGATT=0\r"

/* ----------------------------------------------------------------------*/
/* Parameters for U-Blox LARA R280, SARA U201 and SARA R412M */
#elif ((ATMODEM_HW == ATMODEM_HW_UBLOX_LARA_R280)   || \
       (ATMODEM_HW == ATMODEM_HW_UBLOX_SARA_U201)   || \
       (ATMODEM_HW == ATMODEM_HW_UBLOX_SARA_R412M))

/* 1. modem maximum baud rate */
#define PPP_MAX_MODEM_BAUD_RATE     115200 //921600 //460800 //115200

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
#define AT_CMD_HALT_PPP_DAEMON      "ATH\r"

/* 12. AT command to power off the modem */
#define AT_CMD_POWER_OFF_MODEM      "AT+CPWROFF\r"

/* 13. AT command to reset the modem */
#define AT_CMD_RESET                "AT+CFUN=16\r"

/* 14. while modem is starting up, the hint that indicates it's ready */
#undef AT_RSP_READY                     // unsupported

/* 15. AT command to query UE System Info */
#if (ATMODEM_HW == ATMODEM_HW_UBLOX_SARA_R412M)
#undef AT_CMD_QUERY_UE_INFO             // unsupported
#else
//#define AT_CMD_QUERY_UE_INFO        "AT+CGED=131\r"   // profile switch will fail
#undef AT_CMD_QUERY_UE_INFO             // unsupported
#endif

/* 16. good pattern to look for in UE System Info response */
#if (ATMODEM_HW == ATMODEM_HW_UBLOX_SARA_R412M)
#undef AT_RSP_UE_INFO_PATTERN_LTE       // unsupported
#else
//#define AT_RSP_UE_INFO_PATTERN_LTE    "LTE"
#undef AT_RSP_UE_INFO_PATTERN_LTE       // unsupported
#endif

/* 17. failure pattern to look for in UE System Info response */
#if (ATMODEM_HW == ATMODEM_HW_UBLOX_SARA_R412M)
#undef AT_RSP_UE_INFO_PATTERN_FAILED    // unsupported
#else
//#define AT_RSP_UE_INFO_PATTERN_FAILED "NO SERVICE,Online"
#undef AT_RSP_UE_INFO_PATTERN_FAILED    // unsupported
#endif

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

/* 26. get SIM card profile ICCID */
#define AT_CMD_ICCID                "AT+CCID\r"

/* 27. activate PDP context */
#if (ATMODEM_HW == ATMODEM_HW_UBLOX_LARA_R280)
#define AT_CMD_ACTIVATE_PDP_CONTEXT "AT+CGACT=1,1\r"
#else
#undef AT_CMD_ACTIVATE_PDP_CONTEXT      // SARA U201 failed to get DNS servers
#endif

/* 28. enable BIP */
#undef AT_CMD_ENABLE_BIP                // unsupported

/* 29. Quectel QCFG band */
#undef AT_CMD_SET_QCFG_BAND             // unsupported

/* 30. Quectel QCFG iotopmode */
#undef AT_CMD_SET_QCFG_IOTOPMODE        // unsupported

/* 31. Quectel QCFG nwscanseq */
#undef AT_CMD_SET_QCFG_NWSCANSEQ        // unsupported

/* 32. Quectel QCFG nwscanmode */
#undef AT_CMD_SET_QCFG_NWSCANMODE       // unsupported

/* 33. Modem Identity Info I */
#define AT_CMD_IDENT_0              "ATI0\r"

/* 34. Modem Identity Info II */
#define AT_CMD_IDENT_6              "ATI6\r"

/* 35. Modem Identity Info III */
#define AT_CMD_IDENT_9              "ATI9\r"

/* 36. Check if PDP Context command is available */
//#undef AT_CMD_TEST_PDP_CONTEXT          // response is too verbose; fail to read OK
#define AT_CMD_TEST_PDP_CONTEXT     "AT+CGDCONT=?\r"

/* 37. Enable SIM Toolkit */
//#define AT_CMD_ENABLE_STK           "AT+CFUN=6\r" // profile switch will fail
#undef AT_CMD_ENABLE_STK                // unsupported

/* 38. Test GSM (2G) Network */
#define AT_CMD_TEST_GSM_NETWORK     "AT+CREG=?\r"

/* 39. Query GSM (2G) Network */
#define AT_CMD_QUERY_GSM_NETWORK     "AT+CREG?\r"

/* 40. Set GSM (2G) Network Presentation Number */
#define AT_CMD_SET_GSM_NETWORK_PRESENTATION "AT+CREG=0\r" // ublox's option 2 is too verbose

/* 41. Test GPRS (3G) Network */
#define AT_CMD_TEST_GPRS_NETWORK     "AT+CGREG=?\r"

/* 42. Query GPRS (3G) Network */
#define AT_CMD_QUERY_GPRS_NETWORK    "AT+CGREG?\r"

/* 43. Set GPRS (3G) Network Presentation Number */
#define AT_CMD_SET_GPRS_NETWORK_PRESENTATION  "AT+CGREG=0\r" // ublox's option 2 is too verbose

/* 44. Test EPS (4G) Network */
#if ((ATMODEM_HW == ATMODEM_HW_UBLOX_LARA_R280) || \
     (ATMODEM_HW == ATMODEM_HW_UBLOX_SARA_R412M))
#define AT_CMD_TEST_EPS_NETWORK      "AT+CEREG=?\r"
#elif (ATMODEM_HW == ATMODEM_HW_UBLOX_SARA_U201)
#undef AT_CMD_TEST_EPS_NETWORK
#endif

/* 45. Query EPS (4G) Network */
#if ((ATMODEM_HW == ATMODEM_HW_UBLOX_LARA_R280) || \
     (ATMODEM_HW == ATMODEM_HW_UBLOX_SARA_R412M))
#define AT_CMD_QUERY_EPS_NETWORK     "AT+CEREG?\r"
#elif (ATMODEM_HW == ATMODEM_HW_UBLOX_SARA_U201)
#undef AT_CMD_TEST_EPS_NETWORK
#endif

/* 46. Set EPS (4G) Network Presentation Number */
#if ((ATMODEM_HW == ATMODEM_HW_UBLOX_LARA_R280) || \
     (ATMODEM_HW == ATMODEM_HW_UBLOX_SARA_R412M))
#define AT_CMD_SET_EPS_NETWORK_PRESENTATION  "AT+CEREG=0\r" // ublox's option 2 is too verbose
#elif (ATMODEM_HW == ATMODEM_HW_UBLOX_SARA_U201)
#undef AT_CMD_TEST_EPS_NETWORK
#endif

/* 47. Test GPRS Service / Packet Domain State */
#define AT_CMD_TEST_PACKET_DOMAIN    "AT+CGATT=?\r"

/* 48. Query GPRS Service / Packet Domain State */
#define AT_CMD_QUERY_PACKET_DOMAIN   "AT+CGATT?\r"

/* 49. Attach to GPRS Service / Packet Domain */
#define AT_CMD_ATTACH_PACKET_DOMAIN  "AT+CGATT=1\r"

/* 50. Detach from GPRS Service / Packet Domain */
#define AT_CMD_DETACH_PACKET_DOMAIN  "AT+CGATT=0\r"

/* ----------------------------------------------------------------------*/
/* Parameters for Cinterion EXS62-W */
#elif (ATMODEM_HW == ATMODEM_HW_CINTERION_EXS62W)

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
#define AT_CMD_HALT_PPP_DAEMON      "ATH\r"

/* 12. AT command to power off the modem */
//#undef AT_CMD_POWER_OFF_MODEM           // unsupported
#define AT_CMD_POWER_OFF_MODEM      "AT^SMSO\r"

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

/* 26. get SIM card profile ICCID */
#define AT_CMD_ICCID                "AT^SCID\r"

/* 27. activate PDP context */
#undef AT_CMD_ACTIVATE_PDP_CONTEXT      // unsupported

/* 28. enable BIP */
#undef AT_CMD_ENABLE_BIP                // unsupported

/* 29. Quectel QCFG band */
#undef AT_CMD_SET_QCFG_BAND             // unsupported

/* 30. Quectel QCFG iotopmode */
#undef AT_CMD_SET_QCFG_IOTOPMODE        // unsupported

/* 31. Quectel QCFG nwscanseq */
#undef AT_CMD_SET_QCFG_NWSCANSEQ        // unsupported

/* 32. Quectel QCFG nwscanmode */
#undef AT_CMD_SET_QCFG_NWSCANMODE       // unsupported

/* 33. Modem Identity Info I */
#define AT_CMD_IDENT_0              "ATI1\r"

/* 34. Modem Identity Info II */
#define AT_CMD_IDENT_6              "ATI8\r"

/* 35. Modem Identity Info III */
#undef AT_CMD_IDENT_9                   // unsupported

/* 36. Check if PDP Context command is available */
#define AT_CMD_TEST_PDP_CONTEXT     "AT+CGDCONT=?\r"

/* 37. Enable SIM Toolkit */
#undef AT_CMD_ENABLE_STK                // unsupported

/* 38. Test GSM (2G) Network */
#define AT_CMD_TEST_GSM_NETWORK     "AT+CREG=?\r"

/* 39. Query GSM (2G) Network */
#define AT_CMD_QUERY_GSM_NETWORK     "AT+CREG?\r"

/* 40. Set GSM (2G) Network Presentation Number */
#define AT_CMD_SET_GSM_NETWORK_PRESENTATION "AT+CREG=2\r"

/* 41. Test GPRS (3G) Network */
#define AT_CMD_TEST_GPRS_NETWORK     "AT+CGREG=?\r"

/* 42. Query GPRS (3G) Network */
#define AT_CMD_QUERY_GPRS_NETWORK    "AT+CGREG?\r"

/* 43. Set GPRS (3G) Network Presentation Number */
#define AT_CMD_SET_GPRS_NETWORK_PRESENTATION  "AT+CGREG=2\r"

/* 44. Test EPS (4G) Network */
#define AT_CMD_TEST_EPS_NETWORK      "AT+CEREG=?\r"

/* 45. Query EPS (4G) Network */
#define AT_CMD_QUERY_EPS_NETWORK     "AT+CEREG?\r"

/* 46. Set EPS (4G) Network Presentation Number */
#define AT_CMD_SET_EPS_NETWORK_PRESENTATION  "AT+CEREG=2\r"

/* 47. Test GPRS Service / Packet Domain State */
#define AT_CMD_TEST_PACKET_DOMAIN    "AT+CGATT=?\r"

/* 48. Query GPRS Service / Packet Domain State */
#define AT_CMD_QUERY_PACKET_DOMAIN   "AT+CGATT?\r"

/* 49. Attach to GPRS Service / Packet Domain */
#define AT_CMD_ATTACH_PACKET_DOMAIN  "AT+CGATT=1\r"

/* 50. Detach from GPRS Service / Packet Domain */
#define AT_CMD_DETACH_PACKET_DOMAIN  "AT+CGATT=0\r"

#endif /* ATMODEM_HW */


#ifdef __cplusplus
}
#endif

#endif /* SOURCE_CY_ATMODEM_H_*/
