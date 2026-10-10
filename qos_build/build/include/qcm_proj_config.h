/* Copyright (C) 2020 UNIRTOS Technologies Limited and/or its affiliates("UNIRTOS").
 * All rights reserved.
 */

#ifndef _QCM_CONPONENTS_CONFIG_H_
#define _QCM_CONPONENTS_CONFIG_H_

/**
 * UniRTOS file api interface function
 */
#define CONFIG_QCM_FILE_API_FUNC

/**
 * UniRTOS socket adapter function
 */
#define CONFIG_QCM_SOCKET_ADP_FUNC

/**
 * UniRTOS Vtls ssl function
 */
/* #undef CONFIG_QCM_VTLS_FUNC */

/**
 * UniRTOS qurl function
 */
#define CONFIG_QCM_QURL_FUNC

/**
 * UniRTOS qurl http function
 */
#define CONFIG_QCM_QURL_HTTP_FUNC

/**
 * UniRTOS qurl http function
 */
/* #undef CONFIG_QCM_QURL_FTP_FUNC */

/**
 * UniRTOS qurl http function
 */
/* #undef CONFIG_QCM_QURL_SMTP_FUNC */

/**
 * support base64 decode and encode
 */
/* #undef CONFIG_QCM_UTILS_BASE64_FUNC */

/**
 * support tlv
 */
/* #undef CONFIG_QCM_UTILS_TLV_FUNC */

/**
 * support check UTF-8 characters
 */
#define CONFIG_QCM_UTILS_UTF8_FUNC

/**
 * support public character process functions
 */
/* #undef CONFIG_QCM_UTILS_UTILS_FUNC */

/**
 * UniRTOS coap function
 */
/* #undef QCM_COAP_FUNC */

/**
 * UniRTOS coap api function
 */
/* #undef QCM_COAP_API_FUNC */

/**
 * UniRTOS OSA_LWM2M function
 */
/* #undef CONFIG_QCM_LWM2M_FUNC */

/**
 * UniRTOS OSA_LWM2M function
 */
/* #undef CONFIG_QCM_LWM2M_FOTA_FUNC */

/**
 * UniRTOS OSA_LWM2M function
 */
/* #undef CONFIG_QCM_LWM2M_DTLS_FUNC */

/**
 * UniRTOS mini http function
 */
/* #undef CONFIG_QCM_MINI_HTTP_FUNC */

/**
 * UniRTOS lbs function
 */
/* #undef CONFIG_QCM_LBS_FUNC */

/**
 * UniRTOS mms function
 */
/* #undef CONFIG_QCM_MMS_FUNC */

/**
 * UniRTOS mqtt function
 */
/* #undef CONFIG_QCM_MQTT_FUNC */

/**
 * UniRTOS mqtt5.0 function
 */
/* #undef CONFIG_QCM_MQTT5_FUNC */

/**
 * UniRTOS file sfs(SFS:) function
 */
/* #undef CONFIG_QCM_FILE_SFS_FUNC */

/**
 * UniRTOS ntp function
 */
/* #undef CONFIG_QCM_NTP_FUNC */

/**
 * UniRTOS sbfota function
 */
/* #undef CONFIG_QCM_SBFOTA_FUNC */

/**
 * UniRTOS supoort websocket func
 */
/* #undef CONFIG_QCM_WEBSOCKET_FUNC */

/**
 * UniRTOS ping function
 */
/* #undef CONFIG_QCM_PING_FUNC */

/**
 * UniRTOS urc function
 */
/* #undef CONFIG_QCM_URC_FUNC */

/**
 * UniRTOS urc function
 */
/* #undef CONFIG_QCM_UART_LOG_FUNC */

/**
 * UniRTOS spi nor function
 */
/* #undef CONFIG_QCM_SPI_NOR_FUNC */

/**
 * UniRTOS dmhttp function
 */
/* #undef CONFIG_QCM_DMHTTP_FUNC */

/**
 * UniRTOS virt at function
 */
/* #undef CONFIG_QCM_VIRT_AT_FUNC */

/**
 * UniRTOS qvsim function
 */
/* #undef CONFIG_QCM_QVSIM_FUNC */

/**
 * UniRTOS audio stream function
 */
/* #undef CONFIG_QCM_AUDIO_STREAM_FUNC */

/**
 * UniRTOS audio MP3 function
 */
/* #undef CONFIG_QCM_AUDIO_MP3_FUNC */

/**
 * UniRTOS audio AMR function
 */
/* #undef CONFIG_QCM_AUDIO_AMR_FUNC */

/**
 * UniRTOS audio ESIM function
 */
/* #undef CONFIG_QCM_ESIM_FUNC */

/**
 * UniRTOS mbedtls function
 */
/* #undef CONFIG_QCM_MBEDTLS_LIBRARY_FUNC */

/**
 * UniRTOS mbedtls function
 */
/* #undef CONFIG_QCM_MBEDTLS_2_LIBRARY_FUNC */

/**
 * UniRTOS mbedtls function
 */
/* #undef CONFIG_QCM_MBEDTLS_4_LIBRARY_FUNC */

/**
 * UniRTOS mbedtls file function
 */
/* #undef CONFIG_QCM_MBEDTLS_CONFIG_FILE */


/**
 * Whether mini http enables TCP SACK function
 */
/* #undef CONFIG_QCM_MINI_HTTP_SACK_FUNC */

/**
 * UniRTOS gnss function
 */
/* #undef CONFIG_QCM_GNSS_FUNC */

/**
 * UniRTOS Utils Xmodem function
 */
/* #undef CONFIG_QCM_UTILS_XMODEM_FUNC */


/******************************************************************************
 *    Business layer task control
 ******************************************************************************/

/**
 * @brief Stack size for lwm2m coap fota task
 * @brief Set the stack size for the lwm2m coap fota task.
 */
#define CONFIG_QCM_LWM2M_COPA_FOTA_STACK_SIZE 4096

/**
 * @brief OSA socket monitor task, mainly handles socket events
 * @brief Stack size for OSA socket monitor task
 */
#define CONFIG_QCM_MONITOR_TASK_STACK_SIZE 2048

/**
 * @brief LWM2M app task stack size
 * @brief Stack size for LWM2M app task
 */
#define CONFIG_QCM_LWM2M_TASK_STACK_SIZE 10240

/**
 * @brief MMS business task
 * @brief Stack size for MMS business task
 */
#define CONFIG_QCM_MMS_TASK_STACK_SIZE 8192

/**
 * @brief MQTT business layer task stack size
 * @brief Stack size for MQTT business layer task
 */
#define CONFIG_QCM_MQTT_TASK_STACK_SIZE 8192

/**
 * @brief NTP TASK stack size
 * @brief Stack size for NTP task
 */
#define CONFIG_QCM_NTP_TASK_SIZE 4096

/**
 * @brief websocket task stack size
 * @brief Stack size for WebSocket client task
 */
#define CONFIG_QCM_WEB_CLIENT_TASK_SIZE 20480

/**
 * @brief qurl conn deal
 * @brief qurl conn deal cache size
 */
/* #undef CONFIG_QCM_CONN_DEAL_CACHE_SIZE */

/**
 * @brief ping thread stack size
 * @brief Stack size for ping task
 */
#define CONFIG_QCM_PING_TASK_STACK_SIZE 4096

/**
 * @brief LBS TASK stack size
 * @brief Stack size for LBS task
 */
#define CONFIG_QCM_LBS_TASK_SIZE 20480

/**
 * @brief SoftBankFota TASK stack size
 * @brief Stack size for SBFOTA task
 */
#define CONFIG_QCM_SBFOTA_TASK_STACK_SIZE 4096

/**
 * Whether MQTT supports proactive network disconnection monitoring
 */
/* #undef CONFIG_QCM_MQTT_USE_NET_DOWN_EVENT */

/**
 * @brief urc ri TASK stack size
 * @brief Stack size for urc ri task
 */
#define CONFIG_QCM_URC_RI_TASK_STACK_SIZE 4096

#endif /* _QCM_CONPONENTS_CONFIG_H_ */
