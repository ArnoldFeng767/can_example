/*****************************************************************/ /**
* @file can_demo.c
* @brief CAN interface usage example (based on qosa_can_eigen.h)
* @date 2026-09-29
*
* Demonstrates:
*  1. Pin mux configuration (qosa_pin_set_func)
*  2. Get CAN controller capabilities (qosa_can_get_capabilities)
*  3. Init / deinit (qosa_can_init / qosa_can_uninit)
*  4. Power control (qosa_can_set_power)
*  5. Set bitrate (qosa_can_set_bitrate)
*  6. Set work mode (qosa_can_set_mode)
*  7. Get object capabilities (qosa_can_get_obj_capabilities)
*  8. Set RX filter (qosa_can_set_obj_filter)
*  9. Configure TX/RX objects (qosa_can_set_obj_config)
* 10. Send data (qosa_can_write)
* 11. Receive data (qosa_can_read / qosa_can_get_rx_message_count)
* 12. Read bus status (qosa_can_get_status)
**********************************************************************/

#include "include.h"

//QOSA core definition header
#include "qosa_def.h"
//Include QOSA system API header
#include "qosa_sys.h"
//Include QOSA log system header file
#include "qosa_log.h"
//Include QOSA CAN driver header file
#include "qosa_can_eigen.h"
//Include QOSA device header file (CPU usage API)
#include "qosa_dev.h"
//Include QOSA device eigen header (AON GPIO power / voltage for CAN transceiver)
#include "qosa_dev_eigen.h"
//Include QOSA pinctrl header file for pin function config
#include "qosa_pinctrl.h"
//Standard memory header
#include <string.h>

//Define log information
#define QOS_LOG_TAG   LOG_TAG_DEMO

/*===========================================================================
 * Constant Definitions
 *==========================================================================*/

// CAN bit rate (kHz), options: 1000 / 500 / 250 / 125 / 100
#define CAN_BIT_RATE_KHZ    (500)

// CAN work mode is selected automatically by controller capability (see can_init):
//   internal_loopback supported -> QOSA_CAN_MODE_LOOPBACK_INTERNAL (internal loopback, self TX/RX, not visible on bus)
//   external_loopback supported -> QOSA_CAN_MODE_LOOPBACK_EXTERNAL (external loopback, visible on bus)
//   neither supported            -> QOSA_CAN_MODE_NORMAL (normal TX/RX, needs bus peer ACK)

// CAN pin configuration (adjust per actual hardware schematic)
#define CAN_TX_PIN_NUM      QOSA_PIN_63
#define CAN_TX_PIN_FUNC     7
#define CAN_RX_PIN_NUM      QOSA_PIN_64
#define CAN_RX_PIN_FUNC     7
#define CAN_STB_PIN_NUM     QOSA_PIN_62
#define CAN_STB_PIN_FUNC    7

// CAN identifier format (bit31 is the IDE extended frame flag)
#define ARM_CAN_ID_IDE_Msk       (1UL << 31)
#define ARM_CAN_STANDARD_ID(id)  ((id) & 0x000007FFUL)
#define ARM_CAN_EXTENDED_ID(id)  (((id) & 0x1FFFFFFFUL) | ARM_CAN_ID_IDE_Msk)

// Bit timing segment encoding, used by qosa_can_bitrate_t.bit_segments
#define ARM_CAN_BIT_PROP_SEG(x)    (((x) & 0xFF) << 0)     // propagation segment
#define ARM_CAN_BIT_PHASE_SEG1(x)  (((x) & 0xFF) << 8)     // phase buffer segment 1
#define ARM_CAN_BIT_PHASE_SEG2(x)  (((x) & 0xFF) << 16)    // phase buffer segment 2
#define ARM_CAN_BIT_SJW(x)         (((x) & 0x1F) << 24)    // sync jump width

// CAN event codes (callback function event parameter)
#define ARM_CAN_EVENT_SEND_COMPLETE    (1UL << 0)  // send complete
#define ARM_CAN_EVENT_RECEIVE          (1UL << 1)  // frame received
#define ARM_CAN_EVENT_RECEIVE_OVERRUN  (1UL << 2)  // receive overrun
#define ARM_CAN_EVENT_UNIT_BUS_OFF     (4U)        // bus off

// Driver return code
#define ARM_DRIVER_OK    0

// CAN bus control command codes (qosa_can_control_t.control)
#define ARM_CAN_CONTROL_Pos              0UL
#define ARM_CAN_RECOVER_FROM_BUS_OFF    (253UL << ARM_CAN_CONTROL_Pos)  // recover from bus-off state
#define ARM_CAN_SET_TRANSCEIVER_STANDBY (254UL << ARM_CAN_CONTROL_Pos)  // transceiver standby control: arg 0=wake 1=standby

// CAN unit state codes (qosa_can_status_t.unit_state)
#define ARM_CAN_UNIT_STATE_BUS_OFF      (3U)                            // bus off

// User-defined event flag bits
#define CAN_EVT_UNIT       (1UL << 0)
#define CAN_EVT_RX         (1UL << 1)
#define CAN_EVT_TX_DONE    (1UL << 2)

// Task stack size and priority
#define CAN_DEMO_TASK_STACK_SIZE    (4096)
#define CAN_DEMO_TASK_PRIO          QOSA_PRIORITY_NORMAL

// TX period (ms)
#define CAN_TX_PERIOD_MS            (1000)

/*===========================================================================
 * Global Variables
 *==========================================================================*/

static qosa_task_t g_can_demo_task = QOSA_NULL;   // task handle
static qosa_flag_t g_can_evt      = QOSA_NULL;    // event flag group

static qosa_uint32_t g_rx_obj_idx = 0xFFFFFFFFU;  // RX object index
static qosa_uint32_t g_tx_obj_idx = 0xFFFFFFFFU;  // TX object index

static qosa_can_msg_info_t g_tx_msg_info;         // TX message info
static volatile qosa_uint32_t g_tx_cnt = 0;       // TX counter

static qosa_can_mode_e g_can_work_mode = QOSA_CAN_MODE_NORMAL;  // actually selected work mode (auto-selected by capability)

/*===========================================================================
 * Callback Functions
 *==========================================================================*/

/**
 * @brief CAN unit event callback (called by driver in interrupt/event context)
 * @param event event code, e.g. ARM_CAN_EVENT_UNIT_BUS_OFF
 */
static void can_unit_event_cb(qosa_uint32_t event)
{
    // Note: callback may run in interrupt context, keep it lightweight (flag set only)
    if (g_can_evt != QOSA_NULL)
    {
        qosa_flag_set(g_can_evt, CAN_EVT_UNIT);
    }

    if (event == ARM_CAN_EVENT_UNIT_BUS_OFF)
    {
        QLOGW("CAN unit bus off");
    }
}

/**
 * @brief CAN object event callback (called by driver in interrupt/event context)
 * @param obj_idx object index
 * @param event   event code, e.g. ARM_CAN_EVENT_RECEIVE / ARM_CAN_EVENT_SEND_COMPLETE
 */
static void can_object_event_cb(qosa_uint32_t obj_idx, qosa_uint32_t event)
{
    if (g_can_evt == QOSA_NULL)
    {
        return;
    }

    if ((obj_idx == g_rx_obj_idx) && (event == ARM_CAN_EVENT_RECEIVE))
    {
        qosa_flag_set(g_can_evt, CAN_EVT_RX);
    }
    else if ((obj_idx == g_tx_obj_idx) && (event == ARM_CAN_EVENT_SEND_COMPLETE))
    {
        qosa_flag_set(g_can_evt, CAN_EVT_TX_DONE);
    }
}

/*===========================================================================
 * Internal Functions
 *==========================================================================*/

/**
 * @brief Check the return value, print an error log on failure
 * @param ret  function return value
 * @param step current step description
 * @return 1 on success, 0 on failure
 */
static qosa_int32_t can_ret_check(qosa_int32_t ret, const char *step)
{
    if (ret == ARM_DRIVER_OK)
    {
        QLOGI("%s ok", step);
        return 1;
    }

    QLOGE("%s failed, ret = %d", step, ret);
    return 0;
}

/**
 * @brief Configure the CAN transceiver pins
 * @return 1 on success, 0 on failure
 */
static qosa_int32_t can_pin_config(void)
{
    if (qosa_pin_set_func(CAN_TX_PIN_NUM, CAN_TX_PIN_FUNC) != QOSA_PINCTRL_SUCCESS)
    {
        QLOGE("CAN TX pin config failed");
        return 0;
    }

    if (qosa_pin_set_func(CAN_RX_PIN_NUM, CAN_RX_PIN_FUNC) != QOSA_PINCTRL_SUCCESS)
    {
        QLOGE("CAN RX pin config failed");
        return 0;
    }

    if (qosa_pin_set_func(CAN_STB_PIN_NUM, CAN_STB_PIN_FUNC) != QOSA_PINCTRL_SUCCESS)
    {
        QLOGE("CAN STB pin config failed");
        return 0;
    }

    QLOGI("CAN pin config ok");
    return 1;
}

/**
 * @brief Initialize the CAN controller
 * @return 1 on success, 0 on failure
 */
static qosa_int32_t can_init(void)
{
    qosa_int32_t ret;
    qosa_uint32_t num_objects;
    qosa_can_capabilities_t can_cap;
    qosa_can_obj_capabilities_t can_obj_cap;
    qosa_can_bitrate_t can_bitrate;
    qosa_can_obj_filter_t can_obj_filter;
    qosa_can_obj_config_t can_obj_config;
    qosa_can_control_t can_control;

    // 0. Power the CAN transceiver and set the IO voltage. Without this the
    //    transceiver does not work, the bus has no ACK, and the controller TX
    //    error count rises to 128 and enters bus-off (seen as no loopback RX)
    QLOGI("can transceiver power on (aon gpio + 3.3V)");
    qosa_aon_gpio_power_control(POWER_ON);
    qosa_gpio_set_voltage(VOL_3_30V);

    // 1. Get controller capabilities to obtain the number of available objects
    memset(&can_cap, 0, sizeof(can_cap));
    ret = qosa_can_get_capabilities(QOSA_CAN_DEV_NUM0, &can_cap);
    if (!can_ret_check(ret, "get capabilities"))
    {
        return 0;
    }
    num_objects = can_cap.num_objects;

    // Print controller capability flags to help diagnose loopback support
    QLOGI("can caps: num_obj=%u, internal_lb=%u, external_lb=%u, monitor=%u, restricted=%u",
          num_objects,
          (unsigned)can_cap.internal_loopback,
          (unsigned)can_cap.external_loopback,
          (unsigned)can_cap.monitor_mode,
          (unsigned)can_cap.restricted_mode);

    // Auto-select work mode by capability: internal loopback > external loopback > normal
    if (can_cap.internal_loopback)
    {
        g_can_work_mode = QOSA_CAN_MODE_LOOPBACK_INTERNAL;
    }
    else if (can_cap.external_loopback)
    {
        g_can_work_mode = QOSA_CAN_MODE_LOOPBACK_EXTERNAL;
    }
    else
    {
        g_can_work_mode = QOSA_CAN_MODE_NORMAL;
        QLOGW("loopback not supported, fall back to normal mode");
    }

    // 2. Initialize CAN and register the event callbacks
    ret = qosa_can_init(QOSA_CAN_DEV_NUM0, can_unit_event_cb, can_object_event_cb);
    if (!can_ret_check(ret, "can init"))
    {
        return 0;
    }

    // 3. Power on
    ret = qosa_can_set_power(QOSA_CAN_DEV_NUM0, QOSA_CAN_POWER_FULL);
    if (!can_ret_check(ret, "power on"))
    {
        return 0;
    }

    // 4. Enter initialization mode (required before configuring parameters)
    ret = qosa_can_set_mode(QOSA_CAN_DEV_NUM0, QOSA_CAN_MODE_INITIALIZATION);
    if (!can_ret_check(ret, "enter init mode"))
    {
        return 0;
    }

    // 5. Set the bitrate
    can_bitrate.select       = QOSA_CAN_BITRATE_NOMINAL;
    can_bitrate.bitrate      = CAN_BIT_RATE_KHZ * 1000;
    can_bitrate.bit_segments = ARM_CAN_BIT_PROP_SEG(5U) |
                               ARM_CAN_BIT_PHASE_SEG1(4U) |
                               ARM_CAN_BIT_PHASE_SEG2(3U) |
                               ARM_CAN_BIT_SJW(1U);
    ret = qosa_can_set_bitrate(QOSA_CAN_DEV_NUM0, &can_bitrate);
    if (!can_ret_check(ret, "set bitrate"))
    {
        return 0;
    }

    // 6. Iterate objects to find indexes that support TX and RX
    g_rx_obj_idx = 0xFFFFFFFFU;
    g_tx_obj_idx = 0xFFFFFFFFU;

    for (qosa_uint32_t i = 0; i < num_objects; i++)
    {
        memset(&can_obj_cap, 0, sizeof(can_obj_cap));
        can_obj_cap.obj_idx = i;
        qosa_can_get_obj_capabilities(QOSA_CAN_DEV_NUM0, &can_obj_cap);

        if ((g_rx_obj_idx == 0xFFFFFFFFU) && (can_obj_cap.rx == 1U))
        {
            g_rx_obj_idx = i;
        }
        else if ((g_tx_obj_idx == 0xFFFFFFFFU) && (can_obj_cap.tx == 1U))
        {
            g_tx_obj_idx = i;
            break;
        }
    }

    if ((g_rx_obj_idx == 0xFFFFFFFFU) || (g_tx_obj_idx == 0xFFFFFFFFU))
    {
        QLOGE("can not find valid rx/tx object");
        return 0;
    }
    QLOGI("rx obj idx = %u, tx obj idx = %u", g_rx_obj_idx, g_tx_obj_idx);

    // 7. Set the RX object filter (receive all extended frames)
    memset(&can_obj_filter, 0, sizeof(can_obj_filter));
    can_obj_filter.obj_idx   = g_rx_obj_idx;
    can_obj_filter.operation = QOSA_CAN_FILTER_ID_MASKABLE_ADD;
    can_obj_filter.id        = ARM_CAN_EXTENDED_ID(0);
    can_obj_filter.arg       = 0;
    ret = qosa_can_set_obj_filter(QOSA_CAN_DEV_NUM0, &can_obj_filter);
    if (!can_ret_check(ret, "set obj filter"))
    {
        return 0;
    }

    // 8. Configure the TX object
    memset(&can_obj_config, 0, sizeof(can_obj_config));
    can_obj_config.obj_idx   = g_tx_obj_idx;
    can_obj_config.configure = QOSA_CAN_OBJ_TX;
    ret = qosa_can_set_obj_config(QOSA_CAN_DEV_NUM0, &can_obj_config);
    if (!can_ret_check(ret, "config tx obj"))
    {
        return 0;
    }

    // 9. Configure the RX object
    can_obj_config.obj_idx   = g_rx_obj_idx;
    can_obj_config.configure = QOSA_CAN_OBJ_RX;
    ret = qosa_can_set_obj_config(QOSA_CAN_DEV_NUM0, &can_obj_config);
    if (!can_ret_check(ret, "config rx obj"))
    {
        return 0;
    }

    // 10. Prepare the TX message
    memset(&g_tx_msg_info, 0, sizeof(g_tx_msg_info));
    g_tx_msg_info.id  = ARM_CAN_EXTENDED_ID(0x123);
    g_tx_msg_info.dlc = 8;

    // 11. Enter work mode (loopback or normal, auto-selected by controller capability)
    ret = qosa_can_set_mode(QOSA_CAN_DEV_NUM0, g_can_work_mode);
    if (!can_ret_check(ret, "enter work mode"))
    {
        return 0;
    }

    // 12. Wake the transceiver (exit standby, pull STB low). While in standby the
    //     bus is not driven, so TX gets no ACK and quickly enters bus-off, which
    //     makes loopback receive no frames
    can_control.control = ARM_CAN_SET_TRANSCEIVER_STANDBY;
    can_control.arg = 0;
    ret = qosa_can_control(QOSA_CAN_DEV_NUM0, &can_control);
    if (!can_ret_check(ret, "transceiver wake"))
    {
        return 0;
    }

    return 1;
}

/**
 * @brief Send one CAN frame
 */
static void can_tx_once(void)
{
    qosa_uint8_t tx_data[8];
    qosa_int32_t ret;

    // Build the 8-byte payload
    for (qosa_uint32_t i = 0; i < 8; i++)
    {
        tx_data[i] = (qosa_uint8_t)((g_tx_cnt + i) & 0xFF);
    }

    ret = qosa_can_write(QOSA_CAN_DEV_NUM0, g_tx_obj_idx, &g_tx_msg_info, tx_data, 8);
    if (ret == (qosa_int32_t)8)
    {
        g_tx_cnt++;
        QLOGI("tx frame ok, id = 0x%x, cnt = %u", g_tx_msg_info.id & 0x1FFFFFFF, g_tx_cnt);
    }
    else
    {
        QLOGE("tx frame failed, ret = %d", ret);
    }
}

/**
 * @brief Read and print all received CAN frames
 */
static void can_rx_poll(void)
{
    qosa_int32_t rx_cnt;
    qosa_can_msg_info_t rx_msg_info;
    qosa_uint8_t rx_data[8];

    // Query the number of frames in the RX queue
    rx_cnt = qosa_can_get_rx_message_count(QOSA_CAN_DEV_NUM0, g_rx_obj_idx);
    if (rx_cnt <= 0)
    {
        return;
    }

    for (qosa_int32_t i = 0; i < rx_cnt; i++)
    {
        memset(&rx_msg_info, 0, sizeof(rx_msg_info));
        memset(rx_data, 0, sizeof(rx_data));

        qosa_int32_t len = qosa_can_read(QOSA_CAN_DEV_NUM0, g_rx_obj_idx, &rx_msg_info, rx_data, 8);
        if (len <= 0)
        {
            break;
        }

        QLOGI("rx frame, id = 0x%x, ext = %d, dlc = %d",
              rx_msg_info.id & 0x1FFFFFFF, !!(rx_msg_info.id & ARM_CAN_ID_IDE_Msk), len);

        for (qosa_int32_t j = 0; j < len; j++)
        {
            QLOGI("  data[%d] = 0x%02x", j, rx_data[j]);
        }
    }
}

/**
 * @brief Read and print the bus status
 */
static void can_show_status(void)
{
    qosa_can_status_t status;
    qosa_int32_t ret;

    memset(&status, 0, sizeof(status));
    ret = qosa_can_get_status(QOSA_CAN_DEV_NUM0, &status);
    if (ret != ARM_DRIVER_OK)
    {
        QLOGE("get status failed, ret = %d", ret);
        return;
    }

    QLOGI("unit_state = %u, last_err = %u, tx_err_cnt = %u, rx_err_cnt = %u",
          status.unit_state, status.last_error_code,
          status.tx_error_count, status.rx_error_count);

    // If already bus-off, request recovery so the controller rejoins the bus
    if (status.unit_state == ARM_CAN_UNIT_STATE_BUS_OFF)
    {
        qosa_can_control_t can_control;
        can_control.control = ARM_CAN_RECOVER_FROM_BUS_OFF;
        can_control.arg = 0;
        qosa_int32_t rret = qosa_can_control(QOSA_CAN_DEV_NUM0, &can_control);
        QLOGW("bus off detected, recover ret = %d", rret);
    }
}

/*===========================================================================
 * CPU Usage / Stack Usage
 *==========================================================================*/

// CPU usage monitoring period (ms), valid range 200~60000
#define CAN_CPU_USAGE_PERIOD_MS     (2000)

// Stack free-space report interval (in task loop iterations, report roughly every N loops)
#define CAN_STACK_REPORT_CYCLE      (5)

/**
 * @brief CPU usage callback (called periodically by the qosa_cpu_usage module in worker task context)
 * @param cpu_usage_pct CPU usage percentage of the last period (0-100)
 */
static void can_cpu_usage_cb(qosa_uint8_t cpu_usage_pct)
{
    QLOGI("cpu usage: %u%%", (unsigned)cpu_usage_pct);
}

/**
 * @brief Query and print the remaining stack space of the CAN demo task
 */
static void can_show_stack_usage(void)
{
    qosa_int32_t free_stack = 0;

    if (g_can_demo_task == QOSA_NULL)
    {
        return;
    }

    if (qosa_task_get_stack_space(g_can_demo_task, &free_stack) == QOSA_ERROR_OK)
    {
        QLOGI("stack usage: free = %d bytes (total = %d)",
              (int)free_stack, CAN_DEMO_TASK_STACK_SIZE);
    }
    else
    {
        QLOGE("get stack space failed");
    }
}

/*===========================================================================
 * Task and Init Entry
 *==========================================================================*/

/**
 * @brief CAN demo task main function
 */
static void can_demo_process(void *ctx)
{
    QLOGI("enter can demo task !!!");

    // Configure pins
    if (!can_pin_config())
    {
        return;
    }

    // Initialize CAN
    if (!can_init())
    {
        return;
    }

    // Start periodic CPU usage monitoring (callback prints the percentage in worker context)
    qosa_int32_t cpu_ret = qosa_cpu_usage_start(CAN_CPU_USAGE_PERIOD_MS, can_cpu_usage_cb);
    if (cpu_ret != QOSA_CPU_USAGE_OK)
    {
        QLOGE("cpu usage start failed, ret = %d", cpu_ret);
    }

    // Send one frame immediately
    can_tx_once();

    // Stack report counter
    qosa_uint32_t stack_cycle_cnt = 0;

    while (1)
    {
        // Send one frame periodically: the rhythm is driven by the fixed delay below,
        // not by whether the event flag times out, to avoid the TX rhythm getting out of
        // control or stopping entirely when bus error / loopback self-RX events keep firing
        can_tx_once();

        // Wait one TX period (one frame every 1 second)
        qosa_task_sleep_ms(CAN_TX_PERIOD_MS);

        // Poll the RX queue (in loopback mode the frame just sent loops back into RX)
        can_rx_poll();

        // Periodically report the task's remaining stack space and bus status
        stack_cycle_cnt++;
        if (stack_cycle_cnt >= CAN_STACK_REPORT_CYCLE)
        {
            stack_cycle_cnt = 0;
            can_show_stack_usage();
            can_show_status();
        }
    }
}

/**
 * @brief CAN demo initialization function
 */
void unir_can_demo_init(void)
{
    QLOGI("enter can demo !!!");

    // Create the event flag group
    if (qosa_flag_create(&g_can_evt) != 0)
    {
        QLOGE("create flag failed");
        return;
    }

    // Create the task
    if (g_can_demo_task == QOSA_NULL)
    {
        qosa_task_create(
            &g_can_demo_task,               // task handle pointer
            CAN_DEMO_TASK_STACK_SIZE,       // task stack size
            CAN_DEMO_TASK_PRIO,             // task priority
            "unir_can_demo",                // task name
            can_demo_process,               // task handler function
            QOSA_NULL);                     // task parameter
    }
}

// Not registered in the application init list separately; it is called by
// unir_hello_world_init() in main.c
