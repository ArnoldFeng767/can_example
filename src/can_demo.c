/*****************************************************************/ /**
* @file can_demo.c
* @brief CAN 接口使用示例 (based on qosa_can_eigen.h)
* @date 2026-09-29
*
* 演示内容：
*  1. 引脚复用配置 (qosa_pin_set_func)
*  2. 获取 CAN 控制器能力 (qosa_can_get_capabilities)
*  3. 初始化/去初始化 (qosa_can_init / qosa_can_uninit)
*  4. 电源控制 (qosa_can_set_power)
*  5. 设置波特率 (qosa_can_set_bitrate)
*  6. 设置工作模式 (qosa_can_set_mode)
*  7. 获取对象能力 (qosa_can_get_obj_capabilities)
*  8. 设置接收过滤器 (qosa_can_set_obj_filter)
*  9. 配置收发对象 (qosa_can_set_obj_config)
* 10. 发送数据 (qosa_can_write)
* 11. 接收数据 (qosa_can_read / qosa_can_get_rx_message_count)
* 12. 读取总线状态 (qosa_can_get_status)
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
 * 常量定义
 *==========================================================================*/

// CAN 波特率 (kHz)，可选: 1000 / 500 / 250 / 125 / 100
#define CAN_BIT_RATE_KHZ    (500)

// CAN 工作模式按控制器能力自动选择（见 can_init）：
//   internal_loopback 支持 → QOSA_CAN_MODE_LOOPBACK_INTERNAL（内部回环，自发自收，总线不可见）
//   external_loopback 支持 → QOSA_CAN_MODE_LOOPBACK_EXTERNAL（外部回环，总线可见）
//   都不支持              → QOSA_CAN_MODE_NORMAL（正常收发，需总线对端 ACK）

// CAN 引脚配置 (需根据实际硬件原理图调整)
#define CAN_TX_PIN_NUM      QOSA_PIN_63
#define CAN_TX_PIN_FUNC     7
#define CAN_RX_PIN_NUM      QOSA_PIN_64
#define CAN_RX_PIN_FUNC     7
#define CAN_STB_PIN_NUM     QOSA_PIN_62
#define CAN_STB_PIN_FUNC    7

// CAN 标识符格式 (bit31 为 IDE 扩展帧标志位)
#define ARM_CAN_ID_IDE_Msk       (1UL << 31)
#define ARM_CAN_STANDARD_ID(id)  ((id) & 0x000007FFUL)
#define ARM_CAN_EXTENDED_ID(id)  (((id) & 0x1FFFFFFFUL) | ARM_CAN_ID_IDE_Msk)

// 位时序分段编码，用于 qosa_can_bitrate_t.bit_segments
#define ARM_CAN_BIT_PROP_SEG(x)    (((x) & 0xFF) << 0)     // 传播段
#define ARM_CAN_BIT_PHASE_SEG1(x)  (((x) & 0xFF) << 8)     // 相位缓冲段1
#define ARM_CAN_BIT_PHASE_SEG2(x)  (((x) & 0xFF) << 16)    // 相位缓冲段2
#define ARM_CAN_BIT_SJW(x)         (((x) & 0x1F) << 24)    // 同步跳转宽度

// CAN 事件码 (回调函数 event 参数)
#define ARM_CAN_EVENT_SEND_COMPLETE    (1UL << 0)  // 发送完成
#define ARM_CAN_EVENT_RECEIVE          (1UL << 1)  // 收到报文
#define ARM_CAN_EVENT_RECEIVE_OVERRUN  (1UL << 2)  // 接收溢出
#define ARM_CAN_EVENT_UNIT_BUS_OFF     (4U)        // 总线关闭

// 驱动返回码
#define ARM_DRIVER_OK    0

// CAN 总线控制命令码 (qosa_can_control_t.control)
#define ARM_CAN_CONTROL_Pos              0UL
#define ARM_CAN_RECOVER_FROM_BUS_OFF    (253UL << ARM_CAN_CONTROL_Pos)  // 从总线关闭状态恢复
#define ARM_CAN_SET_TRANSCEIVER_STANDBY (254UL << ARM_CAN_CONTROL_Pos)  // 收发器待机控制: arg 0=唤醒 1=待机

// CAN 单元状态码 (qosa_can_status_t.unit_state)
#define ARM_CAN_UNIT_STATE_BUS_OFF      (3U)                            // 总线关闭

// 用户自定义事件标志位
#define CAN_EVT_UNIT       (1UL << 0)
#define CAN_EVT_RX         (1UL << 1)
#define CAN_EVT_TX_DONE    (1UL << 2)

// 任务栈大小与优先级
#define CAN_DEMO_TASK_STACK_SIZE    (4096)
#define CAN_DEMO_TASK_PRIO          QOSA_PRIORITY_NORMAL

// 发送周期 (ms)
#define CAN_TX_PERIOD_MS            (1000)

/*===========================================================================
 * 全局变量
 *==========================================================================*/

static qosa_task_t g_can_demo_task = QOSA_NULL;   // 任务句柄
static qosa_flag_t g_can_evt      = QOSA_NULL;    // 事件标志组

static qosa_uint32_t g_rx_obj_idx = 0xFFFFFFFFU;  // 接收对象索引
static qosa_uint32_t g_tx_obj_idx = 0xFFFFFFFFU;  // 发送对象索引

static qosa_can_msg_info_t g_tx_msg_info;         // 发送报文信息
static volatile qosa_uint32_t g_tx_cnt = 0;       // 发送计数

static qosa_can_mode_e g_can_work_mode = QOSA_CAN_MODE_NORMAL;  // 实际选择的工作模式（按能力自动选择）

/*===========================================================================
 * 回调函数
 *==========================================================================*/

/**
 * @brief CAN 单元事件回调 (由驱动在中断/事件上下文中调用)
 * @param event 事件码，如 ARM_CAN_EVENT_UNIT_BUS_OFF
 */
static void can_unit_event_cb(qosa_uint32_t event)
{
    // 注意：回调可能处于中断上下文，仅做标志置位等轻量操作
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
 * @brief CAN 对象事件回调 (由驱动在中断/事件上下文中调用)
 * @param obj_idx 对象索引
 * @param event   事件码，如 ARM_CAN_EVENT_RECEIVE / ARM_CAN_EVENT_SEND_COMPLETE
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
 * 内部函数
 *==========================================================================*/

/**
 * @brief 校验返回结果，失败则打印错误日志
 * @param ret  函数返回值
 * @param step 当前步骤描述
 * @return 成功返回 1，失败返回 0
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
 * @brief 配置 CAN 收发引脚
 * @return 成功返回 1，失败返回 0
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
 * @brief 初始化 CAN 控制器
 * @return 成功返回 1，失败返回 0
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

    // 0. 给 CAN 收发器供电并设置 IO 电压，否则收发器不工作、总线无 ACK，
    //    控制器发送错误计数会涨到 128 进入 bus-off（表现为无回环 RX）
    QLOGI("can transceiver power on (aon gpio + 3.3V)");
    qosa_aon_gpio_power_control(POWER_ON);
    qosa_gpio_set_voltage(VOL_3_30V);

    // 1. 获取控制器能力，得到可用对象个数
    memset(&can_cap, 0, sizeof(can_cap));
    ret = qosa_can_get_capabilities(QOSA_CAN_DEV_NUM0, &can_cap);
    if (!can_ret_check(ret, "get capabilities"))
    {
        return 0;
    }
    num_objects = can_cap.num_objects;

    // 打印控制器能力标志，便于诊断回环支持情况
    QLOGI("can caps: num_obj=%u, internal_lb=%u, external_lb=%u, monitor=%u, restricted=%u",
          num_objects,
          (unsigned)can_cap.internal_loopback,
          (unsigned)can_cap.external_loopback,
          (unsigned)can_cap.monitor_mode,
          (unsigned)can_cap.restricted_mode);

    // 按能力自动选择工作模式：内部回环 > 外部回环 > 正常
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

    // 2. 初始化 CAN，注册事件回调
    ret = qosa_can_init(QOSA_CAN_DEV_NUM0, can_unit_event_cb, can_object_event_cb);
    if (!can_ret_check(ret, "can init"))
    {
        return 0;
    }

    // 3. 上电
    ret = qosa_can_set_power(QOSA_CAN_DEV_NUM0, QOSA_CAN_POWER_FULL);
    if (!can_ret_check(ret, "power on"))
    {
        return 0;
    }

    // 4. 进入初始化模式 (配置参数前需要)
    ret = qosa_can_set_mode(QOSA_CAN_DEV_NUM0, QOSA_CAN_MODE_INITIALIZATION);
    if (!can_ret_check(ret, "enter init mode"))
    {
        return 0;
    }

    // 5. 设置波特率
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

    // 6. 遍历对象，找到支持发送和接收的对象索引
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

    // 7. 设置接收对象过滤器 (接收所有扩展帧)
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

    // 8. 配置发送对象
    memset(&can_obj_config, 0, sizeof(can_obj_config));
    can_obj_config.obj_idx   = g_tx_obj_idx;
    can_obj_config.configure = QOSA_CAN_OBJ_TX;
    ret = qosa_can_set_obj_config(QOSA_CAN_DEV_NUM0, &can_obj_config);
    if (!can_ret_check(ret, "config tx obj"))
    {
        return 0;
    }

    // 9. 配置接收对象
    can_obj_config.obj_idx   = g_rx_obj_idx;
    can_obj_config.configure = QOSA_CAN_OBJ_RX;
    ret = qosa_can_set_obj_config(QOSA_CAN_DEV_NUM0, &can_obj_config);
    if (!can_ret_check(ret, "config rx obj"))
    {
        return 0;
    }

    // 10. 准备发送报文
    memset(&g_tx_msg_info, 0, sizeof(g_tx_msg_info));
    g_tx_msg_info.id  = ARM_CAN_EXTENDED_ID(0x123);
    g_tx_msg_info.dlc = 8;

    // 11. 进入工作模式（按控制器能力自动选择：回环或正常）
    ret = qosa_can_set_mode(QOSA_CAN_DEV_NUM0, g_can_work_mode);
    if (!can_ret_check(ret, "enter work mode"))
    {
        return 0;
    }

    // 12. 唤醒收发器（退出待机，STB 拉低）。缺省处于待机时总线无驱动，
    //     发送得不到 ACK 会迅速进入 bus-off，导致回环收不到帧
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
 * @brief 发送一帧 CAN 报文
 */
static void can_tx_once(void)
{
    qosa_uint8_t tx_data[8];
    qosa_int32_t ret;

    // 组装 8 字节数据
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
 * @brief 读取并打印所有已接收的 CAN 报文
 */
static void can_rx_poll(void)
{
    qosa_int32_t rx_cnt;
    qosa_can_msg_info_t rx_msg_info;
    qosa_uint8_t rx_data[8];

    // 查询接收队列中的报文数量
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
 * @brief 读取并打印总线状态
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

    // 若已进入总线关闭，主动请求恢复，让控制器重新参与总线收发
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
 * CPU 占用率 / 堆栈使用情况
 *==========================================================================*/

// CPU 占用率监测周期 (ms)，取值范围 200~60000
#define CAN_CPU_USAGE_PERIOD_MS     (2000)

// 堆栈剩余空间上报间隔（以任务循环次数计，约 N 次循环上报一次）
#define CAN_STACK_REPORT_CYCLE      (5)

/**
 * @brief CPU 占用率回调（由 qosa_cpu_usage 模块在 worker 任务上下文周期调用）
 * @param cpu_usage_pct 最近一个周期的 CPU 占用百分比 (0-100)
 */
static void can_cpu_usage_cb(qosa_uint8_t cpu_usage_pct)
{
    QLOGI("cpu usage: %u%%", (unsigned)cpu_usage_pct);
}

/**
 * @brief 查询并打印 CAN 示例任务的剩余栈空间
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
 * 任务与初始化入口
 *==========================================================================*/

/**
 * @brief CAN 示例任务主函数
 */
static void can_demo_process(void *ctx)
{
    QLOGI("enter can demo task !!!");

    // 配置引脚
    if (!can_pin_config())
    {
        return;
    }

    // 初始化 CAN
    if (!can_init())
    {
        return;
    }

    // 启动 CPU 占用率周期监测（回调在 worker 上下文打印百分比）
    qosa_int32_t cpu_ret = qosa_cpu_usage_start(CAN_CPU_USAGE_PERIOD_MS, can_cpu_usage_cb);
    if (cpu_ret != QOSA_CPU_USAGE_OK)
    {
        QLOGE("cpu usage start failed, ret = %d", cpu_ret);
    }

    // 首次立即发送一帧
    can_tx_once();

    // 堆栈上报计数
    qosa_uint32_t stack_cycle_cnt = 0;

    while (1)
    {
        // 周期发送一帧：节奏由下面的固定延时决定，不依赖事件标志是否超时，
        // 避免总线错误事件/回环自收事件持续触发导致发送节奏失控或彻底停发
        can_tx_once();

        // 等待一个发送周期（每 1 秒一帧）
        qosa_task_sleep_ms(CAN_TX_PERIOD_MS);

        // 轮询接收队列（回环模式下刚发送的帧会回收到 RX）
        can_rx_poll();

        // 周期性上报任务剩余栈空间与总线状态
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
 * @brief CAN 示例初始化函数
 */
void unir_can_demo_init(void)
{
    QLOGI("enter can demo !!!");

    // 创建事件标志组
    if (qosa_flag_create(&g_can_evt) != 0)
    {
        QLOGE("create flag failed");
        return;
    }

    // 创建任务
    if (g_can_demo_task == QOSA_NULL)
    {
        qosa_task_create(
            &g_can_demo_task,               // 任务句柄指针
            CAN_DEMO_TASK_STACK_SIZE,       // 任务栈大小
            CAN_DEMO_TASK_PRIO,             // 任务优先级
            "unir_can_demo",                // 任务名称
            can_demo_process,               // 任务处理函数
            QOSA_NULL);                     // 任务参数
    }
}

// 不再单独注册到应用初始化列表，由 main.c 的 unir_hello_world_init() 统一调用
