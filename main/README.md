# UniRTOS 示例工程：CAN 收发 / CPU 占用率 / 任务堆栈

基于 Quectel UniRTOS SDK（`eigen_718` / EC718PM 平台）的应用示例，演示三项功能：

| 功能 | 对外 API | 说明 |
| --- | --- | --- |
| CAN 收发（回环） | `qosa_can_*` | 引脚复用、控制器初始化、波特率、收发对象配置，周期发送 + 轮询接收 |
| CPU 占用率 | `qosa_cpu_usage_start` / `qosa_cpu_usage_stop` | 周期回调上报 CPU 占用百分比 |
| 任务堆栈 | `qosa_task_get_stack_space` | 周期查询任务剩余栈空间 |

## 目录结构

```
main/
├── inc/
│   └── include.h          # 公共头文件
└── src/
    ├── main.c             # 应用入口：创建 hello world 任务，并启动 CAN demo
    └── can_demo.c         # CAN / CPU 占用率 / 任务堆栈 三项功能实现
```

## 功能说明

### 1. CAN（can_demo.c）

- 自动探测控制器能力，按 `内部回环 → 外部回环 → 正常模式` 顺序选择工作模式；
- 默认波特率 `500 Kbps`，扩展帧，ID `0x12345678`；
- 每 `1` 秒发送一帧，并轮询接收；
- 引脚配置（`qosa_pin_set_func`）：
  - CAN_TX：`QOSA_PIN_63`，func `7`
  - CAN_RX：`QOSA_PIN_64`，func `7`
  - CAN_STB：`QOSA_PIN_62`，func `7`

### 2. CPU 占用率

- 调用 `qosa_cpu_usage_start(2000, cb)`，每 `2` 秒回调一次，打印 `cpu usage: N%`。

### 3. 任务堆栈

- 调用 `qosa_task_get_stack_space()`，每 `5` 秒查询一次，打印 `stack usage: free = N bytes`。

## 编译

本目录为 UniRTOS 应用源码，需在 Quectel UniRTOS SDK（`eigen_718` 平台）工程中编译。
