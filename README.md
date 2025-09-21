# FOC 电机控制库

## 简介

本库提供了一套完整的磁场定向控制（FOC）算法，用于精确控制无刷直流电机（BLDC）和永磁同步电机（PMSM）。它包括了坐标变换、SVPWM、PID控制等核心模块，并针对STM32等MCU平台进行了优化。

## 特性

-   **通用性**: 适用于BLDC和PMSM电机控制。
-   **模块化**: 包含坐标变换（Clark/Park）、SVPWM、PID控制器等独立模块，易于定制和扩展。
-   **可移植性**: 驱动层与算法层解耦，方便移植到不同MCU平台。
-   **参数可配置**: 电机参数、PID参数等均可通过结构体配置，灵活适应不同电机。
-   **代码注释详尽**: 核心算法和接口均有详细注释，方便理解和使用。

## 文件结构

```
ESC_V1.0.0/
├── ESC/
│   └── FOC/
│       ├── foc_alg.h     # FOC算法头文件：包含结构体定义、函数声明
│       ├── foc_alg.c     # FOC算法源文件：实现FOC核心算法
│       ├── foc_drv.h     # FOC驱动头文件：定义硬件接口
│       ├── foc_drv.c     # FOC驱动源文件：实现STM32硬件接口
│       └── README.md     # 自述文件
├── ...
└── ...
```

## 依赖

-   **硬件**:
    -   STM32或其他MCU
    -   无刷直流电机或永磁同步电机
    -   编码器（可选，用于闭环控制）
    -   电流传感器
-   **软件**:
    -   标准C语言库
    -   STM32 HAL库（如果使用STM32）

## 使用方法

1.  **配置电机参数**:
    -   修改`foc_alg.h`中的`Motor_ConfigTypeDef`结构体，设置电机极对数、额定电流/电压、定子电感/电阻等参数。
2.  **实现硬件接口**:
    -   在`foc_drv.c`中，根据您的硬件平台，实现`Motor_DrvTypeDef`结构体中定义的函数指针，包括PWM输出、电流/角度采集、延时等接口。
3.  **初始化FOC**:
    -   调用`foc_init()`函数，初始化FOC算法相关参数和结构体。
4.  **控制电机**:
    -   在主循环中，周期性调用以下函数：
        -   `update_dt()`: 更新时间差。
        -   `update_IaIbIc()`: 更新三相电流。
        -   `update_angle_el()`: 更新电角度。
        -   `Calculate_PID()`: 计算PID输出。
        -   `update_svpwm()`或`update_spwm()`: 更新SVPWM或SPWM，控制电机。

## 接口说明

### 结构体

-   `Time_t`: 时间管理结构体
-   `LPF_t`: 一阶低通滤波器结构体
-   `PID_t`: PID控制器结构体
-   `Motor_ConfigTypeDef`: 电机硬件参数配置结构体
-   `Motor_AlgTypeDef`: 电机控制算法层结构体
-   `Motor_DataTypeDef`: 电机数据采集与校准结构体
-   `Motor_DrvTypeDef`: 电机驱动层函数指针结构体
-   `Motor_HandleTypeDef`: 电机控制总句柄结构体

### 函数

-   `foc_init(Motor_HandleTypeDef *motor)`: 初始化FOC
-   `update_dt(Motor_HandleTypeDef *motor)`: 更新时间差
-   `update_IaIbIc(Motor_HandleTypeDef *motor)`: 更新三相电流
-   `update_angle_el(Motor_HandleTypeDef *motor)`: 更新电角度
-   `Calculate_PID(float target, float feedback, float dt ,PID_t* pid)`: PID计算
-   `update_svpwm(Motor_HandleTypeDef *motor)`: 更新SVPWM
-   `update_spwm(Motor_HandleTypeDef *motor)`: 更新SPWM

## 示例代码

```c
// filepath: main.c
#include "foc_alg.h"
#include "foc_drv.h"

Motor_HandleTypeDef motor;

int main(void)
{
    // ... 初始化 ...

    foc_init(&motor); // 初始化FOC

    while (1)
    {
        update_dt(&motor); // 更新时间差
        update_IaIbIc(&motor); // 更新三相电流
        update_angle_el(&motor); // 更新电角度

        // ... 计算目标速度/位置 ...
        float target_velocity = 10.0f; // 目标速度

        // 计算速度环PID输出
        motor.MotorAlg.Uq = Calculate_PID(target_velocity, motor.MotorAlg.Velocity, motor.time.dt, &motor.MotorAlg.velocity_pid);

        // SVPWM控制
        update_svpwm(&motor);
    }
}
```

## 贡献

欢迎提交Pull Request，贡献您的代码和想法！

## 许可证

MIT许可证
