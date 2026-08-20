# FreeRTOS 教程

FreeRTOS 是一个开源的实时操作系统（RTOS），专为嵌入式设备设计。本教程帮助小白快速入门 FreeRTOS。

---

## 1. 什么是 FreeRTOS

### RTOS 是什么

RTOS（Real-Time Operating System，实时操作系统）是一种能够精确控制任务执行时间的操作系统。

**实时系统的特点**：
- 确定性：在严格的时间约束内响应
- 优先级：重要任务可以抢占不重要任务
- 多任务：可以同时管理多个任务

### 什么时候需要 RTOS

| 场景 | 说明 |
|------|------|
| 多任务 | 需要同时处理多个独立功能（如同时控制电机和读取传感器） |
| 实时响应 | 对外部事件有严格的时间要求 |
| 模块化 | 代码需要清晰分离，便于维护 |

### FreeRTOS 优势

- **开源免费**：MIT 许可证，商业和个人使用都免费
- **轻量级**：内核仅 3-4K 行代码，RAM 占用小
- **可移植性**：支持 40+ 种 MCU 架构
- **易学习**：API 简洁，文档完善

---

## 2. 核心概念

### 任务（Task）

任务是 FreeRTOS 的基本执行单元，可以理解为一个小程序。

```
┌─────────────────┐
│     Task 1      │ ← 独立运行
│  (LED 闪烁)     │
└─────────────────┘
┌─────────────────┐
│     Task 2      │ ← 独立运行
│  (读取传感器)    │
└─────────────────┘
┌─────────────────┐
│     Task 3      │ ← 独立运行
│  (发送数据)     │
└─────────────────┘
```

### 任务状态

```
        创建
          │
          ▼
    ┌──────────┐
───►│  Running  │◄──── 调度器选中
    └────┬─────┘
         │
    ┌────┴────┐
    │         │
    ▼         ▼
┌───────┐ ┌────────┐
│Ready  │ │ Blocked │
└───────┘ └────────┘
```

| 状态 | 说明 |
|------|------|
| Running | 正在执行 |
| Ready | 就绪，等待被调度 |
| Blocked | 阻塞（等待延时、信号量、队列等） |
| Suspended | 挂起（被显式暂停） |

### 调度器（Scheduler）

调度器决定哪个任务在什么时候运行。

**调度策略**：
- 优先级抢占：高优先级任务可以打断低优先级任务
- 时间片轮转：同优先级任务轮流执行

```
优先级: 高 ──────────────────────────────► 低

Task A (优先级3) ████                    ████                    ████
Task B (优先级2)      ██████████████           ██████████████
Task C (优先级1)           ████                    ████
                    t1      t2      t3      t4      t5      t6
```

---

## 3. 创建第一个 FreeRTOS 项目

### 方式一：STM32CubeIDE（推荐）

1. 创建新项目，选择你的 STM32 芯片
2. 在 **Middleware** 中启用 **FREERTOS**
3. 配置任务（Task）
4. 生成代码

### 方式二：Arduino

1. 安装 FreeRTOS 库：`Sketch -> Include Library -> Manage Libraries`，搜索 FreeRTOS 并安装
2. 重启 Arduino IDE

### 方式三：ESP32

ESP-IDF 已集成 FreeRTOS，直接使用即可。

---

## 4. 基础 API

### 创建任务

```c
#include "FreeRTOS.h"
#include "task.h"

// 任务函数
void TaskBlink(void *pvParameters) {
    while(1) {
        // 任务代码
        vTaskDelay(pdMS_TO_TICKS(1000));  // 延时 1 秒
    }
}

void app_main(void) {
    // 创建任务
    xTaskCreate(
        TaskBlink,           // 任务函数
        "Blink",             // 任务名称
        1024,                // 栈大小（字节）
        NULL,                // 参数
        1,                   // 优先级
        NULL                 // 任务句柄
    );
}
```

### 任务参数说明

| 参数 | 说明 |
|------|------|
| pvTaskCode | 任务函数指针 |
| pcName | 任务名称（用于调试） |
| usStackDepth | 栈大小（字为单位） |
| pvParameters | 传给任务的参数 |
| uxPriority | 优先级（数值越大优先级越高） |
| pvCreatedTask | 任务句柄（可用于删除/挂起） |

### 延时

```c
// 方法1：相对延时（从调用时刻开始延时）
vTaskDelay(pdMS_TO_TICKS(1000));  // 延时 1000ms

// 方法2：绝对延时（周期性任务，推荐使用）
TickType_t lastWakeTime = xTaskGetTickCount();
while(1) {
    // 执行任务
    vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(500));  // 每 500ms 执行一次
}
```

---

## 5. 实战示例

### 示例 1：双 LED 交替闪烁

```c
#include "Arduino_FreeRTOS.h"

void Task1(void *pvParameters) {
    pinMode(LED_BUILTIN, OUTPUT);
    while(1) {
        digitalWrite(LED_BUILTIN, HIGH);
        vTaskDelay(pdMS_TO_TICKS(500));
        digitalWrite(LED_BUILTIN, LOW);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void Task2(void *pvParameters) {
    pinMode(7, OUTPUT);
    while(1) {
        digitalWrite(7, HIGH);
        vTaskDelay(pdMS_TO_TICKS(200));
        digitalWrite(7, LOW);
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

void setup() {
    Serial.begin(9600);
    
    xTaskCreate(Task1, "Task1", 128, NULL, 1, NULL);
    xTaskCreate(Task2, "Task2", 128, NULL, 1, NULL);
    
    vTaskStartScheduler();
}

void loop() {
    // 空循环，任务由 FreeRTOS 管理
}
```

### 示例 2：ESP32 多任务

```c
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

void Task1(void *pvParameters) {
    while(1) {
        Serial.println("Task 1 running");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void Task2(void *pvParameters) {
    while(1) {
        Serial.println("Task 2 running");
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void setup() {
    Serial.begin(115200);
    
    xTaskCreate(Task1, "Task1", 2048, NULL, 1, NULL);
    xTaskCreate(Task2, "Task2", 2048, NULL, 2, NULL);  // Task2 优先级更高
}

void loop() {
    vTaskDelay(pdMS_TO_TICKS(1000));
}
```

---

## 6. 任务管理

### 挂起和恢复任务

```c
TaskHandle_t myTaskHandle;

// 创建任务
xTaskCreate(TaskFunction, "MyTask", 1024, NULL, 1, &myTaskHandle);

// 挂起任务
vTaskSuspend(myTaskHandle);

// 恢复任务
vTaskResume(myTaskHandle);
```

### 删除任务

```c
void TaskToDelete(void *pvParameters) {
    // 执行完成后删除自己
    vTaskDelete(NULL);
}

// 或从其他任务删除
vTaskDelete(myTaskHandle);
```

### 获取任务信息

```c
// 获取任务数量
UBaseType_t count = uxTaskGetNumberOfTasks();

// 获取高水位（剩余栈空间）
UBaseType_t highWaterMark = uxTaskGetStackHighWaterMark(NULL);
```

---

## 7. 任务间通信

### 队列（Queue）

队列用于在任务之间传递数据。

```c
QueueHandle_t xQueue;

// 创建队列（10个 int 类型数据）
xQueue = xQueueCreate(10, sizeof(int));

// 发送数据
int sendData = 100;
xQueueSend(xQueue, &sendData, 0);

// 接收数据
int receiveData;
xQueueReceive(xQueue, &receiveData, portMAX_DELAY);  // 永久等待
```

### 信号量（Semaphore）

信号量用于同步和资源保护。

```c
SemaphoreHandle_t xSemaphore;

// 创建信号量
xSemaphore = xSemaphoreCreateMutex();

// 获取信号量
xSemaphoreTake(xSemaphore, portMAX_DELAY);

// 释放信号量
xSemaphoreGive(xSemaphore);
```

---

## 8. 常见问题

### 任务不运行

| 检查项 | 解决方法 |
|--------|----------|
| 栈空间太小 | 增加 `usStackDepth` |
| 堆空间不足 | 增加 `configTOTAL_HEAP_SIZE` |
| 优先级设置 | 确保任务优先级 > 0 |
| 延时函数 | 使用 `vTaskDelay()` 而不是普通 `delay()` |

### 堆栈溢出

启用堆栈溢出检测：

```c
// FreeRTOSConfig.h
#define configCHECK_FOR_STACK_OVERFLOW 2
```

### 内存不足

实现内存分配失败钩子：

```c
void vApplicationMallocFailedHook(void) {
    Serial.println("Memory allocation failed!");
}
```

---

## 9. 参考资源

- [FreeRTOS 官方文档](https://www.freertos.org/)
- [FreeRTOS API 参考](https://www.freertos.org/a00106.html)
- [FreeRTOS 官方示例](https://www.freertos.org/a00104.html)

---

## 10. 下一步

- 学习队列和信号量深入使用
- 了解中断与任务的配合
- 探索 FreeRTOS+ 组件（TCP、MQTT 等）
