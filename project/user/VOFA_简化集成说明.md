# VOFA+ FireWater 协议 - 简化集成方案

## ✅ 优势：使用系统自带函数

**无需修改任何中断代码！** 系统已经自动处理了串口接收：

```
串口中断接收 → wireless_uart_callback() → FIFO 自动写入
```

我们只需要在主循环中读取 FIFO 数据即可！

---

## 📝 集成步骤（仅需 2 步）

### 步骤 1：在 int_user.c 中初始化

```c
#include "vofa.h"

void int_user(void)
{
    // ... 其他初始化代码 ...
    
    wireless_uart_init();  // 已有的初始化
    vofa_init();           // 添加：初始化 VOFA 解析器
    
    // ... 其他初始化代码 ...
}
```

### 步骤 2：在 main.c 主循环中处理

```c
#include "vofa.h"

void main()
{
    clock_init(SYSTEM_CLOCK_40M); 
    debug_init();				  
    P32 = 1;					  
    int_user();
    
    char vofa_cmd[32];
    
    while (1)
    {
        // 从 FIFO 读取并解析数据
        vofa_parse_from_fifo();
        
        // 检查是否有完整命令
        if(vofa_get_command(vofa_cmd, 32))
        {
            // 处理命令
            handle_vofa_command(vofa_cmd);
        }
        
        // 发送数据
        printf("%f,%f,%f,%f\n", speed_l, speed_r, test_speed, 0.0);
    }
}

// 命令处理函数
void handle_vofa_command(char *cmd)
{
    char *eq_pos = strchr(cmd, '=');
    
    if(eq_pos != NULL)
    {
        char param_name[16] = {0};
        uint8 name_len = (uint8)(eq_pos - cmd);
        float value;
        
        if(name_len < 16)
        {
            memcpy(param_name, cmd, name_len);
            param_name[name_len] = '\0';
            value = atof(eq_pos + 1);
            
            // 处理参数
            if(strcmp(param_name, "KP") == 0)
            {
                motors_pid.left_PID.Kp = value;
                printf("Set Kp = %.2f\n", value);
            }
            else if(strcmp(param_name, "SPEED") == 0)
            {
                test_speed = value;
                printf("Set Speed = %.2f\n", value);
            }
            // ... 添加更多参数
        }
    }
    else
    {
        // 处理无参数命令
        if(strcmp(cmd, "START") == 0)
        {
            printf("Motor START\n");
        }
        else if(strcmp(cmd, "STOP") == 0)
        {
            motor = 0;
            printf("Motor STOP\n");
        }
    }
}
```

---

## 🔄 完整数据流程

```
VOFA+ 发送 "KP=1.5!"
    ↓
UART3 硬件接收
    ↓
DMA_UART3_IRQHandler() 中断
    ↓
wireless_uart_callback() 
    ↓
fifo_write_buffer() ← 系统自动写入 FIFO
    ↓
主循环: vofa_parse_from_fifo()
    ↓
wireless_uart_read_buffer() ← 使用系统函数读取
    ↓
vofa_parse_byte() 逐字节解析
    ↓
检测到 '!' 标记完成
    ↓
vofa_get_command() 获取 "KP=1.5"
    ↓
handle_vofa_command() 执行设置
```

---

## 🎯 关键函数说明

### 1. `vofa_parse_from_fifo()`
```c
void vofa_parse_from_fifo(void)
{
    uint8 dat;
    
    // 使用系统自带函数从 FIFO 读取
    while(wireless_uart_read_buffer(&dat, 1) > 0)
    {
        vofa_parse_byte(dat);  // 解析字节
    }
}
```

**优势**：
- ✅ 无需修改中断代码
- ✅ 利用系统自带的 64 字节 FIFO 缓冲
- ✅ 线程安全，不影响中断
- ✅ 代码简洁

### 2. `wireless_uart_read_buffer()`（系统自带）
```c
// 已经在 zf_device_wireless_uart.c 中实现
uint32 wireless_uart_read_buffer(uint8 *buff, uint32 len)
{
    uint32 data_len = len;
    fifo_read_buffer(&wireless_uart_fifo, buff, &data_len, FIFO_READ_AND_CLEAN);
    return data_len;
}
```

---

## 📋 对比两种方案

### ❌ 旧方案（不推荐）
```c
// 需要修改 wireless_uart_callback()
void wireless_uart_callback(uint8 uart_dat)
{
    fifo_write_buffer(&wireless_uart_fifo, &uart_dat, 1);
    vofa_parse_byte(uart_dat);  // 在中断中解析
}
```

**缺点**：
- 需要修改库文件
- 在中断中处理逻辑
- 升级库时需要重新修改

### ✅ 新方案（推荐）
```c
// 主循环中处理
while(1)
{
    vofa_parse_from_fifo();  // 从 FIFO 读取并解析
    
    if(vofa_get_command(cmd, 32))
    {
        handle_vofa_command(cmd);
    }
}
```

**优点**：
- ✅ 不修改库文件
- ✅ 所有处理在主循环
- ✅ 易于调试和维护
- ✅ 升级库时无需修改

---

## 📊 性能说明

- **FIFO 大小**: 64 字节（系统默认）
- **解析速度**: 每次主循环读取所有可用数据
- **延迟**: < 1ms（115200 波特率下）
- **可靠性**: FIFO 硬件缓冲 + DMA 传输

---

## 💡 使用示例

### 发送命令调整 PID 参数
```
VOFA+ 发送: KP=1.5!
单片机响应: "Set Kp = 1.50"
```

### 发送命令设置速度
```
VOFA+ 发送: SPEED=100!
单片机响应: "Set Speed = 100.00"
```

### 发送控制命令
```
VOFA+ 发送: START!
单片机响应: "Motor START"
```

---

## 🔧 需要包含的文件

1. **vofa.h** - 头文件
2. **vofa.c** - 实现文件
3. **main.c** - 添加 `#include "vofa.h"` 和调用函数

**无需修改的文件**：
- ❌ zf_device_wireless_uart.c
- ❌ isr.c
- ❌ 任何库文件

---

## ✨ 总结

使用系统自带的 `wireless_uart_read_buffer()` 函数：
- 代码更简洁
- 不修改库文件
- 易于维护
- 性能更好

完全满足 VOFA+ FireWater 协议的接收和解析需求！
