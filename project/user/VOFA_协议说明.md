# VOFA+ FireWater 协议接收与解析说明（简化版）

## 一、协议原理

### 1. VOFA+ FireWater 协议格式
- **数据格式**: 以 ASCII 字符发送，以 `!` (0x21) 作为帧尾标识
- **示例**: `"PR=2.32!"` 会被转换为 ASCII 码序列发送

### 2. 数据转换过程
```
发送的字符串: "PR=2.32!"

字符 -> ASCII 码（十六进制）:
'P'  -> 0x50 (80)
'R'  -> 0x52 (82)
'='  -> 0x3D (61)
'2'  -> 0x32 (50)
'.'  -> 0x2E (46)
'3'  -> 0x33 (51)
'2'  -> 0x32 (50)
'!'  -> 0x21 (33) ← 帧尾标识
```

## 二、代码集成步骤（仅需 2 步！）

### ✅ 步骤 1: 在 int_user.c 中初始化

在 `int_user.c` 的 `int_user()` 函数中添加：

```c
#include "vofa.h"

void int_user(void)
{
    // ... 其他初始化代码 ...
    
    wireless_uart_init();  // 无线串口初始化（已有）
    vofa_init();           // 添加 VOFA 初始化
    
    // ... 其他初始化代码 ...
}
```

### ✅ 步骤 2: 在主循环中处理命令

在 `main.c` 中添加命令处理：

```c
#include "vofa.h"

void main()
{
    clock_init(SYSTEM_CLOCK_40M); 
    debug_init();				  
    P32 = 1;					  
    int_user();
    
    char vofa_cmd[32];  // 命令缓冲区
    
    while (1)
    {
        // 检查是否接收到 VOFA 命令
        if(vofa_get_command(vofa_cmd, 32))
        {
            // 解析并处理命令
            handle_vofa_command(vofa_cmd);
        }
        
        // 发送数据到 VOFA+
        printf("%f,%f,%f,%f\n", speed_l, speed_r, test_speed, 0.0);
        
        // 其他代码...
    }
}

// 命令处理函数
void handle_vofa_command(char *cmd)
{
    char *eq_pos = strchr(cmd, '=');
    
    if(eq_pos != NULL)
    {
        // 提取参数名
        char param_name[16] = {0};
        uint8 name_len = (uint8)(eq_pos - cmd);
        float value;
        
        if(name_len < 16)
        {
            memcpy(param_name, cmd, name_len);
            param_name[name_len] = '\0';
            value = atof(eq_pos + 1);
            
            // 根据参数名处理
            if(strcmp(param_name, "KP") == 0)
            {
                motors_pid.left_PID.Kp = value;
                printf("Set Kp = %.2f\n", value);
            }
            else if(strcmp(param_name, "KI") == 0)
            {
                motors_pid.left_PID.Ki = value;
                printf("Set Ki = %.2f\n", value);
            }
            else if(strcmp(param_name, "KD") == 0)
            {
                motors_pid.left_PID.Kd = value;
                printf("Set Kd = %.2f\n", value);
            }
            else if(strcmp(param_name, "SPEED") == 0)
            {
                test_speed = value;
                printf("Set Speed = %.2f\n", value);
            }
        }
    }
    else
    {
        // 处理无参数命令
        if(strcmp(cmd, "START") == 0)
        {
            // 启动电机
        }
        else if(strcmp(cmd, "STOP") == 0)
        {
            motor = 0;
            speed_run = 0;
        }
    }
}
```

## 三、完整工作流程

```
VOFA+ 上位机                     单片机
    |                              |
    | 发送 "KP=1.5!"               |
    |----------------------------->|
    |                              | ① 串口硬件接收
    |                              |    ↓
    |                              | ② DMA_UART3_IRQHandler() 中断
    |                              |    ↓
    |                              | ③ wireless_uart_callback() 
    |                              |    ↓
    |                              | ④ fifo_write_buffer() 自动写入 FIFO
    |                              |    （系统自动完成，无需修改）
    |                              |
    |                              | ⑤ 主循环: vofa_parse_from_fifo()
    |                              |    从 FIFO 读取数据
    |                              |    ↓
    |                              | ⑥ vofa_parse_byte() 逐字节解析
    |                              |    检测到 '!' 标记完成
    |                              |    ↓
    |                              | ⑦ vofa_get_command() 获取 "KP=1.5"
    |                              |    ↓
    |                              | ⑧ handle_vofa_command() 处理
    |                              |    提取: param="KP", value=1.5
    |                              |    执行: motors_pid.left_PID.Kp = 1.5
    |                              |
```

**关键优势：无需修改任何中断代码或库文件！**

## 四、使用示例

### 示例 1: 调整 PID 参数
```
VOFA+ 发送: "KP=1.5!"
单片机接收并解析: Kp = 1.5
```

### 示例 2: 设置目标速度
```
VOFA+ 发送: "SPEED=100!"
单片机接收并解析: Speed = 100.0
```

### 示例 3: 执行命令
```
VOFA+ 发送: "START!"
单片机接收并解析: 执行启动操作
```

## 五、关键点说明

1. **帧尾标识**: FireWater 协议使用 `!` (ASCII: 0x21) 作为帧尾
2. **逐字节接收**: 串口中断每次接收一个字节，传递给解析器
3. **状态机解析**: 解析器维护状态，累积字节直到检测到帧尾
4. **命令格式**: 
   - 带参数: `"参数名=值!"`
   - 无参数: `"命令名!"`

## 六、为什么不需要修改中断代码？

系统已经自动完成了数据接收：

1. **串口中断自动触发** → `DMA_UART3_IRQHandler()`
2. **自动调用回调函数** → `wireless_uart_callback()`  
3. **自动写入 FIFO** → `fifo_write_buffer()`
4. **我们只需读取** → `vofa_parse_from_fifo()` 使用 `wireless_uart_read_buffer()` 读取

**完全使用系统自带函数，不破坏原有架构！**

## 七、注意事项

1. 确保在 `main.c` 中包含 `#include "vofa.h"`
2. 在主循环中**必须调用** `vofa_parse_from_fifo()`
3. 缓冲区大小 (VOFA_BUFFER_SIZE = 64) 已足够使用
4. 命令字符串不包含帧尾 `!`，它只是标识符
5. **无需修改任何库文件**（isr.c, wireless_uart.c 等）

## 八、调试建议

1. 在 `handle_vofa_command()` 开头打印接收到的命令：
   ```c
   printf("Received: %s\n", cmd);
   ```

2. 检查 FIFO 是否有数据：
   ```c
   if(wireless_uart_read_buffer(&test, 1) > 0) {
       printf("FIFO has data: 0x%02X\n", test);
   }
   ```

3. 使用 VOFA+ 的日志功能查看发送和接收的数据

## 九、完整使用流程总结

```
[准备工作]
1. 添加 vofa.h 和 vofa.c 到项目
2. 在 main.c 中 #include "vofa.h"

[初始化阶段]
3. int_user() 中调用 vofa_init()

[主循环运行]
4. while(1) {
       vofa_parse_from_fifo();           // 从 FIFO 读取并解析
       if(vofa_get_command(cmd, 32)) {  // 检查是否有完整命令
           handle_vofa_command(cmd);     // 处理命令
       }
       printf("...");                     // 发送数据
   }

[VOFA+ 上位机]
5. 发送命令: "KP=1.5!" 或 "START!"
6. 查看返回数据和日志
```

**就是这么简单！** 🎉
