#ifndef __TYPE_H
#define __TYPE_H  // 头文件保护宏：只写一次，配对一次

#include <stddef.h>   // 标准定义（如 size_t、NULL，老编译器兼容）

/*************************************************************************************
 * 1. 基础数据类型重定义（统一命名风格，明确长度，无依赖）
 ************************************************************************************/
// 底层类型定义（编译器原生支持，不用 stdint.h，兼容老编译器）
typedef unsigned char       uint8_t;  // 8位无符号整数
typedef signed char         int8_t;   // 8位有符号整数
typedef unsigned int        uint16_t; // 16位无符号整数（老编译器 int 通常16位）
typedef signed int          int16_t;  // 16位有符号整数
typedef unsigned long       uint32_t; // 32位无符号整数（老编译器 long 通常32位）
typedef signed long         int32_t;  // 32位有符号整数

// 无符号整数（计数/地址/状态值优先使用）
typedef uint8_t    u8;    // 0 ~ 255
typedef uint16_t   u16;   // 0 ~ 65535
typedef uint32_t   u32;   // 0 ~ 4294967295

// 有符号整数（需表示正负值时使用）
typedef int8_t     s8;    // -128 ~ 127
typedef int16_t    s16;   // -32768 ~ 32767
typedef int32_t    s32;   // -2147483648 ~ 2147483647

// 浮点类型（根据精度需求选择）
typedef float      f32;   // 单精度浮点（32位，部分MCU支持）
typedef double     f64;   // 双精度浮点（64位，老MCU可能不支持，按需保留）

// 通用类型（适配不同场景）
typedef size_t     len_t; // 长度类型（无符号，适配 sizeof 返回值）
typedef ptrdiff_t  diff_t;// 差值类型（有符号，指针/索引差值计算）

/*************************************************************************************
 * 2. 布尔类型定义（兼容老编译器，语义清晰）
 ************************************************************************************/
#define true    1           // 真（宏定义，先声明再使用）
#define false   0           // 假

// 布尔类型别名（枚举类型，用于状态标志/函数返回值）
typedef enum {
    BOOL_FALSE  = false,    // 枚举成员：假（值=0）
    BOOL_TRUE   = true      // 枚举成员：真（值=1）
} bool_t;

// 注意：删除了 "bool_t bool;"！不要在头文件中定义全局变量，仅声明（见下文）
// 若需全局布尔变量，改为：extern bool_t g_global_bool;（定义在 .c 文件中）

/*************************************************************************************
 * 3. 通用枚举定义（项目全局复用，替代魔法数字）
 ************************************************************************************/
// 通用状态码（函数返回值优先使用）
typedef enum {
    STATUS_OK      = 0,    // 成功
    STATUS_ERR     = 1,    // 通用错误
    STATUS_BUSY    = 2,    // 资源忙
    STATUS_TIMEOUT = 3,    // 超时
    STATUS_INVALID = 4,    // 参数无效
    STATUS_FULL    = 5,    // 缓冲区/存储满
    STATUS_EMPTY   = 6     // 缓冲区/存储空
} StatusEnum;

// 通用开关状态
typedef enum {
    SWITCH_OFF = 0,        // 关闭
    SWITCH_ON  = 1         // 开启
} SwitchEnum;

/*************************************************************************************
 * 4. 全局常量/宏定义（通用常量，避免硬编码）
 ************************************************************************************/
// 数据长度常量（字节数）
#define BYTE_LEN    1       // 1字节 = 8位
#define WORD_LEN    2       // 1字 = 16位（按需修改）
#define DWORD_LEN   4       // 双字 = 32位
#define QWORD_LEN   8       // 四字 = 64位

// 通用掩码（位操作常用）
#define MASK_BIT0   (1U << 0)  // 第0位掩码
#define MASK_BIT1   (1U << 1)  // 第1位掩码
#define MASK_BIT2   (1U << 2)  // 第2位掩码
#define MASK_BIT3   (1U << 3)  // 第3位掩码
#define MASK_BIT4   (1U << 4)  // 第4位掩码
#define MASK_BIT5   (1U << 5)  // 第5位掩码
#define MASK_BIT6   (1U << 6)  // 第6位掩码
#define MASK_BIT7   (1U << 7)  // 第7位掩码

// 类型检查宏（可选，编译期校验类型合法性）
#define TYPE_CHECK(type, expr)  ((void)(sizeof((type){0}) == sizeof(expr)))

/*************************************************************************************
 * 5. volatile 增强（中断/多线程共享变量必加）
 ************************************************************************************/
#define VOLATILE    volatile
#define VOL_U8      VOLATILE u8
#define VOL_U16     VOLATILE u16
#define VOL_U32     VOLATILE u32
#define VOL_S32     VOLATILE s32

#endif // __TYPE_H  // 唯一的#endif，与开头的#ifndef配对（必须放在文件末尾）
