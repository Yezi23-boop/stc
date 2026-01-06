#ifndef FOCLIB_VERSION_H
#define FOCLIB_VERSION_H

/* ============================================================================
** Library Version Information
**
** 定义应用程序兼容的库版本范围。
** 版本格式: YYYYMMDDxx (xx为当日的次版本号, 00-99)
** 例如: 2025092401 表示 2025年9月24日 的第 1 个版本。
** ==========================================================================*/
#define FOCLIB_MIN_COMPATIBLE_VERSION 2025111101UL
#define FOCLIB_MAX_COMPATIBLE_VERSION 2025111101UL

/* ============================================================================
** Optional Features
** ==========================================================================*/
/**
 * @brief 宏开关：是否启用变更日志功能。
 *        设置为 1 以启用，设置为 0 以禁用。
 *        禁用此功能可以节省库占用的 ROM 空间。
 */
#define FOCLIB_ENABLE_CHANGELOG 1

/**
 * @brief 获取链接的 FOCLIB 库的运行时版本号。
 * @return 返回库的实际版本号 (格式: YYYYMMDDxx)。
 */
unsigned long FOCLIB_GetVersion(void);

/**
 * @brief 检查当前链接的库版本是否在应用程序兼容的范围内。
 * @param min_version 应用程序支持的最低版本。
 * @param max_version 应用程序支持的最高版本。
 * @return 如果版本兼容则返回 1，否则返回 0。
 */
int FOCLIB_IsVersionCompatible(unsigned long min_version, unsigned long max_version);


/**
 * @brief 获取链接的 FOCLIB 库的运行时版本号。
// ... existing code ...
int FOCLIB_IsVersionCompatible(unsigned long min_version, unsigned long max_version);

/**
 * @brief 获取库的变更日志。
 * @return 如果启用了日志功能，返回一个指向字符串数组的指针（以 NULL 结尾）。
 *         如果禁用了日志功能，返回 NULL。
 */
const char** FOCLIB_GetChangelog(void);

/**
 * @brief 根据版本号获取单条变更日志。
 * @param version 要查询的版本号 (格式: YYYYMMDDxx)。
 * @return 如果找到对应版本的日志，则返回该日志字符串。
 *         如果未找到或日志功能被禁用，则返回 "no this version"。
 */
const char* FOCLIB_GetLogByVersion(unsigned long version);

// void CheckLibVersion(void)
// {
//     // 使用断言来检查版本兼容性。
//     // 如果表达式为假 (0)，assert 将会触发，并停止程序。
//     // 编译器通常会将表达式中的宏展开，便于调试。
//     assert(FOCLIB_IsVersionCompatible(FOCLIB_MIN_COMPATIBLE_VERSION, FOCLIB_MAX_COMPATIBLE_VERSION));

//     // 如果程序能执行到这里，说明断言通过，版本是兼容的。
//     // 您可以保留这条打印语句用于调试，或在最终发布时移除。
//     printf("FOCLIB version check passed. Version: %lu\n", FOCLIB_GetVersion());
// }

#endif // FOCLIB_VERSION_H