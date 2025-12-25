#include "zf_common_headfile.h"
#include "zf_driver_gpio.h"
#include <type.h>
#include "vofa.h"
#include <stdlib.h>


u16 cpu_usage = 0;
VOL_U8 gTaskStick = 0;
void main()
{

	clock_init(SYSTEM_CLOCK_40M); // 时钟初始化
	debug_init();				  // 调试接口初始化
	P32 = 1;					  // 上电安全记录（示例）
	init_user();
#if (ENABLECOMM)
	vofa_init(); // 初始化 VOFA 通信（可选）
#endif
	while (1)
	{
		g_task_scheduler.run();
	}
}
