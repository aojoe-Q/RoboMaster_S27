#include "master_link.h"
#include "bsp_uart.h"
/* APP 侧唯一的消费点。调试时看这份副本，绝不要读 Custom_RC[]。 */
MasterRawSnapshot app_master_raw;
/* 串口任务里周期调用：重启动图传 DMA 接收，并刷新主手/键盘快照。 */
void MasterLink_Poll(void)
{
    extern void BSP_ImageRx_Service(void);
    BSP_ImageRx_Service();
    (void)MasterLink_Read(&app_master_raw);
}
