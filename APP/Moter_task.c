#include "Moter_task.h"
#include "arm_output.h"
#include "cmsis_os.h"
void Moter_Task(void const *argument)
{
    unsigned axis;
    (void)argument;
    for (;;) {
        /* 每轴都重新过门控：许可、代次、反馈新鲜度、限位都在 ArmOutput_Send 里复核。 */
        for(axis=0;axis<7;axis++) {
            ArmOutput_Send(axis);
            osDelay(2);
        }
        osDelay(5);
    }
}
