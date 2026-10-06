#include "main.h"
#include "i2c.h"
#include "spi.h"
#include "eeprom.h"
#include "task_console.h"
#include "task_io_expander.h"
#include "task_eeprom.h"

#if defined(ICE13)
void peripheral_init(void);
void main_app(void);
#endif

int main(void)
{
    cy_rslt_t result;

    /* 1. 初始化板级支持包 (BSP) */
    result = cybsp_init();
    if (result != CY_RSLT_SUCCESS) {
        CY_ASSERT(0);
    }

    /* 2. 启用全局中断 */
    __enable_irq();

#if defined(ICE13)
    peripheral_init();
    main_app();
    for (;;)
    {
    }
#else
    /* 3. 硬件外设初始化 */
    // 初始化 Module Site 2 的 I2C，用于控制扩展器
    result = i2c_init(MODULE_SITE_2);
    if (result != CY_RSLT_SUCCESS) {
        CY_ASSERT(0);
    }

    // 初始化 Module Site 0 的 SPI，用于 M95512 EEPROM
    result = spi_init(EEPROM_SPI_MODULE);
    if (result != CY_RSLT_SUCCESS) {
        CY_ASSERT(0);
    }

    // 初始化 EEPROM CS 引脚
    result = eeprom_init();
    if (result != CY_RSLT_SUCCESS) {
        CY_ASSERT(0);
    }

    /* 4. FreeRTOS 任务初始化 */
    // 启动串口控制台任务
    task_console_init();
    
    // 启动 I/O 扩展器管理任务
    task_io_exp_init();
    
    // 注册 EEPROM CLI 命令 (er / ew)
    task_eeprom_init();

    /* 5. 启动 FreeRTOS 调度器 */
    // 此函数调用后，程序将进入任务循环，不再返回
    vTaskStartScheduler();

    /* 如果程序运行到这里，说明内存不足或启动失败 */
    for (;;) {}
#endif
}
