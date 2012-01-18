#include <common.h>
#include <asm/omap_common.h>
#include <asm/arch/clocks.h>
#include <asm/arch/sys_proto.h>
#include <asm/utils.h>
#include <asm/omap_gpio.h>


#define MPU_BANDGAP_TSHUT_THRESHOLD 0x4A002390
#define GPU_BANDGAP_TSHUT_THRESHOLD 0x4A002394
#define CORE_BANDGAP_TSHUT_THRESHOLD 0x4A002398

#define MPU_BANDGAP_COUNTER 0x4A00239C
#define GPU_BANDGAP_COUNTER 0x4A0023A0
#define CORE_BANDGAP_COUNTER 0x4A0023A4

#define tshut_threshold_val 0x3930384
#define counter_val 0x8001D4C0

void tshut_init()
{
	/* Initialize TSHUT Thresholds */
	writel(tshut_threshold_val, MPU_BANDGAP_TSHUT_THRESHOLD);
	writel(tshut_threshold_val, GPU_BANDGAP_TSHUT_THRESHOLD);
	writel(tshut_threshold_val, CORE_BANDGAP_TSHUT_THRESHOLD);

	/* Initialize the counter */
	writel(counter_val, MPU_BANDGAP_COUNTER);
	writel(counter_val, GPU_BANDGAP_COUNTER);
	writel(counter_val, CORE_BANDGAP_COUNTER);
}

