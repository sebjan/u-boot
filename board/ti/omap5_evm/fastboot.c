#include <common.h>
#include <usb/fastboot.h>

static struct usb_string def_usb_fb_strings[] = {
	{ FB_STR_SERIAL_IDX,            "abcd+efg" },
	{ FB_STR_PROC_REV_IDX,          "ES0.0" },
	{ FB_STR_PROC_TYPE_IDX,         "VirtIO" },
	{  }
};

static struct usb_gadget_strings def_fb_strings = {
	.language       = 0x0409, /* en-us */
	.strings        = def_usb_fb_strings,
};

/*
 * Hardcoded memory region to stash data which comes over USB before it is
 * stored on media
 */
DECLARE_GLOBAL_DATA_PTR;
#define SZ_16M                          0x01000000
#define SZ_128M                         0x08000000
#define CFG_FASTBOOT_TRANSFER_BUFFER (void *)(gd->bd->bi_dram[0].start + SZ_16M)
#define CFG_FASTBOOT_TRANSFER_BUFFER_SIZE (SZ_128M - SZ_16M)

int fastboot_board_init(struct fastboot_config *interface,
		struct usb_gadget_strings **str) {

	interface->transfer_buffer = CFG_FASTBOOT_TRANSFER_BUFFER;
	interface->transfer_buffer_size = CFG_FASTBOOT_TRANSFER_BUFFER_SIZE;

	*str = &def_fb_strings;
	return 0;
}
