#include <common.h>
#include <android_image.h>

u32 android_img_get_end(struct andr_img_hdr *hdr)
{
	u32 size = 0;
	/*
	 * The header takes a full page, the remaining components are aligned
	 * on page boundary
	 */
	size += hdr->page_size;
	size += ALIGN(hdr->kernel_size, hdr->page_size);
	size += ALIGN(hdr->ramdisk_size, hdr->page_size);
	size += ALIGN(hdr->second_size, hdr->page_size);

	return size;
}

u32 android_img_get_kload(struct andr_img_hdr *hdr)
{
	return hdr->kernel_addr;
}

static char andr_tmp_str[ANDR_BOOT_ARGS_SIZE + 1];
int android_image_get_kernel(struct andr_img_hdr *hdr, int verify)
{
	/*
	 * Not all Android tools use the id field for signing the image with
	 * sha1 (or anything) so we don't check it. It is not obvious that the
	 * string is null terminated so we take care of this.
	 */
	strncpy(andr_tmp_str, hdr->name, ANDR_BOOT_NAME_SIZE);
	andr_tmp_str[ANDR_BOOT_NAME_SIZE] = '\0';
	if (strlen(andr_tmp_str))
		printf("Android's image name: %s\n", andr_tmp_str);

	printf("Kernel load addr 0x%08x size %u KiB\n",
			hdr->kernel_addr, DIV_ROUND_UP(hdr->kernel_size, 1024));
	strncpy(andr_tmp_str, hdr->cmdline, ANDR_BOOT_ARGS_SIZE);
	andr_tmp_str[ANDR_BOOT_ARGS_SIZE] = '\0';
	if (strlen(andr_tmp_str)) {
		printf("Kernel command line: %s\n", andr_tmp_str);
		setenv("bootargs", andr_tmp_str);
	}
	if (hdr->ramdisk_size)
		printf("RAM disk load addr 0x%08x size %u KiB\n",
				hdr->ramdisk_addr,
				DIV_ROUND_UP(hdr->ramdisk_size, 1024));
	return 0;
}
