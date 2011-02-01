/*
 * Copyright (c) 2010, The Android Open Source Project.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Neither the name of The Android Open Source Project nor the names
 *    of its contributors may be used to endorse or promote products
 *    derived from this software without specific prior written
 *    permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED 
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 */

#include <common.h>
#include <fastboot.h>

#define EFI_ENTRIES 128
#define EFI_NAMELEN 36

const u8 partition_type[16] = {
	0xa2, 0xa0, 0xd0, 0xeb, 0xe5, 0xb9, 0x33, 0x44,
	0x87, 0xc0, 0x68, 0xb6, 0xb7, 0x26, 0x99, 0xc7,
};

struct efi_entry {
	u8 type_uuid[16];
	u8 uniq_uuid[16];
	u64 first_lba;
	u64 last_lba;
	u64 attr;
	u16 name[EFI_NAMELEN];
};

void import_efi_partition(struct efi_entry *entry)
{
	struct fastboot_ptentry e;
	int n;
	if (memcmp(entry->type_uuid, partition_type, sizeof(partition_type)))
		return;
	for (n = 0; n < (sizeof(e.name)-1); n++)
		e.name[n] = entry->name[n];
	e.name[n] = 0;
	e.start = entry->first_lba;
	e.length = (entry->last_lba - entry->first_lba + 1) * 512;
	e.flags = 0;

	if (!strcmp(e.name,"environment"))
		e.flags |= FASTBOOT_PTENTRY_FLAGS_WRITE_ENV;
	fastboot_flash_add_ptn(&e);

	if (e.length > 0x100000)
		printf("%8d %7dM %s\n", e.start, e.length/0x100000, e.name);
	else
		printf("%8d %7dK %s\n", e.start, e.length/0x400, e.name);
}

void board_mmc_init(void)
{
	/* nothing to do this early */
}

int board_late_init(void)
{
	static fastboot_ptentry ptable = {
		.name = "ptable",
		.start = 0,
		.length = 128*1024,
		.flags = 0,
	};
	static unsigned char data[512];
	static struct efi_entry entry[4];
	int n,m,r;

	fastboot_flash_add_ptn(&ptable);
	if (mmc_init(0)) {
		printf("mmc init failed?\n");
		return 1;
	}
	r = mmc_read(0, 1, data, 512);
	if (r != 1) {
		printf("error reading partition table\n");
		return 1;
	}
	if (memcmp(data, "EFI PART", 8)) {
		printf("efi partition table not found\n");
		return 1;
	}
	for (n = 0; n < (128/4); n++) {
		r = mmc_read(0, 1 + n, (void*) entry, 512);
		if (r != 1) {
			printf("partition read failed\n");
			return 1;
		}
		for (m = 0; m < 4; m ++)
			import_efi_partition(entry + m);
	}
	return 0;
}


