/*
 * (C) Copyright 2011
 * Texas Instruments, <www.ti.com>
 * Author: Vikram Pandita <vikram.pandita@ti.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#include <config.h>
#include <common.h>
#include <sparse.h>

//#define DEBUG

#define SPARSE_HEADER_MAJOR_VER 1

u8
do_unsparse(unsigned char *source,
		u32 sector,
		char *slot_no)
{
	sparse_header_t sparse_header;
	u32 i, outlen = 0;

	char src[32], dest[32], length[32];
	char *mmc_write[6]  = {"mmc", NULL, "write", NULL, NULL, NULL};
	mmc_write[1] = slot_no;
	mmc_write[3] = src;
	mmc_write[4] = dest;
	mmc_write[5] = length;

	printf("sparse: write to mmc slot[%s]\n", slot_no);

	/* Decode sparse header */
	sparse_header = *((sparse_header_t *)source);

	if (sparse_header.magic != SPARSE_HEADER_MAGIC) {
		printf("sparse: Bad Magic \n");
		return 1;
	}

	if (sparse_header.major_version != SPARSE_HEADER_MAJOR_VER) {
		printf("sparse: Unknown major number\n");
		return 1;
	}

	/* Skip the header now */
	source += sparse_header.file_hdr_sz;

	for (i=0; i<sparse_header.total_chunks; i++) {
		unsigned int len = 0;
		chunk_header_t chunk_header;

		chunk_header = *((chunk_header_t *)source);
		/* move to next chunk */
		source += sparse_header.chunk_hdr_sz;

		switch (chunk_header.chunk_type) {
			case CHUNK_TYPE_RAW:
			if (chunk_header.total_sz != (sparse_header.chunk_hdr_sz +
				  (chunk_header.chunk_sz * sparse_header.blk_sz)) ) {
				printf("sparse: bad chunk size for chunk %d, type Raw\n", i);
				return 1;
			}

			len = chunk_header.chunk_sz * sparse_header.blk_sz;

			outlen += len;
			sprintf(src, "0x%x", source);
			sprintf(dest, "0x%x", sector);
			sprintf(length, "0x%x", len);

#ifdef DEBUG
			printf("sparse: writing chunksz[%d] blk_sz[%d]\n", chunk_header.chunk_sz, sparse_header.blk_sz);
				printf("\n\t--->sparse: src-%s dest-%s lenght-%s\n", src, dest, length);
#endif
			if (do_mmc(NULL, 0, 6, mmc_write)) {
				printf("sparse: mmc Writing FAILED!\n");
				return 1;
			}

			sector += (len/512);
			source += len;
			break;

			case CHUNK_TYPE_DONT_CARE:
			if (chunk_header.total_sz != sparse_header.chunk_hdr_sz) {
				printf("sparse: bad chunk size for chunk %d,"
						" type Dont Care\n", i);
				return 1;
			}
			len = chunk_header.chunk_sz * sparse_header.blk_sz;
#ifdef DEBUG
			printf("sparse: writing chunksz[%d] blk_sz[%d]\n", chunk_header.chunk_sz, sparse_header.blk_sz);
				printf("\n\t -->sparse: skipping sector[0x%x] len[%d]\n", sector, len);
#endif

			outlen += len;
			sector += (len/512);
			break;

			default:
				printf("sparse: bad chunk, outlen(0x%x)\n", outlen);
				return 1;
			break;
		}/* chunk-type */

	}/* each chunk */

	printf("\nsparse: out-length-0x%d MB\n", outlen/(1024*1024));
	return 0;
}
