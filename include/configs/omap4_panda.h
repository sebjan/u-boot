/*
 * (C) Copyright 2010
 * Texas Instruments Incorporated.
 * Steve Sakoman  <steve@sakoman.com>
 *
 * Configuration settings for the TI OMAP4 Panda board.
 * See omap4_common.h for OMAP4 common part
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#ifndef __CONFIG_PANDA_H
#define __CONFIG_PANDA_H

/*
 * High Level Configuration Options
 */
#define CONFIG_PANDA		1	/* working with Panda */

/* USB Host networking options */
#define CONFIG_CMD_USB			1
#define CONFIG_USB_STORAGE		1
#define CONFIG_USB_HOST_ETHER		1
#define CONFIG_USB_EHCI			1
#define CONFIG_USB_EHCI_OMAP4		1
#define CONFIG_USB_HOST			1
#define CONFIG_USB_ETHER_SMSC95XX	1

/* BOOTP options */
#define CONFIG_BOOTP_BOOTFILESIZE
#define CONFIG_BOOTP_BOOTPATH
#define CONFIG_BOOTP_GATEWAY
#define CONFIG_BOOTP_HOSTNAME

/* PXE */
#define CONFIG_CMD_PXE
#define CONFIG_MENU
#define CONFIG_BOOTP_PXE
#define CONFIG_BOOTP_PXE_CLIENTARCH	0x100
#define CONFIG_BOOTP_VCI_STRING		"U-boot.armv7.omap4_panda"


/* Ethernet */
#define CONFIG_NET_MULTI		1
#define CONFIG_CMD_PING			1
#define CONFIG_CMD_DHCP			1

#include <configs/omap4_common.h>
#define CONFIG_CMD_NET

/* ENV related config options */
#define CONFIG_ENV_IS_NOWHERE

#define CONFIG_SYS_PROMPT		"Panda # "

#endif /* __CONFIG_PANDA_H */
