/*
 * ks8851.h
 *
 * Micrel KS8851
 *
 * Adapted from the vendor GPL code at:
 * ftp://www.micrel.com/ethernet/8851/
 *
 * Copyright (C) 2007-2008 Micrel, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 *
 */

#include <common.h>
#include <malloc.h>
#include <net.h>

#include "ks8851.h"
#define KS8851_SNL		1

/*
 * GET_KS8851SNL_RWWORD_CMD
 *
 * Description
 *  assemble a KS8851 SNL command for reading/write a WORD register value
 * Parameters
 *  u8 addr (IN), Address to read/write
 *  u8 act (IN), Action definition (ACTION_READ/ACTION_WRITE)
 *  u8 *pco (OUT), Pointer to first byte command.
 *  u8 *pc1 (OUT), Pointer to second byte command.
 *  int *pfReturn (OUT), Pointer to a int variable to indicate function
 *			sucessful (TRUE)/fail (FALSE)
 * Return (None):
 */

static void GET_KS8851SNL_RWWORD_CMD(u8 addr,
				u8 act,
				u8 *pc0,
				u8 *pc1,
				int *pfReturn)
{

	u8 bAlign, flag, Ac0 = 0, Ac1 = 0;
	int f = TRUE;

	if (act == ACTION_READ)
		flag = 0x00;
	else
		flag = 0x10;
	bAlign = (u8)addr & 0x03;
	if (bAlign == 0)
		Ac0 = (flag | 0x3) << 2;
	else if (bAlign == 1)
		Ac0 = (flag | 0x6) << 2;
	else if (bAlign == 2)
		Ac0 = (flag | 0xC) << 2;
	else
		f = FALSE;
	Ac1 = (u8)addr >> 6;
	Ac0 = (u8)Ac0 | (u8)Ac1;
	Ac1 = (u8)addr << 2;
	*pfReturn = f;
	*pc0 = Ac0;
	*pc1 = Ac1;
}

/*
 * KS8851SNL_REG_READ
 *
 * Description
 *  Read a register value from KS8851 SNL chip
 * Parameters
 *  u8 addr (IN), The register address.
 *  u16 * pwValue (OUT), Pointer a u16 to receive the register value.
 *  int *pfReturn (OUT), Pointer to a int variable to indicate function
 *			sucessful (TRUE)/fail (FALSE)
 * Return
 *  None
 */

static void KS8851SNL_REG_READ(u8 addr,
				u16 *pwValue,
				int *pfReturn)
{
	u8 c0 = 0, c1 = 0;
	u16 read_cmd;
	int fReturn = FALSE;

	GET_KS8851SNL_RWWORD_CMD(addr, ACTION_READ, &c0, &c1, &fReturn);
	read_cmd = (c0 << 8) | c1;

	/* SPI master driver to get value */
	*pwValue = spi_ks8851snl_reg_read(read_cmd);
}

/*
 * KS8851SNL_REG_WRITE
 *
 * Description
 *  Write a register value to KS8851 SNL chip
 * Parameters
 *  u8 addr (IN), The register address.
 *  u16 wValue (IN), Pointer a u16 which hold the register value to write.
 *  int *pfReturn (OUT), Pointer to a int variable to indicate function
 *			sucessful (TRUE)/fail (FALSE)
 * Return
 *  None
 */

void  KS8851SNL_REG_WRITE(u8 addr,
			u16 wValue,
			int *pfReturn)
{
	int fReturn;
	u8 c0 = 0, c1 = 0;

	GET_KS8851SNL_RWWORD_CMD(addr, ACTION_WRITE, &c0, &c1, &fReturn);
	if (fReturn) {
		u16 write_cmd;
		write_cmd = (c0 << 8) | c1;
		spi_ks8851snl_reg_write(write_cmd, wValue);
	}
}

/*
 * KS8851SNL_DATA_WRITE
 *
 * Write data to KS8851SNL slave device, before write should check the KS8851
 * TQ memory is avariable or not?
 *
 * Description
 *  Write a Ethernet data to KS8851 SNL chip
 * Parameters
 *  u8 *pBuf (IN), Pointer to data buffer.
 *  u16 wOrgDataLen, The data length;
 *  u16 wNewDataLen, The write length which around to 32 bit data alignment
 *  int *pfReturn (OUT), Pointer to a int variable to indicate function
 *			sucessful (TRUE)/fail (FALSE)
 * Return (None):
 */

void  KS8851SNL_DATA_WRITE(u8 *pBuf,
			u16 wOrgDataLen,
			u16 wNewDataLen,
			int *pfReturn)
{

	u8 cmd[5];
	cmd[0] = 0xC0;
	cmd[1] = 00;
	cmd[2] = 0x80;
	cmd[3] = (u8)(wOrgDataLen & 0x00FF);
	cmd[4] = (u8)(wOrgDataLen >> 8);
	*pfReturn = FALSE;
	if (spi_ks8851snl_data_write(&cmd[0], 5, TRUE, FALSE)) {
		if (spi_ks8851snl_data_write(pBuf, wNewDataLen, FALSE, TRUE))
			*pfReturn = TRUE;
	}
}

/*
 * KS8851SNL_DATA_READ
 *
 * Description
 *  Read the data from KS8851 SNL chip through KSZ8692 SPI Master driver
 *  because the KS8692 SPI master driver maximum read is 16 bytes each time,
 *  so the nDataLen in this functuion should be less 16 byte.
 * Parameters
 *  u8 *pBuf (IN), Pointer to data buffer to receive the data.
 *  int nDataLen, How mant data need to receive.
 *  u32 *pdwBytesTransferred, Pointer to a u32 to indicate how many
 *				bytes transferred.
 *  int *pfReturn (OUT), Pointer to a int variable to indicate function
 *			sucessful (TRUE)/fail (FALSE)
 * Return
 * None
 */

static void  KS8851SNL_DATA_READ(u8 *pBuf,
				int nDataLen,
				u32 *pdwBytesTransferred,
				int *pfReturn)
{
	*pdwBytesTransferred = spi_ks8851snl_data_read(0x80, 1, pBuf, nDataLen);
	if (*pdwBytesTransferred != nDataLen)
		*pfReturn = FALSE;
	else
		*pfReturn = TRUE;
}

/*
 * KS8851SNL_READ_BUFFER
 *
 * Description
 *  Read a Ethernet frame from KS8851 SNL chip.
 * Parameters
 *  void * pHardware (IN), Pointer the HARDWARE instance structure
 *  u8 *pBuf (IN), Pointer to data buffer to receive the data.
 *  u16 nDataLen, How mant data need to receive.
 *  u16 * puReadLen, Pointer to a u16 to indicate how many bytes
 *			actully read.
 * Return
 *  None
 */

void KS8851SNL_READ_BUFFER(void *pHardware,
			u8 *pIn,
			u8 **ppOut,
			u16 uDataLen,
			u16 *puReadLen)
{
	u16 length;
	u16 uLeftLen;
	int  fReturn;
	u32 dwBytesTransferred = 0;
	u16 w, spiBurstFlag, spiBurstLen;
	u32 *pdw;
	u8 *pMove = pIn;
	*ppOut = pIn;
	*puReadLen = 0;

	HW_READ_WORD(pHardware, REG_RX_CTRL2, &w);
	w &= 0xFF0F;

	/* now set the read burst rate according to user defined */
	spiBurstFlag = SPI_READ_BURST_ALL;

	/* get real read length */
	GET_DATA_ALIGNMENT(uDataLen, &length);

	switch (spiBurstFlag) {

	case SPI_READ_BURST_4BYTES:
		w |= 0x0000;
		spiBurstLen = 4;
		break;

	case SPI_READ_BURST_8BYTES:
		w |= 0x0020;
		spiBurstLen = 8;
		break;

	case SPI_READ_BURST_16BYTES:
		w |= 0x0040;
		spiBurstLen = 16;
		break;

	case SPI_READ_BURST_32BYTES:
		w |= 0x0060;
		spiBurstLen = 32;
		break;

	case SPI_READ_BURST_ALL:
		w |= 0x0080;
		spiBurstLen = length;
		break;

	default:
		*puReadLen = 0;
		printf("ks8851snl error: wrong spi read burst value\n");
		return;
	}

	HW_WRITE_WORD(pHardware, REG_RX_CTRL2, w);
	HW_READ_START(pHardware);

	/* each DMA transfer, there 4byte dummy + 4 byte header */
	if (spiBurstFlag == SPI_READ_BURST_4BYTES) {

		KS8851SNL_DATA_READ(pMove, spiBurstLen, &dwBytesTransferred,
					&fReturn);
		KS8851SNL_DATA_READ(pMove, spiBurstLen, &dwBytesTransferred,
					&fReturn);
		pdw = (u32 *)pMove;
		*puReadLen = (u16)((*pdw) >> 16);

	} else if (spiBurstFlag == SPI_READ_BURST_8BYTES) {

		KS8851SNL_DATA_READ(pMove, spiBurstLen, &dwBytesTransferred,
					&fReturn);
		pdw = (u32 *)pMove;
		pdw++;
		*puReadLen = (u16)((*pdw) >> 16);

	} else if (spiBurstFlag == SPI_READ_BURST_16BYTES) {

		KS8851SNL_DATA_READ(pMove, spiBurstLen, &dwBytesTransferred,
					&fReturn);
		pdw = (u32 *)pMove;
		pdw++;
		*puReadLen = (u16)((*pdw) >> 16);
		*ppOut = pIn + 8;
		pMove += 16;
		length -= 8;

	} else if (spiBurstFlag == SPI_READ_BURST_32BYTES) {
		KS8851SNL_DATA_READ(pMove, spiBurstLen, &dwBytesTransferred,
					&fReturn);
		pdw = (u32 *)pMove;
		pdw++;
		*puReadLen = (u16)((*pdw) >> 16);
		*ppOut = pIn + 8;
		pMove += 32;
		length -= 24;
	} else {

		/* spiBurstFlag==SPI_READ_BURST_ALL
		 * read length should add another 8 byte to include 4 byte
		 * dummy data and 4 byte header
		 */
		KS8851SNL_DATA_READ(pMove, spiBurstLen+8, &dwBytesTransferred,
					&fReturn);
		pdw = (u32 *)pMove;
		pdw++;
		*puReadLen = (u16)((*pdw) >> 16);
		*ppOut = pIn + 8;
	}

	if (spiBurstFlag != SPI_READ_BURST_ALL) {

		/*
		 * for a Ethernet packet, minimum length should bigger than
		 * 16 byte so the length=length/SPI_READ_BURST should not be
		 * zero
		 */
		uLeftLen = length%spiBurstLen;
		length = length/spiBurstLen;
		while (length--) {
			KS8851SNL_DATA_READ(pMove, spiBurstLen,
						&dwBytesTransferred, &fReturn);
			pMove += spiBurstLen;
		}

		if (uLeftLen) {
			/*
			 * last one, still should use same burst rate,
			 * otherwise error will happen
			 */
			KS8851SNL_DATA_READ(pMove, spiBurstLen,
					&dwBytesTransferred, &fReturn);
		}
	}

	HW_READ_END(pHardware);
}

/*
 * this function should not be called in a KS8851_SNL compiler flag
 * enabled. here only is a dummy function.
 */

void HW_WRITE_16BITDATA(void *phwi, u16 data)
{
	int fReturn;
	KS8851SNL_DATA_WRITE((u8 *)&data, 2, 2, &fReturn);
}

void HW_WRITE_32BITDATA(void *phwi, u32 data)
{
	int fReturn = FALSE;
	KS8851SNL_DATA_WRITE((u8 *)&data, 4, 4, &fReturn);
}


void HW_READ_16BITDATA(void *phwi, u16 *data)
{
	int fReturn;
	u32 dwBytesTransferred = 0;
	KS8851SNL_DATA_READ((u8 *)data, 2, &dwBytesTransferred, &fReturn);
}

void HW_READ_32BITDATA(void *phwi, u32 *data)
{
	int fReturn;
	u32 dwBytesTransferred = 0;
	KS8851SNL_DATA_READ((u8 *)data, 4, &dwBytesTransferred, &fReturn);
}

void HW_READ_WORD(void *pHardware, u16 addr, u16 *pw)
{
	int fReturn = FALSE;
	KS8851SNL_REG_READ((u8)addr, pw, &fReturn);
}


void HW_WRITE_WORD(void *pHardware, u16 addr, u16 w)
{
	int fReturn = FALSE;
	KS8851SNL_REG_WRITE((u8)addr, w, &fReturn);
}

/* network order */
u8 DEFAULT_MAC_ADDRESS[] = {0x08, 0x00, 0x28, 0x01, 0x4D, 0x4D};

#define DEVICE_NAME "ks8851snl"

static HARDWARE  gHardware;

/*
 * U-Boot ethernet interface
 */

void ks8851_eth_reset(void *pHardware, u16 type)
{
	/* Write 1 to active reset and wait */
	HW_WRITE_WORD(pHardware, REG_RESET_CTRL, type);
	udelay(500000);

	/* Write 0 to clear reset and wait */
	HW_WRITE_WORD(pHardware, REG_RESET_CTRL, 0);
	udelay(500000);
}

static void ks8851_eth_halt(struct eth_device *dev)
{
	return;
}

int ks8851_get_ethaddr (bd_t * bd)
{

	PHARDWARE pHardware = &gHardware;

	int ethaddr_env_size = 0;
	int ethaddr_env_set = 0;
	int ethaddr_rom_set = 0;

	int i;

	char *s = NULL;
	char *t = NULL;
	
	char ethaddr_tmp_data[64];
	char ethaddr_env_data[64];	
	
	char ethaddr_rom_data[6];
	char ethaddr_tmp[6];

	char *ethaddr_data, ethaddr_pattern[] = "00:00:00:00:00:00";

	/* Check if we have ethaddr set in environment */
	ethaddr_env_size = getenv_f("ethaddr", ethaddr_tmp_data,
					sizeof (ethaddr_tmp_data));
	if (ethaddr_env_size == sizeof(ethaddr_pattern)) {
		ethaddr_env_set = 1;
		s = ethaddr_tmp_data;
		for (i = 0; i < 6; ++i) {
			ethaddr_tmp[i] = s ? simple_strtoul (s, &t, 16) : 0;
			if (s)
				s = (*t) ? t + 1 : t;
		}
		ethaddr_data = (char *)ethaddr_tmp;
		sprintf(ethaddr_env_data, "%02X:%02X:%02X:%02X:%02X:%02X\n",
			ethaddr_tmp[0], ethaddr_tmp[1],
			ethaddr_tmp[2], ethaddr_tmp[3],
			ethaddr_tmp[4], ethaddr_tmp[5]);
	}

  /* Check if we have ethaddr set in ethernet rom memory */
	HW_READ_WORD(pHardware, REG_MAC_ADDR_4, (u16*) &ethaddr_rom_data[4]);
	HW_READ_WORD(pHardware, REG_MAC_ADDR_2, (u16*) &ethaddr_rom_data[2]);
	HW_READ_WORD(pHardware, REG_MAC_ADDR_0, (u16*) &ethaddr_rom_data[0]);
	
//	printf("ethaddr rom: %02X:%02X:%02X:%02X:%02X:%02X\n",
//			ethaddr_rom_data[5], ethaddr_rom_data[4],
//			ethaddr_rom_data[3], ethaddr_rom_data[2],
//			ethaddr_rom_data[1], ethaddr_rom_data[0]);

//	if (ethaddr_rom_data != sizeof(ethaddr_pattern)) {
	ethaddr_rom_set = 1;
//	}
	
	/* ethaddr, both environment & rom invalid */
	if (!ethaddr_env_set && !ethaddr_rom_set) {
		printf ("%s: ethaddr env & rom are not valid! cannot continue\n",
				DEVICE_NAME);
		return -1;

	/* ethaddr, environment invalid & rom valid */	
	} else if (!ethaddr_env_set && ethaddr_rom_set) {
//		printf ("%s: ethaddr env is not valid\n",
//				DEVICE_NAME);

		sprintf(ethaddr_tmp_data, "%02X:%02X:%02X:%02X:%02X:%02X",
			ethaddr_rom_data[5], ethaddr_rom_data[4],
			ethaddr_rom_data[3], ethaddr_rom_data[2],
			ethaddr_rom_data[1], ethaddr_rom_data[0]);

//		printf("seting environment from HW MAC addr = \"%s\"\n", ethaddr_tmp_data);
		setenv("ethaddr", ethaddr_tmp_data);

	/* ethaddr, environment valid & rom invalid */
 	} else if (ethaddr_env_set && !ethaddr_rom_set) {
		printf ("%s: ethaddr rom is not valid! using env ethaddr\n",
				DEVICE_NAME);
	}
	  	
	return 0;
}

static int readSingleFrame(FR_HEADER_INFO *prxFrameHeader)
{
	u16 wDataLen, w;
	u8 *pOut;
	PHARDWARE pHardware = &gHardware;
	pHardware->m_uFramesRemained--;
	pHardware->m_uCurFrameIndex++;

#ifdef DEBUG
	printf("ks8851recv: remained frames =%d\n",
				pHardware->m_uFramesRemained);
#endif

	if (prxFrameHeader->rxLength == 0) {
#ifdef DEBUG	
		printf("ks8851_recv error: the receive length is zero\n");
#endif		
		HW_READ_WORD(pHardware, REG_RXQ_CMD, &w);
		w |= RXQ_CMD_FREE_PACKET;
		HW_WRITE_WORD(pHardware, REG_RXQ_CMD, w);
#if 0
		HW_READ_START(pHardware);
		HW_READ_END(pHardware);
#endif
		return 0;
	}

	if ( (!(prxFrameHeader)->rxStatus) & (RX_VALID | RX_ICMP_ERROR | \
					RX_IP_ERROR | RX_TCP_ERROR | \
					RX_UDP_ERROR)) {
#ifdef DEBUG					
		printf("ks8851_recv error: received packet is invalid\n");
#endif		
		HW_READ_WORD(pHardware, REG_RXQ_CMD, &w);
		w |= RXQ_CMD_FREE_PACKET;
		HW_WRITE_WORD(pHardware, REG_RXQ_CMD, w);
#if 0
		HW_READ_START(pHardware);
		HW_READ_END(pHardware);
#endif
		return 0;
	}

	if (prxFrameHeader->rxLength > RX_BUF_SIZE) {
#ifdef DEBUG	
		printf("ks8851_recv error: the received packet \
			length=%d is bigger than buffer size=%d, \
			can not handle it\n",
			prxFrameHeader->rxLength,
			RX_BUF_SIZE);
#endif
		HW_READ_WORD(pHardware, REG_RXQ_CMD, &w);
		w |= RXQ_CMD_FREE_PACKET;
		HW_WRITE_WORD(pHardware, REG_RXQ_CMD, w);
		return 0;
	}

	KS8851SNL_READ_BUFFER(pHardware,
			pHardware->m_recvBuffer,
			&pOut,
			prxFrameHeader->rxLength,
			&wDataLen);
	NetReceive(pOut, prxFrameHeader->rxLength);

#ifdef DEBUG
	printf("\nrxLength=%d wDataLen=%d\n",
			prxFrameHeader->rxLength,
			wDataLen);
#endif

	return wDataLen;

}

static int ks8851_eth_recv(struct eth_device *dev)
{
	u16 uh, i, IntStatus;
	u32 dwDataLen = 0;
	PHARDWARE pHardware = &gHardware;
	u32 rxFrameCount;
	FR_HEADER_INFO *prxFrameHeader = NULL;

	/* Are there any frames pending to be processed? */
	if (pHardware->m_uFramesRemained > 0) {

		prxFrameHeader = pHardware->m_pRevHdrInfo;
		dwDataLen = readSingleFrame(
			(prxFrameHeader+pHardware->m_uCurFrameIndex));

	} else {

		/* check if we have more packets based on rx interrupt */
		HW_READ_WORD(pHardware, REG_INT_STATUS, &IntStatus);

		if (IntStatus == 0) {
#ifdef DEBUG
			printf("ks8851: no pending interrupt\n");
#endif
			return 0;
		}

		/* Acknowledge the rx interrupt */
		HW_WRITE_WORD(pHardware, REG_INT_STATUS, IntStatus);

		if (IntStatus & INT_RX) {

			/*
			 * Read current total amount of received
			 * frame count from RXFCTR
			 */

			HW_READ_WORD(pHardware, REG_RX_FRAME_CNT_THRES, &uh);
			rxFrameCount = (uh & 0xFF00) >> 8;

#ifdef DEBUG
			printf("frame count = %d\n", rxFrameCount);
#endif

			pHardware->m_uFramesRemained = rxFrameCount;

			if (rxFrameCount == 0)
				return 0;

			if (rxFrameCount >= pHardware->m_uCurRecvFrams) {

				free(pHardware->m_pRevHdrInfo);
				pHardware->m_uCurRecvFrams = rxFrameCount + 5;
				pHardware->m_pRevHdrInfo =
					(FR_HEADER_INFO *)malloc(
					sizeof(FR_HEADER_INFO)*(
					rxFrameCount + 5));
				if (!pHardware->m_pRevHdrInfo) {
					printf("ks8851_eth_recv: fail \
						to allocate memory\n");
					pHardware->m_uCurRecvFrams = 0;
					return 0;
				}
			}

			prxFrameHeader = pHardware->m_pRevHdrInfo;

			/* read all head information */
			for (i = 0; i < rxFrameCount; i++) {

				/* Checking Received packet status */
				HW_READ_WORD(pHardware,
					REG_RX_FHR_STATUS,
					&(prxFrameHeader+i)->rxStatus);

				/*
				 * Get received packet length from hardware
				 * packet memory
				 */
				HW_READ_WORD(pHardware,
					REG_RX_FHR_BYTE_CNT,
					&(prxFrameHeader+i)->rxLength);

				/*
				 * Record read packet length in 32 bit
				 * alignment
				 */
				(prxFrameHeader+i)->rxLength &=
							RX_BYTE_CNT_MASK;
			}

			/* read a single frame */
			dwDataLen = readSingleFrame(prxFrameHeader);

		} else {
#ifdef DEBUG
			printf("the interrupt is not a receiving\n");
#endif
		}
	}
	return dwDataLen;
}

/* the send function will send a complete ethernet frame */
static int ks8851_eth_send(struct eth_device *dev,
		volatile void *packet,
		int length)
{
	PHARDWARE pHardware = &gHardware;
	u32 uiPacketLength;
	int fReturn = FALSE;

	uiPacketLength = length;
	GET_DATA_ALIGNMENT(length, &uiPacketLength);
	HW_WRITE_START(pHardware);

#ifdef KS8851_SNL
	KS8851SNL_DATA_WRITE((u8 *)packet, length, uiPacketLength, &fReturn);
#else
{
	u8 *pOut = NULL;
	u16 uiPacketLength = legnth;
	HW_WRITE_DATA_HEADER(pHardware, ((u16 *)&uiPacketLength));
	HW_WRITE_DATA_BUFFER(pHardware,
				packet,
				&pOut,
				length,
				((u16 *)&uiPacketLength));
}
#endif

	HW_WRITE_END(pHardware);
	/*
	 * Issue TXQ Command (Enqueue Tx frame from TX buffer into
	 * TXQ for transmit)
	 */
	HW_WRITE_WORD(pHardware, REG_TXQ_CMD, TXQ_ENQUEUE);
	return 1;
}

static int ks8851_eth_init(struct eth_device *dev, bd_t *bd)
{

	u16 txCntl, rxCntl, w, intMask;
	PHARDWARE pHardware = &gHardware;

	pHardware->m_uCurFrameIndex = 0;
	pHardware->m_uFramesRemained = 0;
	pHardware->m_uCurRecvFrams = MAX_RECV_FRAMES;
	pHardware->m_pRevHdrInfo = (FR_HEADER_INFO *)malloc(
				sizeof(FR_HEADER_INFO)*MAX_RECV_FRAMES);

	/* Setup Transmit Frame Data Pointer Auto-Increment (TXFDPR) */
	HW_WRITE_WORD(pHardware, REG_TX_ADDR_PTR, ADDR_PTR_AUTO_INC);

	/* Enable QMU TxQ Auto-Enqueue frame */
	HW_WRITE_WORD(pHardware, REG_TXQ_CMD, TXQ_AUTO_ENQUEUE);

	/* Configure the QMU transmit module function */
	txCntl = (
		TX_CTRL_ICMP_CHECKSUM |
		/* TX_CTRL_UDP_CHECKSUM | */
		TX_CTRL_TCP_CHECKSUM |
		TX_CTRL_IP_CHECKSUM |
		TX_CTRL_FLOW_ENABLE |
		TX_CTRL_PAD_ENABLE |
		TX_CTRL_CRC_ENABLE
	);

	HW_WRITE_WORD(pHardware, REG_TX_CTRL, txCntl);

	/* Setup Receive Frame Data Pointer Auto-Increment */
	HW_WRITE_WORD(pHardware, REG_RX_ADDR_PTR, ADDR_PTR_AUTO_INC);

	/* Setup Receive Frame Threshold - 1 frame (RXFCTFC) */
	HW_WRITE_WORD(pHardware, REG_RX_FRAME_CNT_THRES,
			1 & RX_FRAME_THRESHOLD_MASK);

	/* Configure the receive function */
	rxCntl = (
		RX_CTRL_UDP_CHECKSUM |
		RX_CTRL_TCP_CHECKSUM |
		RX_CTRL_IP_CHECKSUM |
		RX_CTRL_MAC_FILTER |
		RX_CTRL_FLOW_ENABLE |
		RX_CTRL_BAD_PACKET |
		RX_CTRL_ALL_MULTICAST |
		RX_CTRL_UNICAST |
		RX_CTRL_PROMISCUOUS
	);

	HW_WRITE_WORD(pHardware, REG_RX_CTRL1, rxCntl);

	/* Setup RxQ Command Control (RXQCR) */
	HW_WRITE_WORD(pHardware, REG_RXQ_CMD, RXQ_CMD_CNTL);

	/* FixMe for now force 10BT half duplex */
	HW_WRITE_WORD(pHardware, REG_PORT_CTRL, 0x0040);
	HW_READ_WORD(pHardware, REG_PORT_STATUS, &txCntl);
	printf("port status information = 0x%x \n", txCntl);

	/* Clear the interrupts status */
	HW_WRITE_WORD(pHardware, REG_INT_STATUS, 0xFFFF);

	/* Enables QMU Transmit (TXCR). */
	HW_READ_WORD(pHardware, REG_TX_CTRL, &txCntl);
	txCntl |= TX_CTRL_ENABLE;
	HW_WRITE_WORD(pHardware, REG_TX_CTRL, txCntl);

	/* Enables QMU Receive (RXCR1). */
	HW_READ_WORD(pHardware, REG_RX_CTRL1, &rxCntl);
	rxCntl |= RX_CTRL_ENABLE;
	HW_WRITE_WORD(pHardware, REG_RX_CTRL1, rxCntl);

	intMask = (
		INT_PHY |
		INT_RX |
		INT_TX |
		INT_RX_OVERRUN |
		INT_TX_STOPPED |
		INT_RX_STOPPED |
#ifdef KS8851_SNL
		INT_RX_SPI_ERROR |
#endif
		INT_TX_SPACE
	);

	intMask = INT_RX;
	HW_WRITE_WORD(pHardware, REG_INT_MASK, intMask);

	return 0;

	/* RX Frame Count Threshold Enable and Auto-Dequeue RXQ Frame Enable */
	HW_READ_WORD(pHardware, REG_RXQ_CMD, &w);
	w |= RXQ_FRAME_CNT_INT;
	HW_WRITE_WORD(pHardware, REG_RXQ_CMD, w);

}

int ks8851_eth_initialize(bd_t *bd)
{
	struct eth_device *dev;
	PHARDWARE pHardware = &gHardware;
	u16 uRegData;

	if (!(dev = (struct eth_device *) malloc(sizeof *dev))) {
		printf("Failed to allocate memory\n");
		return 0;
	}
	
	spi_init();

	/* Reset eth device */
	ks8851_eth_reset(pHardware, GLOBAL_SOFTWARE_RESET);

	/* Read the device chip ID */
	HW_READ_WORD(pHardware, REG_CHIP_ID, &uRegData);
	if ((uRegData & 0xFFF0) != 0x8870) {
		printf("ks8851 check chip id fail, \
			readback value=0x%x \n", uRegData);
		return 0;
	}
	
	/* Verify if ethaddr from environment is set */
	ks8851_get_ethaddr(bd);

	memset(dev, 0, sizeof(*dev));
	sprintf(dev->name, "KS8851SNL");
	dev->init = ks8851_eth_init;
	dev->halt = ks8851_eth_halt;
	dev->send = ks8851_eth_send;
	dev->recv = ks8851_eth_recv;
	eth_register(dev);
	return 1;
}

void HW_WRITE_DATA_HEADER(void *phwi, u16 *puiPacketLength)
{
	u16 length;
	u32 dw = 0;

	/*
	 * When do the writing, the data length writed to chip's buffer
	 * should be 32 bit data alignment
	 */
	GET_DATA_ALIGNMENT(*puiPacketLength, &length);

	if (DATA_ALIGNMENT == 2) {
		HW_WRITE_16BITDATA(phwi, TX_CTRL_INTERRUPT_ON);
		/* data length in the head should be original data length */
		HW_WRITE_16BITDATA(phwi, *puiPacketLength);
	} else if (DATA_ALIGNMENT == 4) {
		dw = ((*puiPacketLength) << 16) | TX_CTRL_INTERRUPT_ON;
		HW_WRITE_32BITDATA(phwi, dw);
	}

	/* adjusted packet length return to the caller */
	*puiPacketLength = length;

	/*
`	 * at begining of each sending packet,
	 * we must clean the m_dwPrevData and m_uPrevByte
	 */
	((PHARDWARE)phwi)->m_dwPrevData = 0;
	((PHARDWARE)phwi)->m_uPrevByte = 0;
}

/*
 * Function name HW_WRITE_DATA_BUFFER
 *
 * Parameters
 *  phwi (IN), a void pointer to HARDWARE structure
 *  pbuf (IN), a u8 * pointer to input data buffer
 *  ppOut (OUT), a u8 * * pointer to u8 * variable to hold current data
 *		sending pointer.
 *  buflen (IN), data length in the data buffer
 *  pPacketLen, a u16 pointer to a u16 address which hold a current
 *		sending packets length
 *
 * Purpose
 *  When a protocol layer send a Ethernet pack to low level driver,
 *  the whole Ethernet packet may be broken into several small data buffers.
 *  Each data buffer may hold diffetent legnth of data. For the KS8851
 *  hardware, each QMU write should be 32 bit data alignment.
 *  Because each data buffer sent by protocal layer, the data length may
 *  not be round to 32 bit alignment.
 *  For example: A 101 bytes long Ethernet packet, may be broken to 3 data
 *  buffer to send, each length is  22, 22 and 57 byte long.
 *  If KS8851 hardware is 32 bit data bus, write 22 byte (which last write
 *  is 2 byte) will cause the data broken. So we need to save last 2 byte
 *  and added next buffers first 2 byte to complete a 32 bit write.
 *  The following function is for solving this problem.
 *
 * Note
 *  The m_uPrevByte is a u16 variable in HARDWARE structure to hold number
 *  of bytes which does not wrote to hardware at last function call.
 *  The m_dwPrevData is u32 variable in HARDWARE structure to hold the
 *  data which not wrote to hardware at last function call.
 *  The whole packet length hold in the  *pPacketLen should has been rounded
 *  to 32 bit alignment.
 *
 * DATA_ALIGNMENT in the 16 bit data bus is defined to 2
 * DATA_ALIGNMENT in the 32 bit data bus is defined to 4
 * HW_WRITE_16BITDATA is a extern funtion for write 16 bit data to hardware
 * HW_WRITE_32BITDATA is a extern funtion for write 32 bit data to hardware
 */

void HW_WRITE_DATA_BUFFER(
			void *phwi,
			u8 *pbuf,
			u8 **ppOut,
			u16 buflen,
			u16 *pPacketLen)
{

	/* first we check if there some data left before */
	int len;
	u8 *pByte, *p;
	*ppOut = pbuf;
	while (((PHARDWARE)phwi)->m_uPrevByte) {
		pByte = (u8 *)&((PHARDWARE)phwi)->m_dwPrevData;
		pByte += ((PHARDWARE)phwi)->m_uPrevByte;

		if ((buflen-(DATA_ALIGNMENT - ((
				PHARDWARE)phwi)->m_uPrevByte)) > 0) {
			memmove(pByte, pbuf,
			(DATA_ALIGNMENT-((PHARDWARE)phwi)->m_uPrevByte));
		} else {
			/* it should only happen at last sent portion */
			break;
		}

		if (DATA_ALIGNMENT == 2) {
			u16 w = (u16)((PHARDWARE)phwi)->m_dwPrevData;
			HW_WRITE_16BITDATA(phwi, w);
		} else if (DATA_ALIGNMENT == 4) {
			HW_WRITE_32BITDATA(phwi,
				((PHARDWARE)phwi)->m_dwPrevData);
		}

		/* RETAILMSG(TRUE, (TEXT("there is a previous data\n"))); */
		pbuf += (DATA_ALIGNMENT - ((PHARDWARE)phwi)->m_uPrevByte);
		*ppOut = (*ppOut) + (DATA_ALIGNMENT -
			((PHARDWARE)phwi)->m_uPrevByte);
		buflen -= (DATA_ALIGNMENT - ((PHARDWARE)phwi)->m_uPrevByte);
		*pPacketLen -= DATA_ALIGNMENT;
		((PHARDWARE)phwi)->m_uPrevByte = 0;
		break;
	}

	len = buflen/DATA_ALIGNMENT;

	if (len == 0) {

		/*
		 * it should only happen on the last data to send
		 * because we send a data by first change the original
		 * length to a DWORD alignment;
		 * at this case we can not use current buffer, because current
		 * buffer length may not include the DWORD alignment,
		 * for example current buffer length is 10, but DWORD
		 * alignment is 12. Now the buffer point to byte 8, so pass
		 * to this function's buflen=2. We need to send last data with
		 * DWORD alignment is 4 byte. But we can not use current buffer
		 *  pointer, because current buffer pointer + 4 will excess
		 * the valid buffer size, will cause system memory failure.
		 */

		if ((*pPacketLen/DATA_ALIGNMENT) > 0)
			len = (*pPacketLen / DATA_ALIGNMENT);
		else
			len = 1;

		memmove(&((PHARDWARE)phwi)->m_dwPrevData, pbuf, buflen);
		pbuf = (u8 *)&((PHARDWARE)phwi)->m_dwPrevData;
		*pPacketLen = 0;

	} else {

		*ppOut = (*ppOut) + (len * DATA_ALIGNMENT);
		((PHARDWARE)phwi)->m_uPrevByte = buflen % DATA_ALIGNMENT;
		*pPacketLen -= (len * DATA_ALIGNMENT);

	}

	if (DATA_ALIGNMENT == 2) {

		u16 *pw = (u16 *)pbuf;
		while (len--)
			HW_WRITE_16BITDATA(phwi, *pw++);
		p = (u8 *)pw;

	} else if (DATA_ALIGNMENT == 4) {

		u32 *pdw = (u32 *)pbuf;
		while (len--)
			HW_WRITE_32BITDATA(phwi, *pdw++);
		p = (u8 *)pdw;
	}

	pByte = (u8 *)&((PHARDWARE)phwi)->m_dwPrevData;
	if (((PHARDWARE)phwi)->m_uPrevByte)
		memmove(pByte, p, ((PHARDWARE)phwi)->m_uPrevByte);
}

/*
 * HW_READ_BUFFER
 *
 * Description
 *  This function read the data from the hardware
 * Parameters
 *  PHARDWARE pHardware (IN), Pointer to hardware instance
 *  u8 * data (IN), Pointer to a receive buffer
 *  u16 len (IN), the receving data length got from header before
 *  u16 * pRealLen (OUT), Actually received data length
 * Return
 *  None
 */

void HW_READ_BUFFER(
		void *phw,
		u8 *data,
		u16 len,
		u16 *pRealLen)
{

	u16 length;
	u16 *pwData;
	u32 *pdwData;
	HARDWARE *phwi = (HARDWARE *)phw;
	GET_DATA_ALIGNMENT(len, &length);
	HW_READ_START(phwi);

	length = length / DATA_ALIGNMENT;

	if (DATA_ALIGNMENT == 2) {
		pwData = (u16 *)data;
		/*
		 * for the 16 bit data bus, there are 2 byte
		 * dummy data at begining
		 */

		/* we need to skip them */
		HW_READ_16BITDATA(phwi, pwData);

		/* 4 byte header */
		HW_READ_16BITDATA(phwi, pwData);
		HW_READ_16BITDATA(phwi, pwData);
		*pRealLen = *pwData;

		/* now is valid data */
		while (length--)
			HW_READ_16BITDATA(phwi, pwData++);

	} else if (DATA_ALIGNMENT == 4) {

		pdwData = (u32 *)data;
		/*
		 * for the 32 bit data bus, there are 4 byte dummy data
		 * at begining
		 */

		/* we need to skip them */
		HW_READ_32BITDATA(phwi, pdwData);

		/* 4 byte header */
		HW_READ_32BITDATA(phwi, pdwData);
		*pRealLen = (u16)((*pdwData) >> 16);

		/* now is valid data */
		while (length--)
			HW_READ_32BITDATA(phwi, pdwData++);
	}
	HW_READ_END(phwi);
}

