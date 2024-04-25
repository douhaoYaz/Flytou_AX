/**********************************************************************************
 *
 * Copyright (c) 2019-2020 Beijing AXera Technology Co., Ltd. All Rights Reserved.
 *
 * This source file is the property of Beijing AXera Technology Co., Ltd. and
 * may not be copied or distributed in any isomorphic form without the prior
 * written consent of Beijing AXera Technology Co., Ltd.
 *
 **********************************************************************************/
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <fcntl.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include <stdint.h>
#include <getopt.h>


#include "spi.h"

AX_U8  mode;
AX_U8  bits;
AX_U32 speed;
AX_U8 lsb;

#define LITTLE_ENDIAN_BITFIELD
static void bit_order(AX_U8 *byte)
{

    AX_U8 byte1 = 0x0;

    AX_U8 bit0 = (*byte) & 0x01;
    AX_U8 bit1 = (*byte) & 0x02;
    AX_U8 bit2 = (*byte) & 0x04;
    AX_U8 bit3 = (*byte) & 0x08;
    AX_U8 bit4 = (*byte) & 0x10;
    AX_U8 bit5 = (*byte) & 0x20;
    AX_U8 bit6 = (*byte) & 0x40;
    AX_U8 bit7 = (*byte) & 0x80;

    // printf("%s byte =0x%x\r\n", __func__, *byte);
    byte1 = bit0 << 7 | bit1 << 5 | bit2 << 3 | bit3 << 1 | bit4 >> 1 | bit5 >> 3 | bit6 >> 5 | bit7 >> 7;
    //printf("%s ptr =0x%x\r\n", __func__, byte1);

    *byte = (AX_U8)byte1;
}

AX_U8 SPI_test(AX_S32 fd)
{
    AX_S32 ret;
    AX_U8 readCmd[1];
    AX_U16 spiStateReg[1];
    AX_U16 receiveData[1];
    struct spi_ioc_transfer xfer[2];

    readCmd[0] = 0x03;
    spiStateReg[0] = 0x55aa;
    printf("%s, spiStateReg 0x%x\r\n", __func__, spiStateReg[0]);
    memset(xfer, 0, 2 * sizeof(struct spi_ioc_transfer));

    xfer[0].tx_buf = (unsigned long)readCmd;
    xfer[0].len = 1;


    xfer[1].tx_buf = (unsigned long)spiStateReg;
    xfer[1].rx_buf = (unsigned long)receiveData;
    xfer[1].len = 2;

    ret = ioctl(fd, SPI_IOC_MESSAGE(2), &xfer);
    if (ret < 1)
        printf("%s, can't xfer spi message1\r\n", __func__);

    printf("receiveData:0x%x \r\n", receiveData[0]);

    return receiveData[0];
}

#if 0
AX_U8 SPI_ReadByte_LSB(AX_S32 fd, AX_U32 readAddress)
{
    AX_S32 ret;
    AX_U8 readCmd[3];
    AX_U8 spiStateReg[1];
    AX_U8 receiveData[1];
    struct spi_ioc_transfer xfer[1];

    readCmd[0] = 0xFF;
    readCmd[1] = (AX_U8)(readAddress >> 8);
    readCmd[2] = (AX_U8)(readAddress >> 0);
    //spiStateReg[0] = (AX_U16)readAddress;
    printf("%s, readAddress 0x%x\r\n", __func__, readAddress);
    printf("%s, cmd 0x%x 0x%x 0x%x\r\n", __func__, readCmd[0], readCmd[1], readCmd[2]);
    memset(xfer, 0, 1 * sizeof(struct spi_ioc_transfer));

    xfer[0].tx_buf = (AX_U64)readCmd;
    xfer[0].rx_buf = (AX_U64)receiveData;
    xfer[0].bits_per_word = 8;
    xfer[0].len = 4;
#if 0
    xfer[1].tx_buf = 0;
    xfer[1].bits_per_word = 8;
    xfer[1].len = 1;
    xfer[1].rx_buf = (AX_U64)receiveData;
    //xfer[1].cs_change = 0;
#endif
    ret = ioctl(fd, SPI_IOC_MESSAGE(1), &xfer);
    if (ret < 1)
        printf("%s, can't xfer spi message1\r\n", __func__);

    printf("receiveData:0x%x ;\r\n", receiveData[0]);

    return receiveData[0];
}

#else
AX_U8 SPI_ReadByte_LSB(AX_S32 fd, AX_U32 readAddress)
{
    AX_S32 ret;
    AX_U8 readCmd[1];
    AX_U8 spiStateReg[2];
    AX_U8 receiveData[1];
    struct spi_ioc_transfer xfer[3];

    readCmd[0] = 0xff;
    spiStateReg[0] = (AX_U8)(readAddress >> 0);
    spiStateReg[1] = (AX_U8)(readAddress >> 8);
    bit_order(&spiStateReg[0]);
    bit_order(&spiStateReg[1]);
    //spiStateReg[0] = (AX_U16)readAddress;
    printf("%s, readAddress 0x%x\r\n", __func__, readAddress);
    printf("%s, cmd 0x%x \r\n", __func__, readCmd[0]);
    printf("%s, spiStateReg 0x%x, 0x%x \r\n", __func__, spiStateReg[0],  spiStateReg[1]);
    memset(xfer, 0, 3 * sizeof(struct spi_ioc_transfer));

    xfer[0].tx_buf = (AX_ULONG)readCmd;
    xfer[0].bits_per_word = 8;
    xfer[0].len = 1;

    xfer[1].tx_buf = (AX_ULONG)spiStateReg;
    //xfer[1].rx_buf = (AX_U64)receiveData;
    xfer[1].bits_per_word = 8;
    xfer[1].len = 2;

    xfer[2].rx_buf = (AX_ULONG)receiveData;
    xfer[2].bits_per_word = 8;
    xfer[2].len = 1;

    ret = ioctl(fd, SPI_IOC_MESSAGE(3), &xfer);
    if (ret < 1)
        printf("%s, can't xfer spi message1\r\n", __func__);

    bit_order(&receiveData[0]);
    printf("receiveData:0x%x ;\r\n", receiveData[0]);

    return receiveData[0];
}
#endif

AX_S32 SPI_WriteByte_LSB(AX_S32 fd, AX_U32 writeAddress, AX_U8 writedata)
{
    AX_S32 ret;
    AX_U8 writeCmd[1];
    AX_U16 spiStateReg[1];
    AX_U8 rev[1];
    struct spi_ioc_transfer xfer[3];

    writeCmd[0] = 0x03;
    spiStateReg[0] = (AX_U16)writeAddress;
    rev[0] = writedata;
    printf("%s, writeAddress 0x%x\r\n", __func__, writeAddress);
    memset(xfer, 0, 3 * sizeof(struct spi_ioc_transfer));

    xfer[0].tx_buf = (AX_ULONG)writeCmd;
    xfer[0].len = 1;

    xfer[1].tx_buf = (AX_ULONG)spiStateReg;
    xfer[1].len = 1;

    xfer[2].tx_buf = (AX_ULONG)rev;
    xfer[2].len = 1;


    ret = ioctl(fd, SPI_IOC_MESSAGE(3), &xfer);
    if (ret < 1)
        printf("%s, can't xfer spi message1\r\n", __func__);

    return 0;
}

AX_S32 SPI_WriteBytes_LSB(AX_S32 fd, AX_U32 reg, AX_U8 *data, AX_U8 length)
{
    AX_S32 ret;
    AX_U8 writeCmd[1];
    AX_U16 spiStateReg[1];
    AX_U8 receiveData[1];
    struct spi_ioc_transfer xfer[3];

    writeCmd[0] = 0x02;
    spiStateReg[0] = (AX_U16)reg;
    memset(xfer, 0, 3 * sizeof(struct spi_ioc_transfer));
    printf("%s, 0x%x\r\n", __func__, reg);
    xfer[0].tx_buf = (unsigned long)writeCmd;
    xfer[0].len = 1;

    xfer[1].tx_buf = (unsigned long)spiStateReg;
    xfer[1].len = 2;

    xfer[2].tx_buf = (unsigned long)data;
    xfer[2].len = length;

    ret = ioctl(fd, SPI_IOC_MESSAGE(3), &xfer);
    if (ret < 1)
        printf("%s, can't xfer spi message1\r\n", __func__);

    return 0;
}

AX_S32 SPI_ReadBytes_LSB(AX_S32 fd, AX_U32 reg, AX_U8 *data, AX_U8 length)
{
    AX_S32 ret, i;
    AX_U8 readCmd[1];
    AX_U16 spiStateReg[1];
    AX_U8 receiveData[1];

    struct spi_ioc_transfer xfer[3];

    readCmd[0] = 0x03;
    spiStateReg[0] = (AX_U16)reg;
    memset(xfer, 0, 3 * sizeof(struct spi_ioc_transfer));
    printf("%s, 0x%x\r\n", __func__, reg);
    printf("%s, spiStateReg[0] 0x%x\r\n", __func__, spiStateReg[0]);
    xfer[0].tx_buf = (unsigned long)readCmd;
    xfer[0].rx_buf = 0;
    xfer[0].len = 1;

    xfer[1].tx_buf = (unsigned long)spiStateReg;
    xfer[1].rx_buf = 0;
    xfer[1].len = 2;
    //xfer[1].bits_per_word=8;

    xfer[2].rx_buf = (unsigned long)receiveData;
    xfer[2].tx_buf = 0;
    xfer[2].len = length;

    ret = ioctl(fd, SPI_IOC_MESSAGE(3), &xfer);
    if (ret < 1)
        printf("%s, can't xfer spi message1 \r\n", __func__);

    for (i = length; i > 0; i--) {
        printf("%s, rxbuf[%d] = 0x%02x \r\n", __func__, (i - 1), receiveData[i - 1]);
    }

    return 0;
}

AX_S32 SPI_write(AX_S32 spi_fd, AX_U8 reg, AX_U16 data)
{
    AX_S32 ret;
    AX_U8 spiReg[1];
    AX_U8 spiData0[1];
    AX_U32 spiData1[1];
    AX_U8 receiveData[2];
    struct spi_ioc_transfer xfer[3];
    unsigned int uiValue = 0;
    int i = 0;
    for(i=0;i<8;i++)
    {
        uiValue = (uiValue << 1)+(reg & 0x01);
        reg = reg >> 1;
    }

    spiReg[0] = (AX_U8)uiValue;
    uiValue = 0;
    for(i=0;i<16;i++)
    {
        uiValue = (uiValue << 1)+(data & 0x01);
        data = data >> 1;
    }

    data = uiValue;
    AX_U32 tmp = data & 0x00ff;
    data = (tmp << 8) | ((data & 0xff00) >> 8);

    spiData1[0] = spiReg[0] | data << 8;
    memset(xfer, 0, sizeof(xfer));
    //printf("%s, reg 0x%x data0 0x%llx \n", __func__, spiReg[0],spiData1[0]);

    xfer[0].tx_buf = (unsigned long)spiData1;
    xfer[0].len = 3;
    ret = ioctl(spi_fd, SPI_IOC_MESSAGE(1), &xfer);
    if (ret < 1)
        printf("%s, can't xfer spi message1\r\n", __func__);

    return 0;
}

AX_S32 SPI_init(AX_U8 bus_num, AX_U8 cs)
{
    AX_S32 ret;
    AX_S32 spi_fd = -1;
    char bus_index[8];
    char spi_cs[8];
    char device_name[64];

    sprintf(bus_index, "%d", bus_num);
    sprintf(spi_cs, "%d", cs);
    sprintf(device_name, "/dev/spidev%s.%s", bus_index, spi_cs);

    printf("%s: spi device is %s \r\n", __func__, device_name);

    spi_fd = open(device_name, O_RDWR);
    if (spi_fd < 0) {
        printf("Open /dev/%s failed!!!\r\n", device_name);
        return -1;
    }

    /*
     * spi mode
     */
    ret = ioctl(spi_fd, SPI_IOC_RD_MODE, &mode);
    if (ret == -1)
        printf("%s: can't get spi mode \r\n", __func__);
    printf("%s: set spi mode %d\r\n", __func__, mode);
    mode = 0x3;
    ret = ioctl(spi_fd, SPI_IOC_WR_MODE, &mode);
    if (ret == -1)
        printf("%s: can't set spi mode \r\n", __func__);

    /*
     * bits per word
     */
    ret = ioctl(spi_fd, SPI_IOC_RD_BITS_PER_WORD, &bits);
    if (ret == -1)
        printf("%s: can't get bits per word \r\n", __func__);
    printf("%s: set spi bits %d\r\n", __func__, bits);
    ret = ioctl(spi_fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
    if (ret == -1)
        printf("%s: can't set bits per word \r\n", __func__);

    /*
     * max speed hz
     */
    ret = ioctl(spi_fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed);
    if (ret == -1)
        printf("%s: can't get max speed hz \r\n", __func__);
    printf("%s: set spi speed %d\r\n", __func__, speed);
    speed = 1000000;
    ret = ioctl(spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
    if (ret == -1)
        printf("%s: can't set max speed hz \r\n", __func__);

    /*
     * lsb fisrt
     */
    ret = ioctl(spi_fd, SPI_IOC_RD_LSB_FIRST, &lsb);
    if (ret == -1)
        printf("%s: can't s get lsb hz \r\n", __func__);
    printf("%s: set spi lsb %d\r\n", __func__, lsb);
    lsb = 0x8;
    ret = ioctl(spi_fd, SPI_IOC_WR_LSB_FIRST, &lsb);
    if (ret == -1)
        printf("%s: can't set lsb %d \r\n", __func__, ret);

    printf("Open spi dev 0x%x \r\n", spi_fd);

    return spi_fd;
}
