/**********************************************************************************
 *
 * Copyright (c) 2019-2020 Beijing AXera Technology Co., Ltd. All Rights Reserved.
 *
 * This source file is the property of Beijing AXera Technology Co., Ltd. and
 * may not be copied or distributed in any isomorphic form without the prior
 * written consent of Beijing AXera Technology Co., Ltd.
 *
 **********************************************************************************/

#ifndef __SPI_H__
#define __SPI_H__

#include "ax_base_type.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define SPI_NR_DAT_BYTES_1 (1)
#define SPI_NR_DAT_BYTES_2 (2)
#define SPI_NR_DAT_BYTES_4 (4)

typedef AX_S32 SpiReturnType;

enum SPI_RESULT {
    SPI_RET_SUCCESS = 0,  /**< The operation was successfully completed */
    SPI_RET_FAILURE = -1, /**< Generic error */
    SPI_RET_INVALID_PARM = -2,
};

extern AX_S32 g_spi_fd;

AX_U8 SPI_ReadByte_LSB(AX_S32 fd, AX_U32 readAddress);
AX_S32 SPI_WriteByte_LSB(AX_S32 fd, AX_U32 writeAddress, AX_U8 writedata);
AX_S32 SPI_WriteBytes_LSB(AX_S32 fd, AX_U32 reg, AX_U8 *data, AX_U8 length);
AX_S32 SPI_ReadBytes_LSB(AX_S32 fd, AX_U32 reg, AX_U8 *data, AX_U8 length);
AX_S32 SPI_init(AX_U8 bus_num, AX_U8 cs);
AX_S32 SPI_write(AX_S32 spi_fd, AX_U8 reg, AX_U16 data);

#ifdef __cplusplus
}
#endif

#endif /* __SPI_H__ */
