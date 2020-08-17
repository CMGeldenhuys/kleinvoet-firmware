/**
  ******************************************************************************
  * @file   fatfs.h
  * @brief  Header for fatfs applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __fatfs_H
#define __fatfs_H
#ifdef __cplusplus
extern "C" {
#endif

#include "ff.h"
#include "ff_gen_drv.h"
#include "sd_diskio.h" /* defines SD_Driver as external */

/* USER CODE BEGIN Includes */
#include "tty.h"
#include "logger.h"

#define FATFS_EOL SERIAL_EOL
/* USER CODE END Includes */

extern uint8_t retSD; /* Return value for SD */
extern char SDPath[4]; /* SD logical drive path */
extern FATFS SDFatFS; /* File system object for SD logical drive */
extern FIL SDFile; /* File object for SD */

void MX_FATFS_Init (void);

/* USER CODE BEGIN Prototypes */
int FATFS_pfree (uint32_t *free, uint32_t *total);

int FATFS_free();

int CMD_free (__unused int argc, __unused char *argv[]);

int FATFS_open (FIL *fp, const TCHAR *path, BYTE mode);

#define FATFS_write(fp, buff, len) FATFS_slwrite((fp), (buff), (len), 0, -1)

#define FATFS_swrite(fp, buff, len, sync) FATFS_slwrite((fp), (buff), (len), (sync), -1)

#define FATFS_lwrite(fp, buff, len, pos) FATFS_slwrite((fp), (buff), (len), 0, (pos))

int FATFS_slwrite (FIL *fp, const void *buff, size_t len, int sync, int pos);

int FATFS_close (FIL *fp);

int FATFS_mount ();
/* USER CODE END Prototypes */
#ifdef __cplusplus
}
#endif
#endif /*__fatfs_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
