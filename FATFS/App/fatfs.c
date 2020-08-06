/**
  ******************************************************************************
  * @file   fatfs.c
  * @brief  Code for fatfs applications
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

#include "fatfs.h"

uint8_t retSD;    /* Return value for SD */
char SDPath[4];   /* SD logical drive path */
FATFS SDFatFS;    /* File system object for SD logical drive */
FIL SDFile;       /* File object for SD */

/* USER CODE BEGIN Variables */
FIL LogFile;

int FATFS_errHandle_(FRESULT ret);
/* USER CODE END Variables */

void MX_FATFS_Init(void)
{
  /*## FatFS: Link the SD driver ###########################*/
  retSD = FATFS_LinkDriver(&SD_Driver, SDPath);

  /* USER CODE BEGIN Init */
  /* additional user code for init */
  TTY_registerCommand("free", &FATFS_free);
  FRESULT ret = f_mount(&SDFatFS, SDPath, 1);
  if (ret == FR_OK) {
    // Open all files
    ret = f_open(&LogFile, "tmp.log", FA_WRITE|FA_CREATE_ALWAYS);
    if (ret != FR_OK) {
      //TODO: HANDLE FAIL TO OPEN
      FATFS_errHandle_(ret);
    }
  }
  else {
    //TODO: HANDLE FAILED MOUNT!
    FATFS_errHandle_(ret);
  }
  /* USER CODE END Init */
}

/**
  * @brief  Gets Time from RTC
  * @param  None
  * @retval Time in DWORD
  */
DWORD get_fattime(void)
{
  /* USER CODE BEGIN get_fattime */
  return 0;
  /* USER CODE END get_fattime */
}

/* USER CODE BEGIN Application */
int FATFS_free(__unused int argc, __unused char *argv[])
{
  DWORD freClstr; FATFS *fs;
  FRESULT ret = f_getfree(SDPath, &freClstr, &fs);
  if (ret == FR_OK) {
    DWORD totSect = (fs->n_fatent - 2) * fs->csize;
    DWORD freSect = freClstr * fs->csize;

    TTY_printf("%10lu MiB total drive space.\r\n", totSect / 2 / 1024);
    TTY_printf("%10lu MiB available.\r\n", freSect / 2 / 1024);
  }
  else {
    FATFS_errHandle_(ret);
  }
  return 1;
}

int FATFS_errHandle_(__unused FRESULT ret)
{
  //TODO: Implement
  for(;;);
  return -1;
}

/* LOG CODE START Application */
int LOG_write(uint8_t *buf, size_t len)
{
  UINT bytesWritten; FRESULT ret;

  ret = f_write(&LogFile, buf, len, &bytesWritten);

  if (ret != FR_OK) return FATFS_errHandle_(ret);
  else return bytesWritten;
}

int LOG_flush()
{
  FRESULT ret;

  ret = f_sync(&LogFile);
  if(ret != FR_OK) return FATFS_errHandle_(ret);
  else return 1;
}
/* LOG CODE END Application */
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
