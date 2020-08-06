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
int FATFS_errHandle_(FRESULT ret);
/* USER CODE END Variables */

void MX_FATFS_Init(void)
{
  /*## FatFS: Link the SD driver ###########################*/
  retSD = FATFS_LinkDriver(&SD_Driver, SDPath);

  /* USER CODE BEGIN Init */
  /* additional user code for init */
  TTY_registerCommand("free", &FATFS_free);
  TTY_registerCommand("mount", &FATFS_mount);

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

int FATFS_mount(__unused int argc, __unused char *argv[])
{
  FRESULT  ret = f_mount(&SDFatFS, SDPath, 1);
  if (ret == FR_OK) {
    TTY_println("SD Card mounted successfully");
    return 1;
  }
  else {
    TTY_println("Failed to mount SD Card");
    return FATFS_errHandle_(ret);
  }
}

int FATFS_errHandle_(__unused FRESULT ret)
{
  return 1;
}


/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
