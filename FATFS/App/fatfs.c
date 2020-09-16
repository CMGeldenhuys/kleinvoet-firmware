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
FIL                      *LogFile;
extern RTC_HandleTypeDef hrtc;

FIL *files[_FS_LOCK] = {0};

int FATFS_errHandle_ (FRESULT res);

/* USER CODE END Variables */

void MX_FATFS_Init(void)
{
  /*## FatFS: Link the SD driver ###########################*/
  retSD = FATFS_LinkDriver(&SD_Driver, SDPath);

  /* USER CODE BEGIN Init */
  /* additional user code for init */
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
  DWORD           dt;
  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
  HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

  // Assume 2000's
  dt = ((unsigned) (2000 + sDate.Year - 1980) & 0x7FU) << 25U
       | ((unsigned) (sDate.Month) & 0x0FU) << 21U
       | ((unsigned) (sDate.Date) & 0x1FU) << 16U
       | ((unsigned) (sTime.Hours) & 0x1FU) << 11U
       | ((unsigned) (sTime.Minutes) & 0x3FU) << 5U
       | ((unsigned) (sTime.Seconds >> 1U) & 0x1FU) << 0U;


  return dt;
  /* USER CODE END get_fattime */
}

/* USER CODE BEGIN Application */
int FATFS_mount ()
{
  // NB: CANT CALL LOGGING YET AS IT HAS NOT BEEN INIT.
  TTY_registerCommand("free", &CMD_free);
  TTY_registerCommand("sync", &CMD_sync);
  TTY_registerCommand("cat", &CMD_cat);

  // Mount to check if FS is working
  FRESULT ret = f_mount(&SDFatFS, SDPath, 1);
  if (ret == FR_OK) {
    FILINFO fno;
    char    path[sizeof("REC_000")] = "DEBUG";
#ifndef DEBUG
    // TODO: Find a better way but this is just a quick fix
    // Look for folders across 0...999
    for (uint16_t idx = 0; idx <= 1000; idx++) {
      if (idx == 1000) {
        DBUG("Ran out of folder names");
        return -2;
      }
      // TODO: Rather check if recording exists to stop many folder problem
      snprintf(path, sizeof(path), "REC_%03u", idx);
      ret = f_stat(path, &fno);

      // Check for folders and files
      if (ret == FR_OK) {
        DBUG("Folder %s exists, trying next...", path);
        continue;
      }
      else if (ret == FR_NO_FILE) {
        DBUG("Folder '%s' doesn't exist", path);
        break;
      }
      else {
        return FATFS_errHandle_(ret);
      }
    }

    // Create folder and open dir
    ret = f_mkdir(path);
    if (ret != FR_OK) return FATFS_errHandle_(ret);
#else
    // Check if debug dir exists
    ret = f_stat(path, &fno);
    if (ret != FR_OK) {
      // If not create it
      ret = f_mkdir(path);
      if (ret != FR_OK) return FATFS_errHandle_(ret);
    }
#endif

    // Change directory
    ret = f_chdir(path);
    if (ret != FR_OK) return FATFS_errHandle_(ret);

#ifndef DEBUG
    // Open all files
    // TODO: Move log handling out to own file
    LogFile = FATFS_malloc(1);
    ret     = f_open(LogFile, "KV.LOG", FA_WRITE | FA_CREATE_ALWAYS);

    if (ret != FR_OK) {
      //TODO: HANDLE FAIL TO OPEN
      return FATFS_errHandle_(ret);
    }
#endif

  }
  else {
    //TODO: HANDLE FAILED MOUNT!
    return FATFS_errHandle_(ret);
  }

  return 1;
}

int FATFS_pfree (uint32_t *free, uint32_t *total)
{
  DWORD   freClstr;
  FATFS   *fs;
  FRESULT ret = f_getfree(SDPath, &freClstr, &fs);
  if (ret == FR_OK) {
    if (total) *total = (fs->n_fatent - 2) * fs->csize / 2;
    *free             = freClstr * fs->csize / 2;

    return *free;
  }
  else {
    return FATFS_errHandle_(ret);
  }
}

int FATFS_free ()
{
  uint32_t free;
  return FATFS_pfree(&free, NULL);
}

int CMD_free (__unused int argc, __unused char *argv[])
{
  uint32_t free, tot;

  if (FATFS_pfree(&free, &tot) > 0) {
    TTY_printf("%10u MiB total drive space%s", tot / 1024, SERIAL_EOL);
    TTY_printf("%10u MiB free drive space%s", free / 1024, SERIAL_EOL);
    return 1;
  }
  else {
    TTY_println("Failed to get free space");
    return 0;
  }
}


int FATFS_open (FIL *fp, const TCHAR *path, BYTE mode)
{
  FRESULT res = f_open(fp, path, mode);
  if (res == FR_OK) {
    DBUG("Successfully opened '%s'", path);
    return 1;
  }
  else {
    WARN("Failed to open '%s'", path);
    return FATFS_errHandle_(res);
  }
}


int FATFS_slwrite (FIL *fp, const void *buff, size_t len, int sync, int pos)
{
  UINT    bw;
  FRESULT res;
  FSIZE_t tail;

  // Move file pointer tail pos
  if (pos >= 0) {
    DBUG("Storing current file pointer tail");
    tail = f_tell(fp);
    DBUG("Setting tail to %u", pos);
    res = f_lseek(fp, pos);
    if (res != FR_OK) ERR("Failed to move file pointer tail");
  }


  res = f_write(fp, buff, len, &bw);

  if (res == FR_OK) {
    if (sync > 0) {
      DBUG("Wrote %u bytes to file (SAFE)", bw);
      res = f_sync(fp);
      if (res != FR_OK) return FATFS_errHandle_(res);
    }
    else {
      DBUG("Wrote %u bytes to file (UNSAFE)", bw);
    }
  }
  else
    return FATFS_errHandle_(res);

  // Restore file pointer tail
  if (pos >= 0) {
    if (tail) {
      DBUG("Restoring file pointer to tail (%u)", tail);
      res = f_lseek(fp, tail);
      if (res != FR_OK) ERR("Failed to restore file pointer tail");
    }
    else {
      DBUG("Not restoring fp since first write");
    }
  }

  return bw;
}

int FATFS_close (FIL *fp)
{
  FRESULT res;

  res = f_close(fp);

  if (res == FR_OK) {
    DBUG("File closed successfully");
    return 1;
  }
  else
    return FATFS_errHandle_(res);

}

int FATFS_errHandle_ (FRESULT res)
{
#ifdef DEBUG
  switch (res) {

    case FR_OK: {
      DBUG("Error handler called on OK value");
      break;
    }
    case FR_DISK_ERR:
    case FR_INT_ERR: {
      ERR("Disk Error!");
      break;
    }

    case FR_NOT_READY: {
      ERR("FS not ready!");
      break;
    }

    case FR_INVALID_OBJECT:
    case FR_NO_PATH:
    case FR_NO_FILE:
    case FR_INVALID_NAME: {
      WARN("No such file/path/obj");
      break;
    }

    case FR_LOCKED:
    case FR_WRITE_PROTECTED:
    case FR_DENIED: {
      WARN("Access denied!");
      break;
    }

    case FR_EXIST: {
      INFO("File exists");
      break;
    }

    case FR_INVALID_DRIVE:
    case FR_NOT_ENABLED:
    case FR_NO_FILESYSTEM:
    case FR_MKFS_ABORTED: {
      ERR("Invalid FS");
      break;
    }
    case FR_TIMEOUT: {
      WARN("FS timed out");
      break;
    }

    case FR_NOT_ENOUGH_CORE: {
      ERR("Out of memory");
      break;
    }

    case FR_TOO_MANY_OPEN_FILES: {
      WARN("Too many files open");
      break;
    }

    case FR_INVALID_PARAMETER: {
      DBUG("Invalid parameters");
      break;
    }

    default: {
      DBUG("Unknown Err");
      break;
    }
  }
#endif
  //TODO: Find a better way to do this
  Error_Handler();
  return 0;
}

FIL *FATFS_malloc (BYTE sync)
{
  DBUG("Creating new file pointer");
  FIL *newFile = (FIL *) malloc(sizeof(FIL));
  if (sync) {
    DBUG("Storing reference to new file for sync");
    for (size_t i = 0; i < _FS_LOCK; i++) {
      if (files[i] == NULL) {
        files[i] = newFile;
        break;
      }
    }
  }
  return newFile;
}

int FATFS_expand (FIL *fp, FSIZE_t fsize, BYTE opt)
{
  FRESULT ret = f_expand(fp, fsize, opt);
  if (ret != FR_OK) return 0;
  return 1;
}

int FATFS_sync (FIL *fp)
{
  FRESULT ret;
  if (fp != NULL) {
    ret = f_sync(fp);
    if (ret != FR_OK) return 0;
  }
  // Sync all tracked files
  else {
    INFO("Syncing all tracked files");
    for(size_t i = 0; i < _FS_LOCK; i++) {
      fp = files[i];
      if(fp != NULL){
        ret = f_sync(fp);
        if(ret != FR_OK) {
          ERR("Failed to sync tracked files to disk");
          return 0;
        }
      }
      else break;
    }
  }
  DBUG("Synced all files");
  return 1;
}

int CMD_sync(__unused int argc, __unused char * args[])
{
  TTY_println("Syncing all tracked files");
  return FATFS_sync(NULL);
}

// TODO: Can't read file while open
int CMD_cat(int argc, char * args[])
{
  if(argc != 1) return TTY_println("Please specify one file name");

  char * fname = args[0];
  FILINFO fno;
  FRESULT ret = f_stat(fname, &fno);
  // File exists
  if(ret == FR_OK) {
    FIL * fp = FATFS_malloc(0);

    ret = f_open(fp, fname, FA_READ);
    if (ret != FR_OK) return FATFS_errHandle_(ret);

    const FSIZE_t bufLen = 512;
    BYTE buf[bufLen];
    UINT br = bufLen;
    while(br >= bufLen) {
      ret = f_read(fp, buf, bufLen, &br);
      if(ret != FR_OK) return FATFS_errHandle_(ret);

      TTY_write(buf, br);
    }
    DBUG("End of file reached");

    FATFS_close(fp);
    free(fp);
  }
  // No such file
  else if (ret == FR_NO_FILE){
    TTY_println("No such file");
  }
  // Some other error
  else return FATFS_errHandle_(ret);

  return 1;
}

/* LOG CODE START Application */
#ifndef DEBUG

int LOG_write (uint8_t *buf, size_t len)
{
  UINT    bytesWritten;
  FRESULT ret;

  ret = f_write(LogFile, buf, len, &bytesWritten);

  if (ret != FR_OK) return FATFS_errHandle_(ret);
  else return bytesWritten;
}

int LOG_flush ()
{
  return FATFS_sync(NULL);
}

#endif
/* LOG CODE END Application */
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
