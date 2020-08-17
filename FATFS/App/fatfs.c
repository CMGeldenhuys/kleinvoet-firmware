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

int FATFS_errHandle_ (FRESULT res);

/* USER CODE END Variables */

void MX_FATFS_Init (void) {
  /*## FatFS: Link the SD driver ###########################*/
  retSD = FATFS_LinkDriver(&SD_Driver, SDPath);

  /* USER CODE BEGIN Init */
  /* additional user code for init */
  // NB: CANT CALL LOGGING YET AS IT HAS NOT BEEN INIT.
  TTY_registerCommand("free", &CMD_free);
  // TODO: Make it so that it flushes all files
  TTY_registerCommand("sync", &LOG_flush);
  FRESULT ret = f_mount(&SDFatFS, SDPath, 1);
  if (ret == FR_OK) {
    // Open all files
    // TODO: Move log handling out to own file
    ret = f_open(&LogFile, "tmp.log", FA_WRITE | FA_CREATE_ALWAYS);
    if (ret != FR_OK) {
      //TODO: HANDLE FAIL TO OPEN
      FATFS_errHandle_(ret);
    }
  } else {
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
DWORD get_fattime (void) {
  /* USER CODE BEGIN get_fattime */
  return 0;
  /* USER CODE END get_fattime */
}

/* USER CODE BEGIN Application */
int FATFS_free (uint32_t *free, uint32_t *total) {
  DWORD freClstr;
  FATFS *fs;
  FRESULT ret = f_getfree(SDPath, &freClstr, &fs);
  if (ret == FR_OK) {
    *total = (fs->n_fatent - 2) * fs->csize / 2;
    *free = freClstr * fs->csize / 2;

    return *free;
  } else {
    return FATFS_errHandle_(ret);
  }
}

int CMD_free (__unused int argc, __unused char *argv[]) {
  uint32_t free, tot;

  if (FATFS_free(&free, &tot) > 0) {
    TTY_printf("%10u MiB total drive space%s", tot / 1024, SERIAL_EOL);
    TTY_printf("%10u MiB free drive space%s", free / 1024, SERIAL_EOL);
    return 1;
  } else {
    TTY_println("Failed to get free space");
    return 0;
  }
}


int FATFS_open (FIL *fp, const TCHAR *path, BYTE mode) {
  FRESULT res = f_open(fp, path, mode);
  if (res == FR_OK) {
    DBUG("Successfully opened '%s'", path);
    return 1;
  } else {
    WARN("Failed to open '%s'", path);
    return FATFS_errHandle_(res);
  }
}


int FATFS_slwrite (FIL *fp, const void *buff, size_t len, int sync, int pos) {
  UINT bw;
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
    } else {
      DBUG("Wrote %u bytes to file (UNSAFE)", bw);
    }
  } else
    return FATFS_errHandle_(res);

  // Restore file pointer tail
  if (pos >= 0) {
    if (tail) {
      DBUG("Restoring file pointer to tail (%u)", tail);
      res = f_lseek(fp, tail);
      if (res != FR_OK) ERR("Failed to restore file pointer tail");
    } else {
      DBUG("Not restoring fp since first write");
    }
  }

  return bw;
}

int FATFS_close (FIL *fp) {
  FRESULT res;

  res = f_close(fp);

  if (res == FR_OK) {
    DBUG("File closed successfully");
    return 1;
  } else
    return FATFS_errHandle_(res);

}

int FATFS_errHandle_ (FRESULT res) {
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

  return 0;
}

/* LOG CODE START Application */
#ifndef DEBUG
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
  DBUG("Log cache flushed")
  ret = f_sync(&LogFile);
  if(ret != FR_OK) return FATFS_errHandle_(ret);
  else return 1;
}
#endif
/* LOG CODE END Application */
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
