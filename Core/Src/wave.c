//
// Created by CM Geldenhuys on 2020/08/11.
//
#include "wave.h"

int WAVE_createHeader_ (WAVE_t *wav);

int WAVE_writeHeader_ (WAVE_t *wav);

int WAVE_createFile (WAVE_t *wav, const char *fname)
{
  //TODO: Handle errors

  if (wav->fp) WAVE_close(wav);

  DBUG("Allocating memory to WAVE file");
  wav->fp = (FIL *) malloc(sizeof(FIL));
  FATFS_open(wav->fp, fname, FA_CREATE_ALWAYS | FA_WRITE);

  WAVE_createHeader_(wav);
  WAVE_writeHeader_(wav);
  // TODO: Expand file to 4GB
  // Need to check for space


 // TODO: This is just test code
  uint8_t tmp[128] = {0};

  WAVE_appendData(wav, tmp, sizeof(tmp), 0);
  WAVE_appendData(wav, tmp, sizeof(tmp), 1);

  return 1;
}

int WAVE_writeHeader_ (WAVE_t *wav)
{
  FRESULT ret;
  FSIZE_t tail;
  // TODO: Wrap in FATFS class
  DBUG("Getting current file pointer tail");
  tail = f_tell(wav->fp);
  DBUG("Setting tail to 0 (HEAD)");
  ret = f_lseek(wav->fp, 0);
  if (ret != FR_OK) ERR("f_lseek failed");
  DBUG("Writing file header");
  FATFS_write(wav->fp, wav->header, sizeof(WAVE_header_t), 0);
  if (tail) {
    DBUG("Restoring file pointer to tail (%u)", tail);
    ret = f_lseek(wav->fp, tail);
    if (ret != FR_OK) ERR("f_lseek failed");
  } else {
    DBUG("Not restoring fp since first write");
  }
  return 1;
}

int WAVE_appendData (WAVE_t *wav, const void *buff, size_t len, int sync)
{
  // TODO: Check if 4 GB file size!!!
  // Create new file .wav .w01 .w02 .w03
  wav->header->RIFF_chunk.ChunkSize += len;
  wav->header->data_chunk.SubchunkSize += len;

  // TODO: Rather adjust header than rewriting header each time
  WAVE_writeHeader_(wav);

  DBUG("Appending WAVE data");
  FATFS_write(wav->fp, buff, len, sync);

  return 1;
}

int WAVE_createHeader_ (WAVE_t *wav)
{

  // Prevent memory leak if file already exists, otherwise reuse header
  if (!wav->header) {
    DBUG("Allocating memory to WAVE file header");
    wav->header = (WAVE_header_t *) malloc(sizeof(WAVE_header_t));
  }


  // RIFF Chunk
  WAVE_RIFF_chunk_t *RIFF_chunk = &wav->header->RIFF_chunk;
  RIFF_chunk->ChunkID   = WAVE_CID_RIFF;
  RIFF_chunk->ChunkSize = 36; //TODO: Update with number of samples
  RIFF_chunk->Format    = WAVE_RIFF_FORMAT;



  // fmt Subchunk
  WAVE_fmt_subchunk_t *fmt_chunk = &wav->header->fmt_chunk;
  fmt_chunk->SubchunkID    = WAVE_CID_FMT;
  fmt_chunk->SubchunkSize  = 16; // TODO: Don't use magic number
  fmt_chunk->AudioFormat   = WAVE_AUDIO_PCM;
  fmt_chunk->NumChannels   = WAVE_NCHANNELS;
  fmt_chunk->SampleRate    = WAVE_SAMPLE_RATE;
  fmt_chunk->ByteRate      = WAVE_SAMPLE_RATE * WAVE_NCHANNELS * WAVE_BPS / 8;
  fmt_chunk->BlockAlign    = WAVE_NCHANNELS * WAVE_BLOCK_SIZE / 8;
  fmt_chunk->BitsPerSample = WAVE_BPS;



  // data Subchunk
  WAVE_data_subchunk_t *data_chunk = &wav->header->data_chunk;
  data_chunk->SubchunkID   = WAVE_CID_DATA;
  data_chunk->SubchunkSize = 0; //TODO: Update with data size

  DBUG("WAVE Header created");
//  FATFS_open(wav->fp, "test.wav", FA_WRITE | FA_CREATE_ALWAYS);
//  FATFS_write(wav->fp, &wav, sizeof(wav), 1);
  return 1;
}

int WAVE_close (WAVE_t *wav)
{
  if (FATFS_close(wav->fp) > 0) {
    INFO("Freeing WAV resources");

    // Prevent memory leak
    free(wav->header);
    free(wav->fp);

    // Clear references
    wav->header = NULL;
    wav->fp     = NULL;

    return 1;
  }
  else return 0;
}

