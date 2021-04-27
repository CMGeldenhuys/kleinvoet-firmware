//
// Created by CM Geldenhuys on 2020/08/11.
//
#include "wave.h"
#include <string.h>
#include "perf.h"

int WAVE_createHeader_ (WAVE_t *wav);

int WAVE_writeHeader_ (WAVE_t *wav);

int WAVE_createFile (WAVE_t *wav)
{
#ifdef WAVE_MOCK_WRITES
  WARN("MOCK WRITES ENABLED!");
  return 1;
#endif

  //TODO: Handle errors
  DBUG("Will split on %lu bytes (%lu KiB | %lu MiB | %lu GiB)",
       WAVE_FILE_SPILT,
       WAVE_B_to_KiB(WAVE_FILE_SPILT),
       WAVE_B_to_MiB(WAVE_FILE_SPILT),
       WAVE_B_to_GiB(WAVE_FILE_SPILT));
  if (wav->fp) WAVE_close(wav);

  DBUG("Allocating memory to WAVE file");
  wav->fp = FATFS_malloc(0);
  if (wav->fp == NULL) {
    ERR("Failed to allocate memory for file");
    return -1;
  }

  // Create empty string buffer
  char fname[WAVE_MAX_FILE_NAME_LEN];
  fname[0] = '\0';

  // Subsequent files
  if (wav->subfile > 0) {
    snprintf(fname, WAVE_MAX_FILE_NAME_LEN, "%s.W%02u", wav->fname, wav->subfile);
  }
    // Original file
  else {
    snprintf(fname, WAVE_MAX_FILE_NAME_LEN, "%s.WAV", wav->fname);
  }

  INFO("Creating new wav file (%s)", fname);
  if (FATFS_open(wav->fp, fname, FA_CREATE_ALWAYS | FA_WRITE) <= 0) {
    ERR("Failed to open/create file");
    // Clean up memory
    WAVE_close(wav);
    return -1;
  }

#ifdef WAVE_STATIC_FILE_ALLOC
    // Expanding the file increase write speed but on file reallocation the buffers overflow
    // Improves writing speed as the FS doesn't need to still allocated blocks of
    // space to file on write. Rather precompute and clear to ensure there will
    // also be enough space.
    // Also helps with file wearing
    INFO("Creating static file allocation of %lu bytes...", WAVE_FILE_SPILT);
    WARN("WAVE header will not be updated with each block write");
    INFO("WAVE header will only be updated on close");
    if (FATFS_expand(wav->fp, WAVE_FILE_SPILT, 1) <= 0){
      ERR("Could not allocate continuous file space");
      return -1;
    }
#endif

  WAVE_createHeader_(wav);
  WAVE_writeHeader_(wav);
  return 1;
}

int WAVE_writeHeader_ (WAVE_t *wav)
{
  DBUG("Writing file header");
  return FATFS_lwrite(wav->fp, wav->header, sizeof(WAVE_header_t), 0);
}

int WAVE_appendData (WAVE_t *wav, const void *buff, size_t len, int sync)
{
  // Disable actual writes to perserve SD
#ifdef WAVE_MOCK_WRITES
  HAL_Delay(50);
  return 1;
#endif

  if (len < FATFS_free()) {
    if (f_tell(wav->fp) < WAVE_FILE_SPILT) {
#ifndef WAVE_STATIC_FILE_ALLOC
        wav->header->RIFF_chunk.ChunkSize += len;
        wav->header->data_chunk.SubchunkSize += len;

        // TODO: Rather adjust header than rewriting header each time
        DBUG("Updating WAVE header (rewrite)");
        PERF_START("WAVE_writeHeader_");
        PERF_THRESHOLD(20);
        WAVE_writeHeader_(wav);
        PERF_END("WAVE_writeHeader_");
#endif

      DBUG("Appending WAVE data");
      // TODO: Handle errs
      int temp;
      PERF_START("WAVE_write");
      PERF_THRESHOLD(50);
      temp = FATFS_swrite(wav->fp, buff, len, sync);
      PERF_END("WAVE_write");
      if(temp <= 0)ERR("Writing Error");
      return temp;
    }
    else {
      // TODO: Check return
      // Spilt Files
      // TODO: Make use of BWF format (link chunk)
      // https://tech.ebu.ch/docs/tech/tech3285s4.pdf
      INFO("Slitting WAVE file");
      wav->subfile++; //TODO: Check overrun
      WAVE_createFile(wav);
      return WAVE_appendData(wav, buff, len, sync);
    }
  }
  else {
    ERR("SD Card full!");
    return 0;
  }
}

int WAVE_createHeader_ (WAVE_t *wav)
{

  // TODO: Stop using magic numbers and calculate
  #define WAVE_RIFF_CHUCK_MAGIC_SIZE 36
  #define WAVE_FMT_CHUCK_MAGIC_SIZE 16
  #define WAVE_HEADER_MAGIC_SIZE 44

  // Prevent memory leak if file already exists, otherwise reuse header
  if (!wav->header) {
    DBUG("Allocating memory to WAVE file header (creating new header)");
    wav->header = (WAVE_header_t *) malloc(sizeof(WAVE_header_t));

    if (!wav->sampleRate) wav->sampleRate       = WAVE_DEFAULT_SAMPLE_RATE;
    if (!wav->nChannels) wav->nChannels         = WAVE_DEFAULT_NCHANNELS;
    if (!wav->blockSize) wav->blockSize         = WAVE_DEFAULT_BLOCK_SIZE;
    if (!wav->bitsPerSample) wav->bitsPerSample = WAVE_DEFAULT_BPS;
  }
  // RIFF Chunk
  WAVE_RIFF_chunk_t *RIFF_chunk = &wav->header->RIFF_chunk;
  RIFF_chunk->ChunkID   = WAVE_CID_RIFF;
#ifdef WAVE_STATIC_FILE_ALLOC
  RIFF_chunk->ChunkSize = WAVE_FILE_SPILT;
#else
  RIFF_chunk->ChunkSize = WAVE_RIFF_CHUCK_MAGIC_SIZE;
#endif
  RIFF_chunk->Format    = WAVE_RIFF_FORMAT;



  // fmt Subchunk
  WAVE_fmt_subchunk_t *fmt_chunk = &wav->header->fmt_chunk;
  fmt_chunk->SubchunkID    = WAVE_CID_FMT;
  fmt_chunk->SubchunkSize  = WAVE_FMT_CHUCK_MAGIC_SIZE;
  fmt_chunk->AudioFormat   = WAVE_AUDIO_PCM;
  fmt_chunk->NumChannels   = wav->nChannels;
  fmt_chunk->SampleRate    = wav->sampleRate;
  fmt_chunk->BlockAlign    = wav->blockSize;
  fmt_chunk->ByteRate      = wav->sampleRate * fmt_chunk->BlockAlign;
  fmt_chunk->BitsPerSample = wav->bitsPerSample;



  // data Subchunk
  WAVE_data_subchunk_t *data_chunk = &wav->header->data_chunk;
  data_chunk->SubchunkID   = WAVE_CID_DATA;
#ifdef WAVE_STATIC_FILE_ALLOC
  data_chunk->SubchunkSize = WAVE_FILE_SPILT - WAVE_HEADER_MAGIC_SIZE;
#else
  data_chunk->SubchunkSize = 0;
#endif

  DBUG("WAVE Header created");
  return 1;
}

int WAVE_close (WAVE_t *wav)
{
  if (FATFS_close(wav->fp) > 0) {
    INFO("Freeing WAV file resources");

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

