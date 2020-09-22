//
// Created by CM Geldenhuys on 2020/08/11.
//
#include "wave.h"
#include <string.h>

int WAVE_createHeader_ (WAVE_t *wav);

int WAVE_writeHeader_ (WAVE_t *wav);

int WAVE_createFile (WAVE_t *wav)
{
  //TODO: Handle errors
  DBUG("Will split on %lu bytes (%lu KiB | %lu MiB | %lu GiB)", WAVE_FILE_SPILT, WAVE_FILE_SPILT >> 10U,
       WAVE_FILE_SPILT >> 10U, WAVE_FILE_SPILT >> 10U);
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

  // Expanding the file increase write speed but on file reallocation the buffers overflow
//  DBUG("Creating file allocation of %lu bytes", WAVE_FILE_SPILT);
//  if (FATFS_expand(wav->fp, WAVE_FILE_SPILT, 1) <= 0)
//    WARN("Could not allocate continuous file space");


  WAVE_createHeader_(wav);
  WAVE_writeHeader_(wav);

  // TODO: Expand file to 4GB/ File split
  // Improves writing speed as the FS doesn't need to still allocated blocks of
  // space to file on write. Rather precompute and clear to ensure there will
  // also be enough space.

  // TODO: Need to check for space

  return 1;
}

int WAVE_writeHeader_ (WAVE_t *wav)
{
  DBUG("Writing file header");
  return FATFS_lwrite(wav->fp, wav->header, sizeof(WAVE_header_t), 0);
}

int WAVE_appendData (WAVE_t *wav, const void *buff, size_t len, int sync)
{
  if (len < FATFS_free()) {
    if (wav->header->RIFF_chunk.ChunkSize + len < WAVE_FILE_SPILT) {
      wav->header->RIFF_chunk.ChunkSize += len;
      wav->header->data_chunk.SubchunkSize += len;

      // TODO: Rather adjust header than rewriting header each time
      WAVE_writeHeader_(wav);

      DBUG("Appending WAVE data");
      // TODO: Handle errs
      return FATFS_swrite(wav->fp, buff, len, sync);
    }
    else {
      // TODO: Check return
      // Spilt Files
      // TODO: Make us of BWF format (link chunk)
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
  RIFF_chunk->ChunkSize = 36; //TODO: Update with number of samples
  RIFF_chunk->Format    = WAVE_RIFF_FORMAT;



  // fmt Subchunk
  WAVE_fmt_subchunk_t *fmt_chunk = &wav->header->fmt_chunk;
  fmt_chunk->SubchunkID    = WAVE_CID_FMT;
  fmt_chunk->SubchunkSize  = 16; // TODO: Don't use magic number
  fmt_chunk->AudioFormat   = WAVE_AUDIO_PCM;
  fmt_chunk->NumChannels   = wav->nChannels;
  fmt_chunk->SampleRate    = wav->sampleRate;
  fmt_chunk->ByteRate      = wav->sampleRate * wav->nChannels * wav->bitsPerSample / 8;
  fmt_chunk->BlockAlign    = wav->nChannels * wav->blockSize / 8;
  fmt_chunk->BitsPerSample = wav->bitsPerSample;



  // data Subchunk
  WAVE_data_subchunk_t *data_chunk = &wav->header->data_chunk;
  data_chunk->SubchunkID   = WAVE_CID_DATA;
  data_chunk->SubchunkSize = 0; //TODO: Update with data size

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

