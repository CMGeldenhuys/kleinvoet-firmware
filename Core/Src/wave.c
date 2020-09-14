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
  if(wav->fp == NULL) {
    ERR("Failed to allocate memory for file");
    return -1;
  }
  if(FATFS_open(wav->fp, fname, FA_CREATE_ALWAYS | FA_WRITE) <= 0) {
    ERR("Failed to open/create file");
    return -1;
  }

  WAVE_createHeader_(wav);
  WAVE_writeHeader_(wav);
  // TODO: Expand file to 4GB
  // Need to check for space

  return 1;
}

int WAVE_writeHeader_ (WAVE_t *wav)
{
  DBUG("Writing file header");
  return FATFS_lwrite(wav->fp, wav->header, sizeof(WAVE_header_t), 0);
}

int WAVE_appendData (WAVE_t *wav, const void *buff, size_t len, int sync)
{
  if(len < FATFS_free()){
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
      // TOOD: Check return
      // Spilt Files
      DBUG("Slitting WAVE file");
      WAVE_createFile(wav, "TEST.W01");
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
//  FATFS_swrite(wav->fp, &wav, sizeof(wav), 1);
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

