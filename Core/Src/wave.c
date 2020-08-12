//
// Created by CM Geldenhuys on 2020/08/11.
//
#include "wave.h"

int WAVE_header_ (WAVE_t *wav);


int WAVE_createFile (WAVE_t *wav, const char *fname)
{
  //TODO: Handle errors

  if (wav->fp) WAVE_close(wav);

  DBUG("Allocating memory to WAVE file");
  wav->fp = (FIL *) malloc(sizeof(FIL));

  WAVE_header_(wav);

  FATFS_open(wav->fp, fname, FA_WRITE | FA_CREATE_ALWAYS);
  // TODO: Expand file to 4GB
  // Need to check for space

  FATFS_write(wav->fp, wav->header, sizeof(WAVE_header_t), 1);
  return 1;
}

int WAVE_header_ (WAVE_t *wav)
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
  fmt_chunk->SubchunkSize  = 16;
  fmt_chunk->AudioFormat   = WAVE_AUDIO_PCM;
  fmt_chunk->NumChannels   = WAVE_NCHANNELS;
  fmt_chunk->SampleRate    = WAVE_SAMPLE_RATE;
  fmt_chunk->ByteRate      = 16 / 8 * WAVE_NCHANNELS * WAVE_BPS;
  fmt_chunk->BlockAlign    = WAVE_NCHANNELS;
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
    INFO("Freeing WAV File resources");

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

