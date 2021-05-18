//
// Created by CM Geldenhuys on 2020/08/11.
//
#include "wave.h"
#include <string.h>
#include "perf.h"

int WAVE_createHeader_ (WAVE_t *wav);


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
    INFO("Creating static file allocation of %lu bytes (%u MiB)...", WAVE_FILE_SPILT, WAVE_B_to_MiB(WAVE_FILE_SPILT));
    WARN("WAVE header will not be updated with each block write");
    INFO("WAVE header will only be updated on close");
    if (FATFS_expand(wav->fp, WAVE_FILE_SPILT, 1) <= 0){
      ERR("Could not allocate continuous file space");
      return -1;
    }
#endif

  WAVE_createHeader_(wav);
  WAVE_writeHeader(wav);
  return 1;
}

int WAVE_writeHeader (WAVE_t *wav)
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
#ifndef WAVE_STATIC_FILE_ALLOC
  if (len >= FATFS_free()) {
    ERR("SD Card full!");
    return 0;
  }
#endif

  if (f_tell(wav->fp) < WAVE_FILE_SPILT) {
    wav->header->RIFFChunk.ChunkSize += len;
    wav->header->dataSubChunk.SubchunkSize += len;
#ifndef WAVE_STATIC_FILE_ALLOC
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
      // TODO Don't sync every write might help with throughput and reduce dropped frames?
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

int WAVE_createHeader_ (WAVE_t *wav)
{
  // Prevent memory leak if file already exists, otherwise reuse header
  if (!wav->header) {
    INFO("Allocating memory to WAVE file header (%u bytes)", sizeof(WAVE_header_t));
    wav->header = (WAVE_header_t *) malloc(sizeof(WAVE_header_t));

    if (!wav->sampleRate) wav->sampleRate       = WAVE_DEFAULT_SAMPLE_RATE;
    if (!wav->nChannels) wav->nChannels         = WAVE_DEFAULT_NCHANNELS;
    if (!wav->blockSize) wav->blockSize         = WAVE_DEFAULT_BLOCK_SIZE;
    if (!wav->bitsPerSample) wav->bitsPerSample = WAVE_DEFAULT_BPS;
  }
  // RIFF Chunk
  WAVE_RIFF_chunk_t *RIFF_chunk = &wav->header->RIFFChunk;
  RIFF_chunk->ChunkID   = WAVE_CHUNKID_RIFF;
  RIFF_chunk->ChunkSize = WAVE_SIZEOF_SUBCHUNK(WAVE_header_t);
  RIFF_chunk->Format    = WAVE_RIFF_FORMAT_WAVE;
  DBUG("RIFF_chunk->ChunkSize: %u", RIFF_chunk->ChunkSize);

  // LIST Chunk
  WAVE_LIST_chunk_t *LIST_chunk = &wav->header->listChunk;
  LIST_chunk->ChunkID    = WAVE_CHUNKID_LIST;
  LIST_chunk->ChunkSize = WAVE_SIZEOF_SUBCHUNK(WAVE_LIST_chunk_t);
  LIST_chunk->Format   = WAVE_FORMAT_INFO;
  DBUG("LIST_chunk->ChunkSize: %u", LIST_chunk->ChunkSize);

  WAVE_info_subchunk_t *iEngineer_subchunk = &LIST_chunk->subChunks[WAVE_INFO_IDX_ENGINEER];
  iEngineer_subchunk->SubchunkID   = WAVE_INFO_TAG_ENGINEER;
  iEngineer_subchunk->SubchunkSize = WAVE_SIZEOF_SUBCHUNK(WAVE_info_subchunk_t);
  strncpy(iEngineer_subchunk->Value, AUTHORS, WAVE_MAX_INFO_VALUE_LEN);
  DBUG("iEngineer_subchunk->SubchunkSize: %u", iEngineer_subchunk->SubchunkSize);

  WAVE_info_subchunk_t *iVersion_subchunk = &LIST_chunk->subChunks[WAVE_INFO_IDX_VERSION];
  iVersion_subchunk->SubchunkID   = WAVE_INFO_TAG_VERSION;
  iVersion_subchunk->SubchunkSize = WAVE_SIZEOF_SUBCHUNK(WAVE_info_subchunk_t);
  snprintf(iVersion_subchunk->Value, WAVE_MAX_INFO_VALUE_LEN, "KV_" VERSION "_%08X", KLEINVOET_UUID);
  DBUG("iVersion_subchunk->SubchunkSize: %u", iVersion_subchunk->SubchunkSize);

  WAVE_info_subchunk_t *iLocation_subchunk = &LIST_chunk->subChunks[WAVE_INFO_IDX_LOCATION];
  iLocation_subchunk->SubchunkID   = WAVE_INFO_TAG_LOCATION;
  iLocation_subchunk->SubchunkSize = WAVE_SIZEOF_SUBCHUNK(WAVE_info_subchunk_t);
  strncpy(iLocation_subchunk->Value, "NO LOCK", WAVE_MAX_INFO_VALUE_LEN);
  DBUG("iLocation_subchunk->SubchunkSize: %u", iLocation_subchunk->SubchunkSize);

#ifdef DEBUG
  WAVE_info_subchunk_t *iComment_subchunk = &LIST_chunk->subChunks[WAVE_INFO_IDX_COMMENT];
  iComment_subchunk->SubchunkID   = WAVE_INFO_TAG_COMMENT;
  iComment_subchunk->SubchunkSize = WAVE_SIZEOF_SUBCHUNK(WAVE_info_subchunk_t);
  strncpy(iComment_subchunk->Value, "NO COMMENT", WAVE_MAX_INFO_VALUE_LEN);
  DBUG("iComment_subchunk->SubchunkSize: %u", iComment_subchunk->SubchunkSize);
#endif

  // fmt Subchunk
  WAVE_fmt_subchunk_t *fmt_chunk = &wav->header->fmtSubChunk;
  fmt_chunk->SubchunkID    = WAVE_SUBCHUNKID_FMT;
  fmt_chunk->SubchunkSize  = WAVE_SIZEOF_SUBCHUNK(WAVE_fmt_subchunk_t);
  fmt_chunk->AudioFormat   = WAVE_AUDIO_PCM;
  fmt_chunk->NumChannels   = wav->nChannels;
  fmt_chunk->SampleRate    = wav->sampleRate;
  fmt_chunk->BlockAlign    = wav->blockSize;
  fmt_chunk->ByteRate      = wav->sampleRate * fmt_chunk->BlockAlign;
  fmt_chunk->BitsPerSample = wav->bitsPerSample;
  DBUG("fmt_chunk->SubchunkSize: %u", fmt_chunk->SubchunkSize);

  // data Subchunk
  WAVE_data_subchunk_t *data_chunk = &wav->header->dataSubChunk;
  data_chunk->SubchunkID   = WAVE_SUBCHUNKID_DATA;
  data_chunk->SubchunkSize = 0;

  DBUG("WAVE Header created");
  return 1;
}

int WAVE_close (WAVE_t *wav)
{
  INFO("Closing WAVE file...");
#ifdef WAVE_STATIC_FILE_ALLOC
  DBUG("Updating WAVE Header");
  WAVE_writeHeader(wav);
  DBUG("Truncating WAVE file");
  f_truncate(wav->fp);
#endif
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

int WAVE_infoChunkPrintf(WAVE_t *wav, WAVE_INFO_IDX_e infoTag, const char* fmt, ...)
{
  INFO("Changing info tag (%u)", infoTag);
  WAVE_LIST_chunk_t *LIST_chunk = &wav->header->listChunk;
  WAVE_info_subchunk_t *infoSubchunk = &LIST_chunk->subChunks[infoTag];

  // Parse varargs
  va_list args;
  va_start(args, fmt);

  // Parse message format
  int msgLen = vsnprintf(infoSubchunk->Value, WAVE_MAX_INFO_VALUE_LEN, fmt, args);

  // Free varargs
  va_end(args);

  return msgLen;
}

