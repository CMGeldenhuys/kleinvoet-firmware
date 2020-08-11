//
// Created by CM Geldenhuys on 2020/08/11.
//
#include "wave.h"
#include "fatfs.h"

const static DWORD WAVE_CID_RIFF      =  0x46464952   ; // "RIFF"
const static DWORD WAVE_CID_FMT       =  0x20746d66   ; // "fmt "
const static DWORD WAVE_CID_DATA      =  0x61746164   ; // "data"

const static DWORD WAVE_RIFF_FORMAT   =  0x45564157   ; // "WAVE"

const static  WORD WAVE_AUDIO_PCM     =  1u           ;


typedef struct {
  DWORD   ChunkID       ;
  DWORD   ChunkSize     ;
  DWORD   Format        ;
} WAVE_RIFF_chunk_t;

typedef struct {
  DWORD   SubchunkID    ;
  DWORD   SubchunkSize  ;
   WORD   AudioFormat   ;
   WORD   NumChannels   ;
  DWORD   SampleRate    ;
  DWORD   ByteRate      ;
   WORD   BlockAlign    ;
   WORD   BitsPerSample ;
} WAVE_fmt_subchunk_t;

//TODO: remove
#define nSamples  64
#define nChannels 2

typedef struct {
  DWORD   SubchunkID    ;
  DWORD   SubchunkSize  ;
  BYTE  Data[nSamples][nChannels];
} WAVE_data_subchunk_t;

typedef struct {
  WAVE_RIFF_chunk_t RIFF_chunk;
  WAVE_fmt_subchunk_t fmt_chunk;
  WAVE_data_subchunk_t data_chunk;
} WAVE_file_t;



void WAVE_genData_(BYTE data[nSamples][nChannels]){
  for (size_t sample = 0; sample < nSamples; sample++) {
    for (size_t channel = 0; channel < nChannels; channel++) {
      data[sample][channel] = (BYTE)sample;
    }
  }
}
WAVE_file_t wav;
FIL file;
int WAVE_header()
{
  WAVE_genData_(wav.data_chunk.Data);

  // RIFF Chunk
  wav.RIFF_chunk.ChunkID = WAVE_CID_RIFF;
  wav.RIFF_chunk.ChunkSize = 36 + nSamples * nChannels;
  wav.RIFF_chunk.Format = WAVE_RIFF_FORMAT;

  // fmt Subchunk
  wav.fmt_chunk.SubchunkID = WAVE_CID_FMT;
  wav.fmt_chunk.SubchunkSize = 16;
  wav.fmt_chunk.AudioFormat = WAVE_AUDIO_PCM;
  wav.fmt_chunk.NumChannels = nChannels;
  wav.fmt_chunk.SampleRate  = 16;
  wav.fmt_chunk.ByteRate = 16 * nChannels * 8/8;
  wav.fmt_chunk.BlockAlign = nChannels;
  wav.fmt_chunk.BitsPerSample = 8;

  // data Subchunk
  wav.data_chunk.SubchunkID = WAVE_CID_DATA;
  wav.data_chunk.SubchunkSize = nSamples * nChannels * 8/8;

  DBUG("WAVE Header created")

  FRESULT res = f_open(&file, "test.wav", FA_WRITE|FA_CREATE_ALWAYS);

  if (res == FR_OK) {
    DBUG("Opened file")
    UINT bw;
    res = f_write(&file, &wav, sizeof(wav), &bw);
    if (res == FR_OK)
    {
      DBUG("Data written")
      res = f_sync(&file);
      if (res == FR_OK){
        DBUG("FS Synced")
      }
    }
  }

  return 1;
}

