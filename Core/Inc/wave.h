//
// Created by CM Geldenhuys on 2020/08/11.
//

#ifndef WAVE_H_
#define WAVE_H_
#ifdef __cplusplus
extern "C" {
#endif

#include "fatfs.h"

#ifndef WAVE_SAMPLE_RATE
#define WAVE_SAMPLE_RATE 4000
#endif

#ifndef WAVE_BLOCK_SIZE
#define WAVE_BLOCK_SIZE 32
#endif

#ifndef WAVE_BPS
#define WAVE_BPS 24
#endif

#ifndef WAVE_NCHANNELS
#define WAVE_NCHANNELS 2
#endif


#ifndef WAVE_FILE_SPLIT
#define WAVE_FILE_SPILT (-1u)
#endif

const static DWORD WAVE_CID_RIFF = 0x46464952; // "RIFF"
const static DWORD WAVE_CID_FMT = 0x20746d66; // "fmt "
const static DWORD WAVE_CID_DATA = 0x61746164; // "data"

const static DWORD WAVE_RIFF_FORMAT = 0x45564157; // "WAVE"

const static WORD WAVE_AUDIO_PCM = 1u;


typedef struct {
    DWORD ChunkID;
    DWORD ChunkSize;
    DWORD Format;
} WAVE_RIFF_chunk_t;

typedef struct {
    DWORD SubchunkID;
    DWORD SubchunkSize;
    WORD AudioFormat;
    WORD NumChannels;
    DWORD SampleRate;
    DWORD ByteRate;
    WORD BlockAlign;
    WORD BitsPerSample;
} WAVE_fmt_subchunk_t;


typedef struct {
    DWORD SubchunkID;
    DWORD SubchunkSize;
} WAVE_data_subchunk_t;

typedef struct {
    WAVE_RIFF_chunk_t RIFF_chunk;
    WAVE_fmt_subchunk_t fmt_chunk;
    WAVE_data_subchunk_t data_chunk;
} WAVE_header_t;

typedef struct {
    WAVE_header_t *header;
    FIL *fp;
} WAVE_t;

int WAVE_createFile (WAVE_t *wav, const char *fname);

int WAVE_appendData (WAVE_t *wav, const void *buff, size_t len, int sync);

int WAVE_close (WAVE_t *wav);

#ifdef __cplusplus
}
#endif
#endif //FIRMWARE_WAVE_H