//
// Created by CM Geldenhuys on 2020/08/11.
//

#ifndef WAVE_H_
#define WAVE_H_
#ifdef __cplusplus
extern "C" {
#endif

#include "fatfs.h"

#ifndef WAVE_DEFAULT_SAMPLE_RATE
#define WAVE_DEFAULT_SAMPLE_RATE 4000
#endif

#ifndef WAVE_DEFAULT_BLOCK_SIZE
#define WAVE_DEFAULT_BLOCK_SIZE 32
#endif

#ifndef WAVE_DEFAULT_BPS
#define WAVE_DEFAULT_BPS 24
#endif

#ifndef WAVE_DEFAULT_NCHANNELS
#define WAVE_DEFAULT_NCHANNELS 4
#endif

#ifndef WAVE_MAX_FILE_NAME_LEN
#define WAVE_MAX_FILE_NAME_LEN 32
#endif

#define WAVE_FILE_SPLIT_KiB(b) ((unsigned)(b) << 10U)
#define WAVE_FILE_SPLIT_MiB(b) ((unsigned)(WAVE_FILE_SPLIT_KiB(b)) << 10U)
#define WAVE_FILE_SPLIT_GiB(b) ((unsigned)(WAVE_FILE_SPLIT_MiB(b)) << 10U)

#ifndef WAVE_FILE_SPLIT
#ifdef DEBUG
#define WAVE_FILE_SPILT WAVE_FILE_SPLIT_MiB(32)
#else
#define WAVE_FILE_SPILT WAVE_FILE_SPLIT_GiB(2)
#endif
#endif

const static DWORD WAVE_CID_RIFF = 0x46464952; // "RIFF"
const static DWORD WAVE_CID_FMT  = 0x20746d66; // "fmt "
const static DWORD WAVE_CID_DATA = 0x61746164; // "data"

const static DWORD WAVE_RIFF_FORMAT = 0x45564157; // "WAVE"

const static WORD WAVE_AUDIO_PCM = 1u;

// Very helpful: http://soundfile.sapp.org/doc/WaveFormat/

typedef struct {
    DWORD ChunkID;
    DWORD ChunkSize;
    DWORD Format;
}                 WAVE_RIFF_chunk_t;

typedef struct {
    DWORD SubchunkID;
    DWORD SubchunkSize;
    WORD  AudioFormat;
    WORD  NumChannels;
    DWORD SampleRate;
    DWORD ByteRate;
    WORD  BlockAlign;
    WORD  BitsPerSample;
}                 WAVE_fmt_subchunk_t;


typedef struct {
    DWORD SubchunkID;
    DWORD SubchunkSize;
}                 WAVE_data_subchunk_t;

typedef struct {
    WAVE_RIFF_chunk_t    RIFF_chunk;
    WAVE_fmt_subchunk_t  fmt_chunk;
    WAVE_data_subchunk_t data_chunk;
}                 WAVE_header_t;

typedef struct {
    WAVE_header_t *header;
    FIL           *fp;
    const char *fname;
    uint8_t  subfile;
    uint32_t sampleRate;
    uint16_t nChannels;
    uint16_t blockSize;
    uint16_t bitsPerSample;
}                 WAVE_t;

int WAVE_createFile (WAVE_t *wav);

int WAVE_appendData (WAVE_t *wav, const void *buff, size_t len, int sync);

int WAVE_close (WAVE_t *wav);

#ifdef __cplusplus
}
#endif
#endif //FIRMWARE_WAVE_H