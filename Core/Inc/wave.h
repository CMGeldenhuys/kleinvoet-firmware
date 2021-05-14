/**
 *
 * @file wave.h
 * @author CM Geldenhuys
 * @date 12 Aug. 2020
 *
 * @brief WAVE audio file management
 *
 * A means to abstract away the creation and of Microsoft WAVE files and its
 * interactions.
 *
 */

#ifndef WAVE_H_
#define WAVE_H_
#ifdef __cplusplus
extern "C" {
#endif

#include "fatfs.h"

#ifndef WAVE_DEFAULT_SAMPLE_RATE
#define WAVE_DEFAULT_SAMPLE_RATE 8000
#endif

#ifndef WAVE_DEFAULT_BLOCK_SIZE
#define WAVE_DEFAULT_BLOCK_SIZE WAVE_FMT_BLOCK_SIZE_4B
#endif

#ifndef WAVE_DEFAULT_BPS
#define WAVE_DEFAULT_BPS WAVE_FMT_BPS_24
#endif

#ifndef WAVE_DEFAULT_NCHANNELS
#define WAVE_DEFAULT_NCHANNELS 2
#endif

#ifndef WAVE_MAX_FILE_NAME_LEN
#define WAVE_MAX_FILE_NAME_LEN 32
#endif

#ifndef WAVE_MAX_INFO_VALUE_LEN
#define WAVE_MAX_INFO_VALUE_LEN 14
#endif

#ifndef WAVE_MAX_INFO_N_SLOTS
#define WAVE_MAX_INFO_N_SLOTS 1
#endif

#if ((WAVE_MAX_INFO_VALUE_LEN * WAVE_MAX_INFO_N_SLOTS) % (4) != 0)
#warning "WAVE_MAX_INFO_VALUE_LEN must be block aligned. Enforcing struct packing to compensate."
#endif

#define WAVE_FILE_SPLIT_KiB(b) ((unsigned)(b) << 10U)
#define WAVE_FILE_SPLIT_MiB(b) ((unsigned)(WAVE_FILE_SPLIT_KiB(b)) << 10U)
#define WAVE_FILE_SPLIT_GiB(b) ((unsigned)(WAVE_FILE_SPLIT_MiB(b)) << 10U)

#define WAVE_B_to_KiB(__B__)    ((unsigned)((__B__))                 >> 10U)
#define WAVE_B_to_MiB(__B__)    ((unsigned)(WAVE_B_to_KiB((__B__)))  >> 10U)
#define WAVE_B_to_GiB(__B__)    ((unsigned)(WAVE_B_to_MiB((__B__)))  >> 10U)

#ifndef WAVE_FILE_SPLIT
#define WAVE_FILE_SPILT WAVE_FILE_SPLIT_MiB(256)
#endif

#define WAVE_FMT_BLOCK_SIZE_1B (1U)
#define WAVE_FMT_BLOCK_SIZE_2B (2U)
#define WAVE_FMT_BLOCK_SIZE_4B (4U)
#define WAVE_FMT_BLOCK_SIZE_8  WAVE_FMT_BLOCK_SIZE_1B
#define WAVE_FMT_BLOCK_SIZE_16 WAVE_FMT_BLOCK_SIZE_2B
#define WAVE_FMT_BLOCK_SIZE_32 WAVE_FMT_BLOCK_SIZE_4B

#define WAVE_FMT_BPS_8  (8U)
#define WAVE_FMT_BPS_16 (16U)
#define WAVE_FMT_BPS_24 (24U)
#define WAVE_FMT_BPS_32 (32U)

#define WAVE_RIFF_HEADER_CHUNK_SIZE (sizeof(DWORD) + sizeof(WAVE_RIFF_STR_u)) // legnth + name
#define WAVE_SIZEOF_SUBCHUNK(__chunk__) (sizeof(__chunk__) - WAVE_RIFF_HEADER_CHUNK_SIZE)

typedef union {
    DWORD  raw;
    u_char str[sizeof(DWORD)];
} WAVE_RIFF_STR_u;

const static WAVE_RIFF_STR_u WAVE_RIFF_FORMAT_WAVE = {.str = {'W', 'A', 'V', 'E'}};
const static WAVE_RIFF_STR_u WAVE_CHUNKID_RIFF  = {.str = {'R', 'I', 'F', 'F'}};
const static WAVE_RIFF_STR_u WAVE_CHUNKID_LIST   = {.str = {'L', 'I', 'S', 'T'}};
const static WAVE_RIFF_STR_u WAVE_FORMAT_INFO    = {.str = {'I', 'N', 'F', 'O'}};
const static WAVE_RIFF_STR_u WAVE_SUBCHUNKID_FMT = {.str = {'f', 'm', 't', ' '}};
const static WAVE_RIFF_STR_u WAVE_SUBCHUNKID_DATA  = {.str = {'d', 'a', 't', 'a'}};

const static WAVE_RIFF_STR_u WAVE_INFO_TAG_ICMT    = {.str = {'I', 'C', 'M', 'T'}};

const static WORD WAVE_AUDIO_PCM = 0x0001U;

// Very helpful: http://soundfile.sapp.org/doc/WaveFormat/
// Also helpful: http://www-mmsp.ece.mcgill.ca/Documents/AudioFormats/WAVE/WAVE.html

typedef struct {
    WAVE_RIFF_STR_u ChunkID;
    DWORD           ChunkSize;
    WAVE_RIFF_STR_u Format;
}                            WAVE_RIFF_chunk_t;

typedef struct {
    WAVE_RIFF_STR_u SubchunkID;
    DWORD           SubchunkSize;
    WORD          AudioFormat;
    WORD          NumChannels;
    DWORD         SampleRate;
    DWORD         ByteRate;
    WORD          BlockAlign;
    WORD          BitsPerSample;
}                            WAVE_fmt_subchunk_t;


typedef struct {
    WAVE_RIFF_STR_u SubchunkID;
    DWORD           SubchunkSize;
}                            WAVE_data_subchunk_t;

typedef struct __packed {
    WAVE_RIFF_STR_u SubchunkID;
    DWORD           SubchunkSize;
    char          Value[WAVE_MAX_INFO_VALUE_LEN];
}                            WAVE_info_subchunk_t;

typedef struct __packed {
    WAVE_RIFF_STR_u      ChunkID;
    DWORD                ChunkSize;
    WAVE_RIFF_STR_u      Format;
    WAVE_info_subchunk_t subChunks[WAVE_MAX_INFO_N_SLOTS];
}                            WAVE_LIST_chunk_t;

typedef struct __packed {
    WAVE_RIFF_chunk_t    RIFFChunk;
    WAVE_fmt_subchunk_t  fmtSubChunk;
    WAVE_LIST_chunk_t    listChunk;
    WAVE_data_subchunk_t dataSubChunk;
}                 WAVE_header_t;

typedef struct {
    WAVE_header_t *header;
    FIL           *fp;
    const char    *fname;
    uint8_t       subfile;
    uint32_t      sampleRate;
    uint16_t      nChannels;
    uint16_t      blockSize;
    uint16_t      bitsPerSample;
}                 WAVE_t;

/**
 * @brief Create WAVE file structure
 *
 * @param [in] wav Reference to WAVE object
 *
 * @return Returns a value greater than 0 if successful
 */
int WAVE_createFile (WAVE_t *wav);

/**
 * @brief Append data to the specified file
 *
 * @param [in] wav Reference to WAVE object
 * @param [in] buff Data to be written to data section of WAVE file
 * @param [in] len Size of data to be written
 * @param [in] sync Should a filesystem sync be initiated
 *
 * @return Returns a value greater than 0 if successful
 */
int WAVE_appendData (WAVE_t *wav, const void *buff, size_t len, int sync);

/**
 * @brief Safely close WAVE file
 *
 * @param [in] wav Reference to WAVE object
 *
 * @return Returns a value greater than 0 if successful
 */
int WAVE_close (WAVE_t *wav);

#ifdef __cplusplus
}
#endif
#endif //FIRMWARE_WAVE_H