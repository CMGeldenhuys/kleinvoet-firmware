//
// Created by CM Geldenhuys on 2020/08/17.
//

#ifndef ADC_H_
#define ADC_H_

#include "main.h"
#include "wave.h"

#ifndef ADC_FILENAME
#define ADC_FILENAME "REC"
#endif

#ifndef ADC_SAMPLING_RATE
#define ADC_SAMPLING_RATE 48000
#endif

#ifndef ADC_N_CHANNELS
#define ADC_N_CHANNELS 2
#endif

#ifndef ADC_CRYSTAL_FREQ
#define ADC_CRYSTAL_FREQ 12288000
#endif


#ifndef ADC_DMA_BUF_LEN
#define ADC_DMA_BUF_LEN  (0x10000)
#endif

#define ADC_N_FRAMES (ADC_DMA_BUF_LEN / sizeof(uint32_t))
#define ADC_N_SAMPLES (ADC_N_FRAMES / ADC_N_CHANNELS)

#ifndef ADC_I2C_ADDR0
#define ADC_I2C_ADDR0 0b00000000U
//                       ^
#endif

#ifndef ADC_I2C_ADDR1
#define ADC_I2C_ADDR1 0b00000000U
//                      ^
#endif
//                        0bAA10001RW
#define ADC_I2C_ADDR_BASE 0b00100010U
#define ADC_I2C_ADDR      (ADC_I2C_ADDR_BASE | ADC_I2C_ADDR1 | ADC_I2C_ADDR0)

#define U_(__OP__) ((unsigned)((__OP__)))

#define ADC_is_recording      (adc.state.mode == ADC_REC)
#define ADC_is_interrupt_set  (adc.state.flags.rec & ADC_FLAG_CPLT_FIELD)
#define ADC_is_err_set        (adc.state.flags.err & ADC_FLAG_ERR_FIELD)
#define ADC_is_cplt_half      (adc.state.flags.rec & U_(ADC_CPLT_HALF))
#define ADC_is_cplt_full      (adc.state.flags.rec & U_(ADC_CPLT_FULL))

#define ADC_clear_flag_cplt   (adc.state.flags.rec &= ~ADC_FLAG_CPLT_FIELD)
#define ADC_clear_flag_err    (adc.state.flags.err &= ~ADC_FLAG_ERR_FIELD)

#define ADC_FLAG_CPLT_FIELD     (U_(ADC_CPLT_HALF) | U_(ADC_CPLT_FULL))

#define ADC_FLAG_ERR_FIELD      (U_(ADC_ERR_N_REC) | U_(ADC_ERR_SAMPLE_MISSED))

typedef enum {
    ADC_UNDEF = 0x00U,
    ADC_REC,
    ADC_SETUP,
    ADC_IDLE,
} ADC_state_major_e;

typedef enum {
    ADC_CPLT_HALF = 0b00000001U,
    ADC_CPLT_FULL = 0b00000010U,
    // ADC_CPLT_INTERRUPT           ^^
} ADC_state_flag_rec_e;

typedef enum {
    ADC_ERR_SAMPLE_MISSED = 0b00010000U,
    ADC_ERR_N_REC         = 0b00100000U,
    // ADC_ERR                ^^^^
} ADC_state_flag_err_e;

typedef struct {
    ADC_state_flag_rec_e rec;
    ADC_state_flag_err_e err;
} ADC_state_flags_t;

typedef struct {
    ADC_state_major_e mode;
    ADC_state_flags_t flags;
} ADC_state_t;


typedef struct {
    I2C_HandleTypeDef    *control;
    SAI_HandleTypeDef    *audioPort;
    TIM_HandleTypeDef    *tim;
    volatile ADC_state_t state;
    volatile uint32_t    nFramesMissed;
    volatile uint32_t    nFrames;
    WAVE_t               wav;
    uint8_t              *dmaBuf;
} ADC_t;


int ADC_init (I2C_HandleTypeDef *controlInterface, SAI_HandleTypeDef *audioInterface, TIM_HandleTypeDef *timInterface);

uint8_t ADC_readRegister (uint8_t registerAddr);

int ADC_writeRegister (uint8_t registerAddr, uint8_t data);

void ADC_reset ();

int ADC_powerUp ();

int ADC_powerDown ();

int ADC_setState (ADC_state_major_e state);

int ADC_yield ();

// Size of I2C registers
#define ADC_REG_SIZE            0x01u
// Max delay used for blocking IO
#ifndef ADC_MAX_DELAY
#define ADC_MAX_DELAY           HAL_MAX_DELAY
#endif

#define ADC_REG_M_POWER         0x00U
#define ADC_MASK_S_RST          0x80U
#define ADC_S_RST               0x80U
#define ADC_MASK_PWUP           0x01U
#define ADC_PWUP_PWDWN          0x00U
#define ADC_PWUP                0x01U

#define ADC_REG_PLL_CONTROL     0x01U
#define ADC_MASK_PLL_LOCK       0x80U
#define ADC_MASK_PLL_MUTE       0x40U
#define ADC_PLL_MUTE_ON         0x40U
#define ADC_MASK_CLK_S          0x10U
#define ADC_CLK_S_MCLK          0x00U
#define ADC_CLK_S_LRCLK         0x10U
#define ADC_MASK_MCS            0x07U
#define ADC_MCS_128             0x00U
#define ADC_MCS_256             0x01U
#define ADC_MCS_384             0x02U
#define ADC_MCS_512             0x03U
#define ADC_MCS_768             0x04U

#define ADC_REG_BLOCK_POWER_SAI 0x04U
#define ADC_MASK_LR_POL         0x80U
#define ADC_LR_POL_LH           0x00U
#define ADC_LR_POL_HL           0x80U
#define ADC_MASK_BCLKEDGE       0x40U
#define ADC_BCLKEDGE_FALLING    0x00U
#define ADC_BCLKEDGE_RISING     0x40U
#define ADC_MASK_LDO_EN         0x20U
#define ADC_LDO_EN_ON           0x20U
#define ADC_LDO_EN_OFF          0x00U
#define ADC_MASK_VREF_EN        0x10U
#define ADC_VREF_EN_OFF         0x00U
#define ADC_VREG_EN_ON          0x10U
#define ADC_MASK_ADC_EN4        0x08U
#define ADC_ADC_EN4_ON          0x08U
#define ADC_ADC_EN4_OFF         0x00U
#define ADC_MASK_ADC_EN3        0x04U
#define ADC_ADC_EN3_ON          0x04U
#define ADC_ADC_EN3_OFF         0x00U
#define ADC_MASK_ADC_EN2        0x02U
#define ADC_ADC_EN2_ON          0x02U
#define ADC_ADC_EN2_OFF         0x00U
#define ADC_MASK_ADC_EN1        0x01U
#define ADC_ADC_EN1_ON          0x01U
#define ADC_ADC_EN1_OFF         0x00U

#define ADC_REG_SAI_CTRL0       0x05U
#define ADC_MASK_SDATA_FMT      0xC0U
#define ADC_SDATA_FMT_I2S       0x00U
#define ADC_SDATA_FMT_LJ        0x40U
#define ADC_SDATA_FMT_RJ_24     0x80U
#define ADC_SDATA_FMT_RJ_16     0xC0U
#define ADC_MASK_SAI            0x38U
#define ADC_SAI_STEREO          0x00U
#define ADC_SAI_TDM2            0x08U
#define ADC_SAI_TDM4            0x10U
#define ADC_SAI_TDM8            0x18U
#define ADC_SAI_TDM16           0x20U
#define ADC_MASK_FS             0x07U
#define ADC_FS_8_12             0x00U
#define ADC_FS_16_24            0x01U
#define ADC_FS_32_48            0x02U
#define ADC_FS_64_96            0x03U
#define ADC_FS_128_192          0x04U

#define ADC_REG_SAI_CTRL1       0x06U
#define ADC_MASK_SDATA_SEL      0x80U
#define ADC_SDATA_SEL_1         0x00U
#define ADC_SDATA_SEL_2         0x80U
#define ADC_MASK_SLOT_WIDTH     0x60U
#define ADC_SLOT_WIDTH_32       0x00U
#define ADC_SLOT_WIDTH_24       0x20U
#define ADC_SLOT_WIDTH_16       0x40U
#define ADC_MASK_DATA_WIDTH     0x10U
#define ADC_DATA_WIDTH_24       0x00U
#define ADC_DATA_WIDTH_16       0x10U
#define ADC_MASK_LR_MODE        0x08U
#define ADC_LR_MODE_50          0x00U
#define ADC_LR_MODE_PULSE       0x08U
#define ADC_MASK_SAI_MSB        0x04U
#define ADC_SAI_MSB_MSB         0x00U
#define ADC_SAI_MSB_LSB         0x04U
#define ADC_MASK_BLCKRATE       0x02U
#define ADC_BCLKRATE_32         0x00U
#define ADC_BCLKRATE_16         0x02U
#define ADC_MASK_SAI_MS         0x01U
#define ADC_SAI_MS_SLAVE        0x00U
#define ADC_SAI_MS_MASTER       0x01U

#define ADC_REG_SAI_CMAP12      0x07U
#define ADC_MASK_CMAP_C2        0xF0U
#define ADC_CMAP_C2_1           0x00U
#define ADC_CMAP_C2_2           0x10U
#define ADC_CMAP_C2_3           0x20U
#define ADC_CMAP_C2_4           0x30U
#define ADC_CMAP_C2_5           0x40U
#define ADC_CMAP_C2_6           0x50U
#define ADC_CMAP_C2_7           0x60U
#define ADC_CMAP_C2_8           0x70U
#define ADC_CMAP_C2_9           0x80U
#define ADC_CMAP_C2_10          0x90U
#define ADC_CMAP_C2_11          0xA0U
#define ADC_CMAP_C2_12          0xB0U
#define ADC_CMAP_C2_13          0xC0U
#define ADC_CMAP_C2_14          0xD0U
#define ADC_CMAP_C2_15          0xE0U
#define ADC_CMAP_C2_16          0xF0U
#define ADC_MASK_CMAP_C1        0x0FU
#define ADC_CMAP_C1_1           0x00U
#define ADC_CMAP_C1_2           0x01U
#define ADC_CMAP_C1_3           0x02U
#define ADC_CMAP_C1_4           0x03U
#define ADC_CMAP_C1_5           0x04U
#define ADC_CMAP_C1_6           0x05U
#define ADC_CMAP_C1_7           0x06U
#define ADC_CMAP_C1_8           0x07U
#define ADC_CMAP_C1_9           0x08U
#define ADC_CMAP_C1_10          0x09U
#define ADC_CMAP_C1_11          0x0AU
#define ADC_CMAP_C1_12          0x0BU
#define ADC_CMAP_C1_13          0x0CU
#define ADC_CMAP_C1_14          0x0DU
#define ADC_CMAP_C1_15          0x0EU
#define ADC_CMAP_C1_16          0x0FU

#define ADC_REG_SAI_CMAP34      0x08U
#define ADC_MASK_CMAP_C4        0xF0U
#define ADC_CMAP_C4_1           0x00U
#define ADC_CMAP_C4_2           0x10U
#define ADC_CMAP_C4_3           0x20U
#define ADC_CMAP_C4_4           0x30U
#define ADC_CMAP_C4_5           0x40U
#define ADC_CMAP_C4_6           0x50U
#define ADC_CMAP_C4_7           0x60U
#define ADC_CMAP_C4_8           0x70U
#define ADC_CMAP_C4_9           0x80U
#define ADC_CMAP_C4_10          0x90U
#define ADC_CMAP_C4_11          0xA0U
#define ADC_CMAP_C4_12          0xB0U
#define ADC_CMAP_C4_13          0xC0U
#define ADC_CMAP_C4_14          0xD0U
#define ADC_CMAP_C4_15          0xE0U
#define ADC_CMAP_C4_16          0xF0U
#define ADC_MASK_CMAP_C3        0x0FU
#define ADC_CMAP_C3_1           0x00U
#define ADC_CMAP_C3_2           0x01U
#define ADC_CMAP_C3_3           0x02U
#define ADC_CMAP_C3_4           0x03U
#define ADC_CMAP_C3_5           0x04U
#define ADC_CMAP_C3_6           0x05U
#define ADC_CMAP_C3_7           0x06U
#define ADC_CMAP_C3_8           0x07U
#define ADC_CMAP_C3_9           0x08U
#define ADC_CMAP_C3_10          0x09U
#define ADC_CMAP_C3_11          0x0AU
#define ADC_CMAP_C3_12          0x0BU
#define ADC_CMAP_C3_13          0x0CU
#define ADC_CMAP_C3_14          0x0DU
#define ADC_CMAP_C3_15          0x0EU
#define ADC_CMAP_C3_16          0x0FU

#define ADC_REG_SAI_OVERTEMP    0x09U
#define ADC_MASK_SAI_DRV_C4     0x80U
#define ADC_SAI_DRV_C4_ON       0x80U
#define ADC_SAI_DRV_C4_OFF      0X00U
#define ADC_MASK_SAI_DRV_C3     0x40U
#define ADC_SAI_DRV_C3_ON       0x40U
#define ADC_SAI_DRV_C3_OFF      0x00U
#define ADC_MASK_SAI_DRV_C2     0x20U
#define ADC_SAI_DRV_C2_ON       0x20U
#define ADC_SAI_DRV_C2_OFF      0x00U
#define ADC_MASK_SAI_DRV_C1     0x10U
#define ADC_SAI_DRV_C1_ON       0x10U
#define ADC_SAI_DRV_C1_OFF      0x00U
#define ADC_MASK_DRV_HIZ        0x08U
#define ADC_DRV_HIZ_HIZ         0x08U
#define ADC_DRV_HIZ_LOW         0x00U
#define ADC_MASK_OT             0x01U

#define ADC_REG_POSTADC_GAIN1   0x0AU
#define ADC_REG_POSTADC_GAIN2   0x0BU
#define ADC_REG_POSTADC_GAIN3   0x0CU
#define ADC_REG_POSTADC_GAIN4   0x0DU

#define ADC_REG_MISC_CONTROL    0x0EU
#define ADC_MASK_SUM_MODE       0xC0U
#define ADC_SUM_MODE_NONE       0x00U
#define ADC_SUM_MODE_2          0x40U
#define ADC_SUM_MODE_1          0x80U
#define ADC_MASK_MMUTE          0x10U
#define ADC_MMUTE_NONE          0x00U
#define ADC_MMUTE_ALL           0x10U
#define ADC_MASK_DC_CAL         0x01U
#define ADC_DC_CAL_NO           0x00U
#define ADC_DC_CAL_PERF         0x01U

#define ADC_REG_ASDC_CLIP       0x19U
#define ADC_MASK_ADC_CLIP4      0x08U
#define ADC_MASK_ADC_CLIP3      0x04U
#define ADC_MASK_ADC_CLIP2      0X02U
#define ADC_MASK_ADC_CLIP1      0X01U

#define ADC_REG_HPF_CAL         0x1AU
#define ADC_MASK_DC_SUB_C4      0x80U
#define ADC_DC_SUB_C4_OFF       0x00U
#define ADC_DC_SUB_C4_ON        0x80U
#define ADC_MASK_DC_SUB_C3      0x40U
#define ADC_DC_SUB_C3_OFF       0x00U
#define ADC_DC_SUB_C3_ON        0x40U
#define ADC_MASK_DC_SUB_C2      0x20U
#define ADC_DC_SUB_C2_OFF       0x00U
#define ADC_DC_SUB_C2_ON        0x20U
#define ADC_MASK_DC_SUB_C1      0x10U
#define ADC_DC_SUB_C1_OFF       0x00U
#define ADC_DC_SUB_C1_ON        0x10U
#define ADC_MASK_DC_HPF_C4      0x08U
#define ADC_DC_HPF_C4_OFF       0x00U
#define ADC_DC_HPF_C4_ON        0x08U
#define ADC_MASK_DC_HPF_C3      0X04U
#define ADC_DC_HPF_C3_OFF       0x00U
#define ADC_DC_HPF_C3_ON        0x04U
#define ADC_MASK_DC_HPF_C2      0x02U
#define ADC_DC_HPF_C2_OFF       0x00U
#define ADC_DC_HPF_C2_ON        0x02U
#define ADC_MASK_DC_HPF_C1      0x01U
#define ADC_DC_HPF_C1_OFF       0x00U
#define ADC_DC_HPF_C1_ON        0x01U

// Predefined Commands
#define ADC_CMD_IS_PLL_LOCKED() (ADC_readRegister(ADC_REG_PLL_CONTROL) & ADC_MASK_PLL_LOCK)

#endif // ADC_H_
