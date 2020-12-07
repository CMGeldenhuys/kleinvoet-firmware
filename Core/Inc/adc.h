//
// Created by CM Geldenhuys on 2020/08/17.
//

#ifndef ADC_H_
#define ADC_H_

#include "main.h"
#include "wave.h"

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

typedef enum {
    ADC_UNDEF,
    ADC_IDLE
} ADC_state_e;

typedef struct {
    I2C_HandleTypeDef *control;
    SAI_HandleTypeDef *audioPort;
    ADC_state_e       state;
//    uint8_t           rx[ADC_RX_LEN][ADC_NUM_CH][ADC_FRAME_SIZE];
//    size_t            rxPos;
//    size_t            sampleCount;
//    uint32_t          *storePtr;
//    WAVE_t            wav;
} ADC_t;


int ADC_init (I2C_HandleTypeDef *controlInterface, SAI_HandleTypeDef *audioInterface);

uint8_t ADC_readRegister(uint8_t registerAddr);
int ADC_writeRegister(uint8_t registerAddr, uint8_t data);

int ADC_yield ();

// Size of I2C registers
#define ADC_REG_SIZE            0x01u
// Max delay used for blocking IO
#define ADC_MAX_DELAY           HAL_MAX_DELAY

#define ADC_REG_M_POWER         0x00U
#define ADC_MASK_S_RST          0x80U
#define ADC_MASK_PWUP           0x01U

#define ADC_REG_PLL_CONTROL     0x01U
#define ADC_MASK_PLL_LOCK       0x80U
#define ADC_MASK_PLL_MUTE       0x40U
#define ADC_MASK_CLK_S          0x10U
#define ADC_MASK_MCS            0x07U

#define ADC_REG_BLOCK_POWER_SAI 0x04U
#define ADC_MASK_LR_POL         0x80U
#define ADC_MASK_BCLKEDGE       0x40U
#define ADC_MASK_LDO_EN         0x20U
#define ADC_MASK_VREF_EN        0x10U
#define ADC_MASK_ADC_EN4        0x08U
#define ADC_MASK_ADC_EN3        0x04U
#define ADC_MASK_ADC_EN2        0x02U
#define ADC_MASK_ADC_EN1        0x01U

#define ADC_REG_SAI_CTRL0       0x05U
#define ADC_MASK_SDATA_FMT      0xC0U
#define ADC_MASK_SAI            0x38U
#define ADC_MASK_FS             0x07U

#define ADC_REG_SAI_CTRL1       0x06U
#define ADC_MASK_SDATA_SEL      0x80U
#define ADC_MASK_SLOT_WIDTH     0x60U
#define ADC_MASK_DATA_WIDTH     0x10U
#define ADC_MASK_LR_MODE        0x08U
#define ADC_MASK_SAI_MSB        0x04U
#define ADC_MASK_BLCKRATE       0x02U
#define ADC_MASK_SAI_MS         0x01U

#define ADC_REG_SAI_CMAP12      0x07U
#define ADC_MASK_CMAP_C2        0xF0U
#define ADC_MASK_CMAP_C1        0x0FU

#define ADC_REG_SAI_CMAP34      0x08U
#define ADC_MASK_CMAP_C4        0xF0U
#define ADC_MASK_CMAP_C3        0x0FU

#define ADC_REG_SAI_OVERTEMP    0x09U
#define ADC_MASK_SAI_DRV_C4     0x80U
#define ADC_MASK_SAI_DRV_C3     0x40U
#define ADC_MASK_SAI_DRV_C2     0x20U
#define ADC_MASK_SAI_DRV_C1     0x10U
#define ADC_MASK_DRV_HIZ        0x08U
#define ADC_MASK_OT             0x01U

#define ADC_REG_POSTADC_GAIN1   0x0AU
#define ADC_REG_POSTADC_GAIN2   0x0BU
#define ADC_REG_POSTADC_GAIN3   0x0CU
#define ADC_REG_POSTADC_GAIN4   0x0DU

#define ADC_REG_MISC_CONTROL    0x0EU
#define ADC_MASK_SUM_MODE       0xC0U
#define ADC_MASK_MMUTE          0x10U
#define ADC_MASK_DC_CAL         0x01U

#define ADC_REG_ASDC_CLIP       0x19U
#define ADC_MASK_ADC_CLIP4      0x08U
#define ADC_MASK_ADC_CLIP3      0x04U
#define ADC_MASK_ADC_CLIP2      0X02U
#define ADC_MASK_ADC_CLIP1      0X01U

#define ADC_REG_HPF_CAL         0x1AU
#define ADC_MASK_DC_SUB_C4      0x80U
#define ADC_MASK_DC_SUB_C3      0x40U
#define ADC_MASK_DC_SUB_C2      0x20U
#define ADC_MASK_DC_SUB_C1      0x10U
#define ADC_MASK_DC_HPF_C4      0x08U
#define ADC_MASK_DC_HPF_C3      0X04U
#define ADC_MASK_DC_HPF_C2      0x02U
#define ADC_MASK_DC_HPF_C1      0X01U

#endif // ADC_H_
