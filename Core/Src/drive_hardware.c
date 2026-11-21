#include "drive_hardware.h"

#include <stdio.h>
#include <string.h>

extern I2C_HandleTypeDef hi2c3;
extern SPI_HandleTypeDef hspi1;

#define CALIB_FLASH_ADDR 0x0801F800u
#define CALIB_MAGIC 0xCA11B0A1u
#define STSPIN_I2C_ADDR (0x47 << 1)   // I2Cのスレーブアドレスには7bitアドレスと8bitアドレスがあり、アドレスの後ろに読み込みか書き込みかを示す1bit（R/Wビット）がくっついて送信される。

typedef struct __attribute__((packed)) {
    uint32_t magic;
    float    zero_offset_rad;
    uint32_t crc;
} calib_data_t;

/* regはレジスタの番地、valは書き込む値。I2C3でSTSPINを指定し、
 * 8ビットの番地に1バイトを書き込む。タイムアウトは100 ms。
 */
static HAL_StatusTypeDef stspin_write_reg(uint8_t reg, uint8_t val)
{
    return HAL_I2C_Mem_Write(&hi2c3, STSPIN_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &val, 1, 100);
}

/* valは読み出した1バイトの保存先。 */
HAL_StatusTypeDef stspin_read_reg(uint8_t reg, uint8_t *val)
{
    return HAL_I2C_Mem_Read(&hi2c3, STSPIN_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, val, 1, 100);
}

// STATUSレジスタ(0x80)を読み、フォルトがあればCLEAR(0x09)に0xFFを書く
uint8_t stspin_clear_faults(void)
{
    uint8_t status = 0;
    stspin_read_reg(0x80, &status);
    if (status & 0x0F) {                       // もしstatusの下位4bitの中に1があり、エラーがある場合はifが通る
        stspin_write_reg(0x09, 0xFF); // CLEARレジスタ
    }
    return status;
}

bool drive_v3_charge_bootstrap(void)
{
    const uint16_t low_side = GPIO_PIN_8 | GPIO_PIN_10 | GPIO_PIN_12;
    const uint16_t pwm_pins = low_side | GPIO_PIN_9 | GPIO_PIN_11 | GPIO_PIN_13;
    GPIO_InitTypeDef gpio = {0};
    uint8_t status;

    // PWM出力とカウンターを止めてから、6本すべてをGPIOのLow出力にする。
    CLEAR_BIT(TIM1->BDTR, TIM_BDTR_MOE);
    CLEAR_BIT(TIM1->CCER, TIM_CCER_CC1E | TIM_CCER_CC1NE | TIM_CCER_CC2E | TIM_CCER_CC2NE | TIM_CCER_CC3E | TIM_CCER_CC3NE);
    CLEAR_BIT(TIM1->CR1, TIM_CR1_CEN);

    HAL_GPIO_WritePin(GPIOE, pwm_pins, GPIO_PIN_RESET);
    gpio.Pin = pwm_pins;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOE, &gpio);
    HAL_Delay(1);

    // V3と同じく、全出力OFFで故障をクリアしてから充電する。
    if (stspin_write_reg(0x09, 0xFF) != HAL_OK) {
        return false;
    }
    HAL_Delay(1);
    if (stspin_read_reg(0x80, &status) != HAL_OK) {
        return false;
    }
    if (status & 0x0F) {
        printf("BOOT before charge STATUS=0x%02X\r\n", (unsigned int)status);
        return false;
    }

    // 下側だけを50 msオンにすると、上側用のブートストラップが充電される。
    HAL_GPIO_WritePin(GPIOE, low_side, GPIO_PIN_SET);
    HAL_Delay(50);
    HAL_GPIO_WritePin(GPIOE, pwm_pins, GPIO_PIN_RESET);

    if (stspin_read_reg(0x80, &status) != HAL_OK) {
        return false;
    }
    if (status & 0x0F) {
        printf("BOOT after charge STATUS=0x%02X\r\n", (unsigned int)status);
        return false;
    }

    // 呼び出し側で設定した50%の比較値を反映し、6本をTIM1の出力へ戻す。
    TIM1->CNT = 0;
    TIM1->EGR = TIM_EGR_UG;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Alternate = GPIO_AF2_TIM1;
    HAL_GPIO_Init(GPIOE, &gpio);
    return true;
}

static uint16_t spi_transfer_16(uint16_t tx)
{
    uint16_t rx = 0;

    HAL_GPIO_WritePin(SPI1_SS_GPIO_Port, SPI1_SS_Pin, GPIO_PIN_RESET);// 通信開始 AS5047Pでは、このピンを Low にすると通信を開始
    HAL_SPI_TransmitReceive(&hspi1, (uint8_t*)&tx, (uint8_t*)&rx, 1, HAL_MAX_DELAY);// 16ビット送受信
    HAL_GPIO_WritePin(SPI1_SS_GPIO_Port, SPI1_SS_Pin, GPIO_PIN_SET); // 通信終了

   return rx;
}

static uint16_t as5047p_read_raw16(void)
{
    spi_transfer_16(0xFFFF);          // ANGLECOMの読み出しを要求する。
    return spi_transfer_16(0xC000);   // NOPを送る間に、直前に要求した角度の生データを受け取る。
}

// bit15がパリティビット(偶数パリティ)。16bit全体のXORが0なら正常。
static uint8_t as5047p_parity_bad(uint16_t frame)
{
    uint16_t v = frame;
    v ^= v >> 8;
    v ^= v >> 4;
    v ^= v >> 2;
    v ^= v >> 1;
    return (v & 1);
}

uint16_t as5047p_read_angle_checked(uint8_t *fault_out)
{
    uint16_t frame = as5047p_read_raw16();
    uint8_t bad = 0;

    if (as5047p_parity_bad(frame)) bad = 1;   // as5047p_parity_bad() は、受信した16ビットに 1 が奇数個あれば 1、偶数個なら 0 を返す関数 AS5047Pは偶数パリティを使うので、戻り値 1 がパリティ異常
    if (frame & 0x4000)            bad = 1;   // AS5047PのEFビット(内部エラー、0xFFFFもここで捕捉される)

    *fault_out = bad;
    return frame & 0x3FFF;
}

// 既存コードとの互換用
uint16_t as5047p_read_angle(void)
{
    uint8_t dummy;
    return as5047p_read_angle_checked(&dummy);
}

/* 保存した角度補正値が変化していないか確認するためのXORチェック値。 */
static uint32_t calib_simple_crc(uint32_t magic, float val)
{
    uint32_t v;
    memcpy(&v, &val, sizeof(v));
    return magic ^ v ^ 0xA5A5A5A5u;
}

int calib_load(float *zero_offset_rad_out)
{
    const calib_data_t *stored = (const calib_data_t *)CALIB_FLASH_ADDR;

    if (stored->magic != CALIB_MAGIC) {
        return 0;
    }
    uint32_t expect_crc = calib_simple_crc(stored->magic, stored->zero_offset_rad);
    if (stored->crc != expect_crc) {
        return 0;
    }

    *zero_offset_rad_out = stored->zero_offset_rad;
    return 1;
}

HAL_StatusTypeDef calib_save(float zero_offset_rad_val)
{
    calib_data_t data;
    data.magic = CALIB_MAGIC;
    data.zero_offset_rad = zero_offset_rad_val;
    data.crc = calib_simple_crc(data.magic, data.zero_offset_rad);

    HAL_FLASH_Unlock();

    FLASH_EraseInitTypeDef erase = {0};
    uint32_t page_error = 0;
    erase.TypeErase = FLASH_TYPEERASE_PAGES;
    erase.Banks     = FLASH_BANK_1;
    erase.Page      = (CALIB_FLASH_ADDR - FLASH_BASE) / FLASH_PAGE_SIZE;
    erase.NbPages   = 1;

    HAL_StatusTypeDef st = HAL_FLASHEx_Erase(&erase, &page_error);
    if (st != HAL_OK) {
        HAL_FLASH_Lock();
        return st;
    }

    uint64_t buf[2] = {0};
    memcpy(buf, &data, sizeof(data));   // 12byte分をコピー、残りは0埋め

    uint32_t addr = CALIB_FLASH_ADDR;
    for (int i = 0; i < 2; i++) {
        st = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, addr + i * 8, buf[i]);
        if (st != HAL_OK) break;
    }

    HAL_FLASH_Lock();
    return st;
}
