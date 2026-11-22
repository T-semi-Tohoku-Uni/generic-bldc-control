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

static struct {
    uint32_t spi_count;
    uint32_t parity_count;
    uint32_t ef_count;
    uint32_t first_ms;
    uint32_t first_spi_error;
    uint16_t first_request_rx;
    uint16_t first_response_rx;
    HAL_StatusTypeDef first_request_status;
    HAL_StatusTypeDef first_response_status;
    bool first_saved;
} encoder_diag;

// コードに出る値	データシート上の意味	このコードでの使い方
// 0xFFFF	角度レジスタ ANGLECOM（番地 0x3FFF）の読み出し命令	モーター軸の角度を要求する。角度データは下位14ビットで、`0〜16383` が1回転に対応する。
// 0xC000	NOP レジスタの読み出し。AS5047Pでは何もしない命令と同じ	SPIの応答は1フレーム遅れるので、これを送る間に直前の命令への応答を受け取る。
// 受信の bit 15	偶数パリティ	受信した16ビット全体で、`1` の個数が偶数か確認する。
// 0x4000	受信の bit 14 を取り出すマスク。bit 14 は EF	EF=1 なら、AS5047P側が通信エラーを検出している。
// 0x4001	エラーレジスタ ERRFL（番地 0x0001）の読み出し命令	エラーの内訳を取得する。読むと記録がクリアされるので、値をログに残す。
// ERRFL = 0x0004	bit 2 の PARERR が1	今回出たエラー。AS5047P側が受け取った命令のパリティ異常を検出した。bit 1 は INVCOMM（無効アクセス）、bit 0 は FRERR（フレーム異常）。
// 0xFFFC	診断レジスタ DIAAGC（番地 0x3FFC）の読み出し命令	センサーの準備状態と磁界・内部演算の異常を確認する。
// (diag_frame & 0x0F00U) == 0x0100U	bit 8〜11のうち、準備完了 LF だけが1	LF=1 かつ COF=0、`MAGH=0`、`MAGL=0` なら、起動チェックのセンサー条件を通す

static HAL_StatusTypeDef spi_transfer_16(uint16_t tx, uint16_t *rx, uint32_t timeout)
{
    HAL_GPIO_WritePin(SPI1_SS_GPIO_Port, SPI1_SS_Pin, GPIO_PIN_RESET);
    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(
        &hspi1, (const uint8_t *)&tx, (uint8_t *)rx, 1U, timeout);
    HAL_GPIO_WritePin(SPI1_SS_GPIO_Port, SPI1_SS_Pin, GPIO_PIN_SET);
    return status;
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

/* 起動時専用。1フレームあたり2 msのHALタイムアウトで読む。 command は送る16ビットの命令です。frameは受信結果を書き込む先*/
static bool as5047p_startup_read(uint16_t command, uint16_t *frame)
{
    uint16_t previous_response = 0U; // 前の命令への応答を一時的に受け取る変数

    // 1回目はコマンドを送り、2回目でその応答を受け取る。
    if (spi_transfer_16(command, &previous_response, 2U) != HAL_OK || spi_transfer_16(0xC000U, frame, 2U) != HAL_OK) {
        return false;
    }

    return !as5047p_parity_bad(*frame); 
}

bool as5047p_startup_check(void)
{
    unsigned int consecutive_ok = 0U;  // 正常な読み取りが何回連続したかを数える
    bool errfl_read = false;           //  この起動チェック中に ERRFL を読んだかを覚える    

    for (unsigned int attempt = 0U; attempt < 8U; ++attempt) {
        uint16_t angle_frame = 0U;
        uint16_t diag_frame = 0U;

        const bool angle_ok = as5047p_startup_read(0xFFFFU, &angle_frame); // 角度レジスタを読む0x3FFF 角度レジスタの番地0x7FFF   bit 14を1にする → 読み出しを指定。1が15個 0xFFFF   bit 15も1にする → 1が16個になり、偶数
        const bool diag_ok  = as5047p_startup_read(0xFFFCU, &diag_frame);  // 診断レジスタを読む 0x3FFC   診断レジスタの番地 0x7FFC   bit 14を1にする → 読み出しを指定。1が13個 0xFFFC   bit 15も1にする → 1が14個になり、偶数

        // SPI転送とパリティが正常な応答のEFだけを判定に使う。
        const bool ef = (angle_ok && (angle_frame & 0x4000U) != 0U) || (diag_ok  && (diag_frame  & 0x4000U) != 0U); // & 0x4000Uで通信エラー通知があるか分かる

        if (ef && !errfl_read) {  // ef が true なら、AS5047Pが通信エラー !errfl_readならこの起動チェックでは、まだ ERRFL を読んでいない
            uint16_t errfl = 0U;
            errfl_read = true;

            // ERRFLを読むとセンサー側の通信エラーフラグが消える。
            if (!as5047p_startup_read(0x4001U, &errfl)) { // 0x4001U はERRFL を読んでという命令
                printf("ENC START: ERRFL read failed\r\n");
                return false;
            }

            printf("ENC START: ERRFL=0x%04X PARERR=%u INVCOMM=%u FRERR=%u\r\n", (unsigned int)(errfl & 0x3FFFU), (unsigned int)((errfl >> 2) & 1U), (unsigned int)((errfl >> 1) & 1U), (unsigned int)(errfl & 1U));
        }

        // DIAAGCはLF=1、COF=0、MAGH=0、MAGL=0を正常とする。
        const bool sensor_ready = (diag_frame & 0x0F00U) == 0x0100U;

        if (angle_ok && diag_ok && !ef && sensor_ready) {
            ++consecutive_ok;
            if (consecutive_ok == 3U) {
                printf("ENC START: OK\r\n");
                return true;
            }
        } else {
            consecutive_ok = 0U;
            printf("ENC START: try=%u angle_ok=%u angle=0x%04X ""diag_ok=%u diag=0x%04X\r\n",attempt + 1U, (unsigned int)angle_ok, (unsigned int)angle_frame, (unsigned int)diag_ok, (unsigned int)diag_frame);
        }

        HAL_Delay(1U);
    }

    return false;
}

uint16_t as5047p_read_angle_checked(uint8_t *fault_out)
{
    uint16_t request_rx = 0U;
    uint16_t frame = 0U;
    // 角度を要求し、次の16bitフレームで角度の応答を受け取る。
    HAL_StatusTypeDef request = spi_transfer_16(0xFFFFU, &request_rx, HAL_MAX_DELAY);
    uint32_t spi_error = request == HAL_OK ? 0U : HAL_SPI_GetError(&hspi1);
    HAL_StatusTypeDef response = spi_transfer_16(0xC000U, &frame, HAL_MAX_DELAY);
    if (response != HAL_OK) {
        spi_error |= HAL_SPI_GetError(&hspi1);
    }

    bool spi_bad = request != HAL_OK || response != HAL_OK;
    bool parity_bad = !spi_bad && as5047p_parity_bad(frame);
    bool ef = !spi_bad && !parity_bad && (frame & 0x4000U) != 0U;
    *fault_out = spi_bad || parity_bad || ef;

    if (*fault_out) {
        encoder_diag.spi_count += spi_bad;
        encoder_diag.parity_count += parity_bad;
        encoder_diag.ef_count += ef;
        if (!encoder_diag.first_saved) {
            encoder_diag.first_ms = HAL_GetTick();
            encoder_diag.first_spi_error = spi_error;
            encoder_diag.first_request_rx = request_rx;
            encoder_diag.first_response_rx = frame;
            encoder_diag.first_request_status = request;
            encoder_diag.first_response_status = response;
            encoder_diag.first_saved = true;
        }
    }

    return frame & 0x3FFFU;
}

void as5047p_report_stopped_fault(void)
{
    if (!encoder_diag.first_saved) return;

    printf("ENC FIRST t=%lu ms req_hal=%d req_rx=0x%04X resp_hal=%d resp_rx=0x%04X "
           "spi_error=0x%08lX counts spi=%lu parity=%lu ef=%lu\r\n",
           (unsigned long)encoder_diag.first_ms,
           (int)encoder_diag.first_request_status, (unsigned int)encoder_diag.first_request_rx,
           (int)encoder_diag.first_response_status, (unsigned int)encoder_diag.first_response_rx,
           (unsigned long)encoder_diag.first_spi_error, (unsigned long)encoder_diag.spi_count,
           (unsigned long)encoder_diag.parity_count, (unsigned long)encoder_diag.ef_count);

    if (encoder_diag.ef_count == 0U) return;

    // 停止後に一度だけ読んで記録する。ERRFLは読み出しでクリアされる。
    uint16_t ignored = 0U;
    uint16_t frame = 0U;
    HAL_StatusTypeDef request = spi_transfer_16(0x4001U, &ignored, 2U);
    HAL_StatusTypeDef response = request == HAL_OK ? spi_transfer_16(0xC000U, &frame, 2U) : request;
    if (request != HAL_OK || response != HAL_OK || as5047p_parity_bad(frame)) {
        printf("ENC ERRFL read failed: req_hal=%d resp_hal=%d rx=0x%04X\r\n",
               (int)request, (int)response, (unsigned int)frame);
        return;
    }

    printf("ENC ERRFL=0x%04X PARERR=%u INVCOMM=%u FRERR=%u\r\n",
           (unsigned int)(frame & 0x3FFFU), (unsigned int)((frame >> 2) & 1U),
           (unsigned int)((frame >> 1) & 1U), (unsigned int)(frame & 1U));
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
