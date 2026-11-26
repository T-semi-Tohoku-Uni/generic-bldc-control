#include "drive_can.h"
#include "drive_control.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#define DRIVE_CAN_ID_FIRST  0x301u
#define DRIVE_CAN_ID_LAST   0x303u
#define DRIVE_CAN_ID_FLASH  0x0801F000u
#define DRIVE_CAN_ID_MAGIC  0x43414E49u
#define USER_SW_DEBOUNCE_MS 50u
#define DRIVE_MAX_RPM 2000.0f

extern FDCAN_HandleTypeDef hfdcan1;
static volatile uint32_t drive_can_id = DRIVE_CAN_ID_FIRST;
static volatile uint8_t pending_presses;
typedef struct __attribute__((packed)) {
    float   speed_target;               /* byte0-3 */
    uint8_t pid_mode;                   /* byte4: 0=locate_pid, 1=speed_pid 1ではなぜか上手くいかないので0を用いて*/
    uint8_t control_motor_mode;         /* byte5: 0=電圧制御, 1=電流PI制御*/
    uint8_t reserved[2];                /* byte6-7 */
} can_motor_cmd_t;

static uint32_t load_can_id(void);
static HAL_StatusTypeDef save_can_id(uint32_t id);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void drive_can_button_task(void);

static uint32_t load_can_id(void)
{
    const uint64_t record = *(const volatile uint64_t *)DRIVE_CAN_ID_FLASH;
    const uint32_t id = (uint32_t)record;
    const uint32_t magic = (uint32_t)(record >> 32);

    if (magic == DRIVE_CAN_ID_MAGIC &&
        id >= DRIVE_CAN_ID_FIRST && id <= DRIVE_CAN_ID_LAST) {
        return id;
    }

    return DRIVE_CAN_ID_FIRST;
}

static HAL_StatusTypeDef save_can_id(uint32_t id)
{
    const uint64_t record = ((uint64_t)DRIVE_CAN_ID_MAGIC << 32) | id;
    FLASH_EraseInitTypeDef erase = {0};
    uint32_t page_error;

    erase.TypeErase = FLASH_TYPEERASE_PAGES;
    erase.Banks = FLASH_BANK_1;
    erase.Page = (DRIVE_CAN_ID_FLASH - FLASH_BASE) / FLASH_PAGE_SIZE;
    erase.NbPages = 1u;

    HAL_StatusTypeDef status = HAL_FLASH_Unlock();
    if (status != HAL_OK) {
        return status;
    }

    status = HAL_FLASHEx_Erase(&erase, &page_error);
    if (status == HAL_OK) {
        status = HAL_FLASH_Program(
            FLASH_TYPEPROGRAM_DOUBLEWORD, DRIVE_CAN_ID_FLASH, record);
    }

    HAL_FLASH_Lock();

    if (status == HAL_OK &&
        *(const volatile uint64_t *)DRIVE_CAN_ID_FLASH != record) {
        return HAL_ERROR;
    }
    return status;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    static uint32_t last_press_ms;
    static uint8_t has_pressed;

    if (GPIO_Pin != USER_SW_Pin) {
        return;
    }

    const uint32_t now = HAL_GetTick();
    if (has_pressed && now - last_press_ms < USER_SW_DEBOUNCE_MS) {
        return;
    }

    last_press_ms = now;
    has_pressed = 1u;

    if (pending_presses < UINT8_MAX) {
        pending_presses++;
    }
}

void drive_can_button_task(void)
{
    printf("\r\ncan_id=0x%03lX\r\n", (unsigned long)drive_can_id);

    const uint32_t mask = __get_PRIMASK();
    __disable_irq();
    const uint8_t pressed = pending_presses != 0u;
    if (pressed) {
        pending_presses--;
    }
    __set_PRIMASK(mask);

    if (!pressed || !drive_begin_can_id_change()) {
        return;
    }

    const uint32_t next = (drive_can_id == DRIVE_CAN_ID_LAST)
                        ? DRIVE_CAN_ID_FIRST : drive_can_id + 1u;

    const HAL_StatusTypeDef status = save_can_id(next);
    if (status == HAL_OK) {
        drive_can_id = next;
    }

    drive_end_can_id_change();

    if (status == HAL_OK) {
        printf("CAN ID saved: 0x%03lX\r\n", (unsigned long)next);
    } else {
        printf("CAN ID save failed\r\n");
    }
}

/* 平ドラがどのCANメッセージを受け取るかを設定し、FDCAN1を起動する関数 MX_FDCAN1_Init() の末尾にある1行で呼び出される*/
void FDCAN1_ConfigFilterAndStart(void)
{
    drive_can_id = load_can_id();

    FDCAN_FilterTypeDef sFilterConfig;

    /* 0x301～0x303は各基板に割り当てるID。この基板では設定した1つを通す。
     * 標準IDは11ビット、拡張IDは29ビット。フィルタは0番の枠に登録する。
     * DUALはFilterID1かFilterID2に一致すれば通し、両方に同じIDを設定している。
     * 一致したフレームの保存先は受信キューのRX FIFO 0。
     */
    sFilterConfig.IdType       = FDCAN_STANDARD_ID;
    sFilterConfig.FilterIndex  = 0;
    sFilterConfig.FilterType   = FDCAN_FILTER_RANGE;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    sFilterConfig.FilterID1    = DRIVE_CAN_ID_FIRST;
    sFilterConfig.FilterID2    = DRIVE_CAN_ID_LAST;

    /* 構造体で組み立てたIDフィルタをHAL経由でFDCAN1へ登録する。 */
    if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK)
    {
        Error_Handler();
    }

    /* 1つ目のFDCAN_REJECTは標準、2つ目は拡張のフィルタ不一致に適用する。
     * FDCAN_FILTER_REMOTEはリモートフレームにもIDフィルタを適用する。
     */
    if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT,
                                      FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK)
    {
        Error_Handler();
    }
    /* NEW_MESSAGEでRX FIFO 0の新着割り込みを有効にする。 HAL_FDCAN_RxFifo0Callbackを結果として呼び出す
     * 割り込みを受けたHALが下のコールバックを呼び、GetRxMessage()で取り出す。
     */
    if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
    {
        Error_Handler();
    }

    /* FDCAN1を起動 */
    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
    {
        Error_Handler();
    }
}

/* CANの受信キューから1件取り出し、使ってよい指令かを確認して、モーターの目標回転数を更新する */
/* HAL_FDCAN_ActivateNotificationの受信キュー0に新着が入ったら知らせてほしいという設定によって呼び出される */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    /* Instanceで割り込み元を確認し、&で新着イベントのビットだけを調べる。 FDCAN1かも確認 */
    if (hfdcan->Instance != FDCAN1 || (RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0) {
        return;
    }

    FDCAN_RxHeaderTypeDef RxHeader;
    uint8_t RxData[64];

    /* 受信キューから1件取り出す*/
    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
    {
        return;
    }

    /* この基板向けのClassic CAN速度指令か確認する。 設定1つ1つ確認 */
    if (RxHeader.IdType != FDCAN_STANDARD_ID || RxHeader.Identifier != drive_can_id || RxHeader.RxFrameType != FDCAN_DATA_FRAME || RxHeader.FDFormat != FDCAN_CLASSIC_CAN || RxHeader.BitRateSwitch != FDCAN_BRS_OFF || RxHeader.DataLength != FDCAN_DLC_BYTES_8) {
        return;
    }

    can_motor_cmd_t cmd;
    memcpy(&cmd, RxData, sizeof(cmd));  // 受信したバイト列を cmd にコピー 受信した8バイトを指令フィールドとして参照する。

    /* 受信値が正しいものであるかを確認 isfinite() で NaN や無限大を弾く max_speed以下か */
    if (!isfinite(cmd.speed_target) || fabsf(cmd.speed_target) > DRIVE_MAX_RPM || cmd.pid_mode > 1U || cmd.control_motor_mode > 1U) {
      return;
    }

    /* 各平ドラは自基板のIDで届いた指令を、接続された1台のモーターに渡す。 */
    drive_accept_can_command(cmd.speed_target, cmd.pid_mode, cmd.control_motor_mode);
}
