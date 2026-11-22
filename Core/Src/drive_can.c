#include "drive_can.h"
#include "drive_control.h"

#include <math.h>
#include <string.h>

extern FDCAN_HandleTypeDef hfdcan1;

#define CAN_MOTOR_CMD_BASE_ID 0x302u
#define DRIVE_MAX_RPM 2000.0f

typedef struct __attribute__((packed)) {
    float   speed_target;               /* byte0-3 */
    uint8_t pid_mode;                   /* byte4: 0=locate_pid, 1=speed_pid 1ではなぜか上手くいかないので0を用いて*/
    uint8_t control_motor_mode;         /* byte5: 0=電圧制御, 1=電流制御 基本的に0を用いて*/
    uint8_t reserved[2];                /* byte6-7 */
} can_motor_cmd_t;

/* 平ドラがどのCANメッセージを受け取るかを設定し、FDCAN1を起動する関数 MX_FDCAN1_Init() の末尾にある1行で呼び出される*/
void FDCAN1_ConfigFilterAndStart(void)
{
    FDCAN_FilterTypeDef sFilterConfig;

    /* 0x301～0x303は各基板に割り当てるID。この基板では設定した1つを通す。
     * 標準IDは11ビット、拡張IDは29ビット。フィルタは0番の枠に登録する。
     * DUALはFilterID1かFilterID2に一致すれば通し、両方に同じIDを設定している。
     * 一致したフレームの保存先は受信キューのRX FIFO 0。
     */
    sFilterConfig.IdType       = FDCAN_STANDARD_ID;
    sFilterConfig.FilterIndex  = 0;
    sFilterConfig.FilterType   = FDCAN_FILTER_DUAL;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    sFilterConfig.FilterID1    = CAN_MOTOR_CMD_BASE_ID;
    sFilterConfig.FilterID2    = CAN_MOTOR_CMD_BASE_ID;

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
    if (RxHeader.IdType != FDCAN_STANDARD_ID || RxHeader.Identifier != CAN_MOTOR_CMD_BASE_ID || RxHeader.RxFrameType != FDCAN_DATA_FRAME || RxHeader.FDFormat != FDCAN_CLASSIC_CAN || RxHeader.BitRateSwitch != FDCAN_BRS_OFF || RxHeader.DataLength != FDCAN_DLC_BYTES_8) {
        return;
    }

    can_motor_cmd_t cmd;
    memcpy(&cmd, RxData, sizeof(cmd));  // 受信したバイト列を cmd にコピー 受信した8バイトを指令フィールドとして参照する。

    /* 受信値が正しいものであるかを確認 isfinite() で NaN や無限大を弾く max_speed以下か */
    if (!isfinite(cmd.speed_target) || fabsf(cmd.speed_target) > DRIVE_MAX_RPM || cmd.pid_mode > 1U || cmd.control_motor_mode != 0U) {
        return;
    }

    /* 各平ドラは自基板のIDで届いた指令を、接続された1台のモーターに渡す。 */
    drive_accept_can_command(cmd.speed_target, cmd.pid_mode, cmd.control_motor_mode);
}
