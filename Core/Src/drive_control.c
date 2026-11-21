#include "drive_control.h"
#include "drive_hardware.h"
#include "drive_pid.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern FDCAN_HandleTypeDef hfdcan1;
extern OPAMP_HandleTypeDef hopamp1;
extern OPAMP_HandleTypeDef hopamp2;
extern OPAMP_HandleTypeDef hopamp3;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;

#define PWM_PERIOD 3999  // TIM1が上向きに数えるときの上限。
#define POLE_PAIRS 7     // ローター上のN極・S極の組の数。
#define CURRENT_SHUNT_OHMS 0.001    //電流計測に用いる抵抗値
#define CURRENT_OPAMP_GAIN 16.0                 //cudemxで設定した値
#define MAX_PHASE_CURRENT_A 30.0f
#define ADC_REFERENCE_V 3.3               //マイコンの駆動電圧
#define ENCODER_FAULT_THRESHOLD 30    // スコアがここまで貯まったら停止
#define ENCODER_FAULT_SCORE_MAX 30    // スコアの上限
#define DRIVE_CAN_TIMEOUT_MS 300U

typedef enum {
    DRIVE_BOARD_V2,
    DRIVE_BOARD_V3
} DriveBoardMode;

/* 実際に接続する基板に合わせて選ぶ */
static const DriveBoardMode motor_board_mode = DRIVE_BOARD_V3;

/* 各基板のシャント信号の伝達率。電流計測と過電流判定の換算に */
#define LEGACY_CURRENT_SENSE_RATIO 1.0f
#define V3_CURRENT_SENSE_RATIO 0.94f

volatile float motor_speed;
float motor_p;
float motor_i;
float motor_d;
volatile float motor_speed_target;
volatile float motor_now_speed_target;
volatile float motor_different_sum;
volatile float motor_low_pass_different_sum;
volatile float motor_last_difference;
volatile float motor_last_input;
volatile float motor_low_pass_derivative;
volatile float motor_last_last_difference;
volatile float motor_current_d;
volatile float motor_current_p;
volatile float motor_current_d_target;
volatile float motor_current_p_target;
volatile float motor_current_d_different_sum;
volatile float motor_current_d_low_pass_different_sum;
volatile float motor_current_d_last_difference;  //currentは電流といういみ　現在ではない
volatile float motor_current_p_different_sum;
volatile float motor_current_p_low_pass_different_sum;
volatile float motor_current_p_last_difference;
volatile float motor_current_d_pgain;
volatile float motor_current_d_igain;
volatile float motor_current_p_pgain;
volatile float motor_current_p_igain;

static inline float fast_sin(float x) { return sinf(x); }
static inline float fast_cos(float x) { return cosf(x); }

volatile int pid_mode = 0;             //0ならlocate_pid 1ならspeed_pid

/* 制御方式 */
typedef enum {
    CONTROL_VOLTAGE = 0,
    CONTROL_CURRENT_PI = 1
} DriveControlMode;

/* CAN指令を待つか、コードで指定した目標で単体試験するか */
typedef enum {
    DRIVE_COMMAND_CAN,
    DRIVE_COMMAND_LOCAL
} DriveCommandSource;

static const DriveCommandSource drive_command_source = DRIVE_COMMAND_LOCAL;
static const DriveControlMode control_motor_mode = CONTROL_CURRENT_PI;

volatile uint32_t rx_ok_count = 0;

/* The first valid CAN command arms PWM; a timeout holds the STOP state. */
enum {
    DRIVE_LOCAL,     // 単体試験。コードで設定した目標回転数を使う。
    DRIVE_CAN_WAIT,  // 最初の有効なCAN指令を待つ。
    DRIVE_CAN_ARM,   // 最初の指令を受理。次のTIM2周期でPWMを有効にする準備。
    DRIVE_CAN_RUN,   // CAN指令に従って速度を制御する。
    DRIVE_CAN_STOP   // 通信断でPWMを止め、リセットまで保持する。
};
static volatile uint8_t drive_state;
static volatile uint32_t drive_last_rx_ms;
static volatile uint32_t tim2_cnt = 0;

volatile uint8_t encoder_fault = 0;
static int16_t encoder_fault_score = 0;

/*電流値　単位はアンペア*/
volatile float current_u = 0.0f;
volatile float current_v = 0.0f;
volatile float current_w = 0.0f;
volatile uint8_t overcurrent_fault = 0;

//現在の速度を計算する？ための関数
volatile float speed_now = 0;

/*実際にADCで読み取った値*/
uint32_t offset_u = 2048;
uint32_t offset_v = 2048;
uint32_t offset_w = 2048;

volatile int cutoff;
static float electrical_direction = 0.0f;
static float zero_offset_rad = 0.0f;  //初期位置のずれを確認する

uint16_t angle_raw;

static volatile uint32_t adc_done_count;
static volatile uint32_t adc_observed_u;
static volatile uint32_t adc_observed_w;
static volatile uint32_t adc_u_min = 4095U;
static volatile uint32_t adc_u_max = 0U;
static volatile uint32_t adc_w_min = 4095U;
static volatile uint32_t adc_w_max = 0U;

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance != ADC1) {
        return;
    }

    adc_observed_u = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_1);
    adc_observed_w = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_2);
    adc_done_count++;

    // ADCコールバック内で、adc_observed_u / adc_observed_w を更新した直後
    uint32_t u = adc_observed_u;
    uint32_t w = adc_observed_w;

    if (u < adc_u_min) adc_u_min = u;
    if (u > adc_u_max) adc_u_max = u;
    if (w < adc_w_min) adc_w_min = w;
    if (w > adc_w_max) adc_w_max = w;
}

static void set_pwm(float ua, float ub, float uc)
{
    /*各相の電圧指令値は0から1であるので不正な値を消去 ua・ub・uc は、U・V・Wそれぞれの 0〜1に正規化した指令*/
    if (ua < 0.0f) ua = 0.0f;
    if (ua > 1.0f) ua = 1.0f;
    if (ub < 0.0f) ub = 0.0f;
    if (ub > 1.0f) ub = 1.0f;
    if (uc < 0.0f) uc = 0.0f;
    if (uc > 1.0f) uc = 1.0f;

    /*CCRがCNT(0からPWM_PERIODまでの高速カウント)より大きい時low、それ以外はhighとなる*/
    TIM1->CCR1 = (uint32_t)(ua * PWM_PERIOD);
    TIM1->CCR2 = (uint32_t)(ub * PWM_PERIOD);
    TIM1->CCR3 = (uint32_t)(uc * PWM_PERIOD);
}

static void motor_emergency_stop(void)
{
    set_pwm(0.5f, 0.5f, 0.5f);

    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);     // TIM1 の CH1  を無効にする
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);  // TIM1 の CH1N を無効にする
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);
}

void drive_accept_can_command(float speed_target, uint8_t requested_pid_mode, uint8_t requested_control_mode)
{
    /* LOCALでは単体試験の目標、STOPでは通信断停止を保持する。
     * WAITは最初の指令を待つ状態。受理済みの状態で300 ms経過したら、
     * TIM2がSTOPにする前であっても新しい指令は受理しない。
     */
    uint32_t now = HAL_GetTick();
    uint8_t state = drive_state;

    if (requested_control_mode != (uint8_t)control_motor_mode ||
        state == DRIVE_LOCAL || state == DRIVE_CAN_STOP ||
        (state != DRIVE_CAN_WAIT && now - drive_last_rx_ms >= DRIVE_CAN_TIMEOUT_MS)) {
        return;
    }

    motor_speed_target = speed_target;
    pid_mode = requested_pid_mode ? 1 : 0;

    /* 前回の値を確認し、次この関数が呼ばれた時のnowと比較することによってtimeoutでないか確認 */
    drive_last_rx_ms = now;
    /* DRIVE_CAN_WAIT は起動後の最初の指令を待つ状態 */
    if (state == DRIVE_CAN_WAIT) {
        drive_state = DRIVE_CAN_ARM;
    }
    rx_ok_count++;
}

static void measure_current(void)
{
  /*HAL_ADCEx_InjectedGetValueはADCが直近の変換で保存した値を読み出すHAL関数 基板では1番目がU相、2番目がW相 */
  uint32_t raw_u = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
  uint32_t raw_w = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2);

  /* ADCの1カウントが何アンペアか計算 */
  float sense_ratio = motor_board_mode == DRIVE_BOARD_V3 ? V3_CURRENT_SENSE_RATIO : LEGACY_CURRENT_SENSE_RATIO;          // V3なら 0.94、旧基板なら 1.0
  float magnification_conversion = (ADC_REFERENCE_V / 4096.0f) / (CURRENT_OPAMP_GAIN * CURRENT_SHUNT_OHMS * sense_ratio);// ADC_REFERENCE_V / 4096.0fが	ADCの1カウントに相当する電圧で16 × 0.001 × sense_ratioが相電流が1 A変わったとき、ADC入力の電圧が何V変わるか

  /* 実際の電流値を計算 offsetは0Aの時のADCの値 */
  current_u = ((float)raw_u - (float)offset_u) * magnification_conversion;
  current_w = ((float)raw_w - (float)offset_w) * magnification_conversion;
  current_v = -(current_u + current_w);   // u + v + w = 0より

  /*各相電流の絶対値が閾値を超えたら即座にPWM停止 */
  if (!overcurrent_fault && (fabsf(current_u) > MAX_PHASE_CURRENT_A ||  fabsf(current_v) > MAX_PHASE_CURRENT_A || fabsf(current_w) > MAX_PHASE_CURRENT_A)) {
      overcurrent_fault = 1;
      motor_emergency_stop();
  }

  // Clarke変換
  /* u + v + wを固定された直角の2軸 α・βに変換 */
  float i_alpha = current_u;
  float i_beta  = (current_u + 2.0f * current_v) * 0.57735f;
  /* Id は磁石の向きに沿った電流 Iqはそこから電気角で90°ずれた方向の電流で、主にトルクを作成 */
  float id =  i_alpha * fast_cos(electrical_direction) + i_beta * fast_sin(electrical_direction);
  float iq = -i_alpha * fast_sin(electrical_direction) + i_beta * fast_cos(electrical_direction);

  motor_current_d = id;
  motor_current_p = iq;
}

void test_openloop_spin_veryslow(void)
{
    static float test_angle = 0.0f;
    float test_vq = 0.15f;

    test_angle += 0.0002f;
    if (test_angle > 2.0f*M_PI) test_angle -= 2.0f*M_PI;

    float va = -test_vq * fast_sin(test_angle);
    float vb =  test_vq * fast_cos(test_angle);
    float u = va;
    float v = -0.5f*va + 0.866f*vb;
    float w = -0.5f*va - 0.866f*vb;
    set_pwm(u+0.5f, v+0.5f, w+0.5f);

}

static void update_openloop(float voltage)
{

    static float filtered_speed = 0.0;
    static int speed_calc_cnt = 0;
    static uint16_t angle_at_last_calc = 0;
    static uint8_t speed_initialized = 0;

    uint8_t enc_bad = 0;
    angle_raw = as5047p_read_angle_checked(&enc_bad); //　読み取りに異常があれば関数内で enc_bad に 1 が書き込まれる

    /* エンコーダーの異常は、1回なら更新を飛ばし、積み重なるとPWMを止める */
    if (enc_bad) {
        if (encoder_fault_score < ENCODER_FAULT_SCORE_MAX) encoder_fault_score++;
    } else {
        if (encoder_fault_score > 0) encoder_fault_score--;
    }

    if (encoder_fault_score >= ENCODER_FAULT_THRESHOLD && !encoder_fault) {
        encoder_fault = 1;
        motor_emergency_stop();
    }

    if (enc_bad || encoder_fault) {
      if (control_motor_mode == CONTROL_CURRENT_PI) {
        measure_current();
      }
      return;
    }

    /* 最初の正常な角度を、速度計算の基準にする */
    if (!speed_initialized) {
      angle_at_last_calc = angle_raw;
      speed_calc_cnt = 0;
      filtered_speed = 0.0f;
      speed_now = 0.0f;
      motor_speed = 0.0f;

      speed_initialized = 1;
      return;  /* 初回は速度差分を計算しない */
    }

    float angle_mech = ((float)angle_raw / 16384.0f) * 2.0f * M_PI;                      // angle_mech は軸が実際に回った機械角
    static float motor_direction = 1.0f;
    electrical_direction = (angle_mech - zero_offset_rad) * POLE_PAIRS * motor_direction;// zero_offset_rad を引き、極対数7を掛けて電気角を作成 現在角 angle_mech からその基準角を引けば、基準位置で差はゼロになります。そこから現行の POLE_PAIRS = 7 を掛けると、軸が基準から何度動き、磁極の並びとして何度分進んだかを求められる

    // 速度計算は25回に1回(5ms間隔)だけ行う
    speed_calc_cnt++;
    if (speed_calc_cnt >= 25) {
        speed_calc_cnt = 0;
        int16_t diff = (int16_t)angle_raw - (int16_t)angle_at_last_calc;
        if (diff > 8192)  diff -= 16384;
        if (diff < -8192) diff += 16384;
        angle_at_last_calc = angle_raw;

        speed_now = ((float)diff / 16384.0f) / (DRIVE_CONTROL_PERIOD_S * 25.0f) * 60.0f;
        filtered_speed = filtered_speed * 0.7f + speed_now * 0.3f;
        motor_speed = filtered_speed;
    }

    float vd;
    float vq;
    /*電圧制御*/
    if (control_motor_mode == CONTROL_VOLTAGE) {

      /* 速度PIDの出力をq軸の電圧指令にし、後段で三相へ座標変換する。 */
      vd = 0.0f;      // d軸は、モーターの中心からローターの磁石が向いている方向へ伸びる半径方向 電圧制御では0にしているが電流制御ではここを変更することによって効率を上げることができる
      vq = voltage;   // q軸は、d軸に対して電気角で90°ずれた方向 回す力を出すための電圧指令といえる

    }

    /*電流制御*/
    if (control_motor_mode == CONTROL_CURRENT_PI){
        const float voltage_limit = 0.10f; //単体試験用の正規化電圧上限

        measure_current();
        if (overcurrent_fault) {
          return;
        }

        const float id_sum_before = motor_current_d_different_sum;
        const float iq_sum_before = motor_current_p_different_sum;

        vd = locate_pid( motor_current_d, motor_current_d_target, motor_current_d_pgain, motor_current_d_igain, 0.0f, &motor_current_d_different_sum, &motor_current_d_low_pass_different_sum, &motor_current_d_last_difference, cutoff);
        vq = locate_pid( motor_current_p, motor_current_p_target, motor_current_p_pgain, motor_current_p_igain, 0.0f, &motor_current_p_different_sum, &motor_current_p_low_pass_different_sum, &motor_current_p_last_difference, cutoff);

        /* 電圧が上限に達しているとき、PIの積分がさらに溜まりすぎるのを防ぐ処理 pidが正しい方向にいっているか確認 */
        const float voltage_sq = vd * vd + vq * vq;

        if (voltage_sq > voltage_limit * voltage_limit) {
          const float delta_vd_i = motor_current_d_igain * (motor_current_d_different_sum - id_sum_before);
          const float delta_vq_i = motor_current_p_igain * (motor_current_p_different_sum - iq_sum_before);

          if (vd * delta_vd_i + vq * delta_vq_i > 0.0f) {
            motor_current_d_different_sum = id_sum_before;
            motor_current_p_different_sum = iq_sum_before;
          }

          /* vd と vq を同じ倍率で縮めて、合成した電圧指令を上限ちょうどに収めている */
          const float scale = voltage_limit / sqrtf(voltage_sq);
          vd *= scale;
          vq *= scale;
        }
    }

    /* ローターと一緒に回るd軸・q軸をモーターの固定側にあるα軸・β軸に変換 α軸がU相の向きで、β軸はそこから電気角で90°*/
    float va = vd * fast_cos(electrical_direction) - vq * fast_sin(electrical_direction);
    float vb = vd * fast_sin(electrical_direction) + vq * fast_cos(electrical_direction);

    /* va, vbをu,v,wに変換*/
    float u = va;
    float v = -0.5f * va + 0.866f * vb;
    float w = -0.5f * va - 0.866f * vb;

    float offset = 0.5f;
    set_pwm(u + offset, v + offset, w + offset);  // 0.5を基準としてPWMの出力に

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

    if (htim->Instance == TIM2)
    {
        tim2_cnt++;

        uint32_t mask = __get_PRIMASK();// 今の割り込みの設定を保存
        __disable_irq();                // ここから通常の割り込みを一時的に止める
        uint8_t state = drive_state;
        uint8_t timed_out = (state == DRIVE_CAN_ARM || state == DRIVE_CAN_RUN) && HAL_GetTick() - drive_last_rx_ms >= DRIVE_CAN_TIMEOUT_MS;
        if (timed_out) {
            drive_state = DRIVE_CAN_STOP;
        }
        __set_PRIMASK(mask);    // 保存しておいた設定に戻す

        if (timed_out) {
            motor_emergency_stop();
            motor_speed_target = 0.0f;
            motor_now_speed_target = 0.0f;
            /* DRIVE_CAN_STOP stays latched until reset, which reinitializes the PID. */
        }
        if (timed_out || state == DRIVE_CAN_WAIT || state == DRIVE_CAN_STOP) {
            return;
        }

        if (overcurrent_fault || encoder_fault) {
            return;
        }

        if (state == DRIVE_CAN_ARM) {
            __HAL_TIM_MOE_ENABLE(&htim1);
            drive_state = DRIVE_CAN_RUN;
        }

        if (control_motor_mode == CONTROL_CURRENT_PI) {
          update_openloop(0.0f);
          return;
        }

        /*最初電流差でがたがたいうので速度目標を少しずつ上げることで回避したい*/
        if (motor_speed_target == 0.0f) {
          /* ドリフト防止 */
          if (motor_now_speed_target < motor_speed_target) {
            motor_now_speed_target += 0.05f;
            if (motor_now_speed_target > motor_speed_target) {
              motor_now_speed_target = motor_speed_target;
            }
          }
          if (motor_now_speed_target > motor_speed_target) {
            motor_now_speed_target -= 0.05f;
            if (motor_now_speed_target < motor_speed_target) {
              motor_now_speed_target = motor_speed_target;
            }
          }
        }
        else if (motor_now_speed_target < motor_speed_target) {
          motor_now_speed_target += 0.03f;
          if (motor_now_speed_target > motor_speed_target) {
            motor_now_speed_target = motor_speed_target;
          }
        } else if (motor_now_speed_target > motor_speed_target) {
          motor_now_speed_target -= 0.03f;
          if (motor_now_speed_target < motor_speed_target) {
            motor_now_speed_target = motor_speed_target;
          }
        }

        float voltage_out = 0.0f;  // ← スコープを外に出す

        if (pid_mode == 0) {
          voltage_out = locate_pid( motor_speed, motor_now_speed_target, motor_p, motor_i, motor_d, &motor_different_sum, &motor_low_pass_different_sum, &motor_last_difference, cutoff);
        }

        if (pid_mode == 1) {
            voltage_out = speed_pid( motor_speed, motor_now_speed_target, motor_p, motor_i, motor_d, &motor_low_pass_different_sum, &motor_last_difference, &motor_last_last_difference, motor_last_input, &motor_low_pass_derivative, cutoff);
            motor_last_input = voltage_out;
        }

        // locate_pidとspeed_pidの出力は-16384〜16384なので-0.3〜0.3にスケール変換
        float voltage = voltage_out / 16384.0f * 0.3f;

        update_openloop(voltage);

        measure_current();   //電流測定
    }
}

/* motorの初期化をする関数 */
static void initialize_motor_variables(void)
{
  motor_speed = 0.0;
  motor_speed_target = 0.0;
  if (pid_mode == 0) {
    /* N2830を使用していたときの速度PID設定（参考）
     * motor_p = 37.0;
     * motor_i = 2.0;
     * motor_d = 1.4;
     */
    /* 現在のN5065用速度PID設定。 */
    if(control_motor_mode == CONTROL_VOLTAGE){
      motor_p=35;
      motor_i=1.8;
      motor_d=1.4;
    }
  }
  if (pid_mode == 1) {
    if(control_motor_mode == CONTROL_VOLTAGE){
      motor_p=1.0;
      motor_i=0.05;
      motor_d=0.0;
    }
  }
  motor_now_speed_target = 0.0;
  motor_different_sum = 0.0;
  motor_low_pass_different_sum = 0.0;
  motor_last_difference = 0.0;
  motor_last_input = 0.0;
  motor_low_pass_derivative = 0.0;
  motor_last_last_difference = 0.0;
  motor_current_d = 0.0;
  motor_current_p = 0.0;
  motor_current_d_target = 0.0;
  motor_current_p_target = 2.0f;  // Iqの初期目標 [A]
  motor_current_d_different_sum = 0.0;
  motor_current_d_low_pass_different_sum = 0.0;
  motor_current_d_last_difference = 0.0;      //currentは電流といういみ　現在ではない
  motor_current_p_different_sum = 0.0;
  motor_current_p_low_pass_different_sum = 0.0;
  motor_current_p_last_difference = 0.0;
  if (control_motor_mode == CONTROL_CURRENT_PI) {
    motor_current_d_pgain = 0.01f;
    motor_current_d_igain = 0.10f;
    motor_current_p_pgain = 0.01f;
    motor_current_p_igain = 0.10f;
  }
  cutoff = 8;
}

static void drive_startup_failed(const char *reason)
{
  motor_emergency_stop();
  printf("%s\r\n", reason);
  fflush(stdout);

  while (1) {
    HAL_Delay(100);
  }
}

static void start_motor_pwm(void)
{
  if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK ||     // U相の上側FETへのPWM信号
      HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1) != HAL_OK ||  // U相の下側FETへの相補PWM信号
      HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2) != HAL_OK ||
      HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2) != HAL_OK ||
      HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3) != HAL_OK ||
      HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3) != HAL_OK) {
    drive_startup_failed("PWM START FAILED");
  }
}

static void start_current_measurement(void)
{
  // ADC回路自体の誤差を校正する。電流のゼロ点は後で測る。
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
  HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);

  printf("step6: ADC calibrated\r\n");

  // OPAMPスタート アナログ信号を増幅する回路を開始
  HAL_OPAMP_Start(&hopamp1);
  HAL_OPAMP_Start(&hopamp2);
  HAL_OPAMP_Start(&hopamp3);

  HAL_Delay(2);

  printf("step7: OPAMP started\r\n");

  // 設定済みのInjected変換を割り込みなしで開始し、ゼロ点を測る。
  HAL_ADCEx_InjectedStart(&hadc2);
  HAL_ADCEx_InjectedStart(&hadc1);

  printf("step8: injected ADC started\r\n");

  /*offsetを計算するための変数*/
  uint32_t sum_u = 0.0;
  uint32_t sum_v = 0.0;
  uint32_t sum_w = 0.0;

  for (int i = 0; i < 100; i++) {
    HAL_Delay(1); // 次のADC測定を待ち、指定したRank（測定順）の結果を読む。
    sum_u += HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
    sum_v += HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1);
    sum_w += HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2);
  }

  offset_u = sum_u / 100;
  offset_v = sum_v / 100;
  offset_w = sum_w / 100;

  // V3は通電前に、走行中の電流計測で使うU相とW相のゼロ点を確認する。
  if (motor_board_mode == DRIVE_BOARD_V3 && (offset_u == 0U || offset_w == 0U)) {
    printf("OFFSET FAILED: U=%lu V=%lu W=%lu\r\n", offset_u, offset_v, offset_w);
    drive_startup_failed("MOTOR STARTUP ABORTED");
  }

  printf("Offset U:%lu, V:%lu, W:%lu\r\n", offset_u, offset_v, offset_w);
  printf("step9: offset measured U:%lu V:%lu W:%lu\r\n", offset_u, offset_v, offset_w);

  // Injected変換をいったん止める。
  HAL_ADCEx_InjectedStop(&hadc1);
  HAL_ADCEx_InjectedStop(&hadc2);

  // ゼロ点を求めた後、変換完了割り込み付きで再開する。
  HAL_ADCEx_InjectedStart_IT(&hadc2);
  HAL_ADCEx_InjectedStart_IT(&hadc1);

  printf("step10: injected IT started\r\n");
}

static void align_motor_encoder(void)
{
  /* 起動時の原点合わせ。現在は毎回ローターを固定して電気角のゼロ点を測る。
   * 旧方式ではcalib_load(&loaded_zero_offset)に成功すると、Flashの校正値を
   * zero_offset_radに設定し、読み出せない場合だけ原点合わせをしていた。
   */
  uint8_t align_bad_before = 0;
  uint8_t align_bad_after = 0;

  uint16_t align_before = as5047p_read_angle_checked(&align_bad_before);

  /* U相とV・W相に差を作ると、ローターが磁界の向きにそろう。1秒後のAS5047Pの角度を読み、通信とドライバ状態が正常なら、その位置を zero_offset_rad に */
  set_pwm(0.58f, 0.46f, 0.46f);  // U相とV・W相に電圧差を作り、ローターを一定方向に固定する。
  HAL_Delay(1000);               // 落ち着くまで1秒待ち、その位置を電気角のゼロ点にする。

  uint16_t align_after = as5047p_read_angle_checked(&align_bad_after);

  uint8_t align_status = 0xFF;
  HAL_StatusTypeDef align_i2c = stspin_read_reg(0x80, &align_status);

  printf("ALIGN before=%u after=%u bad_before=%u bad_after=%u " "I2C=%d STATUS=0x%02X\r\n", (unsigned int)align_before, (unsigned int)align_after, (unsigned int)align_bad_before, (unsigned int)align_bad_after, (int)align_i2c, (unsigned int)align_status);
  fflush(stdout);

  /* 通信・ドライバ異常がある状態では原点を採用しない */
  if (align_bad_before || align_bad_after || align_i2c != HAL_OK || (align_status & 0x0F)) {

    motor_emergency_stop();
    printf("ALIGN FAILED: stopped\r\n");
    fflush(stdout);

    while (1) {
        HAL_Delay(100);
    }
  }

  zero_offset_rad = ((float)align_after / 16384.0f) * 2.0f * M_PI;    // 16384は一周分の目盛りの数、2πを掛けてラジアンにしている

  if (calib_save(zero_offset_rad) == HAL_OK) {
      printf("step11: FCO lock done, saved to flash, zero_offset_rad=%d\r\n", (int)(zero_offset_rad * 1000));
  } else {
      printf("step11: FCO lock done, FLASH SAVE FAILED, zero_offset_rad=%d\r\n", (int)(zero_offset_rad * 1000));
  }

  /* ロックが実際に効いているかの確認は、ロックした場合のみ実施 */
  uint16_t angle_before = as5047p_read_angle();
  HAL_Delay(200);
  uint16_t angle_after = as5047p_read_angle();
  int16_t diff = (int16_t)angle_after - (int16_t)angle_before;
  if (diff > 8192) diff -= 16384;
  if (diff < -8192) diff += 16384;
  if (abs(diff) > 50) { // ロック後も動いている＝ロック失敗
    printf("WARNING: FCO lock unstable! diff=%d\r\n", diff);
  }
}

void drive_init(void)
{
  printf("step1: peripherals init done\r\n");

  if (drive_command_source == DRIVE_COMMAND_CAN && control_motor_mode != CONTROL_VOLTAGE) {
    drive_startup_failed("CONFIG ERROR: CAN requires voltage control");
  }
  initialize_motor_variables();

  printf("step2: motor variables init done\r\n");

  HAL_GPIO_WritePin(SPI1_SS_GPIO_Port, SPI1_SS_Pin, GPIO_PIN_SET);  // SPIの選択ピンをHighに
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);                               // tim3をエンコーダ―モードで開始
  HAL_GPIO_WritePin(WAKE_GPIO_Port, WAKE_Pin, GPIO_PIN_SET);         // STM32がPWMを出す → ゲートドライバがその信号でFETを切り替える → モーターのコイルに電流が流れる、という流れがある。WAKEはこのうちゲートドライバを動作できる状態にする信号
  HAL_Delay(100);

  printf("step3: encoder + WAKE done\r\n");

  printf("board mode: %s\r\n", motor_board_mode == DRIVE_BOARD_V3 ? "V3" : "LEGACY");

  if (motor_board_mode == DRIVE_BOARD_V3) {
    // TIM1でADCの測定を駆動する。ここではPWMチャンネル出力を許可しない。
    if (HAL_TIM_Base_Start(&htim1) != HAL_OK) {
      drive_startup_failed("TIM1 START FAILED");
    }
    start_current_measurement();

    // ゼロ点を求めた後、PWM開始の直前にV3のブートストラップを充電する。
    set_pwm(0.5f, 0.5f, 0.5f);
    if (!drive_v3_charge_bootstrap()) {
      drive_startup_failed("BOOTSTRAP FAILED: PWM disabled");
    }
    start_motor_pwm();
    printf("bootstrap charged, PWM started\r\n");
  } else {
    // 旧基板はこれまでの順序で、故障フラグを解除してからPWMを開始する。
    HAL_Delay(5);
    stspin_clear_faults();
    HAL_Delay(5);
    stspin_clear_faults();  // 念のため2回
    printf("step4: stspin faults cleared\r\n");

    start_motor_pwm();
    printf("step5: PWM started\r\n");

    start_current_measurement();
  }

  align_motor_encoder();

  // TIM2でコントロールループ開始（割り込み）
  HAL_NVIC_SetPriority(TIM2_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(TIM2_IRQn); // CPUがTIM2の割り込みを受け付けられるようにする

  HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 0, 0);
  HAL_NVIC_SetPriority(TIM2_IRQn, 2, 0);

  set_pwm(0.5f, 0.5f, 0.5f);

  if (drive_command_source == DRIVE_COMMAND_LOCAL) {
    drive_state = DRIVE_LOCAL;
  } else {
    __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(&htim1);
    drive_state = DRIVE_CAN_WAIT;
  }

  HAL_TIM_Base_Start_IT(&htim2);    // 平ドラのモーター制御を200 µsごとに制御しているtim2を起動 HAL_TIM_PeriodElapsedCallbackが起動される

  /* どういうことか */
  // HAL_TIM_Base_Start_IT(&htim2)       ← 起動時に1回だけ呼ぶ
  //       ↓
  // TIM2が1周期数え終わり、割り込みを発生
  //       ↓
  // TIM2_IRQHandler()                  ← CPUが呼ぶ
  //       ↓
  // HAL_TIM_IRQHandler(&htim2)         ← HALが割り込みの原因を確認
  //       ↓
  // HAL_TIM_PeriodElapsedCallback(&htim2)
  //       ↓
  // if (htim->Instance == TIM2)        ← TIM2の制御処理に入る

  setvbuf(stdout, NULL, _IONBF, 0);                  //  printf() の文字をバッファにためず、その場で出力する設定
  printf("boot\r\n");
  printf("boot: control loop started\r\n");
}

static void poll_driver_faults(void)
{
    static uint32_t fault_check_cnt = 0;
    if (++fault_check_cnt >= 5) {   // 200 ms間隔に近い呼び出し5回で約1秒。
        fault_check_cnt = 0;
        uint8_t st = stspin_clear_faults();  // stspin_clear_faultsが返す解除する前に読んだ状態を確認
        if (st & 0x04) {                     // ビット2のエラー VDS保護が作動
            printf("VDS protection tripped! cleared.\r\n");
        }
        if (st & 0x08) {                     // ビット3のエラー	STSPINの内部レジスタが初期状態に戻った記録
            printf("Device RESET detected! cleared.\r\n");
        }
    }
}

void drive_report(void)
{
  poll_driver_faults();

  fflush(stdout);
  HAL_Delay(200);

  /* 速度だけを表示する診断例
   * printf("speed=%d rpm, target=%d rpm\r\n",
   *        (int)motor_speed, (int)motor_speed_target);
   * 角度を度で表示する診断例
   * uint32_t display_deg = (uint32_t)((float)angle_raw * 360.0f / 16384.0f);
   * printf("RAW=%u (%lu deg)\r\n", (unsigned int)angle_raw, (unsigned long)display_deg);
   */
  printf("spd=%d I_U=%d I_V=%d I_W=%d\n", (int)motor_speed, (int)(current_u * 1000000), (int)(current_v * 1000000), (int)(current_w * 1000000));  // μA単位
  fflush(stdout);
  HAL_Delay(200);
  poll_driver_faults();
  printf("ADC done=%lu U_raw=%lu W_raw=%lu\r\n",
       (unsigned long)adc_done_count,
       (unsigned long)adc_observed_u,
       (unsigned long)adc_observed_w);
  printf("DQ speed=%d rpm Id=%ld mA Iq=%ld mA\r\n",
       (int)motor_speed,
       (long)(motor_current_d * 1000.0f),
       (long)(motor_current_p * 1000.0f));
  uint32_t irq_mask = __get_PRIMASK();
__disable_irq();

uint32_t u_min = adc_u_min;
uint32_t u_max = adc_u_max;
uint32_t w_min = adc_w_min;
uint32_t w_max = adc_w_max;

adc_u_min = 4095U;
adc_u_max = 0U;
adc_w_min = 4095U;
adc_w_max = 0U;

__set_PRIMASK(irq_mask);

if (u_max >= u_min && w_max >= w_min) {
    printf("ADC range U=%lu..%lu (span=%lu) W=%lu..%lu (span=%lu)\r\n",
           (unsigned long)u_min, (unsigned long)u_max,
           (unsigned long)(u_max - u_min),
           (unsigned long)w_min, (unsigned long)w_max,
           (unsigned long)(w_max - w_min));
}
  /* CAN受信状況を表示 */
  FDCAN_ProtocolStatusTypeDef pstatus;
  FDCAN_ErrorCountersTypeDef  ecounters;
  HAL_FDCAN_GetProtocolStatus(&hfdcan1, &pstatus);
  if (pstatus.BusOff) {
    __HAL_FDCAN_CLEAR_FLAG(&hfdcan1, FDCAN_FLAG_BUS_OFF);
    HAL_FDCAN_Start(&hfdcan1);
  }
  HAL_FDCAN_GetErrorCounters(&hfdcan1, &ecounters);
  printf("drive_state=%u ramp_rpm=%ld\r\n", (unsigned int)drive_state, (long)motor_now_speed_target);
  printf("rx_ok=%lu BusOff=%lu ErrPassive=%lu TEC=%lu REC=%lu target=%d " "tim2_cnt=%lu pid_mode=%d ctrl_mode=%d speed=%d angle=%u OC_FAULT=%d " "I_U=%d I_V=%d I_W=%d\r\n", rx_ok_count, (unsigned long)pstatus.BusOff, (unsigned long)pstatus.ErrorPassive, (unsigned long)ecounters.TxErrorCnt, (unsigned long)ecounters.RxErrorCnt, (int)motor_speed_target, (unsigned long)tim2_cnt, pid_mode, (int)control_motor_mode, (int)motor_speed, angle_raw, overcurrent_fault, (int)(current_u*1000), (int)(current_v*1000), (int)(current_w*1000));
  printf("encoder_fault=%d encoder_fault_score=%d", encoder_fault, encoder_fault_score);
}
