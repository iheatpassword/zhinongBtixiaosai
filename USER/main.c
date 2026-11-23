/**
  * @file    main.c
  * @brief   C30D 4WD/4WS car - ENCODER SPEED CLOSED LOOP + serial commands
  *
  * Four drive motors are speed-controlled (PID, 50 Hz) from the four encoders:
  *     MotorA -> E1 (TIM2)   FL
  *     MotorB -> E2 (TIM3)   FR
  *     MotorC -> E3 (TIM4)   RL
  *     MotorD -> E4 (TIM5)   RR
  * Speed is measured as encoder counts per 20 ms control period.
  *
  * Serial (USART1 "serial1" CH9102 / COM7, USART3 "serial3", 115200 8N1):
  *   F            drive forward, speed closed loop at target
  *   B            drive backward
  *   S            stop (also resets PID)
  *   V<n>         set target speed, counts per 20 ms, e.g. V120
  *   T / T-       steering servos +90 / -90 deg (broadcast)
  *   C            steering servos centre (0 deg)
  *   H / ?        help
  *
  * Safety: 5 s without a movement command -> stop.
  * Motor polarity (calibrated): FL=-1 FR=-1 RL=+1 RR=+1.
  *
  * Based on the WHEELTEC C30D V2.0 official source; only USER/main.c replaced,
  * the FreeRTOS scheduler is NOT started.
  */
#include "system.h"
#include "adc.h"
#include "app_if.h"
#include "laser.h"
#include "centering.h"
#include "fsm2.h"
#include "safety.h"

/* ----------------------------------------------------------------------- */
/*  configuration                                                          */
/* ----------------------------------------------------------------------- */
#define CTRL_MS            20u      /* speed loop period                    */
#define SPD_DEFAULT        150      /* counts per CTRL_MS                   */
#define SPD_MAX            2000

/* single-wheel wiring/order test (USER key long press) */
#define WHEEL_TEST_MS      1500u    /* spin window per wheel, ms            */
#define WHEEL_TEST_PWM     2500     /* fixed open-loop PWM in the test      */

/* a new FSM2 start needs all lasers this far above SAFE_STOP_MM */
#define SAFE_RESTART_MARGIN_MM 5u

/* 70 cm straight test: 4*500 lines * 30 gear = 60000 counts / wheel rev */
#define WHEEL_DIAMETER_MM  76.4f   /* calibrated: 69 mm nominal gave 77.5 cm per 70 cm */
#define ENCODER_CPR        60000.0f
#define RUN_CM             100.0f
#define REV_CM             70.0f
#define RUN_SPD            2400     /* counts per 20 ms while running */
#define RUN_LAT_KP         0.0f     /* disabled: corridor has only one side wall */
#define RUN_LAT_MAX        5.0f     /* clamp, deg                               */
#define RUN_LAT_SIGN       (1.0f)   /* old forward-lateral assist sign    */
#define EDGE_TARGET_MIN_MM 40u      /* never latch a target closer than this */
#define EDGE_TARGET_LATCH_MAX_MM 120u /* latch the first dL below this      */
#define EDGE_KP            0.0f     /* 0 = pure differential (no steering); restore 0.10 for 4WS edge */
#define EDGE_MAX           5.0f     /* clamp, deg                         */
#define EDGE_SPD           1200     /* forward speed while edge/ENTRY runs */
#define EDGE_SIGN          (-1.0f)  /* positive servo = right; verified by the FF=0 run: negative command moves the car left */
#define EDGE_YAW_KP        0.0f     /* front/rear steer deg per deg yaw   */
#define EDGE_YAW_KD        0.10f    /* per deg/s                          */
#define EDGE_YAW_MAX       10.0f     /* clamp, deg                         */
#define EDGE_YAW_SIGN      (1.0f)   /* + = right(clockwise); yaw<0 -> left */   /* back to +1 per bench observation         */
#define RUN_YAW_KP         0.6f     /* front steering deg per deg heading error */
#define RUN_YAW_KD         0.1f     /* per deg/s                                */
#define RUN_YAW_MAX        5.0f     /* clamp, deg                               */
#define RUN_YAW_SIGN       (1.0f)   /* flip if heading runs away                */
#define RUN_TIMEOUT_MS     15000u
#define CENT_TEST_SPD      600      /* centering test base speed counts/20ms */
#define CENT_TEST_CM       150u     /* centering test distance, cm */
#define SHIFT_TEST_CM      280u     /* fallback distance only; laser boundary stops first */
#define SHIFT_SPD          2400     /* counts/20ms during the sideways shift */
#define SHIFT_RAMP         12       /* speed ramp per 20ms tick (soft start)  */
#define SHIFT_WAIT_MS      600u     /* servo settle time */
#define SHIFT_TIMEOUT_MS   45000u

/* crab (进垄) closed-loop: yaw hold via front/rear toe, front-laser lateral hold */
#define SHIFT_DF_TARGET_MM  240.0f   /* front laser to ridge wall when centred  */
#define SHIFT_BOUNDARY_MM   150u     /* side laser <= this -> field boundary    */
#define SHIFT_YAW_KP        2.2f     /* deg toe per deg yaw                     */
#define SHIFT_YAW_KD        0.4f     /* deg toe per deg/s yaw rate              */
#define SHIFT_DF_KI         0.0f     /* disabled: single front laser cannot centre */
#define SHIFT_DF_LOOP_SIGN  (-1.0f)  /* flipped per bench: +1 drove the wrong way */
#define SHIFT_YAW_TARGET_MAX 5.0f    /* outer-loop yaw target clamp, deg         */
/* aux ridge-side sensor: lateral offset = distance to the straddled ridge wall */
#define RIDGE_TARGET_MM     148.0f   /* measured on-ridge distance 2026-11-21      */
#define RIDGE_FF_DEG        7.5f     /* 2200: still far -> FF 7.5                  */
#define RIDGE_KI            0.004f   /* 3200: overshoot -> halve the integral gain  */
#define RIDGE_I_MAX         3.0f     /* integrator clamp, deg                        */
#define RIDGE_OFFSET_MAX    18.0f    /* clamp, deg                                  */
#define RIDGE_DEADBAND_MM   10.0f    /* no integral update inside this band          */
#define RIDGE_VALID_MIN     60u      /* ignore dRidge outside 60..300 mm             */
#define RIDGE_VALID_MAX     300u
#define RIDGE_SIGN          (1.0f)   /* +delta moves the car TOWARD the ridge (verified) */
#define RIDGE_BIAS_DEG      12.0f    /* fixed crab-angle offset for the bias test   */
#define RIDGE_USE_BIAS      0u       /* 0 = closed loop (bias test finished)         */
#define SHIFT_DF_FILT_ALPHA  0.3f    /* low-pass on dF                           */
#define SHIFT_TOE_MAX       8.0f     /* toe clamp, deg                          */
#define SHIFT_DRIFT_MAX     8.0f     /* common offset clamp, deg                */
#define SHIFT_TOE_SIGN      (-1.0f)  /* unused when speed-diff yaw hold is active */
#define SHIFT_YAW_SPD_KP    40.0f    /* counts/20ms per deg yaw                   */
#define SHIFT_YAW_SPD_KD    8.0f     /* counts/20ms per deg/s yaw rate            */
#define SHIFT_YAW_SPD_MAX   150.0f   /* speed differential clamp                  */
#define SHIFT_YAW_SPD_SIGN  (-1.0f)  /* flipped per bench test: + grew the drift   */

/* ICM-20948 heading hold (onboard IMU, I2C) */
#define GYRO_LSB_PER_DPS   65.5f
#define GYRO_SIGN          (+1.0f)
#define YAW_KP             6.0f     /* counts/20ms correction per degree   */
#define YAW_KD             1.5f     /* yaw-rate damping                    */
#define YAW_CORR_MAX       150
#define KP_NUM             10       /* PWM = KP_NUM*err/10 + integral       */
#define KI_DIV             8        /* integral += err/KI_DIV               */
#define PWM_SLEW           400      /* max PWM change per 20 ms tick        */
#define PWM_FF             250      /* feed-forward base (dead zone)        */
#define INTEG_MAX          10000
#define PWM_LIM            12000
#define SERVO_FAST_MS      150u    /* centering steering update time */
#define CMD_TIMEOUT_MS     600000u   /* effectively disabled; IWDG still protects */

#define SERVO_CENTER       500
#define SERVO_UNIT_PER_DEG 4.1667f
#define SERVO_MOVE_MS      300u
#define BROADCAST_ID       0xFEu

/* wheels: FL=0 FR=1 RL=2 RR=3 ; servos: FL=4 FR=3 RL=2 RR=1
   measured 2026-11-23 (single-wheel test):
     FL=A(LF)  FR=B(RF)  RL=C(LB)  RR=D(RB)
     encoder E1..E4 (TIM2..TIM5) peaks with the OLD AB polarity = - + + -
   AB was then flipped (front wheels ran backwards), so A/B forward signs are
   expected to invert; re-check with the fixed test.
   The speed loop takes |count|, so the signs are informational. */
static const u8 WHEEL_MOTOR[4] = { 0u, 1u, 2u, 3u };
static const u8 WHEEL_SERVO_ID[4] = { 4u, 3u, 2u, 1u };

/* +1 / -1 : PWM sign used as "drive forward" for that wheel (calibrated)
   2026-11-23: the first working wheel test showed A/B (front) running
   backwards, so both were flipped from -1 to +1. */
static s8 motor_dir[4] = { 1, 1, 1, 1 };

static s8  motor_run;        /* 0 stop, +1 forward, -1 backward */
static s16 spd_target;       /* counts per control period */
static s16 spd_tgt[4];       /* per-wheel target (heading correction) */
static float gyro_bias;
static float yaw_deg;
static float yaw_rate_dps;
static int   yaw_corr;

static u8  open_loop;        /* 1 = fixed PWM mode (diagnostic) */
static s16 open_pwm;
static u16 servo_angle;
static u32 last_cmd_ms;
static char last_cmd[8];
static u8  last_port;

/* speed loop state */
static s16 meas_spd[4];      /* filtered absolute counts per period */
static s16 prev_raw[4];
static s16 pwm_out[4];
static s32 integral[4];

/* 70 cm straight-run test state */
static u8  app_override;     /* 1 = spd_tgt supplied by App_SetWheelSpeed */
static s32 odom_counts;      /* monotonic average encoder counts (odometry) */
static u8  cent_mode;        /* 1 = centering straight test in progress */
static u16 cent_lost_ticks;  /* consecutive 20 ms ticks without a fresh laser frame */
static u8  cent_show;        /* 1 = show centering result on OLED */
static u32 cent_start_ms;

/* USER-key long press = 70 cm crab lane-shift test */
static u8  shift_mode;
static u8  shift_phase;
static u32 shift_start_ms;
static u32 shift_t0;
static u16 shift_base_cm;
static u8  shift_show;
static s16 shift_dir;          /* +1 = crab right, -1 = crab left */
static u8  rev_state;
static void abort_all(void);
static void shift_start(s16 dir);
static u8  wheel_test_mode;
static u8  wheel_idx;
static u32 wheel_t_on;
static s16 wheel_peak[4];      /* max |encoder count| seen in the window */
static u8  diff_test_mode;
static u8  diff_show;
static s32 diff_test_counts;
static u16 diff_test_base;
static s16 diff_test_pwm = 2500;
static u8  edge_mode;
static float edge_yaw_i;
static u16 edge_target_mm;   /* latched left-laser target for this run */
static u8  edge_target_ok;
static u16 edge_base_cm;     /* odometry baseline for the edge run     */
static s16 edge_steer_dbg;   /* last edge/ENTRY steer command, 0.1 deg */
static u16 laser_dl_hold_mm; /* last valid left-laser distance         */
static u32 laser_dl_hold_ms;
static u8  laser_dl_hold_ok;
static float diff_kp = 1.0f, diff_ki = 0.0f, diff_kd = 0.0f, diff_max = 600.0f;
static float diff_ff = -55.0f;  /* constant left/right offset, counts (pure-diff trim) */
static float diff_i;
static u8  diff_cal_mode;
static s8  diff_cal_dir;
static s32 diff_cal_counts;
static u16 cal_base_cm;
static u16 diff_cal_base_cm;
static u8  cal_mode;
static s8  cal_dir;
static s32 cal_counts;
static u8  fsm2_mode;
static u16 fsm2_dist0;
static SafetyFilt g_sf;
static SafetyCrab g_sc;
static SafetyStall g_stall;
static Fsm2Input  g_f2in;
static Fsm2Output g_f2out;          /* 1 = reverse run in progress */
static s32 rev_counts;
static u32 rev_start_ms;
static s32 rev_target_counts;
static s16 shift_doff_dbg;
static s16 shift_af_dbg;
static s16 shift_ridge_dbg;
static float shift_yaw_target;
static float shift_df_filt;
static float ridge_integ;
static float batt_volt;
static u16 shift_yaw_fault_cnt;
static u32 next_heartbeat;
static u16 cent_base_cm;     /* odometry baseline at test start */
static u8  run_state;        /* 0 idle, 1 running, 2 done */
static s32 run_counts;
static s32 last_run_counts;
static s16 run_lat_off_dbg;
static s32 run_hist_cnt[3];
static s16 run_hist_yaw[3];
static u8  run_hist_n;
static s16 run_lat_deg_dbg;
static s16 run_yaw_deg_dbg;       /* average encoder counts travelled */
static s32 run_target;       /* counts for 70 cm */
static u32 run_start_ms;

/* ======================================================================= */
/*  1 ms time base (TIM6) - the official ms_now() does not run in    */
/*  bare metal because the FreeRTOS tick counter is only incremented by the */
/*  scheduler, so provide our own millisecond counter.                      */
/* ======================================================================= */
static volatile u32 ms_cnt;

void TIM6_DAC_IRQHandler(void)
{
  if (TIM_GetITStatus(TIM6, TIM_IT_Update) != RESET)
  {
    TIM_ClearITPendingBit(TIM6, TIM_IT_Update);
    ms_cnt++;
  }
}

static u32 ms_now(void)
{
  return ms_cnt;
}

static void ms_init(void)
{
  TIM_TimeBaseInitTypeDef tb;
  NVIC_InitTypeDef nv;

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);

  tb.TIM_Prescaler     = 84u - 1u;      /* 84 MHz / 84 = 1 MHz          */
  tb.TIM_Period        = 1000u - 1u;    /* 1 MHz / 1000 = 1 kHz (1 ms)  */
  tb.TIM_CounterMode   = TIM_CounterMode_Up;
  tb.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseInit(TIM6, &tb);
  TIM_ITConfig(TIM6, TIM_IT_Update, ENABLE);

  nv.NVIC_IRQChannel                   = TIM6_DAC_IRQn;
  nv.NVIC_IRQChannelPreemptionPriority = 2u;
  nv.NVIC_IRQChannelSubPriority        = 0u;
  nv.NVIC_IRQChannelCmd                = ENABLE;
  NVIC_Init(&nv);

  TIM_Cmd(TIM6, ENABLE);
}

/* ======================================================================= */
/*  generic UART transmit / receive                                        */
/* ======================================================================= */
static void tx_putc(USART_TypeDef *u, u8 b)
{
  u32 guard = 200000u;
  while ((USART_GetFlagStatus(u, USART_FLAG_TXE) == RESET) && (guard > 0u))
  {
    guard--;
  }
  if (guard == 0u) return;          /* never hang the main loop on a stuck UART */
  USART_SendData(u, b);
}

static void tx_puts(USART_TypeDef *u, const char *s)
{
  while (*s != '\0') tx_putc(u, (u8)*s++);
}

static void tx_num(USART_TypeDef *u, int v)
{
  char b[10];
  u8 n = 0;
  if (v < 0) { tx_putc(u, '-'); v = -v; }
  if (v == 0) { tx_putc(u, '0'); return; }
  while (v > 0) { b[n++] = (char)('0' + (v % 10)); v /= 10; }
  while (n > 0) tx_putc(u, (u8)b[--n]);
}

typedef struct
{
  USART_TypeDef      *usart;
  DMA_Stream_TypeDef *stream;
  u8   buf[128];
  u16  last;
  char line[32];
  u8   len;
} CmdPort;

static CmdPort port1;
static CmdPort port3;
static CmdPort port5;   /* UART5: PC12=TX, PD2=RX */

static void cmd_port_init(CmdPort *p, USART_TypeDef *u, DMA_Stream_TypeDef *s, u32 ch)
{
  DMA_InitTypeDef dma;

  p->usart  = u;
  p->stream = s;
  p->last   = 0u;
  p->len    = 0u;

  USART_ITConfig(u, USART_IT_RXNE, DISABLE);

  DMA_DeInit(s);
  dma.DMA_Channel            = ch;
  dma.DMA_PeripheralBaseAddr = (u32)&u->DR;
  dma.DMA_Memory0BaseAddr    = (u32)p->buf;
  dma.DMA_DIR                = DMA_DIR_PeripheralToMemory;
  dma.DMA_BufferSize         = 128u;
  dma.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
  dma.DMA_MemoryInc          = DMA_MemoryInc_Enable;
  dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  dma.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;
  dma.DMA_Mode               = DMA_Mode_Circular;
  dma.DMA_Priority           = DMA_Priority_High;
  dma.DMA_FIFOMode           = DMA_FIFOMode_Disable;
  dma.DMA_FIFOThreshold      = DMA_FIFOThreshold_1QuarterFull;
  dma.DMA_MemoryBurst        = DMA_MemoryBurst_Single;
  dma.DMA_PeripheralBurst    = DMA_PeripheralBurst_Single;
  DMA_Init(s, &dma);
  DMA_Cmd(s, ENABLE);
  USART_DMACmd(u, USART_DMAReq_Rx, ENABLE);
}

/* ======================================================================= */
/*  USART2 : HTS-35H bus servo                                            */
/* ======================================================================= */
static void u2_putc(u8 b)
{
  {
    u32 guard = 200000u;
    while ((USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET) && (guard > 0u))
    {
      guard--;
    }
    if (guard == 0u) return;      /* servo bus stuck: drop the byte, never hang */
  }
  USART_SendData(USART2, b);
}

static void servo_send(u8 id, u8 cmd, const u8 *p, u8 n)
{
  u8  i;
  u8  len = (u8)(n + 3u);
  u16 sum = (u16)(id + len + cmd);

  for (i = 0; i < n; i++) sum = (u16)(sum + p[i]);

  u2_putc(0x55); u2_putc(0x55); u2_putc(id); u2_putc(len); u2_putc(cmd);
  for (i = 0; i < n; i++) u2_putc(p[i]);
  u2_putc((u8)(255u - (sum & 0xFFu)));
}

static void servo_move(u8 id, u16 pos, u16 move_ms)
{
  u8 p[4];
  p[0] = (u8)(pos & 0xFFu);
  p[1] = (u8)(pos >> 8);
  p[2] = (u8)(move_ms & 0xFFu);
  p[3] = (u8)(move_ms >> 8);
  servo_send(id, 1u, p, 4u);
}

static void servo_broadcast_deg(int deg)
{
  s16 pos = (s16)((float)SERVO_CENTER + (float)deg * SERVO_UNIT_PER_DEG);
  if (pos < 0)    pos = 0;
  if (pos > 1000) pos = 1000;
  servo_move(BROADCAST_ID, (u16)pos, SERVO_MOVE_MS);
  servo_angle = (u16)(deg < 0 ? -deg : deg);
}

/* ======================================================================= */
/*  ICM-20948 gyroscope yaw (onboard IMU, software I2C)                    */
/* ======================================================================= */
static int16_t gyro_z_raw(void)
{
  u8 lo = I2C_ReadOneByte(0xD0u, 0x38u);   /* GYRO_ZOUT_L */
  u8 hi = I2C_ReadOneByte(0xD0u, 0x37u);   /* GYRO_ZOUT_H */
  return (int16_t)(((u16)hi << 8) | lo);
}

static void imu_init(void)
{
  long sum = 0;
  u16 i;

  I2C_WriteOneByte(0xD0u, 0x7Fu, 0x00u);   /* bank 0 */
  delay_ms(20u);
  for (i = 0; i < 200u; i++)
  {
    sum += gyro_z_raw();
    delay_ms(3u);
  }
  gyro_bias = (float)sum / 200.0f;
  yaw_deg = 0.0f;
}

static float gyro_z_dps(void)
{
  return ((float)gyro_z_raw() - gyro_bias) / GYRO_LSB_PER_DPS;
}

/* ======================================================================= */
/*  speed loop                                                            */
/* ======================================================================= */
static void speed_reset(void)
{
  u8 w;
  for (w = 0; w < 4u; w++)
  {
    (void)Read_Encoder((u8)(2u + w));   /* clear counter */
    meas_spd[w] = 0;
    prev_raw[w] = 0;
    pwm_out[w]  = 0;
    integral[w] = 0;
  }
  Set_Pwm(0, 0, 0, 0, 1500);
}

static void speed_apply(void)
{
  int m[4];
  u8  w;
  int p;

  for (w = 0; w < 4u; w++)
  {
    if (motor_run == 0) { m[WHEEL_MOTOR[w]] = 0; continue; }
    p = pwm_out[w];
    p = (motor_run > 0) ? p : -p;
    m[WHEEL_MOTOR[w]] = p * (int)motor_dir[w];
  }
  Set_Pwm(m[0], m[1], m[2], m[3], 1500);
}

/* one speed-loop tick: measure encoders, PID, apply PWM (50 Hz) */
static void speed_tick(void)
{
  u8  w;
  int d = 0, e, p;
  int raw;
  int raw_sum = 0;
  float rate;
  int corr = 0;

  if (wheel_test_mode != 0u)
  {
    u8  ww;
    int wr;
    u8  on = (u8)(((ms_now() - wheel_t_on) < WHEEL_TEST_MS) ? 1u : 0u);

    for (ww = 0u; ww < 4u; ww++)
    {
      wr = Read_Encoder((u8)(2u + ww));   /* keep the sign: polarity check */
      if (wr >  32000) wr =  32000;
      if (wr < -32000) wr = -32000;
      meas_spd[ww] = (s16)wr;

      /* peak-hold so the result is still readable after the wheel stops */
      if (on != 0u)
      {
        int a = (wr < 0) ? -wr : wr;
        int b = (wheel_peak[ww] < 0) ? -(int)wheel_peak[ww]
                                     :  (int)wheel_peak[ww];
        if (a > b) wheel_peak[ww] = (s16)wr;
      }

      pwm_out[ww] = ((on != 0u) && (ww == wheel_idx))
                    ? (s16)WHEEL_TEST_PWM : (s16)0;
    }

    /* speed_apply() is gated by motor_run; drive directly while in the window.
       (This re-arms every 20 ms, so a low-battery stop cannot end the window;
        run this test with the car lifted.) */
    motor_run = (on != 0u) ? 1 : 0;
    speed_apply();
    return;
  }

  if (diff_test_mode != 0u)
  {
    u8 dw;
    int draw;

    for (dw = 0u; dw < 4u; dw++)
    {
      draw = Read_Encoder((u8)(2u + dw));
      if (draw < 0) draw = -draw;
      if (draw > 32000) draw = 32000;
      raw_sum += draw;
    }
    odom_counts += (raw_sum / 4);
    diff_test_counts += (raw_sum / 4);

    pwm_out[0] =  diff_test_pwm;   /* FL forward */
    pwm_out[1] = -diff_test_pwm;   /* FR reverse */
    pwm_out[2] =  diff_test_pwm;   /* RL forward */
    pwm_out[3] = -diff_test_pwm;   /* RR reverse */
    speed_apply();
    return;
  }

  if ((motor_run != 0) && (app_override == 0u))
  {
    rate = yaw_rate_dps;
    corr = (int)(YAW_KP * yaw_deg + YAW_KD * rate);
    if (corr > YAW_CORR_MAX)  corr = YAW_CORR_MAX;
    if (corr < -YAW_CORR_MAX) corr = -YAW_CORR_MAX;
    if (motor_run < 0) corr = -corr;   /* reverse: mirror the correction */
    yaw_corr = corr;
  }

  if (app_override == 0u)
  {
    /* left wheels (FL, RL) slow down, right wheels speed up when yaw > 0 */
    spd_tgt[0] = (s16)((int)spd_target - corr);
    spd_tgt[1] = (s16)((int)spd_target + corr);
    spd_tgt[2] = (s16)((int)spd_target - corr);
    spd_tgt[3] = (s16)((int)spd_target + corr);
  }

  for (w = 0; w < 4u; w++)
  {
    raw = Read_Encoder((u8)(2u + w));
    if (raw < 0) raw = -raw;
    if (raw > 32000) raw = 32000;
    raw_sum += raw;
    d = raw;

    if (motor_run == 0)
    {
      /* keep the measured speed alive while stopped, so pushing the car by
         hand / free-spinning a wheel shows up in the log (encoder health) */
      meas_spd[w] = (s16)d;
      prev_raw[w] = (s16)d;
      integral[w] = 0;
      pwm_out[w]  = 0;
      continue;
    }

    if (open_loop != 0u)
    {
      prev_raw[w] = (s16)d;
      meas_spd[w] = (s16)d;
      pwm_out[w]  = open_pwm;
      continue;
    }

    /* simple 2-point average to reduce encoder quantisation noise */
    meas_spd[w] = (s16)((d + prev_raw[w]) / 2);
    prev_raw[w] = (s16)d;

    e = (int)spd_tgt[w] - (int)meas_spd[w];
    integral[w] += (s32)e / KI_DIV;
    if (integral[w] > INTEG_MAX)  integral[w] = INTEG_MAX;
    if (integral[w] < -INTEG_MAX) integral[w] = -INTEG_MAX;

    /* feed-forward: measured speed ~ 0.4*PWM - 100  =>  PWM ~ 2.5*spd + 250 */
    p = (KP_NUM * e) / 10 + (int)integral[w] + PWM_FF + ((int)spd_tgt[w] * 25) / 10;
    if (p > PWM_LIM) p = PWM_LIM;
    if (p < 0)       p = 0;

    /* slew-rate limit to avoid oscillation from wheel inertia */
    if (p > (int)pwm_out[w] + PWM_SLEW) p = (int)pwm_out[w] + PWM_SLEW;
    if (p < (int)pwm_out[w] - PWM_SLEW) p = (int)pwm_out[w] - PWM_SLEW;

    pwm_out[w] = (s16)p;
  }
  if (motor_run != 0) odom_counts += (raw_sum / 4);
  if (run_state == 1u) run_counts += (raw_sum / 4);
  if (rev_state == 1u) rev_counts += (raw_sum / 4);
  speed_apply();
}

static void run_start(void)
{
  run_counts = 0;
  run_start_ms = ms_now();
  yaw_deg = 0.0f;
  App_SetServoCenter();
  run_state = 1u;
  app_override = 0u;
  open_loop = 0u;
  motor_run = 1;
  spd_target = RUN_SPD;
  speed_reset();
  tx_puts(USART1, "70CM START\r\n");
  tx_puts(USART3, "70CM START\r\n");
  tx_puts(UART5,  "70CM START\r\n");
}

static void run_stop(const char *why)
{
  /* keep the last three runs for later readout */
  run_hist_cnt[0] = run_hist_cnt[1];
  run_hist_cnt[1] = run_hist_cnt[2];
  run_hist_cnt[2] = run_counts;
  run_hist_yaw[0] = run_hist_yaw[1];
  run_hist_yaw[1] = run_hist_yaw[2];
  run_hist_yaw[2] = (s16)(yaw_deg * 10.0f);
  if (run_hist_n < 3u) run_hist_n++;
  last_run_counts = run_counts;
  motor_run = 0;
  speed_reset();
  run_state = 2u;
  tx_puts(USART1, why);
  tx_puts(USART3, why);
  tx_puts(UART5,  why);
  tx_puts(USART1, " counts=");
  tx_puts(USART3, " counts=");
  tx_puts(UART5,  " counts=");
  tx_num(USART1, (int)run_counts);
  tx_num(USART3, (int)run_counts);
  tx_num(UART5,  (int)run_counts);
  tx_puts(USART1, "\r\n");
  tx_puts(USART3, "\r\n");
}

/* distance in 0.1 mm units (avoids float in the display path) */
static s32 run_dist_01mm(void)
{
  return (s32)((float)run_counts * 3.14159f * WHEEL_DIAMETER_MM * 10.0f / ENCODER_CPR);
}

/* ======================================================================= */
/*  OLED                                                                  */
/* ======================================================================= */
static void show_signed(u8 x, u8 y, int32_t v, u8 digits)
{
  if (v < 0) { OLED_ShowString(x, y, (u8 *)"-"); v = -v; }
  else       { OLED_ShowString(x, y, (u8 *)"+"); }
  OLED_ShowNumber((u8)(x + 8u), y, (u32)v, digits, 12);
}

static void iwdg_init(void)
{
  IWDG->KR  = 0x5555u;   /* unlock PR/RLR */
  IWDG->PR  = 0x04u;     /* LSI/64 = 500 Hz */
  IWDG->RLR = 500u;      /* ~1 s */
  IWDG->KR  = 0xAAAAu;   /* reload */
  IWDG->KR  = 0xCCCCu;   /* start */
}

static void screen(void)
{
  u8 i, y;
  s32 d01 = run_dist_01mm();

  if (wheel_test_mode != 0u)
  {
    u8 k;

    OLED_Clear();
    OLED_ShowString(0, 0, (u8 *)"MOTOR ");
    OLED_ShowChar(48, 0, (u8)('A' + WHEEL_MOTOR[wheel_idx]), 12, 1);
    OLED_ShowString(56, 0, (u8 *)"3CLK=EXIT");
    for (k = 0u; k < 4u; k++)
    {
      u8 yy = (u8)(12u + 12u * k);
      OLED_ShowString(0, yy, (u8 *)"E");
      OLED_ShowChar(8, yy, (u8)('1' + k), 12, 1);
      show_signed(24, yy, (int32_t)wheel_peak[k], 5);
    }
    OLED_Refresh_Gram();
    return;
  }

  if ((diff_test_mode != 0u) || (diff_show != 0u))
  {
    u8 k2;
    OLED_Clear();
    OLED_ShowString(0, 0, (u8 *)(diff_test_mode ? "DIFF TEST" : "DIFF RESULT"));
    OLED_ShowString(0, 10, (u8 *)"CMD L+ R-");
    for (k2 = 0u; k2 < 4u; k2++)
    {
      u8 yy = (u8)(20u + 10u * k2);
      OLED_ShowString(0, yy, (u8 *)((k2 == 0u) ? "FL" : (k2 == 1u) ? "FR" : (k2 == 2u) ? "RL" : "RR"));
      show_signed(18, yy, (int32_t)meas_spd[k2], 4);
    }
    OLED_Refresh_Gram();
    return;
  }

  if ((edge_mode != 0u) || ((fsm2_mode != 0u) && (Fsm2_GetState() == F2_ENTRY)))
  {
    OLED_Clear();
    OLED_ShowString(0, 0, (u8 *)"EDGE / ENTRY");
    OLED_ShowString(0, 10, (u8 *)"L=");
    OLED_ShowNumber(24, 10, (u32)((g_sf.dL == 0xFFFFu) ? 0u : g_sf.dL), 4, 12);
    OLED_ShowString(0, 20, (u8 *)"TGT=");
    if (edge_target_ok != 0u)
    {
      OLED_ShowNumber(32, 20, (u32)edge_target_mm, 3, 12);
    }
    else
    {
      OLED_ShowString(32, 20, (u8 *)"---");
    }
    OLED_ShowString(0, 30, (u8 *)"DB=");
    OLED_ShowNumber(24, 30, (u32)((g_sf.dB == 0xFFFFu) ? 0u : g_sf.dB), 4, 12);
    OLED_ShowString(0, 40, (u8 *)"YAW=");
    show_signed(32, 40, (int32_t)(yaw_deg * 10.0f), 3);
    OLED_ShowString(0, 50, (u8 *)"F2=");
    OLED_ShowNumber(24, 50, (u32)Fsm2_GetState(), 2, 12);
    OLED_Refresh_Gram();
    return;
  }

  if ((fsm2_mode != 0u) && (Fsm2_GetState() >= F2_CRAB_R1) && (Fsm2_GetState() <= F2_EXIT_L))
  {
    Fsm2State st = Fsm2_GetState();
    u16 v = 0;
    const char *nm = "L";
    u16 tgt = 100;

    if ((st == F2_CRAB_R1) || (st == F2_CRAB_R3) || (st == F2_CRAB_R5))
    {
      v = g_sf.dR; nm = "R";
    }
    else
    {
      v = g_sf.dL; nm = "L";
    }
    OLED_Clear();
    OLED_ShowString(0, 0, (u8 *)"CRAB");
    OLED_ShowString(40, 0, (u8 *)nm);
    OLED_ShowString(56, 0, (u8 *)"=");
    OLED_ShowNumber(72, 0, (u32)((v == 0xFFFFu) ? 0u : v), 4, 12);
    OLED_ShowString(0, 10, (u8 *)"TGT=100");
    OLED_ShowString(0, 20, (u8 *)"DB=");
    OLED_ShowNumber(24, 20, (u32)((g_sf.dB == 0xFFFFu) ? 0u : g_sf.dB), 4, 12);
    OLED_ShowString(0, 30, (u8 *)"YAW=");
    show_signed(32, 30, (int32_t)(yaw_deg * 10.0f), 3);
    OLED_ShowString(0, 40, (u8 *)"F2=");
    OLED_ShowNumber(24, 40, (u32)st, 2, 12);
    OLED_Refresh_Gram();
    return;
  }

  if ((shift_mode != 0u) || (shift_show != 0u))
  {
    OLED_Clear();
    OLED_ShowString(0, 0, (u8 *)(shift_mode != 0u ? "SHIFT RUNNING" : "SHIFT RESULT"));
    OLED_ShowString(0, 10, (u8 *)"DIST");
    OLED_ShowNumber(40, 10, (u32)((u16)(App_GetDistanceCm() - shift_base_cm)), 4, 12);
    OLED_ShowString(0, 20, (u8 *)"CMD");
    OLED_ShowNumber(40, 20, (u32)shift_phase, 1, 12);
    OLED_ShowString(0, 30, (u8 *)"L");
    OLED_ShowNumber(16, 30, (u32)((g_laser.dL == 0xFFFFu) ? 0u : g_laser.dL), 4, 12);
    OLED_ShowString(64, 30, (u8 *)"R");
    OLED_ShowNumber(80, 30, (u32)((g_laser.dR == 0xFFFFu) ? 0u : g_laser.dR), 4, 12);
    OLED_Refresh_Gram();
    return;
  }

  if ((cent_mode != 0u) || (cent_show != 0u))
  {
    OLED_Clear();
    OLED_ShowString(0, 0, (u8 *)(cent_mode != 0u ? "CENTER RUNNING" : "CENTER RESULT"));
    OLED_ShowString(0, 10, (u8 *)"E");
    show_signed(16, 10, (int32_t)(g_centering.e_cm * 10.0f), 4);
    OLED_ShowString(64, 10, (u8 *)"0.1cm");
    OLED_ShowString(0, 20, (u8 *)"D");
    show_signed(16, 20, (int32_t)(g_centering.delta_deg * 10.0f), 4);
    OLED_ShowString(0, 30, (u8 *)"T");
    show_signed(16, 30, (int32_t)(g_centering.toe_deg * 10.0f), 4);
    OLED_ShowString(0, 40, (u8 *)"DIST");
    OLED_ShowNumber(40, 40, (u32)((u16)(App_GetDistanceCm() - cent_base_cm)), 4, 12);
    OLED_ShowString(0, 50, (u8 *)(g_centering.saturated ? "SAT" : "   "));
    OLED_Refresh_Gram();
    return;
  }

  OLED_Clear();
  OLED_ShowString(0, 0, (u8 *)"70cm STRAIGHT");
  OLED_ShowString(0, 10, (u8 *)(run_state == 0u ? "READY KEY/G" :
                                (run_state == 1u ? "RUNNING" : "DONE")));
  OLED_ShowString(64, 10, (u8 *)"D:");
  OLED_ShowNumber(80, 10, (u32)(d01 / 10), 3, 12);
  OLED_ShowString(104, 10, (u8 *)"cm");

  for (i = 0; i < 4u; i++)
  {
    y = (u8)(20u + 10u * i);
    OLED_ShowString(0, y, (u8 *)((i == 0u) ? "FL" : (i == 1u) ? "FR" : (i == 2u) ? "RL" : "RR"));
    OLED_ShowNumber(18, y, (u32)meas_spd[i], 3, 12);
    OLED_ShowString(38, y, (u8 *)"/");
    OLED_ShowNumber(44, y, (u32)spd_target, 3, 12);
    OLED_ShowNumber(70, y, (u32)pwm_out[i], 4, 12);
  }
  OLED_Refresh_Gram();
}

/* ======================================================================= */
/*  commands                                                              */
/* ======================================================================= */
static void cmd_help(USART_TypeDef *u)
{
  tx_puts(u, "\r\n=== C30D speed-loop command ===\r\n");
  tx_puts(u, "F       : forward (speed closed loop)\r\n");
  tx_puts(u, "B       : backward\r\n");
  tx_puts(u, "S       : stop\r\n");
  tx_puts(u, "V<n>    : target speed counts/20ms, e.g. V120\r\n");
  tx_puts(u, "O<n>    : open-loop fixed PWM (diagnostic), e.g. O2500\r\n");
  tx_puts(u, "G       : run straight 70 cm (button does the same)\r\n");
  tx_puts(u, "N       : centering straight test (2.5 m, needs laser)\r\n");
  tx_puts(u, "A       : start autonomous competition FSM\r\n");
  tx_puts(u, "Q       : abort FSM / reset to IDLE\r\n");
  tx_puts(u, "K       : diff PID: K kp ki kd max [ff] (kp/ki/kd x100)\r\n");
  tx_puts(u, "T / T-  : servos +90 / -90 deg (broadcast)\r\n");
  tx_puts(u, "C       : servos centre\r\n");
  tx_puts(u, "W / W-  : crab test right / left (front/rear diff)\r\n");
  tx_puts(u, "H / ?   : help\r\n");
  tx_puts(u, "===============================\r\n");
}

static int parse_next(const char **s)
{
  int v = 0, sign = 1;
  while (**s == ' ') (*s)++;
  if (**s == '-') { sign = -1; (*s)++; }
  while ((**s >= '0') && (**s <= '9')) { v = v * 10 + (**s - '0'); (*s)++; }
  return sign * v;
}

static int parse_num(const char *s)
{
  int v = 0;
  u8  i = 0u;
  while ((s[i] >= '0') && (s[i] <= '9')) { v = v * 10 + (s[i] - '0'); i++; }
  return v;
}

static void cmd_exec(USART_TypeDef *u, const char *line)
{
  char c[40];
  u8  i = 0u;
  u8  j;

  while ((line[i] != '\0') && (i < 39u))
  {
    c[i] = line[i];
    if ((c[i] >= 'a') && (c[i] <= 'z')) c[i] = (char)(c[i] - 'a' + 'A');
    i++;
  }
  c[i] = '\0';
  if (i == 0u) return;

  /* movement commands are not allowed while FSM2 is running (S/Q still stop) */
  if ((fsm2_mode != 0u) &&
      ((c[0] == 'F') || (c[0] == 'B') ||
       (c[0] == 'G') || (c[0] == 'V') || (c[0] == 'O')))
  {
    tx_puts(u, "ERR BUSY (Q/S to stop)\r\n");
    return;
  }

  for (j = 0u; (j < 7u) && (c[j] != '\0'); j++) last_cmd[j] = c[j];
  last_cmd[j] = '\0';
  last_cmd_ms = ms_now();
  last_port = (u == USART1) ? 1u : ((u == USART3) ? 3u : 5u);

  if (c[0] == 'F')
  {
    cent_show = 0u;
    cent_mode = 0u;
    run_state = 0u;
    app_override = 0u;
    open_loop = 0u;
    motor_run = 1;
    yaw_deg = 0.0f;
    speed_reset();
    tx_puts(u, "OK FWD\r\n");
  }
  else if (c[0] == 'B')
  {
    cent_show = 0u;
    cent_mode = 0u;
    run_state = 0u;
    app_override = 0u;
    open_loop = 0u;
    motor_run = -1;
    yaw_deg = 0.0f;
    speed_reset();
    tx_puts(u, "OK BACK\r\n");
  }
  else if ((c[0] == 'S') || (c[0] == 'X'))
  {
    cent_show = 0u;
    shift_show = 0u;
    abort_all();                 /* clears edge/fsm2/run/shift/... too */
    tx_puts(u, "OK STOP\r\n");
  }
  else if (c[0] == 'O')
  {
    int v = parse_num(&c[1]);
    if ((v >= 0) && (v <= 12000))
    {
      open_loop = 1u;
      open_pwm = (s16)v;
      motor_run = 1;
      speed_reset();
      tx_puts(u, "OK OPEN PWM=");
      tx_num(u, v);
      tx_puts(u, "\r\n");
    }
    else tx_puts(u, "ERR: O 0..12000\r\n");
  }
  else if (c[0] == 'V')
  {
    int v = parse_num(&c[1]);
    if ((v >= 0) && (v <= SPD_MAX))
    {
      spd_target = (s16)v;
      tx_puts(u, "OK V=");
      tx_num(u, v);
      tx_puts(u, "\r\n");
    }
    else tx_puts(u, "ERR: V 0..600\r\n");
  }
  else if (c[0] == 'T')
  {
    if (c[1] == '-') { servo_broadcast_deg(-90); tx_puts(u, "OK SERVO -90\r\n"); }
    else             { servo_broadcast_deg(+90); tx_puts(u, "OK SERVO +90\r\n"); }
  }
  else if (c[0] == 'C')
  {
    servo_broadcast_deg(0);
    tx_puts(u, "OK SERVO 0\r\n");
  }
  else if (c[0] == 'W')
  {
    shift_start((s16)((c[1] == '-') ? -1 : 1));
    tx_puts(u, (c[1] == '-') ? "OK CRAB LEFT\r\n" : "OK CRAB RIGHT\r\n");
  }
  else if (c[0] == 'G')
  {
    cent_mode = 0u;
    run_start();
    tx_puts(u, "OK 70CM START\r\n");
  }
  else if (c[0] == 'N')
  {
    if (fsm2_mode != 0u) { tx_puts(u, "ERR BUSY\r\n"); }
    else
    {
      cent_mode = 1u;
      cent_show = 0u;
      cent_lost_ticks = 0u;
      cent_start_ms = ms_now();
      cent_base_cm = App_GetDistanceCm();
      Centering_Init();
      tx_puts(u, "OK CENT START\r\n");
    }
  }
  else if (c[0] == 'A')
  {
    abort_all();
    Fsm2_Start();
    fsm2_mode = 1u;
    fsm2_dist0 = App_GetDistanceCm();
    tx_puts(u, "OK FSM2 START\r\n");
  }
  else if ((c[0] == 'P') || (c[0] == 'M'))
  {
    abort_all();
    cal_mode = 1u;
    cal_dir = (s8)((c[0] == 'P') ? 1 : -1);
    cal_counts = 0;
    cal_base_cm = App_GetDistanceCm();
    yaw_deg = 0.0f;
    App_SetServoCenter();
    app_override = 0u;
    open_loop = 0u;
    motor_run = 1;
    spd_target = 900;
    speed_reset();
    App_SetSteer4(8.0f * cal_dir, 8.0f * cal_dir, -8.0f * cal_dir, -8.0f * cal_dir);
    tx_puts(u, "OK CAL START\r\n");
  }
  else if (c[0] == 'K')
  {
    const char *q = &c[1];
    diff_kp  = (float)parse_next(&q) / 100.0f;
    diff_ki  = (float)parse_next(&q) / 100.0f;
    diff_kd  = (float)parse_next(&q) / 100.0f;
    diff_max = (float)parse_next(&q);
    {
      const char *p = q;
      while (*p == ' ') p++;
      if (*p != '\0') diff_ff = (float)parse_next(&q);  /* optional 5th, counts */
    }
    tx_puts(u, "OK K\r\n");
  }
  else if (c[0] == 'D')
  {
    abort_all();
    diff_cal_mode = 1u;
    diff_cal_dir = (s8)((c[1] == '-') ? -1 : 1);
    diff_cal_counts = 0;
    diff_cal_base_cm = App_GetDistanceCm();
    yaw_deg = 0.0f;
    App_SetServoCenter();
    {
      s16 dsp[4];
      dsp[0] = 800; dsp[1] = 800; dsp[2] = 800; dsp[3] = 800;
      App_SetWheelSpeed(dsp);
    }
    speed_reset();
    tx_puts(u, "OK DIFF CAL\r\n");
  }
  else if (c[0] == 'E')
  {
    abort_all();
    edge_mode = 1u;
    edge_yaw_i = 0.0f;
    diff_i = 0.0f;
    App_SetServoCenter();
    {
      s16 esp[4];
      esp[0] = EDGE_SPD; esp[1] = EDGE_SPD; esp[2] = EDGE_SPD; esp[3] = EDGE_SPD;
      App_SetWheelSpeed(esp);      /* app_override=1: no differential yaw */
    }
    speed_reset();
    tx_puts(u, "OK EDGE START\r\n");
  }
  else if (c[0] == 'Q')
  {
    abort_all();                 /* full stop incl. edge/fsm2 */
    tx_puts(u, "OK AUTO ABORT\r\n");
  }
  else if ((c[0] == 'H') || (c[0] == '?'))
  {
    cmd_help(u);
  }
  else
  {
    tx_puts(u, "ERR: H for help\r\n");
  }
}

static void cmd_poll(CmdPort *p)
{
  u16 pos = (u16)(128u - DMA_GetCurrDataCounter(p->stream));
  u8  b;

  while (p->last != pos)
  {
    b = p->buf[p->last];
    p->last = (u16)((p->last + 1u) & 127u);

    if ((b == '\r') || (b == '\n'))
    {
      if (p->len > 0u)
      {
        p->line[p->len] = '\0';
        p->len = 0u;
        cmd_exec(p->usart, p->line);
      }
    }
    else if (p->len < (u8)(sizeof(p->line) - 1u))
    {
      p->line[p->len++] = (char)b;
    }
  }
}

static void abort_all(void)
{
  motor_run = 0;
  speed_reset();
  run_state = 0u;
  rev_state = 0u;
  shift_mode = 0u;
  cent_mode = 0u;
  fsm2_mode = 0u;
  edge_mode = 0u;
  cal_mode = 0u;
  diff_cal_mode = 0u;
  diff_test_mode = 0u;         /* an aborted DIFFTEST must not outlive the stop */
  diff_show = 0u;
  wheel_test_mode = 0u;        /* any abort also leaves the wheel test */
  Fsm2_Abort();
  app_override = 0u;
  open_loop = 0u;
}

/* ---- key driver: 1/2/3 clicks + long press, non-blocking --------------- */
static u8  key_event;        /* 0 none, 1 single, 2 double, 3 triple, 4 long */
static u8  k_curr, k_prev;
static u8  k_sample;
static u8  k_clicks;
static u8  k_long_done;
static u32 k_down_ms, k_up_ms;

static u8 key_raw(void)
{
  return (GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_0) == Bit_RESET) ? 1u : 0u;
}

static void key_tick(void)
{
  u32 now = ms_now();
  u8 raw;

  k_sample++;
  if (k_sample < 4u) return;      /* sample every 20 ms */
  k_sample = 0u;

  raw = key_raw();
  k_prev = k_curr;
  k_curr = raw;

  if ((k_curr != 0u) && (k_prev == 0u))
  {
    k_down_ms = now;
    k_long_done = 0u;
    if (k_clicks < 3u) k_clicks++;
  }

  if ((k_curr != 0u) && (k_long_done == 0u) && ((now - k_down_ms) >= 1200u))
  {
    k_long_done = 1u;
    k_clicks = 0u;
    key_event = 4u;
    tx_puts(UART5, "KL\r\n");
  }

  if ((k_curr == 0u) && (k_prev != 0u))
  {
    k_up_ms = now;
  }

  if ((k_curr == 0u) && (k_long_done == 0u) && (k_clicks > 0u) &&
      ((now - k_up_ms) >= 300u))
  {
    key_event = k_clicks;
    if (k_clicks == 1u) tx_puts(UART5, "K1\r\n");
    if (k_clicks == 2u) tx_puts(UART5, "K2\r\n");
    if (k_clicks == 3u) tx_puts(UART5, "K3\r\n");
    k_clicks = 0u;
  }
}

/* start FSM2 (competition flow) */
/* A new FSM2 run is allowed only when no safety-stop cause is still active. */
static u8 safety_restart_ok(void)
{
  if (g_stall.stalled != 0u) return 0u;
  if (Safety_MinDistanceStop(&g_sf, (u16)(SAFE_STOP_MM + SAFE_RESTART_MARGIN_MM)) != 0u)
  {
    return 0u;
  }
  return 1u;
}

/* Left-laser dropout compensation: hold the last valid dL for 300 ms.
   The hardware link is still flaky; this keeps the edge loop running over
   short dropouts instead of punching the steering to 0 every lost frame. */
static u16 edge_dl_read(void)
{
  u32 now = ms_now();

  if ((g_laser.dL != 0xFFFFu) && ((g_laser.flags & 0x02u) != 0u))
  {
    laser_dl_hold_mm = g_laser.dL;
    laser_dl_hold_ms = now;
    laser_dl_hold_ok = 1u;
    return laser_dl_hold_mm;
  }
  if ((laser_dl_hold_ok != 0u) && ((now - laser_dl_hold_ms) <= 300u))
  {
    return laser_dl_hold_mm;
  }
  laser_dl_hold_ok = 0u;
  return 0xFFFFu;
}

static void edge_target_reset(void)
{
  edge_target_ok = 0u;
  edge_target_mm = 0u;

  /* If the car is already within the 40..120 mm band when the run starts,
     hold that distance.  Otherwise leave the target unlatched and take the
     first dL < 120 mm after the car starts moving. */
  if ((g_laser.dL != 0xFFFFu) &&
      (g_laser.dL >= EDGE_TARGET_MIN_MM) &&
      (g_laser.dL <= EDGE_TARGET_LATCH_MAX_MM))
  {
    edge_target_mm = g_laser.dL;
    edge_target_ok = 1u;
  }
}

static void fsm2_begin(void)
{
  abort_all();
  edge_target_reset();
  Fsm2_Start();
  fsm2_mode = 1u;
  fsm2_dist0 = App_GetDistanceCm();
  diff_i = 0.0f;
  yaw_deg = 0.0f;             /* heading reference = current placement */
  tx_puts(UART5, "FSM2 START\r\n");
}

/* start edge-follow test */
static void edge_begin(void)
{
  s16 esp[4];

  abort_all();
  Fsm2_Init();                /* clear the stale F2 fault/state for this test */
  if ((EDGE_KP > 0.0f) &&
      ((g_laser.dL == 0xFFFFu) || ((g_laser.flags & 0x02u) == 0u)))
  {
    tx_puts(UART5, "EDGE: LEFT LASER INVALID\r\n");
    return;                   /* 4WS edge needs dL; pure differential does not */
  }
  edge_target_reset();
  edge_base_cm = App_GetDistanceCm();
  edge_mode = 1u;
  edge_yaw_i = 0.0f;
  diff_i = 0.0f;
  yaw_deg = 0.0f;             /* heading reference = current placement */
  App_SetServoCenter();
  esp[0] = EDGE_SPD; esp[1] = EDGE_SPD; esp[2] = EDGE_SPD; esp[3] = EDGE_SPD;
  App_SetWheelSpeed(esp);
  speed_reset();
  tx_puts(UART5, "EDGE START\r\n");
}

void reverse_start(void)
{
  rev_counts = 0;
  rev_start_ms = ms_now();
  yaw_deg = 0.0f;
  App_SetServoCenter();
  rev_state = 1u;
  app_override = 0u;
  open_loop = 0u;
  motor_run = -1;
  spd_target = RUN_SPD;
  speed_reset();
  tx_puts(USART1, "REV START\r\n");
  tx_puts(UART5,  "REV START\r\n");
}

static void shift_start(s16 dir)
{
  abort_all();
  Fsm2_Init();                /* clear the stale F2 fault/state for this test */
  shift_mode  = 1u;
  shift_dir   = dir;
  shift_show  = 0u;
  shift_phase = 0u;
  shift_start_ms = ms_now();
  shift_t0    = ms_now();
  shift_base_cm = App_GetDistanceCm();
  cent_show   = 0u;
  yaw_deg     = 0.0f;
  shift_yaw_target = 0.0f;
  shift_df_filt = 0.0f;
  ridge_integ = 0.0f;
  App_DriveStop();
  App_SetSteerAll((float)(90 * dir));   /* all four wheels crab 90 deg */
  tx_puts(USART1, "SHIFT START\r\n");
  tx_puts(UART5,  "SHIFT START\r\n");
}

static void tx_hexb(USART_TypeDef *u, u8 v)
{
  const char *h = "0123456789ABCDEF";
  tx_putc(u, (u8)h[v >> 4]);
  tx_putc(u, (u8)h[v & 0x0Fu]);
}

/* ======================================================================= */
/*  app_if.h hooks (used by the delegated modules: centering / fsm)        */
/* ======================================================================= */
u32 App_MsNow(void)
{
  return ms_now();
}

void App_DriveStop(void)
{
  motor_run = 0;
  app_override = 0u;
  speed_reset();
}

void App_SetWheelSpeed(const s16 spd_tgt_in[4])
{
  u8 w;
  int sum = 0;

  for (w = 0u; w < 4u; w++)
  {
    spd_tgt[w] = spd_tgt_in[w];
    sum += (int)spd_tgt_in[w];
  }

  if (sum > 0)
  {
    if (motor_run != 1) { speed_reset(); }
    motor_run = 1;
  }
  else if (sum < 0)
  {
    if (motor_run != -1) { speed_reset(); }
    motor_run = -1;
  }
  else
  {
    motor_run = 0;
  }
  app_override = 1u;
}

u16 App_GetDistanceCm(void)
{
  return (u16)((float)odom_counts * 3.14159f * WHEEL_DIAMETER_MM
               / (10.0f * ENCODER_CPR));
}

static void servo_set_deg(u8 id, float deg)
{
  float units = (float)SERVO_CENTER + deg * SERVO_UNIT_PER_DEG;
  s16 pos;
  if (units < 0.0f)    units = 0.0f;
  if (units > 1000.0f) units = 1000.0f;
  pos = (s16)units;
  servo_move(id, (u16)pos, SERVO_FAST_MS);
}

void App_SetServoCenter(void)
{
  servo_broadcast_deg(0);
}

void App_SetSteerAll(float deg)
{
  servo_broadcast_deg((int)deg);
}

void App_SetSteerRotate(float deg, float rot)
{
  u8 w;
  float d;
  for (w = 0u; w < 4u; w++)
  {
    d = (w < 2u) ? (deg * rot) : (-deg * rot);
    servo_set_deg(WHEEL_SERVO_ID[w], d);
  }
}

void App_SetSteer4(float fl, float fr, float rl, float rr)
{
  servo_set_deg(WHEEL_SERVO_ID[0], fl);
  servo_set_deg(WHEEL_SERVO_ID[1], fr);
  servo_set_deg(WHEEL_SERVO_ID[2], rl);
  servo_set_deg(WHEEL_SERVO_ID[3], rr);
}

/* ======================================================================= */
/*  main                                                                  */
/* ======================================================================= */
void Fault_Dump(void)
{
  tx_puts(UART5, "\r\nFAULT CFSR=");
  tx_hexb(UART5, (u8)((SCB->CFSR >> 24) & 0xFFu)); tx_hexb(UART5, (u8)((SCB->CFSR >> 16) & 0xFFu));
  tx_hexb(UART5, (u8)((SCB->CFSR >> 8) & 0xFFu));  tx_hexb(UART5, (u8)(SCB->CFSR & 0xFFu));
  tx_puts(UART5, " HFSR=");
  tx_hexb(UART5, (u8)((SCB->HFSR >> 24) & 0xFFu)); tx_hexb(UART5, (u8)((SCB->HFSR >> 16) & 0xFFu));
  tx_hexb(UART5, (u8)((SCB->HFSR >> 8) & 0xFFu));  tx_hexb(UART5, (u8)(SCB->HFSR & 0xFFu));
  tx_puts(UART5, " MMFAR=");
  tx_hexb(UART5, (u8)((SCB->MMFAR >> 24) & 0xFFu)); tx_hexb(UART5, (u8)((SCB->MMFAR >> 16) & 0xFFu));
  tx_hexb(UART5, (u8)((SCB->MMFAR >> 8) & 0xFFu));  tx_hexb(UART5, (u8)(SCB->MMFAR & 0xFFu));
  tx_puts(UART5, " BFAR=");
  tx_hexb(UART5, (u8)((SCB->BFAR >> 24) & 0xFFu)); tx_hexb(UART5, (u8)((SCB->BFAR >> 16) & 0xFFu));
  tx_hexb(UART5, (u8)((SCB->BFAR >> 8) & 0xFFu));  tx_hexb(UART5, (u8)(SCB->BFAR & 0xFFu));
  tx_puts(UART5, "\r\n");
}

static void boot_reset_report(void)
{
  u32 csr = RCC->CSR;

  tx_puts(USART1, "RST:");
  tx_puts(UART5,  "RST:");
  if ((csr & (1u << 29)) != 0u) { tx_puts(USART1, "IWDG "); tx_puts(UART5, "IWDG "); }
  if ((csr & (1u << 28)) != 0u) { tx_puts(USART1, "SW   "); tx_puts(UART5, "SW   "); }
  if ((csr & (1u << 27)) != 0u) { tx_puts(USART1, "POR  "); tx_puts(UART5, "POR  "); }
  if ((csr & (1u << 26)) != 0u) { tx_puts(USART1, "PIN  "); tx_puts(UART5, "PIN  "); }
  if ((csr & (1u << 25)) != 0u) { tx_puts(USART1, "BOR  "); tx_puts(UART5, "BOR  "); }
  if ((csr & (1u << 24)) != 0u) { tx_puts(USART1, "PDR  "); tx_puts(UART5, "PDR  "); }
  if ((csr & (1u << 30)) != 0u) { tx_puts(USART1, "WWDG "); tx_puts(UART5, "WWDG "); }
  if ((csr & (1u << 31)) != 0u) { tx_puts(USART1, "LPWR "); tx_puts(UART5, "LPWR "); }
  tx_puts(USART1, " CSR=");
  tx_puts(UART5,  " CSR=");
  tx_hexb(USART1, (u8)((csr >> 24) & 0xFFu)); tx_hexb(USART1, (u8)((csr >> 16) & 0xFFu));
  tx_hexb(USART1, (u8)((csr >> 8) & 0xFFu));  tx_hexb(USART1, (u8)(csr & 0xFFu));
  tx_hexb(UART5,  (u8)((csr >> 24) & 0xFFu)); tx_hexb(UART5, (u8)((csr >> 16) & 0xFFu));
  tx_hexb(UART5,  (u8)((csr >> 8) & 0xFFu));  tx_hexb(UART5, (u8)(csr & 0xFFu));
  tx_puts(USART1, "\r\n");
  tx_puts(UART5,  "\r\n");
  RCC_ClearFlag();
}

int main(void)
{
  u32 next_ctrl;
  u32 next_screen;
  u32 next_log;
  u32 now;

  ms_init();

  systemInit();

  imu_init();

  Laser_Init();

  uart2_init(115200u);
  NVIC_DisableIRQ(USART2_IRQn);
  NVIC_ClearPendingIRQ(USART2_IRQn);

  Safety_Init();
  Fsm2_Init();
  Centering_Init();      /* needs USART2 servo bus ready */

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1 | RCC_AHB1Periph_DMA2, ENABLE);
  NVIC_DisableIRQ(USART1_IRQn);
  NVIC_ClearPendingIRQ(USART1_IRQn);
  NVIC_DisableIRQ(USART3_IRQn);
  NVIC_ClearPendingIRQ(USART3_IRQn);
  NVIC_DisableIRQ(UART5_IRQn);
  NVIC_ClearPendingIRQ(UART5_IRQn);
  cmd_port_init(&port1, USART1, DMA2_Stream2, DMA_Channel_4);
  cmd_port_init(&port3, USART3, DMA1_Stream1, DMA_Channel_4);
  cmd_port_init(&port5, UART5, DMA1_Stream0, DMA_Channel_4);

  boot_reset_report();
  tx_puts(USART1, "\r\nC30D speed-loop ready\r\n");
  tx_puts(USART3, "\r\nC30D speed-loop ready\r\n");
  tx_puts(UART5,  "\r\nC30D speed-loop ready\r\n");
  cmd_help(USART1);
  cmd_help(USART3);
  cmd_help(UART5);

  iwdg_init();                 /* ~1 s watchdog: a hung loop resets the board */
  servo_broadcast_deg(0);
  delay_ms(50u);
  servo_broadcast_deg(0);
  delay_ms(50u);
  servo_broadcast_deg(0);      /* repeat: a reset mid-shift must still centre the servos */
  run_target = (s32)((RUN_CM * 10.0f / (3.14159f * WHEEL_DIAMETER_MM)) * ENCODER_CPR);
  rev_target_counts = (s32)((REV_CM * 10.0f / (3.14159f * WHEEL_DIAMETER_MM)) * ENCODER_CPR);
  run_state = 0u;
  spd_target = SPD_DEFAULT;
  motor_run = 0;
  speed_reset();
  delay_ms(500u);

  next_heartbeat = ms_now() + 250u;
  next_ctrl   = ms_now();
  next_screen = ms_now();
  next_log    = ms_now() + 1000u;

  while (1)
  {
    IWDG->KR = 0xAAAAu;        /* feed the watchdog */
    now = ms_now();

    if (now >= next_heartbeat)
    {
      next_heartbeat += 250u;
      GPIO_ToggleBits(GPIOE, GPIO_Pin_8);   /* V1.1 on-board LED: system heartbeat */
    }

    if (now >= next_ctrl)
    {
      next_ctrl += CTRL_MS;
      if ((s32)(now - next_ctrl) > 200) next_ctrl = now + CTRL_MS;  /* catch up */

      {
        float r = gyro_z_dps() * GYRO_SIGN;
        yaw_rate_dps = r;
        if ((motor_run != 0) && (wheel_test_mode == 0u))
        {
          yaw_deg += r * 0.02f;   /* the wheel test spins on the spot: no yaw */
        }

        if (rev_state != 0u)
        {
          if ((rev_counts >= rev_target_counts) || ((ms_now() - rev_start_ms) > 30000u))
          {
            rev_state = 0u;
            motor_run = 0;
            speed_reset();
            last_run_counts = rev_counts;
            tx_puts(USART1, "REV DONE\r\n");
            tx_puts(UART5,  "REV DONE\r\n");
          }
        }

        if (shift_mode != 0u)
        {
          if (shift_phase == 0u)
          {
            if ((ms_now() - shift_t0) > SHIFT_WAIT_MS) shift_phase = 1u;
          }
          else if (shift_phase == 1u)
          {
            s16 sp[4];
            float r = gyro_z_dps() * GYRO_SIGN;
            float a;
            float doff = 0.0f;
            float cy, yaw_err;

            /* lateral offset from the aux ridge-side laser (direct measurement) */
            /* feed-forward mechanical crab angle + slow integral on ridge distance */
            doff = RIDGE_FF_DEG;
            if ((g_laser.ridge_valid != 0u) &&
                (g_laser.dRidge >= RIDGE_VALID_MIN) && (g_laser.dRidge <= RIDGE_VALID_MAX))
            {
              float rerr = (float)g_laser.dRidge - RIDGE_TARGET_MM;
              if ((rerr > RIDGE_DEADBAND_MM) || (rerr < -RIDGE_DEADBAND_MM))
              {
                ridge_integ += RIDGE_KI * rerr * 0.02f;
              }
              if (ridge_integ >  RIDGE_I_MAX) ridge_integ =  RIDGE_I_MAX;
              if (ridge_integ < -RIDGE_I_MAX) ridge_integ = -RIDGE_I_MAX;
            }
            else
            {
              ridge_integ = 0.0f;   /* sensor sees nothing plausible: drop the windup */
            }
            doff += ridge_integ;
            if (doff >  RIDGE_OFFSET_MAX) doff =  RIDGE_OFFSET_MAX;
            if (doff < 0.0f) doff = 0.0f;
            shift_ridge_dbg = (s16)(doff * 10.0f);
            a = (float)(90 * shift_dir) + doff;

            /* outer loop: front-laser distance -> slow heading target so that
               the car axis stays parallel to the ridge wall                  */
            if ((g_laser.dF != 0xFFFFu) && (g_laser.dF >= 80u) && (g_laser.dF <= 500u))
            {
              if (shift_df_filt == 0.0f) shift_df_filt = (float)g_laser.dF;
              shift_df_filt += SHIFT_DF_FILT_ALPHA * ((float)g_laser.dF - shift_df_filt);

              shift_yaw_target += SHIFT_DF_LOOP_SIGN * SHIFT_DF_KI *
                                  (SHIFT_DF_TARGET_MM - shift_df_filt) * 0.02f;
              if (shift_yaw_target >  SHIFT_YAW_TARGET_MAX) shift_yaw_target =  SHIFT_YAW_TARGET_MAX;
              if (shift_yaw_target < -SHIFT_YAW_TARGET_MAX) shift_yaw_target = -SHIFT_YAW_TARGET_MAX;
            }

            /* inner loop: yaw error against the slow target -> front/rear speed differential */
            yaw_err = yaw_deg - shift_yaw_target;
            cy = SHIFT_YAW_SPD_KP * yaw_err + SHIFT_YAW_SPD_KD * r;
            if (cy >  SHIFT_YAW_SPD_MAX) cy =  SHIFT_YAW_SPD_MAX;
            if (cy < -SHIFT_YAW_SPD_MAX) cy = -SHIFT_YAW_SPD_MAX;
            cy *= SHIFT_YAW_SPD_SIGN;

            App_SetSteer4(a, a, a, a);

            shift_doff_dbg = (s16)(shift_yaw_target * 10.0f);
            shift_af_dbg   = (s16)(yaw_err * 10.0f);

            sp[0] = (s16)(SHIFT_SPD - cy);   /* front-left  */
            sp[1] = (s16)(SHIFT_SPD - cy);   /* front-right */
            sp[2] = (s16)(SHIFT_SPD + cy);   /* rear-left   */
            sp[3] = (s16)(SHIFT_SPD + cy);   /* rear-right  */
            App_SetWheelSpeed(sp);

            if ((yaw_deg > 12.0f) || (yaw_deg < -12.0f))
            {
              shift_yaw_fault_cnt++;
              if (shift_yaw_fault_cnt > 15u)
              {
                shift_phase = 2u;
                App_DriveStop();
                App_SetServoCenter();
                shift_t0 = ms_now();
                tx_puts(USART1, "SHIFT YAW FAULT\r\n");
                tx_puts(UART5,  "SHIFT YAW FAULT\r\n");
              }
            }
            else
            {
              shift_yaw_fault_cnt = 0u;
            }

            if ((u16)(App_GetDistanceCm() - shift_base_cm) >= SHIFT_TEST_CM)
            {
              shift_phase = 2u;
              App_DriveStop();
              App_SetServoCenter();
              shift_t0 = ms_now();
            }
            else if ((g_laser.dF != 0xFFFFu) && (g_laser.dF < 150u))
            {
              shift_phase = 2u;
              App_DriveStop();
              App_SetServoCenter();
              shift_t0 = ms_now();
              tx_puts(USART1, "SHIFT FRONT NEAR\r\n");
              tx_puts(UART5,  "SHIFT FRONT NEAR\r\n");
            }
            else if (((u16)(App_GetDistanceCm() - shift_base_cm) > 50u) &&
                     (((g_laser.dL != 0xFFFFu) && (g_laser.dL <= SHIFT_BOUNDARY_MM)) ||
                      ((g_laser.dR != 0xFFFFu) && (g_laser.dR <= SHIFT_BOUNDARY_MM))))
            {
              shift_phase = 2u;
              App_DriveStop();
              App_SetServoCenter();
              shift_t0 = ms_now();
              tx_puts(USART1, "SHIFT BOUNDARY\r\n");
              tx_puts(UART5,  "SHIFT BOUNDARY\r\n");
            }
          }
          else
          {
            if ((ms_now() - shift_t0) > SHIFT_WAIT_MS)
            {
              shift_mode = 0u;
              shift_show = 1u;
              tx_puts(USART1, "SHIFT DONE\r\n");
              tx_puts(UART5,  "SHIFT DONE\r\n");
            }
          }
          if ((shift_phase < 2u) &&
              ((ms_now() - shift_start_ms) > SHIFT_TIMEOUT_MS))
          {
            shift_mode = 0u;
            shift_show = 1u;
            App_DriveStop();
            App_SetServoCenter();
            tx_puts(USART1, "SHIFT TIMEOUT\r\n");
            tx_puts(UART5,  "SHIFT TIMEOUT\r\n");
          }
        }

        if (cent_mode != 0u)
        {
          s16 cspd[4];
          Centering_Update((const LaserData *)&g_laser, yaw_deg, r, CENT_TEST_SPD, cspd);
          if (g_centering.valid == 0u)
          {
            cspd[0] = CENT_TEST_SPD; cspd[1] = CENT_TEST_SPD;
            cspd[2] = CENT_TEST_SPD; cspd[3] = CENT_TEST_SPD;
          }
          App_SetWheelSpeed(cspd);
          if ((u16)(App_GetDistanceCm() - cent_base_cm) >= CENT_TEST_CM)
          {
            cent_mode = 0u;
            cent_show = 1u;
            App_DriveStop();
            tx_puts(USART1, "CENT DONE\r\n");
            tx_puts(UART5,  "CENT DONE\r\n");
          }
          else if ((g_laser.dF != 0xFFFFu) && (g_laser.dF < 300u))
          {
            cent_mode = 0u;
            cent_show = 1u;
            App_DriveStop();
            tx_puts(USART1, "CENT FRONT OBSTACLE\r\n");
            tx_puts(UART5,  "CENT FRONT OBSTACLE\r\n");
          }
          else if ((ms_now() - cent_start_ms) > 40000u)
          {
            cent_mode = 0u;
            cent_show = 1u;
            App_DriveStop();
            tx_puts(USART1, "CENT TIMEOUT\r\n");
            tx_puts(UART5,  "CENT TIMEOUT\r\n");
          }
          else if (Laser_IsFresh() == 0u)
          {
            cent_lost_ticks++;
            if (cent_lost_ticks > 25u)      /* 500 ms sustained loss */
            {
              cent_mode = 0u;
              cent_show = 1u;
              App_DriveStop();
              tx_puts(USART1, "CENT LASER LOST\r\n");
              tx_puts(UART5,  "CENT LASER LOST\r\n");
            }
          }
          else
          {
            cent_lost_ticks = 0u;
          }
        }

        if (run_state == 1u)
        {
          {
            float rdps = gyro_z_dps() * GYRO_SIGN;
            float sd   = RUN_YAW_KP * yaw_deg + RUN_YAW_KD * rdps;
            if (sd >  RUN_YAW_MAX) sd =  RUN_YAW_MAX;
            if (sd < -RUN_YAW_MAX) sd = -RUN_YAW_MAX;
            sd *= RUN_YAW_SIGN;
            run_yaw_deg_dbg = (s16)(sd * 10.0f);
            App_SetSteer4(sd, sd, 0.0f, 0.0f);   /* front steer, rear straight */
          }

          if ((g_laser.dL != 0xFFFFu) && (g_laser.dR != 0xFFFFu) &&
              (g_laser.dL < 1000u) && (g_laser.dR < 1000u))
          {
            float off = ((float)g_laser.dL - (float)g_laser.dR) * 0.5f;
            float dd  = 0.0f;
            if ((off > 15.0f) || (off < -15.0f))
            {
              dd = RUN_LAT_SIGN * RUN_LAT_KP * off;
            }
            run_lat_off_dbg = (s16)off;
            run_lat_deg_dbg = (s16)(dd * 10.0f);
            if (dd >  RUN_LAT_MAX) dd =  RUN_LAT_MAX;
            if (dd < -RUN_LAT_MAX) dd = -RUN_LAT_MAX;
            App_SetSteer4(dd, dd, dd, dd);
          }
        }

        if (diff_test_mode != 0u)
        {
          if ((s32)((u16)(App_GetDistanceCm() - diff_test_base)) >= 100)
          {
            diff_test_mode = 0u;
            diff_show = 1u;
            motor_run = 0;
            speed_reset();
            App_SetServoCenter();
            tx_puts(UART5, "DIFFTEST DONE\r\n");
          }
        }

        if (diff_cal_mode != 0u)
        {
          s16 dsp[4];
          /* 0=LF 1=RF 2=LB 3=RB: D+ = left slow / right fast */
          if (diff_cal_dir > 0) { dsp[0] = 700; dsp[1] = 900; dsp[2] = 700; dsp[3] = 900; }
          else                  { dsp[0] = 900; dsp[1] = 700; dsp[2] = 900; dsp[3] = 700; }
          App_SetWheelSpeed(dsp);
          diff_cal_counts = (s32)((u16)(App_GetDistanceCm() - diff_cal_base_cm));
          if (diff_cal_counts >= 100)
          {
            diff_cal_mode = 0u;
            motor_run = 0;
            speed_reset();
            App_SetServoCenter();
            tx_puts(UART5, "DIFFCAL DONE yaw=");
            tx_num(UART5, (int)(yaw_deg * 10.0f));
            tx_puts(UART5, "\r\n");
          }
        }

        if (cal_mode != 0u)
        {
          App_SetSteer4(8.0f * cal_dir, 8.0f * cal_dir, -8.0f * cal_dir, -8.0f * cal_dir);
          cal_counts = (s32)((u16)(App_GetDistanceCm() - cal_base_cm));
          if (cal_counts >= 100)
          {
            cal_mode = 0u;
            motor_run = 0;
            speed_reset();
            App_SetServoCenter();
            tx_puts(UART5, "CAL DONE yaw=");
            tx_num(UART5, (int)(yaw_deg * 10.0f));
            tx_puts(UART5, "\r\n");
          }
        }

        if ((edge_mode != 0u) &&
            ((Safety_MinDistanceStop(&g_sf, SAFE_STOP_MM) != 0u) ||
             (g_stall.stalled != 0u)))
        {
          App_DriveStop();
          edge_mode = 0u;
          tx_puts(UART5, "EDGE SAFE STOP\r\n");
        }

        if (edge_mode != 0u)
        {
          float e = 0.0f;
          float d = 0.0f;
          u16   dl = edge_dl_read();

          {
            float dy;
            float fr, rr;

            if ((edge_target_ok == 0u) &&
                (dl != 0xFFFFu) &&
                (dl >= EDGE_TARGET_MIN_MM) &&
                (dl <= EDGE_TARGET_LATCH_MAX_MM))
            {
              edge_target_mm = dl;
              edge_target_ok = 1u;
            }

            if ((edge_target_ok != 0u) &&
                (dl != 0xFFFFu) && (dl < 1000u))
            {
              e = (float)dl - (float)edge_target_mm;
              d = EDGE_SIGN * EDGE_KP * e;
              if (d >  EDGE_MAX) d =  EDGE_MAX;
              if (d < -EDGE_MAX) d = -EDGE_MAX;
            }
            edge_steer_dbg = (s16)(d * 10.0f);

            edge_yaw_i += 1.0f * yaw_deg * 0.02f;
            if (edge_yaw_i >  5.0f) edge_yaw_i =  5.0f;
            if (edge_yaw_i < -5.0f) edge_yaw_i = -5.0f;
            dy = EDGE_YAW_SIGN * (EDGE_YAW_KP * yaw_deg + edge_yaw_i +
                                  EDGE_YAW_KD * (gyro_z_dps() * GYRO_SIGN));
            if (dy >  EDGE_YAW_MAX) dy =  EDGE_YAW_MAX;
            if (dy < -EDGE_YAW_MAX) dy = -EDGE_YAW_MAX;

            fr = d + dy;
            rr = d - dy;
            App_SetSteer4(fr, fr, rr, rr);
          }

          {
            s16 esp[4];
            float rate2 = gyro_z_dps() * GYRO_SIGN;
            float corr;

            diff_i += diff_ki * yaw_deg * 0.02f;
            if (diff_i >  diff_max) diff_i =  diff_max;
            if (diff_i < -diff_max) diff_i = -diff_max;

            corr = diff_ff + diff_kp * yaw_deg + diff_i + diff_kd * rate2;
            if (corr >  diff_max) corr =  diff_max;
            if (corr < -diff_max) corr = -diff_max;

            /* left/right differential: 0=LF 1=RF 2=LB 3=RB */
            esp[0] = (s16)(EDGE_SPD + corr);   /* LF */
            esp[1] = (s16)(EDGE_SPD - corr);   /* RF */
            esp[2] = (s16)(EDGE_SPD + corr);   /* LB */
            esp[3] = (s16)(EDGE_SPD - corr);   /* RB */
            App_SetWheelSpeed(esp);
          }

          if (((g_laser.dF != 0xFFFFu) && (g_laser.dF <= ENTRY_STOP_MM)) ||
              ((g_laser.dF == 0xFFFFu) &&
               ((u16)(App_GetDistanceCm() - edge_base_cm) > 400u)))
          {
            edge_mode = 0u;
            motor_run = 0;
            speed_reset();
            App_SetServoCenter();
            tx_puts(UART5, "EDGE DONE\r\n");
          }
        }

        /* Sample the safety sensors every 20 ms, even while FSM2 is idle.
           Otherwise a SAFE STOP freezes the median filter at the close
           reading and the first tick of the next start re-trips before the
           filter can see that the obstacle is gone. */
        {
          SafetyRaw raw;
          u16 apwm[4];
          u8 wi;

          raw.dF = g_laser.dF; raw.dB = g_laser.dRidge;
          raw.dL = g_laser.dL; raw.dR = g_laser.dR;
          raw.vF = (u8)(((g_laser.flags & 0x01u) != 0u) ? 1u : 0u);
          raw.vL = (u8)(((g_laser.flags & 0x02u) != 0u) ? 1u : 0u);
          raw.vR = (u8)(((g_laser.flags & 0x04u) != 0u) ? 1u : 0u);
          raw.vB = g_laser.ridge_valid;

          Safety_Filter(&raw, &g_sf);
          Safety_CrabFusion(&g_sf, &g_sc);
          for (wi = 0u; wi < 4u; wi++)
          {
            apwm[wi] = (u16)((pwm_out[wi] < 0) ? -pwm_out[wi] : pwm_out[wi]);
          }
          Safety_StallUpdate(&g_stall, apwm, (const s16 *)meas_spd, 20u);
        }

        if (fsm2_mode != 0u)
        {
          s16 spd2[4];

          if ((Safety_MinDistanceStop(&g_sf, SAFE_STOP_MM) != 0u) || (g_stall.stalled != 0u))
          {
            App_DriveStop();
            Fsm2_Abort();
            fsm2_mode = 0u;
            tx_puts(UART5, "SAFE STOP\r\n");
          }
          else
          {
            g_f2in.dF = g_sf.dF; g_f2in.dB = g_sf.dB;
            g_f2in.dL = g_sf.dL; g_f2in.dR = g_sf.dR;
            g_f2in.vF = g_sf.vF; g_f2in.vB = g_sf.vB;
            g_f2in.vL = g_sf.vL; g_f2in.vR = g_sf.vR;
            g_f2in.dist_cm = (s32)((u16)(App_GetDistanceCm() - fsm2_dist0));
            g_f2in.yaw_deg = yaw_deg;
            g_f2in.stall = g_stall.stalled;
            g_f2in.ms = ms_now();

            Fsm2_Tick(&g_f2in, &g_f2out);

            if (g_f2out.request_stop != 0u)
            {
              App_DriveStop();
            }
            else
            {
              spd2[0] = RUN_SPD; spd2[1] = RUN_SPD;
              spd2[2] = RUN_SPD; spd2[3] = RUN_SPD;
              switch (g_f2out.action)
              {
                case F2A_FWD:
                  if (Fsm2_GetState() == F2_ENTRY)
                  {
                    s16 esp[4];
                    float e2 = 0.0f, d2 = 0.0f, c2, r2;
                    u16   dl = edge_dl_read();
                    if ((edge_target_ok == 0u) &&
                        (dl != 0xFFFFu) &&
                        (dl >= EDGE_TARGET_MIN_MM) &&
                        (dl <= EDGE_TARGET_LATCH_MAX_MM))
                    {
                      edge_target_mm = dl;
                      edge_target_ok = 1u;
                    }

                    if ((edge_target_ok != 0u) &&
                        (dl != 0xFFFFu) && (dl < 1000u))
                    {
                      e2 = (float)dl - (float)edge_target_mm;
                      d2 = EDGE_SIGN * EDGE_KP * e2;
                      if (d2 >  EDGE_MAX) d2 =  EDGE_MAX;
                      if (d2 < -EDGE_MAX) d2 = -EDGE_MAX;
                    }
                    edge_steer_dbg = (s16)(d2 * 10.0f);
                    App_SetSteer4(d2, d2, d2, d2);
                    r2 = gyro_z_dps() * GYRO_SIGN;
                    diff_i += diff_ki * yaw_deg * 0.02f;
                    if (diff_i >  diff_max) diff_i =  diff_max;
                    if (diff_i < -diff_max) diff_i = -diff_max;
                    c2 = diff_ff + diff_kp * yaw_deg + diff_i + diff_kd * r2;
                    if (c2 >  diff_max) c2 =  diff_max;
                    if (c2 < -diff_max) c2 = -diff_max;
                    /* left/right differential: 0=LF 1=RF 2=LB 3=RB */
                    esp[0] = (s16)(EDGE_SPD + c2);   /* LF */
                    esp[1] = (s16)(EDGE_SPD - c2);   /* RF */
                    esp[2] = (s16)(EDGE_SPD + c2);   /* LB */
                    esp[3] = (s16)(EDGE_SPD - c2);   /* RB */
                    App_SetWheelSpeed(esp);
                  }
                  else
                  {
                    App_SetSteerAll(0.0f);
                    App_SetWheelSpeed(spd2);
                  }
                  break;
                case F2A_CRAB_R: App_SetSteerAll(90.0f);  App_SetWheelSpeed(spd2); break;
                case F2A_CRAB_L: App_SetSteerAll(-90.0f); App_SetWheelSpeed(spd2); break;
                case F2A_REV:    App_SetSteerAll(0.0f);
                                 spd2[0] = -RUN_SPD; spd2[1] = -RUN_SPD;
                                 spd2[2] = -RUN_SPD; spd2[3] = -RUN_SPD;
                                 App_SetWheelSpeed(spd2); break;
                default:         App_DriveStop(); break;
              }
            }

            if ((g_f2out.done != 0u) || (Fsm2_GetState() == F2_FAULT))
            {
              App_DriveStop();
              fsm2_mode = 0u;
            }
          }
        }
      }
      speed_tick();
    }

    delay_ms(5u);
    Laser_Poll();
    cmd_poll(&port1);
    cmd_poll(&port3);
    cmd_poll(&port5);

    key_tick();

    if (key_event != 0u)
    {
      u8 ev = key_event;
      key_event = 0u;

      if (ev == 1u)
      {
        if (wheel_test_mode != 0u)
        {
          wheel_idx = (u8)((wheel_idx + 1u) & 3u);
          wheel_t_on = ms_now();
          wheel_peak[0] = wheel_peak[1] = wheel_peak[2] = wheel_peak[3] = 0;
        }
        else if (fsm2_mode != 0u)
        {
          tx_puts(UART5, "FSM2 RUNNING\r\n");
        }
        else if (edge_mode != 0u)
        {
          tx_puts(UART5, "EDGE RUNNING\r\n");
        }
        else if (safety_restart_ok() != 0u)
        {
          edge_begin();              /* single click: edge-follow test (S1) */
        }
        else
        {
          tx_puts(UART5, "SAFE: MOVE AWAY\r\n");
        }
      }
      else if (ev == 2u)
      {
        if (fsm2_mode != 0u)
        {
          tx_puts(UART5, "FSM2 RUNNING\r\n");
        }
        else if (safety_restart_ok() != 0u)
        {
          fsm2_begin();              /* double click: full FSM2 flow */
        }
        else
        {
          tx_puts(UART5, "SAFE: MOVE AWAY\r\n");
        }
      }
      else if (ev == 3u)
      {
        if (wheel_test_mode != 0u)
        {
          abort_all();               /* also clears wheel_test_mode */
          tx_puts(UART5, "WHEEL TEST EXIT\r\n");
        }
        else
        {
          shift_start(1);            /* triple click: crab right test */
          tx_puts(UART5, "CRAB RIGHT START\r\n");
        }
      }
      else if (ev == 4u)
      {
        abort_all();
        App_SetServoCenter();      /* wheels straight: "forward" is meaningful */
        wheel_test_mode = 1u;
        wheel_idx = 0u;
        wheel_t_on = ms_now();
        wheel_peak[0] = wheel_peak[1] = wheel_peak[2] = wheel_peak[3] = 0;
        tx_puts(UART5, "WHEEL ID START\r\n");
        diff_test_mode = 0u;
      }
      else if (ev == 5u)
      {
        abort_all();
        diff_test_mode = 1u;
        diff_show = 0u;
        diff_test_counts = 0;
        diff_test_base = App_GetDistanceCm();
        motor_run = 1;
        app_override = 1u;
        App_SetServoCenter();
        speed_reset();
        tx_puts(UART5, "DIFFTEST START\r\n");
      }
    }

    if ((run_state == 1u) && (run_counts >= run_target))
    {
      run_stop("70CM DONE");
    }
    if ((run_state == 1u) && ((ms_now() - run_start_ms) > RUN_TIMEOUT_MS))
    {
      run_stop("70CM TIMEOUT");
    }

    if ((batt_volt > 3.0f) && (batt_volt < 10.5f) && (motor_run != 0))
    {
      motor_run = 0;
      speed_reset();
      App_DriveStop();
      tx_puts(UART5, "LOW BATT STOP\r\n");
    }

    if ((cent_mode == 0u) && (shift_mode == 0u) && (rev_state == 0u) &&
        (fsm2_mode == 0u) && (edge_mode == 0u) && (cal_mode == 0u) &&
        (diff_cal_mode == 0u) && (run_state != 1u) &&
        (motor_run != 0) &&
        ((ms_now() - last_cmd_ms) > CMD_TIMEOUT_MS))
    {
      motor_run = 0;
      speed_reset();
      tx_puts(USART1, "TIMEOUT -> STOP\r\n");
      tx_puts(USART3, "TIMEOUT -> STOP\r\n");
    }

    if (ms_now() >= next_screen)
    {
      next_screen += 200u;
      screen();
    }

    if (ms_now() >= next_log)
    {
      u8 k, w;
      next_log += 1000u;
      for (k = 0u; k < 3u; k++)
      {
        USART_TypeDef *u = (k == 0u) ? USART1 : ((k == 1u) ? USART3 : UART5);
        tx_puts(u, (open_loop != 0u) ? "OL PWM=" : (motor_run > 0 ? "SPD FWD " : (motor_run < 0 ? "SPD BACK " : "SPD STOP ")));
        if (open_loop != 0u) { tx_num(u, (int)open_pwm); tx_putc(u, ' '); }
        tx_puts(u, "tgt=");
        tx_num(u, (int)spd_target);
        tx_puts(u, " MR=");
        tx_num(u, (int)motor_run);
        tx_puts(u, " CM=");
        tx_num(u, (int)cent_mode);
        tx_puts(u, " T0=");
        tx_num(u, (int)spd_tgt[0]);
        tx_puts(u, " ");
        for (w = 0u; w < 4u; w++)
        {
          tx_num(u, (int)meas_spd[w]);
          tx_putc(u, '/');
          tx_num(u, (int)pwm_out[w]);
          tx_putc(u, ' ');
        }
        batt_volt = (float)Get_Adc(9) * 3.3f * 11.0f * 0.99f / 4096.0f;
        tx_puts(u, " F2=");
        tx_num(u, (int)Fsm2_GetState());
        tx_puts(u, " FA=");
        tx_num(u, (int)Fsm2_GetFaultCode());
        tx_puts(u, " K=");
        tx_num(u, (int)key_raw());
        tx_puts(u, " VB=");
        tx_num(u, (int)(batt_volt * 10.0f));
        tx_puts(u, " SY=");
        tx_num(u, (int)run_yaw_deg_dbg);
        tx_puts(u, " R1=");
        tx_num(u, (int)run_hist_cnt[0]); tx_putc(u, ',');
        tx_num(u, (int)run_hist_yaw[0]);
        tx_puts(u, " R2=");
        tx_num(u, (int)run_hist_cnt[1]); tx_putc(u, ',');
        tx_num(u, (int)run_hist_yaw[1]);
        tx_puts(u, " R3=");
        tx_num(u, (int)run_hist_cnt[2]); tx_putc(u, ',');
        tx_num(u, (int)run_hist_yaw[2]);
        tx_puts(u, " LE=");
        tx_num(u, (int)run_lat_off_dbg);
        tx_puts(u, " LD=");
        tx_num(u, (int)run_lat_deg_dbg);
        tx_puts(u, " LC=");
        tx_num(u, (int)last_run_counts);
        tx_puts(u, " DD=");
        tx_num(u, (int)((u16)(App_GetDistanceCm() - shift_base_cm)));
        tx_puts(u, " RD=");
        tx_num(u, (int)shift_ridge_dbg);
        tx_puts(u, " DF=");
        tx_num(u, (int)shift_doff_dbg);
        tx_puts(u, " AF=");
        tx_num(u, (int)shift_af_dbg);
        tx_puts(u, " CE=");
        tx_num(u, (int)(g_centering.e_cm * 10.0f));
        tx_puts(u, " CD=");
        tx_num(u, (int)(g_centering.delta_deg * 10.0f));
        tx_puts(u, " CT=");
        tx_num(u, (int)(g_centering.toe_deg * 10.0f));
        tx_puts(u, " LZ=");
        tx_puts(u, Laser_IsFresh() ? "OK " : "LOST ");
        tx_puts(u, " LS=");
        tx_num(u, (g_laser.dL == 0xFFFFu) ? -1 : (int)g_laser.dL);
        tx_putc(u, ',');
        tx_num(u, (g_laser.dR == 0xFFFFu) ? -1 : (int)g_laser.dR);
        tx_putc(u, ',');
        tx_num(u, (g_laser.dF == 0xFFFFu) ? -1 : (int)g_laser.dF);
        tx_putc(u, ',');
        tx_num(u, (int)g_laser.flags);
        tx_puts(u, " RG=");
        tx_num(u, (g_laser.dRidge == 0xFFFFu) ? -1 : (int)g_laser.dRidge);
        tx_putc(u, ',');
        tx_num(u, (int)g_laser.ridge_flags);
        tx_putc(u, ',');
        tx_num(u, (int)g_laser.ridge_valid);
        tx_puts(u, " LZN=");
        tx_num(u, (int)g_laser.raw_count);
        tx_puts(u, " LB=");
        {
          u8 k2;
          for (k2 = 0u; k2 < 8u; k2++)
          {
            tx_hexb(u, g_laser.raw_last[(u8)((g_laser.raw_idx + k2) & 7u)]);
            tx_putc(u, ' ');
          }
        }
        tx_puts(u, "YAW=");
        tx_num(u, (int)(yaw_deg * 10.0f));
        tx_puts(u, " C=");
        tx_num(u, yaw_corr);
        tx_puts(u, " FF=");
        tx_num(u, (int)diff_ff);
        tx_puts(u, " ET=");
        tx_num(u, (edge_target_ok != 0u) ? (int)edge_target_mm : 0);
        tx_puts(u, " ES=");
        tx_num(u, (int)edge_steer_dbg);
        tx_puts(u, " SVO=");
        tx_num(u, (int)servo_angle);
        tx_puts(u, " P=");
        tx_num(u, (int)last_port);
        tx_puts(u, " CMD=");
        tx_puts(u, last_cmd);
        tx_puts(u, "\r\n");
      }
    }
  }
}
