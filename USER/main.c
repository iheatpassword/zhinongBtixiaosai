/******************************************************************************
 * C30D 4WS / 4WD car : "go straight" + "turn in place 90 degrees"
 *
 * Based on the WHEELTEC C30D V2.0 official source (WHEELTEC V5.0, 2025.12.26).
 * Only USER/main.c is replaced.  The FreeRTOS scheduler is NOT started.
 *
 * Hardware mapping (measured on this car):
 *   wheel        motor    encoder    steering servo (HTS-35H, 0x55 protocol)
 *   FL (front-left)   A    E1/TIM2    ID 4
 *   FR (front-right)  B    E2/TIM3    ID 3
 *   RL (rear-left)    C    E3/TIM4    ID 2
 *   RR (rear-right)   D    E4/TIM5    ID 1
 *
 * Motion primitives
 *   Go_Straight(ms, pwm) : servos to centre, all motors drive "forward"
 *   Turn_InPlace(deg)    : front wheels steer +45 deg, rear wheels -45 deg,
 *                          all motors drive forward -> the car rotates about
 *                          its centre.  A gyroscope (ICM-20948 on I2C) is used
 *                          to stop exactly at the requested angle.
 *
 * Serial ports: USART1 = debug log (115200), UART5 = laser (reserved),
 *               UART4 = IMU (reserved), USART2 = servo bus (115200).
 *
 * IMPORTANT calibration constants below (MOTOR_DIR / SERVO_DIR / SERVO_TRIM /
 * GYRO_SIGN): if a wheel drives the wrong way or the steering looks inverted,
 * flip the corresponding entry and rebuild.
 ******************************************************************************/

#include "system.h"
#include <math.h>

/* ----------------------------------------------------------------------- */
/*  calibration / configuration                                            */
/* ----------------------------------------------------------------------- */
#define SERVO_CENTER        500     /* HTS-35H mechanical centre            */
#define SERVO_UNIT_PER_DEG  4.1667f /* 1000 units / 240 deg                 */
#define SERVO_MIN           0
#define SERVO_MAX           1000

#define STRAIGHT_PWM        3500    /* PWM for straight driving (16799 max) */
#define STRAIGHT_MS         1500u   /* straight duration                    */
#define TURN_PWM            4200    /* PWM while rotating                    */
#define TURN_ANGLE_DEG      90.0f   /* requested in-place rotation          */
#define TURN_STEER_DEG      45.0f   /* counter-steer angle used to rotate   */
#define TURN_TIMEOUT_MS     6000u   /* safety timeout for one rotation      */

#define IMU_GYRO_LSB_PER_DPS 65.5f  /* ICM-20948, +/-500 dps range          */
#define GYRO_SIGN           (+1.0f) /* flip if the yaw sign is inverted     */

/* per physical wheel index: FL=0, FR=1, RL=2, RR=3                    */
static const u8  WHEEL_SERVO[4] = { 4u, 3u, 2u, 1u };
static const u8  WHEEL_MOTOR[4] = { 0u, 1u, 2u, 3u };

/* +1 / -1 : PWM sign that drives that wheel forward                    */
static const s8  MOTOR_DIR[4]   = { -1, 1, -1, 1 };

/* +1 / -1 : steering direction sign (positive command = same side)     */
static const s8  SERVO_DIR[4]   = { 1, 1, 1, 1 };

/* mechanical centre trim per wheel, in servo units                     */
static const s16 SERVO_TRIM[4]  = { 0, 0, 0, 0 };

static float gyro_bias_z;       /* raw LSB while standing still         */

/* ======================================================================= */
/*  USART1 debug                                                          */
/* ======================================================================= */
static void u1_putc(u8 b)
{
  while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET) { }
  USART_SendData(USART1, b);
}

static void u1_puts(const char *s)
{
  while (*s != '\0') u1_putc((u8)*s++);
}

static void u1_num(int v)
{
  char b[10];
  u8 n = 0;
  if (v < 0) { u1_putc('-'); v = -v; }
  if (v == 0) { u1_putc('0'); return; }
  while (v > 0) { b[n++] = (char)('0' + (v % 10)); v /= 10; }
  while (n > 0) u1_putc((u8)b[--n]);
}

static void u1_dec1(int v10)   /* print v10/10 with one decimal */
{
  if (v10 < 0) { u1_putc('-'); v10 = -v10; }
  u1_num(v10 / 10);
  u1_putc('.');
  u1_putc((u8)('0' + (v10 % 10)));
}

/* ======================================================================= */
/*  USART2 : HTS-35H bus servo                                            */
/* ======================================================================= */
static void u2_putc(u8 b)
{
  while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET) { }
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

/* ----------------------------------------------------------------------- */
/*  steering / driving                                                     */
/* ----------------------------------------------------------------------- */
static s16 servo_pos_for_wheel(u8 wheel, float deg)
{
  float units = SERVO_CENTER + (float)SERVO_TRIM[wheel] +
                (float)SERVO_DIR[wheel] * deg * SERVO_UNIT_PER_DEG;
  s16 pos = (s16)units;
  if (pos < SERVO_MIN) pos = SERVO_MIN;
  if (pos > SERVO_MAX) pos = SERVO_MAX;
  return pos;
}

/* steer all wheels to the same angle (deg, + = one side) */
static void steer_all(float deg, u16 move_ms)
{
  u8 w;
  for (w = 0; w < 4u; w++)
  {
    servo_move(WHEEL_SERVO[w], (u16)servo_pos_for_wheel(w, deg), move_ms);
  }
}

/* for in-place rotation: front wheels +deg, rear wheels -deg
   rot = +1 / -1 chooses the rotation direction                     */
static void steer_rotate(float deg, float rot, u16 move_ms)
{
  u8 w;
  float d;
  for (w = 0; w < 4u; w++)
  {
    d = (w < 2u) ? (deg * rot) : (-deg * rot);
    servo_move(WHEEL_SERVO[w], (u16)servo_pos_for_wheel(w, d), move_ms);
  }
}

/* drive all four wheels, pwm > 0 = car forward */
static void drive(int pwm)
{
  u8 w;
  int m[4];
  for (w = 0; w < 4u; w++) m[WHEEL_MOTOR[w]] = pwm * (int)MOTOR_DIR[w];
  Set_Pwm(m[0], m[1], m[2], m[3], 1500);
}

static void drive_stop(void)
{
  Set_Pwm(0, 0, 0, 0, 1500);
}

/* ======================================================================= */
/*  ICM-20948 gyroscope (I2C, software I2C on PB10/PB11)                  */
/* ======================================================================= */
static int16_t gyro_z_raw(void)
{
  u8 lo = I2C_ReadOneByte(0xD0u, 0x38u);   /* GYRO_ZOUT_L */
  u8 hi = I2C_ReadOneByte(0xD0u, 0x37u);   /* GYRO_ZOUT_H */
  return (int16_t)(((u16)hi << 8) | lo);
}

/* select bank 0 and measure the Z gyro bias while the car stands still */
static void gyro_init(void)
{
  long sum = 0;
  u16 i;

  I2C_WriteOneByte(0xD0u, 0x7Fu, 0x00u);   /* REG_BANK_SEL = bank 0 */
  delay_ms(20u);

  for (i = 0; i < 200u; i++)
  {
    sum += gyro_z_raw();
    delay_ms(3u);
  }
  gyro_bias_z = (float)sum / 200.0f;
}

static float gyro_z_dps(void)
{
  return ((float)gyro_z_raw() - gyro_bias_z) / IMU_GYRO_LSB_PER_DPS;
}

/* ======================================================================= */
/*  OLED status                                                           */
/* ======================================================================= */
static void show_signed(u8 x, u8 y, int32_t v, u8 digits)
{
  if (v < 0) { OLED_ShowString(x, y, (u8 *)"-"); v = -v; }
  else       { OLED_ShowString(x, y, (u8 *)"+"); }
  OLED_ShowNumber((u8)(x + 8u), y, (u32)v, digits, 12);
}

static void screen(const char *mode, float yaw_deg)
{
  int16_t e0, e1, e2, e3;

  e0 = (int16_t)Read_Encoder(2);
  e1 = (int16_t)Read_Encoder(3);
  e2 = (int16_t)Read_Encoder(4);
  e3 = (int16_t)Read_Encoder(5);

  OLED_Clear();
  OLED_ShowString(0, 0, (u8 *)"C30D STD/TURN90");
  OLED_ShowString(0, 10, (u8 *)mode);

  OLED_ShowString(0, 20, (u8 *)"YAW");
  show_signed(24, 20, (int32_t)(yaw_deg * 10.0f), 4);

  OLED_ShowString(0, 30, (u8 *)"E1");
  show_signed(16, 30, e0, 4);
  OLED_ShowString(58, 30, (u8 *)"E2");
  show_signed(74, 30, e1, 4);

  OLED_ShowString(0, 40, (u8 *)"E3");
  show_signed(16, 40, e2, 4);
  OLED_ShowString(58, 40, (u8 *)"E4");
  show_signed(74, 40, e3, 4);

  OLED_ShowString(0, 50, (u8 *)"B");
  oled_showfloat(Get_battery_volt(), 8, 50, 2, 1);

  OLED_Refresh_Gram();
}

/* ======================================================================= */
/*  straight driving                                                      */
/* ======================================================================= */
static void Go_Straight(u16 ms, int pwm)
{
  u16 t;
  float yaw = 0.0f;

  u1_puts("Go_Straight ");
  u1_num((int)ms);
  u1_puts(" ms, pwm=");
  u1_num(pwm);
  u1_puts("\r\n");

  steer_all(0.0f, 400u);       /* wheels straight */
  delay_ms(500u);
  drive(pwm);

  for (t = 0; t < ms; t += 20u)
  {
    delay_ms(20u);
    yaw += gyro_z_dps() * 0.02f;       /* just for display  */
    screen("STRAIGHT", yaw);
  }

  drive_stop();
  u1_puts("  yaw=");
  u1_dec1((int)(yaw * 10.0f));
  u1_puts(" deg\r\n");
}

/* ======================================================================= */
/*  in-place rotation                                                     */
/* ======================================================================= */
static void Turn_InPlace(float angle_deg)
{
  float target = fabsf(angle_deg);
  float rot = (angle_deg >= 0.0f) ? 1.0f : -1.0f;
  float yaw = 0.0f;
  float rate, y_test;
  u16 t = 0;

  u1_puts("Turn_InPlace ");
  u1_dec1((int)(target * 10.0f));
  u1_puts(" deg, rot=");
  u1_num((int)rot);
  u1_puts("\r\n");

  /* front wheels +45 deg, rear wheels -45 deg; the drive direction decides
     the rotation direction.  First 300 ms: detect the actual yaw direction
     and flip the drive sign if necessary.                              */
  steer_rotate(TURN_STEER_DEG, 1.0f, 500u);
  delay_ms(700u);

  drive(TURN_PWM);
  y_test = 0.0f;
  for (t = 0; t < 300u; t += 20u)
  {
    delay_ms(20u);
    y_test += (gyro_z_dps() * GYRO_SIGN) * 0.02f;
    screen("TURN 90", y_test);
  }
  u1_puts("  yaw test=");
  u1_dec1((int)(y_test * 10.0f));
  u1_puts("\r\n");

  if ((y_test * rot) < 0.0f)
  {
    drive(-TURN_PWM);          /* wrong direction -> drive backwards */
    u1_puts("  drive sign flipped\r\n");
  }

  yaw = 0.0f;
  t = 0;
  while ((yaw * rot < target) && (t < TURN_TIMEOUT_MS))
  {
    delay_ms(10u);
    t = (u16)(t + 10u);
    rate = gyro_z_dps() * GYRO_SIGN;
    yaw += rate * 0.01f;
    screen("TURN 90", yaw);
  }

  drive_stop();
  steer_all(0.0f, 500u);      /* back to straight */

  u1_puts("  turned ");
  u1_dec1((int)(yaw * 10.0f));
  u1_puts(" deg, t=");
  u1_num((int)t);
  u1_puts(" ms\r\n");

  delay_ms(300u);
}

/* ======================================================================= */
/*  reserved UARTs (laser UART5, IMU UART4)                               */
/* ======================================================================= */
static void uart4_init(u32 baud)
{
  GPIO_InitTypeDef  g;
  USART_InitTypeDef u;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);

  GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_UART4);
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_UART4);

  g.GPIO_Pin   = GPIO_Pin_10 | GPIO_Pin_11;
  g.GPIO_Mode  = GPIO_Mode_AF;
  g.GPIO_Speed = GPIO_Speed_50MHz;
  g.GPIO_OType = GPIO_OType_PP;
  g.GPIO_PuPd  = GPIO_PuPd_UP;
  GPIO_Init(GPIOC, &g);

  u.USART_BaudRate            = baud;
  u.USART_WordLength          = USART_WordLength_8b;
  u.USART_StopBits            = USART_StopBits_1;
  u.USART_Parity              = USART_Parity_No;
  u.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  u.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(UART4, &u);
  USART_Cmd(UART4, ENABLE);
}

/* ======================================================================= */

int main(void)
{
  u8 w;

  systemInit();          /* official hardware init, no RTOS start */

  /* serial ports */
  uart2_init(115200u);   /* bus servos */
  NVIC_DisableIRQ(USART2_IRQn);
  NVIC_ClearPendingIRQ(USART2_IRQn);
  NVIC_DisableIRQ(UART5_IRQn);          /* reserved: laser */
  NVIC_ClearPendingIRQ(UART5_IRQn);
  uart4_init(115200u);                  /* reserved: IMU   */

  u1_puts("\r\nC30D straight / 90deg turn demo\r\n");

  /* boot screen */
  OLED_Clear();
  OLED_ShowString(0, 0, (u8 *)"C30D STD/TURN90");
  OLED_ShowString(0, 10, (u8 *)"SERVO:USART2");
  OLED_ShowString(0, 20, (u8 *)"FL A-E1-S4");
  OLED_ShowString(0, 30, (u8 *)"FR B-E2-S3");
  OLED_ShowString(0, 40, (u8 *)"RL C-E3-S2");
  OLED_ShowString(0, 50, (u8 *)"RR D-E4-S1");
  OLED_Refresh_Gram();
  delay_ms(1500);

  /* centre all steering servos */
  steer_all(0.0f, 500u);
  drive_stop();
  delay_ms(800u);

  /* gyro bias calibration, car must stand still (about 0.6 s) */
  gyro_init();
  u1_puts("gyro bias=");
  u1_num((int)gyro_bias_z);
  u1_puts("\r\n");

  for (w = 0; w < 4u; w++) (void)Read_Encoder((u8)(2u + w));

  while (1)
  {
    Go_Straight(STRAIGHT_MS, STRAIGHT_PWM);
    delay_ms(700u);

    Turn_InPlace(+TURN_ANGLE_DEG);
    delay_ms(700u);

    Go_Straight(STRAIGHT_MS, STRAIGHT_PWM);
    delay_ms(700u);

    Turn_InPlace(-TURN_ANGLE_DEG);
    delay_ms(700u);
  }
}
