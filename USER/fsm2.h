/**
  * @file    fsm2.h
  * @brief   ¢ҵ״̬߼ģ飬κӲ
  *
  *  Ŀƽ̨: STM32F407 + Keil MDK / ARMCC5C90޶̬ڴ棬 printf
  *  ģֻ Fsm2Input ͼ Fsm2OutputIMU ջ
  *  ȫⲿִвɡ
  *
  *  붯ģ
  *  --------------
  *  5 ¢ÿ 220 cm¢ࣨ¢ľࣩ70 cm
  *  ʼմֱ¢ת
  *    - ǰ / ֶ 0 ȣشֱ¢ķƶ
  *    - з / зУֶ +90 / -90 ȣ¢ƶ
  *  ÿ¢зг 260 cm = 220 cm ¢ + 40 cm ˲ȵ
  *  ͣоǱ߽缤⵽ 100 mm260 cm ֻס
  *  ¢ǰǶ 0 Ⱥ󵹳 70 cm
  *
  *  ״̬ /  / ʱ
  *  --------------------------------
  *   IDLE       ֹͣȴ Fsm2_Start()
  *   ENTRY      ǰ 0dF <= 206  400 cm -> CRAB_R130 s
  *   CRAB_R1    з +90dR <= 100  260 cm -> REV_1260 s
  *   REV_12      70 cm 0 -> CRAB_L260 s
  *   CRAB_L2    з -90dL <= 100  260 cm -> REV_2360 s
  *   REV_23      70 cm -> CRAB_R360 s
  *   CRAB_R3    з +90dR <= 100  260 cm -> REV_3460 s
  *   REV_34      70 cm -> CRAB_L460 s
  *   CRAB_L4    з -90dL <= 100  260 cm -> REV_4560 s
  *   REV_45      70 cm -> CRAB_R560 s
  *   CRAB_R5    з +90dR <= 100  260 cm -> EXIT_L60 s
  *   EXIT_L     з -90dL <= 100  260 cm -> REV_PARK60 s
  *   REV_PARK    100 cm -> DONE60 s
  *   DONE       ֹͣdone = 1
  *   FAULT      ֹͣ棻Fsm2_Start()  Fsm2_Init() 
  *
  *  змලR1 / L2 / R3 / L4 / R5 / EXIT_L ͬ
  *  ----------------------------------------------------
  *   - ں: e = ƽ(dF - 206, dB - 137)ֻȡЧ·
  *      dF Ч dF dB Ч dB·Ч -> ͣ
  *     ͣȽЧ FAULT IMU / Ҽ
  *   - Ӳȫ: stall = 1һ·Ч < 50 mm ->  FAULT
  *   - оڡ·Чоݣ⵽˲䶪󱨡
  *   - ߽缤Чʱֻʣ 260 cm ףⱨϡ
  *
  *  ִвԼ
  *  ----------------
  *   - request_stop = 1ִвͣ/֣Ҫִб tick  action
  *   - ÿνз / ״̬ȱ F2_SETTLE_MSĬ 600 ms
  *       action = STOPrequest_stop = 1 steer_deg е½Ƕȣ
  *       öתλͣȺٿʼ¶ENTRY ȴǰ
  *   - action = F2A_CRAB_R / F2A_CRAB_L ʱ steer_deg = +90 / -90
  *     СǶ΢ⲿ centering / safety ģ鰴 dF/dB ںɣ
  *     ģֻͣоݡ
  *   - action = F2A_REV ʱ target_cm ǡ X cmͼ
  *     IMU ֺⲿִС
  *
  *  ÷ 20 ms ڣ
  *  ---------------------------
  *      Fsm2_Init();
  *      Fsm2_Start();
  *      for (;;) {
  *        Fsm2Input  in;
  *        Fsm2Output out;
  *        ...  in ...
  *        Fsm2_Tick(&in, &out);
  *        ...  out  ...
  *      }
  *
  *  : GBK/ANSI빤 system.h һ£벿ִ ASCII
  *         UTF-8 ༭򿪳룬밴 GBK/ANSI ´򿪡
  */
#ifndef __FSM2_H
#define __FSM2_H

#include "app_if.h"          /* u8 / u16 / s16 / s32 ̻ */

/* ======================================================================= */
/*  ֳɵλ: mm / cm / ms                                     */
/*  ע: Ŀ 6 ԭ F2_ ǰ׺빤       */
/*      safety.h ĺֻ±룬ض߼          */
/* ======================================================================= */

/* ENTRY: ǰ⵽ 1 ¢ֵͣ, mm */
#define ENTRY_STOP_MM            320u

/* ENTRY : dF һֱЧʱǰô cm Ҳ CRAB_R1 */
#define F2_ENTRY_FALLBACK_CM     400u

/* з: ߽缤⣨зп dRзп dLֵͣ, mm */
#define CRAB_STOP_MM             100u

/* з: г, cm260 = 220 ¢ + 40 ȵ */
#define CRAB_FALLBACK_CM         260u

/* ¢, cmֶ 0 ȣ */
#define REV_RIDGE_CM             70u

/* ⵹, cm */
#define REV_PARK_CM              100u

/* ʱ, msǰ 30 sз /  60 s */
#define F2_ENTRY_TIMEOUT_MS      30000u
#define CRAB_TIMEOUT_MS          60000u
#define F2_REV_TIMEOUT_MS        60000u

/* Ӳȫ: һ·ЧСڸֵ ->  FAULT, mm */
#define F2_LASER_MIN_MM          30u

/* ںϱֵ: dF ¢ 206 mmdB = ʵ 100 mm + 37 mm ƫ = 137 mm */
#define F2_DF_NOMINAL_MM         206u
#define F2_DB_NOMINAL_MM         137u
#define F2_TOF_BIAS_MM           37u

/* з / ״̬ͣ / λʱ, ms 0 ɹرոñ֡
   ENTRY Ӱ죬Start ǰ */
#define F2_SETTLE_MS             600u

/* һ˲: ÿ tick ֵȨ 1/(1<<F2_FILTER_SHIFT)Ĭ 1/4
   ˲ֵֻں eͣоԭʼֵ˲ͺ߽硣 */
#define F2_FILTER_SHIFT          2u

/*  tick 䳬ֵΪⲿ / 쳣ȡ׼, cm */
#define F2_DIST_JUMP_MAX_CM      30u

/* Ƕ, ȡ */
#define F2_STEER_STRAIGHT_DEG    0
#define F2_STEER_CRAB_DEG        90

/* ں:  safety.h  CRAB_ERR_SIGN һ£ֳ෴ɸ -1
   e > 0 ʾʵڱֵò¢ƫԶϡ */
#define F2_CRAB_ERR_SIGN         (+1)

/* ======================================================================= */
/*  Fsm2_GetFaultCode() ֵ                                             */
/* ======================================================================= */
#define F2_FAULT_NONE            0u   /* ޹                            */
#define F2_FAULT_STALL           1u   /* ת                          */
#define F2_FAULT_NEAR_F          2u   /* dF Ч < 50 mm                 */
#define F2_FAULT_NEAR_B          3u   /* dB Ч < 50 mm                 */
#define F2_FAULT_NEAR_L          4u   /* dL Ч < 50 mm                 */
#define F2_FAULT_NEAR_R          5u   /* dR Ч < 50 mm                 */
#define F2_FAULT_NO_LATERAL      6u   /* зʱ dF / dB ·Ч         */
#define F2_FAULT_TIMEOUT         7u   /* ǰ״̬ʱ                      */
#define F2_FAULT_ABORT           8u   /* ⲿ Fsm2_Abort()             */

/* ======================================================================= */
/*  ״̬ݽӿ                                                         */
/* ======================================================================= */
typedef enum
{
  F2_IDLE = 0, F2_ENTRY,          /* ǰ 1 ¢                   */
  F2_CRAB_R1, F2_REV_12,
  F2_CRAB_L2, F2_REV_23,
  F2_CRAB_R3, F2_REV_34,
  F2_CRAB_L4, F2_REV_45,
  F2_CRAB_R5, F2_EXIT_L, F2_REV_PARK,
  F2_DONE, F2_FAULT
} Fsm2State;

typedef struct
{
  u16 dF, dB, dL, dR;      /* mm, 0xFFFF = invalid                    */
  u8  vF, vB, vL, vR;      /* 1 = validʱ / ʱɵ 0*/
  s32 dist_cm;             /* , cmۼֵʼʱ㣬
                              ģ״̬ / ͣȽʱȡ׼ dist0
                               |dist_cm - dist0| Ƴ̣ҲƳ      */
  float yaw_deg;           /* IMU , degģ鲻ʹãϲ㣩*/
  u8  stall;               /* 1 = תⲿ safety ģ     */
  u32 ms;                  /* 1 ms ɼʱ              */
} Fsm2Input;

typedef enum { F2A_STOP=0, F2A_FWD, F2A_REV, F2A_CRAB_R, F2A_CRAB_L } Fsm2Action;

typedef struct
{
  Fsm2Action action;
  u16 target_cm;           /* ζĿ루 70/100зж 260*/
  u16 stop_mm;             /* ֵͣз 100ǰ 206        */
  s16 steer_deg;           /* Ƕȣ0+90-90                     */
  u8  request_stop;        /* 1 = Ҫͣ action ִУ    */
  u8  done;                /* 1 =                           */
} Fsm2Output;

/* ======================================================================= */
/*  ӿ                                                               */
/* ======================================================================= */

/* ϵʼ: ״̬˲̻׼ F2_IDLE */
void Fsm2_Init(void);

/* : ֻ IDLE / DONE / FAULT ãеñԡ
    FAULT 벢¿ʼ */
void Fsm2_Start(void);

/* ֹ:  F2_FAULT 棨йͣ
   Ҫ Fsm2_Start()  Fsm2_Init() ˳ FAULT */
void Fsm2_Abort(void);

/* ڵã 20 msin / out Ϊָʱȫأ
   out Ϊֱӷأin Ϊʱֻͣı䵱ǰ״̬ */
void Fsm2_Tick(const Fsm2Input *in, Fsm2Output *out);

/* ǰ״̬ */
Fsm2State Fsm2_GetState(void);

/* ״̬ASCII ַ OLED / ڴӡ */
const char *Fsm2_StateName(Fsm2State s);

/* ---- ΪֳõֻϽӿڣӰ״̬Ǩƣ ---- */

/* һι룬 F2_FAULT_xxx0 = ޹ϡ */
u8 Fsm2_GetFaultCode(void);

/* ǰѾߵľ, cmͣȱڼΪ 0 */
u16 Fsm2_GetPhaseCm(void);

/* ǰ / ںϺ, mmlat_valid = 0 ʱΪ 0 */
s16 Fsm2_GetLateralMm(void);

/* 1 = dF / dB һ·Чںֵã0 = ·Ч */
u8 Fsm2_GetLateralValid(void);

#endif /* __FSM2_H */
