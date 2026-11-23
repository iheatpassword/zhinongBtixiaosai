/**
  * @file    fsm2.h
  * @brief   跨垄作业流程状态机（纯逻辑模块，不访问任何硬件）
  *
  *  目标平台: STM32F407 + Keil MDK / ARMCC5，C90，无动态内存，无 printf。
  *  本模块只根据 Fsm2Input 产生动作意图 Fsm2Output；电机、舵机、IMU 闭环
  *  全部由外部执行层完成。
  *
  *  场地与动作模型
  *  --------------
  *  5 条垄，每条长 220 cm，垄距（相邻垄中心距）70 cm。
  *  车身长边始终垂直于垄，不旋转车身：
  *    - 前进 / 倒车：四轮舵机 0 度，沿垂直于垄的方向移动；
  *    - 右蟹行 / 左蟹行：四轮舵机 +90 / -90 度，沿垄长方向移动。
  *  每条垄的蟹行理论行程 260 cm = 220 cm 垄长 + 40 cm 端部廊道；
  *  主停车判据是边界激光到 100 mm，260 cm 只作编码器兜底。
  *  换垄不用前进，而是舵机回 0 度后倒车 70 cm。
  *
  *  状态表（动作 / 完成条件 / 超时）
  *  --------------------------------
  *   IDLE       停止，等待 Fsm2_Start()
  *   ENTRY      前进，舵机 0；dF <= 206 或 400 cm -> CRAB_R1；30 s
  *   CRAB_R1    右蟹行 +90；dR <= 100 或 260 cm -> REV_12；60 s
  *   REV_12     倒车 70 cm，舵机 0 -> CRAB_L2；60 s
  *   CRAB_L2    左蟹行 -90；dL <= 100 或 260 cm -> REV_23；60 s
  *   REV_23     倒车 70 cm -> CRAB_R3；60 s
  *   CRAB_R3    右蟹行 +90；dR <= 100 或 260 cm -> REV_34；60 s
  *   REV_34     倒车 70 cm -> CRAB_L4；60 s
  *   CRAB_L4    左蟹行 -90；dL <= 100 或 260 cm -> REV_45；60 s
  *   REV_45     倒车 70 cm -> CRAB_R5；60 s
  *   CRAB_R5    右蟹行 +90；dR <= 100 或 260 cm -> EXIT_L；60 s
  *   EXIT_L     左蟹行 -90；dL <= 100 或 260 cm -> REV_PARK；60 s
  *   REV_PARK   倒车 100 cm -> DONE；60 s
  *   DONE       停止，done = 1
  *   FAULT      停止并锁存；Fsm2_Start() 或 Fsm2_Init() 才能清除
  *
  *  蟹行监督（R1 / L2 / R3 / L4 / R5 / EXIT_L 规则相同）
  *  ----------------------------------------------------
  *   - 横向融合: e = 平均(dF - 206, dB - 137)，只取有效路；
  *     仅 dF 有效用 dF，仅 dB 有效用 dB；两路都无效 -> 停车，
  *     停稳结束后仍无效则 FAULT（不拿 IMU / 左右激光替代）。
  *   - 硬安全: stall = 1，或任一路“有效”激光 < 50 mm -> 立即 FAULT。
  *   - 正常完成判据优先于“两路无效”判据，避免到点瞬间丢数误报。
  *   - 边界激光无效时只剩 260 cm 编码器兜底，不额外报故障。
  *
  *  输出与执行层约定
  *  ----------------
  *   - request_stop = 1：执行层必须先停车/保持，不要执行本 tick 的 action。
  *   - 每次进入蟹行 / 倒车状态先保持 F2_SETTLE_MS（默认 600 ms）：
  *       action = STOP、request_stop = 1，但 steer_deg 已切到新角度，
  *       让舵机转到位、车身停稳后再开始新动作。ENTRY 不等待，启动即前进。
  *   - action = F2A_CRAB_R / F2A_CRAB_L 时 steer_deg = +90 / -90；
  *     横向小角度微调由外部 centering / safety 模块按 dF/dB 融合完成，
  *     本模块只给大方向和停车判据。
  *   - action = F2A_REV 时 target_cm 就是“倒车到 X cm”的意图，
  *     IMU 保持航向由外部执行。
  *
  *  调用方法（建议 20 ms 周期）
  *  ---------------------------
  *      Fsm2_Init();
  *      Fsm2_Start();
  *      for (;;) {
  *        Fsm2Input  in;
  *        Fsm2Output out;
  *        ... 填 in ...
  *        Fsm2_Tick(&in, &out);
  *        ... 按 out 驱动 ...
  *      }
  *
  *  编码: GBK/ANSI（与工程内 system.h 一致），代码部分纯 ASCII。
  *        若用 UTF-8 编辑器打开出现乱码，请按 GBK/ANSI 重新打开。
  */
#ifndef __FSM2_H
#define __FSM2_H

#include "app_if.h"          /* u8 / u16 / s16 / s32 及工程基础类型 */

/* ======================================================================= */
/*  现场可调参数（单位: mm / cm / ms）                                     */
/*  注: 题目点名的 6 个参数保持原名；其余加 F2_ 前缀，避免与工程内       */
/*      fsm.h / safety.h 的宏重名。改完只需重新编译，不必动逻辑。          */
/* ======================================================================= */

/* ENTRY: 前激光到第 1 条垄侧面的停车阈值, mm。 */
#define ENTRY_STOP_MM            206u

/* ENTRY 兜底: dF 一直无效时，前进这么多 cm 后也进入 CRAB_R1。 */
#define F2_ENTRY_FALLBACK_CM     400u

/* 蟹行: 边界激光（右蟹行看 dR、左蟹行看 dL）的停车阈值, mm。 */
#define CRAB_STOP_MM             100u

/* 蟹行: 编码器兜底行程, cm。260 = 220 垄 + 40 廊道。 */
#define CRAB_FALLBACK_CM         260u

/* 换垄倒车距离, cm（四轮舵机回 0 度）。 */
#define REV_RIDGE_CM             70u

/* 最后入库倒车距离, cm。 */
#define REV_PARK_CM              100u

/* 超时, ms：前进 30 s，蟹行 / 倒车 60 s。 */
#define F2_ENTRY_TIMEOUT_MS      30000u
#define CRAB_TIMEOUT_MS          60000u
#define F2_REV_TIMEOUT_MS        60000u

/* 硬安全: 任一路有效激光小于该值 -> 立即 FAULT, mm。 */
#define F2_LASER_MIN_MM          50u

/* 融合标称值: dF 对垄侧 206 mm；dB = 真实 100 mm + 37 mm 偏置 = 137 mm。 */
#define F2_DF_NOMINAL_MM         206u
#define F2_DB_NOMINAL_MM         137u
#define F2_TOF_BIAS_MM           37u

/* 进入蟹行 / 倒车状态后的停稳 / 舵机到位时间, ms。设 0 可关闭该保持。
   ENTRY 不受影响，Start 后立即输出前进。 */
#define F2_SETTLE_MS             600u

/* 一阶滤波: 每 tick 新值权重 1/(1<<F2_FILTER_SHIFT)，默认 1/4。
   滤波值只用于诊断融合误差 e；停车判据仍用原始值，避免滤波滞后冲过边界。 */
#define F2_FILTER_SHIFT          2u

/* 编码器单 tick 跳变超过该值视为外部清零 / 异常，重新取基准, cm。 */
#define F2_DIST_JUMP_MAX_CM      30u

/* 舵机角度, 度。 */
#define F2_STEER_STRAIGHT_DEG    0
#define F2_STEER_CRAB_DEG        90

/* 融合误差符号: 与 safety.h 的 CRAB_ERR_SIGN 含义一致，现场方向相反可改 -1。
   e > 0 表示实测距离大于标称值（车体离该侧垄壁偏远），仅作诊断。 */
#define F2_CRAB_ERR_SIGN         (+1)

/* ======================================================================= */
/*  Fsm2_GetFaultCode() 返回值                                             */
/* ======================================================================= */
#define F2_FAULT_NONE            0u   /* 无故障                            */
#define F2_FAULT_STALL           1u   /* 电机堵转                          */
#define F2_FAULT_NEAR_F          2u   /* dF 有效且 < 50 mm                 */
#define F2_FAULT_NEAR_B          3u   /* dB 有效且 < 50 mm                 */
#define F2_FAULT_NEAR_L          4u   /* dL 有效且 < 50 mm                 */
#define F2_FAULT_NEAR_R          5u   /* dR 有效且 < 50 mm                 */
#define F2_FAULT_NO_LATERAL      6u   /* 蟹行时 dF / dB 两路都无效         */
#define F2_FAULT_TIMEOUT         7u   /* 当前状态超时                      */
#define F2_FAULT_ABORT           8u   /* 外部调用 Fsm2_Abort()             */

/* ======================================================================= */
/*  状态与数据接口                                                         */
/* ======================================================================= */
typedef enum
{
  F2_IDLE = 0, F2_ENTRY,          /* 前进到第 1 条垄                   */
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
  u8  vF, vB, vL, vR;      /* 1 = valid（超时 / 超量程时由调用者清 0）*/
  s32 dist_cm;             /* 编码器里程, cm：累计值或动作开始时清零，
                              本模块在状态进入 / 停稳结束时取基准 dist0，
                              按 |dist_cm - dist0| 计程，反向也计程      */
  float yaw_deg;           /* IMU 航向, deg（本模块不使用，保留给上层）*/
  u8  stall;               /* 1 = 电机堵转（外部 safety 模块给出）     */
  u32 ms;                  /* 1 ms 自由计数时基，允许回绕              */
} Fsm2Input;

typedef enum { F2A_STOP=0, F2A_FWD, F2A_REV, F2A_CRAB_R, F2A_CRAB_L } Fsm2Action;

typedef struct
{
  Fsm2Action action;
  u16 target_cm;           /* 本次动作目标距离（倒车 70/100、蟹行兜底 260）*/
  u16 stop_mm;             /* 激光停车阈值（蟹行 100、前进 206）        */
  s16 steer_deg;           /* 舵机角度：0、+90、-90                     */
  u8  request_stop;        /* 1 = 需要立即停车（优先于 action 执行）    */
  u8  done;                /* 1 = 整个流程完成                          */
} Fsm2Output;

/* ======================================================================= */
/*  公开接口                                                               */
/* ======================================================================= */

/* 上电初始化: 清空状态、滤波、里程基准，进入 F2_IDLE。 */
void Fsm2_Init(void);

/* 启动流程: 只允许从 IDLE / DONE / FAULT 调用；运行中调用被忽略。
   从 FAULT 启动会清除故障码并重新开始。 */
void Fsm2_Start(void);

/* 立即中止: 进入 F2_FAULT 并锁存（已有故障码则保留），输出停车。
   需要 Fsm2_Start() 或 Fsm2_Init() 才能退出 FAULT。 */
void Fsm2_Abort(void);

/* 周期调用（建议 20 ms）。in / out 为空指针时安全返回：
   out 为空直接返回；in 为空时只输出停车，不改变当前状态。 */
void Fsm2_Tick(const Fsm2Input *in, Fsm2Output *out);

/* 当前状态。 */
Fsm2State Fsm2_GetState(void);

/* 状态名（ASCII 字符串，便于 OLED / 串口打印）。 */
const char *Fsm2_StateName(Fsm2State s);

/* ---- 以下为现场调试用的只读诊断接口（不影响状态迁移） ---- */

/* 最近一次故障码，见 F2_FAULT_xxx；0 = 无故障。 */
u8 Fsm2_GetFaultCode(void);

/* 当前动作已经走的距离, cm（停稳保持期间为 0）。 */
u16 Fsm2_GetPhaseCm(void);

/* 前 / 后融合横向误差, mm；lat_valid = 0 时恒为 0。 */
s16 Fsm2_GetLateralMm(void);

/* 1 = dF / dB 至少一路有效，融合值可用；0 = 两路都无效。 */
u8 Fsm2_GetLateralValid(void);

#endif /* __FSM2_H */
