/**
  * @file    safety.h
  * @brief   激光有效性 / 中值滤波 / 前-后融合 / 最小距离停车 / 堵转检测
  *          (纯逻辑模块: 不访问任何硬件, 不依赖 RTOS, 无 printf, 无动态内存)
  *
  *  数据来源 (由调用者填进 SafetyRaw, 本模块只做判定):
  *    dF : 前激光 -> 垄目标距离,        单位 mm
  *    dB : 后侧 TOF (所跨垄的侧壁),     单位 mm, 真实间隙 = 读数 - 37
  *    dL : 左激光,                      单位 mm
  *    dR : 右激光,                      单位 mm
  *    vX : 该路本次"有帧且有回波"标志,   0 = 本次无数据
  *         (本项目可由 LaserData.flags 的对应位 / ridge_valid 填入)
  *
  *  无效数据共有三种来源, 一律按"无效"处理:
  *    1) 0xFFFF : 链路/驱动侧的"无数据" (驱动层把 s16 的 -1 存成 u16 也是 0xFFFF)
  *    2) 2000   : VB02A 超量程 / 无回波 (长距 >1.2 m 时偶发)
  *    3) vX == 0: 本次没有收到有效帧
  *  无效帧不参与中值、不参与融合, 也不会被"补"成某个合法值。
  *
  *  符号约定 (crab_err_mm, 蟹行横向控制用):
  *    crab_err_mm = CRAB_ERR_SIGN * (融合后的 "实测 - 目标"),  单位 mm
  *      > 0 : 实测距离大于目标 => 车体离垄的基准面偏远,
  *            横向(蟹行)应朝 "让 dF/dB 读数变小" 的一侧平移修正;
  *      < 0 : 实测距离小于目标 => 车体压得过近,
  *            横向应朝 "让 dF/dB 读数变大" 的一侧平移修正;
  *      = 0 且 crab_ok == 1 : 已在目标位置附近(理想);
  *    crab_ok == 0 : 前/后两路都无效 => 融合值"不可用",
  *            此时 crab_err_mm 恒为 0。调用者必须停车/保持, 严禁把 0
  *            当作"误差为零"继续跑横向控制 (本模块不做任何兜底)。
  *
  *  CRAB_ERR_SIGN 默认 +1。现场若发现横向修正方向相反, 直接改成 -1,
  *  不必改算法。标定方法: 把车摆到标准位置, 人工沿横向推 20 mm 到某一侧,
  *  观察 crab_err_mm 符号是否与"应该往哪边修正"一致。
  *
  *  TARGET_F_MM / TARGET_B_MM 标定方法:
  *    把车停在垄上理想位置(左右居中、前后对正), 连续采集 3~5 s,
  *    取中值滤波后的稳定读数作为目标:
  *      TARGET_F_MM = 前激光稳定读数       (当前 206, 实测 +6 偏置已并入)
  *      TARGET_B_MM = 真实间隙 + 37        (当前真实 100 -> 137)
  *    短距(70~100 mm)左右激光精度 +-1 mm, 因此理想位置上偏差在 +-3 mm
  *    以内即可认为"对正", 不需要继续调。
  *
  *  建议调用顺序 (每个 10~50 ms 控制周期一次):
  *    Safety_Filter(&raw, &filt);
  *    if (Safety_MinDistanceStop(&filt, SAFE_STOP_MM)) App_DriveStop();
  *    Safety_StallUpdate(&stall, pwm, spd, dt_ms);
  *    Safety_CrabFusion(&filt, &crab);
  *
  *  编码: UTF-8 (无 BOM), 注释为中文, 代码本身为纯 ASCII。
  *        Keil 用 GBK 打开时可能显示乱码, 只是显示问题, 不影响编译;
  *        需要时可用记事本/Notepad++ 转存为 GBK。
  */
#ifndef __SAFETY_H
#define __SAFETY_H

#include "app_if.h"          /* u8 / u16 / s16 / s32 等基础类型 */

/* ======================================================================= */
/*  现场可调参数 (改完重新编译即可, 不需要动逻辑)                          */
/* ======================================================================= */
#define SAFE_STOP_MM        50      /* 任一路"有效"距离 < 该值 -> 立即停车, mm   */
#define TARGET_F_MM         206     /* 前激光到垄的目标值, mm (+6 偏置已并入)    */
#define TARGET_B_MM         137     /* 垄侧 TOF 目标值, mm (真实 100 + 37 偏置)  */
#define MEDIAN_N            3       /* 中值滤波窗口帧数, 必须为奇数, 默认 3      */
#define STALL_PWM_MIN       10000   /* 堵转: 四轮最大 PWM 至少要到这个值          */
#define STALL_SPD_MAX       50      /* 堵转: 四轮最大 |速度| 不超过这个值(计数)   */
#define STALL_TIME_MS       300     /* 堵转: 上述条件持续这么久才判堵转, ms       */

/* ======================================================================= */
/*  内部常量 (一般不需要改)                                                */
/* ======================================================================= */
#define SAFE_INVALID_MM     0xFFFFu /* 无效标记: 滤波输出的"无效"值              */
#define SAFE_OVERRANGE_MM   2000u   /* VB02A 超量程/无回波, 视作无效              */
#define SAFE_TOF_BIAS_MM    37      /* 垄侧 TOF 固定偏置, 仅换算用, 不参与判定    */
#define CRAB_ERR_SIGN       (+1)    /* 融合误差符号: 现场方向反了就改成 (-1)      */

/* ======================================================================= */
/*  数据结构                                                               */
/* ======================================================================= */
typedef struct
{
  u16 dF;   /* 前激光原始读数, mm (可能是 0xFFFF / 2000 等无效值) */
  u16 dB;   /* 垄侧 TOF 原始读数, mm                              */
  u16 dL;   /* 左激光原始读数, mm                                  */
  u16 dR;   /* 右激光原始读数, mm                                  */
  u8  vF;   /* 前: 1 = 本次收到帧且有回波, 0 = 无数据              */
  u8  vB;   /* 后: 同上                                            */
  u8  vL;   /* 左: 同上                                            */
  u8  vR;   /* 右: 同上                                            */
} SafetyRaw;

typedef struct
{
  u16 dF;   /* 滤波后距离, mm; 无效时为 0xFFFF                     */
  u16 dB;
  u16 dL;
  u16 dR;
  u8  vF;   /* 滤波后有效标志: 1 = 可用, 0 = 无效                  */
  u8  vB;
  u8  vL;
  u8  vR;
} SafetyFilt;

typedef struct
{
  u8    crab_ok;       /* 1 = 前/后至少一路有效; 0 = 两路都无效(不可用)    */
  float crab_err_mm;   /* 融合横向误差, mm; 符号见文件头 "符号约定"         */
  float dF_err;        /* dF - TARGET_F_MM, mm; dF 无效时置 0               */
  float dB_err;        /* dB - TARGET_B_MM, mm; dB 无效时置 0               */
} SafetyCrab;

typedef struct
{
  u8  stalled;         /* 1 = 堵转(已连续满足条件 >= STALL_TIME_MS)         */
  u32 stall_ms;        /* 已持续满足堵转条件的累计时间, ms (饱和到阈值)     */
} SafetyStall;

/* ======================================================================= */
/*  接口                                                                   */
/* ======================================================================= */

/* 清空四路滤波状态; 上电初始化时调用一次(自检内部也会用到)。 */
void Safety_Init(void);

/* 3 帧中值 + 有效性判定 + 2000/0xFFFF 剔除。
   每调用一次代表"来了 1 帧"; 建议 10~50 ms 调用一次。
   每路维护最近 MEDIAN_N 帧窗口:
     - 无效帧只打"无效"标记, 不污染数值;
     - 中值只在窗口内的有效帧里取;
     - 窗口内一帧有效都没有 (= 连续 3 帧全无效) -> 该路无效, 输出 0xFFFF;
     - 恢复: 之后来 1 帧有效立即恢复有效(低延迟, 不因丢帧长时间"瞎")。
   in 为空指针时输出全部无效(不推进窗口); out 为空指针时直接返回。 */
void Safety_Filter(const SafetyRaw *in, SafetyFilt *out);

/* 前(dF)+后(dB)融合:
     两路都有效 -> 平均;  只有一路有效 -> 用那一路;
     都无效     -> crab_ok = 0, crab_err_mm = 0 (不兜底、不用左右激光替代)。
   任一路无效时, 对应的 dF_err / dB_err 置 0。
   输出 crab_err_mm 已乘 CRAB_ERR_SIGN。 */
void Safety_CrabFusion(const SafetyFilt *f, SafetyCrab *out);

/* 最小距离停车: 只统计"有效"通道, 任一有效距离 < limit_mm -> 返回 1。
   无效通道(0xFFFF / 标志为 0)不参与判断; limit_mm 传 0 时恒返回 0。
   常规用法: if (Safety_MinDistanceStop(&filt, SAFE_STOP_MM)) App_DriveStop(); */
u8 Safety_MinDistanceStop(const SafetyFilt *f, u16 limit_mm);

/* 堵转检测 (四轮一起判):
     四轮最大 PWM >= STALL_PWM_MIN 且 四轮最大 |spd| <= STALL_SPD_MAX
       -> 累计 stall_ms += dt_ms, 累计到 >= STALL_TIME_MS 时 stalled = 1;
     否则 -> stalled = 0, stall_ms = 0 (条件一破坏就清零)。
   pwm 为 PWM 占空比计数(u16), spd 为编码器速度计数(s16), 顺序 FL,FR,RL,RR。
   建议每个控制周期调用一次(20 ms), dt_ms 传真实间隔。 */
void Safety_StallUpdate(SafetyStall *st, const u16 pwm[4], const s16 spd[4], u32 dt_ms);

/* 内置自检: 结果按位或进 *fail_mask, 0 = 全部通过。
   自检会临时改写内部滤波状态, 结束前自动恢复, 因此可在运行中调用,
   但建议上电后、控制循环启动前调用一次。
   fail_mask 为空指针时仍执行自检(只是不写结果)。 */
void Safety_SelfTest(u8 *fail_mask);

/* ---- Safety_SelfTest 位定义 (哪一位为 1 就说明该组用例失败) ---- */
#define SAFE_ST_ALL_INVALID   0x01u  /* 三帧全无效 -> 输出 0xFFFF 且 v = 0        */
#define SAFE_ST_DROP_SENTINEL 0x02u  /* 0xFFFF / 2000 被剔除, 中值只取有效帧       */
#define SAFE_ST_RECOVER       0x04u  /* 连续无效后 1 帧有效即恢复                  */
#define SAFE_ST_MEDIAN_SPIKE  0x08u  /* 单帧尖峰(80/600/82)被中值压掉 -> 82        */
#define SAFE_ST_FUSION        0x10u  /* 双路平均 / 单路直通 / 双路无效 crab_ok = 0  */
#define SAFE_ST_MIN_STOP      0x20u  /* < SAFE_STOP_MM 停车, == 阈值不停, 无效不算  */
#define SAFE_ST_STALL_TIMER   0x40u  /* 持续 >= STALL_TIME_MS 才 stalled = 1       */
#define SAFE_ST_STALL_RESET   0x80u  /* 条件破坏 -> stalled = 0 且 stall_ms 清零   */

#endif /* __SAFETY_H */
