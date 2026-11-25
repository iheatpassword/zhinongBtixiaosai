/**
  * @file    safety.c
  * @brief   激光有效性 / 中值滤波 / 前-后融合 / 最小距离停车 / 堵转检测
  *
  *  设计要点:
  *    - 每路独立的 MEDIAN_N 帧环形缓冲; 无效帧只打标记, 不会把 2000/0xFFFF
  *      这种"哨兵值"混进数据里, 所以单帧尖峰不会污染滤波输出。
  *    - 中值只在窗口内的有效帧中取; 窗口内一帧有效都没有 (= 连续 3 帧全无效)
  *      才判该路无效, 之后来 1 帧有效立即恢复, 避免丢一帧就长时间没有读数。
  *    - 前/后融合绝不兜底: 两路都无效就是 crab_ok = 0, 不允许用左右激光或
  *      上一次的误差顶替, 让上层能够明确地"报不可用"。
  *    - 堵转用"最大 PWM + 最大 |速度|"判: 只要还有一个轮子在转就不算堵转,
  *      这样正常转弯/差速/单轮打滑时不会误判。
  *
  *  C90 兼容 / 无动态内存 / 无 printf / 无硬件头, 可直接加入 Keil 工程。
  */
#include "safety.h"

/* ======================================================================= */
/*  模块内部状态                                                          */
/* ======================================================================= */
typedef struct
{
  u16 val[MEDIAN_N];   /* 最近 MEDIAN_N 帧原始读数(无效帧也存, 但 ok = 0) */
  u8  ok[MEDIAN_N];    /* 对应帧是否有效: 1 = 参与中值                     */
  u8  idx;             /* 下一个写入位置                                   */
  u8  n;               /* 已存帧数, 饱和到 MEDIAN_N                        */
} SafeChan;

/* 通道顺序固定: 0 = dF, 1 = dB, 2 = dL, 3 = dR */
static SafeChan s_ch[4];

/* ======================================================================= */
/*  内部工具函数                                                          */
/* ======================================================================= */

/* 单帧有效性判定: 标志位为 0, 或数据是 0xFFFF / 2000, 都算无效。
   注意 0xFFFF 同时覆盖了"驱动层把 s16 的 -1 存成 u16"的情况。
   0 mm 这种"贴脸"读数仍然算有效, 让最小距离停车去触发停车, 更安全。 */
static u8 Safe_RawValid(u16 d, u8 flag)
{
  if (flag == 0u)
  {
    return 0u;
  }
  if (d == SAFE_INVALID_MM)
  {
    return 0u;
  }
  if (d == (u16)SAFE_OVERRANGE_MM)
  {
    return 0u;
  }
  return 1u;
}

/* 压入一帧到环形缓冲 */
static void Safe_ChanPush(SafeChan *c, u16 d, u8 ok)
{
  c->val[c->idx] = d;
  c->ok[c->idx]  = ok;
  c->idx = (u8)((c->idx + 1u) % (u8)MEDIAN_N);
  if (c->n < (u8)MEDIAN_N)
  {
    c->n = (u8)(c->n + 1u);
  }
}

/* 在窗口内的有效帧上取中值。
   返回中值(mm); 窗口内没有有效帧时置 *ok_out = 0 并返回 0xFFFF。
   偶数个有效帧时取中间两个的平均(四舍五入), MEDIAN_N 为奇数时不会走到。 */
static u16 Safe_Median(const SafeChan *c, u8 *ok_out)
{
  u16 tmp[MEDIAN_N];
  u16 t;
  u32 sum;
  u8  cnt;
  u8  i;
  u8  j;

  cnt = 0u;
  for (i = 0u; i < c->n; i++)
  {
    if (c->ok[i] != 0u)
    {
      tmp[cnt] = c->val[i];
      cnt = (u8)(cnt + 1u);
    }
  }

  if (cnt == 0u)                 /* 连续 MEDIAN_N 帧全无效 -> 该路无效 */
  {
    *ok_out = 0u;
    return SAFE_INVALID_MM;
  }

  /* 插入排序 (cnt <= 3, 开销可忽略) */
  for (i = 1u; i < cnt; i++)
  {
    t = tmp[i];
    j = i;
    while ((j > 0u) && (tmp[j - 1u] > t))
    {
      tmp[j] = tmp[j - 1u];
      j = (u8)(j - 1u);
    }
    tmp[j] = t;
  }

  *ok_out = 1u;
  if ((cnt & 1u) != 0u)
  {
    return tmp[cnt >> 1];
  }
  sum = (u32)tmp[(cnt >> 1) - 1u] + (u32)tmp[cnt >> 1];
  return (u16)((sum + 1u) >> 1);
}

/* ======================================================================= */
/*  公开接口                                                              */
/* ======================================================================= */

void Safety_Init(void)
{
  u8 c;
  u8 k;

  for (c = 0u; c < 4u; c++)
  {
    s_ch[c].idx = 0u;
    s_ch[c].n   = 0u;
    for (k = 0u; k < (u8)MEDIAN_N; k++)
    {
      s_ch[c].val[k] = SAFE_INVALID_MM;
      s_ch[c].ok[k]  = 0u;
    }
  }
}

void Safety_Filter(const SafetyRaw *in, SafetyFilt *out)
{
  u16 d[4];
  u8  v[4];
  u8  i;

  if (out == 0)
  {
    return;
  }

  /* 输入指针非法时输出"全无效", 且不推进窗口, 防止把一切判成坏值 */
  if (in == 0)
  {
    out->dF = SAFE_INVALID_MM;
    out->dB = SAFE_INVALID_MM;
    out->dL = SAFE_INVALID_MM;
    out->dR = SAFE_INVALID_MM;
    out->vF = 0u;
    out->vB = 0u;
    out->vL = 0u;
    out->vR = 0u;
    return;
  }

  d[0] = in->dF;  v[0] = Safe_RawValid(in->dF, in->vF);
  d[1] = in->dB;  v[1] = Safe_RawValid(in->dB, in->vB);
  d[2] = in->dL;  v[2] = Safe_RawValid(in->dL, in->vL);
  d[3] = in->dR;  v[3] = Safe_RawValid(in->dR, in->vR);

  for (i = 0u; i < 4u; i++)
  {
    Safe_ChanPush(&s_ch[i], d[i], v[i]);
  }

  out->dF = Safe_Median(&s_ch[0], &out->vF);
  out->dB = Safe_Median(&s_ch[1], &out->vB);
  out->dL = Safe_Median(&s_ch[2], &out->vL);
  out->dR = Safe_Median(&s_ch[3], &out->vR);
}

void Safety_CrabFusion(const SafetyFilt *f, SafetyCrab *out)
{
  u8 okF;
  u8 okB;

  if (out == 0)
  {
    return;
  }

  out->crab_ok     = 0u;
  out->crab_err_mm = 0.0f;
  out->dF_err      = 0.0f;
  out->dB_err      = 0.0f;

  if (f == 0)
  {
    return;
  }

  /* 防御性判断: 标志为 1 但数据是 0xFFFF 的畸形输入也按无效处理 */
  okF = ((f->vF != 0u) && (f->dF != SAFE_INVALID_MM)) ? 1u : 0u;
  okB = ((f->vB != 0u) && (f->dB != SAFE_INVALID_MM)) ? 1u : 0u;

  if (okF != 0u)
  {
    out->dF_err = (float)f->dF - (float)TARGET_F_MM;
  }
  if (okB != 0u)
  {
    out->dB_err = (float)TARGET_B_MM - (float)f->dB;
  }

  if ((okF != 0u) && (okB != 0u))
  {
    out->crab_ok     = 1u;
    out->crab_err_mm = 0.5f * (out->dF_err + out->dB_err);
  }
  else if (okF != 0u)
  {
    out->crab_ok     = 1u;
    out->crab_err_mm = out->dF_err;
  }
  else if (okB != 0u)
  {
    out->crab_ok     = 1u;
    out->crab_err_mm = out->dB_err;
  }
  else
  {
    /* 两路都无效: 明确报"不可用", 不兜底 */
    out->crab_ok     = 0u;
    out->crab_err_mm = 0.0f;
    return;
  }

  /* 现场方向标定: CRAB_ERR_SIGN = +1 / -1 (见 safety.h 符号约定) */
  out->crab_err_mm = out->crab_err_mm * (float)CRAB_ERR_SIGN;
}

u8 Safety_MinDistanceStop(const SafetyFilt *f, u16 limit_mm)
{
  if ((f == 0) || (limit_mm == 0u))
  {
    return 0u;
  }

  if ((f->vF != 0u) && (f->dF != SAFE_INVALID_MM) && (f->dF < limit_mm))
  {
    return 1u;
  }
  if ((f->vB != 0u) && (f->dB != SAFE_INVALID_MM) && (f->dB < limit_mm))
  {
    return 1u;
  }
  if ((f->vL != 0u) && (f->dL != SAFE_INVALID_MM) && (f->dL < limit_mm))
  {
    return 1u;
  }
  if ((f->vR != 0u) && (f->dR != SAFE_INVALID_MM) && (f->dR < limit_mm))
  {
    return 1u;
  }

  return 0u;
}

void Safety_StallUpdate(SafetyStall *st, const u16 pwm[4], const s16 spd[4], u32 dt_ms)
{
  u16 pmax;
  s32 smax;
  s32 a;
  u8  i;
  u8  hit;

  if ((st == 0) || (pwm == 0) || (spd == 0))
  {
    return;
  }

  pmax = 0u;
  smax = 0;
  for (i = 0u; i < 4u; i++)
  {
    if (pwm[i] > pmax)
    {
      pmax = pwm[i];
    }
    a = (s32)spd[i];
    if (a < 0)
    {
      a = -a;
    }
    if (a > smax)
    {
      smax = a;
    }
  }

  /* 四轮里 PWM 已经给足了, 但速度还是上不去 -> 疑似堵转 */
  hit = ((pmax >= (u16)STALL_PWM_MIN) && (smax <= (s32)STALL_SPD_MAX)) ? 1u : 0u;

  if (hit == 0u)
  {
    st->stalled  = 0u;
    st->stall_ms = 0u;
    return;
  }

  /* 累计时间, 并做饱和处理, 避免长期堵转把 u32 撑爆 */
  if (dt_ms > 0u)
  {
    if (st->stall_ms > (0xFFFFFFFFu - dt_ms))
    {
      st->stall_ms = 0xFFFFFFFFu;
    }
    else
    {
      st->stall_ms = st->stall_ms + dt_ms;
    }
  }

  if (st->stall_ms >= (u32)STALL_TIME_MS)
  {
    st->stalled = 1u;
    /* 饱和在阈值上: 只表示"已达到判定时间", 不再无限增长;
       上层若需要"堵了多久", 自己用 App_MsNow() 打时间戳即可 */
    st->stall_ms = (u32)STALL_TIME_MS;
  }
  else
  {
    st->stalled = 0u;
  }
}
