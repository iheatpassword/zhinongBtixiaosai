#include "motion_trace.h"
#include "app_if.h"
#include "app_config.h"
#include "app_report.h"
#include "fsm2.h"
#include "servo/servo.h"
#include "FreeRTOS.h"
#include "queue.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Capture transitions at control rate; format and transmit in the log task. */
typedef struct {
  u32 ms, path_mm, dropped, param_revision;
  u16 tail[3];
  u8 use_tail, tail_flags;
  u16 range[4], speed_mm_s, target, peak_pwm;
  s16 steer10, error10;
  u8 age[4], mode, stage, flags, fault, wait_reason, quiet, ready;
} MotionSample;

#define QUEUE_DEPTH 24u
static StaticQueue_t queue_state;
static uint8_t storage[QUEUE_DEPTH * sizeof(MotionSample)];
static QueueHandle_t queue;
static u32 drops;

void MotionTrace_Init(void)
{
  queue=xQueueCreateStatic(QUEUE_DEPTH,sizeof(MotionSample),storage,&queue_state);
}

void MotionTrace_Capture(const AppInputs *in, const AppOutputs *out)
{
  static MotionSample prev;
  static u32 last_active, last_sample, prev_tick, steer_ms;

  static s16 prev_steer[4];
  static unsigned quiet_ticks;
  static uint64_t total_counts;
  static int recording;
  static u32 braking_since;
  static int braking;

  u32 dt=in->ms-prev_tick;
  prev_tick=in->ms;
  int active=in->heading.mode!=APP_MODE_IDLE && in->heading.mode!=APP_MODE_WHEELTEST;
  if (active) last_active=in->ms;
  if (!active && (!recording || in->ms-last_active>1000u)) {
    recording=0;
    return;
  }

  MotionSample s={0};
  s.ms=in->ms;
  s.param_revision=in->heading.param_revision;
  s.mode=in->heading.mode;
  s.stage=in->heading.stage;
  s.flags=s.mode==APP_MODE_FSM2 ? Fsm2_TraceFlags() : 0;
  s.fault=Fsm2_GetFaultCode();
  s.wait_reason=(s.mode==APP_MODE_FSM2 || (s.mode==APP_MODE_IDLE && s.fault==F2_MISSING_RANGE)) ? Fsm2_TraceWait() : 0;
  s.use_tail=in->use_tail;
  s.tail_flags=in->tail_flags;
  s.tail[0]=in->tail_left; s.tail[1]=in->tail_right; s.tail[2]=in->tail_back;
  if (s.use_tail && (!in->tail_fresh || (s.tail_flags & 0x16u)!=0x16u)) s.wait_reason=128u;
  s.range[0]=in->dL; s.range[1]=in->dR;
  s.range[2]=in->dF; s.range[3]=in->dB;
  memcpy(s.age,in->range_age,sizeof s.age);
  if (!recording || memcmp(prev_steer,out->steer_deg10,sizeof prev_steer)) {
    steer_ms=in->ms;
    memcpy(prev_steer,out->steer_deg10,sizeof prev_steer);
  }
  if (!recording) {
    total_counts=0;
    quiet_ticks=0;
  }

  unsigned counts=0, target=0, peak_count=0;
  s.ready=1;
  s.steer10=out->steer_deg10[0];
  for (unsigned i=0;i<4;i++) {
    unsigned mag=abs(in->wheel_cnt[i]);
    counts+=mag;
    if (mag>peak_count) peak_count=mag;
    target+=in->heading.target[i];
    unsigned pwm=abs(out->pwm[i]);
    if (pwm>s.peak_pwm) s.peak_pwm=pwm;
    if (in->steer_feedback10[i]==SERVO_ANGLE_NA ||
        (s32)(Servo_FeedbackTime(i)-steer_ms)<0) {
      s.error10=-1;
      s.ready=0;
    } else {
      int err=abs(in->steer_feedback10[i]-out->steer_deg10[i]);
      if (s.error10>=0 && err>s.error10) s.error10=err;
      if (err>20) s.ready=0;
    }
  }
  if (peak_count>20u) quiet_ticks=0;
  else if (quiet_ticks<3u) quiet_ticks++;
  s.quiet=quiet_ticks==3u;
  s.target=target/4u;
  total_counts+=counts;
  const float mm_per_count=3.14159265f*WHEEL_DIAMETER_MM/(4.0f*ENCODER_CPR);
  s.path_mm=(u32)(total_counts*mm_per_count+0.5f);
  s.speed_mm_s=dt ? (u16)(counts*mm_per_count*1000.0f/dt+0.5f) : 0;
  /* TGT is the mean magnitude. Ignore tiny rounding variations in wheel mix. */
  int changed=!recording || s.param_revision!=prev.param_revision || s.mode!=prev.mode || s.stage!=prev.stage ||
      s.tail_flags!=prev.tail_flags || s.wait_reason!=prev.wait_reason || s.flags!=prev.flags || s.fault!=prev.fault || s.quiet!=prev.quiet ||
      (s.target==0)!=(prev.target==0) || abs((int)s.target-prev.target)>100 ||
      (s.target==0 && (s.ready!=prev.ready || s.steer10!=prev.steer10));
  if (recording && prev.target && !s.target) {
    braking=1;
    braking_since=in->ms;
  }
  if (s.target || s.quiet || !recording || in->ms-braking_since>=600u) braking=0;
  if (changed || in->ms-last_sample >= (braking ? 40u : 200u)) {
    s.dropped=drops;
    if (xQueueSend(queue,&s,0)!=pdPASS) drops++;
    else last_sample=in->ms;
  }
  prev=s;
  recording=1;
}

void MotionTrace_Report(void)
{
  MotionSample s;
  char line[288];
  /* Bound UART work per wakeup; DROP exposes any lost records. */
  for (unsigned n=0;queue && n<3 && xQueueReceive(queue,&s,0)==pdPASS;n++) {
    int len=snprintf(line,sizeof line,
      "MOT MS=%lu M=%u F2=%u FL=%u FA=%u WAIT=%u MM=%lu V=%u T=%u PWM=%u"
      " LRFB=%u/%u/%u/%u AGE=%u/%u/%u/%u Q=%u S10=%d ERR10=%d OK=%u DROP=%lu PV=%lu CFG=%u TF=%u TLRB=%u/%u/%u\r\n",
      (unsigned long)s.ms,s.mode,s.stage,s.flags,s.fault,s.wait_reason,(unsigned long)s.path_mm,
      s.speed_mm_s,s.target,s.peak_pwm,s.range[0],s.range[1],s.range[2],s.range[3],
      s.age[0],s.age[1],s.age[2],s.age[3],s.quiet,s.steer10,s.error10,s.ready,
      (unsigned long)s.dropped,(unsigned long)s.param_revision,
      s.use_tail ? 6u : 3u,s.tail_flags,s.tail[0],s.tail[1],s.tail[2]);
    if (len>0 && (size_t)len<sizeof line) App_Puts(line);
  }
}
