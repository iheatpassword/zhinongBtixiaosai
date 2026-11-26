#ifndef MOTION_TRACE_H
#define MOTION_TRACE_H

#include "app_tasks.h"

void MotionTrace_Init(void);
void MotionTrace_Capture(const AppInputs *in, const AppOutputs *out);
void MotionTrace_Report(void);
#endif
