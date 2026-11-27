#ifndef ROUTE_RANGES_H
#define ROUTE_RANGES_H

#include "app_tasks.h"

typedef struct {
  u16 mm;
  u8 age, seq, from_tail;
} RouteRange;

/* side is LW_LEFT or LW_RIGHT. Tail substitution is meaningful only where
   both side beams are expected to observe the same outside wall. */
RouteRange RouteRange_Side(const AppInputs *in, unsigned side, int allow_tail);

typedef enum { RIDGE_MISSING, RIDGE_VALID, RIDGE_DISAGREE, RIDGE_PAIR } RidgeStatus;
RidgeStatus RouteRange_Ridge(const AppInputs *in, float *error_mm);

#endif
