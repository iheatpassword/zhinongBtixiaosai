#ifndef TUNE_STORE_H
#define TUNE_STORE_H

#include "tune_params.h"

/* Non-volatile copy of the tuning table.  Load returns 1 only for a record
   that passes the magic, layout, size and CRC checks, so an interrupted save
   always degrades to "no record" instead of a half-valid one. */
int Tune_Store_Load(TuneValues *out);

/* 0 on success, negative on the step that failed:
     -1 no data   -2 unlock  -4 program  -5 readback  -6 mismatch
     -7 sector full (512 slots used; needs an erase)                        */
int Tune_Store_Save(const TuneValues *in, uint32_t revision);

#endif
