#ifndef TUNE_PARAMS_H
#define TUNE_PARAMS_H

#include <stdint.h>

typedef struct {
#define TUNE_I(name, field, initial, lo, hi) int field;
#define TUNE_F(name, field, initial, lo, hi) float field;
#include "tune_params.def"
#undef TUNE_I
#undef TUNE_F
} TuneValues;

/* The command task publishes edits between control ticks. */
extern const volatile TuneValues *const app_tune;
void Tune_Init(void);
uint32_t Tune_Revision(void);
void Tune_Command(char *line);
void Tune_PollList(void);
#endif
