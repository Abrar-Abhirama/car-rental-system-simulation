#ifndef SIMLIB_H
#define SIMLIB_H

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "simlibdefs.h"

#ifdef __cplusplus
extern "C" {
#endif

extern int next_event_type;
extern float sim_time;
extern float transfer[MAX_ATTR + 1];
extern int list_size[MAX_LIST + 1];

void init_simlib(void);
void list_file(int option, int list);
void list_remove(int option, int list);
void timing(void);
void event_schedule(float time_value, int event_type);
void event_cancel(int event_type);

float expon(float mean, int stream);
float uniform(float a, float b, int stream);
float lcgrand(int stream);
void lcgrandst(long zset, int stream);
long lcgrandgt(int stream);

void sampst(float value, int var);
void timest(float value, int var);
void filest(int list);
void out_sampst(FILE *unit, int low_var, int high_var);
void out_timest(FILE *unit, int low_var, int high_var);
void out_filest(FILE *unit, int low_list, int high_list);

#ifdef __cplusplus
}
#endif

#endif
