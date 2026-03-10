#ifndef SCHEDULER_H
#define SCHEDULER_H

#include <stdint.h>

typedef struct {
    void (*func)(void);
    uint32_t interval;
    uint32_t last_run;
} Task;

void init_scheduler(Task tasks[], uint32_t count);
void run_scheduler(void);

#endif
