#include "scheduler.h"
#include "systick.h"
#include <stdint.h>

static Task *tasks;
static uint32_t task_count = 0;

void init_scheduler(Task task_array[], uint32_t count) {
    tasks = task_array;
    task_count = count;
}

void run_scheduler() {
    uint32_t now = millis();
    for (uint32_t i = 0; i < task_count; i++) {
        if (tasks[i].interval == 0 ||
            (now - tasks[i].last_run >= tasks[i].interval)) {
            tasks[i].func();
            tasks[i].last_run = now;
        }
    }
}
