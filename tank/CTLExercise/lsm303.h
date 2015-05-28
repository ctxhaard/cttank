#ifndef __LSM303_H
#define __LSM303_H

#define HEADING_UNAVAIL (0xFFFF)

extern CTL_MESSAGE_QUEUE_t qHeading;

void lsm303_config(void);
void lsm303_task_code(void *p);
#endif // __LSM303_H
