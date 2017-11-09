#ifndef __CIR_QUEUE_H__

#define __CIR_QUEUE_H__

#define QUE_SIZE 1024

#include "gps_hub.h"

typedef struct cir_queue_t

{

  JT808Type data[QUE_SIZE];

  int front;

  int rear;

  int count;
    
  int running;

}cir_queue_t;


void init_cir_queue(cir_queue_t* q);

int is_empty_cir_queue(cir_queue_t* q);

int is_full_cir_queue(cir_queue_t* q);

void push_cir_queue(cir_queue_t* q, JT808Type x);

JT808Type pop_cir_queue(cir_queue_t* q);

void destroy_cir_queue(cir_queue_t* q);

int isRunning(cir_queue_t *q);

void setStop(cir_queue_t *q);

void clear_cir_queue(cir_queue_t *q);

#endif
