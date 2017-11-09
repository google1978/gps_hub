#include <stdio.h>

#include <stdlib.h>

#include <unistd.h>

#include <string.h>

#include <pthread.h>

#include <semaphore.h>

#include "cir_queue.h"

pthread_mutex_t queue_mutex;

pthread_cond_t notempty;

void init_cir_queue(cir_queue_t *q){
    
    int res;
    
    res = pthread_mutex_init(&queue_mutex, NULL);
    if (res != 0)
    {
        perror("Mutex init failed.\n");
        exit(EXIT_FAILURE);
    }
    
    pthread_cond_init(&notempty, NULL);
    
    memset(q->data, 0, QUE_SIZE*sizeof(JT808Type));
    
    q->front = q->rear = 0;
    q->count = 0;
    
    q->running = 1;

}

int isRunning(cir_queue_t *q){
    return q->running;
}

void setStop(cir_queue_t *q){

    q->running = 0;
    pthread_cond_signal(&notempty);

}

int is_empty_cir_queue(cir_queue_t *q){

    int empty_flag;
    
    empty_flag = q->front == q->rear;
    
    return empty_flag;
}

int is_full_cir_queue(cir_queue_t *q){
    
    int full_flag;
    
    full_flag = q->rear == QUE_SIZE - 1 + q->front;
    
    return full_flag;
}

void push_cir_queue(cir_queue_t *q, JT808Type x){
    
    pthread_mutex_lock(&queue_mutex);
    while (is_full_cir_queue(q)) {
        printf("queue overflow.\n");
        pthread_mutex_unlock(&queue_mutex);
        return;
    }
    
    q->count++;
    q->data[q->rear] = x;
    q->rear = (q->rear+1) % QUE_SIZE;
    
    pthread_mutex_unlock(&queue_mutex);
    
    pthread_cond_signal(&notempty);
}

JT808Type pop_cir_queue(cir_queue_t *q){

    JT808Type temp;
    
    pthread_mutex_lock(&queue_mutex);
    
    while ((is_empty_cir_queue(q)) && (q->running)){
        
        pthread_cond_wait(&notempty, &queue_mutex);
    }
    
    temp = q->data[q->front];
    
    memset(&(q->data[q->front]), 0, sizeof(JT808Type));
    
    q->count--;
    q->front = (q->front+1) % QUE_SIZE;
    
    pthread_mutex_unlock(&queue_mutex);
    return temp;
}

void destroy_cir_queue(cir_queue_t *q){

    pthread_mutex_destroy(&queue_mutex);
    pthread_cond_destroy(&notempty);
    return;

}

void clear_cir_queue(cir_queue_t *q){
    
    pthread_mutex_lock(&queue_mutex);
    
    memset(q->data, 0, QUE_SIZE*sizeof(JT808Type));
    
    q->front = q->rear = 0;
    q->count = 0;
    
    q->running = 1;
    
    pthread_mutex_unlock(&queue_mutex);
}

