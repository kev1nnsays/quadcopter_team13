#ifndef WP_Q_H
#define WP_Q_H 
#include <stdlib.h>
#include <stdio.h>
#include  <stdbool.h>
typedef struct wp_queue wp_queue_t;
struct wp_queue{
  int start_idx;
  int end_idx;
  int max_capacity;
  int el_sz;
  double* buffer;
};

wp_queue_t* wp_queue_create(int max_capacity);

bool wp_queue_push(wp_queue_t* q,double* waypoint);

// Returns a live pointer..Caller sould be careful..
double* wp_queue_pop(wp_queue_t* q);

void wp_print(double* wp);

void wp_queue_print(wp_queue_t* q);

void wp_queue_destroy(wp_queue_t* q);

#endif