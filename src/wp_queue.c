#define EXTERN extern

#include "../include/quadcopter_main.h"
wp_queue_t* wp_queue_create(int max_capacity,int el_sz){
	wp_queue_t* q =  (wp_queue_t*) calloc(1,sizeof(wp_queue_t));
	q->start_idx = 0;
	q->end_idx = 0;
	q->max_capacity = max_capacity;
	q->el_sz = el_sz;
	q->buffer = (double*) calloc(max_capacity*q->el_sz,sizeof(double));
	return q;
}

bool wp_queue_push(wp_queue_t* q,double* waypoint){
	if(q->end_idx < q->max_capacity){
		memcpy(q->buffer + q->end_idx*q->el_sz,waypoint,q->el_sz*sizeof(double));
		q->end_idx++;
		return true;
	}
	return false;
}

// Returns a live pointer..Caller sould be careful..
double* wp_queue_pop(wp_queue_t* q){
	if(q->start_idx < q->end_idx){
		double* ptr = q->buffer+q->el_sz*q->start_idx;
		q->start_idx++;
		return ptr;
	}
	return NULL;
}

void wp_print(double* wp,int sz){
	if(sz == 8){
	printf("\n");
	printf("X = %0.5lf\tY = %0.5lf\tZ = %0.5lf\tYaw = %0.5lf\n",wp[0],wp[1],wp[2],wp[3]);
	printf("XD = %0.5lf\tYD = %0.5lf\tZD = %0.5lf\tYawD = %0.5lf\n",wp[4],wp[5],wp[6],wp[7]);	
	printf("\n");
	}
}

void wp_queue_print(wp_queue_t* q){
	for(int i=q->start_idx;i<q->end_idx;i++){
		wp_print(q->buffer+q->el_sz*i,q->el_sz);
	}
}

void wp_queue_destroy(wp_queue_t* q){
	free(q->buffer);
	free(q);
}
