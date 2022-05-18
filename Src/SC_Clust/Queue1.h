/*
 * Queue1.h
 *
 *  Created on: Oct 16, 2020
 *      Author: jad
 */

#ifndef SC_CLUST_QUEUE1_H_
#define SC_CLUST_QUEUE1_H_

#define MAX 10

typedef struct Queue {
	L3_packet packets[MAX];
	uint8_t front;
	int8_t rear;
	uint8_t size;

} Queue1;

void createQueue(Queue1 *q) {
	q->front = 0;
	q->size = 0;
	q->rear = 0;
}

L3_packet front(Queue1 *q) {
   return q->packets[q->front];
}

uint8_t isEmpty(Queue1 *q) {
   return q->size == 0;
}

uint8_t isFull(Queue1 *q) {
   return q->size == MAX;
}

uint8_t size(Queue1 *q) {
   return q->size;
}

void enQueue(Queue1* q, L3_packet packet) {
	if(!isFull(q)) {

	  if(q->rear == MAX) {
		 q->rear = 0;
	  }

	  q->packets[q->rear++] = packet;
	  q->size++;
   }
}

L3_packet deQueue(Queue1 *q) {
	L3_packet data;
	if(!isEmpty(q)) {
		data = q->packets[q->front++];

		if(q->front == MAX) {
		  q->front = 0;
		}

		q->size--;
		return data;
	} else {
		data.io_port = 255;
		return data;
	}

}


#endif /* SC_CLUST_QUEUE1_H_ */
