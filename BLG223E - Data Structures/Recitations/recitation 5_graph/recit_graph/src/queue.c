#include <stdlib.h>
#include <stdio.h>
#include "queue.h"

void initQueue(Queue *queue, int capacity) {
    queue->data = (Vertex **)malloc(capacity * sizeof(Vertex *));
    if (queue->data == NULL) {
        fprintf(stderr, "Memory allocation failed\n");
        exit(EXIT_FAILURE);
    }
    queue->front = 0;
    queue->rear = -1;
    queue->capacity = capacity;
    queue->size = 0;
}

bool isQueueEmpty(Queue *queue) {
    return queue->size == 0;
}

bool isQueueFull(Queue *queue) {
    return queue->size == queue->capacity;
}

void enqueue(Queue *queue, Vertex *value) {
    if (isQueueFull(queue)) {
        fprintf(stderr, "Queue overflow\n");
        return;
    }
    queue->rear = (queue->rear + 1) % queue->capacity;
    queue->data[queue->rear] = value;
    queue->size++;
}

Vertex *dequeue(Queue *queue) {
    if (isQueueEmpty(queue)) {
        fprintf(stderr, "Queue underflow\n");
        return NULL; // Return NULL to indicate underflow
    }
    Vertex *value = queue->data[queue->front];
    queue->front = (queue->front + 1) % queue->capacity;
    queue->size--;
    return value;
}

Vertex *peekQueue(Queue *queue) {
    if (isQueueEmpty(queue)) {
        fprintf(stderr, "Queue is empty\n");
        return NULL; // Return NULL to indicate empty queue
    }
    return queue->data[queue->front];
}

void freeQueue(Queue *queue) {
    free(queue->data);
    queue->data = NULL;
    queue->front = 0;
    queue->rear = -1;
    queue->capacity = 0;
    queue->size = 0;
}