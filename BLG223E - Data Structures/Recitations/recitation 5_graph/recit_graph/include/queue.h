#ifndef QUEUE_H
#define QUEUE_H

#include <stdbool.h>
#include "graph.h"

typedef struct Queue {
    Vertex **data;
    int front;
    int rear;
    int capacity;
    int size;
} Queue;

// Function to initialize the queue
void initQueue(Queue *queue, int capacity);

// Function to check if the queue is empty
bool isQueueEmpty(Queue *queue);

// Function to check if the queue is full
bool isQueueFull(Queue *queue);

// Function to enqueue an element into the queue
void enqueue(Queue *queue, Vertex *value);

// Function to dequeue an element from the queue
Vertex *dequeue(Queue *queue);

// Function to peek at the front element of the queue
Vertex *peekQueue(Queue *queue);

// Function to free the queue
void freeQueue(Queue *queue);

#endif // QUEUE_H