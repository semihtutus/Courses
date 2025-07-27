#ifndef STACK_H
#define STACK_H

#include <stdbool.h>
#include "graph.h"

typedef struct Stack {
    struct Vertex **data;
    int top;
    int capacity;
} Stack;

// Function to initialize the stack
void initStack(Stack *stack, int capacity);

// Function to check if the stack is empty
bool isEmpty(Stack *stack);

// Function to check if the stack is full
bool isFull(Stack *stack);

// Function to push an element onto the stack
void push(Stack *stack, struct Vertex *value);

// Function to pop an element from the stack
struct Vertex *pop(Stack *stack);

// Function to peek at the top element of the stack
struct Vertex *peek(Stack *stack);

// Function to free the stack
void freeStack(Stack *stack);

#endif // STACK_H