#include <stdlib.h>
#include <stdio.h>
#include "stack.h"

void initStack(Stack *stack, int capacity) {
    stack->data = (Vertex **)malloc(capacity * sizeof(Vertex*));
    if (stack->data == NULL) {
        fprintf(stderr, "Memory allocation failed\n");
        exit(EXIT_FAILURE);
    }
    stack->top = -1;
    stack->capacity = capacity;
}

bool isEmpty(Stack *stack) {
    return stack->top == -1;
}

bool isFull(Stack *stack) {
    return stack->top == stack->capacity - 1;
}

void push(Stack *stack, Vertex *value) {
    if (isFull(stack)) {
        fprintf(stderr, "Stack overflow\n");
        return;
    }
    stack->data[++stack->top] = value;
}

Vertex *pop(Stack *stack) {
    if (isEmpty(stack)) {
        fprintf(stderr, "Stack underflow\n");
        return NULL; // Return an invalid value to indicate underflow
    }
    return stack->data[stack->top--];
}

Vertex *peek(Stack *stack) {
    if (isEmpty(stack)) {
        fprintf(stderr, "Stack is empty\n");
        return NULL; // Return an invalid value to indicate empty stack
    }
    return stack->data[stack->top];
}

void freeStack(Stack *stack) {
    free(stack->data);
    stack->data = NULL;
    stack->top = -1;
    stack->capacity = 0;
}