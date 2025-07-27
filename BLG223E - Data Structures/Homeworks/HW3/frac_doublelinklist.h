#ifndef DOUBLY_LL_H
#define DOUBLY_LL_H

#include <stdlib.h>


typedef struct Node {
    struct Triangle* data;
    struct Node* next;
    struct Node* previous;
} Node;

typedef struct DoublyList {
    Node* head;
    Node* tail;
    int elemcount;
} DoublyList;

// Initialize DoublyList
void initDoublyList(DoublyList* list) {
    list->head = NULL;
    list->tail = NULL;
    list->elemcount = 0;
}

// Add an element to the end of the list
void addBack(DoublyList* list, struct Triangle* new_element) {
    Node* newnode = (Node*)malloc(sizeof(Node));
    newnode->data = new_element;
    newnode->next = NULL;
    newnode->previous = list->tail;

    if (list->tail != NULL)
        list->tail->next = newnode;

    list->tail = newnode;
    list->elemcount++;

    if (list->elemcount == 1)
        list->head = newnode;
}

// Get an element at a specific position in the list
struct Triangle* get(DoublyList* list, int position) {
    Node* iterator = list->head;
    for (int index = 0; index < position; index++) {
        iterator = iterator->next;
    }
    return iterator->data;
}

// Remove the last element from the list
void removeBack(DoublyList* list) {
    if (list->tail != NULL) {
        Node* old = list->tail;

        if (list->tail->previous != NULL)
            list->tail->previous->next = NULL;

        list->tail = list->tail->previous;
        free(old);
        list->elemcount--;
    }
}

// Remove the first element from the list
void removeFront(DoublyList* list) {
    if (list->head != NULL) {
        Node* old = list->head;

        if (list->head->next != NULL)
            list->head->next->previous = NULL;

        list->head = list->head->next;
        free(old);
        list->elemcount--;
    }
}

// Add an element to the front of the list
void addFront(DoublyList* list, struct Triangle* new_element) {
    Node* newnode = (Node*)malloc(sizeof(Node));
    newnode->data = new_element;
    newnode->next = list->head;
    newnode->previous = NULL;

    if (list->head != NULL)
        list->head->previous = newnode;

    list->head = newnode;
    list->elemcount++;

    if (list->elemcount == 1)
        list->tail = newnode;
}

// Add an element at a specific position in the list
void add(DoublyList* list, struct Triangle* new_element, int position) {
    if (position == 0) {
        addFront(list, new_element);
        return;
    } else if (position > list->elemcount) {
        return;
    }

    Node* newnode = (Node*)malloc(sizeof(Node));
    newnode->data = new_element;
    Node* prev_node = NULL;
    Node* position_pointer = list->head;

    for (int index = 0; index < position; index++) {
        prev_node = position_pointer;
        position_pointer = position_pointer->next;
    }

    prev_node->next = newnode;
    newnode->next = position_pointer;
    newnode->previous = prev_node;
    if (position_pointer != NULL) {
        position_pointer->previous = newnode;
    }

    list->elemcount++;
}

// Remove an element at a specific position in the list
void remove_(DoublyList* list, int position) {
    if (position == 0) {
        removeFront(list);
        return;
    } else if (position == list->elemcount - 1) {
        removeBack(list);
        return;
    } else if (position >= list->elemcount) {
        return;
    }

    Node* prev_node = NULL;
    Node* position_pointer = list->head;

    for (int index = 0; index < position; index++) {
        prev_node = position_pointer;
        position_pointer = position_pointer->next;
    }

    Node* old = position_pointer;
    prev_node->next = position_pointer->next;

    if (position_pointer->next != NULL)
        position_pointer->next->previous = prev_node;

    free(old);
    list->elemcount--;
}



#endif
