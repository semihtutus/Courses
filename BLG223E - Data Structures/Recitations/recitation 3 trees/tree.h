#ifndef TREE_H
#define TREE_H

typedef struct Node
{
    int data;
    struct Node* left;
    struct Node* right;
} Node;


Node* create_node(int data);

Node* delete_node(Node* root, int data);

Node* insert_recursive(Node* root, int data);

Node* insert_iterative(Node* root, int data);

void traversal(Node* root, int order);

void deleteTree(Node* root);


Node* createBinaryTreeFromDescriptions(int** descriptions, int descriptionsSize);


#endif 