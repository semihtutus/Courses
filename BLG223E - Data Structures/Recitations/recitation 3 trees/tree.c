#include <stdlib.h>
#include <stdbool.h>
#include <stdio.h>
#include "tree.h" // Ensure to include the header file

// Function to create a new node
Node *create_node(int data)
{
    Node *newNode = (Node *)malloc(sizeof(Node));
    newNode->data = data;
    newNode->left = NULL;
    newNode->right = NULL;
    return newNode;
}
// Function to insert a value into the binary tree
Node *insert_recursive(Node *root, int data)
{
    if (root == NULL)
        return create_node(data);


    if (data < root->data)
    {
        root->left = insert_recursive(root->left, data);
    }
    else
    {
        root->right = insert_recursive(root->right, data);
    }
    return root;
}

Node *insert_iterative(Node *root, int data)
{
    Node *newNode = create_node(data);
    if (root == NULL)
    {
        return newNode;
    }

    Node *current = root;
    while (1)
    {
        if (data < current->data)
        {
            if (current->left == NULL)
            {
                current->left = newNode;
                break;
            }
            current = current->left;
        }
        else
        {
            if (current->right == NULL)
            {
                current->right = newNode;
                break;
            }
            current = current->right;
        }
    }
    return root;
}



Node *minValueNode(Node *node)
{
    Node *current = node;
    while (current && current->left != NULL)
    {
        current = current->left;
    }
    return current;
}


Node *delete_node(Node *root, int data)
{
    if (root == NULL)
        return root;

    if (data < root->data)
    {
        root->left = delete_node(root->left, data);
    }
    else if (data > root->data)
    {
        root->right = delete_node(root->right, data);
    }
    else
    {
        if (root->left == NULL) // Left is NULL, Right may or may not be NULL
        {
            Node *temp = root->right;
            free(root);
            return temp;
        }
        else if (root->right == NULL) // Right is NULL, Left is not NULL
        {
            Node *temp = root->left;
            free(root);
            return temp;
        }
        // Right and Left are not NULL
        // Find the inorder successor, in other words, the smallest node in the right subtree 
        // and replace with the current node
        Node *temp = minValueNode(root->right);

        root->data = temp->data; // This is tricky! Instead of replacing the node, we replace the data

        root->right = delete_node(root->right, temp->data); // Now delete inorder successor
    }
    return root;
}


// Function for in-order traversal of the tree
void traversal(Node *root, int order)
{
    if (root != NULL)
    {
        
        if (order == 1)  // Pre-order
            printf("%d ", root->data);
        
        traversal(root->left, order);

        if (order == 2) // In-order
            printf("%d ", root->data);
        
        traversal(root->right, order);

        if (order == 3) // Post-order
            printf("%d ", root->data);
    }
}

void deleteTree(Node *root)
{
    if (root != NULL)
    {
        deleteTree(root->left);
        deleteTree(root->right);
        free(root);
    }
}

Node* createBinaryTreeFromDescriptions(int** descriptions, int descriptionsSize) {
    Node* nodes[1001] = {NULL};  // Assuming values are between 0-1000
    bool isChild[1001] = {false};  // Track which nodes are children

    // Create nodes and set relationships
    for (int i = 0; i < descriptionsSize; i++) {
        int parent = descriptions[i][0];
        int child = descriptions[i][1];
        bool isLeft = descriptions[i][2]; 

        // Create parent node if it doesn't exist
        if (nodes[parent] == NULL) {
            nodes[parent] = create_node(parent);
        }
        // Create child node if it doesn't exist
        if (nodes[child] == NULL) {
            nodes[child] = create_node(child);
        }

        // Set child relationship
        if (isLeft) {
            nodes[parent]->left = nodes[child];
        } else {
            nodes[parent]->right = nodes[child];
        }
        isChild[child] = true;
    }

    // Find root (node that is not a child)
    for (int i = 0; i < 1001; i++) {
        if (nodes[i] != NULL && !isChild[i]) {
            return nodes[i];
        }
    }
    return NULL;
}
