#include <stdio.h>
#include <stdlib.h>
#include "../include/avl_tree.h"

// Create a new AVL tree node
AVLNode* create_node(int data) {
    AVLNode* node = (AVLNode*)malloc(sizeof(AVLNode));
    node->data = data;
    node->left = NULL;
    node->right = NULL;
    node->height = 1;  // New node is initially at height 1
    return node;
}

// Get the height of a node
int get_height(AVLNode* node) {
    if (node == NULL)
        return 0;
    return node->height;
}

// Get the maximum of two integers
static int max(int a, int b) {
    return (a > b) ? a : b;
}

// Get the balance factor of a node
int get_balance(AVLNode* node) {
    if (node == NULL)
        return 0;
    return get_height(node->left) - get_height(node->right);
}

// Right rotate subtree rooted with y
AVLNode* right_rotate(AVLNode* y) {
    AVLNode* x = y->left;
    AVLNode* T2 = x->right;

    x->right = y;
    y->left = T2;

    // Update heights
    y->height = max(get_height(y->left), get_height(y->right)) + 1;
    x->height = max(get_height(x->left), get_height(x->right)) + 1;

    return x;
}

// Left rotate subtree rooted with x
AVLNode* left_rotate(AVLNode* x) {
    AVLNode* y = x->right;
    AVLNode* T2 = y->left;

    y->left = x;
    x->right = T2;

    // Update heights
    x->height = max(get_height(x->left), get_height(x->right)) + 1;
    y->height = max(get_height(y->left), get_height(y->right)) + 1;

    return y;
}

// Insert a node into the AVL Tree
AVLNode* insert(AVLNode* node, int data) {
    // 1. Perform normal BST insertion
    if (node == NULL)
        return create_node(data);

    if (data < node->data)
        node->left = insert(node->left, data);
    else if (data > node->data)
        node->right = insert(node->right, data);
    else // Equal keys are not allowed in BST
        return node;

    // 2. Update height of this ancestor node
    node->height = 1 + max(get_height(node->left), get_height(node->right));

    // 3. Get the balance factor
    int balance = get_balance(node);

    // Left Left Case
    if (balance > 1 && data < node->left->data)
        return right_rotate(node);

    // Right Right Case
    if (balance < -1 && data > node->right->data)
        return left_rotate(node);

    // Left Right Case
    if (balance > 1 && data > node->left->data) {
        node->left = left_rotate(node->left);
        return right_rotate(node);
    }

    // Right Left Case
    if (balance < -1 && data < node->right->data) {
        node->right = right_rotate(node->right);
        return left_rotate(node);
    }

    return node;
}

// Find the node with minimum value
AVLNode* min_value_node(AVLNode* node) {
    AVLNode* current = node;
    while (current->left != NULL)
        current = current->left;
    return current;
}

// Delete a node from the AVL Tree
AVLNode* delete_node(AVLNode* root, int data) {
    if (root == NULL)
        return root;

    if (data < root->data)
        root->left = delete_node(root->left, data);
    else if (data > root->data)
        root->right = delete_node(root->right, data);
    else {
        // Node with only one child or no child
        if (root->left == NULL || root->right == NULL) {
            AVLNode* temp = root->left ? root->left : root->right;

            // No child case
            if (temp == NULL) {
                temp = root;
                root = NULL;
            } else // One child case
                *root = *temp; // Copy the contents of the non-empty child
            
            free(temp);
        } else {
            // Node with two children
            AVLNode* temp = min_value_node(root->right);
            root->data = temp->data;
            root->right = delete_node(root->right, temp->data);
        }
    }

    // If the tree had only one node then return
    if (root == NULL)
        return root;

    // Update height
    root->height = 1 + max(get_height(root->left), get_height(root->right));

    // Get the balance factor
    int balance = get_balance(root);

    // Left Left Case
    if (balance > 1 && get_balance(root->left) >= 0)
        return right_rotate(root);

    // Left Right Case
    if (balance > 1 && get_balance(root->left) < 0) {
        root->left = left_rotate(root->left);
        return right_rotate(root);
    }

    // Right Right Case
    if (balance < -1 && get_balance(root->right) <= 0)
        return left_rotate(root);

    // Right Left Case
    if (balance < -1 && get_balance(root->right) > 0) {
        root->right = right_rotate(root->right);
        return left_rotate(root);
    }

    return root;
}

// Print the tree in-order
void inorder_traversal(AVLNode* root) {
    if (root != NULL) {
        inorder_traversal(root->left);
        printf("%d ", root->data);
        inorder_traversal(root->right);
    }
}

// Free the entire tree
void free_tree(AVLNode* root) {
    if (root != NULL) {
        free_tree(root->left);
        free_tree(root->right);
        free(root);
    }
}
