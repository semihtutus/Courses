#include <stdio.h>
#include "../include/avl_tree.h"

int main() {
    AVLNode* root = NULL;

    // Create the example tree from documentation
    root = insert(root, 10);
    root = insert(root, 5);
    root = insert(root, 15);
    root = insert(root, 3);
    root = insert(root, 7);
    root = insert(root, 18);
    root = insert(root, 12);
    root = insert(root, 20);
    root = insert(root, 4);
    root = insert(root, 6);
    
    printf("\nInorder traversal of the AVL tree: ");
    inorder_traversal(root);
    printf("\n");

    // Free the tree
    free_tree(root);

    return 0;
} 