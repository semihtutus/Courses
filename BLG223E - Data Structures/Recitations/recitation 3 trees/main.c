#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

#include "tree.h"

int main()
{
    // // BST recaps: Please examine the code in debug mode

    // // Iterative
    // Node* root = insert_iterative(NULL, 100);
    // root = insert_iterative(root, 150);
    // root = insert_iterative(root, 90);

    // // Recursive
    // root = insert_recursive(root, 80);
    // root = insert_recursive(root, 140);
    // root = insert_recursive(root, 160);
    // root = insert_recursive(root, 155);
    // root = insert_recursive(root, 154);
    // root = insert_recursive(root, 156);
    // root = insert_recursive(root, 170);

    // // Print the tree using in-order traversal
    // // 1: Pre-order, 2: In-order, 3: Post-order
    // traversal(root, 2);

    // // Delete a node from the tree
    // // node with no child
    // root = delete_node(root, 170);
    // printf("\nAfter deleting 170: \n");
    // traversal(root, 2);

    // // node with one child
    // root = delete_node(root, 90);
    // printf("\nAfter deleting 90: \n");
    // traversal(root, 2);

    // // node with two children
    // root = delete_node(root, 160);
    // printf("\nAfter deleting 160: \n");
    // traversal(root, 2);
    // printf("\n");

    // Exercise: Create a binary tree using the descriptions provided 
    // (This exercise is taken from https://leetcode.com/problems/create-binary-tree-from-descriptions/description/)

    // Try it by hand to construct the tree, you will see the challenge.
    int descriptions_arr[6][3] = {
        {50, 40, 1},
        {60, 55, 1},
        {60, 65, 0},
        {30, 10, 1},
        {40, 30, 1},
        {50, 60, 0}
        };

    int *descriptions[6];
    for (int i = 0; i < 6; i++)
    {
        descriptions[i] = descriptions_arr[i];
    }

    Node* root = createBinaryTreeFromDescriptions(descriptions, 6);
    
    // Print the tree using in-order traversal
    printf("In-order traversal of the created tree: ");
    traversal(root, 2);
    printf("\n");

    // If you have better idea to solve this question,
    // you can implement and upload to Leet code

    return 0;
}