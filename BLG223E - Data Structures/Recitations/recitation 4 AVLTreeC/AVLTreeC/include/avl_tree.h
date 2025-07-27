#ifndef AVL_TREE_H
#define AVL_TREE_H

typedef struct AVLNode {
    int data;
    struct AVLNode* left;
    struct AVLNode* right;
    int height;
} AVLNode;

/*
 * Create a new AVL tree node with the given data
 * 
 *     [data]
 *     /   \
 *   NULL  NULL
 */
AVLNode* create_node(int data);

/*
 * Get the height of a node
 * Height of a leaf node is 1
 * Height of NULL is 0
 * 
 *     [A] height = 3
 *     /  \
 *   [B]  [C] height = 2
 *   / \   / \
 *  D   E F   G height = 1
 */
int get_height(AVLNode* node);

/*
 * Get the balance factor of a node
 * balance = height(left subtree) - height(right subtree)
 * 
 * Left heavy:     Right heavy:
 *     [A]+2         [A]-2
 *     /               \
 *   [B]               [B]
 *   /                   \
 * [C]                   [C]
 */
int get_balance(AVLNode* node);

/*
 * Right rotate subtree rooted with y
 * 
 * Before:          After:
 *     y             x
 *    / \          /  \
 *   x   T3  =>   T1   y
 *  / \               / \
 * T1  T2           T2  T3
 */
AVLNode* right_rotate(AVLNode* y);

/*
 * Left rotate subtree rooted with x
 * 
 * Before:          After:
 *   x               y
 *  / \            /  \
 * T1  y    =>    x   T3
 *    / \        / \
 *   T2  T3     T1 T2
 */
AVLNode* left_rotate(AVLNode* x);

/*
 * Insert a node into the AVL Tree
 * The tree remains balanced after insertion
 * 
 * Example insertion of 4:
 *     3            3          2
 *    /     =>    /  \   =>   / \
 *   2           2    4      1   3
 *  /           /                  \
 * 1           1                    4
 */
AVLNode* insert(AVLNode* node, int data);

/*
 * Delete a node from the AVL Tree
 * The tree remains balanced after deletion
 * 
 * Example deletion of 3:
 *     2             2
 *    / \    =>    / \
 *   1   3        1   4
 *        \
 *         4
 */
AVLNode* delete_node(AVLNode* root, int data);

/*
 * Find the minimum value node in a tree
 * Follows left children until reaching a leaf
 * 
 *     [3]
 *     /  \
 *   [2]  [4]  => returns [1]
 *   /
 * [1]
 */
AVLNode* min_value_node(AVLNode* node);

/*
 * Print the tree in-order (Left-Root-Right)
 * 
 *     [2]
 *     / \      => prints: 1 2 3
 *   [1] [3]
 */
void inorder_traversal(AVLNode* root);

/*
 * Free the entire tree
 * Post-order traversal (Left-Right-Root)
 * 
 *     [A]
 *     / \      => Free order: D,E,B,F,G,C,A
 *   [B] [C]
 *   / \ / \
 *  D  E F  G
 */
void free_tree(AVLNode* root);

#endif // AVL_TREE_H 