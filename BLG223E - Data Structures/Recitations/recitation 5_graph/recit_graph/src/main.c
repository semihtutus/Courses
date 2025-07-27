#include <stdio.h>
#include <stdlib.h>
#include "graph.h"

typedef struct Node
{
    int value;
} Node;

Node* createNode(int value)
{
    Node *node = (Node *)malloc(sizeof(Node));
    node->value = value;
    return node;
}

// graph related functions do not know how to compare the data
int compare(void *a, void *b)
{
    return ((Node *)a)->value - ((Node *)b)->value;
}

// graph related functions do not know how to print the data
void printData(void *data)
{
    printf("%d ", ((Node *)data)->value);
}

// search function to find the vertex with the given data
int searchByKey(void *data, void *key)
{
    return ((Node *)data)->value - *(int *)key;
}

void mazeProblem();
void countConnectedCompProblem();
void topologicalSortProblem();
void shortestPathProblem();

int main() {
    
    mazeProblem();
    // countConnectedCompProblem();
    // topologicalSortProblem();
    // shortestPathProblem();

    return 0;
}

void mazeProblem(){
    Graph *graph = createGraph(compare);
    // A: 0, B: 1, C: 2, D: 3, E: 4, F: 5, G: 6, H: 7, I: 8, J: 9, K: 10, L: 11, M: 12, N: 13, O: 14
    // Edges: A-B, B-C, B-E, E-F, ...
    int keys[15];
    for (int i = 0; i < 15; i++)
    {
        keys[i] = i;
        addVertexToGraph(graph, createNode(keys[i]));
    }


    // Maze configuration
    addEdgeToGraph(graph, &keys[0], &keys[1], 1, searchByKey);
    addEdgeToGraph(graph, &keys[1], &keys[0], 1, searchByKey);

    addEdgeToGraph(graph, &keys[1], &keys[2], 1, searchByKey);
    addEdgeToGraph(graph, &keys[2], &keys[1], 1, searchByKey);
    
    addEdgeToGraph(graph, &keys[4], &keys[1], 1, searchByKey);
    addEdgeToGraph(graph, &keys[1], &keys[4], 1, searchByKey);
    
    addEdgeToGraph(graph, &keys[2], &keys[3], 1, searchByKey);
    addEdgeToGraph(graph, &keys[3], &keys[2], 1, searchByKey);

    addEdgeToGraph(graph, &keys[2], &keys[6], 1, searchByKey);
    addEdgeToGraph(graph, &keys[6], &keys[2], 1, searchByKey);
    
    addEdgeToGraph(graph, &keys[4], &keys[5], 1, searchByKey);
    addEdgeToGraph(graph, &keys[5], &keys[4], 1, searchByKey);
    
    addEdgeToGraph(graph, &keys[4], &keys[6], 1, searchByKey);
    addEdgeToGraph(graph, &keys[6], &keys[4], 1, searchByKey);
    
    addEdgeToGraph(graph, &keys[5], &keys[6], 1, searchByKey);
    addEdgeToGraph(graph, &keys[6], &keys[5], 1, searchByKey);
    
    addEdgeToGraph(graph, &keys[5], &keys[7], 1, searchByKey);
    addEdgeToGraph(graph, &keys[7], &keys[5], 1, searchByKey);
    
    addEdgeToGraph(graph, &keys[7], &keys[8], 1, searchByKey);
    addEdgeToGraph(graph, &keys[8], &keys[7], 1, searchByKey);
    
    addEdgeToGraph(graph, &keys[7], &keys[9], 1, searchByKey);
    addEdgeToGraph(graph, &keys[9], &keys[7], 1, searchByKey);
    
    addEdgeToGraph(graph, &keys[9], &keys[10], 1, searchByKey);
    addEdgeToGraph(graph, &keys[10], &keys[9], 1, searchByKey);
    
    addEdgeToGraph(graph, &keys[9], &keys[11], 1, searchByKey);
    addEdgeToGraph(graph, &keys[11], &keys[9], 1, searchByKey);
    
    addEdgeToGraph(graph, &keys[13], &keys[11], 1, searchByKey);
    addEdgeToGraph(graph, &keys[11], &keys[13], 1, searchByKey);
    
    addEdgeToGraph(graph, &keys[12], &keys[11], 1, searchByKey);
    addEdgeToGraph(graph, &keys[11], &keys[12], 1, searchByKey);
    
    addEdgeToGraph(graph, &keys[11], &keys[14], 1, searchByKey);
    addEdgeToGraph(graph, &keys[14], &keys[11], 1, searchByKey);

    // TODO starting point
    mazeDFS(graph, printData, &keys[0], &keys[14], searchByKey);

    freeGraph(graph);
}

void countConnectedCompProblem(){
    Graph *graph = createGraph(compare);
    // int keys[15] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 
    //                 10, 11, 12, 13, 14};

    int keys[14];
    for (int i = 0; i < 14; i++)
    {
        keys[i] = i;
        addVertexToGraph(graph, createNode(keys[i]));
    }


    // Connected component configuration
    addEdgeToGraph(graph, &keys[0], &keys[1], 1, searchByKey);
    addEdgeToGraph(graph, &keys[0], &keys[2], 1, searchByKey);
    addEdgeToGraph(graph, &keys[1], &keys[0], 1, searchByKey);
    addEdgeToGraph(graph, &keys[1], &keys[2], 1, searchByKey);
    addEdgeToGraph(graph, &keys[2], &keys[0], 1, searchByKey);
    addEdgeToGraph(graph, &keys[2], &keys[1], 1, searchByKey);

    addEdgeToGraph(graph, &keys[3], &keys[4], 1, searchByKey);
    addEdgeToGraph(graph, &keys[3], &keys[5], 1, searchByKey);
    addEdgeToGraph(graph, &keys[4], &keys[3], 1, searchByKey);
    addEdgeToGraph(graph, &keys[4], &keys[5], 1, searchByKey);
    addEdgeToGraph(graph, &keys[5], &keys[3], 1, searchByKey);
    addEdgeToGraph(graph, &keys[5], &keys[4], 1, searchByKey);

    addEdgeToGraph(graph, &keys[6], &keys[7], 1, searchByKey);
    addEdgeToGraph(graph, &keys[6], &keys[8], 1, searchByKey);
    addEdgeToGraph(graph, &keys[6], &keys[9], 1, searchByKey);
    addEdgeToGraph(graph, &keys[7], &keys[6], 1, searchByKey);
    addEdgeToGraph(graph, &keys[7], &keys[8], 1, searchByKey);
    addEdgeToGraph(graph, &keys[8], &keys[6], 1, searchByKey);
    addEdgeToGraph(graph, &keys[8], &keys[7], 1, searchByKey);
    addEdgeToGraph(graph, &keys[9], &keys[6], 1, searchByKey);

    addEdgeToGraph(graph, &keys[10], &keys[11], 1, searchByKey);
    addEdgeToGraph(graph, &keys[10], &keys[12], 1, searchByKey);
    addEdgeToGraph(graph, &keys[10], &keys[13], 1, searchByKey);
    addEdgeToGraph(graph, &keys[11], &keys[10], 1, searchByKey);
    addEdgeToGraph(graph, &keys[11], &keys[12], 1, searchByKey);
    addEdgeToGraph(graph, &keys[12], &keys[10], 1, searchByKey);
    addEdgeToGraph(graph, &keys[12], &keys[11], 1, searchByKey);
    addEdgeToGraph(graph, &keys[12], &keys[13], 1, searchByKey);
    addEdgeToGraph(graph, &keys[13], &keys[10], 1, searchByKey);
    addEdgeToGraph(graph, &keys[13], &keys[12], 1, searchByKey);

    connectedComponentsBFS(graph, printData);

    freeGraph(graph);
}

void topologicalSortProblem()
{
    Graph *graph = createGraph(compare);

    int keys[10];
    for (int i = 0; i < 10; i++)
    {
        keys[i] = i;
        addVertexToGraph(graph, createNode(keys[i]));
    }
    // 0: C, 1: DS, 2: OOP, 3: Algo1, 4: OS, 5: Algo2, 6: GradProject, 7: DC, 8:CompOrg, 9: CompArch

    // Topological Sort Configuration
    addEdgeToGraph(graph, &keys[0], &keys[1], 1, searchByKey);
    addEdgeToGraph(graph, &keys[0], &keys[2], 1, searchByKey);

    addEdgeToGraph(graph, &keys[1], &keys[3], 1, searchByKey);
    addEdgeToGraph(graph, &keys[1], &keys[4], 1, searchByKey);

    addEdgeToGraph(graph, &keys[2], &keys[3], 1, searchByKey);
    addEdgeToGraph(graph, &keys[3], &keys[5], 1, searchByKey);
    addEdgeToGraph(graph, &keys[4], &keys[6], 1, searchByKey);
    addEdgeToGraph(graph, &keys[5], &keys[6], 1, searchByKey);
    addEdgeToGraph(graph, &keys[7], &keys[8], 1, searchByKey);
    addEdgeToGraph(graph, &keys[8], &keys[9], 1, searchByKey);
    addEdgeToGraph(graph, &keys[9], &keys[6], 1, searchByKey);

    topologicalSort(graph, printData);

    freeGraph(graph);
}

void shortestPathProblem()
{
    Graph *graph = createGraph(compare);

    int keys[8];
    for (int i = 0; i < 8; i++)
    {
        keys[i] = i;
        addVertexToGraph(graph, createNode(keys[i]));
    }

    // Dijkstra configuration
    addEdgeToGraph(graph, &keys[0], &keys[1], 7, searchByKey);
    addEdgeToGraph(graph, &keys[0], &keys[2], 5, searchByKey);
    addEdgeToGraph(graph, &keys[1], &keys[0], 7, searchByKey);
    addEdgeToGraph(graph, &keys[1], &keys[2], 8, searchByKey);
    addEdgeToGraph(graph, &keys[1], &keys[3], 3, searchByKey);
    addEdgeToGraph(graph, &keys[2], &keys[0], 5, searchByKey);
    addEdgeToGraph(graph, &keys[2], &keys[1], 8, searchByKey);
    addEdgeToGraph(graph, &keys[3], &keys[1], 3, searchByKey);
    addEdgeToGraph(graph, &keys[3], &keys[5], 9, searchByKey);
    addEdgeToGraph(graph, &keys[3], &keys[7], 5, searchByKey);
    addEdgeToGraph(graph, &keys[4], &keys[7], 2, searchByKey);
    addEdgeToGraph(graph, &keys[5], &keys[3], 9, searchByKey);
    addEdgeToGraph(graph, &keys[7], &keys[3], 5, searchByKey);
    addEdgeToGraph(graph, &keys[7], &keys[4], 2, searchByKey);

    dijkstraShortestPath(graph, &keys[0], printData, searchByKey);

    freeGraph(graph);
}
