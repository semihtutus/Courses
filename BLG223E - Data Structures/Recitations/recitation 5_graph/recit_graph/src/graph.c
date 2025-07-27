#include "graph.h"
#include "stack.h"
#include "queue.h"
#include <stdlib.h>
#include <stdio.h>

// allocate memory for the graph and initialize its fields
Graph *createGraph(int (*compare)(void *, void *))
{
    Graph *graph = (Graph *)malloc(sizeof(Graph));
    graph->vertices = NULL;
    graph->num_vertices = 0;
    graph->capacity = 0;
    graph->compare = compare;
    return graph;
}

// allocate memory for the vertex and initialize its fields
Vertex *createVertex(void *data)
{
    Vertex *node = (Vertex *)malloc(sizeof(Vertex));
    node->data = data;
    node->adjacents = NULL;
    node->num_adj = 0;
    node->capacity = 0;
    return node;
}

// if the graph is full, double the capacity, then add the vertex to the graph
void addVertexToGraph(Graph *graph, void *data)
{
    if (graph->num_vertices == graph->capacity)
    {
        graph->capacity = graph->capacity == 0 ? 2 : graph->capacity * 2;
        graph->vertices = (Vertex **)realloc(graph->vertices, graph->capacity * sizeof(Vertex *));
    }
    graph->vertices[graph->num_vertices++] = createVertex(data);
}

// this is a helper function to add an edge between two vertices
// add dst vertex to the src vertex's adjacents array
void _addEdgeToVertices(Vertex *src_vertex, Edge *edge)
{
    if (src_vertex->num_adj == src_vertex->capacity)
    {
        src_vertex->capacity = src_vertex->capacity == 0 ? 2 : src_vertex->capacity * 2;
        src_vertex->adjacents = (Edge **)realloc(src_vertex->adjacents, src_vertex->capacity * sizeof(Edge *));
    }

    src_vertex->adjacents[src_vertex->num_adj++] = edge;
}

// find the vertex with the given key
Vertex *findVertex(Graph *graph, void *key, int (*searchByKey)(void *data, void *key))
{
    for (int i = 0; i < graph->num_vertices; i++)
    {
        if (searchByKey(graph->vertices[i]->data, key) == 0)
        {
            return graph->vertices[i];
        }
    }
    return NULL;
}

// find the vertices with the given data and add an edge between them
void addEdgeToGraph(Graph *graph, void *src_key, void *dst_key, int weight, int (*searchByKey)(void *data, void *key))
{
    Vertex *src_vertex = findVertex(graph, src_key, searchByKey);
    Vertex *dst_vertex = findVertex(graph, dst_key, searchByKey);
    if (src_vertex == NULL || dst_vertex == NULL)
    {
        return;
    }

    Edge *newEdge = (Edge *)malloc(sizeof(Edge));
    newEdge->dst = dst_vertex;
    newEdge->weight = weight;

    _addEdgeToVertices(src_vertex, newEdge);
}

// free the memory allocated for the vertex
void freeVertex(Vertex *vertex)
{
    free(vertex->data);
    for (int i = 0; i < vertex->num_adj; i++)
    {
        free(vertex->adjacents[i]);
    }
    free(vertex->adjacents);
    free(vertex);
}

// free the memory allocated for the graph
void freeGraph(Graph *graph)
{
    for (int i = 0; i < graph->num_vertices; i++)
    {
        freeVertex(graph->vertices[i]);
    }
    free(graph->vertices);
    free(graph);
}

// print the graph using the given printData function
void printGraph(Graph *graph, void (*printData)(void *))
{
    for (int i = 0; i < graph->num_vertices; i++)
    {
        printf("Vertex %d:\nData: ", i);
        printData(graph->vertices[i]->data);
        printf("\nAdjacents: ");
        for (int j = 0; j < graph->vertices[i]->num_adj; j++)
        {
            printData(graph->vertices[i]->adjacents[j]->dst->data);
        }
        printf("\n");
    }
}

// Pseudocode for mazeDFS
// 1. Initialize a stack
// 2. Push the start vertex onto the stack
// 3. While the stack is not empty
// 4.   Pop the top vertex from the stack
// 5.   If the vertex is the goal return
// 6.   Otherwise, push all adjacent vertices that have not been visited onto the stack
// 7.   Mark the vertex as visited
void mazeDFS(Graph *graph, void (*printData)(void *), void *start_key, void *goal_key, int (*searchByKey)(void *data, void *key))
{
    printf("Depth First Search\n");

    if (!graph || graph->num_vertices == 0)
        return;

    Stack *stack = (Stack *)malloc(sizeof(Stack));
    initStack(stack, graph->num_vertices);

    // Prep for search
    for (int i = 0; i < graph->num_vertices; i++)
    {
        graph->vertices[i]->processed = 0;
    }

    Vertex *start_vertex = findVertex(graph, start_key, searchByKey);
    if (!start_vertex)
    {
        printf("Start vertex not found.\n");
        freeStack(stack);
        free(stack);
        return;
    }

    push(stack, start_vertex);
    start_vertex->processed = 1;

    Vertex *tmp;
    while (!isEmpty(stack))
    {
        tmp = pop(stack);
        printData(tmp->data);

        if (searchByKey(tmp->data, goal_key) == 0)
        {
            printf("\nSuccess!");

            freeStack(stack);
            free(stack);
            return;
        }

        printf(" -> ");

        // Push all adjacent vertices that have not been visited
        for (int j = 0; j < tmp->num_adj; j++)
        {
            if (tmp->adjacents[j]->dst->processed == 0)
            {
                push(stack, tmp->adjacents[j]->dst);
                tmp->adjacents[j]->dst->processed = 1;
            }
        }
    }

    freeStack(stack);
    free(stack);
    printf("\n");
}

// Pseudocode for connectedComponentsBFS
// 1. Initialize a queue
// 2. Iterate through all vertices and mark them as unvisited
// 3. For each vertex, if it has not been visited, apply BFS to it and increment the connected components counter
// 4.   While the queue is not empty
// 5.       Dequeue the front vertex from the queue
// 6.       Print the vertex
// 7.       Enqueue all adjacent vertices that have not been visited
// 8.       Repeat
void connectedComponentsBFS(Graph *graph, void (*printData)(void *))
{
    if (!graph || graph->num_vertices == 0)
        return;

    printf("Breadth First Search: \n");
    Vertex *tmp;
    Queue *queue = (Queue *)malloc(sizeof(Queue));
    initQueue(queue, graph->num_vertices);
    int connected_components = 0;

    // Prep for search
    for (int i = 0; i < graph->num_vertices; i++)
    {
        graph->vertices[i]->processed = 0;
    }

    // Iterate through all vertices to cover disconnected graphs
    for (int i = 0; i < graph->num_vertices; i++)
    {
        if (graph->vertices[i]->processed == 0)
        {
            connected_components++;
            printf("\n");
            enqueue(queue, graph->vertices[i]);
            graph->vertices[i]->processed = 1;
            while (!isQueueEmpty(queue))
            {
                tmp = dequeue(queue);
                printData(tmp->data);

                // Enqueue all adjacent vertices that have not been visited
                for (int j = 0; j < tmp->num_adj; j++)
                {
                    if (tmp->adjacents[j]->dst->processed == 0)
                    {
                        enqueue(queue, tmp->adjacents[j]->dst);
                        tmp->adjacents[j]->dst->processed = 1;
                    }
                }
            }
        }

    }

    freeQueue(queue);
    free(queue);
    printf("\nThere are %d connected components in this graph.\n", connected_components);
}

// Pseudocode for topologicalSort
// 1. Initialize a stack

// 3. For each vertex, if it has not been visited, call topologicalSortUtil
// 4.   topologicalSortUtil will push the vertex onto the stack and then recursively call itself for all adjacent vertices that have not been visited
// 5.   After the recursive call, push the vertex onto the stack
// 6. After the iteration, print the stack
void topologicalSortUtil(Graph* graph, int vertex_index, Stack *stack) {
    graph->vertices[vertex_index]->processed = 1;
    
    Vertex* current = graph->vertices[vertex_index];
    for (int i = 0; i < current->num_adj; i++) {
        // Find index of adjacent vertex
        int adj_index = -1;
        for (int j = 0; j < graph->num_vertices; j++) {
            if (graph->vertices[j] == current->adjacents[i]->dst) {
                adj_index = j;
                break;
            }
        }
        
        if (adj_index != -1 && !graph->vertices[adj_index]->processed) {
            topologicalSortUtil(graph, adj_index, stack);
        }
    }
    
    push(stack, graph->vertices[vertex_index]);
}

void topologicalSort(Graph* graph, void (*printData)(void*)) {
    Stack *stack = (Stack*)malloc(sizeof(Stack));
    initStack(stack, graph->num_vertices);

    // Prep for sort
    for (int i = 0; i < graph->num_vertices; i++) {
        graph->vertices[i]->processed = 0;
    }
    
    for (int i = 0; i < graph->num_vertices; i++) {
        if (!graph->vertices[i]->processed) {
            topologicalSortUtil(graph, i, stack);
        }
    }
    
    printf("Topological Sort Order:\n");
    while (!isEmpty(stack)) {
        printData(peek(stack)->data);
        pop(stack);
    }
    printf("\n");
    
    freeStack(stack);
    free(stack);
}

// Pseudocode for dijkstraShortestPath
// 1. Initialize distance and previous arrays
// 2. Set initial distances to infinity (using INT_MAX) and previous to -1
// 3. Set distance to start vertex to 0
// 4. For each vertex, if it is the start vertex, set its distance to 0
// 5. While the queue is not empty
// 6.   Find vertex with minimum distance
// 7.   Mark the vertex as visited
// 8.   Update distances to adjacent vertices
// 9.   Repeat
void dijkstraShortestPath(Graph *graph, void *start_data, void (*printData)(void *), int (*searchByKey)(void *data, void *key))
{
    if (!graph || !start_data || graph->num_vertices == 0)
    {
        return;
    }

    // Find start vertex
    Vertex *start = findVertex(graph, start_data, searchByKey);
    if (!start)
    {
        printf("Start vertex not found.\n");
        return;
    }

    // Initialize distance and previous arrays
    int *distances = (int *)malloc(graph->num_vertices * sizeof(int));
    int *previous = (int *)malloc(graph->num_vertices * sizeof(int));
    int *visited = (int *)calloc(graph->num_vertices, sizeof(int));

    // Set initial distances to infinity (using INT_MAX) and previous to -1
    for (int i = 0; i < graph->num_vertices; i++)
    {
        distances[i] = __INT_MAX__;
        previous[i] = -1;
    }

    // Set distance to start vertex to 0
    // int start_index = -1;
    for (int i = 0; i < graph->num_vertices; i++)
    {
        if (graph->vertices[i] == start)
        {
            //(start_index = i;
            distances[i] = 0;
            break;
        }
    }

    // Main Dijkstra loop
    for (int count = 0; count < graph->num_vertices - 1; count++)
    {
        // Find vertex with minimum distance
        int min_dist = __INT_MAX__;
        int min_index = -1;

        for (int v = 0; v < graph->num_vertices; v++)
        {
            if (!visited[v] && distances[v] < min_dist)
            {
                min_dist = distances[v];
                min_index = v;
            }
        }

        if (min_index == -1)
        {
            break; // No reachable unvisited vertices
        }

        // Mark vertex as visited
        visited[min_index] = 1;

        // Update distances to adjacent vertices
        Vertex *current = graph->vertices[min_index];
        for (int i = 0; i < current->num_adj; i++)
        {
            // Find index of adjacent vertex
            int adj_index = -1;
            for (int j = 0; j < graph->num_vertices; j++)
            {
                if (graph->vertices[j] == current->adjacents[i]->dst)
                {
                    adj_index = j;
                    break;
                }
            }

            if (adj_index != -1 && !visited[adj_index])
            {
                int new_dist = distances[min_index] + current->adjacents[i]->weight;
                if (new_dist < distances[adj_index])
                {
                    distances[adj_index] = new_dist;
                    previous[adj_index] = min_index;
                }
            }
        }
    }

    // Print results
    printf("\nShortest paths from vertex: ");
    printData(start_data);
    printf("\n");

    for (int i = 0; i < graph->num_vertices; i++)
    {
        printf("To vertex: ");
        printData(graph->vertices[i]->data);
        printf(" Distance: ");
        if (distances[i] == __INT_MAX__)
        {
            printf("INFINITY");
        }
        else
        {
            printf("%d", distances[i]);
        }

        // Print path
        if (distances[i] != __INT_MAX__)
        {
            printf(" Path: ");
            // Store path in temporary array
            int path[graph->num_vertices];
            int path_length = 0;
            int curr = i;
            while (curr != -1)
            {
                path[path_length++] = curr;
                curr = previous[curr];
            }
            // Print path in correct order
            for (int j = path_length - 1; j >= 0; j--)
            {
                printData(graph->vertices[path[j]]->data);
                if (j > 0)
                    printf(" -> ");
            }
        }
        printf("\n");
    }

    // Free allocated memory
    free(distances);
    free(previous);
    free(visited);
}