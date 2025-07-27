#ifndef GRAPH_H
#define GRAPH_H

typedef struct Edge
{
    struct Vertex *dst;
    int weight;
} Edge;

typedef struct Vertex
{
    void *data;
    struct Edge **adjacents; // array of pointers to edges
    int num_adj;
    int capacity;
    int processed;
} Vertex;

typedef struct Graph
{
    Vertex **vertices; // array of pointers to vertices
    int num_vertices;
    int capacity;
    int (*compare)(void *, void *);
} Graph;

// Graph functions

Graph *createGraph(int (*compare)(void *, void *));

Vertex *createVertex(void *data);

void addVertexToGraph(Graph *graph, void *data);

void _addEdgeToVertices(Vertex *src_vertex, Edge *edge);

void addEdgeToGraph(Graph *graph, void *src_key, void *dst_key, int weight, int (*searchByKey)(void *data, void *key));

Vertex *findVertex(Graph *graph, void *data, int (*searchByKey)(void *data, void *key));

void freeVertex(Vertex *vertex);

void freeGraph(Graph *graph);

void printGraph(Graph *graph, void (*printData)(void *));

// Problem specific functions

void mazeDFS(Graph *graph, void (*printData)(void *), void *start_key, void *goal_key, int (*searchByKey)(void *data, void *key));

void connectedComponentsBFS(Graph *graph, void (*printData)(void *));

void topologicalSort(Graph* graph, void (*printData)(void*));

void dijkstraShortestPath(Graph* graph, void* start_data, void (*printData)(void*), int (*searchByKey)(void *data, void *key));

#endif // GRAPH_H
