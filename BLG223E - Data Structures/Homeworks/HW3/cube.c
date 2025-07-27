#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "frac_doublelinklist.h"

typedef struct point {
    float x, y, z;
} Point;

typedef struct Cube {
    Point center;
    float size;
} Cube;

typedef struct mesh {
    DoublyList cube_array;
} Mesh;

Point createPoint(float x, float y, float z) {
    Point p;
    p.x = x; p.y = y; p.z = z;
    return p;
}

Cube* createCube(float cx, float cy, float cz, float size) {
    Cube* c = (Cube*)malloc(sizeof(Cube));
    c->center = createPoint(cx, cy, cz);
    c->size = size;
    return c;
}

void generateFractal(Mesh* mesh, float cx, float cy, float cz, float size, int depth) {
    // Add the current cube
    addBack(&mesh->cube_array, (struct Triangle*)createCube(cx, cy, cz, size));
    
    if (depth <= 0) {
        return;
    }
    
    float new_size = size / 2.0f;
    float offset = size / 2.0f + new_size / 2.0f;
    
    // Generate 6 new cubes on each face
    generateFractal(mesh, cx, cy, cz + offset, new_size, depth - 1); // Front
    generateFractal(mesh, cx, cy, cz - offset, new_size, depth - 1); // Back
    generateFractal(mesh, cx + offset, cy, cz, new_size, depth - 1); // Right
    generateFractal(mesh, cx - offset, cy, cz, new_size, depth - 1); // Left
    generateFractal(mesh, cx, cy + offset, cz, new_size, depth - 1); // Top
    generateFractal(mesh, cx, cy - offset, cz, new_size, depth - 1); // Bottom
}

void freeMesh(Mesh* mesh) {
    Node* current = mesh->cube_array.head;
    while (current != NULL) {
        Node* next = current->next;
        free(current->data); // Free the Cube
        free(current);       // Free the Node
        current = next;
    }
    mesh->cube_array.head = NULL;
    mesh->cube_array.tail = NULL;
    mesh->cube_array.elemcount = 0;
}

void save_stl(const char* filename, Mesh* mesh) {
    FILE* f = fopen(filename, "w");
    if (!f) {
        perror("Error opening STL file");
        return;
    }
    fprintf(f, "solid fractal_cube\n");

    Node* node = mesh->cube_array.head;
    while (node) {
        Cube* c = (Cube*)node->data;
        float half = c->size / 2.0f;
        Point center = c->center;
        
        Point vertices[8] = {
            {center.x - half, center.y - half, center.z - half}, // 0: back bottom left
            {center.x + half, center.y - half, center.z - half}, // 1: back bottom right
            {center.x + half, center.y + half, center.z - half}, // 2: back top right
            {center.x - half, center.y + half, center.z - half}, // 3: back top left
            {center.x - half, center.y - half, center.z + half}, // 4: front bottom left
            {center.x + half, center.y - half, center.z + half}, // 5: front bottom right
            {center.x + half, center.y + half, center.z + half}, // 6: front top right
            {center.x - half, center.y + half, center.z + half}  // 7: front top left
        };
        
        // Define all 6 faces (12 triangles)
        int faces[6][4] = {
            {0, 1, 2, 3}, // Back face
            {4, 5, 6, 7}, // Front face
            {0, 3, 7, 4}, // Left face
            {1, 2, 6, 5}, // Right face
            {3, 2, 6, 7}, // Top face
            {0, 1, 5, 4}  // Bottom face
        };
        
        float normals[6][3] = {
            {0, 0, -1},  // Back
            {0, 0, 1},   // Front
            {-1, 0, 0},  // Left
            {1, 0, 0},   // Right
            {0, 1, 0},    // Top
            {0, -1, 0}    // Bottom
        };
        
        for (int i = 0; i < 6; i++) {
            fprintf(f, "facet normal %f %f %f\n", normals[i][0], normals[i][1], normals[i][2]);
            fprintf(f, "  outer loop\n");
            fprintf(f, "    vertex %f %f %f\n", vertices[faces[i][0]].x, vertices[faces[i][0]].y, vertices[faces[i][0]].z);
            fprintf(f, "    vertex %f %f %f\n", vertices[faces[i][1]].x, vertices[faces[i][1]].y, vertices[faces[i][1]].z);
            fprintf(f, "    vertex %f %f %f\n", vertices[faces[i][2]].x, vertices[faces[i][2]].y, vertices[faces[i][2]].z);
            fprintf(f, "  endloop\n");
            fprintf(f, "endfacet\n");
            
            fprintf(f, "facet normal %f %f %f\n", normals[i][0], normals[i][1], normals[i][2]);
            fprintf(f, "  outer loop\n");
            fprintf(f, "    vertex %f %f %f\n", vertices[faces[i][0]].x, vertices[faces[i][0]].y, vertices[faces[i][0]].z);
            fprintf(f, "    vertex %f %f %f\n", vertices[faces[i][2]].x, vertices[faces[i][2]].y, vertices[faces[i][2]].z);
            fprintf(f, "    vertex %f %f %f\n", vertices[faces[i][3]].x, vertices[faces[i][3]].y, vertices[faces[i][3]].z);
            fprintf(f, "  endloop\n");
            fprintf(f, "endfacet\n");
        }
        
        node = node->next;
    }
    fprintf(f, "endsolid fractal_cube\n");
    fclose(f);
}

int main() {
    while (1) {
        int option;
        printf("------------------------------------\n");
        printf("1- Cube Pattern\n");
        printf("2- Quit\n");
        printf("------------------------------------\n");
        printf("Please enter an action: ");
        scanf("%d", &option);

        if (option == 1) {
            int iteration;
            printf("Enter Iteration count: ");
            scanf("%d", &iteration);

            Mesh mesh;
            initDoublyList(&mesh.cube_array);
            generateFractal(&mesh, 0.0f, 0.0f, 0.0f, 1.0f, iteration);
            save_stl("cube_result.stl", &mesh);
            freeMesh(&mesh);
            printf("STL file 'cube_result.stl' generated.\n");
        } else if (option == 2) {
            break;
        }
    }
    return 0;
}