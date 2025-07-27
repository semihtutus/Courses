#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define RED "\x1b[91m"
#define GREEN "\x1b[92m"
#define YELLOW "\x1b[93m"
#define BLUE "\x1b[94m"
#define RESET "\x1b[0m"

struct WormPart {
    int x, y;
};

struct Node {
    struct WormPart* data;
    struct Node* next;
    struct Node* previous;
};

struct DoublyList {
    struct Node* head;
    struct Node* tail;
    int elemcount;
};

void addFront(struct DoublyList* list, struct WormPart* new_element) {
    struct Node* newnode = (struct Node*)malloc(sizeof(struct Node));
    newnode->data = new_element;
    newnode->next = list->head;
    newnode->previous = NULL;

    if (list->head != NULL)
        list->head->previous = newnode;

    list->head = newnode;
    list->elemcount++;

    if (list->elemcount == 1)
        list->tail = newnode;
}

void removeNode(struct DoublyList* list, struct Node* node) {
    if (node == NULL || list == NULL) return;

    if (node->previous != NULL)
        node->previous->next = node->next;
    else
        list->head = node->next;

    if (node->next != NULL)
        node->next->previous = node->previous;
    else
        list->tail = node->previous;

    free(node->data);
    free(node);
    list->elemcount--;
}

struct DoublyList* create_worm(char* filename) {
    struct DoublyList* worm = (struct DoublyList*)malloc(sizeof(struct DoublyList));
    worm->head = NULL;
    worm->tail = NULL;
    worm->elemcount = 0;

    FILE* infile = fopen(filename, "r");
    if (!infile) {
        printf("Failed to open %s\n", filename);
        return worm;
    }

    char line[256];
    while (fgets(line, sizeof(line), infile)) {
        int x, y;
        if (sscanf(line, "%d %d", &x, &y) == 2) {
            struct WormPart* new_part = (struct WormPart*)malloc(sizeof(struct WormPart));
            new_part->x = x;
            new_part->y = y;
            addFront(worm, new_part);
        }
    }

    fclose(infile);
    return worm;
}

//Matrix view function
void view_matrix(struct DoublyList* wormfield[], int wormcount) {
    char matrix[10][10];
    // Dots to represent empty cells
    for (int i = 0; i < 10; i++)
        for (int j = 0; j < 10; j++)
            matrix[i][j] = '.';

    // Place worms in the matrix
    char worm_chars[] = {'1', '2', '3', '4'};
    const char* colors[] = {RED, GREEN, YELLOW, BLUE};
    
    for (int w = 0; w < wormcount; w++) {
        if (!wormfield[w]) continue;
        struct Node* ptr = wormfield[w]->head;
        while (ptr != NULL) {
            int x = ptr->data->x - 1;
            int y = ptr->data->y - 1;
            if (x >= 0 && x < 10 && y >= 0 && y < 10)
                matrix[x][y] = worm_chars[w];
            ptr = ptr->next;
        }
    }

    // Write the matrix
    printf("\n");
    for (int row = 0; row < 10; row++) {
        printf("    "); 
        for (int col = 0; col < 10; col++) {
            char c = matrix[row][col];
            int widx = -1;
            for (int i = 0; i < wormcount; i++) {
                if (c == worm_chars[i]) widx = i;
            }
            if (widx != -1)
                printf("%s %c %s", colors[widx], c, RESET);
            else
                printf(" %c ", c);
        }
        printf("\n\n");
    }
    printf("\n");
}

int attack(struct DoublyList* wormfield[], int wormcount, int x, int y) {
    for (int w = 0; w < wormcount; w++) {
        if (wormfield[w] == NULL || wormfield[w]->elemcount == 0) continue;

        struct Node* current = wormfield[w]->head;

        while (current != NULL) {
            int cx = current->data->x;
            int cy = current->data->y;

            if (cx == x && cy == y) {
                struct Node* to_delete[3];
                int count = 0;

                to_delete[count++] = current;

                if (current->previous != NULL) {
                    int px = current->previous->data->x;
                    int py = current->previous->data->y;
                    if ((px == x + 1 && py == y) || (px == x - 1 && py == y) ||
                        (px == x && py == y + 1) || (px == x && py == y - 1)) {
                        to_delete[count++] = current->previous;
                    }
                }

                if (current->next != NULL && count < 3) {
                    int nx = current->next->data->x;
                    int ny = current->next->data->y;
                    if ((nx == x + 1 && ny == y) || (nx == x - 1 && ny == y) ||
                        (nx == x && ny == y + 1) || (nx == x && ny == y - 1)) {
                        to_delete[count++] = current->next;
                    }
                }

                for (int i = 0; i < count; i++) {
                    removeNode(wormfield[w], to_delete[i]);
                }
                return 1;
            }

            current = current->next;
        }
    }
    return 0;
}

int main() {
    struct DoublyList* wormfield[100] = {NULL};
    int wormcount = 4;

    char* filenames[4] = {"worms/worm1.txt", "worms/worm2.txt", "worms/worm3.txt", "worms/worm4.txt"};

    for (int i = 0; i < wormcount; i++) {
        wormfield[i] = create_worm(filenames[i]);
    }

    while (1) {
        int option;
        printf("\n------------------------------------\n");
        printf("1- View the worms\n");
        printf("2- Attack\n");
        printf("3- Quit\n");
        printf("------------------------------------\n");
        printf("Please enter an action: ");
        scanf("%d", &option);

        if (option == 1) {
            printf("\nWorm List\n");
            printf("---------\n");

            for (int i = 0; i < wormcount; i++) {
                if (!wormfield[i]) continue;
                printf("Worm %d: ", i + 1);

                struct Node* ptr = wormfield[i]->head;
                while (ptr != NULL) {
                    printf("(%d,%d) ", ptr->data->x, ptr->data->y);
                    ptr = ptr->next;
                }
                printf("\n");
            }

            view_matrix(wormfield, wormcount);

        } else if (option == 2) {
            int x, y;
            printf("\nEnter x y coordinates to attack (1-10): ");
            scanf("%d %d", &x, &y);

            if (x < 1 || x > 10 || y < 1 || y > 10) {
                printf("Invalid coordinates! Please enter values between 1 and 10.\n");
                continue;
            }

            int result = attack(wormfield, wormcount, x, y);
            if (result == 1)
                printf("\nSuccessful attack at (%d,%d)!\n", x, y);
            else
                printf("\nMissed attack at (%d,%d)!\n", x, y);

        } else if (option == 3) {
            break;
        } else {
            printf("\nInvalid input! Please enter 1, 2, or 3.\n");
        }
    }

    for (int i = 0; i < wormcount; i++) {
        if (wormfield[i] != NULL) {
            while (wormfield[i]->elemcount > 0) {
                removeNode(wormfield[i], wormfield[i]->head);
            }
            free(wormfield[i]);
        }
    }

    printf("\nGame ended. Thanks for playing!\n");
    return 0;
}