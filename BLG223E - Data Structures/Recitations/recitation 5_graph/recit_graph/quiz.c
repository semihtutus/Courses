#include <stdio.h>
#include <stdlib.h>

// Question 1
void func(int *ptr_y){
    printf("(3) Address of ptr_y: %p\n", &ptr_y);
    printf("(4) Address stored by ptr_y: %p\n", ptr_y);
}
int main(){
    int x = 5;
    int *ptr_x = &x;
    printf("(1) Address of ptr_x: %p\n", &ptr_x);
    printf("(2) Address stored by ptr_x: %p\n", ptr_x);
    func(ptr_x);
    return 0;
}

// Question 2
// void func(int *ptr_y){
//     ptr_y = (int *)malloc(sizeof(int));
//     *ptr_y = 10;
// }
// int main(){
//     int *ptr_x = (int *)malloc(sizeof(int));
//     func(ptr_x);
//     printf("%d\n", *ptr_x);
//     free(ptr_x);
//     return 0;
// }


// Question 3
// void func(int *ptr_y){
//     ptr_y = (int *)malloc(sizeof(int));
//     *ptr_y = 10;
// }
// int main(){
//     int a = 5;
//     int *ptr_x = &a; 
//     func(ptr_x);
//     printf("%d\n", *ptr_x);
//     free(ptr_x);
//     return 0;
// }


// // Question 4
// void func(int *ptr_y){
//     int y = 10;
//     ptr_y = &y;
// }
// int main() {
//     int x = 5;
//     int *ptr_x = &x;

//     func(ptr_x);
//     printf("x: %d\n", *ptr_x);
//     return 0;
// }

// Question 5
// int* func(){
//     int arr_x[5] = {1, 2, 3, 4, 5};
//     return arr_x;
// }
// int main() {
//     int *ptr_x = func();
//     printf("x: %d\n", ptr_x[0]);
//     return 0;
// }


// Question 6
// void func(int **local_ptr) {
//     local_ptr = (int **)malloc(sizeof(int *));
//     *local_ptr = (int *)malloc(sizeof(int));
//     **local_ptr = 10;
//     printf("Address 2: %p\n", &local_ptr);
// }
// int main() {
//     int **ptr = (int **)malloc(sizeof(int *));
//     printf("Address 1: %p\n", &ptr);
//     func(ptr);
//     printf("%d\n", **ptr);
//     free(ptr);
//     free(*ptr);
//     return 0;
// }

// Question 7
// void func(int *func_ptr) {
//     printf("(3) Size of func_ptr: %lu\n", sizeof(func_ptr));
// }
// int main() {
//     int arr[5] = {1, 2, 3, 4, 5};

//     int *ptr = arr;
//     printf("(1) Size of arr: %lu\n", sizeof(arr));
//     printf("(2) Size of ptr: %lu\n", sizeof(ptr));
//     func(ptr);
// }


// Question X
// struct Node {
//     int data1;
//     int data2;
// };

// int main() {
//     int size = 10;
//     struct Node *arr = (struct Node *)malloc(size * sizeof(struct Node));
//     for (int i = 0; i < size; i++) {
//         arr[i].data1 = i;
//         arr[i].data2 = i*2;
//     }
//     for (int i = 0; i < size; i++) {
//         printf("%d %d\n", arr[i].data1, arr[i].data2);
//     }
//     free(arr);
//     return 0;
// }

