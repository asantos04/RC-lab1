// Implementation of project utility functions

#include "utility.h"

void int_to_char4(char* arr, int n) {
    arr[0] = (n >> 24) & 0xff;
    arr[1] = (n >> 16) & 0xff;
    arr[2] = (n >> 8) & 0xff;
    arr[3] = (n >> 0) & 0xff;
}

void uint_to_char4(char* arr, unsigned int n) {
    arr[0] = (n >> 24) & 0xff;
    arr[1] = (n >> 16) & 0xff;
    arr[2] = (n >> 8) & 0xff;
    arr[3] = (n >> 0) & 0xff;
}

int char4_to_int(char *arr) {
    return (arr[0] << 24) | (arr[1] << 16) | (arr[2] << 8) | (arr[3]);
}

unsigned int char4_to_uint(char *arr) {
    return (arr[0] << 24) | (arr[1] << 16) | (arr[2] << 8) | (arr[3]);
}
