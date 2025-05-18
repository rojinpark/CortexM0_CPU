/*********************************************************
 *                                                       *
 *  EE511 Project 1                                      *
 *                                                       *
 *  C test code                                          *
 *                                                       *
 *********************************************************/

/*your test code here*/
#include <stdio.h>

void sort_string(const char *src, char *dst, int len) {
    int i, j;
    for(i = 0; i < len; i++) dst[i] = src[i];
    dst[len] = '\0';
    for(i = 0; i < len-1; i++) {
        for(j = i+1; j < len; j++) {
            if(dst[i] > dst[j]) {
                char tmp = dst[i];
                dst[i] = dst[j];
                dst[j] = tmp;
            }
        }
    }
}

int main() {
    char x[13] = "QWERTYASDFGH"; 
    char y[13];
    sort_string(x, y, 12);
    return 0;
}

