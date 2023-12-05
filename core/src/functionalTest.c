#include "rocc.h"
// check if need to code address using special regs!!!!!!!!1
#include <inttypes.h>
#include <assert.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

float dot_product_simple(float v[], float u[], int n)
{
    float result = 0.0;
    for (int i = 0; i < n; i++)
        result += v[i]*u[i];
    return result;
}

void print_arr(float arr_head[], int length){
    uint32_t gtruth;
    printf("[ ");
    for (int i=0; i < length; i++) {
        gtruth = *((uint32_t*)& (arr_head[i]));
        printf("%f ", gtruth);
    }
    printf("]");
}

// this is a simplified version of test
// all vector are preset, no need to append zeros and only take 1 fold
int simple_functional_test() {
    // --------------- CASE 1 ---------------------------
    float t_row1[4] = {1.1, 2.2, 3.3, -4.4 };
    float y_xhat1[4] = {-1, 2, 3, 4};
    
    uint32_t sum = 0;

    uint32_t start_time;
    uint32_t end_time;

    start_time = HAL_CLINT_getTime();
    asm volatile ("fence");
    ROCC_INSTRUCTION_SS(0, t_row1, y_xhat1, 0);   // passing two vector base       
    ROCC_INSTRUCTION_DS(0, sum, 4, 1);  // passing result pointer and the length
    asm volatile ("fence");
    end_time = HAL_CLINT_getTime();
    printf("result sum  %x \n", sum);
    printf("Total time for calculation of LQR: %d \n", end_time - start_time);

    start_time = HAL_CLINT_getTime();
    float a = dot_product_simple (t_row1, y_xhat1, 4);
    end_time = HAL_CLINT_getTime();
    uint32_t gtruth = *((uint32_t*)& (a));
    
    printf("truth  %x  \n", gtruth);
    printf("Total time for calculation of default: %d \n", end_time - start_time);

    printf("-------------end of case 1:4-------------------\n");
    
    // --------------- CASE 2 ---------------------------
    float t_row2[5] = {1.1, 2.2, 3.3, -4.4, 7};
    float y_xhat2[5] = {-1, 2, 3, 4, 98};
    
    uint32_t sum_2 = 0;
    
    start_time = HAL_CLINT_getTime();
    asm volatile ("fence");
    ROCC_INSTRUCTION_SS(0, t_row2, y_xhat2, 0);   // passing two vector base       
    ROCC_INSTRUCTION_DS(0, sum_2, 5, 1);  // passing result pointer and the length
    asm volatile ("fence");
    end_time = HAL_CLINT_getTime();
    printf("result sum  %x \n", sum_2);
    printf("Total time for calculation of LQR: %d \n", end_time - start_time);

    start_time = HAL_CLINT_getTime();
    a = dot_product_simple (t_row2, y_xhat2, 5);
    end_time = HAL_CLINT_getTime();
    gtruth = *((uint32_t*)& (a));
    printf("truth  %x  \n", gtruth);
    printf("Total time for calculation of default: %d \n", end_time - start_time);

    printf("-------------end of case 2:5-------------------\n");

    float t_row3[15] = {1.1, 2.2, 3.3, -4.4, 7, 1.1, 2.2, 3.3, -4.4, 7, 1.1, 2.2, 3.3, -4.4, 7};
    float y_xhat3[15] = {-1, 2, 3, 4, 98, -1, 2, 3, 4, 98, -1, 2, 3, 4, 98};

    start_time = HAL_CLINT_getTime();
    asm volatile ("fence");
    ROCC_INSTRUCTION_SS(0, t_row3, y_xhat3, 0);   // passing two vector base       
    ROCC_INSTRUCTION_DS(0, sum_2, 15, 1);  // passing result pointer and the length
    asm volatile ("fence");
    end_time = HAL_CLINT_getTime();
    printf("result sum  %x \n", sum_2);
    printf("Oirginal t_row value: ");
    print_arr(*((uint32_t*)& (t_row3)), 15);
    printf("Oirginal t_row value: ");
    print_arr(*((uint32_t*)& (y_xhat3)), 15);
    printf("Total time for calculation of LQR: %d \n", end_time - start_time);

    start_time = HAL_CLINT_getTime();
    a = dot_product_simple (t_row3, y_xhat3, 15);
    end_time = HAL_CLINT_getTime();
    gtruth = *((uint32_t*)& (a));
    printf("truth  %x  \n", gtruth);
    printf("Total time for calculation of default: %d \n", end_time - start_time);

    printf("-------------end of case 3:15-------------------\n");

    float t_row4[11] = {1.1, 2.2, 3.3, -4.4, 7, 1.1, 2.2, 3.3, -4.4, 7, 1.1};
    float y_xhat4[11] = {-1, 2, 3, 4, 98, -1, 2, 3, 4, 98, -1};

    asm volatile ("fence");
    ROCC_INSTRUCTION_SS(0, t_row4, y_xhat4, 0);   // passing two vector base       
    ROCC_INSTRUCTION_DS(0, sum_2, 11, 1);  // passing result pointer and the length
    asm volatile ("fence");

    a = dot_product_simple (t_row4, y_xhat4, 11);
    gtruth = *((uint32_t*)& (a));
    printf("result sum  %x \n", sum_2);

    printf("truth  %x  \n", gtruth);
    printf("-------------end of case 4:11-------------------\n");

    float t_row5[12] = {1.1, 2.2, 3.3, -4.4, 7, 1.1, 2.2, 3.3, -4.4, 7, 1.1, 2.2};
    float y_xhat5[12] = {-1, 2, 3, 4, 98, -1, 2, 3, 4, 98, -1, 2};

    asm volatile ("fence");
    ROCC_INSTRUCTION_SS(0, t_row3, y_xhat3, 0);   // passing two vector base       
    ROCC_INSTRUCTION_DS(0, sum_2, 12, 1);  // passing result pointer and the length
    asm volatile ("fence");

    a = dot_product_simple (t_row5, y_xhat5, 12);
    gtruth = *((uint32_t*)& (a));
    printf("result sum  %x \n", sum_2);

    printf("truth  %x  \n", gtruth);
    printf("-------------end of case 5:12-------------------\n");

    float t_row6[13] = {1.1, 2.2, 3.3, -4.4, 7, 1.1, 2.2, 3.3, -4.4, 7, 1.1, 2.2, 3.3};
    float y_xhat6[13] = {-1, 2, 3, 4, 98, -1, 2, 3, 4, 98, -1, 2, 3};

    asm volatile ("fence");
    ROCC_INSTRUCTION_SS(0, t_row6, y_xhat6, 0);   // passing two vector base       
    ROCC_INSTRUCTION_DS(0, sum_2, 13, 1);  // passing result pointer and the length
    asm volatile ("fence");

    a = dot_product_simple (t_row6, y_xhat6, 13);
    gtruth = *((uint32_t*)& (a));
    printf("result sum  %x \n", sum_2);

    printf("truth  %x  \n", gtruth);
    printf("-------------end of case 6:13-------------------\n");

    float t_row7[14] = {1.1, 2.2, 3.3, -4.4, 7, 1.1, 2.2, 3.3, -4.4, 7, 1.1, 2.2, 3.3, -4.4};
    float y_xhat7[14] = {-1, 2, 3, 4, 98, -1, 2, 3, 4, 98, -1, 2, 3, 4};

    asm volatile ("fence");
    ROCC_INSTRUCTION_SS(0, t_row7, y_xhat7, 0);   // passing two vector base       
    ROCC_INSTRUCTION_DS(0, sum_2, 14, 1);  // passing result pointer and the length
    asm volatile ("fence");

    a = dot_product_simple (t_row7, y_xhat7, 14);
    gtruth = *((uint32_t*)& (a));
    printf("result sum  %x \n", sum_2);

    printf("truth  %x  \n", gtruth);
    printf("-------------end of case 7:14-------------------\n");

    float t_row8[16] = {1.1, 2.2, 3.3, -4.4, 7, 1.1, 2.2, 3.3, -4.4, 7, 1.1, 2.2, 3.3, -4.4, 7, 13.8};
    float y_xhat8[16] = {-1, 2, 3, 4, 98, -1, 2, 3, 4, 98, -1, 2, 3, 4, 98, -1.3};

    asm volatile ("fence");
    ROCC_INSTRUCTION_SS(0, t_row8, y_xhat8, 0);   // passing two vector base       
    ROCC_INSTRUCTION_DS(0, sum_2, 16, 1);  // passing result pointer and the length
    asm volatile ("fence");

    a = dot_product_simple (t_row8, y_xhat8, 16);
    gtruth = *((uint32_t*)& (a));
    printf("result sum  %x \n", sum_2);

    printf("truth  %x  \n", gtruth);
    printf("-------------end of case 8:16-------------------\n");

    float t_row9[17] = {1.1, 2.2, 3.3, -4.4, 7, 1.1, 2.2, 3.3, -4.4, 7, 1.1, 2.2, 3.3, -4.4, 7, 13.8, -20.4};
    float y_xhat9[17] = {-1, 2, 3, 4, 98, -1, 2, 3, 4, 98, -1, 2, 3, 4, 98, -1.3, -3.3};

    asm volatile ("fence");
    ROCC_INSTRUCTION_SS(0, t_row9, y_xhat9, 0);   // passing two vector base       
    ROCC_INSTRUCTION_DS(0, sum_2, 17, 1);  // passing result pointer and the length
    asm volatile ("fence");

    a = dot_product_simple (t_row9, y_xhat9, 17);
    gtruth = *((uint32_t*)& (a));
    printf("result sum  %x \n", sum_2);

    printf("truth  %x  \n", gtruth);
    printf("-------------end of case 9:17-------------------\n");

    float t_row10[18] = {1.1, 2.2, 3.3, -4.4, 7, 1.1, 2.2, 3.3, -4.4, 7, 1.1, 2.2, 3.3, -4.4, 7, 13.8, -20.4, 38.5};
    float y_xhat10[18] = {-1, 2, 3, 4, 98, -1, 2, 3, 4, 98, -1, 2, 3, 4, 98, -1.3, -3.3, 54};

    asm volatile ("fence");
    ROCC_INSTRUCTION_SS(0, t_row10, y_xhat10, 0);   // passing two vector base       
    ROCC_INSTRUCTION_DS(0, sum_2, 18, 1);  // passing result pointer and the length
    asm volatile ("fence");

    a = dot_product_simple (t_row10, y_xhat10, 18);
    gtruth = *((uint32_t*)& (a));
    printf("result sum  %x \n", sum_2);

    printf("truth  %x  \n", gtruth);
    printf("-------------end of case 10:18-------------------\n");

    float t_row11[19] = {1.1, 2.2, 3.3, -4.4, 7, 1.1, 2.2, 3.3, -4.4, 7, 1.1, 2.2, 3.3, -4.4, 7, 13.8, -20.4, 38.5, -0.3};
    float y_xhat11[19] = {-1, 2, 3, 4, 98, -1, 2, 3, 4, 98, -1, 2, 3, 4, 98, -1.3, -3.3, 54, 320};

    asm volatile ("fence");
    ROCC_INSTRUCTION_SS(0, t_row11, y_xhat11, 0);   // passing two vector base       
    ROCC_INSTRUCTION_DS(0, sum_2, 19, 1);  // passing result pointer and the length
    asm volatile ("fence");

    a = dot_product_simple (t_row11, y_xhat11, 19);
    gtruth = *((uint32_t*)& (a));
    printf("result sum  %x \n", sum_2);

    printf("truth  %x  \n", gtruth);
    printf("-------------end of case 11:19-------------------\n");

    float t_row12[20] = {1.1, 2.2, 3.3, -4.4, 7, 1.1, 2.2, 3.3, -4.4, 7, 1.1, 2.2, 3.3, -4.4, 7, 13.8, -20.4, 38.5, -0.3, 33};
    float y_xhat12[20] = {-1, 2, 3, 4, 98, -1, 2, 3, 4, 98, -1, 2, 3, 4, 98, -1.3, -3.3, 54, -50.4};

    asm volatile ("fence");
    ROCC_INSTRUCTION_SS(0, t_row12, y_xhat12, 0);   // passing two vector base       
    ROCC_INSTRUCTION_DS(0, sum_2, 20, 1);  // passing result pointer and the length
    asm volatile ("fence");

    a = dot_product_simple (t_row12, y_xhat12, 20);
    gtruth = *((uint32_t*)& (a));
    printf("result sum  %x \n", sum_2);

    printf("truth  %x  \n", gtruth);
    printf("-------------end of case 12:20-------------------\n");
    return 0;
}
