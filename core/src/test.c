#include "rocc.h"
#include "bitcasts.h"

#include <assert.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

// check the parameter "n" inside /generators/lqr-rocc/src/main/scala/lqrRoCC.scala
#define ACC_SIZE 12

#define N 6 // number of internal states
#define P 4 // number of sensor inputs
#define M 2 // number of controller (motors, actuators) outputs

#define DATA_SIZE 31 // M+P

#define Y_SIZE 11 // P
#define X_SIZE 20 // N

// T matrix has a dimension of (N+M) rows, each row has (P+N) elements
float T_matrix[((N+M)*(P+N))] =
{
  85.23, 16, 42, 73, 87, 95, 92,  0, 58, 91,   
  76,  3, 59, 67, 78, 51, 66, 62, -80.19, 86, 
  98.343, -60, 15.3,  -0.23,  6, 76, 78, 22, 44, 72, 
  85.23, 16, 42, 73, 87, 95, 92,  0, 58, 91,   
  76,  3, 59, 67, 78, 51, 66, 62, -80.19, 86, 
  98.343, -60, 15.3,  -0.23,  6, 76, 78, 22, 44, 72, 
  20, 45, 97, 61.34, 28, -92, -84, -43, -59, 12, 
  20, 45, -97.98, 61, -28, 92, 84, 43, 59, 12, 
};
// 8 * 10

float Y_data_matrix[P] =
{
  46, 77,  7, -86.1
};
// size 4


float X_data_matrix[N] =
{
  -93, 10.23, 3230.33, -9.6,  -5, 90.232
};
// size 6

float U_data_matrix[M] =
{
  23.32, -203.88
};

float T_row_data[DATA_SIZE] =
{
  85.23, 16, 42, 73, 87, 95, 92,  0, 58, 91, 91, 76,  3, 59, 67, 78, 51, 66, 62, -80.19, 86, 21, 96, 55, 53, 27, 64, 83, 25, 13, 58
};

float Y_data[Y_SIZE] =
{
  46, 77,  7, -86.1, 74, 64, 50, 65, 37, 41, -34.32
};


float X_data[X_SIZE] =
{
  46, 77,  7, -86.1, 74, 64, 50, 65, 37, 41, -34.32, 43, 93, 10, 30, 96,  5, 90, 86, 79
};

void *array_concat(const void *a, size_t an,
               const void *b, size_t bn, size_t s)
{
    char *p = malloc(s * (an + bn));
    memcpy(p, a, an*s);
    memcpy(p + an*s, b, bn*s);
    return p;
}

float dot_product(float v[], float u[], int n)
{
    float result = 0.0;
    for (int i = 0; i < n; i++)
        result += v[i]*u[i];
    return result;
}

float get_groundtruth_vecDvec(float *TMat_row_head, float *yVec_addr, float *xHat_vec_addr, int y_len, int x_len)
{
    float* y_xhat_concat ; // array holding y and xHat
    y_xhat_concat = array_concat(yVec_addr, y_len, xHat_vec_addr, x_len, sizeof(float));
    float sum;
    sum = dot_product(TMat_row_head, y_xhat_concat, y_len + x_len);
    return sum;
}

float* get_groundtruth_MatDvec(float *TMat_head, float *yVec_addr, float *xHat_vec_addr, int y_len, int x_len, int t_col_len, int t_row_len)
{
    float* y_xhat_concat ; // array holding y and xHat
    float* u_xhat_concat ; // result
    float* current_row = TMat_head;

    y_xhat_concat = array_concat(yVec_addr, y_len, xHat_vec_addr, x_len, sizeof(float));
    u_xhat_concat = malloc(t_col_len * sizeof(float));

    for (int i = 0; i < t_col_len; i++) {
        current_row = TMat_head + t_row_len * i; 
        u_xhat_concat[i] = dot_product(current_row, y_xhat_concat, y_len + x_len);
       
    }
    return u_xhat_concat;
}

void print_float_arr_hex(float arr_head[], int length){
    uint32_t gtruth;
    
    for (int i=0; i < length; i++) {
        gtruth = *((uint32_t*)& (arr_head[i]));
        printf("vec[%d]:  %x  \n", i, gtruth);
    }
}

/**
* Perform a dot product between one row of T matrix and 
* the y_xHat vector
* @param float* TMat_row_head, yVec_addr, xHat_vec_addr
* @param int t_len, y_len, x_len
* @param bool do_concat 
* @param float* sum
* Note: 1. dimension should agree, meaning that dim(y)+dim(x)
*          should be equal to dim(t_row)
*       2. param sum is used to return one dot product result.
*          it is a pointer, please pass in something like &sum
*/
bool ROCC_VecDotVec(float *TMat_row_head, float *yVec_addr, float *xHat_vec_addr, int t_len, int y_len, int x_len, float *sum, bool do_concat)
{
    int N_per_fold = ACC_SIZE;
    int folds = t_len / N_per_fold ;
    int remainder = t_len % N_per_fold;

    int size_to_ROCC;

    float* mat_leftover; // array holding matrix row
    float* y_xhat_leftover ; // array holding y and xHat

    float* y_xhat_concat ; // array holding y and xHat

    uint32_t tmp = 0;
    float tmp_f;

    float* vec_head_1;
    float* vec_head_2;


    // Step 1:
    // Make sure dimension of T matrix match with input vector and state vector
    if (t_len != y_len + x_len){
        printf("Error!"); 
        return false;
    }
    else {
        // Step 2:
        // concat Y and XHat together
        if(do_concat){
            y_xhat_concat = array_concat(yVec_addr, y_len, xHat_vec_addr, x_len, sizeof(float));
        }else{
            y_xhat_concat = yVec_addr;
        }
        

        // Step 3:
        // add another fold.
        // for example, vector of size 17 need to be sent to accelerator in 2 fold
        // 0-16 first, 17 second
        if(remainder>0){                               
            folds = folds + 1; // dont forget that!
        }

        // Step 4: sending vector to RoCC by Fold
        for(int i = 0; i < folds; i++){
            vec_head_1 = TMat_row_head + i*N_per_fold; 
            vec_head_2 = y_xhat_concat + i*N_per_fold;
            if( (i == folds-1) && (remainder)){
                //vec_head_1 = &mat_leftover[0];
                //vec_head_2 = &y_xhat_leftover[0];
                size_to_ROCC = remainder;
            }else{
                size_to_ROCC = N_per_fold;
            }
            asm volatile ("fence");
            ROCC_INSTRUCTION_SS(0, vec_head_1, vec_head_2, 0);   // passing two vector base       
            ROCC_INSTRUCTION_DS(0, tmp, size_to_ROCC, 1);  // passing result pointer and the length
            asm volatile ("fence");  
            
            // Crucial!!
            // need to convert tmp from Uint back to float before adding to sum
            tmp_f = fp32_from_bits(tmp); 
            *sum = *sum + tmp_f;
           
        }
        return true;
    }
}

/**
* Perform a dot product between T matrix and the y_xHat vector
* @param float* TMat_head, yVec_addr, xHat_vec_addr
* @param int t_col_len, t_row_len, y_len, x_len
* @param float* u_nextX_vector
* Note: 1. dimension should agree, meaning that y_len + x_len
*          should be equal to t_row_len
*       2. u_nextX_vector is the head of return vector
*/
bool ROCC_MatDotVec(float *TMat_head, float *yVec_addr, float *xHat_addr, int t_col_len, int t_row_len, int y_len, int x_len, float *u_nextX_vector) 
{
    float* current_row = TMat_head;
    float* y_xhat_concat ; // array holding y and xHat

    // Step 1:
    // Make sure dimension of T matrix match with input vector and state vector
    if (t_row_len != y_len + x_len){
        printf("Error!"); 
        return false;
    }else{
        // Step 2:
        // Allocate the result vector
        y_xhat_concat = array_concat(yVec_addr, y_len, xHat_addr, x_len, sizeof(float));

        for (int i = 0; i < t_col_len; i++) {
            current_row = TMat_head + t_row_len * i; 
            //ROCC_VecDotVec(current_row, yVec_addr, xHat_addr, t_row_len, y_len, x_len, &u_nextX_vector[i]);
            ROCC_VecDotVec(current_row, y_xhat_concat, xHat_addr, t_row_len, y_len, x_len, &u_nextX_vector[i], false);
            
        }
        return true;
    }
}


int main_test() {
    uint32_t start_time;
    uint32_t end_time;
    
    uint32_t gtruth;
    start_time = HAL_CLINT_getTime();
    float a = get_groundtruth_vecDvec(T_row_data, Y_data, X_data, Y_SIZE, X_SIZE);
    gtruth = *((uint32_t*)& (a));
    end_time = HAL_CLINT_getTime();
    printf("truth  %x  \n", gtruth);
    printf("Total time for calculation of default: %d \n", end_time - start_time);

    float sum = 0;
    start_time = HAL_CLINT_getTime();
    ROCC_VecDotVec(T_row_data, Y_data, X_data, DATA_SIZE, Y_SIZE, X_SIZE, &sum, true);
    gtruth = *((uint32_t*)& (sum));
    end_time = HAL_CLINT_getTime();
    printf("result  %x  \n", gtruth);
    printf("Total time for calculation of LQR: %d \n", end_time - start_time);

    printf(" ***** case 1 end : vec[31] ****** \n");


    float T_row_data1[32] = {-660.43, 85.23, 16, 42, 73, 87, 95, 92,  0, 58, 91, 91, 76,  3, 59, 67, 78, 51, 66, 62, -80.19, 86, 21, 96, 55, 53, 27, 64, 83, 25, 13, 58};
    float Y_data1[13] ={46, 77,  7, -86.1, 74, 64, 50, 65, 37, 41, -34.32, -34.32, -1.4223};
    float X_data1[19] ={12,  3, 55, 30,  3.553, -95.1, 53, 38, 75, 54, 57, 81, 92, 29, 56, 76, 17, 22, 26};
    start_time = HAL_CLINT_getTime();
    a = get_groundtruth_vecDvec(T_row_data1, Y_data1, X_data1, 13, 19);
    gtruth = *((uint32_t*)& (a));
    end_time = HAL_CLINT_getTime();
    printf("truth  %x  \n", gtruth);
    printf("Total time for calculation of default: %d \n", end_time - start_time);

    sum = 0;
    start_time = HAL_CLINT_getTime();
    ROCC_VecDotVec(T_row_data1, Y_data1, X_data1, 32, 13, 19, &sum, true);
    gtruth = *((uint32_t*)& (sum));
    end_time = HAL_CLINT_getTime();
    printf("result  %x  \n", gtruth);
    printf("Total time for calculation of LQR: %d \n", end_time - start_time);

    printf(" ***** case 2 end : vec[32] ****** \n");


    float T_row_data2[6] = {-660.43, 85.23, 16, 42, 73, 87};
    float Y_data2[2] ={46, 77.343};
    float X_data2[4] ={3.553, -95.1, 53, 38};
    start_time = HAL_CLINT_getTime();
    a = get_groundtruth_vecDvec(T_row_data2, Y_data2, X_data2, 2, 4);
    gtruth = *((uint32_t*)& (a));
    end_time = HAL_CLINT_getTime();
    printf("truth  %x  \n", gtruth);
    printf("Total time for calculation of default: %d \n", end_time - start_time);

    sum = 0;
    start_time = HAL_CLINT_getTime();
    ROCC_VecDotVec(T_row_data2, Y_data2, X_data2, 6, 2, 4, &sum, true);
    gtruth = *((uint32_t*)& (sum));
    end_time = HAL_CLINT_getTime();
    printf("result  %x  \n", gtruth);
    printf("Total time for calculation of LQR: %d \n", end_time - start_time);

    printf(" ***** case 3 end ****** : vec[6] \n");
    

    //-----------------------------Mat_vec test ------------------------------
    printf(" \n***** case matrix_vec start ****** \n");
    // get ground truth printed

    // from constMatrix.h
    start_time = HAL_CLINT_getTime();
    float* truth = get_groundtruth_MatDvec(T_matrix, Y_data_matrix, X_data_matrix, P, N, N+M, P+N);
    end_time = HAL_CLINT_getTime();
    printf("Truth: ");
    printf("Total time for calculation of default: %d \n", end_time - start_time);
    print_float_arr_hex(truth, N+M);

    float u_nextX_vector[(N+M)];
    start_time = HAL_CLINT_getTime();
    ROCC_MatDotVec(T_matrix, Y_data_matrix, X_data_matrix, N+M, P+N, P, N, u_nextX_vector);
    end_time = HAL_CLINT_getTime();
    printf("Result: ");
    printf("Total time for calculation of LQR: %d \n", end_time - start_time);
    print_float_arr_hex(u_nextX_vector, N+M);

    return 0;
}
