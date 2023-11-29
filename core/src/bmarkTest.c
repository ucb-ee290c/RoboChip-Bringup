#include "rocc.h"
#include "bitcasts.h"

#include <assert.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#define ACC_SIZE 12

#define N_bmark 6 // number of internal states
#define P_bmark 4 // number of sensor inputs
#define M_bmark 2 // number of controller (motors, actuators) outputs

// T matrix has a dimension of (N+M) rows, each row has (P+N) elements
float T_matrix_bmark[((N_bmark+M_bmark)*(P_bmark+N_bmark))] =
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

float Y_data_matrix_bmark[P_bmark] =
{
  46, 77,  7, -86.1
};
// size 4


float X_data_matrix_bmark[N_bmark] =
{
  -93, 10.23, 3230.33, -9.6,  -5, 90.232
};
// size 6

float U_data_matrix_bmark[M_bmark] =
{
  23.32, -203.88
};

void *array_concat_bmark(const void *a, size_t an,
               const void *b, size_t bn, size_t s)
{
    char *p = malloc(s * (an + bn));
    memcpy(p, a, an*s);
    memcpy(p + an*s, b, bn*s);
    return p;
}

float dot_product_bmark(float v[], float u[], int n)
{
    float result = 0.0;
    for (int i = 0; i < n; i++)
        result += v[i]*u[i];
    return result;
}

float get_groundtruth_vecDvec_bmark(float *TMat_row_head, float *yVec_addr, float *xHat_vec_addr, int y_len, int x_len)
{
    float* y_xhat_concat ; // array holding y and xHat
    y_xhat_concat = array_concat_bmark(yVec_addr, y_len, xHat_vec_addr, x_len, sizeof(float));
    float sum;
    sum = dot_product_bmark(TMat_row_head, y_xhat_concat, y_len + x_len);
    return sum;
}

float* get_groundtruth_MatDvec_bmark(float *TMat_head, float *yVec_addr, float *xHat_vec_addr, int y_len, int x_len, int t_col_len, int t_row_len)
{
    float* y_xhat_concat ; // array holding y and xHat
    float* u_xhat_concat ; // result
    float* current_row = TMat_head;

    y_xhat_concat = array_concat_bmark(yVec_addr, y_len, xHat_vec_addr, x_len, sizeof(float));
    u_xhat_concat = malloc(t_col_len * sizeof(float));

    for (int i = 0; i < t_col_len; i++) {
        current_row = TMat_head + t_row_len * i; 
        u_xhat_concat[i] = dot_product_bmark(current_row, y_xhat_concat, y_len + x_len);
       
    }
    return u_xhat_concat;
}

void print_float_arr_hex_bmark(float arr_head[], int length){
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
* @param float* sum
* Note: 1. dimension should agree, meaning that dim(y)+dim(x)
*          should be equal to dim(t_row)
*       2. param sum is used to return one dot product result.
*          it is a pointer, please pass in something like &sum
*/
bool ROCC_VecDotVec_bmark(float *TMat_row_head, float *yVec_addr, float *xHat_vec_addr, int t_len, int y_len, int x_len, float *sum)
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
        y_xhat_concat = array_concat_bmark(yVec_addr, y_len, xHat_vec_addr, x_len, sizeof(float));

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
bool ROCC_MatDotVec_bmark(float *TMat_head, float *yVec_addr, float *xHat_addr, int t_col_len, int t_row_len, int y_len, int x_len, float *u_nextX_vector) 
{
    //float* u_nextX_vector;
    float* current_row = TMat_head;
    // Step 1:
    // Make sure dimension of T matrix match with input vector and state vector
    if (t_row_len != y_len + x_len){
        printf("Error!"); 
        return false;
    }else{
        // Step 2:
        // Allocate the result vector
        //u_nextX_vector = malloc(t_col_len * sizeof(float));

        for (int i = 0; i < t_col_len; i++) {
            current_row = TMat_head + t_row_len * i; 
            ROCC_VecDotVec_bmark(current_row, yVec_addr, xHat_addr, t_row_len, y_len, x_len, &u_nextX_vector[i]);
            //TMat_head += sizeof(float) * t_row_len; //Move across T rows, changing the vector head
            
        }
        return true;
    }
}


int bmark_test() {
    
    uint32_t gtruth;
    float sum = 0;
    float a = 0;
    float T_row_data1[16] = {-660.43, 85.23, 16, 42, 73, 87, 95, 92,  0, 58, 91, 91, 76,  3, 59, 67};
    float Y_data1[8] ={64, 50, 65, 37, 41, -34.32, -34.32, -1.4223};
    float X_data1[8] ={12,  3, 55, 30,  3.553, -95.1, 53, 38};

    uint32_t start_time;
    uint32_t end_time;


    start_time = HAL_CLINT_getTime();
    asm volatile ("fence");
    ROCC_INSTRUCTION_SS(0, T_row_data1, T_row_data1, 3);   // fake instr, to measure cycles    
    asm volatile ("fence");
    a = get_groundtruth_vecDvec_bmark(T_row_data1, Y_data1, X_data1, 8, 8);
    asm volatile ("fence");
    ROCC_INSTRUCTION_SS(0, T_row_data1, T_row_data1, 4);   // fake instr, to measure cycles    
    asm volatile ("fence");
    gtruth = *((uint32_t*)& (a));
    end_time = HAL_CLINT_getTime();
    printf("truth  %x  \n", gtruth);
    printf("Total time for calculation of default: %d \n", end_time - start_time);

    sum = 0;
    start_time = HAL_CLINT_getTime();
    asm volatile ("fence");
    ROCC_INSTRUCTION_SS(0, T_row_data1, T_row_data1, 3);   // fake instr, to measure cycles    
    asm volatile ("fence");
    ROCC_VecDotVec_bmark(T_row_data1, Y_data1, X_data1, 16, 8, 8, &sum);
    asm volatile ("fence");
    ROCC_INSTRUCTION_SS(0, T_row_data1, T_row_data1, 4);   // fake instr, to measure cycles    
    asm volatile ("fence");
    gtruth = *((uint32_t*)& (sum));
    end_time = HAL_CLINT_getTime();
    printf("result  %x  \n", gtruth);
    printf("Total time for calculation of LQR: %d \n", end_time - start_time);

    

    //-----------------------------Mat_vec test ------------------------------
    printf(" \n***** case matrix_vec start ****** \n");
    // get ground truth printed

    // from constMatrix.h
    
    start_time = HAL_CLINT_getTime();
    asm volatile ("fence");
    ROCC_INSTRUCTION_SS(0, T_row_data1, T_row_data1, 3);   // fake instr, to measure cycles    
    asm volatile ("fence");
    float* truth = get_groundtruth_MatDvec_bmark(T_matrix_bmark, Y_data_matrix_bmark, X_data_matrix_bmark, P_bmark, N_bmark, N_bmark+M_bmark, P_bmark+N_bmark);
    asm volatile ("fence");
    ROCC_INSTRUCTION_SS(0, T_row_data1, T_row_data1, 4);   // fake instr, to measure cycles    
    asm volatile ("fence");
    printf("Truth: \n");
    print_float_arr_hex_bmark(truth, N_bmark+M_bmark);
    printf("Total time for calculation of default: %d \n", end_time - start_time);
    end_time = HAL_CLINT_getTime();

    start_time = HAL_CLINT_getTime();
    float u_nextX_vector[(N_bmark+M_bmark)];
    asm volatile ("fence");
    ROCC_INSTRUCTION_SS(0, T_row_data1, T_row_data1, 3);   // fake instr, to measure cycles    
    asm volatile ("fence");
    ROCC_MatDotVec_bmark(T_matrix_bmark, Y_data_matrix_bmark, X_data_matrix_bmark, N_bmark+M_bmark, P_bmark+N_bmark, P_bmark, N_bmark, u_nextX_vector);
    asm volatile ("fence");
    ROCC_INSTRUCTION_SS(0, T_row_data1, T_row_data1, 4);   // fake instr, to measure cycles    
    asm volatile ("fence");
    end_time = HAL_CLINT_getTime();
    printf("Result: \n");
    printf("Total time for calculation of LQR: %d \n", end_time - start_time);
    print_float_arr_hex_bmark(u_nextX_vector, N_bmark+M_bmark);

    return 0;
}
