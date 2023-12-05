#include "rocc.h"
#include "bitcasts.h"

#include <assert.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

// computes (t^T)y
#define ACC_SIZE 12

bool ROCC_dot(float *TMat_row_head, float *yVec_addr, int t_len, int y_len, float *sum)
{
    int N_per_fold = ACC_SIZE;
    int folds = t_len / N_per_fold ;
    int remainder = t_len % N_per_fold;

    int size_to_ROCC;

    float* mat_leftover; // array holding matrix row
    float* y_xhat_leftover; // array holding y and xHat

    float* y_xhat_concat; // array holding y and xHat

    uint32_t tmp = 0;
    float tmp_f;

    float* vec_head_1;
    float* vec_head_2;


    // Step 1:
    // Make sure dimension of T matrix match with input vector and state vector
    if (t_len != y_len){
        printf("Error!"); 
        return false;
    }
    else {
        // Step 2:
        // concat Y and XHat together
        y_xhat_concat = yVec_addr;
        

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

// computes x = Ay
bool ROCC_gemv(float *TMat_head, float *yVec_addr, int t_col_len, int t_row_len, int y_len, float *u_nextX_vector) 
{
    float* current_row = TMat_head;
    float* y_xhat_concat ; // array holding y and xHat

    // Step 1:
    // Make sure dimension of T matrix match with input vector and state vector
    if (t_row_len != y_len){
        printf("Error!"); 
        return false;
    }else{
        // Step 2:
        // Allocate the result vector
        y_xhat_concat = yVec_addr;

        for (int i = 0; i < t_col_len; i++) {
            current_row = TMat_head + t_row_len * i; 
            //ROCC_VecDotVec(current_row, yVec_addr, xHat_addr, t_row_len, y_len, x_len, &u_nextX_vector[i]);
            ROCC_dot(current_row, y_xhat_concat, t_row_len, y_len, &u_nextX_vector[i]);
            
        }
        return true;
    }
}

float A_d[16] = {
    1.0f, 0.00999f, 0.0000999f, 0.000000332f,
    0.0f, 0.998f, 0.01998f, 0.00009938f,
    0.0f, -0.000004997f, 1.0003f, 0.01f,
    0.0f, -0.000999f, 0.059996f, 1.0003f
};

float B_d[4] = {
    0.00000993f,
    0.001998f,
    0.000004997f,
    0.0009991f
};

float K_d[4] = {
    -840.798, -626.993, 3470.245, 1463.734
};

void pendulum_test(int its) {
    //position, velocity, angle, angular velocity
    float state[4] = {0.5f, 0.0f, 0.0f, 0.0f};
    float next_state[4];
    float u;
    for (int i = 0; i < its; ++i) {
        printf("step %d: %f, %f, %f, %f\n", i, state[0], state[1], state[2], state[3]);
        ROCC_dot(K_d, state, 4, 4, &u); // u = K_d @ x
        ROCC_gemv(A_d, state, 4, 4, 4, &next_state[0]); // A_d @ x

        state[0] = next_state[0] + u * B_d[0];
        state[1] = next_state[1] + u * B_d[1];
        state[2] = next_state[2] + u * B_d[2];
        state[3] = next_state[3] + u * B_d[3];
    }
}