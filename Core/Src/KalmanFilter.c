/*
 * KalmanFilter.c
 *
 *  Created on: 14 Μαρ 2023
 *      Author: PANAGIOTIS
 */


float *KalmanFilter(float Ptemp,float input, float prev_input)
{
	static float Kalman_params[3];
	float H = 1.0;
    float Q = 0.005;
    float R = 0.0412;

	float K = (Ptemp*H)/(H*Ptemp*H + R);
	float Kalman_omni_out = K*(input - H*prev_input);
	float Pnew = (1-K*H)*Ptemp + Q;
	prev_input = Kalman_omni_out;
	Ptemp = Pnew;

	Kalman_params[0] = Kalman_omni_out;
	Kalman_params[1] = Ptemp;
	Kalman_params[2] = prev_input;
	return Kalman_params;
}
