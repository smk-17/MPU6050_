/*
 * Kalman.h
 *
 *  Created on: 22-Sept-2026
 *      Author: manoj_kumar
 */

#ifndef INC_KALMAN_H_
#define INC_KALMAN_H_

typedef struct
{
    float angle;
    float bias;

    float P[2][2];

    float Q_angle;
    float Q_bias;
    float R_measure;

} KALMAN_T;


void Kalman_Init(KALMAN_T *k);

float Kalman_Update(
    KALMAN_T *k,
    float newAngle,
    float newRate,
    float dt
);


#endif /* INC_KALMAN_H_ */
