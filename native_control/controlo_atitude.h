#ifndef CONTROLO_ATITUDE_H
#define CONTROLO_ATITUDE_H

#include <stdint.h>
#include <stdbool.h>

#define MAX_TORQUE_COMMAND   0.2f
#define MAX_TARGET_VELOCITY  1.0f

typedef struct {
    float kp;
    float ki;
    float kd;
    float integral;
    float erro_anterior;
    float i_max;
} PID_Controller;

void init_controladores(void);
void reset_controladores(void);
float calcular_malha_externa_yaw(float erro_posicao, float dt);
float calcular_malha_intermedia_yaw(float vel_angular_desejada, float vel_angular_real, float dt);
float calcular_damping_roll_pitch(float vel_angular_real, float ganho_damping);

#endif
