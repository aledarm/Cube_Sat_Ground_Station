#include "controlo_atitude.h"
#include <math.h>

/* Source integrated from team branch feature/attitude-rate-control-level0. */
static PID_Controller pid_posicao_yaw;
static PID_Controller pid_velocidade_yaw;

void init_controladores(void) {
    pid_posicao_yaw.kp = 2.0f;
    pid_posicao_yaw.ki = 0.05f;
    pid_posicao_yaw.kd = 0.1f;
    pid_posicao_yaw.integral = 0.0f;
    pid_posicao_yaw.erro_anterior = 0.0f;
    pid_posicao_yaw.i_max = 5.0f;

    pid_velocidade_yaw.kp = 0.5f;
    pid_velocidade_yaw.ki = 0.1f;
    pid_velocidade_yaw.kd = 0.02f;
    pid_velocidade_yaw.integral = 0.0f;
    pid_velocidade_yaw.erro_anterior = 0.0f;
    pid_velocidade_yaw.i_max = MAX_TORQUE_COMMAND * 0.5f;
}

void reset_controladores(void) {
    pid_posicao_yaw.integral = 0.0f;
    pid_posicao_yaw.erro_anterior = 0.0f;
    pid_velocidade_yaw.integral = 0.0f;
    pid_velocidade_yaw.erro_anterior = 0.0f;
}

float calcular_malha_externa_yaw(float erro_posicao, float dt) {
    if (dt <= 0.0001f) return 0.0f;
    pid_posicao_yaw.integral += erro_posicao * dt;
    if (pid_posicao_yaw.integral > pid_posicao_yaw.i_max) pid_posicao_yaw.integral = pid_posicao_yaw.i_max;
    else if (pid_posicao_yaw.integral < -pid_posicao_yaw.i_max) pid_posicao_yaw.integral = -pid_posicao_yaw.i_max;

    float derivada = (erro_posicao - pid_posicao_yaw.erro_anterior) / dt;
    pid_posicao_yaw.erro_anterior = erro_posicao;
    float vel_angular_desejada = (pid_posicao_yaw.kp * erro_posicao) +
                                 (pid_posicao_yaw.ki * pid_posicao_yaw.integral) +
                                 (pid_posicao_yaw.kd * derivada);
    if (vel_angular_desejada > MAX_TARGET_VELOCITY) vel_angular_desejada = MAX_TARGET_VELOCITY;
    else if (vel_angular_desejada < -MAX_TARGET_VELOCITY) vel_angular_desejada = -MAX_TARGET_VELOCITY;
    return vel_angular_desejada;
}

float calcular_malha_intermedia_yaw(float vel_angular_desejada, float vel_angular_real, float dt) {
    if (dt <= 0.0001f) return 0.0f;
    float erro_vel = vel_angular_desejada - vel_angular_real;
    float termo_p = pid_velocidade_yaw.kp * erro_vel;
    float derivada = (erro_vel - pid_velocidade_yaw.erro_anterior) / dt;
    float termo_d = pid_velocidade_yaw.kd * derivada;
    float integracao_provisoria = pid_velocidade_yaw.integral + (erro_vel * dt);
    float termo_i = pid_velocidade_yaw.ki * integracao_provisoria;
    float comando_torque = termo_p + termo_i + termo_d;
    if (comando_torque > MAX_TORQUE_COMMAND) comando_torque = MAX_TORQUE_COMMAND;
    else if (comando_torque < -MAX_TORQUE_COMMAND) comando_torque = -MAX_TORQUE_COMMAND;
    else pid_velocidade_yaw.integral = integracao_provisoria;
    pid_velocidade_yaw.erro_anterior = erro_vel;
    return comando_torque;
}

float calcular_damping_roll_pitch(float vel_angular_real, float ganho_damping) {
    return -ganho_damping * vel_angular_real;
}
