#include "controlo_atitude.h"
#if defined(_WIN32)
#define API __declspec(dllexport)
#else
#define API __attribute__((visibility("default")))
#endif
API void gs_init_controladores(void) { init_controladores(); }
API void gs_reset_controladores(void) { reset_controladores(); }
API float gs_malha_externa_yaw(float erro_posicao, float dt) { return calcular_malha_externa_yaw(erro_posicao, dt); }
API float gs_malha_intermedia_yaw(float desejada, float real, float dt) { return calcular_malha_intermedia_yaw(desejada, real, dt); }
API float gs_damping_roll_pitch(float real, float ganho) { return calcular_damping_roll_pitch(real, ganho); }
