#ifndef _HEAT_H
#define _HEAT_H

#include <stdint.h>

#define DEV_NAME   "/dev/heat_pwm"

#define HEAT_PWM_CMD_SET_DUTY         0x01
#define HEAT_PWM_CMD_SET_PERIOD       0x02
#define HEAT_PWM_CMD_SET_BOTH         0x03
#define HEAT_PWM_CMD_ENABLE           0x04
#define HEAT_PWM_CMD_DISABLE          0x05

#ifdef __cplusplus
extern "C" {
#endif

int heat_pwm_init();
void heat_pwm_set_duty(uint8_t duty);
int heat_pwm_deinit();

#ifdef __cplusplus
}
#endif

#endif