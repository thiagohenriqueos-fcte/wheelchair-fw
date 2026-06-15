#ifndef WHEELCHAIR_MOTOR_PWM_H
#define WHEELCHAIR_MOTOR_PWM_H

#include "esp_err.h"

esp_err_t motor_pwm_init(void);
esp_err_t motor_pwm_set_left(float command);
esp_err_t motor_pwm_set_right(float command);
esp_err_t motor_pwm_stop_all(void);

#endif
