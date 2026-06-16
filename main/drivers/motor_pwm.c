#include "motor_pwm.h"

#include <stdbool.h>

#include "driver/mcpwm_prelude.h"

/* GPIO assignments for IBT-2 H-bridge inputs */
#define LEFT_RPWM_GPIO   10
#define LEFT_LPWM_GPIO   11
#define RIGHT_RPWM_GPIO  12
#define RIGHT_LPWM_GPIO  13

/*
 * 20 MHz resolution -> period_ticks = 20 000 000 / 25 000 = 800.
 * Gives 0.125 % duty-cycle granularity at 25 kHz, well within IBT-2 requirements.
 */
#define MCPWM_RESOLUTION_HZ  20000000U
#define PWM_FREQ_HZ          25000U
#define MCPWM_PERIOD_TICKS   (MCPWM_RESOLUTION_HZ / PWM_FREQ_HZ)

typedef struct {
    mcpwm_timer_handle_t timer;
    mcpwm_oper_handle_t  oper;
    mcpwm_cmpr_handle_t  cmpr_fwd;  /* comparator for forward/RPWM output */
    mcpwm_cmpr_handle_t  cmpr_rev;  /* comparator for reverse/LPWM output */
    mcpwm_gen_handle_t   gen_fwd;
    mcpwm_gen_handle_t   gen_rev;
} motor_channel_t;

static motor_channel_t s_left;
static motor_channel_t s_right;
static bool            s_initialized;

static esp_err_t channel_init(motor_channel_t *ch, int gpio_fwd, int gpio_rev)
{
    esp_err_t ret;

    const mcpwm_timer_config_t timer_cfg = {
        .group_id      = 0,
        .clk_src       = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = MCPWM_RESOLUTION_HZ,
        .count_mode    = MCPWM_TIMER_COUNT_MODE_UP,
        .period_ticks  = MCPWM_PERIOD_TICKS,
    };
    ret = mcpwm_new_timer(&timer_cfg, &ch->timer);
    if (ret != ESP_OK) return ret;

    const mcpwm_operator_config_t oper_cfg = { .group_id = 0 };
    ret = mcpwm_new_operator(&oper_cfg, &ch->oper);
    if (ret != ESP_OK) return ret;

    ret = mcpwm_operator_connect_timer(ch->oper, ch->timer);
    if (ret != ESP_OK) return ret;

    /* Update compare value at timer-zero to avoid glitches during duty changes */
    const mcpwm_comparator_config_t cmpr_cfg = {
        .flags.update_cmp_on_tez = true,
    };
    ret = mcpwm_new_comparator(ch->oper, &cmpr_cfg, &ch->cmpr_fwd);
    if (ret != ESP_OK) return ret;
    ret = mcpwm_new_comparator(ch->oper, &cmpr_cfg, &ch->cmpr_rev);
    if (ret != ESP_OK) return ret;

    mcpwm_comparator_set_compare_value(ch->cmpr_fwd, 0);
    mcpwm_comparator_set_compare_value(ch->cmpr_rev, 0);

    const mcpwm_generator_config_t gen_cfg_fwd = { .gen_gpio_num = gpio_fwd };
    ret = mcpwm_new_generator(ch->oper, &gen_cfg_fwd, &ch->gen_fwd);
    if (ret != ESP_OK) return ret;

    const mcpwm_generator_config_t gen_cfg_rev = { .gen_gpio_num = gpio_rev };
    ret = mcpwm_new_generator(ch->oper, &gen_cfg_rev, &ch->gen_rev);
    if (ret != ESP_OK) return ret;

    /* Standard up-count PWM: set high at counter zero, set low at compare match */
    ret = mcpwm_generator_set_actions_on_timer_event(ch->gen_fwd,
        MCPWM_GEN_TIMER_EVENT_ACTION(
            MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH),
        MCPWM_GEN_TIMER_EVENT_ACTION_END());
    if (ret != ESP_OK) return ret;
    ret = mcpwm_generator_set_actions_on_compare_event(ch->gen_fwd,
        MCPWM_GEN_COMPARE_EVENT_ACTION(
            MCPWM_TIMER_DIRECTION_UP, ch->cmpr_fwd, MCPWM_GEN_ACTION_LOW),
        MCPWM_GEN_COMPARE_EVENT_ACTION_END());
    if (ret != ESP_OK) return ret;

    ret = mcpwm_generator_set_actions_on_timer_event(ch->gen_rev,
        MCPWM_GEN_TIMER_EVENT_ACTION(
            MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH),
        MCPWM_GEN_TIMER_EVENT_ACTION_END());
    if (ret != ESP_OK) return ret;
    ret = mcpwm_generator_set_actions_on_compare_event(ch->gen_rev,
        MCPWM_GEN_COMPARE_EVENT_ACTION(
            MCPWM_TIMER_DIRECTION_UP, ch->cmpr_rev, MCPWM_GEN_ACTION_LOW),
        MCPWM_GEN_COMPARE_EVENT_ACTION_END());
    if (ret != ESP_OK) return ret;

    /* Force both outputs low immediately — safe state before motor is used */
    mcpwm_generator_set_force_level(ch->gen_fwd, 0, true);
    mcpwm_generator_set_force_level(ch->gen_rev, 0, true);

    ret = mcpwm_timer_enable(ch->timer);
    if (ret != ESP_OK) return ret;

    return mcpwm_timer_start_stop(ch->timer, MCPWM_TIMER_START_NO_STOP);
}

/*
 * Set one motor channel.  command is in [-1.0, +1.0].
 * Positive -> fwd (RPWM) active, rev (LPWM) forced low.
 * Negative -> rev (LPWM) active, fwd (RPWM) forced low.
 * Zero     -> both forced low.
 * RPWM and LPWM are never simultaneously active.
 */
static esp_err_t channel_set(motor_channel_t *ch, float cmd)
{
    if (cmd > 1.0f)  cmd = 1.0f;
    if (cmd < -1.0f) cmd = -1.0f;

    if (cmd > 0.0f) {
        const uint32_t duty = (uint32_t)(cmd * MCPWM_PERIOD_TICKS);
        mcpwm_comparator_set_compare_value(ch->cmpr_fwd, duty);
        mcpwm_generator_set_force_level(ch->gen_rev, 0, true);   /* hold rev at 0 */
        mcpwm_generator_set_force_level(ch->gen_fwd, -1, true);  /* release fwd to PWM */
    } else if (cmd < 0.0f) {
        const uint32_t duty = (uint32_t)(-cmd * MCPWM_PERIOD_TICKS);
        mcpwm_comparator_set_compare_value(ch->cmpr_rev, duty);
        mcpwm_generator_set_force_level(ch->gen_fwd, 0, true);   /* hold fwd at 0 */
        mcpwm_generator_set_force_level(ch->gen_rev, -1, true);  /* release rev to PWM */
    } else {
        mcpwm_generator_set_force_level(ch->gen_fwd, 0, true);
        mcpwm_generator_set_force_level(ch->gen_rev, 0, true);
    }
    return ESP_OK;
}

esp_err_t motor_pwm_init(void)
{
    esp_err_t ret;

    ret = channel_init(&s_left, LEFT_RPWM_GPIO, LEFT_LPWM_GPIO);
    if (ret != ESP_OK) return ret;

    ret = channel_init(&s_right, RIGHT_RPWM_GPIO, RIGHT_LPWM_GPIO);
    if (ret != ESP_OK) return ret;

    s_initialized = true;
    return ESP_OK;
}

esp_err_t motor_pwm_set_left(float command)
{
    if (!s_initialized) return ESP_ERR_INVALID_STATE;
    return channel_set(&s_left, command);
}

esp_err_t motor_pwm_set_right(float command)
{
    if (!s_initialized) return ESP_ERR_INVALID_STATE;
    return channel_set(&s_right, command);
}

esp_err_t motor_pwm_stop_all(void)
{
    if (!s_initialized) return ESP_ERR_INVALID_STATE;
    channel_set(&s_left, 0.0f);
    channel_set(&s_right, 0.0f);
    return ESP_OK;
}
