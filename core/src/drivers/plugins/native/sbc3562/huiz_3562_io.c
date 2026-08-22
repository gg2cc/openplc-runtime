/**
 * @file huiz_3562_io.c
 *
 * @brief HUIZ_3562 GPIO I/O Mapping Plugin for OpenPLC
 *
 * Maps SBC3562 hardware GPIO pins (via sysfs) to OpenPLC I/O buffers based on XML device descriptions:
 *
 *   Input pins (16-bit DI, sysfs GPIO numbers 487..502, active_low=TRUE):
 *     -> %IX0.0 .. %IX0.7  (bool_input[0][0..7], GPIO 487..494)
 *     -> %IX1.0 .. %IX1.7  (bool_input[1][0..7], GPIO 495..502)
 *
 *   Output pins (16-bit DO, sysfs GPIO numbers 471..486, active_low=FALSE):
 *     -> %QX0.0 .. %QX0.7  (bool_output[0][0..7], GPIO 471..478)
 *     -> %QX1.0 .. %QX1.7  (bool_output[1][0..7], GPIO 479..486)
 *
 *   Buzzer output pin (GPIO 507, active_low=TRUE):
 *     -> %QX2.0            (bool_output[2][0], GPIO 507)
 *
 *   LED Output pins (/sys/class/leds):
 *     -> %QX3.0            (bool_output[3][0], LED1 brightness)
 *     -> %QX3.1            (bool_output[3][1], LED2 brightness)
 *
 * GPIO access is performed via /sys/class/gpio (sysfs interface) and /sys/class/leds.
 *
 * @version V1.0
 * @date 2026-08-21
 **/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <pthread.h>

#include "../plugin_logger.h"
#include "../../../../lib/iec_types.h"
#include "../../../plugin_types.h"

#define NUM_INPUTS   16
#define NUM_OUTPUTS  16

typedef struct {
    int pin;
    int active_low;
    int byte_idx;
    int bit_idx;
    int fd;
} gpio_channel_t;

/* 16 Digital Inputs configuration */
static gpio_channel_t g_inputs[NUM_INPUTS] = {
    { 487, 1, 0, 0, -1 },
    { 488, 1, 0, 1, -1 },
    { 489, 1, 0, 2, -1 },
    { 490, 1, 0, 3, -1 },
    { 491, 1, 0, 4, -1 },
    { 492, 1, 0, 5, -1 },
    { 493, 1, 0, 6, -1 },
    { 494, 1, 0, 7, -1 },
    { 495, 1, 1, 0, -1 },
    { 496, 1, 1, 1, -1 },
    { 497, 1, 1, 2, -1 },
    { 498, 1, 1, 3, -1 },
    { 499, 1, 1, 4, -1 },
    { 500, 1, 1, 5, -1 },
    { 501, 1, 1, 6, -1 },
    { 502, 1, 1, 7, -1 }
};

/* 16 Digital Outputs configuration */
static gpio_channel_t g_outputs[NUM_OUTPUTS] = {
    { 471, 0, 0, 0, -1 },
    { 472, 0, 0, 1, -1 },
    { 473, 0, 0, 2, -1 },
    { 474, 0, 0, 3, -1 },
    { 475, 0, 0, 4, -1 },
    { 476, 0, 0, 5, -1 },
    { 477, 0, 0, 6, -1 },
    { 478, 0, 0, 7, -1 },
    { 479, 0, 1, 0, -1 },
    { 480, 0, 1, 1, -1 },
    { 481, 0, 1, 2, -1 },
    { 482, 0, 1, 3, -1 },
    { 483, 0, 1, 4, -1 },
    { 484, 0, 1, 5, -1 },
    { 485, 0, 1, 6, -1 },
    { 486, 0, 1, 7, -1 }
};

/* Buzzer configuration: GPIO 507, active_low=TRUE -> %QX2.0 */
static gpio_channel_t g_buzzer = { 507, 1, 2, 0, -1 };

/* LED configuration */
#define LED1_PATH "/sys/class/leds/led1/brightness"
#define LED2_PATH "/sys/class/leds/led2/brightness"
static int g_led1_fd = -1;
static int g_led2_fd = -1;

/* Internal plugin state */
static plugin_logger_t g_logger;
static plugin_runtime_args_t g_args;
static int plugin_initialized = 0;
static int plugin_running     = 0;

/* Export a GPIO pin through sysfs */
static int gpio_export(int pin)
{
    char path[64];
    snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d", pin);

    if (access(path, F_OK) == 0) {
        return 0;
    }

    int fd = open("/sys/class/gpio/export", O_WRONLY);
    if (fd < 0) {
        plugin_logger_error(&g_logger, "gpio_export: cannot open export file: %s", strerror(errno));
        return -1;
    }

    char buf[16];
    int len = snprintf(buf, sizeof(buf), "%d", pin);
    if (write(fd, buf, len) < 0) {
        plugin_logger_error(&g_logger, "gpio_export: write failed for pin %d: %s", pin, strerror(errno));
        close(fd);
        return -1;
    }
    close(fd);
    usleep(100000);
    return 0;
}

/* Set the direction of a GPIO pin ("in" or "out") */
static int gpio_set_direction(int pin, const char *direction)
{
    char path[64];
    snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/direction", pin);

    int fd = open(path, O_WRONLY);
    if (fd < 0) {
        plugin_logger_error(&g_logger, "gpio_set_direction: cannot open %s: %s", path, strerror(errno));
        return -1;
    }

    int len = (int)strlen(direction);
    if (write(fd, direction, len) < 0) {
        plugin_logger_error(&g_logger, "gpio_set_direction: write failed for pin %d: %s", pin, strerror(errno));
        close(fd);
        return -1;
    }
    close(fd);
    return 0;
}

/* Open the GPIO value file for reading or writing */
static int gpio_open_value(int pin, int flags)
{
    char path[64];
    snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/value", pin);

    int fd = open(path, flags);
    if (fd < 0) {
        plugin_logger_error(&g_logger, "gpio_open_value: cannot open %s: %s", path, strerror(errno));
    }
    return fd;
}

/* Initialize I/O */
int init(void *args)
{
    plugin_logger_init(&g_logger, "HUIZ_AMR_IO", NULL);
    plugin_logger_info(&g_logger, "Initializing HUIZ_3562 GPIO I/O mapping plugin...");

    if (!args) {
        plugin_logger_error(&g_logger, "init: args is NULL");
        return -1;
    }

    memcpy(&g_args, args, sizeof(plugin_runtime_args_t));
    plugin_logger_init(&g_logger, "HUIZ_AMR_IO", args);

    int i;

    /* Initialize 16 Digital Inputs */
    for (i = 0; i < NUM_INPUTS; i++) {
        g_inputs[i].fd = -1;

        if (gpio_export(g_inputs[i].pin) != 0) {
            plugin_logger_error(&g_logger, "Failed to export input pin %d", g_inputs[i].pin);
            return -1;
        }
        if (gpio_set_direction(g_inputs[i].pin, "in") != 0) {
            plugin_logger_error(&g_logger, "Failed to set direction for input pin %d", g_inputs[i].pin);
            return -1;
        }
        g_inputs[i].fd = gpio_open_value(g_inputs[i].pin, O_RDONLY);
        if (g_inputs[i].fd < 0) {
            plugin_logger_error(&g_logger, "Failed to open value for input pin %d", g_inputs[i].pin);
            return -1;
        }
    }
    plugin_logger_info(&g_logger, "16 Digital Inputs (GPIO %d..%d) -> %%IX0.0..%%IX1.7 ready",
                       g_inputs[0].pin, g_inputs[NUM_INPUTS - 1].pin);

    /* Initialize 16 Digital Outputs */
    for (i = 0; i < NUM_OUTPUTS; i++) {
        g_outputs[i].fd = -1;

        if (gpio_export(g_outputs[i].pin) != 0) {
            plugin_logger_error(&g_logger, "Failed to export output pin %d", g_outputs[i].pin);
            return -1;
        }
        if (gpio_set_direction(g_outputs[i].pin, "out") != 0) {
            plugin_logger_error(&g_logger, "Failed to set direction for output pin %d", g_outputs[i].pin);
            return -1;
        }
        g_outputs[i].fd = gpio_open_value(g_outputs[i].pin, O_WRONLY);
        if (g_outputs[i].fd < 0) {
            plugin_logger_error(&g_logger, "Failed to open value for output pin %d", g_outputs[i].pin);
            return -1;
        }
        /* Default output off (logical 0 -> physical '0' for active_low=0) */
        char init_val = g_outputs[i].active_low ? '1' : '0';
        if (write(g_outputs[i].fd, &init_val, 1) < 0) {
            plugin_logger_warn(&g_logger, "init: write initial state failed for pin %d: %s", g_outputs[i].pin, strerror(errno));
        }
    }
    plugin_logger_info(&g_logger, "16 Digital Outputs (GPIO %d..%d) -> %%QX0.0..%%QX1.7 ready",
                       g_outputs[0].pin, g_outputs[NUM_OUTPUTS - 1].pin);

    /* Initialize Buzzer Output */
    g_buzzer.fd = -1;
    if (gpio_export(g_buzzer.pin) == 0 && gpio_set_direction(g_buzzer.pin, "out") == 0) {
        g_buzzer.fd = gpio_open_value(g_buzzer.pin, O_WRONLY);
        if (g_buzzer.fd >= 0) {
            /* Default off (logical 0 -> physical '1' for active_low=1) */
            char init_val = g_buzzer.active_low ? '1' : '0';
            if (write(g_buzzer.fd, &init_val, 1) < 0) {
                plugin_logger_warn(&g_logger, "init: write initial state failed for buzzer pin %d: %s", g_buzzer.pin, strerror(errno));
            }
            plugin_logger_info(&g_logger, "Buzzer pin %d -> %%QX%d.%d (active_low=%d) ready",
                               g_buzzer.pin, g_buzzer.byte_idx, g_buzzer.bit_idx, g_buzzer.active_low);
        } else {
            plugin_logger_warn(&g_logger, "Failed to open value file for Buzzer pin %d", g_buzzer.pin);
        }
    } else {
        plugin_logger_warn(&g_logger, "Failed to export/set direction for Buzzer pin %d", g_buzzer.pin);
    }

    /* Initialize LED1 -> %QX3.0 */
    g_led1_fd = open(LED1_PATH, O_WRONLY);
    if (g_led1_fd < 0) {
        plugin_logger_warn(&g_logger, "Failed to open LED1 %s: %s", LED1_PATH, strerror(errno));
    } else {
        if (write(g_led1_fd, "0", 1) < 0) {
            plugin_logger_warn(&g_logger, "init: write initial state failed for LED1: %s", strerror(errno));
        }
        plugin_logger_info(&g_logger, "LED1 %s -> %%QX3.0 ready", LED1_PATH);
    }

    /* Initialize LED2 -> %QX3.1 */
    g_led2_fd = open(LED2_PATH, O_WRONLY);
    if (g_led2_fd < 0) {
        plugin_logger_warn(&g_logger, "Failed to open LED2 %s: %s", LED2_PATH, strerror(errno));
    } else {
        if (write(g_led2_fd, "0", 1) < 0) {
            plugin_logger_warn(&g_logger, "init: write initial state failed for LED2: %s", strerror(errno));
        }
        plugin_logger_info(&g_logger, "LED2 %s -> %%QX3.1 ready", LED2_PATH);
    }

    plugin_initialized = 1;
    plugin_logger_info(&g_logger, "HUIZ_AMR_IO GPIO plugin initialized successfully");
    return 0;
}

int start_loop(void)
{
    if (!plugin_initialized) {
        plugin_logger_error(&g_logger, "start_loop: plugin not initialized");
        return -1;
    }
    plugin_running = 1;
    plugin_logger_info(&g_logger, "HUIZ_AMR_IO GPIO plugin loop started");
    return 0;
}

void stop_loop(void)
{
    plugin_running = 0;
    plugin_logger_info(&g_logger, "HUIZ_AMR_IO GPIO plugin loop stopped");
}

/**
 * cycle_start: Read physical inputs and write to OpenPLC bool_input buffers.
 *
 * Called at the beginning of each PLC scan cycle.
 * Uses journal_write_bool so that writes are race-condition-free.
 *   journal_write_bool(type=0 => BOOL_INPUT, byte_index, bit_index, value)
 */
void cycle_start(void)
{
    if (!plugin_initialized || !plugin_running) {
        return;
    }

    int i;
    for (i = 0; i < NUM_INPUTS; i++) {
        if (g_inputs[i].fd < 0) {
            continue;
        }

        char raw_val = '0';
        lseek(g_inputs[i].fd, 0, SEEK_SET);
        if (read(g_inputs[i].fd, &raw_val, 1) < 0) {
            plugin_logger_warn(&g_logger, "cycle_start: read failed for input %d (pin %d): %s",
                               i, g_inputs[i].pin, strerror(errno));
            continue;
        }

        int phys_val = (raw_val == '1') ? 1 : 0;
        int logic_val = g_inputs[i].active_low ? !phys_val : phys_val;

        g_args.journal_write_bool(0, g_inputs[i].byte_idx, g_inputs[i].bit_idx, logic_val);
    }
}

/**
 * cycle_end: Read OpenPLC bool_output buffers and drive physical outputs.
 *
 * Called at the end of each PLC scan cycle.
 */
void cycle_end(void)
{
    if (!plugin_initialized || !plugin_running) {
        return;
    }

    int i;
    for (i = 0; i < NUM_OUTPUTS; i++) {
        if (g_outputs[i].fd < 0) {
            continue;
        }

        int b_idx = g_outputs[i].byte_idx;
        int bit_idx = g_outputs[i].bit_idx;

        if (g_args.bool_output == NULL || g_args.bool_output[b_idx] == NULL || g_args.bool_output[b_idx][bit_idx] == NULL) {
            continue;
        }

        int logic_val = (*g_args.bool_output[b_idx][bit_idx] != 0) ? 1 : 0;
        int phys_val = g_outputs[i].active_low ? !logic_val : logic_val;
        char out_char = phys_val ? '1' : '0';

        lseek(g_outputs[i].fd, 0, SEEK_SET);
        if (write(g_outputs[i].fd, &out_char, 1) < 0) {
            plugin_logger_warn(&g_logger, "cycle_end: write failed for output %d (pin %d): %s",
                               i, g_outputs[i].pin, strerror(errno));
        }
    }

    /* Drive Buzzer pin (%QX2.0) */
    if (g_buzzer.fd >= 0 &&
        g_args.bool_output != NULL &&
        g_args.bool_output[g_buzzer.byte_idx] != NULL &&
        g_args.bool_output[g_buzzer.byte_idx][g_buzzer.bit_idx] != NULL) {

        int logic_val = (*g_args.bool_output[g_buzzer.byte_idx][g_buzzer.bit_idx] != 0) ? 1 : 0;
        int phys_val = g_buzzer.active_low ? !logic_val : logic_val;
        char out_char = phys_val ? '1' : '0';

        lseek(g_buzzer.fd, 0, SEEK_SET);
        if (write(g_buzzer.fd, &out_char, 1) < 0) {
            plugin_logger_warn(&g_logger, "cycle_end: write failed for buzzer (pin %d): %s",
                               g_buzzer.pin, strerror(errno));
        }
    }

    /* Drive LED1: %QX3.0 -> bool_output[3][0] */
    if (g_led1_fd >= 0 &&
        g_args.bool_output != NULL &&
        g_args.bool_output[3] != NULL &&
        g_args.bool_output[3][0] != NULL) {
        char val = (*g_args.bool_output[3][0] != 0) ? '1' : '0';
        lseek(g_led1_fd, 0, SEEK_SET);
        if (write(g_led1_fd, &val, 1) < 0) {
            plugin_logger_warn(&g_logger, "cycle_end: write failed for LED1: %s", strerror(errno));
        }
    }

    /* Drive LED2: %QX3.1 -> bool_output[3][1] */
    if (g_led2_fd >= 0 &&
        g_args.bool_output != NULL &&
        g_args.bool_output[3] != NULL &&
        g_args.bool_output[3][1] != NULL) {
        char val = (*g_args.bool_output[3][1] != 0) ? '1' : '0';
        lseek(g_led2_fd, 0, SEEK_SET);
        if (write(g_led2_fd, &val, 1) < 0) {
            plugin_logger_warn(&g_logger, "cycle_end: write failed for LED2: %s", strerror(errno));
        }
    }
}

/* cleanup: Release all GPIO resources */
void cleanup(void)
{
    plugin_logger_info(&g_logger, "Cleaning up HUIZ_AMR_IO GPIO plugin...");

    int i;
    for (i = 0; i < NUM_OUTPUTS; i++) {
        if (g_outputs[i].fd >= 0) {
            char off_val = g_outputs[i].active_low ? '1' : '0';
            if (write(g_outputs[i].fd, &off_val, 1) < 0) {
                plugin_logger_warn(&g_logger, "cleanup: write off failed for pin %d: %s", g_outputs[i].pin, strerror(errno));
            }
            close(g_outputs[i].fd);
            g_outputs[i].fd = -1;
        }
    }
    for (i = 0; i < NUM_INPUTS; i++) {
        if (g_inputs[i].fd >= 0) {
            close(g_inputs[i].fd);
            g_inputs[i].fd = -1;
        }
    }

    if (g_buzzer.fd >= 0) {
        char off_val = g_buzzer.active_low ? '1' : '0';
        if (write(g_buzzer.fd, &off_val, 1) < 0) {
            plugin_logger_warn(&g_logger, "cleanup: write off failed for buzzer pin %d: %s", g_buzzer.pin, strerror(errno));
        }
        close(g_buzzer.fd);
        g_buzzer.fd = -1;
    }

    if (g_led1_fd >= 0) {
        if (write(g_led1_fd, "0", 1) < 0) {
            plugin_logger_warn(&g_logger, "cleanup: write off failed for LED1: %s", strerror(errno));
        }
        close(g_led1_fd);
        g_led1_fd = -1;
    }

    if (g_led2_fd >= 0) {
        if (write(g_led2_fd, "0", 1) < 0) {
            plugin_logger_warn(&g_logger, "cleanup: write off failed for LED2: %s", strerror(errno));
        }
        close(g_led2_fd);
        g_led2_fd = -1;
    }

    plugin_initialized = 0;
    plugin_running     = 0;
    plugin_logger_info(&g_logger, "HUIZ_AMR_IO GPIO plugin cleanup done");
}
