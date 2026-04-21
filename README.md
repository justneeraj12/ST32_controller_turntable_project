# STM32 Turntable Controller Project

Professional repository scaffold for the University turntable motion-control project using STM32 and PowerSTEP01.

## Repository Layout

```text
.
|-- .devcontainer/
|-- controllers_images/
|-- docs/
|-- firmware/
|-- hardware/
|-- scripts/
|-- tests/
|-- README.md
|-- .gitignore
```

## Hardware Architecture

- MCU Board: NUCLEO-G0B1RE
- Motor Driver Expansion: X-NUCLEO-IHM03A1 (PowerSTEP01)
- Motion Reference Tool: Servo Calibrator

## Network Configuration

- Hostname: `ur-20235301042.clarkson.edu`
- IP Address: `128.153.134.42`

## Development Environment

- IDE Bundle: `st-stm32cubeide_2.1.1_28236_20260312_0043_amd64.deb_bundle.sh`
- Target Platform: STM32Cube HAL

## Engineering Log (Auto-Updating)

This section can be refreshed from your Git commit history:

```bash
bash scripts/update_engineering_log.sh 20
```

The command replaces the entries between the markers below with the latest commit messages.

<!-- ENGINEERING_LOG_START -->
- No commits yet. Create your first commit and rerun this script.
<!-- ENGINEERING_LOG_END -->

## Prompt: Board ID Verification Code

Using STM32Cube HAL, this snippet reads the PowerSTEP01 `STATUS` register over `SPI1` and reports the board status to the Serial ITM console.

```c
#include "main.h"
#include <stdio.h>

extern SPI_HandleTypeDef hspi1;

/*
 * These symbols should match your CubeMX-generated pin names.
 * Example: POWERSTEP_CS_GPIO_Port / POWERSTEP_CS_Pin
 */
#define POWERSTEP_CS_LOW()  HAL_GPIO_WritePin(POWERSTEP_CS_GPIO_Port, POWERSTEP_CS_Pin, GPIO_PIN_RESET)
#define POWERSTEP_CS_HIGH() HAL_GPIO_WritePin(POWERSTEP_CS_GPIO_Port, POWERSTEP_CS_Pin, GPIO_PIN_SET)

static uint16_t PowerSTEP01_ReadStatus(void)
{
    uint8_t tx[3] = {0xD0, 0x00, 0x00}; /* GET_STATUS + 16-bit response */
    uint8_t rx[3] = {0};

    POWERSTEP_CS_LOW();
    HAL_StatusTypeDef hal_status = HAL_SPI_TransmitReceive(&hspi1, tx, rx, 3, HAL_MAX_DELAY);
    POWERSTEP_CS_HIGH();

    if (hal_status != HAL_OK)
    {
        return 0xFFFFU;
    }

    return (uint16_t)(((uint16_t)rx[1] << 8) | rx[2]);
}

void Verify_IHM03A1_Driver(void)
{
    uint16_t status = PowerSTEP01_ReadStatus();

    if ((status == 0x0000U) || (status == 0xFFFFU))
    {
        printf("SPI Communication Error\r\n");
    }
    else
    {
        printf("IHM03A1 Driver Found\r\n");
    }
}
```

## Prompt: PID Setup (The "Carrol" Requirement)

Closed-loop PID function using `TIM2` in Encoder Mode. It maps encoder counts (4096 CPR) to degrees, computes position error for targets `{0, 90, 180}`, and returns a velocity command for PowerSTEP01.

```c
#include "main.h"
#include <stdint.h>

typedef struct
{
    float kp;
    float ki;
    float kd;
    float integral;
    float prev_error;
    float integral_min;
    float integral_max;
    float output_min;
    float output_max;
} PID_Controller_t;

static const float k_targets_deg[3] = {0.0f, 90.0f, 180.0f};

int32_t PID_ComputeVelocityCommand(PID_Controller_t *pid,
                                   TIM_HandleTypeDef *htim2,
                                   uint8_t target_index,
                                   float dt_sec)
{
    const float counts_per_rev = 4096.0f;
    const float deg_per_count = 360.0f / counts_per_rev;
    uint32_t raw_count;
    float position_deg;
    float target_deg;
    float error;
    float derivative;
    float output;

    if ((pid == NULL) || (htim2 == NULL) || (target_index > 2U) || (dt_sec <= 0.0f))
    {
        return 0;
    }

    raw_count = __HAL_TIM_GET_COUNTER(htim2) % (uint32_t)counts_per_rev;
    position_deg = (float)raw_count * deg_per_count;
    target_deg = k_targets_deg[target_index];
    error = target_deg - position_deg;

    /* Wrap error to shortest path on circular motion */
    while (error > 180.0f)
    {
        error -= 360.0f;
    }
    while (error < -180.0f)
    {
        error += 360.0f;
    }

    pid->integral += error * dt_sec;
    if (pid->integral > pid->integral_max)
    {
        pid->integral = pid->integral_max;
    }
    else if (pid->integral < pid->integral_min)
    {
        pid->integral = pid->integral_min;
    }

    derivative = (error - pid->prev_error) / dt_sec;
    output = (pid->kp * error) + (pid->ki * pid->integral) + (pid->kd * derivative);
    pid->prev_error = error;

    if (output > pid->output_max)
    {
        output = pid->output_max;
    }
    else if (output < pid->output_min)
    {
        output = pid->output_min;
    }

    /* Return signed velocity command for PowerSTEP01 (for example, steps/s scale). */
    return (int32_t)output;
}
```

### Notes for Integration

- Ensure `TIM2` is initialized in Encoder Mode and started with `HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);`.
- Tune `kp`, `ki`, and `kd` experimentally for your mechanical setup.
- Map the returned velocity command to your PowerSTEP01 run-speed API.
