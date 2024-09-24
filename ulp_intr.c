/*
 * ulp_intr.c
 *
 *  Created on: 2024年9月24日
 *      Author: ch.wang
 */
#include "sl_driver_gpio.h"
#include "sl_gpio_board.h"
#include "sl_si91x_driver_gpio.h"
#include "rsi_debug.h"

#define ULP_INT_CH        0      // ULP GPIO Pin interrupt 0
#define MODE_0            0      // Initializing GPIO MODE_0 value
#define OUTPUT_VALUE      1      // GPIO output value

typedef sl_gpio_t sl_si91x_gpio_t;
typedef sl_gpio_mode_t sl_si91x_gpio_mode_t;

/*******************************************************************************
 *  This API handles ULP GPIO OR'ed pin interrupt request
 ******************************************************************************/
static void gpio_ulp_pin_interrupt_callback(uint32_t pin_intr)
{
  if (pin_intr == ULP_PIN_INTR_0) {
    // This is with respect to ISR context. Debugout might cause issues
    // sometimes.
    DEBUGOUT("gpio ulp pin interrupt0\n");
  }
}

/*******************************************************************************
 * ULP GPIO initialization function
 ******************************************************************************/
static void gpio_driver_ulp_initialization(void)
{
  sl_status_t status;
  sl_gpio_driver_init();
  sl_si91x_gpio_t gpio_port_pin1 = { SL_SI91X_ULP_GPIO_1_PORT, SL_SI91X_ULP_GPIO_1_PIN };
  sl_si91x_gpio_t gpio_port_pin2 = { SL_SI91X_ULP_GPIO_2_PORT, SL_SI91X_ULP_GPIO_2_PIN };
  sl_si91x_gpio_t gpio_port_pin6 = { SL_SI91X_ULP_GPIO_6_PORT, SL_SI91X_ULP_GPIO_6_PIN };
  sl_gpio_mode_t mode            = MODE_0;

  do {
    DEBUGOUT("\r\n ULP_GPIO_PIN test starts \r\n");
    status = sl_si91x_gpio_driver_enable_clock((sl_si91x_gpio_select_clock_t)ULPCLK_GPIO); // Enable GPIO ULP_CLK
    if (status != SL_STATUS_OK) {
      DEBUGOUT("sl_si91x_gpio_driver_enable_clock, Error code: %lu", status);
      break;
    }
    DEBUGOUT("GPIO driver clock enable is successful \n");
    // Enable pad receiver for ULP GPIO pins
    status = sl_si91x_gpio_driver_enable_ulp_pad_receiver(SL_SI91X_ULP_GPIO_1_PIN);
    if (status != SL_STATUS_OK) {
      DEBUGOUT("sl_si91x_gpio_driver_enable_ulp_pad_receiver, Error code: %lu", status);
      break;
    }
    DEBUGOUT("GPIO driver ulp pad receiver enable is successful \n");
    status = sl_si91x_gpio_driver_enable_ulp_pad_receiver(SL_SI91X_ULP_GPIO_2_PIN);
    if (status != SL_STATUS_OK) {
      DEBUGOUT("sl_si91x_gpio_driver_enable_ulp_pad_receiver, Error code: %lu", status);
      break;
    }
    DEBUGOUT("GPIO driver ulp pad receiver enable is successful \n");
    status = sl_si91x_gpio_driver_enable_ulp_pad_receiver(SL_SI91X_ULP_GPIO_6_PIN);
    if (status != SL_STATUS_OK) {
      DEBUGOUT("sl_si91x_gpio_driver_enable_ulp_pad_receiver, Error code: %lu", status);
      break;
    }
    DEBUGOUT("GPIO driver ulp pad receiver enable is successful \n");

    // Select pad driver strength for ULP GPIO pins
    status =
      sl_si91x_gpio_driver_select_ulp_pad_driver_strength(SL_SI91X_ULP_GPIO_1_PIN,
                                                          (sl_si91x_gpio_driver_strength_select_t)GPIO_TWO_MILLI_AMPS);
    if (status != SL_STATUS_OK) {
      DEBUGOUT("sl_si91x_gpio_driver_select_ulp_pad_driver_strength, Error "
               "code: %lu",
               status);
      break;
    }
    DEBUGOUT("GPIO driver ulp pad driver strength selection is successful \n");
    // Select pad driver disable state for ULP GPIO pins
    status = sl_si91x_gpio_driver_select_ulp_pad_driver_disable_state(SL_SI91X_ULP_GPIO_1_PIN,
                                                                      (sl_si91x_gpio_driver_disable_state_t)GPIO_HZ);
    if (status != SL_STATUS_OK) {
      DEBUGOUT("sl_si91x_gpio_driver_select_ulp_pad_driver_disable_state, "
               "Error code: %lu",
               status);
      break;
    }
    DEBUGOUT("GPIO driver ulp pad driver disable state selection is successful \n");
    // Set the pin mode for ULP GPIO pins.
    status = sl_gpio_driver_set_pin_mode(&gpio_port_pin1, mode, OUTPUT_VALUE);
    if (status != SL_STATUS_OK) {
      DEBUGOUT("sl_gpio_driver_set_pin_mode, Error code: %lu", status);
      break;
    }
    DEBUGOUT("GPIO driver set pin mode is successful \n");
    status = sl_gpio_driver_set_pin_mode(&gpio_port_pin2, mode, OUTPUT_VALUE);
    if (status != SL_STATUS_OK) {
      DEBUGOUT("sl_gpio_driver_set_pin_mode, Error code: %lu", status);
      break;
    }
    DEBUGOUT("GPIO driver set pin mode is successful \n");
    status = sl_gpio_driver_set_pin_mode(&gpio_port_pin6, mode, OUTPUT_VALUE);
    if (status != SL_STATUS_OK) {
      DEBUGOUT("sl_gpio_driver_set_pin_mode, Error code: %lu", status);
      break;
    }
    DEBUGOUT("GPIO driver set pin mode is successful \n");

    // Set the pin direction for ULP GPIO pins.
    status = sl_si91x_gpio_driver_set_pin_direction(SL_SI91X_ULP_GPIO_1_PORT,
                                                    SL_SI91X_ULP_GPIO_1_PIN,
                                                    (sl_si91x_gpio_direction_t)GPIO_INPUT);
    if (status != SL_STATUS_OK) {
      DEBUGOUT("sl_si91x_gpio_driver_set_pin_direction, Error code: %lu", status);
      break;
    }
    DEBUGOUT("GPIO driver set pin direction is successful \n");
    status = sl_si91x_gpio_driver_set_pin_direction(SL_SI91X_ULP_GPIO_2_PORT,
                                                    SL_SI91X_ULP_GPIO_2_PIN,
                                                    (sl_si91x_gpio_direction_t)GPIO_OUTPUT);
    if (status != SL_STATUS_OK) {
      DEBUGOUT("sl_si91x_gpio_driver_set_pin_direction, Error code: %lu", status);
      break;
    }
    DEBUGOUT("GPIO driver set pin direction is successful \n");
    status = sl_si91x_gpio_driver_set_pin_direction(SL_SI91X_ULP_GPIO_6_PORT,
                                                    SL_SI91X_ULP_GPIO_6_PIN,
                                                    (sl_si91x_gpio_direction_t)GPIO_INPUT);
    if (status != SL_STATUS_OK) {
      DEBUGOUT("sl_si91x_gpio_driver_set_pin_direction, Error code: %lu", status);
      break;
    }
    DEBUGOUT("GPIO driver set pin direction is successful \n");
  } while (false);
}

sl_status_t ulp_interrupt_setup(void)
{
  sl_status_t status;
  uint8_t direction;
  sl_si91x_gpio_version_t version;

  // Version information of gpio
  version = sl_si91x_gpio_driver_get_version();
  DEBUGOUT("gpio version is fetched successfully \n");
  DEBUGOUT("API version is %d.%d.%d\n", version.release, version.major, version.minor);
  do {
    sl_si91x_gpio_t gpio_port_pin = { SL_SI91X_ULP_GPIO_6_PORT, SL_SI91X_ULP_GPIO_6_PIN };
    sl_si91x_gpio_t port_pin      = { SL_SI91X_ULP_GPIO_10_PORT, SL_SI91X_ULP_GPIO_10_PIN };
    sl_si91x_gpio_mode_t mode     = MODE_0;
    // GPIO initialization function for ULP instance
    gpio_driver_ulp_initialization();
    // Get the pin direction for ULP GPIO pin
    direction = sl_si91x_gpio_driver_get_pin_direction(gpio_port_pin.port, gpio_port_pin.pin);
    DEBUGOUT("get_pin_direction = %d\n", direction);
    direction = sl_si91x_gpio_driver_get_pin_direction(port_pin.port, port_pin.pin);
    DEBUGOUT("get_pin_direction = %d\n", direction);

    // Get the pin mode for ULP GPIO pin
    status = sl_gpio_driver_get_pin_mode(&gpio_port_pin, &mode);
    if (status != SL_STATUS_OK) {
      DEBUGOUT("sl_gpio_driver_get_pin_mode, Error code: %lu", status);
      break;
    }
    DEBUGOUT("GPIO driver get pin mode is successful \n");
    DEBUGOUT("get_pin_mode = %d\n", mode);
    status = sl_gpio_driver_get_pin_mode(&port_pin, &mode);
    if (status != SL_STATUS_OK) {
      DEBUGOUT("sl_gpio_driver_get_pin_mode, Error code: %lu", status);
      break;
    }
    DEBUGOUT("GPIO driver get pin mode is successful \n");
    DEBUGOUT("get_pin_mode = %d\n", mode);

    // GPIO initialization function for ULP instance
    gpio_driver_ulp_initialization();
    // Configure ULP GPIO pin interrupts
    status = sl_si91x_gpio_driver_configure_ulp_pin_interrupt(
        ULP_INT_CH,
        (sl_si91x_gpio_interrupt_config_flag_t)SL_GPIO_INTERRUPT_FALL_EDGE,
        SL_SI91X_ULP_GPIO_6_PIN,
        (sl_gpio_irq_callback_t)&gpio_ulp_pin_interrupt_callback);
    if (status != SL_STATUS_OK) {
      DEBUGOUT("sl_si91x_gpio_driver_configure_ulp_pin_interrupt, Error code: %lu", status);
      break;
    }
    DEBUGOUT("GPIO driver configure ulp pin interrupt is successful \n");
  }while(false);
  return status;
}
