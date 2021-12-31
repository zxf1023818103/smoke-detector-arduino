#include <zephyr.h>
#include <logging/log.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/gpio.h>
#include <drivers/uart.h>

LOG_MODULE_REGISTER(main);

#ifdef CONFIG_SHIELD_SMOKEDETECTOR_ARDUIONO
# define ARDUINO_SERIAL_NODE DT_NODELABEL(arduino_serial)
#
# if DT_NODE_HAS_STATUS(ARDUINO_SERIAL_NODE, okay)
#  define ARDUINO_SERIAL DT_LABEL(ARDUINO_SERIAL_NODE)
# else
#  error "Unsupported board: arduino_serial device tree node not found"
# endif
#
# define ZEPHYR_USER DT_PATH(zephyr_user)
# if DT_NODE_EXISTS(ZEPHYR_USER)
#  define WAKEUP_PIN_DT_SPEC GPIO_DT_SPEC_GET_BY_IDX(ZEPHYR_USER, wakeup_gpios, 0)
#  define RESET_PIN_DT_SPEC GPIO_DT_SPEC_GET_BY_IDX(ZEPHYR_USER, reset_gpios, 0)
#  define TEST_PIN_DT_SPEC GPIO_DT_SPEC_GET_BY_IDX(ZEPHYR_USER, test_gpios, 0)
# else
#  error "zephyr,user undefined"
# endif
#else
# error "Shield not found"
#endif

void main(void)
{
	const struct device* arduino_serial = device_get_binding(ARDUINO_SERIAL);
	const struct gpio_dt_spec wakeup_pin_dt_spec = WAKEUP_PIN_DT_SPEC;
	const struct gpio_dt_spec test_pin_dt_spec = TEST_PIN_DT_SPEC;
	const struct gpio_dt_spec reset_pin_dt_spec = RESET_PIN_DT_SPEC;
}
