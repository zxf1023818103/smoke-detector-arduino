#include <zephyr.h>
#include <logging/log.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/gpio.h>
#include <drivers/uart.h>

LOG_MODULE_REGISTER(main);

#define ARDUINO_SERIAL_NODE DT_NODELABEL(arduino_serial)
#define ARDUINO_HEADER_NODE DT_NODELABEL(arduino_header)

#if DT_NODE_HAS_STATUS(ARDUINO_SERIAL_NODE, okay)
#define ARDUINO_SERIAL DT_LABEL(ARDUINO_SERIAL_NODE)
#else
#error "Unsupported board: arduino_serial devicetree alias is not defined"
#endif

#if DT_NODE_HAS_STATUS(ARDUINO_HEADER_NODE, okay)

#else
#error "Unsupported board: arduino_header devicetree alias is not defined"
#endif

void main(void)
{
	const struct device* arduino_serial_device = device_get_binding(ARDUINO_SERIAL);
	
	
}
