#include <zephyr.h>
#include <logging/log.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/gpio.h>
#include <drivers/uart.h>
#include <uartframeparser.h>
#include <string.h>
#include <stdlib.h>
#include <assert.h>
#include <stdio.h>

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

static void on_error(void* user_ptr, enum uart_frame_parser_error_types error_type, const char* file, int line, const char* fmt, ...) {

    (void)user_ptr;

    const char* error_type_name[] = { "", "cJSON", "malloc", "Config Parse", "Lua" };

    const char* log_template = "%s:%d:%s: %s";
    char* format = (char*)calloc(snprintf(NULL, 0, log_template, file, line, error_type_name[error_type], fmt) + 1, 1);
    assert(format != NULL);
    sprintf(format, log_template, file, line, error_type_name[error_type], fmt);

    va_list ap1, ap2;
    va_start(ap1, fmt);
    va_copy(ap2, ap1);

    char* message = (char*)calloc(vsnprintf(NULL, 0, format, ap1) + 1, 1);
    assert(message != NULL);
    vsprintf(message, format, ap2);
    LOG_INF("%s", message);
    free(message);
    free(format);
}

void on_data1(void* buffer, struct uart_frame_definition* frame_definition, uint32_t frame_bytes, struct uart_frame_field_info* field_info_head, void* user_ptr) {
    
    struct uart_frame_field_data* field_data_head = uart_frame_parser_read_concerned_fields(buffer, field_info_head, NULL, on_error, user_ptr);

    uart_frame_parser_eval_tostring_expression(field_info_head);

    const char* json = uart_frame_parser_jsonify_frame_data(buffer, frame_definition, frame_bytes, field_data_head);

    LOG_INF("%s", json);

    free((void*)json);

    uart_frame_parser_field_data_release(field_data_head);
}

const char* json = "{\"$schema\":\"./frame.json\",\"definitions\":[{\"name\":\"status_report\",\"fields\":[{\"bytes\":2,\"name\":\"sof\",\"default\":\"return '\\\\x55\\\\xaa'\",\"description\":\"Start of Frame\"},{\"bytes\":1,\"name\":\"length\",\"description\":\"Length of Frame\"},{\"bytes\":\"return byte(3) - 4\",\"name\":\"subframe\",\"frames\":[\"subframe1\",\"subframe2\"]},{\"bytes\":1,\"name\":\"checksum\",\"description\":\"Checksum\"}],\"validator\":\"return byte(1) == 0x55 and byte(2) == 0xaa and byte(byte(3)) == (sum(1, byte(3) - 1) & 0xff)\"},{\"name\":\"subframe1\",\"description\":\"Subframe 1\",\"fields\":[{\"bytes\":1,\"name\":\"sof\",\"description\":\"Start of Frame\",\"default\":\"return '\\\\x01'\",\"tostring\":\"return 'Start of Frame'\"},{\"bytes\":1,\"name\":\"working_status\",\"description\":\"Working status\",\"bitfields\":[{\"name\":\"fan_status\",\"description\":\"Fan Status\",\"bits\":1,\"tostring\":\"return ({'Off','On'})[(byte(2) & 1) + 1]\"},{\"bits\":7}]}],\"validator\":\"return byte(1) == 0x01\"},{\"name\":\"subframe2\",\"description\":\"Subframe 2\",\"fields\":[{\"bytes\":1,\"name\":\"sof\",\"description\":\"Start of Frame\",\"default\":\"return '\\\\x02'\"},{\"bytes\":1,\"name\":\"fan_status\",\"description\":\"Fan status\"}],\"validator\":\"return byte(1) == 0x02\"}],\"frames\":[\"status_report\"],\"init\":\"function sum(from, to) result = 0; for i = from, to, 1 do result = result + byte(i) end; return result; end\"}";

void main(void)
{
	// const struct device* arduino_serial = device_get_binding(ARDUINO_SERIAL);
	// const struct gpio_dt_spec wakeup_pin_dt_spec = WAKEUP_PIN_DT_SPEC;
	// const struct gpio_dt_spec test_pin_dt_spec = TEST_PIN_DT_SPEC;
	// const struct gpio_dt_spec reset_pin_dt_spec = RESET_PIN_DT_SPEC;

	struct uart_frame_parser *parser = uart_frame_parser_create(json, (uint32_t)strlen(json), on_error, on_data1, NULL);

    const uint8_t data[] = { 0x55, 0xaa, 0x07, 0x01, 0x01, 0x02, 0x0a,
                            0x55, 0xaa, 0x07, 0x02, 0x01, 0x02, 0x0b };

    uart_frame_parser_feed_data(parser, (const uint8_t*)data, sizeof data);

    uart_frame_parser_release(parser);
}
