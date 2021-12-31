#include <logging/log.h>
#include <zephyr.h>
#include <device.h>
#include <drivers/gpio.h>
#include <drivers/uart.h>

LOG_MODULE_REGISTER(mspbsl);

static K_FIFO_DEFINE(uart_fifo);

enum mspbsl_commands {
    CMD_RX_DATA_BLOCK = 0x10,
    CMD_RX_PASSWORD = 0x11,
    CMD_MASS_ERASE = 0x15,
    CMD_CRC_CHECK = 0x16,
    CMD_LOAD_PC = 0x17,
    CMD_TX_DATA_BLOCK = 0x18,
    CMD_TX_BSL_VERSION = 0x19,
    CMD_RX_DATA_BLOCK_FAST = 0x1B,
    CMD_CHANGE_BAUD_RATE = 0x52,
};

struct uart_fifo_data {
    
    sys_snode_t node;

    int len;

    uint8_t data[32];
};

static void on_uart_irq(const struct device *uart_device, void *user_data) {
    ARG_UNUSED(user_data);

    while (uart_irq_update(uart_device) && uart_irq_is_pending(uart_device)) {
        int ret = uart_irq_rx_ready(uart_device);
        if (ret == 1) {
            struct uart_fifo_data fifo_data = { 0 };
            for (;;) {
                ret = uart_fifo_read(uart_device, fifo_data.data, sizeof fifo_data.data - fifo_data.len);
                if (ret > 0) {
                    continue;
                }
                else if (ret == 0) {
                    break;
                }
                else {
                    LOG_ERR("uart_fifo_read(): %d", ret);
                    return;
                }
            }
            
            if (fifo_data.len) {
                struct uart_fifo_data *data = k_malloc(sizeof(struct uart_fifo_data));
                if (data) {
                    memcpy(data, &fifo_data, sizeof fifo_data);
                    k_fifo_put(&uart_fifo, data);
                }
                else {
                    LOG_ERR("%s", "k_malloc(): NULL");
                    return;
                }
            }
        }
        else if (ret == 0) {
            continue;
        }
        else {
            LOG_ERR("uart_irq_rx_ready(): %d", ret);
            return;
        }
    }
}

static int configure_serial(const struct device* uart_device) {
    struct uart_config uart_config = {
        .baudrate = 9600,
        .data_bits = UART_CFG_DATA_BITS_8,
        .flow_ctrl = UART_CFG_FLOW_CTRL_NONE,
        .parity = UART_CFG_PARITY_EVEN,
        .stop_bits = UART_CFG_STOP_BITS_1,
    };

    uart_irq_rx_disable(uart_device);
    int ret = uart_configure(uart_device, &uart_config);
    if (!ret) {
        uart_irq_callback_user_data_set(uart_device, on_uart_irq, NULL);
    }
    else {
        LOG_ERR("uart_configure(): %d", ret);
    }
    return ret;
}

static int configure_device(const struct device* uart_device,
                            const struct gpio_dt_spec* test_pin_dt_spec,
                            const struct gpio_dt_spec* reset_pin_dt_spec) {
	int ret = configure_serial(uart_device);
    if (!ret) {
        ret = gpio_pin_configure_dt(test_pin_dt_spec, 0);
        if (!ret) {
            ret = gpio_pin_configure_dt(reset_pin_dt_spec, 0);
            return ret;
        }
        else {
            return ret;
        }
    }
    else {
        return ret;
    }
}

static int send_bsl_entry_sequence(const struct gpio_dt_spec* test_pin_dt_spec,
                                   const struct gpio_dt_spec* reset_pin_dt_spec) {
    int ret;

    ret = gpio_pin_set_dt(test_pin_dt_spec, 0);
    if (ret) {
        goto err;
    }

    ret = gpio_pin_set_dt(reset_pin_dt_spec, 0);
    if (ret) {
        goto err;
    }

    k_msleep(1);

    ret = gpio_pin_set_dt(test_pin_dt_spec, 1);
    if (ret) {
        goto err;
    }

    k_usleep(CONFIG_MSP430_SBW_ENABLE_MICROSECONDS);

    ret = gpio_pin_set_dt(test_pin_dt_spec, 0);
    if (ret) {
        goto err;
    }

    k_usleep(CONFIG_MSP430_SBW_ENABLE_MICROSECONDS);

    ret = gpio_pin_set_dt(test_pin_dt_spec, 1);
    if (ret) {
        goto err;
    }

    k_usleep(CONFIG_MSP430_SBW_ENABLE_MICROSECONDS);

    ret = gpio_pin_set_dt(reset_pin_dt_spec, 1);
    if (ret) {
        goto err;
    }

    ret = gpio_pin_set_dt(test_pin_dt_spec, 0);
    if (ret) {
        goto err;
    }

err:
    LOG_ERR("gpio_pin_set_dt(): %d", ret);
    return ret;
}

static uint16_t calc_checksum(uint8_t* packet, uint16_t packet_size) {
    uint16_t checksum_low, checksum_high;
    checksum_low = checksum_high = 0;
    for (uint16_t i = 0; i < packet_size - 2; i += 2) {
        checksum_low ^= packet[i];
    }
    for (uint16_t i = 1; i < packet_size - 2; i += 2) {
        checksum_high ^= packet[i];
    }

    uint16_t checksum = checksum_low | (checksum_high << 8);
    return checksum;
}

static int send_bsl_data_packet(const struct device* uart_device, enum mspbsl_commands cmd, uint32_t address,
                                const uint8_t* data, uint16_t data_size) {
    uint16_t packet_size = data_size + 9;
    uint16_t length = data_size + 4;
    uint8_t* packet = k_malloc(packet_size);
    if (packet) {
        packet[0] = 0x80; /// header
        packet[1] = length & 0xff;
        packet[2] = (length >> 8) & 0xff;
        packet[3] = cmd;
        packet[4] = address & 0xff;
        packet[5] = (address >> 8) & 0xff;
        packet[6] = (address >> 16) & 0xff;
        memcpy(packet + 7, data, data_size);

        uint16_t checksum = calc_checksum(packet, packet_size);
        packet[packet_size - 2] = checksum & 0xff;
        packet[packet_size - 1] = (checksum >> 8) & 0xff;

        int ret = uart_tx(uart_device, packet, packet_size, SYS_FOREVER_MS);
        if (ret) {
            LOG_ERR("uart_tx(): %d", ret);
            return ret;
        }
        return 0;
    }
    else {
        LOG_ERR("%s", "k_malloc(): NULL");
        return -ENOMEM;
    }
}

static int connect(const struct device* uart_device,
                   const struct gpio_dt_spec* test_pin_dt_spec,
                   const struct gpio_dt_spec* reset_pin_dt_spec) {
    
    int ret = send_bsl_entry_sequence(test_pin_dt_spec, reset_pin_dt_spec);

    send_bsl_data_packet(uart_device, )
}
