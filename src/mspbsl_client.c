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

enum mspbsl_baud_rates {
    BAUDRATE_9600 = 0x02,
    BAUDRATE_19200 = 0x03,
    BAUDRATE_38400 = 0x04,
    BAUDRATE_57600 = 0x05,
    BAUDRATE_115200 = 0x06,
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

static int send_bsl_data_packet(const struct device* uart_device, const uint8_t* data, uint16_t length) {
    uint16_t packet_size = length + 5;
    uint8_t* packet = k_malloc(packet_size);
    if (packet) {
        packet[0] = 0x80; /// header
        packet[1] = length & 0xff;
        packet[2] = (length >> 8) & 0xff;
        memcpy(packet + 3, data, length);

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

static int rx_password(const struct device* uart_device, const uint8_t password[32]) {
    uint8_t data[33];
    data[0] = CMD_RX_PASSWORD;
    memcpy(data + 1, password, 32);
    return send_bsl_data_packet(uart_device, data, sizeof data);
}

static int tx_bsl_version(const struct device* uart_device) {
    uint8_t data[1] = { CMD_TX_BSL_VERSION };
    return send_bsl_data_packet(uart_device, data, sizeof data);
}

static int rx_data_block_with_cmd(const struct device* uart_device, uint8_t cmd, uint32_t address, const uint8_t* data, uint16_t data_size) {
    uint16_t buffer_size = data_size + 4;
    uint8_t *buffer = k_malloc(buffer_size);
    if (buffer) {

        buffer[0] = cmd;
        buffer[1] = address;
        buffer[2] = address >> 8;
        buffer[3] = address >> 16;
        memcpy(buffer + 4, data, data_size);

        int ret = send_bsl_data_packet(uart_device, buffer, buffer_size);
        
        k_free(data);

        return ret;
    }
    else {
        LOG_ERR("%s", "k_malloc(): NULL");
        return -ENOMEM;
    }
}

static int rx_data_block(const struct device* uart_device, uint32_t address, const uint8_t* data, uint16_t data_size) {
    return rx_data_block_with_cmd(uart_device, CMD_RX_DATA_BLOCK, address, data, data_size);
}

static int rx_data_block_fast(const struct device* uart_device, uint32_t address, const uint8_t* data, uint16_t data_size) {
    return rx_data_block_with_cmd(uart_device, CMD_RX_DATA_BLOCK_FAST, address, data, data_size);
}

static int mass_erase(const struct device* uart_device) {
    uint8_t data[1] = { CMD_MASS_ERASE };
    return send_bsl_data_packet(uart_device, data, sizeof data);
}

static int crc_check(const struct device* uart_device, uint32_t address, uint16_t length) {
    uint8_t data[6] = { CMD_CRC_CHECK, address, address >> 8, address >> 16, length, length >> 8 };
    return send_bsl_data_packet(uart_device, data, sizeof data);
}

static int load_pc(const struct device* uart_device, uint32_t address) {
    uint8_t data[4] = { CMD_LOAD_PC, address, address >> 8, address >> 16 };
    return send_bsl_data_packet(uart_device, data, sizeof data);
}

static int change_baud_rate(const struct device* uart_device, enum mspbsl_baud_rates baud_rate) {
    uint8_t data[2] = { CMD_CHANGE_BAUD_RATE, baud_rate };
    return send_bsl_data_packet(uart_device, data, sizeof data);
}

static int connect(const struct device* uart_device,
                   const struct gpio_dt_spec* test_pin_dt_spec,
                   const struct gpio_dt_spec* reset_pin_dt_spec) {
    
    int ret = send_bsl_entry_sequence(test_pin_dt_spec, reset_pin_dt_spec);
    
    uart_irq_rx_enable(uart_device);
}
