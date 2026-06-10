#include <inttypes.h>
#include <stdint.h>
#include <stdlib.h>

#include "i2c_device_hal.h"
#include "i2c_device.h"
#include "driver/i2c_types.h"
#include "string.h"

#define WRITE_DATA_STASH_SIZE 200

i2c_master_bus_handle_t g_bus_handle[I2C_NUM_MAX] = {NULL};

typedef struct {
    i2c_master_bus_handle_t bus_handle;
    uint32_t freq_hz;
} qmsd_i2c_port_handle_t;

static qmsd_i2c_port_handle_t *i2c_port_handle_from_int(int i2c_port)
{
    return (qmsd_i2c_port_handle_t *)(intptr_t)i2c_port;
}

static esp_err_t i2c_add_transaction_device(qmsd_i2c_port_handle_t *port_handle, uint8_t device_addr, i2c_master_dev_handle_t *out_dev)
{
    i2c_device_config_t i2c_dev_conf = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .scl_speed_hz = port_handle->freq_hz,
        .device_address = device_addr,
    };

    return i2c_master_bus_add_device(port_handle->bus_handle, &i2c_dev_conf, out_dev);
}

// Return i2c handle for read and write, -1 mean error
int i2c_dev_init(int i2c_num, i2c_port_obj_t* port_obj) {
    if (i2c_num >= I2C_NUM_MAX) {
        return -1;
    }

    i2c_master_bus_handle_t bus_handle = NULL;
    if (g_bus_handle[i2c_num] == NULL) {
        i2c_master_bus_config_t i2c_bus_config = {
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .i2c_port = i2c_num,
            .scl_io_num = port_obj->scl,
            .sda_io_num = port_obj->sda,
            .glitch_ignore_cnt = 7,
            .flags.enable_internal_pullup = true,
        };

        if (i2c_new_master_bus(&i2c_bus_config, &bus_handle) != ESP_OK) {
            return I2C_PORT_NO_INIT;
        }
        g_bus_handle[i2c_num] = bus_handle;
    } else {
        bus_handle = g_bus_handle[i2c_num];
    }

    qmsd_i2c_port_handle_t *port_handle = calloc(1, sizeof(qmsd_i2c_port_handle_t));
    if (port_handle == NULL) {
        return I2C_PORT_NO_INIT;
    }
    port_handle->bus_handle = bus_handle;
    port_handle->freq_hz = port_obj->freq;
    port_obj->port = (int)(intptr_t)port_handle;
    return port_obj->port;
}

int i2c_dev_update_pins(int i2c_num, i2c_port_obj_t* select_port, i2c_port_obj_t* old_port) {
    i2c_log_e("i2c num %d not support update bin", i2c_num);
    return select_port->port;
}

int i2c_dev_update_freq(int i2c_num, i2c_port_obj_t* port_obj) {
    if (port_obj->port == I2C_PORT_NO_INIT) {
        return -1;
    }
    qmsd_i2c_port_handle_t *port_handle = i2c_port_handle_from_int(port_obj->port);
    port_handle->freq_hz = port_obj->freq;
    return port_obj->port;
}

int i2c_dev_deinit(int i2c_port) {
    // not support now
    return -1;
}

// reg len (byte): 0, 1, 2
// data: NULL or data
// return:
int i2c_dev_write_bytes(int i2c_port, uint8_t device_addr, uint32_t reg_addr, uint8_t reg_len, const uint8_t* data, uint16_t length) {
    if (i2c_port == I2C_PORT_NO_INIT || (length > 0 && data == NULL)) {
        return I2C_FAIL;
    }

    qmsd_i2c_port_handle_t *port_handle = i2c_port_handle_from_int(i2c_port);
    if (reg_len + length == 0) { // i2c_scan
        return i2c_master_probe(port_handle->bus_handle, device_addr, I2C_TIMEOUT_MS) == ESP_OK ? I2C_OK : I2C_FAIL;
    }

    i2c_master_dev_handle_t dev = NULL;
    if (i2c_add_transaction_device(port_handle, device_addr, &dev) != ESP_OK) {
        return I2C_FAIL;
    }

    int ret = I2C_FAIL;
    if (reg_len == 0) {
        ret = i2c_master_transmit(dev, data, length, I2C_TIMEOUT_MS) == ESP_OK ? I2C_OK : I2C_FAIL;
        i2c_master_bus_rm_device(dev);
        return ret;
    }

    if (reg_len + length >= WRITE_DATA_STASH_SIZE) {
        i2c_log_e("data stash size is not enough, reg_len:%d, length:%d, pls used no reg setting", reg_len, length);
        i2c_master_bus_rm_device(dev);
        return I2C_FAIL;
    }
    
    uint8_t data_stash[WRITE_DATA_STASH_SIZE] = {0};
    memcpy(data_stash, &reg_addr, reg_len);
    if (length > 0) {
        memcpy(data_stash + reg_len, data, length);
    }
    ret = i2c_master_transmit(dev, data_stash, reg_len + length, I2C_TIMEOUT_MS) == ESP_OK ? I2C_OK : I2C_FAIL;
    i2c_master_bus_rm_device(dev);
    return ret;
}

int i2c_dev_read_bytes(int i2c_port, uint8_t device_addr, uint32_t reg_addr, uint8_t reg_len, uint8_t* data, uint16_t length) {
    if (i2c_port == I2C_PORT_NO_INIT || (length > 0 && data == NULL)) {
        return I2C_FAIL;
    }
    qmsd_i2c_port_handle_t *port_handle = i2c_port_handle_from_int(i2c_port);
    i2c_master_dev_handle_t dev = NULL;
    if (i2c_add_transaction_device(port_handle, device_addr, &dev) != ESP_OK) {
        return I2C_FAIL;
    }

    int ret = I2C_FAIL;
    if (reg_len == 0) {
        ret = i2c_master_receive(dev, data, length, I2C_TIMEOUT_MS) == ESP_OK ? I2C_OK : I2C_FAIL;
        i2c_master_bus_rm_device(dev);
        return ret;
    }

    ret = i2c_master_transmit_receive(dev, (uint8_t *)&reg_addr, reg_len, data, length, I2C_TIMEOUT_MS) == ESP_OK ? I2C_OK : I2C_FAIL;
    i2c_master_bus_rm_device(dev);
    return ret;
}

i2c_master_bus_handle_t i2c_dev_get_bus_handle(int i2c_num) {
    return g_bus_handle[i2c_num];
}