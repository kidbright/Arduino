#ifndef ESP32_MODBUS_H
#define ESP32_MODBUS_H

#include <stdint.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "mbcontroller.h"

#define MAX_SLAVE               4
#define MAX_SLAVE_OBJECT        16
#define MAX_NAME_CHAR           31
#define MAX_OBJECT_ADDR_LEN     2

#define MODBUS_RUNNING_CORE			1
#define	MODBUS_TASK_PRIORITY    8
#define MODBUS_STACK_SIZE_MIN   8192

#define MODBUS_LOOP_DELAY_MS    100
#define LED_BLINK_DELAY_MS      200

typedef enum {
  LED_ACTIVE_LOW,
  LED_ACTIVE_HIGH
} LedActiveType;

typedef enum { 
    OBJECT_NONE,
    OBJECT_COILS,
    OBJECT_DISCRETE_INPUT,
    OBJECT_INPUT_REGISTER,
    OBJECT_HOLDING_REGISTER
} ModbusObjectType;

typedef enum { 
    OBJECT_ADDRESS_LENGTH_1,
    OBJECT_ADDRESS_LENGTH_2
} ModbusObjectAddressLengthType;

typedef struct ModbusObjectStruct {
  char name[MAX_NAME_CHAR + 1];
  ModbusObjectType type;
  uint16_t address;
  ModbusObjectAddressLengthType length;
  uint16_t values[MAX_OBJECT_ADDR_LEN];
} ModbusObject;

typedef struct ModbusSlaveStruct {
  char name[MAX_NAME_CHAR + 1];
  uint8_t address;
  gpio_num_t led_gpio;  
  ModbusObject objects[MAX_SLAVE_OBJECT];
  bool error;  
} ModbusSlave;

class ESP32ModbusMaster {
  private:
    enum {
      s_start, s_init, s_init_done, s_pre_scan, s_scan, s_error, s_idle
    } state;
    enum {
      s_led_init, s_led_idle
    } led_state;

    // uart config
    uart_port_t uart_num;
    uint32_t baudrate;
    uart_parity_t parity;
    int tx_io_num;
    int rx_io_num;
    int rts_io_num;

    ModbusSlave slaves[MAX_SLAVE];
    int slave_count;
    void *master_handler = NULL;
    mb_communication_info_t comm;
    TickType_t tickcnt, led_tickcnt, printf_tickcnt;
    LedActiveType led_active;
    uint8_t led_toggle_status;    
    bool error;
    int slave_index;
    int object_index;

  public:
    ESP32ModbusMaster(uart_port_t _uart_num, uint32_t _baudrate, uart_parity_t _parity, int _tx_io_num, int _rx_io_num, int _rts_io_num, LedActiveType _led_active);
    static void Task(void *pvParameters);
    ModbusSlave *get_slave(int index);
    void begin(ModbusSlave *_slaves);
    void test(void);
};

#endif
