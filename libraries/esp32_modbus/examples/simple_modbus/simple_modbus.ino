#include "esp32-hal-log.h"
#include "esp32_common.h"
#include "esp32_modbus.h"

static const char* MB_MASTER_TAG = "mb_master";

// modbus slaves table
/*ModbusSlave modbus_slaves[] = {{
  "AC Meter", 1, GPIO_NUM_NC, {
    {"Ua", OBJECT_HOLDING_REGISTER, 0x0025, OBJECT_ADDRESS_LENGTH_1},
    {"Ub", OBJECT_HOLDING_REGISTER, 0x0026, OBJECT_ADDRESS_LENGTH_1},
    {"Uc", OBJECT_HOLDING_REGISTER, 0x0027, OBJECT_ADDRESS_LENGTH_1},
    {"Ia", OBJECT_HOLDING_REGISTER, 0x002b, OBJECT_ADDRESS_LENGTH_1},
    {"Ib", OBJECT_HOLDING_REGISTER, 0x002c, OBJECT_ADDRESS_LENGTH_1},
    {"Ic", OBJECT_HOLDING_REGISTER, 0x002d, OBJECT_ADDRESS_LENGTH_1},
    {"Ps", OBJECT_HOLDING_REGISTER, 0x0031, OBJECT_ADDRESS_LENGTH_1},
    {"PFs", OBJECT_HOLDING_REGISTER, 0x0039, OBJECT_ADDRESS_LENGTH_1},
    {"Ss", OBJECT_HOLDING_REGISTER, 0x003d, OBJECT_ADDRESS_LENGTH_1},
    {"", OBJECT_NONE, 0, OBJECT_ADDRESS_LENGTH_1},
    {"", OBJECT_NONE, 0, OBJECT_ADDRESS_LENGTH_1},
    {"", OBJECT_NONE, 0, OBJECT_ADDRESS_LENGTH_1},
    {"", OBJECT_NONE, 0, OBJECT_ADDRESS_LENGTH_1},
    {"", OBJECT_NONE, 0, OBJECT_ADDRESS_LENGTH_1},
    {"", OBJECT_NONE, 0, OBJECT_ADDRESS_LENGTH_1},
    {"", OBJECT_NONE, 0, OBJECT_ADDRESS_LENGTH_1}
  }}, {
  "DC Meter", 2, GPIO_NUM_NC, {
    {"Voltage RMS", OBJECT_HOLDING_REGISTER, 0x03e8, OBJECT_ADDRESS_LENGTH_2},
    {"Current RMS", OBJECT_HOLDING_REGISTER, 0x03ea, OBJECT_ADDRESS_LENGTH_2},
    {"Positive active energy", OBJECT_HOLDING_REGISTER, 0x03ee, OBJECT_ADDRESS_LENGTH_2},
    {"", OBJECT_NONE, 0, OBJECT_ADDRESS_LENGTH_1},
    {"", OBJECT_NONE, 0, OBJECT_ADDRESS_LENGTH_1},
    {"", OBJECT_NONE, 0, OBJECT_ADDRESS_LENGTH_1},
    {"", OBJECT_NONE, 0, OBJECT_ADDRESS_LENGTH_1},
    {"", OBJECT_NONE, 0, OBJECT_ADDRESS_LENGTH_1},
    {"", OBJECT_NONE, 0, OBJECT_ADDRESS_LENGTH_1},
    {"", OBJECT_NONE, 0, OBJECT_ADDRESS_LENGTH_1},
    {"", OBJECT_NONE, 0, OBJECT_ADDRESS_LENGTH_1},
    {"", OBJECT_NONE, 0, OBJECT_ADDRESS_LENGTH_1},
    {"", OBJECT_NONE, 0, OBJECT_ADDRESS_LENGTH_1}
  }}, {
  "Flow Meter", 3, GPIO_NUM_NC, {
    {"Flow Rate", OBJECT_HOLDING_REGISTER, 0x0001, OBJECT_ADDRESS_LENGTH_2},
    {"Working Timer", OBJECT_HOLDING_REGISTER, 0x0067, OBJECT_ADDRESS_LENGTH_2},
    {"Total Working Time", OBJECT_HOLDING_REGISTER, 0x0069, OBJECT_ADDRESS_LENGTH_1},
    {"Total Power On-Off Time", OBJECT_HOLDING_REGISTER, 0x006a, OBJECT_ADDRESS_LENGTH_1},
    {"", OBJECT_NONE, 0, OBJECT_ADDRESS_LENGTH_1},
    {"", OBJECT_NONE, 0, OBJECT_ADDRESS_LENGTH_1},
    {"", OBJECT_NONE, 0, OBJECT_ADDRESS_LENGTH_1},
    {"", OBJECT_NONE, 0, OBJECT_ADDRESS_LENGTH_1},
    {"", OBJECT_NONE, 0, OBJECT_ADDRESS_LENGTH_1},
    {"", OBJECT_NONE, 0, OBJECT_ADDRESS_LENGTH_1},
    {"", OBJECT_NONE, 0, OBJECT_ADDRESS_LENGTH_1},
    {"", OBJECT_NONE, 0, OBJECT_ADDRESS_LENGTH_1},
    {"", OBJECT_NONE, 0, OBJECT_ADDRESS_LENGTH_1},
    {"", OBJECT_NONE, 0, OBJECT_ADDRESS_LENGTH_1}
  }}, {
  "", 0, GPIO_NUM_NC, {
    {"", OBJECT_NONE, 0, OBJECT_ADDRESS_LENGTH_1},
    {"", OBJECT_NONE, 0, OBJECT_ADDRESS_LENGTH_1},
    {"", OBJECT_NONE, 0, OBJECT_ADDRESS_LENGTH_1},
    {"", OBJECT_NONE, 0, OBJECT_ADDRESS_LENGTH_1},
    {"", OBJECT_NONE, 0, OBJECT_ADDRESS_LENGTH_1},
    {"", OBJECT_NONE, 0, OBJECT_ADDRESS_LENGTH_1},
    {"", OBJECT_NONE, 0, OBJECT_ADDRESS_LENGTH_1},
    {"", OBJECT_NONE, 0, OBJECT_ADDRESS_LENGTH_1},
    {"", OBJECT_NONE, 0, OBJECT_ADDRESS_LENGTH_1},
    {"", OBJECT_NONE, 0, OBJECT_ADDRESS_LENGTH_1},
    {"", OBJECT_NONE, 0, OBJECT_ADDRESS_LENGTH_1},
    {"", OBJECT_NONE, 0, OBJECT_ADDRESS_LENGTH_1},
    {"", OBJECT_NONE, 0, OBJECT_ADDRESS_LENGTH_1},
    {"", OBJECT_NONE, 0, OBJECT_ADDRESS_LENGTH_1},
    {"", OBJECT_NONE, 0, OBJECT_ADDRESS_LENGTH_1},
    {"", OBJECT_NONE, 0, OBJECT_ADDRESS_LENGTH_1}
  }}
};*/

ModbusSlave modbus_slaves[] = {{
  "AC Meter", 3, GPIO_NUM_2, {
    {"Ua", OBJECT_HOLDING_REGISTER, 0x0025, OBJECT_ADDRESS_LENGTH_1},
    {"Ia", OBJECT_HOLDING_REGISTER, 0x002b, OBJECT_ADDRESS_LENGTH_1},
    {"", OBJECT_NONE, 0, OBJECT_ADDRESS_LENGTH_1},
    {"", OBJECT_NONE, 0, OBJECT_ADDRESS_LENGTH_1},
    {"", OBJECT_NONE, 0, OBJECT_ADDRESS_LENGTH_1},
    {"", OBJECT_NONE, 0, OBJECT_ADDRESS_LENGTH_1},
    {"", OBJECT_NONE, 0, OBJECT_ADDRESS_LENGTH_1},
    {"", OBJECT_NONE, 0, OBJECT_ADDRESS_LENGTH_1},
    {"", OBJECT_NONE, 0, OBJECT_ADDRESS_LENGTH_1},
    {"", OBJECT_NONE, 0, OBJECT_ADDRESS_LENGTH_1},
    {"", OBJECT_NONE, 0, OBJECT_ADDRESS_LENGTH_1},
    {"", OBJECT_NONE, 0, OBJECT_ADDRESS_LENGTH_1},
    {"", OBJECT_NONE, 0, OBJECT_ADDRESS_LENGTH_1},
    {"", OBJECT_NONE, 0, OBJECT_ADDRESS_LENGTH_1},
    {"", OBJECT_NONE, 0, OBJECT_ADDRESS_LENGTH_1},
    {"", OBJECT_NONE, 0, OBJECT_ADDRESS_LENGTH_1}
  }}, {
  "DC Meter", 2, GPIO_NUM_15, {
    {"Ua", OBJECT_HOLDING_REGISTER, 0x0025, OBJECT_ADDRESS_LENGTH_1},
    {"Ia", OBJECT_HOLDING_REGISTER, 0x002b, OBJECT_ADDRESS_LENGTH_1},
    {"", OBJECT_NONE, 0, OBJECT_ADDRESS_LENGTH_1},
    {"", OBJECT_NONE, 0, OBJECT_ADDRESS_LENGTH_1},
    {"", OBJECT_NONE, 0, OBJECT_ADDRESS_LENGTH_1},
    {"", OBJECT_NONE, 0, OBJECT_ADDRESS_LENGTH_1},
    {"", OBJECT_NONE, 0, OBJECT_ADDRESS_LENGTH_1},
    {"", OBJECT_NONE, 0, OBJECT_ADDRESS_LENGTH_1},
    {"", OBJECT_NONE, 0, OBJECT_ADDRESS_LENGTH_1},
    {"", OBJECT_NONE, 0, OBJECT_ADDRESS_LENGTH_1},
    {"", OBJECT_NONE, 0, OBJECT_ADDRESS_LENGTH_1},
    {"", OBJECT_NONE, 0, OBJECT_ADDRESS_LENGTH_1},
    {"", OBJECT_NONE, 0, OBJECT_ADDRESS_LENGTH_1},
    {"", OBJECT_NONE, 0, OBJECT_ADDRESS_LENGTH_1},
    {"", OBJECT_NONE, 0, OBJECT_ADDRESS_LENGTH_1},
    {"", OBJECT_NONE, 0, OBJECT_ADDRESS_LENGTH_1}    
  }}, {    
  "", 0, GPIO_NUM_NC, {
    {"", OBJECT_NONE, 0, OBJECT_ADDRESS_LENGTH_1},
    {"", OBJECT_NONE, 0, OBJECT_ADDRESS_LENGTH_1},
    {"", OBJECT_NONE, 0, OBJECT_ADDRESS_LENGTH_1},
    {"", OBJECT_NONE, 0, OBJECT_ADDRESS_LENGTH_1},
    {"", OBJECT_NONE, 0, OBJECT_ADDRESS_LENGTH_1},
    {"", OBJECT_NONE, 0, OBJECT_ADDRESS_LENGTH_1},
    {"", OBJECT_NONE, 0, OBJECT_ADDRESS_LENGTH_1},
    {"", OBJECT_NONE, 0, OBJECT_ADDRESS_LENGTH_1},
    {"", OBJECT_NONE, 0, OBJECT_ADDRESS_LENGTH_1},
    {"", OBJECT_NONE, 0, OBJECT_ADDRESS_LENGTH_1},
    {"", OBJECT_NONE, 0, OBJECT_ADDRESS_LENGTH_1},
    {"", OBJECT_NONE, 0, OBJECT_ADDRESS_LENGTH_1},
    {"", OBJECT_NONE, 0, OBJECT_ADDRESS_LENGTH_1},
    {"", OBJECT_NONE, 0, OBJECT_ADDRESS_LENGTH_1},
    {"", OBJECT_NONE, 0, OBJECT_ADDRESS_LENGTH_1},
    {"", OBJECT_NONE, 0, OBJECT_ADDRESS_LENGTH_1}
  }}
};

// modbus_master(uart num, baudrate, parity, tx gpio, rx gpio, rts gpio, led active)
ESP32ModbusMaster modbus_master(UART_NUM_2, 9600, UART_PARITY_DISABLE, GPIO_NUM_23, GPIO_NUM_19, UART_PIN_NO_CHANGE, LED_ACTIVE_LOW);

void setup() {
  // print esp idf version
  ESP_LOGI(MB_MASTER_TAG, "%s", esp_get_idf_version());

  modbus_master.begin(modbus_slaves);
}

// the loop function runs over and over again forever
void loop() {
  ModbusSlave *slave;
  char str[64];

  slave = modbus_master.get_slave(0);
  ESP_LOGI(MB_MASTER_TAG, "slave #%d %s", slave->address, slave->error == false ? "" : "(error)");
  if (!slave->error) {
    ESP_LOGI(MB_MASTER_TAG, "%s 0x%4.4x = 0x%4.4x", slave->objects[0].name, slave->objects[0].address, slave->objects[0].values[0]);
    ESP_LOGI(MB_MASTER_TAG, "%s 0x%4.4x = 0x%4.4x", slave->objects[1].name, slave->objects[1].address, slave->objects[1].values[0]);
  }

  slave = modbus_master.get_slave(1);
  ESP_LOGI(MB_MASTER_TAG, "slave #%d %s", slave->address, slave->error == false ? "" : "(error)");
  if (!slave->error) {
    ESP_LOGI(MB_MASTER_TAG, "%s 0x%4.4x = 0x%4.4x", slave->objects[0].name, slave->objects[0].address, slave->objects[0].values[0]);
    ESP_LOGI(MB_MASTER_TAG, "%s 0x%4.4x = 0x%4.4x", slave->objects[1].name, slave->objects[1].address, slave->objects[1].values[0]);
  }
  
//  ESP_LOGI(MB_MASTER_TAG, "%s 0x%4.4x = %f", slave->objects[0].name, slave->objects[0].address, ((float)(slave->objects[0].values[0]))/10);
//  ESP_LOGI(MB_MASTER_TAG, "%s 0x%4.4x = %f", slave->objects[1].name, slave->objects[1].address, ((float)(slave->objects[1].values[0]))/1000);

//  snprintf(str, sizeof(str) - 1, "%.1f", ((float)(slave->objects[0].values[0]))/10);
//  printf("%s\n", str);
  
//  snprintf(str, sizeof(str) - 1, "%.3f", ((float)(slave->objects[1].values[0]))/1000);
//  printf("%s\n", str);
  
  
  vTaskDelay(1000 / portTICK_RATE_MS);
}
