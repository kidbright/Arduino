#include <stdio.h>
#include <string.h>
#include "esp32-hal-log.h"
#include "esp32_common.h"
#include "esp32_modbus.h"

static const char* ESP32_MODBUS_TAG = "esp32_modbus";

// https://esp32.com/viewtopic.php?t=2085#post_content9885

void ESP32ModbusMaster::Task(void *pvParameters) {	
	ESP32ModbusMaster *modbus_master = (ESP32ModbusMaster *)pvParameters;
	int i;
	gpio_config_t io_conf;	
	esp_err_t err;
	ModbusSlave *slave;
	ModbusObject *object;
	mb_param_request_t request;
	uint16_t val[MAX_OBJECT_ADDR_LEN];
	char str[128];
	bool skip_delay_flag = false;
	
	while (1) {
		// modbus state machine
		switch (modbus_master->state) {
			case s_start:
				modbus_master->tickcnt = get_tickcnt();
				modbus_master->state = s_init;
				break;

			case s_init:
				if (is_tickcnt_elapsed(modbus_master->tickcnt, 1000)) {
					// init and start modbus controller
					memset(&modbus_master->comm, 0, sizeof(mb_communication_info_t));
					modbus_master->comm.port = modbus_master->uart_num;
					modbus_master->comm.mode = MB_MODE_RTU;
					modbus_master->comm.baudrate = modbus_master->baudrate;
					modbus_master->comm.parity = modbus_master->parity;

					ESP_LOGD(ESP32_MODBUS_TAG, "%s", "init modbus master");
					err = mbc_master_init(MB_PORT_SERIAL_MASTER, &modbus_master->master_handler);
					if (modbus_master->master_handler == NULL) {
						ESP_LOGD(ESP32_MODBUS_TAG, "%s", "mbc_master_init() failed!");
					}
					else
					if (err != ESP_OK) {
						ESP_LOGD(ESP32_MODBUS_TAG, "%s", "mbc_master_init() failed!, returns(0x%x)", (uint32_t) err);
					}
					else
					if ((err = mbc_master_setup((void*) &modbus_master->comm)) != ESP_OK) {
						ESP_LOGD(ESP32_MODBUS_TAG, "%s", "mbc_master_setup() failed!, returns(0x%x)", (uint32_t) err);
					}
					else                             
					// set uart pin numbers
					if ((err = uart_set_pin(modbus_master->uart_num, modbus_master->tx_io_num, modbus_master->rx_io_num, modbus_master->rts_io_num, UART_PIN_NO_CHANGE)) != ESP_OK) {
						ESP_LOGD(ESP32_MODBUS_TAG, "%s", "uart_set_pin() failed!, returned (0x%x)", (uint32_t) err);
					}
					else
					// modbus master start
					if ((err = mbc_master_start()) != ESP_OK) {
						ESP_LOGD(ESP32_MODBUS_TAG, "%s", "mbc_master_start() failed!, returns(0x%x)", (uint32_t) err);
					}
					else
					// set driver mode to half duplex
					if ((err = uart_set_mode(modbus_master->uart_num, UART_MODE_RS485_HALF_DUPLEX)) != ESP_OK) {
						ESP_LOGD(ESP32_MODBUS_TAG, "%s", "uart_set_mode() failed!, returned (0x%x)", (uint32_t) err);
					}
					else {						
						ESP_LOGD(ESP32_MODBUS_TAG, "%s", "modbus master stack initialized");
						modbus_master->error = false;
						modbus_master->tickcnt = get_tickcnt();
						modbus_master->state = s_init_done;
					}

					if (err != ESP_OK) {
						modbus_master->error = true;
						modbus_master->tickcnt = get_tickcnt();
						modbus_master->state = s_error;
					}
				}				
				break;

			case s_init_done:
				if (is_tickcnt_elapsed(modbus_master->tickcnt, 100)) {
					modbus_master->state = s_pre_scan;
				}			
				break;

			case s_pre_scan:
				modbus_master->slave_index = 0;
				modbus_master->object_index = 0;
				modbus_master->state = s_scan;
				// skip current loop delay
				skip_delay_flag = true;
				break;

			case s_scan:
				slave = &modbus_master->slaves[modbus_master->slave_index];
				if ((strcmp(slave->name, "") != 0) && (slave->address != 0) && (modbus_master->slave_index < MAX_SLAVE)) {
					object = &slave->objects[modbus_master->object_index];
					if ((strcmp(object->name, "") != 0) && (object->type != OBJECT_NONE)) {
						// check for supported object type
						if (object->type == OBJECT_HOLDING_REGISTER) {							
							if (object->length == OBJECT_ADDRESS_LENGTH_2) {
								snprintf(str, sizeof(str) - 1, "scan slave #%d => 0x%4.4x..0x%4.4x", slave->address, object->address, object->address + 1);

							}
							else {
								snprintf(str, sizeof(str) - 1, "scan slave #%d => 0x%4.4x", slave->address, object->address);
							}
							ESP_LOGD(ESP32_MODBUS_TAG, "%s", str);						

							request.slave_addr = slave->address;
							request.command = 3; // read holding register
							request.reg_start = object->address; // register address
							request.reg_size = (object->length == OBJECT_ADDRESS_LENGTH_2) ? 2 : 1; // number of register							

							if ((err = mbc_master_send_request(&request, &val[0])) == ESP_OK) {																
								if (object->length == OBJECT_ADDRESS_LENGTH_2) {
									object->values[0] = val[0];
									object->values[1] = val[1];
									snprintf(str, sizeof(str) - 1, "mbc_master_send_request().val = 0x%4.4x 0x%4.4x", val[0], val[1]);
								}
								else {
									object->values[0] = val[0];
									snprintf(str, sizeof(str) - 1, "mbc_master_send_request().val = 0x%4.4x", val[0]);
								}
								ESP_LOGD(ESP32_MODBUS_TAG, "%s", str);

								// next object
								modbus_master->object_index++;
							}
							else {
								ESP_LOGD(ESP32_MODBUS_TAG, "mbc_master_send_request() failed, returns(0x%x).", (uint32_t) err);

								// set current slave error flag
								slave->error = true;
								// skip to next slave
								modbus_master->slave_index++;
								modbus_master->object_index = 0;
							}
						}
						else {
							// next object
							modbus_master->object_index++;
						}						
					}
					else {
						// all objects successfully scanned, clear current slave error flag
						slave->error = false;
						// scan next slave
						modbus_master->slave_index++;
						modbus_master->object_index = 0;
						// skip current loop delay
						skip_delay_flag = true;
					}
				}
				else {
					//ESP_LOGD(ESP32_MODBUS_TAG, "%s", "scan finished.");
					//modbus_master->state = s_idle;
					modbus_master->state = s_pre_scan;
					// skip current loop delay
					skip_delay_flag = true;
				}
				break;

			case s_error:
				if (is_tickcnt_elapsed(modbus_master->tickcnt, 3000)) {
					modbus_master->state = s_init;
				}
				break;

			case s_idle:				
				break;							
		}

		// led state machine
		switch (modbus_master->led_state) {
			case s_led_init:
				// init all slaves led gpio
				for (i = 0; i < modbus_master->slave_count; i++) {
					if (modbus_master->slaves[i].led_gpio != GPIO_NUM_NC) {
						// led init
						io_conf.intr_type = GPIO_INTR_DISABLE; // disable interrupt
						io_conf.mode = GPIO_MODE_OUTPUT; // set as output mode
						io_conf.pin_bit_mask = (1ULL << modbus_master->slaves[i].led_gpio); // pin bit mask
						io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE; // disable pull-down mode
						io_conf.pull_up_en = GPIO_PULLUP_DISABLE; // disable pull-up mode				
						gpio_set_level(modbus_master->slaves[i].led_gpio, modbus_master->led_active ^ 0x01);
						gpio_config(&io_conf);
					}
				}
				modbus_master->led_toggle_status = modbus_master->led_active ^ 0x01;
				modbus_master->led_tickcnt = get_tickcnt();
				modbus_master->led_state = s_led_idle;
				break;

			case s_led_idle:
				if (is_tickcnt_elapsed(modbus_master->led_tickcnt, LED_BLINK_DELAY_MS)) {		
					modbus_master->led_tickcnt = get_tickcnt();
					// update led toggle status
					modbus_master->led_toggle_status = modbus_master->led_toggle_status ^ 0x01;
					// update slave led					
					for (i = 0; i < modbus_master->slave_count; i++) {
						// check led gpio configured
						if (modbus_master->slaves[i].led_gpio != GPIO_NUM_NC) {
							if (!modbus_master->slaves[i].error) {		
								gpio_set_level(modbus_master->slaves[i].led_gpio, modbus_master->led_active);
							}
							else {								
								gpio_set_level(modbus_master->slaves[i].led_gpio, modbus_master->led_toggle_status);
							}
						}
					}
				}			
				break;
		}

		// check if need to skip current loop delay
		if (skip_delay_flag) {
			skip_delay_flag = false;
		}
		else {
			vTaskDelay(MODBUS_LOOP_DELAY_MS / portTICK_RATE_MS);
		}		
	}	
}

ESP32ModbusMaster::ESP32ModbusMaster(uart_port_t _uart_num, uint32_t _baudrate, uart_parity_t _parity, int _tx_io_num, int _rx_io_num, int _rts_io_num, LedActiveType _led_active) {
	uart_num = _uart_num;
	baudrate = _baudrate;
	parity = _parity;
	tx_io_num = _tx_io_num;
	rx_io_num = _rx_io_num;
	rts_io_num = _rts_io_num;
	led_active = _led_active;
}

void ESP32ModbusMaster::begin(ModbusSlave *_slaves) {
	unsigned int len;

	// init
	state = s_init;
	led_state = s_led_init;
	slave_count = 0;
	error = true;

	while ((strcmp(_slaves[slave_count].name, "") != 0) && (_slaves[slave_count].address != 0) && (slave_count < MAX_SLAVE)) {
		// init modbus slave
		snprintf(slaves[slave_count].name, MAX_NAME_CHAR, "%s", _slaves[slave_count].name);
		slaves[slave_count].address = _slaves[slave_count].address;
		slaves[slave_count].error = true;
		slaves[slave_count].led_gpio = _slaves[slave_count].led_gpio;		

		// init modbus objects
		for (int i = 0; i < MAX_SLAVE_OBJECT; i++) {
			snprintf(slaves[slave_count].objects[i].name, MAX_NAME_CHAR, "%s", _slaves[slave_count].objects[i].name);
			slaves[slave_count].objects[i].type = _slaves[slave_count].objects[i].type;
			slaves[slave_count].objects[i].address = _slaves[slave_count].objects[i].address;
			// load address length equal 1 or 2
			slaves[slave_count].objects[i].length = _slaves[slave_count].objects[i].length;
			// init value array to zero
			for (int j = 0; j < MAX_OBJECT_ADDR_LEN; j++) {
				slaves[slave_count].objects[i].values[j] = 0;
			}
		}

		slave_count++;
	}
	
	// create modbus master task
	xTaskCreatePinnedToCore(this->Task, "Modbus Task", MODBUS_STACK_SIZE_MIN, this, MODBUS_TASK_PRIORITY, NULL, MODBUS_RUNNING_CORE);
}

ModbusSlave *ESP32ModbusMaster::get_slave(int index) {
	return &slaves[index];
}

void ESP32ModbusMaster::test(void) {
	//TickType_t t = get_tickcnt();	
	printf("slave_count = %d\n", slave_count);
	for (int i = 0; i < slave_count; i++) {
		printf("%s (%d)\n", slaves[i].name, slaves[i].address);
		for (int j = 0; j < MAX_SLAVE_OBJECT; j++) {
			if (slaves[i].objects[j].address == 0) {
				break;
			}
			printf("\t%s (0x%4.4x)\n", slaves[i].objects[j].name, slaves[i].objects[j].address);
		}
	}
}
