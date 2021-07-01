#include <Arduino.h>
#include "chirp_bsp.h"

#include "Wire.h"
#include "chirp_esp32.h"
#include "soniclib.h"
#include "conf_board.h"

static uint8_t chirp_i2c_addrs[] = CHIRP_I2C_ADDRS;
static uint8_t chirp_i2c_buses[] = CHIRP_I2C_BUSES;

uint32_t chirp_pin_prog[]   = CHIRP_PIN_PROG;   
uint32_t chirp_pin_io[]     = CHIRP_PIN_IO;
uint32_t chirp_pin_io_irq[] = CHIRP_PIN_IO_IRQ;         
uint32_t chirp_led_pins[]   = CHIRP_PIN_LED;

/* Chirp sensor group pointer */
ch_group_t  *sensor_group_ptr;

void find_sensors(void) {
  uint8_t sig_bytes[2];

  pinMode(CHIRP_OK_0, OUTPUT);
  pinMode(CHIRP_RST, OUTPUT);         //reset=output
  digitalWrite(CHIRP_RST, HIGH);      //reset=H
  
  /* Drive PROG low on all sensor ports */
  pinMode(CHIRP_PROG_0, OUTPUT);      //PROG_0=output
  digitalWrite(CHIRP_PROG_0, LOW);    //PROG_0=L

  /* check sensor 0 */
  digitalWrite(CHIRP_PROG_0, HIGH);
  sig_bytes[0] = 0;
  sig_bytes[1] = 0;
  Wire.beginTransmission(CH_I2C_ADDR_PROG);
  Wire.write(0x00);
  Wire.endTransmission(false);
  Wire.requestFrom(CH_I2C_ADDR_PROG, 2);
  sig_bytes[0] = Wire.read();
  sig_bytes[1] = Wire.read();
  Serial.print("Chirp sensor 0 ");
  if ((sig_bytes[0] == CH_SIG_BYTE_0) && (sig_bytes[1] == CH_SIG_BYTE_1)) {
    Serial.print("found\n");
    digitalWrite(CHIRP_OK_0, HIGH);
    delay(500);
    digitalWrite(CHIRP_OK_0, LOW);
    } else {
    Serial.print("not found\n");
  }
  digitalWrite(CHIRP_PROG_0, LOW);
}

void ext_int_init(void){
  pinMode(PIN_EXT_ChirpINT0_MASK, INPUT_PULLDOWN);
  attachInterrupt(PIN_EXT_ChirpINT0_MASK, NULL, RISING);
  /* Initialize PIO interrupt handler, interrupt on rising edge. */

  detachInterrupt(PIN_EXT_ChirpINT0_MASK);
}

void chbsp_board_init(ch_group_t *grp_ptr){
  sensor_group_ptr = grp_ptr;
  grp_ptr->num_ports = CHBSP_MAX_DEVICES;
  grp_ptr->num_i2c_buses = CHBSP_NUM_I2C_BUSES;
  grp_ptr->rtc_cal_pulse_ms = CHBSP_RTC_CAL_PULSE_MS;

  Wire.begin(PIN_SDA, PIN_SCL, I2C_CLK);
  Serial.begin(115200);
  ext_int_init();
  find_sensors();
}

void chbsp_reset_assert(void){
  digitalWrite(CHIRP_RST, LOW);
}

void chbsp_reset_release(void){
  digitalWrite(CHIRP_RST, HIGH);
}

void chbsp_program_enable(ch_dev_t *dev_ptr){
  uint8_t dev_num = ch_get_dev_num(dev_ptr);
  digitalWrite(chirp_pin_prog[dev_num], HIGH);
}

void chbsp_program_disable(ch_dev_t *dev_ptr){
  uint8_t dev_num = ch_get_dev_num(dev_ptr);
  digitalWrite(chirp_pin_prog[dev_num], LOW);  
}

void chbsp_set_io_dir_out(ch_dev_t *dev_ptr) {
  uint8_t dev_num = ch_get_dev_num(dev_ptr);
  pinMode(chirp_pin_io[dev_num], OUTPUT);
}

void chbsp_set_io_dir_in(ch_dev_t *dev_ptr) {
  uint8_t dev_num = ch_get_dev_num(dev_ptr);
  pinMode(chirp_pin_io[dev_num], INPUT);
}

void chbsp_group_set_io_dir_out(ch_group_t *grp_ptr) {
  uint8_t dev_num;
  for (dev_num = 0; dev_num < ch_get_num_ports(grp_ptr); dev_num++){
    ch_dev_t *dev_ptr = ch_get_dev_ptr(grp_ptr, dev_num);

    if (ch_sensor_is_connected(dev_ptr))
      pinMode(chirp_pin_io[dev_num], OUTPUT);
  }
}

void chbsp_group_set_io_dir_in(ch_group_t *grp_ptr) {
  uint8_t dev_num;
  for (dev_num = 0; dev_num < ch_get_num_ports(grp_ptr); dev_num++){
    ch_dev_t *dev_ptr = ch_get_dev_ptr(grp_ptr, dev_num);

    if (ch_sensor_is_connected(dev_ptr))
      pinMode(chirp_pin_io[dev_num], INPUT);
  }
}

void chbsp_group_pin_init(ch_group_t *grp_ptr) {
  uint8_t dev_num;
  uint8_t port_num;

  pinMode(CHIRP_PROG_0, OUTPUT); //PROG_0=output
  digitalWrite(CHIRP_PROG_0, LOW); //PROG_0=L

  pinMode(CHIRP_RST, OUTPUT); //reset=output
  chbsp_reset_assert();

  for (dev_num = 0; dev_num < grp_ptr->num_ports; dev_num++) {
    ch_dev_t *dev_ptr = ch_get_dev_ptr(grp_ptr, dev_num);
    chbsp_program_enable(dev_ptr);
  }

  /* Initialize IO pins */
  chbsp_group_set_io_dir_in(grp_ptr);  

  /* Configure PIOs as input pins. */
  for(port_num = 0; port_num < grp_ptr->num_ports; port_num++ ) {
    pinMode(chirp_pin_io_irq[port_num], INPUT_PULLDOWN);
  }
  attachInterrupt(PIN_EXT_ChirpINT0_MASK, NULL, RISING);  
}

void chbsp_group_io_clear(ch_group_t *grp_ptr) {
  uint8_t dev_num;
  for (dev_num = 0; dev_num < ch_get_num_ports(grp_ptr); dev_num++) {
    ch_dev_t *dev_ptr = ch_get_dev_ptr(grp_ptr, dev_num);

    if (ch_sensor_is_connected(dev_ptr)) {
      digitalWrite(chirp_pin_io[dev_num], LOW);
    }
  }
}

void chbsp_group_io_set(ch_group_t *grp_ptr) {
  uint8_t dev_num;
  for (dev_num = 0; dev_num < ch_get_num_ports(grp_ptr); dev_num++) {
    ch_dev_t *dev_ptr = ch_get_dev_ptr(grp_ptr, dev_num);

    if (ch_sensor_is_connected(dev_ptr)) {
      digitalWrite(chirp_pin_io[dev_num], HIGH);
    }
  }
}

void chbsp_group_io_interrupt_enable(ch_group_t *grp_ptr) {
  uint8_t dev_num;

  for (dev_num = 0; dev_num < ch_get_num_ports(grp_ptr); dev_num++) {
    ch_dev_t *dev_ptr = ch_get_dev_ptr(grp_ptr, dev_num);
    chbsp_io_interrupt_enable(dev_ptr);
  }
}

void chbsp_io_interrupt_enable(ch_dev_t *dev_ptr) {
  uint8_t dev_num = ch_get_dev_num(dev_ptr);

  if (ch_sensor_is_connected(dev_ptr)) {
    attachInterrupt(chirp_pin_io_irq[dev_num], NULL, RISING);
  }
}

void chbsp_group_io_interrupt_disable(ch_group_t *grp_ptr) {
  uint8_t dev_num;

  for (dev_num = 0; dev_num < ch_get_num_ports(grp_ptr); dev_num++) {
    ch_dev_t *dev_ptr = ch_get_dev_ptr(grp_ptr, dev_num);
    chbsp_io_interrupt_disable(dev_ptr);
  }
}

void chbsp_io_interrupt_disable(ch_dev_t *dev_ptr) {

  if (ch_sensor_is_connected(dev_ptr)) {
    detachInterrupt(chirp_pin_io_irq[dev_ptr->io_index]);
  }
}

void chbsp_io_clear(ch_dev_t *dev_ptr) {
  digitalWrite(chirp_pin_io[dev_ptr->io_index], LOW);
}

void chbsp_io_set(ch_dev_t *dev_ptr) {
  digitalWrite(chirp_pin_io[dev_ptr->io_index], HIGH);
}

void chbsp_io_callback_set(ch_io_int_callback_t callback_func_ptr) {
  io_int_callback_ptr = callback_func_ptr;
}

void chbsp_delay_us(uint32_t us) {
  delayMicroseconds(us);
}

void chbsp_delay_ms(uint32_t ms) {
  delay(ms);
}

int chbsp_i2c_init(void) {
  Wire.begin(PIN_SDA, PIN_SCL, I2C_CLK);
  return 0;
}

uint8_t chbsp_i2c_get_info(ch_group_t __attribute__((unused)) *grp_ptr, uint8_t io_index, ch_i2c_info_t *info_ptr) {
  uint8_t ret_val = 1;

  if (io_index <= CHBSP_MAX_DEVICES) {
    info_ptr->address = chirp_i2c_addrs[io_index];
    info_ptr->bus_num = chirp_i2c_buses[io_index];

    info_ptr->drv_flags = 0;  // no special I2C handling by SonicLib driver is needed

    ret_val = 0;
  }
  return ret_val;
}

int chbsp_i2c_write(ch_dev_t *dev_ptr, uint8_t *data, uint16_t num_bytes) {
  int error = 0;
  if (dev_ptr->i2c_bus_index == 0) {
    Wire.beginTransmission(dev_ptr->i2c_address);
    Wire.write(data, num_bytes);
    error = Wire.endTransmission(); //I2C bus 0   
  } 
  return error;
}

int chbsp_i2c_mem_write(ch_dev_t *dev_ptr, uint16_t mem_addr, uint8_t *data, uint16_t num_bytes) {
  int error=0;
  if (dev_ptr->i2c_bus_index == 0) {
    Wire.beginTransmission(dev_ptr->i2c_address);
    Wire.write(highByte(mem_addr));
    Wire.write(lowByte(mem_addr));
    Wire.write(data, num_bytes);
    error = Wire.endTransmission(); //I2C bus 0   
  }
  return error;
}

int chbsp_i2c_read(ch_dev_t *dev_ptr, uint8_t *data, uint16_t num_bytes) {
  int error = 1;    // default is error return
  uint8_t i2c_addr = ch_get_i2c_address(dev_ptr);
  uint8_t bus_num  = ch_get_i2c_bus(dev_ptr);

  if (bus_num == 0) {
    // I2C bus 0 (TWI1)
    Wire.requestFrom(i2c_addr,num_bytes);
    error = Wire.readBytes(data, num_bytes);
  } 
  return !(num_bytes-error);
}

int chbsp_i2c_mem_read(ch_dev_t *dev_ptr, uint16_t mem_addr, uint8_t *data, uint16_t num_bytes) {
  int error = 1;    // default is error return
  uint8_t i2c_addr = ch_get_i2c_address(dev_ptr);
  uint8_t bus_num  = ch_get_i2c_bus(dev_ptr);

  if (bus_num == 0) {
    // I2C bus 0 (TWI1)
    Wire.beginTransmission(i2c_addr); 
    Wire.write(highByte(mem_addr));
    Wire.write(lowByte(mem_addr));
    Wire.endTransmission();

    Wire.requestFrom(i2c_addr,num_bytes);
    error = Wire.readBytes(data, num_bytes);
    } 
  return !(num_bytes-error);
}

void chbsp_i2c_reset(ch_dev_t * dev_ptr) {
  uint8_t  bus_num  = ch_get_i2c_bus(dev_ptr);

  if (bus_num == 0) {          // I2C bus 0 
      Wire.begin(PIN_SDA, PIN_SCL, I2C_CLK);
  } 
}
