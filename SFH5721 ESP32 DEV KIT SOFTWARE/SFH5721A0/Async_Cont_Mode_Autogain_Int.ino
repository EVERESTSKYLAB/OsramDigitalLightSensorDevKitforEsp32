// Hardware-specific includes
#include <Arduino.h>
#include <Wire.h>

// SFH5721 library include
#include <SFH5721.h>

// defines for the SFH5721 I2C adress and interrupt pin number
#define ALS_I2C_ADRESS 0x26
#define INT_Pin 23

// hardware specific defines for I2C initialisation
#define SCL_SENS_Pin 22
#define SDA_SENS_Pin 21
#define I2CSpeed 400000L

// hardware specific I2C initialisation
TwoWire I2C_to_Sens = TwoWire(0);

// functions for low level hardware access

// you have to replace these for your own hardware
// I2C read byte function
uint8_t i2c_r(uint8_t i2c_adress, uint8_t register_adress, uint8_t * read_buffer, uint8_t number_of_bytes)
{
  uint8_t iterator = 0;
  I2C_to_Sens.beginTransmission(i2c_adress);
  I2C_to_Sens.write(register_adress);
  I2C_to_Sens.endTransmission();
  I2C_to_Sens.requestFrom(i2c_adress, number_of_bytes);
  while (iterator  < number_of_bytes && I2C_to_Sens.available()) {
    read_buffer[iterator] = I2C_to_Sens.read();
    iterator ++;
  }
  return iterator;
}

// I2C write byte function
void i2c_w(uint8_t i2c_adress, uint8_t register_adress, uint8_t * write_buffer, uint8_t number_of_bytes)
{
  I2C_to_Sens.beginTransmission(i2c_adress);
  I2C_to_Sens.write(register_adress);
  for (uint8_t i = 0; i < number_of_bytes; i++) {
    I2C_to_Sens.write(write_buffer[i]);
  }
  I2C_to_Sens.endTransmission();
}

// GPIO write function
void gpio_w(uint8_t pin, uint8_t val) {
  pinMode(pin, OUTPUT);
  digitalWrite(pin, val);
}

// GPIO read function
uint8_t gpio_r(uint8_t pin) {
  pinMode(pin, INPUT);
  return digitalRead(pin);
}



// interrupt service routine for data ready interrupt
bool interrupt_flag = false;
void ISR_ALS() {
  interrupt_flag = true;
}

// instantiate SFH5721 Object
SFH5721 ALS1(ALS_I2C_ADRESS, INT_Pin, i2c_w, i2c_r, gpio_w, gpio_r);

float ALS_val, IR_val;
uint16_t count;
uint8_t tmp;



void setup() {
  // configure hardware-specific communication interfaces
  Serial.begin(115200);
  I2C_to_Sens.begin(SDA_SENS_Pin, SCL_SENS_Pin, I2CSpeed);

  // hardware specific enable gpio interrupt for falling edge
  attachInterrupt(digitalPinToInterrupt(INT_Pin), ISR_ALS, FALLING);

  // reset SFH5721 -- all registers are initialized
  ALS1.reset();

  // configure settings for your measurement
  // you can find the defines for this setup in the SFH5721.h file
  // settings are written to the MCONFA MCONFB and MCONFC registers
  // Parameters:
  // 1: analog gain
  // 2: integration time
  // 3: digital gain
  // 4: wait time between measurements
  // 5: dark current compensation
  // 6: digital lowpass filtering enable
  // 7: digital lowpass filter depth
  ALS1.configureMeasurementSettings(AGAIN_1, IT_100_MS, DGAIN_1, WAIT_0_US, SUB, LPFEN3, LPFDEPTH_2);

  // configure interrupt settings
  // you can find the defines for this setup in the SFH5721.h file
  // settings are written to the ICONF registers
  // Parameters:
  // 1: interrupt persistence 
  // 2: iterrupt generation condition
  // 3: interrupt clear condition
  ALS1.configureInterrupt(IPERS_1, INT_WHEN_ONE, INT_CLEAR_ON_IFLAGS_READ);

  // enable interrupt for ALS channel
  ALS1.enableInterrupt(IEN3);

  // configure interrupt low and high threshold
  // this is the percentage of the maximum adc value the conversion result has to rise above or fall below to trigger the interrupt
  // and run the automatic gain and integration time control
  ALS1.configureInterruptThresholds(10, 90);

  // starts asynchronous continuous conversions for every channel by writing to the OPSEL register
  ALS1.startContinuousMeasurementAsync();

}

void loop() {

  ALS1.readContinuousDataAsync(&ALS_val, &IR_val);
  // read counts for the ALS channel
  ALS1.readMeasurementValueRaw(ALS_CHANNEL, &count); 

  if (interrupt_flag == true) {
    interrupt_flag = false;
    // clear interrupt by reading the interrupt flags
    ALS1.readInterruptBits(&tmp);
    // run automatic analog and diigital gain control routine in software
    ALS1.autoGain();
  }

  Serial.println("SFH5721 Measurements:");
  Serial.print("Ev: ");
  Serial.print(ALS_val);
  Serial.print(" Lux \n");
  Serial.print("Ee: ");
  Serial.print(IR_val);
  Serial.print(" uW/cm^2 \n");
  Serial.print("Counts: ");
  Serial.print(count);
  Serial.print("\n");

}
