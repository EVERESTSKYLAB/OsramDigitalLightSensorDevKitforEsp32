#include "SFH5721.h"
//#include <Arduino.h>




/** Generates SFH5721 Object. The parameter for the low level communication functions are optional here. If you skip them you have to call registerCommunicationFunctions() and registerGPIOFunctions() by yourself to register them.
   @param [in] arg_I2C_Adress Address of ALS-Chip on I2C bus
   @param [in] arg_INT_Pin Interrupt-Pin that the ALS-Chip is connected to
   @param [in] arg_I2C_write_byte Pointer to a function to send a specified number of bytes over I2C to a device specified by i2c_adress into the register specified by register_adress. Parameter 0: I2C-adress 1: register-adress 2: pointer to send buffer 3: number of bytes to send
   @param [in] arg_I2C_read_byte Pointer to a function to read a specified number of bytes over I2C from a device specified by i2c_adress from the register specified by register_adress. Parameter 0: I2C-adress 1: register-adress 2: pointer to receive buffer 3: number of bytes to receive. Return: number of bytes actually received.
   @param [in] arg_GPIO_write Pointer to a function to write the value specified by the second parameter of a GPIO_pin specified by the first parameter. Argument is INT_Pin.
   @param [in] arg_GPIO_read  Pointer to a function to read the value of a GPIO_pin specified by the argument and return it as a uint8_t. Argument is INT_Pin.
*/
SFH5721::SFH5721(uint8_t arg_I2C_Adress,
                 uint8_t arg_INT_Pin,
                 void (*arg_I2C_write_byte)(uint8_t, uint8_t, uint8_t*, uint8_t),
                 uint8_t (*arg_I2C_read_byte)(uint8_t, uint8_t, uint8_t*, uint8_t),
                 void (*arg_GPIO_write)(uint8_t, uint8_t),
                 uint8_t (*arg_GPIO_read)(uint8_t)
                )
{
  SFH5721::I2C_Adress = arg_I2C_Adress;
  SFH5721::INT_Pin = arg_INT_Pin;

  SFH5721::I2C_write_byte = arg_I2C_write_byte;
  SFH5721::I2C_read_byte = arg_I2C_read_byte;
  SFH5721::GPIO_write = arg_GPIO_write;
  SFH5721::GPIO_read = arg_GPIO_read;

  SFH5721::digital_gain = 0;
  SFH5721::analog_gain = 0;
  SFH5721::integration_time = 0;

  SFH5721::single_mode = false;
  SFH5721::synchronous_mode = false;
  SFH5721::interrupt_mode = false;

  SFH5721::analog_gain_bm = 0;
  SFH5721::integration_time_bm = 0;
  SFH5721::digital_gain_bm = 0;
  SFH5721::wait_time_bm = 0;
  SFH5721::dark_current_sub_bm = 0;
  SFH5721::lowpass_filter_enable_bm = 0;
  SFH5721::lowpass_filter_depth_bm = 0;
  SFH5721::persistence_bm = 0;
  SFH5721::or_and_bm = 0;
  SFH5721::clear_condition_bm = 0;
  SFH5721::interrupt_enable_bm = 0;

  SFH5721::low_threshold_bm = 0x0000;
  SFH5721::high_threshold_bm = 0xFFFF;

  SFH5721::low_threshold_perc = 0;
  SFH5721::high_threshold_perc = 100;

  SFH5721::mode_of_operation_bm = 0;
  SFH5721::channel_enable_bm = 0;

  SFH5721::analog_gain_index = 0;
  SFH5721::digital_gain_index = 0;
  SFH5721::integration_time_index = 0;

}



/** Destroys SFH5721 Object

*/
SFH5721::~SFH5721() {}

/** Registers functions for I2C communication. If you did this already when instantiating the object you dont have to use this function.
   @param [in] arg_I2C_write_byte Pointer to a function to send a specified number of bytes over I2C to a device specified by i2c_adress into the register specified by register_adress. Parameter 0: I2C-adress 1: register-adress 2: pointer to send buffer 3: number of bytes to send
   @param [in] arg_I2C_read_byte Pointer to a function to read a specified number of bytes over I2C from a device specified by i2c_adress from the register specified by register_adress. Parameter 0: I2C-adress 1: register-adress 2: pointer to receive buffer 3: number of bytes to receive. Return: number of bytes actually received.
*/
void SFH5721::registerCommunicationFunctions(void (*arg_I2C_write_byte)(uint8_t, uint8_t, uint8_t*, uint8_t),
    uint8_t (*arg_I2C_read_byte)(uint8_t, uint8_t, uint8_t*, uint8_t)
                                            )
{
  SFH5721::I2C_write_byte = arg_I2C_write_byte;
  SFH5721::I2C_read_byte = arg_I2C_read_byte;
}

/** Registers functions for GPIO usage. If you did this already when instantiating the object you dont have to use this function.
   @param [in] arg_GPIO_write Pointer to a function to write the value specified by the second parameter of a GPIO_pin specified by the first parameter. Argument is INT_Pin.
   @param [in] arg_GPIO_read  Pointer to a function to read the value of a GPIO_pin specified by the argument and return it as a uint8_t. Argument is INT_Pin.
*/
void SFH5721::registerGPIOFunctions(void (*arg_GPIO_write)(uint8_t, uint8_t),
                                    uint8_t (*arg_GPIO_read)(uint8_t)
                                   )
{
  SFH5721::GPIO_write = arg_GPIO_write;
  SFH5721::GPIO_read = arg_GPIO_read;
}


/** Configures the measurement settings (digital/analog gain, integration time, filtering, darkcurrent compensation, wait time between measurements).
    Settings are stored in class and sent to the ALS-Chip via _I2C_8_write_register.
  @see SFH5721.h for possible bitmasks defines.
    @param [in] arg_analog_gain_bm Bitmask for analog gain. Possible Bitmask looks like AGAIN_#.
    @param [in] arg_integration_time_bm Bitmask for integration time. Possible Bitmask looks like IT_###_US or IT_###_MS.
    @param [in] arg_digital_gain_bm Bitmaks for digital gain. Possible Bitmask looks like DGAIN_#.
    @param [in] arg_wait_time_bm Bitmask for wait time. Possible Bitmask looks like WAIT_###_US or WAIT_###_MS.
    @param [in] arg_dark_current_sub_bm Bitmask for dark-current compensation. Possible Bitmask is SUB.
    @param [in] arg_lowpass_filter_enable_bm Bitmask for lowpass-filter channel selection. Possible Bitmask looks like LPFEN#.
    @param [in] arg_lowpass_filter_depth_bm Bitmask for lowpass-filter depth. Possible Bitmask looks like LPFDEPTH_#.
*/
void SFH5721::configureMeasurementSettings(uint8_t arg_analog_gain_bm,
    uint8_t arg_integration_time_bm,
    uint8_t arg_digital_gain_bm,
    uint8_t arg_wait_time_bm,
    uint8_t arg_dark_current_sub_bm,
    uint8_t arg_lowpass_filter_enable_bm,
    uint8_t arg_lowpass_filter_depth_bm
                                          )
{
  // mask bitmasks so nothing gets misconfigured by mistake
  uint8_t local_analog_gain_bm = arg_analog_gain_bm & (0x7 << 0);
  uint8_t local_integration_time_bm = arg_integration_time_bm & (0xF << 3);
  uint8_t local_digital_gain_bm = arg_digital_gain_bm & (0x7 << 5);
  uint8_t local_wait_time_bm = arg_wait_time_bm & (0xF << 0);
  uint8_t local_dark_current_sub_bm = arg_dark_current_sub_bm & (0x1 << 4);
  uint8_t local_lowpass_filter_enable_bm = arg_lowpass_filter_enable_bm & (0xF << 0);
  uint8_t local_lowpass_filter_depth_bm = arg_lowpass_filter_depth_bm & (0x3 << 4);




  // extract analog gain, integration-time and digital gain from bitmasks
  switch (local_analog_gain_bm) {
    case AGAIN_1 :
      analog_gain = 1;
      analog_gain_index = 0;
      break;
    case AGAIN_4 :
      analog_gain = 4;
      analog_gain_index = 1;
      break;
    case AGAIN_16 :
      analog_gain = 16;
      analog_gain_index = 2;
      break;
    case AGAIN_128 :
      analog_gain = 128;
      analog_gain_index = 3;
      break;
    case AGAIN_256 :
      analog_gain = 256;
      analog_gain_index = 4;
      break;
    default :
      analog_gain = 0;
      analog_gain_index = 5;
      break;
  }

  integration_time = (local_integration_time_bm >> 3);
  integration_time_index = integration_time;
  digital_gain = (1 << (local_digital_gain_bm >> 5));
  digital_gain_index = (local_digital_gain_bm >> 5);

  // store bitmasks
  SFH5721::analog_gain_bm = local_analog_gain_bm;
  SFH5721::integration_time_bm = local_integration_time_bm;
  SFH5721::digital_gain_bm = local_digital_gain_bm;
  SFH5721::wait_time_bm = local_wait_time_bm;
  SFH5721::dark_current_sub_bm = local_dark_current_sub_bm;
  SFH5721::lowpass_filter_enable_bm = local_lowpass_filter_enable_bm;
  SFH5721::lowpass_filter_depth_bm = local_lowpass_filter_depth_bm;

  // send settings to ALS-Chip
  SFH5721::_I2C_8_write_register(MCONFA, local_analog_gain_bm | local_integration_time_bm);
  SFH5721::_I2C_8_write_register(MCONFB, local_digital_gain_bm | local_wait_time_bm | local_dark_current_sub_bm);
  SFH5721::_I2C_8_write_register(MCONFC, local_lowpass_filter_enable_bm | local_lowpass_filter_depth_bm);
}





/** Configure the mode of operation of the ALS-Chip.
    Settings are stored in class and sent to the ALS-Chip via _I2C_8_write_register.
  @see SFH5721.h for possible bitmasks defines.
    @param [in] arg_mode_of_operation_bm Bitmask for mode of operation. Possible Bitmasks are SINGLE / CONTINUOUS and SYNC / ASYNC.
    @param [in] arg_channel_enable_bm Bitmask for channel selection. Possible Bitmask looks like EN#.
*/
void SFH5721::configureOperationMode(uint8_t arg_mode_of_operation_bm,
                                     uint8_t arg_channel_enable_bm
                                    )
{
  uint8_t local_mode_of_operation_bm = arg_mode_of_operation_bm & (0x3 << 4);
  uint8_t local_channel_enable_bm = arg_channel_enable_bm & (0xF << 0);

  SFH5721::single_mode = ((local_mode_of_operation_bm & SINGLE) != 0);
  SFH5721::synchronous_mode = ((local_mode_of_operation_bm & SYNC) != 0);

  SFH5721::mode_of_operation_bm = local_mode_of_operation_bm;
  SFH5721::channel_enable_bm = local_channel_enable_bm;

  SFH5721::_I2C_8_write_register(OPSEL, local_mode_of_operation_bm | local_channel_enable_bm);
}

/** Enables interrupt-functionality of the ALS-Chip for specified channels
    Settings are stored in class and sent to the ALS-Chip via _I2C_8_write_register.
  @see SFH5721.h for possible bitmasks defines.
    @param [in] arg_interrupt_enable_bm Bitmask for channel selection. Possible Bitmask looks like IENL# and IENH#.
*/
void SFH5721::enableInterrupt(uint8_t arg_interrupt_enable_bm)
{
  uint8_t local_interrupt_enable_bm = arg_interrupt_enable_bm & (0x3F << 0);

  SFH5721::interrupt_enable_bm = local_interrupt_enable_bm;
  SFH5721::interrupt_mode = (local_interrupt_enable_bm != 0);
  SFH5721::_I2C_8_write_register(IEN, local_interrupt_enable_bm);
}


/** Disables interrupt-functionality of the ALS-Chip for all channels
    Settings are stored in class and sent to the ALS-Chip via _I2C_8_write_register.
*/
void SFH5721::disableInterrupt()
{
  SFH5721::interrupt_enable_bm = 0;
  SFH5721::interrupt_mode = 0;
  SFH5721::_I2C_8_write_register(IEN, 0x0);
}



/** Sets interrupt thresholds of the ALS-Chip for specified channels
    Settings are sent to the ALS-Chip via _I2C_8_write_register.
    @param [in] arg_low_threshold Value of low-threshold for interrupt generation in percent of max ADC scale
    @param [in] arg_high_threshold Value of high-threshold for interrupt generation in percent of max ADC scale
*/
void SFH5721::configureInterruptThresholds(uint8_t arg_low_threshold,
    uint8_t arg_high_threshold
                                          )
{
  uint16_t local_arg_low_threshold = 0;
  uint16_t local_arg_high_threshold = 0;

  float tmp = 0;

	// calculate threshold values based on given percentage and the maximum value of the adc scale for the given integration time setting
	// values are capped to max 100 %
  if (arg_high_threshold >= 100) {
    local_arg_high_threshold = max_adc_scale[SFH5721::integration_time_index];
  } else {
    tmp = (((float)arg_high_threshold) / 100.0) * (float)max_adc_scale[SFH5721::integration_time_index];
    local_arg_high_threshold = (uint16_t)tmp;
  }

  if (arg_low_threshold >= 100) {
    local_arg_low_threshold = max_adc_scale[SFH5721::integration_time_index];
  } else {
    tmp = (((float)arg_low_threshold) / 100.0) * (float)max_adc_scale[SFH5721::integration_time_index];
    local_arg_low_threshold = (uint16_t)tmp;
  }




  SFH5721::low_threshold_perc = arg_low_threshold;
  SFH5721::high_threshold_perc = arg_high_threshold;

  SFH5721::low_threshold_bm = local_arg_low_threshold;
  SFH5721::high_threshold_bm = local_arg_high_threshold;

  SFH5721::_I2C_8_write_register(ILTL, (local_arg_low_threshold & 0xFF));
  SFH5721::_I2C_8_write_register(ILTH, ((local_arg_low_threshold & 0xFF00) >> 8));
  SFH5721::_I2C_8_write_register(IHTL, (local_arg_high_threshold & 0xFF));
  SFH5721::_I2C_8_write_register(IHTH, ((local_arg_high_threshold & 0xFF00) >> 8));
}


/** Configures interrupt settings for persistence, generation conditions and clear conditions
    Settings are stored in class and sent to the ALS-Chip via _I2C_8_write_register.
  @see SFH5721.h for possible bitmasks defines.
    @param [in] arg_persistence_bm Bitmask for persistence. Possible Bitmask looks like IPERS_#.
    @param [in] arg_or_and_bm Bitmask for generation condition. Possible Bitmask is INT_WHEN_ALL and INT_WHEN_ONE.
    @param [in] arg_clear_condition_bm Bitmask for clear condition. Possible Bitmask is INT_CLEAR_ON_IFLAGS_READ and INT_CLEAR_ON_DATA_READ.
*/
void SFH5721::configureInterrupt(uint8_t arg_persistence_bm,
                                 uint8_t arg_or_and_bm,
                                 uint8_t arg_clear_condition_bm
                                )
{
  uint8_t local_persistence_bm = arg_persistence_bm & (0xF << 0);
  uint8_t local_or_and_bm = arg_or_and_bm & (0x1 << 4);
  uint8_t local_clear_condition_bm = arg_clear_condition_bm & (0x1 << 5);

  SFH5721::persistence_bm = local_persistence_bm;
  SFH5721::or_and_bm = local_or_and_bm;
  SFH5721::clear_condition_bm = local_clear_condition_bm;

  SFH5721::_I2C_8_write_register(ICONF, local_persistence_bm | local_or_and_bm | local_clear_condition_bm);
}


/** Resets the ALS-Chip by setting the RESET-Flag in its OPSEL-Register.
    All registers are set to reset value.
    Local values are reset.
*/
void SFH5721::reset()
{
  SFH5721::_I2C_8_write_register(OPSEL, RESET);


  SFH5721::digital_gain = 0;
  SFH5721::analog_gain = 0;
  SFH5721::integration_time = 0;

  SFH5721::single_mode = false;
  SFH5721::synchronous_mode = false;
  SFH5721::interrupt_mode = false;

  SFH5721::analog_gain_bm = 0;
  SFH5721::integration_time_bm = IT_50_MS;
  SFH5721::digital_gain_bm = 0;
  SFH5721::wait_time_bm = 0;
  SFH5721::dark_current_sub_bm = SUB;
  SFH5721::lowpass_filter_enable_bm = 0;
  SFH5721::lowpass_filter_depth_bm = 0;
  SFH5721::persistence_bm = 0;
  SFH5721::or_and_bm = 0;
  SFH5721::clear_condition_bm = 0;
  SFH5721::interrupt_enable_bm = 0;

  SFH5721::low_threshold_bm = 0x0000;
  SFH5721::high_threshold_bm = 0xFFFF;

  SFH5721::low_threshold_perc = 0;
  SFH5721::high_threshold_perc = 100;

  SFH5721::mode_of_operation_bm = 0;
  SFH5721::channel_enable_bm = 0;

  SFH5721::analog_gain_index = 0;
  SFH5721::digital_gain_index = 0;
  SFH5721::integration_time_index = 0;



}


/** Verifies the setup of the ALS-Chip by reading out all configurable registers and comparing them to the values stored in the class.
  @return Zero if verification was successful.
*/
uint8_t SFH5721::verifyConfiguration()
{
  uint8_t ret = 0;
  uint8_t val = 0;

  ret = SFH5721::_I2C_8_read_register(OPSEL, &val);
  if (val != (SFH5721::mode_of_operation_bm | SFH5721::channel_enable_bm))
    ret |= RETURN_VERIFICATION_FAILED_ERROR;

  ret |= SFH5721::_I2C_8_read_register(IEN, &val);
  if (val != SFH5721::interrupt_enable_bm)
    ret |= RETURN_VERIFICATION_FAILED_ERROR;

  ret |= SFH5721::_I2C_8_read_register(ICONF, &val);
  if (val != (SFH5721::persistence_bm | SFH5721::clear_condition_bm | SFH5721::or_and_bm))
    ret |= RETURN_VERIFICATION_FAILED_ERROR;

  ret |= SFH5721::_I2C_8_read_register(ILTL, &val);
  if (val != (SFH5721::low_threshold_bm & 0x00FF))
    ret |= RETURN_VERIFICATION_FAILED_ERROR;

  ret |= SFH5721::_I2C_8_read_register(ILTH, &val);
  if (val != ((SFH5721::low_threshold_bm & 0xFF00) >> 8))
    ret |= RETURN_VERIFICATION_FAILED_ERROR;

  ret |= SFH5721::_I2C_8_read_register(IHTL, &val);
  if (val != (SFH5721::high_threshold_bm & 0x00FF))
    ret |= RETURN_VERIFICATION_FAILED_ERROR;

  ret |= SFH5721::_I2C_8_read_register(IHTH, &val);
  if (val != ((SFH5721::high_threshold_bm & 0xFF00) >> 8))
    ret |= RETURN_VERIFICATION_FAILED_ERROR;

  ret |= SFH5721::_I2C_8_read_register(MCONFA, &val);
  if (val != (SFH5721::analog_gain_bm | SFH5721::integration_time_bm))
    ret |= RETURN_VERIFICATION_FAILED_ERROR;

  ret |= SFH5721::_I2C_8_read_register(MCONFB, &val);
  if (val != (SFH5721::digital_gain_bm | SFH5721::wait_time_bm | SFH5721::dark_current_sub_bm))
    ret |= RETURN_VERIFICATION_FAILED_ERROR;

  ret |= SFH5721::_I2C_8_read_register(MCONFC, &val);
  if (val != (SFH5721::lowpass_filter_enable_bm | SFH5721::lowpass_filter_depth_bm))
    ret |= RETURN_VERIFICATION_FAILED_ERROR;

  return ret;
}


/** Enables asynchronous continous measurements for all channels
*/
void SFH5721::startContinuousMeasurementAsync()
{
  SFH5721::configureOperationMode(ASYNC | CONTINUOUS, EN1 | EN2 | EN3);
}


/** Disables asynchronous continous measurements for all channels
*/
void SFH5721::stopContinuousMeasurementAsync()
{
  SFH5721::configureOperationMode(ASYNC | CONTINUOUS, 0);
}


/** Waits for the next continuous asynchronous conversion to finish, gets data for infrared and ALS channel and writes them to the specified addresses.
    Continuous asynchronous mode has to be enabled to use this function.
    @param [out] ALS Address to write ALS value to
    @param [out] IR  Address to write IR value to
  @return Zero if no error occurred
*/
uint8_t SFH5721::readContinuousDataAsync(float * ALS,
    float * IR
                                        )
{
  uint8_t ret = 0;
  uint8_t val = 0;
  if (SFH5721::interrupt_mode == true && SFH5721::persistence_bm == IPERS_EC) {
    while ((GPIO_read)(SFH5721::INT_Pin));
  } else {
    while (val != 0x07) {
      //ret |= SFH5721::_I2C_8_read_register(STAT, &val);
      ret |= readDataReadyBits(&val);
      val &= 0x07;
    }
  }
  ret |= SFH5721::readMeasurementALS(ALS);
  ret |= SFH5721::readMeasurementIR(IR);
  return ret;
}


/** Performes a asynchronous one-shot measurement, gets data for infrared and ALS channel and writes them to the specified addresses.
    @param [out] ALS Address to write ALS value to
    @param [out] IR  Address to write IR value to
  @return Zero if no error occurred
*/
uint8_t SFH5721::oneShotMeasurementAsync(float * ALS,
    float * IR
                                        )
{
  uint8_t ret = 0;
  uint8_t val = 0;
  SFH5721::configureOperationMode(ASYNC | SINGLE, EN1 | EN2 | EN3);
  // wait until all data-ready-bits in status register are set
  while (val != 0x07) {
    //ret |= SFH5721::_I2C_8_read_register(STAT, &val);
    ret |= readDataReadyBits(&val);
    val &= 0x07;
  }
  ret |= SFH5721::readMeasurementALS(ALS);
  ret |= SFH5721::readMeasurementIR(IR);
  return ret;
}


/** Enables synchronous continous measurements for all channels
*/
void SFH5721::startContinuousMeasurementSync()
{
  SFH5721::configureOperationMode(SYNC | CONTINUOUS, EN1 | EN2 | EN3);
}


/** Disables synchronous continous measurements for all channels
*/
void SFH5721::stopContinuousMeasurementSync()
{
  SFH5721::configureOperationMode(SYNC | CONTINUOUS, 0);
}


/** Starts next continuous synchronous conversion with negative edge on INT_Pin, gets data for infrared and ALS channel and writes them to the specified addresses.
    Continuous synchronous mode has to be enabled to use this function
    @param [out] ALS Address to write ALS value to
    @param [out] IR  Address to write IR value to
  @return Zero if no error occurred
*/
uint8_t SFH5721::readContinuousDataSync(float * ALS,
                                        float * IR
                                       )
{
  uint8_t ret = 0;
  uint8_t val = 0;
  (GPIO_write)(SFH5721::INT_Pin, 1);
  _shortDelay(1); // wait for one cycle so that pin gets actually set
  (GPIO_write)(SFH5721::INT_Pin, 0);
  if (SFH5721::interrupt_mode == true && SFH5721::persistence_bm == IPERS_EC) {
    while ((GPIO_read)(SFH5721::INT_Pin));
  } else {
    while (val != 0x07) {
      //ret |= SFH5721::_I2C_8_read_register(STAT, &val);
      ret |= readDataReadyBits(&val);
      val &= 0x07;
    }
  }
  ret |= SFH5721::readMeasurementALS(ALS);
  ret |= SFH5721::readMeasurementIR(IR);
  return ret;
}


/** Performes a synchronous one-shot measurement, gets data for infrared and ALS channel and writes them to the specified addresses.
    @param [out] ALS Address to write ALS value to
    @param [out] IR  Address to write IR value to
  @return Zero if no error occurred
*/
uint8_t SFH5721::oneShotMeasurementSync(float * ALS,
                                        float * IR
                                       )
{
  uint8_t ret = 0;
  uint8_t val = 0;
  (GPIO_write)(SFH5721::INT_Pin, 1);
  SFH5721::configureOperationMode(SYNC | SINGLE, EN1 | EN2 | EN3);
  (GPIO_write)(SFH5721::INT_Pin, 0);
  while (val != 0x07) {
    //ret |= SFH5721::_I2C_8_read_register(STAT, &val);
    ret |= readDataReadyBits(&val);
    val &= 0x07;
  }
  ret |= SFH5721::readMeasurementALS(ALS);
  ret |= SFH5721::readMeasurementIR(IR);
  return ret;
}



/** Reads the binary measurement value for a specified channel from the ALS-Chip.
    @param [in] channel Number of channel to read the measurement value from
  @param [out] count Raw binary value read from the ALS-Chip
    @return Zero if no error occurred
*/
uint8_t SFH5721::readMeasurementValueRaw(uint8_t channel,
    uint16_t * count
                                        )
{
  uint8_t lsb = 0;
  uint8_t msb = 0;
  uint8_t ret = 0;
  uint16_t val = 0;

  switch (channel)
  {
    case 1 :
      ret |= SFH5721::_I2C_8_read_register(DATA1L, &lsb);
      ret |= SFH5721::_I2C_8_read_register(DATA1H, &msb);
      break;
    case 2 :
      ret |= SFH5721::_I2C_8_read_register(DATA2L, &lsb);
      ret |= SFH5721::_I2C_8_read_register(DATA2H, &msb);
      break;
    case 3 :
      ret |= SFH5721::_I2C_8_read_register(DATA3L, &lsb);
      ret |= SFH5721::_I2C_8_read_register(DATA3H, &msb);
      break;
    default :
      ret |= RETURN_CHANNEL_OUT_OF_RANGE_ERROR;
      break;
  }

  val = msb;
  val <<= 8;
  val |= lsb;
  *count = val;
  return ret;
}


/** Reads the binary measurement value for the ALS channel from the Chip and converts it to Lux.
    @param [out] ALS Pointer to store measured value for Ev in Lux
  @return Zero if no error occurred
*/
uint8_t SFH5721::readMeasurementALS(float * ALS)
{
  uint16_t val = 0;
  uint8_t ret = 0;
  ret |= readMeasurementValueRaw(ALS_CHANNEL, &val);
  if (digital_gain == 0 || analog_gain == 0) {
    ret |= RETURN_ONE_GAIN_ZERO_ERROR;
    *ALS = -1;
  } else {
    *ALS = (512.0 *  (float)val / (float)((float)digital_gain * (float)analog_gain * (float)(1 << integration_time)));
  }
  return ret;
}


/** Reads the binary measurement value for the infrared channel from the Chip and converts it to (uW)/(cm^2).
    @param [out] IR Pointer to store measured value for Ee in (uW)/(cm^2).
  @return Zero if no error occurred
*/
uint8_t SFH5721::readMeasurementIR(float * IR)
{
  uint16_t val = 0;
  uint8_t ret = 0;
  ret |= readMeasurementValueRaw(IR_CHANNEL, &val);
  if (digital_gain == 0 || analog_gain == 0) {
    ret |= RETURN_ONE_GAIN_ZERO_ERROR;
    *IR = -1;
  } else {
    *IR = (100.0 *  (float)val / (float)((float)digital_gain * (float)analog_gain * (float)(1 << integration_time)));
  }
  return ret;
}


/** Reads the data-ready bits from the STAT-Register
  @param [out] buffer Bitmask for the data-ready bits
    @return Zero if no error occurred
*/
uint8_t SFH5721::readDataReadyBits(uint8_t * buffer)
{
  uint8_t val = 0;
  uint8_t ret = 0;
  ret |= SFH5721::_I2C_8_read_register(STAT, &val);
  *buffer = (val & 0xF);
  return ret;
}


/** Reads the interrupt-flag bits from the IFLAG-Register
  @param [out] buffer Bitmask for the interrupt-flag bits
    @return Zero if no error occurred
*/
uint8_t SFH5721::readInterruptBits(uint8_t * buffer)
{
  return SFH5721::_I2C_8_read_register(IFLAG, buffer);
}


/** Tries to find the measurement settings for analog gain and digital automatically that get the adc reading as close as possible to the middle of the low and high threshold.
  This function can be used in combination with the SFH5721 hardware interrupt or can be periodically called.
  If you want to use it with the hardware interrupt you must not set the arg_low_threshold and arg_high_threshold parameters, so they stay on their default values.
  If you want to call the function periodically you can set thresholds in percent of the max ADC scale that have to be crossed for the functions functionality to activate.
    If the last raw measurement value is within low_threshold and high_threshold the function returns instantly.
  The function works as follows:
  If the reading is higher than the optimum, measurements are taken with decreasing digital gain as long as the digital gain can be decreased and the measured reading is in saturation. If adc is still saturated after that the same is done with the analog gain.
  As soon as a valid reading is made the function does not take any measurements but estimates which setting to choose based on how the different settings would affect the reading (half the gain means half the reading). This is done as long as it gets you closer to the optimum.
  If the reading is less than the optimum, measurements are taking with increasing analog gain as long as the analog gain can be increaset and the measured reading is less than OPT_LOW_VALID_READING. If the reading is still less than that after this procedure the same is done with the digital gain.
  As soon as a valid reading is made the function does not take any measurements but estimates which setting to choose based on how the different settings would affect the reading (double the gain means double the reading). This is done as long as it gets you closer to the optimum.
  After that the found settings are applied to the SFH5721 chip and stored in class.
    @param [in] arg_low_threshold Low threshold adc reading for premature return from function in percent of max ADC scale
    @param [in] arg_high_threshold High threshold adc reading for premature return from function in percent of max ADC scale
    @param [in] channel Number of the channel to use for optimization. Number from 1 .. 3 or define ALS_CHANNEL, IR_CHANNEL.
  @return Zero if no error occurred
*/
uint8_t SFH5721::autoGain(uint8_t channel,
                          uint8_t arg_low_threshold,
                          uint8_t arg_high_threshold
                         )
{
  uint8_t ret = 0;
  uint8_t val = 0;

  uint16_t reading = 0;
  uint16_t optimum = 0;

  uint8_t local_analog_gain_index = SFH5721::analog_gain_index;
  uint8_t local_digital_gain_index = SFH5721::digital_gain_index;

  uint8_t local_integration_time_bm = SFH5721::integration_time_bm;
  uint8_t local_wait_time_bm = SFH5721::wait_time_bm;
  uint8_t local_dark_current_sub_bm = SFH5721::dark_current_sub_bm;
  uint8_t local_lowpass_filter_enable_bm = SFH5721::lowpass_filter_enable_bm;
  uint8_t local_lowpass_filter_depth_bm = SFH5721::lowpass_filter_depth_bm;
  uint8_t local_mode_of_operation_bm = SFH5721::mode_of_operation_bm;
  uint8_t local_channel_enable_bm = SFH5721::channel_enable_bm;
  uint8_t local_interrupt_enable_bm = SFH5721::interrupt_enable_bm;

  uint16_t local_arg_low_threshold, local_arg_high_threshold;


  // if default values for arg_low_threshold and arg_high_threshold are used values set for hardware interrupt are used for determining the optimum
  // else the function parameters values are used
  if (arg_low_threshold == 0 && arg_high_threshold == 0) {
    optimum = (SFH5721::low_threshold_bm + SFH5721::high_threshold_bm) / 2;
  } else {

    float tmp = 0;
    if (arg_high_threshold >= 100) {
      local_arg_high_threshold = max_adc_scale[SFH5721::integration_time_index];
    } else {
      tmp = (((float)arg_high_threshold) / 100.0) * (float)max_adc_scale[SFH5721::integration_time_index];
      local_arg_high_threshold = (uint16_t)tmp;
    }

    if (arg_low_threshold >= 100) {
      local_arg_low_threshold = max_adc_scale[SFH5721::integration_time_index];
    } else {
      tmp = (((float)arg_low_threshold) / 100.0) * (float)max_adc_scale[SFH5721::integration_time_index];
      local_arg_low_threshold = (uint16_t)tmp;
    }

    optimum = (local_arg_low_threshold + local_arg_high_threshold) / 2;
  }


  // if within specified boundaries, return
  ret |= SFH5721::readMeasurementValueRaw(channel, &reading);


  // if no default values are used, check if its necessary to return
  if (arg_low_threshold != 0 && arg_high_threshold != 0) {
    if (reading > local_arg_low_threshold && reading < local_arg_high_threshold) return 0;
  }

  // disable Interrupts
  SFH5721::disableInterrupt();


  // in which direction to optimize
  if (reading < optimum) {


    // increase analog gain until you get a reading higher than OPT_LOW_VALID_READING
    while (local_analog_gain_index < OPT_AGAIN_MAX_IDX && reading < OPT_LOW_VALID_READING) {

      //------------------- perform one shot measurement -------
      SFH5721::configureMeasurementSettings(analog_gain_arr[local_analog_gain_index], local_integration_time_bm, digital_gain_arr[local_digital_gain_index]);
      SFH5721::configureOperationMode(ASYNC | SINGLE, (0x1 << (channel - 1)));
      val = 0;
      while (val != (0x1 << (channel - 1))) { // wait until conversion is done
        //ret |= SFH5721::_I2C_8_read_register(STAT, &val);
        ret |= readDataReadyBits(&val);
        val &= (0x1 << (channel - 1));
      }
      ret |= SFH5721::readMeasurementValueRaw(channel, &reading);
      //---------------------------------------------------------

      if ( reading > OPT_LOW_VALID_READING )
        break; // stop as soon as the valid so you can estimate the rest

      local_analog_gain_index ++;
    }


    unsigned long tmp, tmp_old;
    unsigned long ul_reading = reading;

    if (optimum > ul_reading) {
      tmp_old = (optimum - ul_reading);
    } else {
      tmp_old = (ul_reading - optimum);
    }

    // estimate a analog gain setting to get into certain bounderies around the middle of the scale
    while (local_analog_gain_index < OPT_AGAIN_MAX_IDX) { // break if an overflow would occur while shifting
      ul_reading *= ((analog_gain_arr_val[local_analog_gain_index + 1] + 1) / (analog_gain_arr_val[local_analog_gain_index] + 1));
      if (optimum > ul_reading) {
        tmp = (optimum - ul_reading);
      } else {
        tmp = (ul_reading - optimum);
      }
      if (tmp < tmp_old) {
        local_analog_gain_index ++;
        tmp_old = tmp;
      } else {
        ul_reading /= ((analog_gain_arr_val[local_analog_gain_index + 1] + 1) / (analog_gain_arr_val[local_analog_gain_index] + 1));
        break;
      }
    }




    // increase digital gain until you get a reading higher than OPT_LOW_VALID_READING
    while (local_digital_gain_index < OPT_DGAIN_MAX_IDX && reading < OPT_LOW_VALID_READING) {

      //------------------- perform one shot measurement -------
      SFH5721::configureMeasurementSettings(analog_gain_arr[local_analog_gain_index], local_integration_time_bm, digital_gain_arr[local_digital_gain_index]);
      SFH5721::configureOperationMode(ASYNC | SINGLE, (0x1 << (channel - 1)));
      val = 0;
      while (val != (0x1 << (channel - 1))) { // wait until conversion is done
        //ret |= SFH5721::_I2C_8_read_register(STAT, &val);
        ret |= readDataReadyBits(&val);
        val &= (0x1 << (channel - 1));
      }
      ret |= SFH5721::readMeasurementValueRaw(channel, &reading);
      //---------------------------------------------------------
      ul_reading = reading;
      if ( reading > OPT_LOW_VALID_READING )
        break; // stop as soon as the valid so you can estimate the rest

      local_digital_gain_index ++;
    }


    if (optimum > ul_reading) {
      tmp_old = (optimum - ul_reading);
    } else {
      tmp_old = (ul_reading - optimum);
    }



    // estimate a digital gain again to compensate if analog gain adjustment overshot the optimum
    while (local_digital_gain_index < OPT_DGAIN_MAX_IDX) { // break if an overflow would occur while shifting
      ul_reading *= 2;
      if (optimum > ul_reading) {
        tmp = (optimum - ul_reading);
      } else {
        tmp = (ul_reading - optimum);
      }
      if (tmp < tmp_old) {
        local_digital_gain_index ++;
        tmp_old = tmp;
      } else {
        ul_reading /= 2;
        break;
      }
    }



  } else {




    // increase analog gain until you get a reading different from 0xFFFF
    while (local_digital_gain_index > 0 && reading == max_adc_scale[SFH5721::integration_time_index]) {

      //------------------- perform one shot measurement -------
      SFH5721::configureMeasurementSettings(analog_gain_arr[local_analog_gain_index], local_integration_time_bm, digital_gain_arr[local_digital_gain_index]);
      SFH5721::configureOperationMode(ASYNC | SINGLE, (0x1 << (channel - 1)));
      val = 0;
      while (val != (0x1 << (channel - 1))) { // wait until conversion is done
        //ret |= SFH5721::_I2C_8_read_register(STAT, &val);
        ret |= readDataReadyBits(&val);
        val &= (0x1 << (channel - 1));
      }
      ret |= SFH5721::readMeasurementValueRaw(channel, &reading);
      //---------------------------------------------------------

      if ( reading < max_adc_scale[SFH5721::integration_time_index] )
        break; // stop as soon as the valid so you can estimate the rest

      local_digital_gain_index --;
    }


    unsigned long tmp, tmp_old;
    unsigned long ul_reading = reading;

    if (optimum > ul_reading) {
      tmp_old = (optimum - ul_reading);
    } else {
      tmp_old = (ul_reading - optimum);
    }


    // estimate a digital gain setting to get into certain bounderies around the middle of the scale
    while (local_digital_gain_index > 0) { // break if an overflow would occur while shifting
      ul_reading /= 2;
      if (optimum > ul_reading) {
        tmp = (optimum - ul_reading);
      } else {
        tmp = (ul_reading - optimum);
      }
      if (tmp < tmp_old) {
        local_digital_gain_index --;
        tmp_old = tmp;
      } else {
        ul_reading *= 2;
        break;
      }
    }

    while (local_analog_gain_index > 0 && reading == max_adc_scale[SFH5721::integration_time_index]) {

      //------------------- perform one shot measurement -------
      SFH5721::configureMeasurementSettings(analog_gain_arr[local_analog_gain_index], local_integration_time_bm, digital_gain_arr[local_digital_gain_index]);
      SFH5721::configureOperationMode(ASYNC | SINGLE, (0x1 << (channel - 1)));
      val = 0;
      while (val != (0x1 << (channel - 1))) { // wait until conversion is done
        //ret |= SFH5721::_I2C_8_read_register(STAT, &val);
        ret |= readDataReadyBits(&val);
        val &= (0x1 << (channel - 1));
      }
      ret |= SFH5721::readMeasurementValueRaw(channel, &reading);
      //---------------------------------------------------------
      ul_reading = reading;
      if ( reading < max_adc_scale[SFH5721::integration_time_index] )
        break; // stop as soon as the valid so you can estimate the rest

      local_analog_gain_index --;
    }


    if (optimum > ul_reading) {
      tmp_old = (optimum - ul_reading);
    } else {
      tmp_old = (ul_reading - optimum);
    }


    // estimate a analog gain setting to get into certain bounderies around the middle of the scale
    while (local_analog_gain_index > 0) { // break if an overflow would occur while shifting
      ul_reading /= ((analog_gain_arr_val[local_analog_gain_index] + 1) / (analog_gain_arr_val[local_analog_gain_index - 1] + 1));
      if (optimum > ul_reading) {
        tmp = (optimum - ul_reading);
      } else {
        tmp = (ul_reading - optimum);
      }
      if (tmp < tmp_old) {
        local_analog_gain_index --;
        tmp_old = tmp;
      } else {
        ul_reading *= ((analog_gain_arr_val[local_analog_gain_index] + 1) / (analog_gain_arr_val[local_analog_gain_index - 1] + 1));
        break;
      }
    }

    while (local_digital_gain_index < OPT_DGAIN_MAX_IDX) {
      ul_reading *= 2;
      if (optimum > ul_reading) {
        tmp = (optimum - ul_reading);
      } else {
        tmp = (ul_reading - optimum);
      }
      if (tmp < tmp_old) {
        local_digital_gain_index ++;
        tmp_old = tmp;
      } else {
        ul_reading /= 2;
        break;
      }
    }
  }

  SFH5721::configureMeasurementSettings(analog_gain_arr[local_analog_gain_index], local_integration_time_bm, digital_gain_arr[local_digital_gain_index],
                                        local_wait_time_bm, local_dark_current_sub_bm, local_lowpass_filter_enable_bm, local_lowpass_filter_depth_bm);
  // restore old mode of operation, interrupt and channel settings
  SFH5721::configureOperationMode(local_mode_of_operation_bm, local_channel_enable_bm);
  SFH5721::enableInterrupt(local_interrupt_enable_bm);

  return ret;

}



/** Reads a 8-Bit register over I2C.
   @param [in] adress Adress of register to read from
   @param [out] read_buffer Pointer to buffer to write received data to
   @return Zero if no error occurred
*/
uint8_t SFH5721::_I2C_8_read_register(uint8_t adress,
                                      uint8_t * read_buffer
                                     )
{
  uint8_t ret = 1;
  ret = (SFH5721::I2C_read_byte)(SFH5721::I2C_Adress, adress, read_buffer, 1);
  if (ret != 1) {
    ret = RETURN_I2C_ERROR;
  } else {
    ret = RETURN_OK;
  }
  return ret;
}


/** Writes to a 8-Bit register over I2C
   @param [in] adress Adress of register to write to
   @param [in] write_buffer Value read from register
*/
void SFH5721::_I2C_8_write_register(uint8_t adress,
                                    uint8_t write_buffer
                                   )
{
  uint8_t local_write_buffer = write_buffer;
  (SFH5721::I2C_write_byte)(SFH5721::I2C_Adress, adress, &local_write_buffer, 1);
}


/** Reads N registers over I2C
   @param [in] adress Start adress of registers to read from
   @param [out] read_buffer Adress of buffer to write read bytes to
   @param [in] number_of_bytes Number of bytes to read
   @return Value Number of bytes actually read
*/
uint8_t SFH5721::_I2C_N_read_register(uint8_t adress,
                                      uint8_t * read_buffer,
                                      uint8_t number_of_bytes
                                     )
{
  uint8_t val = 0;
  val = (SFH5721::I2C_read_byte)(SFH5721::I2C_Adress, adress, read_buffer, number_of_bytes);
  return val;
}


/** Writes to N registers over I2C
   @param [in] adress Start adress of registers to write to
   @param [in] write_buffer Adress of buffer of data that has to be transmitted
   @param [in] number_of_bytes Number of bytes in write_buffer
*/
void SFH5721::_I2C_N_write_register(uint8_t adress,
                                    uint8_t * write_buffer,
                                    uint8_t number_of_bytes
                                   )
{
  (SFH5721::I2C_write_byte)(SFH5721::I2C_Adress, adress, write_buffer, number_of_bytes);
}



/** Waits for a specified amount of cycles using asm NOP operations
   @param [in] number_of_cycles Number of cycles to wait
*/
void SFH5721::_shortDelay(uint8_t number_of_cycles)
{
  for (uint8_t i = 0; i < number_of_cycles; i++) {
    __asm__("nop\n\t");
  }

}
