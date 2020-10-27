/*!
 LTC2992: Dual Wide Range Power Monitor

@verbatim

The LTC®2992 is a rail-to-rail system monitor that measures
current, voltage, power, charge and energy. It features an
operating range of 2.7V to 100V and includes a shunt regulator
for supplies above 100V. The current measurement common mode
range of 0V to 100V is independent of the input supply.
A 12-bit ADC measures load current, input voltage and an
auxiliary external voltage. Load current and internally
calculated power are integrated over an external clock or
crystal or internal oscillator time base for charge and energy.
An accurate time base allows the LTC2992 to provide measurement
accuracy of better than ±0.6% for charge and ±1% for power and
energy. Minimum and maximum values are stored and an overrange
alert with programmable thresholds minimizes the need for software
polling. Data is reported via a standard I2C interface.
Shutdown mode reduces power consumption to 15uA.


I2C DATA FORMAT (MSB FIRST):

Data Out:
Byte #1                                    Byte #2                     Byte #3

START  SA6 SA5 SA4 SA3 SA2 SA1 SA0 W SACK  X  X C5 C4 C3 C2 C1 C0 SACK D7 D6 D5 D4 D3 D2 D1 D0 SACK  STOP

Data In:
Byte #1                                    Byte #2                                    Byte #3

START  SA6 SA5 SA4 SA3 SA2 SA1 SA0 W SACK  X  X  C5 C4 C3 C2 C1 C0 SACK  Repeat Start SA6 SA5 SA4 SA3 SA2 SA1 SA0 R SACK

Byte #4                                   Byte #5
MSB                                       LSB
D15 D14  D13  D12  D11  D10  D9 D8 MACK   D7 D6 D5 D4 D3  D2  D1  D0  MNACK  STOP

START       : I2C Start
Repeat Start: I2c Repeat Start
STOP        : I2C Stop
SAx         : I2C Address
SACK        : I2C Slave Generated Acknowledge (Active Low)
MACK        : I2C Master Generated Acknowledge (Active Low)
MNACK       : I2c Master Generated Not Acknowledge
W           : I2C Write (0)
R           : I2C Read  (1)
Cx          : Command Code
Dx          : Data Bits
X           : Don't care



Example Code:

Read power, current and voltage

    CTRLA = LTC2992_CHANNEL_CONFIG_V_C_3|LTC2992_SENSE_PLUS|LTC2992_OFFSET_CAL_EVERY|LTC2992_ADIN_GND;  //! Set Control A register to default value in continuous mode
    ack |= LTC2992_write(LTC2992_I2C_ADDRESS, LTC2992_CTRLA_REG, CTRLA);   //! Sets the LTC2992 to continuous mode

    resistor = .02; // Resistor Value On Demo Board

    ack |= LTC2992_read_24_bits(LTC2992_I2C_ADDRESS, LTC2992_POWER_MSB2_REG, &power_code);  // Reads the ADC registers that contains V^2
    power = LTC2992_code_to_power(power_code, resistor, LTC2992_Power_lsb); // Calculates power from power code, resistor value and power lsb

    ack |= LTC2992_read_12_bits(LTC2992_I2C_ADDRESS, LTC2992_DELTA_SENSE_MSB_REG, &current_code); // Reads the voltage code across sense resistor
    current = LTC2992_code_to_current(current_code, resistor, LTC2992_DELTA_SENSE_lsb); // Calculates current from current code, resistor value and current lsb

    ack |= LTC2992_read_12_bits(LTC2992_I2C_ADDRESS, LTC2992_SENSE_MSB_REG, &SENSE_code);   // Reads SENSE voltage code
    SENSE = LTC2992_SENSE_code_to_voltage(SENSE_code, LTC2992_SENSE_lsb);  // Calculates SENSE voltage from SENSE code and lsb

    ack |= LTC2992_read_32_bits(LTC2992_I2C_ADDRESS, LTC2992_ENERGY_MSB3_REG, &energy_code);  // Reads energy code
  energy = LTC2992_code_to_energy(energy_code,resistor,LTC2992_Power_lsb, LTC2992_INTERNAL_TIME_lsb); //Calculates Energy in Joules from energy_code, resistor, power lsb and time lsb

  ack |= LTC2992_read_32_bits(LTC2992_I2C_ADDRESS, LTC2992_CHARGE_MSB3_REG, &charge_code);  // Reads charge code
    charge = LTC2992_code_to_coulombs(charge_code,resistor,LTC2992_DELTA_SENSE_lsb, LTC2992_INTERNAL_TIME_lsb); //Calculates charge in coulombs from charge_code, resistor, current lsb and time lsb



@endverbatim

http://www.linear.com/product/LTC2992

http://www.linear.com/product/LTC2992#demoboards


Copyright 2018(c) Analog Devices, Inc.

All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
 - Redistributions of source code must retain the above copyright
   notice, this list of conditions and the following disclaimer.
 - Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in
   the documentation and/or other materials provided with the
   distribution.
 - Neither the name of Analog Devices, Inc. nor the names of its
   contributors may be used to endorse or promote products derived
   from this software without specific prior written permission.
 - The use of this software may or may not infringe the patent rights
   of one or more patent holders.  This license does not release you
   from the requirement that you obtain separate licenses from these
   patent holders to use this software.
 - Use of the software either in source or binary form, must be run
   on or directly connected to an Analog Devices Inc. component.

THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR
IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
LIMITED TO, INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

//! @ingroup Power_Monitors
//! @{
//! @defgroup LTC2992 LTC2992: Dual Wide Range Power Monitor
//! @}
/*! @file
    @ingroup LTC2992
    Header for LTC2992: Dual Wide Range Power Monitor

#include <Arduino.h>
#include <stdint.h>
#include "Linduino.h"
#include "LT_I2C.h"
#include "LTC2992.h"
#include <Wire.h>
*/

/***************************/
////FOR I2C comms on TM4C///
#include "LTC2992.h"
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <inc/hw_i2c.h>
#include <inc/hw_memmap.h>
#include <inc/hw_types.h>
#include <inc/hw_gpio.h>
#include <driverlib/i2c.h>
#include <driverlib/sysctl.h>
#include <driverlib/gpio.h>
#include <driverlib/pin_map.h>

//initialize I2C module 0
//Slightly modified version of TI's example code
void InitI2C0(void)
{
    //enable I2C module 0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);
 
    //reset module
    SysCtlPeripheralReset(SYSCTL_PERIPH_I2C0);
     
    //enable GPIO peripheral that contains I2C 0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
 
    // Configure the pin muxing for I2C0 functions on port B2 and B3.
    GPIOPinConfigure(GPIO_PB2_I2C0SCL);
    GPIOPinConfigure(GPIO_PB3_I2C0SDA);
     
    // Select the I2C function for these pins.
    GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);
    GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);
 
    // Enable and initialize the I2C0 master module.  Use the system clock for
    // the I2C0 module.  The last parameter sets the I2C data transfer rate.
    // If false the data rate is set to 100kbps and if true the data rate will
    // be set to 400kbps.
    I2CMasterInitExpClk(I2C0_BASE, SysCtlClockGet(), false);
     
    //clear I2C FIFOs
    HWREG(I2C0_BASE + I2C_O_FIFOCTL) = 80008000;
}

//sends an I2C command to the specified slave
void I2CSend(uint8_t slave_addr, uint8_t num_of_args, ...)
{
    // Tell the master module what address it will place on the bus when
    // communicating with the slave.
    I2CMasterSlaveAddrSet(I2C0_BASE, slave_addr, false);
     
    //stores list of variable number of arguments
    va_list vargs;
     
    //specifies the va_list to "open" and the last fixed argument
    //so vargs knows where to start looking
    va_start(vargs, num_of_args);
     
    //put data to be sent into FIFO
    I2CMasterDataPut(I2C0_BASE, va_arg(vargs, uint32_t));
     
    //if there is only one argument, we only need to use the
    //single send I2C function
    if(num_of_args == 1)
    {
        //Initiate send of data from the MCU
        I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_SEND);
         
        // Wait until MCU is done transferring.
        while(I2CMasterBusy(I2C0_BASE));
         
        //"close" variable argument list
        va_end(vargs);
    }
     
    //otherwise, we start transmission of multiple bytes on the
    //I2C bus
    else
    {
        //Initiate send of data from the MCU
        I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);
         
        // Wait until MCU is done transferring.
        while(I2CMasterBusy(I2C0_BASE));
         
        //send num_of_args-2 pieces of data, using the
        //BURST_SEND_CONT command of the I2C module
        for(uint8_t i = 1; i < (num_of_args - 1); i++)
        {
            //put next piece of data into I2C FIFO
            I2CMasterDataPut(I2C0_BASE, va_arg(vargs, uint32_t));
            //send next data that was just placed into FIFO
            I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_CONT);
     
            // Wait until MCU is done transferring.
            while(I2CMasterBusy(I2C0_BASE));
        }
     
        //put last piece of data into I2C FIFO
        I2CMasterDataPut(I2C0_BASE, va_arg(vargs, uint32_t));
        //send next data that was just placed into FIFO
        I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
        // Wait until MCU is done transferring.
        while(I2CMasterBusy(I2C0_BASE));
         
        //"close" variable args list
        va_end(vargs);
    }
}

//sends an array of data via I2C to the specified slave
void I2CSendString(uint8_t slave_addr, char array[])
{
    // Tell the master module what address it will place on the bus when
    // communicating with the slave.
    I2CMasterSlaveAddrSet(I2C0_BASE, slave_addr, false);
     
    //put data to be sent into FIFO
    I2CMasterDataPut(I2C0_BASE, array[0]);
     
    //if there is only one argument, we only need to use the
    //single send I2C function
    if(array[1] == '\0')
    {
        //Initiate send of data from the MCU
        I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_SEND);
         
        // Wait until MCU is done transferring.
        while(I2CMasterBusy(I2C0_BASE));
    }
     
    //otherwise, we start transmission of multiple bytes on the
    //I2C bus
    else
    {
        //Initiate send of data from the MCU
        I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);
         
        // Wait until MCU is done transferring.
        while(I2CMasterBusy(I2C0_BASE));
         
        //initialize index into array
        uint8_t i = 1;
 
        //send num_of_args-2 pieces of data, using the
        //BURST_SEND_CONT command of the I2C module
        while(array[i + 1] != '\0')
        {
            //put next piece of data into I2C FIFO
            I2CMasterDataPut(I2C0_BASE, array[i++]);
 
            //send next data that was just placed into FIFO
            I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_CONT);
     
            // Wait until MCU is done transferring.
            while(I2CMasterBusy(I2C0_BASE));
        }
     
        //put last piece of data into I2C FIFO
        I2CMasterDataPut(I2C0_BASE, array[i]);
 
        //send next data that was just placed into FIFO
        I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
 
        // Wait until MCU is done transferring.
        while(I2CMasterBusy(I2C0_BASE));
    }
}

//read specified register on slave device
uint32_t I2CReceive(uint8_t slave_addr, uint8_t reg)
{
    //specify that we are writing (a register address) to the
    //slave device
    I2CMasterSlaveAddrSet(I2C0_BASE, slave_addr, false);
 
    //specify register to be read
    I2CMasterDataPut(I2C0_BASE, reg);
 
    //send control byte and register address byte to slave device
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);
     
    //wait for MCU to finish transaction
    while(I2CMasterBusy(I2C0_BASE));
     
    //specify that we are going to read from slave device
    I2CMasterSlaveAddrSet(I2C0_BASE, slave_addr, true);
     
    //send control byte and read from the register we
    //specified
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);
     
    //wait for MCU to finish transaction
    while(I2CMasterBusy(I2C0_BASE));
     
    //return data pulled from the specified register
    return I2CMasterDataGet(I2C0_BASE);
}
// END TM4C I2C comms
/***************************************************/ 
// START LT_I2C comms
/**************************************************
// Write an 8-bit code to the LTC2992.
int8_t LTC2992_write(uint8_t i2c_address, uint8_t adc_command, uint8_t code)
// The function returns the state of the acknowledge bit after the I2C address write. 0=acknowledge, 1=no acknowledge.
{
  int32_t ack;

  ack = i2c_write_byte_data(i2c_address, adc_command, code);

  return ack;

}

// Write a 16-bit code to the LTC2992.
int8_t LTC2992_write_16_bits(uint8_t i2c_address, uint8_t adc_command, uint16_t code)
// The function returns the state of the acknowledge bit after the I2C address write. 0=acknowledge, 1=no acknowledge.
{
  int8_t ack;

  ack = i2c_write_word_data(i2c_address, adc_command, code);
  return(ack);
}

// Write a 24-bit code to the LTC2992.
int8_t LTC2992_write_24_bits(uint8_t i2c_address, uint8_t adc_command, uint32_t code)
// The function returns the state of the acknowledge bit after the I2C address write. 0=acknowledge, 1=no acknowledge.
{
  int8_t ack;

  LT_union_int32_4bytes data;
  data.LT_int32 = code;

  ack = i2c_write_block_data(i2c_address, adc_command, (uint8_t) 3, data.LT_byte);

  return(ack);
}


// Reads an 8-bit adc_code from LTC2992
int8_t LTC2992_read(uint8_t i2c_address, uint8_t adc_command, uint8_t *adc_code)
// The function returns the state of the acknowledge bit after the I2C address write. 0=acknowledge, 1=no acknowledge.
{
  int32_t ack;

  ack = i2c_read_byte_data(i2c_address, adc_command, adc_code);

  return ack;
}

// Reads a 12-bit adc_code from LTC2992
int8_t LTC2992_read_12_bits(uint8_t i2c_address, uint8_t adc_command, uint16_t *adc_code)
// The function returns the state of the acknowledge bit after the I2C address write. 0=acknowledge, 1=no acknowledge.
{
  // Use union type defined in Linduino.h to combine two uint8_t's (8-bit unsigned integers) into one uint16_t (unsigned 16-bit integer)
  // Then, shift by 4 bits and return in *adc_code
  int32_t ack;

  ack = i2c_read_word_data(i2c_address, adc_command, adc_code);

  *adc_code >>= 4;
  return ack;
}

// Reads a 16-bit adc_code from LTC2992
int8_t LTC2992_read_16_bits(uint8_t i2c_address, uint8_t adc_command, uint16_t *adc_code)
// The function returns the state of the acknowledge bit after the I2C address write. 0=acknowledge, 1=no acknowledge.
{
  int32_t ack;

  ack = i2c_read_word_data(i2c_address, adc_command, adc_code);

  return ack;
}

// Reads a 24-bit adc_code from LTC2992
int8_t LTC2992_read_24_bits(uint8_t i2c_address, uint8_t adc_command, uint32_t *adc_code)
// The function returns the state of the acknowledge bit after the I2C address write. 0=acknowledge, 1=no acknowledge.
{
  int8_t ack;

  LT_union_int32_4bytes data;

  ack = i2c_read_block_data(i2c_address, adc_command, (uint8_t)3, data.LT_byte);

  *adc_code = 0x0FFFFFF & data.LT_int32;
  return(ack);
}

// Calculate the LTC2992 SENSE voltage
float LTC2992_SENSE_code_to_voltage(uint16_t adc_code, float LTC2992_SENSE_lsb)
// Returns the SENSE Voltage in Volts
{
  float voltage;
  voltage = (float)adc_code*LTC2992_SENSE_lsb;    //! 1) Calculate voltage from code and lsb
  return(voltage);
}

// Calculate the LTC2992 GPIO voltage
float LTC2992_GPIO_code_to_voltage(uint16_t adc_code, float LTC2992_GPIO_lsb)
// Returns the GPIO Voltage in Volts
{
  float adc_voltage;
  adc_voltage = (float)adc_code*LTC2992_GPIO_lsb;   //! 1) Calculate voltage from code and ADIN lsb
  return(adc_voltage);
}

// Calculate the LTC2992 current with a sense resistor
float LTC2992_code_to_current(uint16_t adc_code, float resistor, float LTC2992_DELTA_SENSE_lsb)
// Returns the LTC2992 current in Amps
{
  float voltage, current;
  voltage = (float)adc_code*LTC2992_DELTA_SENSE_lsb;    //! 1) Calculate voltage from ADC code and delta sense lsb
  current = voltage/resistor;                           //! 2) Calculate current, I = V/R
  return(current);
}

// Calculate the LTC2992 current with a sense resistor
float LTC2992_code_to_current_sum(uint16_t adc_code, float resistor, float LTC2992_DELTA_SENSE_lsb)
// Returns the LTC2992 current in Amps
{
  float voltage, current;
  voltage = (float)(adc_code<<1)*LTC2992_DELTA_SENSE_lsb;    //! 1) Calculate voltage from ADC code and delta sense lsb
  current = voltage/resistor;                           //! 2) Calculate current, I = V/R
  return(current);
}

// Calculate the LTC2992 power
float LTC2992_code_to_power(int32_t adc_code, float resistor, float LTC2992_Power_lsb)
// Returns The LTC2992 power in Watts
{
  float power;
  power = (float)adc_code*LTC2992_Power_lsb/resistor;  //! 1) Calculate Power using Power lsb and resistor

  return(power);
}

// Calculate the LTC2992 power
float LTC2992_code_to_power_sum(int32_t adc_code, float resistor, float LTC2992_Power_lsb)
// Returns The LTC2992 power in Watts
{
  float power;
  power = (float)(adc_code<<1)*LTC2992_Power_lsb/resistor;  //! 1) Calculate Power using Power lsb and resistor

  return(power);
}
**********/
