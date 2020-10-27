/*
 * Solar_power_monitor.ino
 * 
 * Initiates serial ports & reads LTC2992 data
 *
 */

#include <driverlib/sysctl.h>


/* These are device specific */
#include "src/SolarHSK_lib/Wire.h"
#include "src/SolarHSK_lib/MPPT_dev.h"
#define DOWNBAUD 115200 // Baudrate to the SFC

// MPPT I2C stuff
#define PANEL_UPDATE_PERIOD 5000  // period between reads/writes
unsigned long PanelUpdateTime=0;  // for transmit recording time
#define  WIRE_INTERFACES_COUNT 4
unsigned long  i2c_0=0;
unsigned long i2c_1=1;
unsigned long  i2c_2=2;
unsigned long i2c_3=3;
// declare 4 two wire objs
TwoWire *wire_0= new TwoWire(i2c_0); // i2C object for the i2c port on the launchpad
//TwoWire *wire_1= new TwoWire(i2c_1); // i2C object for the i2c port on the launchpad
//TwoWire *wire_2= new TwoWire(i2c_2); // i2C object for the i2c port on the launchpad
//TwoWire *wire_3= new TwoWire(i2c_3); // i2C object for the i2c port on the launchpad

// 4 MPPT objects (to rule them all)
MPPT mppt_0(*wire_0); // MPPT object for sending commands to the LTC2992 chips (with address per command since 9 chips are on each i2c line) 
//MPPT mppt_1(*wire_1); // MPPT object for sending commands to the LTC2992 chips (with address per command since 9 chips are on each i2c line) 
//MPPT mppt_2(*wire_2); // MPPT object for sending commands to the LTC2992 chips (with address per command since 9 chips are on each i2c line) 
//MPPT mppt_3(*wire_3); // MPPT object for sending commands to the LTC2992 chips (with address per command since 9 chips are on each i2c line) 
uint8_t addr=0x67; // ADR0:H ADR1:L -> 0x67, see MPPT_dev.h for the entire table (or the Datasheet of LTC2992).
uint8_t power1[3];
uint8_t power2[3];
uint8_t sense1[2];
uint8_t sense2[2];
uint8_t current1[2];
uint8_t current2[2];
uint8_t gpio1[2];
uint8_t gpio2[2];
uint8_t gpio3[2];
uint8_t gpio4[2];
int gpio_select=1;
int which_read;

typedef enum {
        ePower1 = 65,
        ePower2 = 66,
        eSense1 = 67,
        eSense2 = 68,
        eCurrent1 = 69,
        eCurrent2 = 70,
        eGPIO1 = 71,  //10
        eGPIO2 = 72,  //11
        eGPIO3 = 73,  //12
        eGPIO4 = 74  //13
} print_state;

/*******************************************************************************
* Main program
*******************************************************************************/
void setup()
{
  Serial.begin(DOWNBAUD);
  Serial.println("Starting...");
// Using Objects
// Start all i2c conns
  wire_0->begin();
//  wire_1->begin();
//  wire_2->begin();
//  wire_3->begin();
  mppt_0.Setup(addr);
  // commented out until ready for full chain
  //Setup_all();
  PanelUpdateTime=millis() + PANEL_UPDATE_PERIOD;
  which_read=1;
}

/*******************************************************************************
 * Main program
 ******************************************************************************/
void loop()
{
  if ((long) (millis() - PanelUpdateTime) > 0){
    PanelUpdateTime = millis() + PANEL_UPDATE_PERIOD;
    read_single(103);
  }
  while(Serial.available()){
    uint8_t command=Serial.read();
    send_me_data(command);
  }
 
}

void send_me_data(uint8_t buff){
  uint8_t buffer[3]={0};
  int retval=0;
  switch(buff){
    case ePower1:{ //65/"A"
      memcpy(buffer,power1,sizeof(power1));
      retval=sizeof(power1);
      break;
    }
    case ePower2:{ //66/"B"
      memcpy(buffer,power2,sizeof(power2));
      retval=sizeof(power2);
      break;
    }
    case eSense1:{ //67/"C"
      memcpy(buffer,sense1,sizeof(sense1));
      retval=sizeof(sense1);
      break;
    }
    case eSense2:{ //68/"D"
      memcpy(buffer,sense2,sizeof(sense2));
      retval=sizeof(sense2);
      break;
    }
    case eCurrent1:{ //69/"E"
      memcpy(buffer,current1,sizeof(current1));
      retval=sizeof(current1);
      break;
    }
    case eCurrent2:{ //70/"F"
      memcpy(buffer,current2,sizeof(current2));
      retval=sizeof(current2);
      break;
    }
    case eGPIO1:{ //71/"G"
      memcpy(buffer,gpio1,sizeof(gpio1));
      retval=sizeof(gpio1);
      break;
    }
    case eGPIO2:{ //72/"H"
      memcpy(buffer,gpio2,sizeof(gpio2));
      retval=sizeof(gpio2);
      break;
    }
    case eGPIO3:{ //73/"I"
      memcpy(buffer,gpio3,sizeof(gpio3));
      retval=sizeof(gpio3);
      break;
    }
    case eGPIO4:{ //74/"J"
      memcpy(buffer,gpio4,sizeof(gpio4));
      retval=sizeof(gpio4);
      break;
    }
  }
  for(int i = 0; i<retval;i++){
    Serial.print(buffer[i],HEX);
  }
  Serial.println("done.");
}


void read_single(uint8_t addr_to_read){
  mppt_0.ReadPower1(addr_to_read,power1);
  mppt_0.ReadPower2(addr_to_read,power2);
  mppt_0.ReadSense1(addr_to_read,sense1);
  mppt_0.ReadSense2(addr_to_read,sense2);
  mppt_0.ReadCurrent1(addr_to_read,current1);
  mppt_0.ReadCurrent2(addr_to_read,current2);
  mppt_0.ReadGPIO(1,addr_to_read,gpio1);
  mppt_0.ReadGPIO(2,addr_to_read,gpio2);
  mppt_0.ReadGPIO(3,addr_to_read,gpio3);
  mppt_0.ReadGPIO(4,addr_to_read,gpio4);
}
