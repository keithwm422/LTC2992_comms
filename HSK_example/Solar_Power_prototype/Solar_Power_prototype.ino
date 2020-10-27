/*
 * SolarHSK_prototype.ino
 * 
 * Initiates serial ports & follows HSK protocol for command responses and error
 * reporting. This program can be used on other devices by changing the device
 * address (myID) and the upStream serial connection (direct line to the SFC)
 *
 * 
 */

#include <Core_protocol.h>
#include <PacketSerial.h>
#include <driverlib/sysctl.h>


/* These are device specific */
#include "src/SolarHSK_lib/Wire.h"
#include "src/SolarHSK_lib/MPPT_dev.h"
#include "src/SolarHSK_lib/SolarHSK_protocol.h"
#include "src/SolarHSK_lib/SolarHSK_support_functions.h"
#define DOWNBAUD 115200 // Baudrate to the SFC
#define TEST_MODE_PERIOD 100 // period in milliseconds between testmode packets being sent
#define FIRST_LOCAL_COMMAND 2 // value of hdr->cmd that is the first command local to the board
#define NUM_LOCAL_CONTROLS 7 // how many commands total are local to the board

/* Declare instances of PacketSerial to set up the serial lines */
PacketSerial downStream1;

/*******************************************************************************
* Defines
*******************************************************************************/
/* Name of this device */
housekeeping_id myID = eSolarHsk;

/* Outgoing buffer, for up or downstream. Only gets used once a complete packet
 * is received -- a command or forward is executed before anything else happens,
 * so there shouldn't be any over-writing here. */
uint8_t outgoingPacket [MAX_PACKET_LENGTH] ={0}; 

/* Use pointers for all device's housekeeping headers and the autopriorityperiods*/
housekeeping_hdr_t * hdr_in;     housekeeping_hdr_t * hdr_out;
housekeeping_err_t * hdr_err;   housekeeping_prio_t * hdr_prio;
/* Memory buffers for housekeeping system functions */
uint8_t numDevices = 0;           // Keep track of how many devices are upstream
uint8_t commandPriority[NUM_LOCAL_CONTROLS] = {0};     // Each command's priority takes up one byte
PacketSerial *serialDevices = &downStream1;
                      // Pointer to an address's serial port
uint8_t addressList = 0; // List of all downstream devices
/* Utility variables for internal use */
uint8_t checkin;    // Used for comparing checksum values
size_t hdr_size = sizeof(housekeeping_hdr_t)/sizeof(hdr_out->src); // size of the header
uint8_t numSends = 0; // Used to keep track of number of priority commands executed
int bus = 0;
uint16_t currentPacketCount=0;
unsigned long timelastpacket;
// MPPT I2C stuff
#define PANEL_UPDATE_PERIOD 1000  // period between reads/writes
unsigned long PanelUpdateTime=0;  // for transmit recording time
#define  WIRE_INTERFACES_COUNT 4
unsigned long  i2c_0=0;
unsigned long i2c_1=1;
unsigned long  i2c_2=2;
unsigned long i2c_3=3;
// declare 4 two wire objs
TwoWire *wire_0= new TwoWire(i2c_0); // i2C object for the i2c port on the launchpad
TwoWire *wire_1= new TwoWire(i2c_1); // i2C object for the i2c port on the launchpad
TwoWire *wire_2= new TwoWire(i2c_2); // i2C object for the i2c port on the launchpad
TwoWire *wire_3= new TwoWire(i2c_3); // i2C object for the i2c port on the launchpad

// 4 MPPT objects (to rule them all)
MPPT mppt_0(*wire_0); // MPPT object for sending commands to the LTC2992 chips (with address per command since 9 chips are on each i2c line) 
MPPT mppt_1(*wire_1); // MPPT object for sending commands to the LTC2992 chips (with address per command since 9 chips are on each i2c line) 
MPPT mppt_2(*wire_2); // MPPT object for sending commands to the LTC2992 chips (with address per command since 9 chips are on each i2c line) 
MPPT mppt_3(*wire_3); // MPPT object for sending commands to the LTC2992 chips (with address per command since 9 chips are on each i2c line) 
uint8_t addr=0x67; // ADR0:H ADR1:L -> 0x67, see MPPT_dev.h for the entire table (or the Datasheet of LTC2992).
uint8_t power1[4][3];
uint8_t power2[4][3];
uint8_t sense1[4][2];
uint8_t sense2[4][2];
uint8_t current1[4][2];
uint8_t current2[4][2];
uint8_t gpio1[4][2];
uint8_t gpio2[4][2];
uint8_t gpio3[4][2];
uint8_t gpio4[4][2];
int gpio_select=1;
int which_read;
/*******************************************************************************
* Main program
*******************************************************************************/
void setup()
{
  // to DEBUG this device when connecting directly to computer, use this serial port instead of Serial.1
//  Serial.begin(DOWNBAUD);
//  downStream1.setStream(&Serial);
//  downStream1.setPacketHandler(&checkHdr);
  Serial1.begin(DOWNBAUD);
  downStream1.setStream(&Serial1);
  downStream1.setPacketHandler(&checkHdr);
// Connect to I2C port of MPPT which is LTC2992 (Orthogonal Systems)
// Using Objects
// Start all i2c conns
  wire_0->begin();
  wire_1->begin();
  wire_2->begin();
  wire_3->begin();
  // Point to data in a way that it can be read as a header
  hdr_out = (housekeeping_hdr_t *) outgoingPacket;
  hdr_err = (housekeeping_err_t *) (outgoingPacket + hdr_size);
  currentPacketCount=0;
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
  /* Continuously read in one byte at a time until a packet is received */
  if (downStream1.update() != 0) badPacketReceived(&downStream1);
  // commented out until ready for full chain
  // read the next slave addr
  // if slave addr >112, reset to 103.
  /*if ((long) (millis() - PanelUpdateTime) > 0){
    PanelUpdateTime = millis() + PANEL_UPDATE_PERIOD;
    read_mppts();
    if(gpio_select>4) gpio_select=1;
    which_read++;
    if(which_read>10) {
      which_read=1;
      addr++;
      if(addr>112) addr=103;
    }
  }
  */
  if ((long) (millis() - PanelUpdateTime) > 0){
    PanelUpdateTime = millis() + PANEL_UPDATE_PERIOD;
    read_single(103);
  }
}

void checkHdr(const void *sender, const uint8_t *buffer, size_t len) {
  // Default header & error data values
  hdr_out->src = myID;          // Source of data packet
  hdr_in = (housekeeping_hdr_t *)buffer;
  hdr_prio = (housekeeping_prio_t *) (buffer + hdr_size);
    // If an error occurs at this device from a message
  if (hdr_in->dst == eBroadcast || hdr_in->dst==myID) hdr_err->dst = myID;
  else hdr_err->dst = hdr_in->dst;
  // If the checksum didn't match, throw a bad args error
  // Check for data corruption
  if (!(verifyChecksum((uint8_t *) buffer))) {
      //error_badArgs(hdr_in, hdr_out, hdr_err);  
      buildError(hdr_err, hdr_out, hdr_in, EBADARGS);
      fillChecksum((uint8_t *) outgoingPacket);
      downStream1.send(outgoingPacket, hdr_size + hdr_out->len + 1);
      currentPacketCount++;
  }
  else {
  // Check if the message is a broadcast or local command and only then execute it. 
    if (hdr_in->dst == eBroadcast || hdr_in->dst==myID) {
      if(hdr_in->cmd==eTestMode) handleTestMode(hdr_in, (uint8_t *) hdr_in + hdr_size, (uint8_t *) outgoingPacket);
      else if ((int)(hdr_in->cmd < 254) && (int)(hdr_in->cmd > 249)) handlePriority(hdr_in->cmd, (uint8_t *) outgoingPacket); // for doing a send of priority type.
      else handleLocalCommand(hdr_in, (uint8_t *) hdr_in + hdr_size, (uint8_t *) outgoingPacket); // this constructs the outgoingpacket when its a localcommand and sends the packet.
    } 
  // If the message wasn't meant for this device pass it along (up is away from SFC and down and is to SFC
    else forwardDown(buffer, len, sender);
  }
}
// forward downstream to the SFC
void forwardDown(const uint8_t * buffer, size_t len, const void * sender) {
  downStream1.send(buffer, len);
  checkDownBoundDst(sender);
  currentPacketCount++;
}

/* checkDownBoundDst Function flow:
 * --Checks to see if the downstream device that sent the message is known
 *    --If not, add it to the list of known devices
 *    --If yes, just carry on
 * --Executed every time a packet is received from downStream
 * 
 * Function params:
 * sender:    PacketSerial instance (serial line) where the message was received
 * 
 */
void checkDownBoundDst(const void * sender) {
  if (serialDevices == (PacketSerial *) sender){
    if (addressList == 0) {
      addressList = (uint8_t) hdr_in->src;
      numDevices++;
      return;
    }
  }
}
/* Function flow:
 * --Find the device address that produced the error
 * --Execute the bad length function & send the error to the SFC
 * Function params:
 * sender:    PacketSerial instance which triggered the error protocol
 * Send an error if a packet is unreadable in some way */
void badPacketReceived(PacketSerial * sender){
  if (sender == serialDevices){
    hdr_in->src = addressList;
  }
  hdr_out->src = myID;
  buildError(hdr_err, hdr_out, hdr_in, EBADLEN);
  fillChecksum((uint8_t *) outgoingPacket);
  downStream1.send(outgoingPacket, hdr_size + hdr_out->len + 1);
  currentPacketCount++;
}

// Function for building the error packets to send back when an error is found (see the Core_Protocol.h for the defs of the errors and the error typdefs).
void buildError(housekeeping_err_t *err, housekeeping_hdr_t *respHdr, housekeeping_hdr_t * hdr, int error){
  respHdr->cmd = eError;
  respHdr->len = 4;
  err->src = hdr->src;
  err->dst = hdr->dst;
  err->cmd = hdr->cmd;
  err->error = error;
}

/*******************************************************************************
 * END OF Packet handling functions
 *******************************************************************************/
void Setup_all(){
  uint8_t def_addr=103;
  while(def_addr<112){
    mppt_0.Setup(def_addr);
    mppt_1.Setup(def_addr);
    mppt_2.Setup(def_addr);
    mppt_3.Setup(def_addr);
    def_addr++;
  }
}
void read_single(uint8_t addr_to_read){
  mppt_0.ReadPower1(addr_to_read,power1[0]);
  mppt_0.ReadPower2(addr_to_read,power2[0]);
  mppt_0.ReadSense1(addr_to_read,sense1[0]);
  mppt_0.ReadSense2(addr_to_read,sense2[0]);
  mppt_0.ReadCurrent1(addr_to_read,current1[0]);
  mppt_0.ReadCurrent2(addr_to_read,current2[0]);
  mppt_0.ReadGPIO(1,addr_to_read,gpio1[0]);
  mppt_0.ReadGPIO(2,addr_to_read,gpio2[0]);
  mppt_0.ReadGPIO(3,addr_to_read,gpio3[0]);
  mppt_0.ReadGPIO(4,addr_to_read,gpio4[0]);
}

void read_mppts(){
  switch(which_read){
    case 1:{
      mppt_0.ReadPower1(addr,power1[0]);
      mppt_1.ReadPower1(addr,power1[1]);
      mppt_2.ReadPower1(addr,power1[2]);
      mppt_3.ReadPower1(addr,power1[3]);
      break; 
    }
    case 2:{
      mppt_0.ReadPower2(addr,power2[0]);
      mppt_1.ReadPower2(addr,power2[1]);
      mppt_2.ReadPower2(addr,power2[2]);
      mppt_3.ReadPower2(addr,power2[3]);
      break; 
    }
    case 3:{
      mppt_0.ReadSense1(addr,sense1[0]);
      mppt_1.ReadSense1(addr,sense1[1]);
      mppt_2.ReadSense1(addr,sense1[2]);
      mppt_3.ReadSense1(addr,sense1[3]);
      break; 
    }
    case 4:{
      mppt_0.ReadSense2(addr,sense2[0]);
      mppt_1.ReadSense2(addr,sense2[1]);
      mppt_2.ReadSense2(addr,sense2[2]);
      mppt_3.ReadSense2(addr,sense2[3]);
      break; 
    }
    case 5:{
      mppt_0.ReadCurrent1(addr,current1[0]);
      mppt_1.ReadCurrent1(addr,current1[1]);
      mppt_2.ReadCurrent1(addr,current1[2]);
      mppt_3.ReadCurrent1(addr,current1[3]);
      break; 
    }
    case 6:{
      mppt_0.ReadCurrent2(addr,current2[0]);
      mppt_1.ReadCurrent2(addr,current2[1]);
      mppt_2.ReadCurrent2(addr,current2[2]);
      mppt_3.ReadCurrent2(addr,current2[3]);
      break; 
    }
    default:{
      mppt_0.ReadGPIO(gpio_select,addr,gpio1[0]);
      mppt_1.ReadGPIO(gpio_select,addr,gpio1[1]);
      mppt_2.ReadGPIO(gpio_select,addr,gpio1[2]);
      mppt_3.ReadGPIO(gpio_select,addr,gpio1[3]);
      gpio_select++;
      break; 
    }
  }
}
// function for when a "SetPriority" command is received by this device, adding that commands priority value to the array/list
void setCommandPriority(housekeeping_prio_t * prio, uint8_t * respData, uint8_t len) {
//  housekeeping_prio_t * set_prio = (housekeeping_prio_t *) prio;
  commandPriority[prio->command-FIRST_LOCAL_COMMAND] = (uint8_t) prio->prio_type;
  memcpy(respData, (uint8_t*)prio, len);
}

// sending priority command function
// probably can be cleaned up
// Note: SendAll is 253 and SendLow is 250 so we made SendLow-> int priority=1 for checking the device's list of command's priorities.
void handlePriority(uint8_t prio_in, uint8_t * responsePacketBuffer){
  housekeeping_hdr_t *respHdr = (housekeeping_hdr_t *) responsePacketBuffer;
  uint8_t *respData = responsePacketBuffer + hdr_size;
  int priority=0;
  int retval = 0;
  uint8_t sum = 0; // hdr length of data atatched from all those commands data
//  respHdr->cmd = hdr_in->cmd;
  // priority == 4 when this function is called is code for "eSendAll"
  // otherwise priority=1,2,3 and that maps to eSendLowPriority+priority
  if(prio_in==eSendAll) priority=4;
  else priority = prio_in - 249;
//  int retval;
  respHdr->src = myID;
  respHdr->dst = eSFC;
  respHdr->cmd =  prio_in;
  // go through every priority
  for (int i=0;i<NUM_LOCAL_CONTROLS;i++) {
    if (commandPriority[i] == (uint8_t)priority || priority==4) {
      retval=handleLocalRead((uint8_t) i + FIRST_LOCAL_COMMAND, respData+sum);
      // if that read overflowed the data???? fix later?
      sum+= (uint8_t) retval;
    }
    else sum+=0;
  }
  respHdr->len=sum;
  fillChecksum(responsePacketBuffer);
  downStream1.send(responsePacketBuffer, respHdr->len + hdr_size + 1);
  currentPacketCount++;
}

// Fn to handle a local command write.
// This gets called when a local command is received
// with data (len != 0).
int handleLocalWrite(uint8_t localCommand, uint8_t * data, uint8_t len, uint8_t * respData) {
  int retval = 0;
  switch(localCommand) {
  case eSetPriority:
    setCommandPriority((housekeeping_prio_t *)data,respData,len);
//    memcpy((uint8_t*)respData, (uint8_t*)data, len);
    retval=len;
    break;
  case ePacketCount:
    retval = EBADLEN;
    break;
  default:
    retval= EBADCOMMAND;    
    break;
  }
  return retval;
}

int handleLocalRead(uint8_t localCommand, uint8_t *buffer) {
  int retval = 0;
  switch(localCommand) {
  case ePingPong:
    retval=0;
    break;
  case eIntSensorRead: {
    uint32_t TempRead=analogRead(TEMPSENSOR);
    float TempC = (float)(1475 - ((2475 * TempRead) / 4096)) / 10;
    memcpy(buffer,(uint8_t *) &TempC,sizeof(TempC));
    retval=sizeof(TempC);
    break;
  }
  case ePacketCount:{
    memcpy(buffer, (uint8_t *) &currentPacketCount, sizeof(currentPacketCount));
    retval = sizeof(currentPacketCount);
    break;
  }
  case ePower1:{
    memcpy(buffer,power1,sizeof(power1));
    retval=sizeof(power1);
    break;
  }
  case ePower2:{
    memcpy(buffer,power2,sizeof(power2));
    retval=sizeof(power2);
    break;
  }
  case eSense1:{
    memcpy(buffer,sense1,sizeof(sense1));
    retval=sizeof(sense1);
    break;
  }
  case eSense2:{
    memcpy(buffer,sense2,sizeof(sense2));
    retval=sizeof(sense2);
    break;
  }
  case eCurrent1:{
    memcpy(buffer,current1,sizeof(current1));
    retval=sizeof(current1);
    break;
  }
  case eCurrent2:{
    memcpy(buffer,current2,sizeof(current2));
    retval=sizeof(current2);
    break;
  }
  case eGPIO1:{
    memcpy(buffer,gpio1,sizeof(gpio1));
    retval=sizeof(gpio1);
    break;
  }
  case eGPIO2:{
    memcpy(buffer,gpio2,sizeof(gpio2));
    retval=sizeof(gpio2);
    break;
  }
  case eGPIO3:{
    memcpy(buffer,gpio3,sizeof(gpio3));
    retval=sizeof(gpio3);
    break;
  }
  case eGPIO4:{
    memcpy(buffer,gpio4,sizeof(gpio4));
    retval=sizeof(gpio4);
    break;
  }
  case eISR: {
    uint32_t TempRead=analogRead(TEMPSENSOR);
    float TempC = (float)(1475 - ((2475 * TempRead) / 4096)) / 10;
    memcpy(buffer,(uint8_t *) &TempC,sizeof(TempC));
    retval=sizeof(TempC);
    break;
  }
  default:
    retval=EBADCOMMAND;
  }  
  return retval;
}

// Function to call first when localcommand sent. 
// Store the result as retval (which should be bytes read or written?)
void handleLocalCommand(housekeeping_hdr_t *hdr, uint8_t * data, uint8_t * responsePacketBuffer) {
  int retval=0;
  housekeeping_hdr_t *respHdr = (housekeeping_hdr_t *) responsePacketBuffer;
  uint8_t *respData = responsePacketBuffer + sizeof(housekeeping_hdr_t);
  respHdr->src = myID;
  respHdr->dst = hdr->src;
  if (hdr->len) {
    retval = handleLocalWrite(hdr->cmd, data, hdr->len, respData); // retval is negative construct the baderror hdr and send that instead. 
    if(retval>=0) {
//      *respData= 5;
      respHdr->cmd = hdr->cmd;
      respHdr->len = retval; // response bytes of the write.
    }
    else{
      housekeeping_err_t *err = (housekeeping_err_t *) respData;
      buildError(err, respHdr, hdr, retval);
    }  
  } 
  else {
    // local read. by definition these always go downstream.
    retval = handleLocalRead(hdr->cmd, respData);
    if (retval>=0) {
      respHdr->cmd = hdr->cmd;
      respHdr->len = retval; //bytes read
    }
    else {
      housekeeping_err_t *err = (housekeeping_err_t *) respData;
      buildError(err, respHdr, hdr, retval); // the err pointer is pointing to the data of the response packet based on the line above so this fn fills that packet. 
    }
  }
  fillChecksum(responsePacketBuffer);
  // send to SFC
  downStream1.send(responsePacketBuffer, respHdr->len + hdr_size + 1 );
  currentPacketCount++;
}

void handleTestMode(housekeeping_hdr_t *hdr, uint8_t *data, uint8_t * responsePacketBuffer) {
  housekeeping_hdr_t *respHdr = (housekeeping_hdr_t *) responsePacketBuffer;
  uint8_t *respData = responsePacketBuffer + sizeof(housekeeping_hdr_t);
  respHdr->src = myID;
  respHdr->dst = hdr->src;
// if length was actually placed then go into testmode, else build badlength error.
  if (hdr->len) {
   //construct data incoming to be the num testpackets and send the data packet in a while loop and decrement numtestpackets?
    uint16_t numTestPackets = ((uint16_t) (*(data+1) << 8)) | *(data) ; // figure out the correct way to get 2 bytes into a 16_t
    timelastpacket = millis();
    while(numTestPackets){
      if(long (millis()-timelastpacket)>0) { // only send every 50 milliseconds?
        *(respData) = numTestPackets;    
        *(respData+1) = numTestPackets >> 8;
        respHdr->cmd = hdr->cmd;
        respHdr->len = 0x02; // response bytes of the write.
        fillChecksum(responsePacketBuffer);
        // send to SFC
        downStream1.send(responsePacketBuffer, respHdr->len + sizeof(housekeeping_hdr_t) + 1 );  
        numTestPackets--;
        timelastpacket = timelastpacket+TEST_MODE_PERIOD;
        currentPacketCount++;
      }
    }
  }
  else{
    housekeeping_err_t *err = (housekeeping_err_t *) respData;
    buildError(err, respHdr, hdr, EBADLEN); 
    fillChecksum(responsePacketBuffer);
    // send to SFC
    downStream1.send(responsePacketBuffer, respHdr->len + sizeof(housekeeping_hdr_t) + 1 );  
    currentPacketCount++;
  }  
}
