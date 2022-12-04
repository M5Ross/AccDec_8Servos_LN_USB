// Production 17 Function DCC Decoder 
// Version 5.1  Geoff Bunza 2014,2015,2016
// NO LONGER REQUIRES modified software servo Lib
// Software restructuring mods added from Alex Shepherd and Franz-Peter
//   With sincere thanks

      /*********************************************
       *                                           *
       *  modified by M.Ross => AccDec8ServoLNUSB  *
       *                                           *
       *********************************************/

#define HW_VERSION 11
#define SW_VERSION 21

#define LN_VERSION 2

#include <NmraDcc.h>
//#include <Servo.h>
#include <SoftwareServo.h> 
#include <LocoNet.h>

//Servo servo[8];
SoftwareServo servo[8];

#define servo_start_delay 50
#define servo_init_delay 8
#define servo_slowdown  3   //servo loop counter limit
int servo_slow_counter = 0; //servo loop counter to slowdown servo transit

uint8_t numfpins = 8;
byte fpins [] = {10,9,8,6,12,1,0,2};
byte Rpins [] = {A5,A4,A3,A2,A1,A0,13,5};
#define DCC_PIN 3
#define TASTER_PIN  7
#define TX_PIN  11

// The LocoNet Tx/Rx traffic can be indicated using some LEDs. The default values below are for the Sparkfun Arduino Pro Micro board.
// If your board doesn't have LEDs or you don't want to use them, comment out the two #define lines below
#define TX_LED  30
#define RX_LED  17

#if defined(TX_LED) or defined(RX_LED)
// If you're using LEDs to indicate LocoNet traffic then define the HIGH/LOW state to turn the LEDs ON/OFF in the two lines below
#define LED_ON  LOW
#define LED_OFF HIGH
#include <elapsedMillis.h>
elapsedMillis  txElapsedMillis;
elapsedMillis  rxElapsedMillis;
#endif

NmraDcc  Dcc ;

LocoNetCVClass lnCV;
boolean programmingMode = false;

static   LnBuf        LnTxBuffer ;
static   lnMsg        *LnPacket;
static   lnMsg        *LnTxPacket;
static   lnMsg        *LnRxPacket;
static   lnMsg        *LnStatsPacket;
static   lnMsg        myStats;

// Item Number (Art.-Nr.): 64440
#define ARTNR 6444
#define generalAdr 65535  //General-Adresse 0xFFFF
#define maxLNCV 120 //lesbare LNCV
#define MaxAdr 4096    //höchste nutzbare Adresse

#define SET_CV_Address  99                // THIS ADDRESS IS FOR SETTING CV'S Like a Loco

                                          // WHICH WILL EXTEND FOR 16 MORE SWITCH ADDRESSES
uint8_t CV_DECODER_MASTER_RESET =   120;  // THIS IS THE CV ADDRESS OF THE FULL RESET
#define CV_To_Store_SET_CV_Address	121
#define CV_Accessory_Address CV_ACCESSORY_DECODER_ADDRESS_LSB

#define CV_SINGLE_INV     70
#define CV_MULTI_ADDRESS  80

struct QUEUE
{
  int16_t inuse;
  int16_t current_position;
  int16_t increment;
  int16_t stop_value;
  int16_t start_value;
  int16_t invert;
  int16_t multi_address;
  int16_t single_invert;
};
QUEUE *ftn_queue = new QUEUE[numfpins];

struct CVPair
{
  uint16_t  CV;
  uint8_t   Value;
};

/*********************************************************************************************/
#include "config.h"
/*********************************************************************************************/

confCV   CV28(Dcc) ;

uint8_t FactoryDefaultCVIndex = sizeof(FactoryDefaultCVs)/sizeof(CVPair);

#if defined(SERIALCOM)

  #include <DccSerialCom.h>
  
  DccSerialCom SerialCom(FactoryDefaultCVIndex, CV_MULTI_ADDRESS, CV_SINGLE_INV);

  void notifyExecuteFunction(uint8_t function, uint8_t state) {
    exec_function(function, state);
  }
  
  uint16_t notifyGetCVnum(uint16_t index) {
    return FactoryDefaultCVs[index].CV;
  }

  uint16_t notifyGetCVval(uint16_t CV) {
    return Dcc.getCV(CV);
  }

  void notifySetCV(uint16_t CV, uint16_t value) {
    Dcc.setCV(CV, value);
  }
  
#endif

bool program_mode = false;
bool ProgAdr = false;
bool SwState = false;
bool resetfactorydefault = false;
uint8_t counter = 0;

#define enServo(x, y)  digitalWrite(Rpins[x], y)

void notifyCVResetFactoryDefault()
{
  // Make FactoryDefaultCVIndex non-zero and equal to num CV's to be reset 
  // to flag to the loop() function that a reset to Factory Defaults needs to be done
  FactoryDefaultCVIndex = sizeof(FactoryDefaultCVs)/sizeof(CVPair);
};

void setup()   //******************************************************
{
  pinMode(TASTER_PIN, INPUT_PULLUP);
  program_mode = !digitalRead(TASTER_PIN);
  
  int i;
  //uint8_t cv_value;
  // initialize the digital pins as outputs
  for (i=0; i < numfpins; i++) {
      pinMode(fpins[i], OUTPUT);
      digitalWrite(fpins[i], 0);
      pinMode(Rpins[i], OUTPUT);
      digitalWrite(Rpins[i], 0);
     }
  
  // Setup which External Interrupt, the Pin it's associated with that we're using 
  Dcc.pin(DCC_PIN, 0);
  // Call the main DCC Init function to enable the DCC Receiver
  Dcc.init( MAN_ID_DIY, 100, FLAGS_OUTPUT_ADDRESS_MODE | FLAGS_DCC_ACCESSORY_DECODER, CV_To_Store_SET_CV_Address);
  delay(800);
   
  ResetCV(0);

  CV28.init();
  
#if defined(DEBUG) | defined(SERIALCOM)
  if(CV28.GetLnUSB()) {
    if(program_mode) {
      #ifdef RX_LED
        pinMode(RX_LED, OUTPUT);
        digitalWrite(RX_LED, LED_ON);
      #endif
      Serial.begin(9600);
      SerialCom.init(Serial);
      while(!Serial);
      CV28.SetLnUSB(false);
    }
    else {
      Serial.begin(57600);
      SerialCom.end();
      while(!Serial);
      initLnBuf(&LnTxBuffer);
      LnRxPacket = NULL;
      LnTxPacket = NULL;
      LnStatsPacket = NULL;
    }
  }
  else {
    Serial.begin(9600);
    SerialCom.init(Serial);
  }
#endif

  LocoNet.init(TX_PIN);

#ifdef RX_LED
  pinMode(RX_LED, OUTPUT);
  digitalWrite(RX_LED, LED_ON);
#endif

#ifdef TX_LED
  pinMode(TX_LED, OUTPUT);
  digitalWrite(TX_LED, LED_ON);
#endif
 
  for ( i=0; i < numfpins; i++) {
      //servo
       {
        CVrefresh(i);

        // attaches servo on pin to the servo object 
        servo[i].attach(fpins[i]);

#ifdef DEBUG
	 Serial.print("InitServo ID= ");
	 Serial.println(i, DEC) ;
#endif
	      servo[i].write(ftn_queue[i].current_position);
        enServo(i, 1);
        for (int t=0; t<servo_start_delay; t++) 
		      {SoftwareServo::refresh(); 
		       delay(servo_init_delay);}

        enServo(i, 0);
        ftn_queue[i].inuse = 0;
        //servo[i].detach();
        //pinMode(fpins[i],INPUT);
        }
  }
#ifdef RX_LED
  if(!program_mode)
    digitalWrite(RX_LED, LED_OFF);
#endif

#ifdef TX_LED
  digitalWrite(TX_LED, LED_OFF);
#endif
}

void loop()   //**********************************************************************
{
  // Serial comunication
#if defined(SERIALCOM)
  if(CV28.GetLnUSB()) {
    SerialUSBLoconetCom();
  }
  else {
    SerialCom.process();
#endif
    LoconetCom();
#if defined(SERIALCOM)
  }
#endif

  if(resetfactorydefault) {
    ResetCV(1);
    resetfactorydefault = false;
  }

  //handle programming switch:
  if (SwState == false && digitalRead(TASTER_PIN) == LOW) {
    //down
    SwState = true;
  }
  if (SwState == true && digitalRead(TASTER_PIN) == HIGH) {
    //up
    SwState = false;
    ProgAdr = !ProgAdr;
    if(!ProgAdr)
      digitalWrite(TX_LED, LED_OFF);
  }
  
  if (ProgAdr == true) {  //Blinken bei Programmierung
    counter++;
    if(counter > 20) {
      counter = 0;
      
      #ifdef TX_LED
        digitalWrite(TX_LED, !digitalRead(TX_LED));
        txElapsedMillis = 0;
      #endif
    }
  }
  
  //MUST call the NmraDcc.process() method frequently 
  // from the Arduino loop() function for correct library operation
  Dcc.process();
  SoftwareServo::refresh();
  delay(8);
  if (servo_slow_counter++ > servo_slowdown) {
    for (int i=0; i < numfpins; i++) {
      if (ftn_queue[i].inuse==1)  {
      
#ifdef RX_LED
        digitalWrite(RX_LED, LED_ON);
        rxElapsedMillis = 0;
#endif
        ftn_queue[i].current_position = ftn_queue[i].current_position + ftn_queue[i].increment;
        enServo(i, 1);
        servo[i].write(ftn_queue[i].current_position);
        SoftwareServo::refresh();
        delay(1);
        
        servo_slow_counter = 0;
  
        if (ftn_queue[i].increment > 0) {
          if (ftn_queue[i].current_position >= ftn_queue[i].stop_value) {
            ftn_queue[i].current_position = ftn_queue[i].stop_value;
            ftn_queue[i].inuse = 0;
            //servo[i].detach();
  
            if(CV28.GetSave()) Dcc.setCV( 34+(i*5), ftn_queue[i].current_position);
  
            enServo(i, 0);
  
            #ifdef RX_LED
              digitalWrite(RX_LED, LED_OFF);
            #endif
            
            #ifdef DEBUG
              Serial.print("Servo");
              Serial.print(i, DEC) ;
              Serial.print(" pos = ");
              Serial.println(ftn_queue[i].current_position, DEC) ;
            #endif
          }
        }
        if (ftn_queue[i].increment < 0) { 
          if (ftn_queue[i].current_position <= ftn_queue[i].start_value) { 
            ftn_queue[i].current_position = ftn_queue[i].start_value;
            ftn_queue[i].inuse = 0;
            //servo[i].detach();
  
            if(CV28.GetSave()) Dcc.setCV( 34+(i*5), ftn_queue[i].current_position);
            
            enServo(i, 0);
  
            #ifdef RX_LED
              digitalWrite(RX_LED, LED_OFF);
            #endif
            
            #ifdef DEBUG
              Serial.print("Servo");
              Serial.print(i, DEC) ;
              Serial.print(" pos = ");
              Serial.println(ftn_queue[i].current_position, DEC) ;
            #endif
          }
        }
        //break;
      }
    }
  }
}

void notifySwitchRequest( uint16_t Address, uint8_t Output, uint8_t Direction )
{
  if(Output)
    notifyDccAccTurnoutOutput( Address, Direction ? 0x01 : 0x00, 0);
    //notifyDccAccState( Address, 0, Direction ? 0x01 : 0x00, 0);
}

void LoconetCom(void) {
  LnPacket = LocoNet.receive();

  if( LnPacket )
  {
    uint8_t packetConsumed(LocoNet.processSwitchSensorMessage(LnPacket));
    if (packetConsumed == 0) {
      packetConsumed = lnCV.processLNCVMessage(LnPacket);
    }
  }
}

#if defined(SERIALCOM)
void SerialUSBLoconetCom(void) {
  // Before we check for a new LocoNet packet, make sure we haven't already got a previously unset packet
  if(LnRxPacket == NULL)
  {
    if(LnStatsPacket)
    {
      LnRxPacket = LnStatsPacket;
      LnStatsPacket = NULL;
    }
    
    else
      LnRxPacket = LocoNet.receive() ;
  }

  uint8_t packetConsumed(LocoNet.processSwitchSensorMessage(LnPacket));
  if (packetConsumed == 0) {
    packetConsumed = lnCV.processLNCVMessage(LnPacket);
    if(packetConsumed)
      LnRxPacket = NULL;
  }

  if( LnRxPacket )
  {
#ifdef RX_LED
    digitalWrite(RX_LED, LED_ON);
    rxElapsedMillis = 0;
#endif    
      // Get the length of the received packet
    uint8_t Length = getLnMsgSize( LnRxPacket ) ;

    uint8_t USBWriteBufferFree = Serial.availableForWrite();
    if( USBWriteBufferFree >= Length)
    {
      Serial.write((uint8_t*)LnRxPacket, Length);
      LnRxPacket = NULL;
    }
  }

    // Before we check for a new LocoNet TX packet, make sure we haven't already got a previously unset packet
  if(LnTxPacket == NULL)
  {
    int charWaiting;
    
      // Check to see if there are any bytes from the PC
    while( (charWaiting = Serial.available()) && (LnTxPacket == NULL) )
    {
        // Read the byte
      uint8_t inByte = Serial.read() & 0xFF;
      
        // Add it to the buffer
      addByteLnBuf( &LnTxBuffer, inByte ) ;
      
        // Check to see if we have received a complete packet yet
      LnTxPacket = recvLnMsg( &LnTxBuffer ) ;
    }
  }
    
    // Send the received packet from the PC to the LocoNet
  if(LnTxPacket )
  {
      // Check for a Request for LocoNet Stats
    if(LnTxPacket->data[0] == OPC_BUSY)
    {
      LnTxPacket = NULL;
      updateStats();
    }
    
    else if(LocoNet.send( LnTxPacket ) == LN_DONE)
    {
      LocoNet.processSwitchSensorMessage(LnTxPacket);
      LnTxPacket = NULL;
      
#ifdef TX_LED
      digitalWrite(TX_LED, LED_ON);
      txElapsedMillis = 0;
#endif      
    }
  }

#ifdef RX_LED
  if(rxElapsedMillis > 50)
    digitalWrite(RX_LED, LED_OFF);
#endif

#ifdef TX_LED
  if(txElapsedMillis > 50)
    digitalWrite(TX_LED, LED_OFF);
#endif
}

#endif

void ResetCV(bool forcereset) {
  #if defined(DECODER_LOADED)
  if ( Dcc.getCV(CV_DECODER_MASTER_RESET)== CV_DECODER_MASTER_RESET || Dcc.getCV(CV_ACCESSORY_DECODER_ADDRESS_LSB) == 255 || forcereset == true) 
  #endif  
     {
       for (int j=0; j < sizeof(FactoryDefaultCVs)/sizeof(CVPair); j++ )
         Dcc.setCV( FactoryDefaultCVs[j].CV, FactoryDefaultCVs[j].Value);
     }
}

void CVrefresh(uint8_t i) {
  if (CV28.GetMultiAdr()) {
    ftn_queue[i].multi_address = int (word(char (Dcc.getCV( CV_MULTI_ADDRESS+1+(i*2))), char (Dcc.getCV( CV_MULTI_ADDRESS+(i*2)))));
  }
  else {
    ftn_queue[i].multi_address = 0;
  }
  ftn_queue[i].single_invert = int (Dcc.getCV( CV_SINGLE_INV+i));
  ftn_queue[i].current_position = int (Dcc.getCV( 34+(i*5)));
  ftn_queue[i].stop_value = int (Dcc.getCV( 33+(i*5)));
  ftn_queue[i].start_value = int (Dcc.getCV( 32+(i*5)));
  ftn_queue[i].invert = int (char (Dcc.getCV( 30+(i*5))));
  ftn_queue[i].increment = -int (char (Dcc.getCV( 31+(i*5)))); 
}

// This function is called by the NmraDcc library
// when a DCC ACK needs to be sent
// Calling this function should cause an increased 60ma current drain
// on the power supply for 6ms to ACK a CV Read
void notifyCVAck(void) {
}

void notifyCVChange( uint16_t CV, uint8_t Value) {
  //digitalWrite( LED, HIGH );
  //delay(20);
  //digitalWrite( LED, LOW );
  uint8_t lnUSBcom = CV28.GetLnUSB();
  CV28.init();
  if(CV28.GetLnUSB())
    CV28.SetLnUSB(lnUSBcom);
  
  for (uint8_t i=0; i < numfpins; i++) {
    CVrefresh(i);
  }
  if(CV == CV_DECODER_MASTER_RESET && Value == CV_DECODER_MASTER_RESET && resetfactorydefault == false) {
    resetfactorydefault = true;
  }
}

extern void notifyDccAccTurnoutOutput( uint16_t Addr, uint8_t Direction, uint8_t OutputPower ) {
  if (ProgAdr == true) {
    Dcc.setCV(CV_ACCESSORY_DECODER_ADDRESS_LSB, lowByte(Addr));
    Dcc.setCV(CV_ACCESSORY_DECODER_ADDRESS_MSB, highByte(Addr));
    #ifdef TX_LED
      digitalWrite(TX_LED, LED_OFF);
    #endif
    ProgAdr = false;
  }
  else {  
    if (CV28.GetMultiAdr()) {
      for (uint8_t i=0; i < numfpins; i++) {
        if (ftn_queue[i].multi_address == Addr) {
          if (ftn_queue[i].single_invert) {
            exec_function(i, Direction ? 0 : 1);
          }
          else {
            exec_function(i, Direction ? 1 : 0);
          }
        }
      }
    }
    else {
      uint16_t Current_Decoder_Addr = Dcc.getAddr();
      if ( Addr >= Current_Decoder_Addr && Addr < Current_Decoder_Addr+numfpins) { //Controls Accessory_Address+8
        if (ftn_queue[Addr-Current_Decoder_Addr].single_invert) {
          exec_function(Addr-Current_Decoder_Addr, Direction ? 0 : 1);
        }
        else {
          exec_function(Addr-Current_Decoder_Addr, Direction ? 1 : 0);
        }
      }
    }
  }
}

/*
void notifyDccAccState( uint16_t Addr, uint16_t BoardAddr, uint8_t OutputAddr, uint8_t State) {
  if (ProgAdr == true) {
    Dcc.setCV(CV_ACCESSORY_DECODER_ADDRESS_LSB, lowByte(Addr));
    Dcc.setCV(CV_ACCESSORY_DECODER_ADDRESS_MSB, highByte(Addr));
    #ifdef TX_LED
      digitalWrite(TX_LED, LED_OFF);
    #endif
    ProgAdr = false;
  }
  else {
    uint16_t Current_Decoder_Addr;
    uint8_t Bit_State;
    Current_Decoder_Addr = Dcc.getAddr();
    Bit_State = OutputAddr & 0x01;
    
    if ( Addr >= Current_Decoder_Addr && Addr < Current_Decoder_Addr+8) { //Controls Accessory_Address+7
  #ifdef DEBUG
     Serial.print("Addr = ");
     Serial.println(Addr);
       Serial.print("BoardAddr = ");
     Serial.println(BoardAddr);
     Serial.print("Bit_State = ");
     Serial.println(Bit_State);
  #endif
    exec_function(Addr-Current_Decoder_Addr, Bit_State );
    }
  }
}
*/

void exec_function (int function, int FuncState)  {
    // Servo
      //if (ftn_queue[function].inuse == 0)  {
	    ftn_queue[function].inuse = 1;
      //pinMode(pin,OUTPUT);
		//servo[function].attach(pin);
	  //}
      bool invert_command = false;
      if(CV28.GetInv())
        invert_command = ftn_queue[function].invert;
      
      if (FuncState ^ invert_command) {
        ftn_queue[function].increment = char ( Dcc.getCV( 31+(function*5)));
        //ftn_queue[function].stop_value = Dcc.getCV( 33+(function*5));
      }
      else {
        ftn_queue[function].increment = - char(Dcc.getCV( 31+(function*5)));
        //ftn_queue[function].stop_value = Dcc.getCV( 32+(function*5));
      }
}

  // Format a LocoBuffer II Status Response
void updateStats()
{
  
  memset(&myStats, 0, sizeof(peerXferMsg));
  
  myStats.data[ 0] = OPC_PEER_XFER;
  myStats.data[ 1] = 0x10;
  myStats.data[ 2] = 0x50;
  myStats.data[ 3] = 0x50;
  myStats.data[ 4] = 0x01;

  LnBufStats* pLnStats = LocoNet.getStats();
  long Errors = pLnStats->RxErrors + pLnStats->TxErrors;

  uint8_t myStatsData[8];

  myStatsData[0] = 0x00;
  myStatsData[1] = (Errors >> 16) & 0xFF;
  myStatsData[2] = (Errors >>  8) & 0xFF;
  myStatsData[3] = Errors & 0xFF;
  
  myStatsData[4] = LN_VERSION;
  myStatsData[5] = 0x00;
  myStatsData[6] = (pLnStats->Collisions >> 8) & 0xFF ;
  myStatsData[7] = pLnStats->Collisions & 0xFF;

  encodePeerData(&myStats.px, myStatsData);

  uint8_t CheckSum = 0xFF ;

  for( uint8_t lnTxIndex = 0; lnTxIndex < sizeof(peerXferMsg) - 1; lnTxIndex++ ) {
    CheckSum ^= myStats.data[ lnTxIndex ] ;
  }

  myStats.data[sizeof(peerXferMsg) - 1] = CheckSum ; 

  LnStatsPacket = &myStats;
}

int8_t notifyLNCVread(uint16_t ArtNr, uint16_t lncvAddress, uint16_t, uint16_t & lncvValue) {
  /*Serial.print("Enter notifyLNCVread(");
  Serial.print(ArtNr, HEX);
  Serial.print(", ");
  Serial.print(lncvAddress, HEX);
  Serial.print(", ");
  Serial.print(", ");
  Serial.print(lncvValue, HEX);
  Serial.print(")");*/
  // Step 1: Can this be addressed to me?
  // All ReadRequests contain the ARTNR. For starting programming, we do not accept the broadcast address.
  if (programmingMode) {
    if (ArtNr == ARTNR) {
      if (lncvAddress < maxLNCV) {
        lncvValue = Dcc.getCV(lncvAddress);
        /*Serial.print(" LNCV Value: ");
        Serial.print(lncvValue);
        Serial.print("\n");*/
        return LNCV_LACK_OK;
      } else {
        // Invalid LNCV address, request a NAXK
        return LNCV_LACK_ERROR_UNSUPPORTED;
      }
    } else {
      //Serial.print("ArtNr invalid.\n");
      return -1;
    }
  } else {
    //Serial.print("Ignoring Request.\n");
    return -1;
  }
}

int8_t notifyLNCVprogrammingStart(uint16_t & ArtNr, uint16_t & ModuleAddress) {
  // Enter programming mode. If we already are in programming mode,
  // we simply send a response and nothing else happens.
  //Serial.print("notifyLNCVProgrammingStart ");
  if (ArtNr == ARTNR) {
    //Serial.print("artnrOK ");
    if (ModuleAddress == Dcc.getAddr()) {
      //Serial.print("moduleUNI ENTERING PROGRAMMING MODE\n");
      programmingMode = true;
      return LNCV_LACK_OK;
    } else if (ModuleAddress == generalAdr) {
      //Serial.print("moduleBC ENTERING PROGRAMMING MODE\n");
      ModuleAddress = Dcc.getAddr();
      return LNCV_LACK_OK;
    }
  }
  //Serial.print("Ignoring Request.\n");
  return -1;
}

  /**
   * Notifies the code on the reception of a write request
   */
   
int8_t notifyLNCVwrite(uint16_t ArtNr, uint16_t lncvAddress, uint16_t lncvValue) {
  //Serial.print("notifyLNCVwrite, ");
  //  dumpPacket(ub);
  if (!programmingMode) {
    //Serial.print("not in Programming Mode.\n");
    return -1;
  }

  if (ArtNr == ARTNR) {
    //Serial.print("Artnr OK, ");

    if (lncvAddress < maxLNCV) {
      Dcc.setCV(lncvAddress, lncvValue);
      return LNCV_LACK_OK;
    }
    else {
      return LNCV_LACK_ERROR_UNSUPPORTED;
    }

  }
  else {
    //Serial.print("Artnr Invalid.\n");
    return -1;
  }
}

  /**
   * Notifies the code on the reception of a request to end programming mode
   */
   
void notifyLNCVprogrammingStop(uint16_t ArtNr, uint16_t ModuleAddress) {
  //Serial.print("notifyLNCVprogrammingStop ");
  if (programmingMode) {
    if (ArtNr == ARTNR && ModuleAddress == Dcc.getAddr()) {
      programmingMode = false;
      //Serial.print("End Programing Mode.\n");
    }
    else {
      if (ArtNr != ARTNR) {
        //Serial.print("Wrong Artnr.\n");
        return;
      }
      if (ModuleAddress != Dcc.getAddr()) {
        //Serial.print("Wrong Module Address.\n");
        return;
      }
    }
  }
  else {
    //Serial.print("Ignoring Request.\n");
  }
}
