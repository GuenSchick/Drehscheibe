// DCC Stepper Motor Controller ( A4988 ) Example
// See: https://www.dccinterface.com/how-to/assemblyguide/
// 
// Author: Alex Shepherd 2017-12-04
// 
// This example requires two Arduino Libraries:
//
// 1) The AccelStepper library from: http://www.airspayce.com/mikem/arduino/AccelStepper/index.html
//
// 2) The NmraDcc Library from: http://mrrwa.org/download/
//
// Both libraries can be installed via the Arduino IDE Library Manager 
//

#include <AccelStepper.h>
#include <NmraDcc.h>

// Uncomment to enable Powering-Off the Stepper if its not running 
#define STEPPER_ENABLE_PIN 6

// Home Position Sensor Input
#define HOME_SENSOR_PIN 3

typedef struct
{
  int address;
  int stationFront;
  int stationBack;
}
DCCAccessoryAddress;
DCCAccessoryAddress gAddresses[12];  //Anzahl der Gleisabgänge/Haltepunkte

// for a 1.8 deg stepper, there are 200 full steps
#define FULL_STEPS_PER_REVOLUTION 889 // für das von mir verwendete Getriebe angepasst.

// Uncomment the lime below for the Driver Board Settings
//#define FULL_TURN_STEPS (FULL_STEPS_PER_REVOLUTION)     // full steps
//#define FULL_TURN_STEPS (FULL_STEPS_PER_REVOLUTION * 2) // 1/2 steps
//#define FULL_TURN_STEPS (FULL_STEPS_PER_REVOLUTION * 4) // 1/4 steps
//#define FULL_TURN_STEPS (FULL_STEPS_PER_REVOLUTION * 8) // 1/8 steps
#define FULL_TURN_STEPS (FULL_STEPS_PER_REVOLUTION * 16) // 1/16 steps

// home location
#define entryStation 0 //steps bis zur Grundposition vom Sensor aus.

volatile bool bInterruptDetected = false;
bool bHomePositionFound = false;

// Now we'll wrap the stepper in an AccelStepper object
AccelStepper stepper1(1, 4, 5);

NmraDcc  Dcc ;
uint16_t lastAddr = 0xFFFF ;
uint8_t lastDirection = 0xFF;
//
// Decoder Init
//
void ConfigureStations()
{
  // this is home
  gAddresses[0].address = 200;                // Einfahrt Drehscheibe
  gAddresses[0].stationFront = entryStation;
  gAddresses[0].stationBack = entryStation;

  gAddresses[1].address = 202;                // Einfahrt Aschegleis
  gAddresses[1].stationFront = ((FULL_TURN_STEPS / 24) * 21);
  gAddresses[1].stationBack =  ((FULL_TURN_STEPS / 24) * 21);
  
  gAddresses[2].address = 203;                // Einfahrt Kohle
  gAddresses[2].stationFront = ((FULL_TURN_STEPS / 24) * 20);
  gAddresses[2].stationBack =  ((FULL_TURN_STEPS / 24) * 20);

  gAddresses[3].address = 204;                // Einfahrt Wasser
  gAddresses[3].stationFront = ((FULL_TURN_STEPS / 24) * 19);
  gAddresses[3].stationBack =  ((FULL_TURN_STEPS / 24) * 19);

  gAddresses[4].address = 205;                // Einfahrt Lokschuppen Tor 1
  gAddresses[4].stationFront = ((FULL_TURN_STEPS / 24) * 18);
  gAddresses[4].stationBack =  ((FULL_TURN_STEPS / 24) * 18);
  
  gAddresses[5].address = 206;                // Einfahrt Lokschuppen Tor 2
  gAddresses[5].stationFront = ((FULL_TURN_STEPS / 24) * 17);
  gAddresses[6].stationBack =  ((FULL_TURN_STEPS / 24) * 17);
  
  gAddresses[6].address = 207;                // Einfahrt Lokschuppen Tor 3
  gAddresses[6].stationFront = ((FULL_TURN_STEPS / 24) * 16);
  gAddresses[6].stationBack =  ((FULL_TURN_STEPS / 24) * 16);
  
  gAddresses[7].address = 208;                // Einfahrt Lokschuppen Tor 4
  gAddresses[7].stationFront = ((FULL_TURN_STEPS / 24) * 15);
  gAddresses[7].stationBack =  ((FULL_TURN_STEPS / 24) * 15);
  
  gAddresses[8].address = 209;                // Einfahrt Lokschuppen Tor 5
  gAddresses[8].stationFront = ((FULL_TURN_STEPS / 24) * 14);
  gAddresses[8].stationBack =  ((FULL_TURN_STEPS / 24) * 14);
  
  gAddresses[9].address = 211;                // Einfahrt Lokschuppen Tor 6
  gAddresses[9].stationFront = ((FULL_TURN_STEPS / 24) * 10);
  gAddresses[9].stationBack =  ((FULL_TURN_STEPS / 24) * 10);
  
  gAddresses[10].address = 212;                // Einfahrt Lokschuppen Tor 7
  gAddresses[10].stationFront = ((FULL_TURN_STEPS / 24) * 9);
  gAddresses[10].stationBack =  ((FULL_TURN_STEPS / 24) * 9);
  
  gAddresses[11].address = 213;                // Einfahrt Lokschuppen Tor 8
  gAddresses[11].stationFront = ((FULL_TURN_STEPS / 24) * 8);
  gAddresses[11].stationBack =  ((FULL_TURN_STEPS / 24) * 8);
}

// This function is called whenever a normal DCC Turnout Packet is received
void notifyDccAccTurnoutOutput( uint16_t Addr, uint8_t Direction, uint8_t OutputPower )
{
  Serial.print("notifyDccAccTurnoutOutput: ") ;
  Serial.print(Addr,DEC) ;
  Serial.print(',');
  Serial.print(Direction,DEC) ;
  Serial.print(',');
  Serial.println(OutputPower, HEX) ;

  for (int i = 0; i < (sizeof(gAddresses) / sizeof(DCCAccessoryAddress)) ; i++)
  {
    if ((Addr == gAddresses[i].address) && ((Addr != lastAddr) || (Direction != lastDirection)) && OutputPower)
    {
      lastAddr = Addr ;
      lastDirection = Direction ;
      
      Serial.print(F("Moving to Station : "));
      Serial.println(i, DEC);

#ifdef STEPPER_ENABLE_PIN
      stepper1.enableOutputs();
#endif
      if (Direction)
      {
        Serial.print(F("Moving to Front Position : "));
        Serial.println(gAddresses[i].stationFront, DEC);
        stepper1.moveTo(gAddresses[i].stationFront);
        break;
      }
      else
      {
        Serial.print(F("Moving to Back Position : "));
        Serial.println(gAddresses[i].stationBack, DEC);
        stepper1.moveTo(gAddresses[i].stationBack);
        break;
      }
    }
  }
};

bool lastIsRunningState ; 

void setupStepperDriver()
{
#ifdef STEPPER_ENABLE_PIN
  stepper1.setPinsInverted(false, false, true); // Its important that these commands are in this order
  stepper1.setEnablePin(STEPPER_ENABLE_PIN);    // otherwise the Outputs are NOT enabled initially
#endif
   
  stepper1.setMaxSpeed(210.0);
  stepper1.setAcceleration(9);
  stepper1.setSpeed(210);
 
  lastIsRunningState = stepper1.isRunning();
}

void moveToHomePosition()
{
  pinMode(HOME_SENSOR_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(HOME_SENSOR_PIN), interruptEvent, FALLING);

  bInterruptDetected = false;
 
  Serial.println(F("Performing 2 complete turns to find home."));
  stepper1.move(FULL_TURN_STEPS * 2);
}

void interruptEvent()
{
  detachInterrupt(digitalPinToInterrupt(HOME_SENSOR_PIN));
  bInterruptDetected = true;
}

void setupDCCDecoder()
{
  Serial.println(F("Setting up DCC Decorder..."));

  // Setup which External Interrupt, the Pin it's associated with that we're using and enable the Pull-Up 
  Dcc.pin(0, 2, 1);
  
  // Call the main DCC Init function to enable the DCC Receiver
  Dcc.init( MAN_ID_DIY, 10, CV29_ACCESSORY_DECODER | CV29_OUTPUT_ADDRESS_MODE, 0 );
}

void setup()
{
  Serial.begin(115200);
  while(!Serial);   // Wait for the USB Device to Enumerate

  Serial.println(F("Example Turntable Program - www.dccinterface.com"));

  ConfigureStations();
  
  setupStepperDriver();

  Serial.println(F("Finding home...."));
  moveToHomePosition();
}

void loop()
{
  if (bInterruptDetected)
  {
    bInterruptDetected = false;
    bHomePositionFound = true;

    Serial.println(F("Found Home - Setting Current Position to 0"));

    stepper1.setCurrentPosition(0);

    Serial.print("Moving to position ");
    Serial.println(entryStation, DEC);
#ifdef STEPPER_ENABLE_PIN
    stepper1.enableOutputs();
#endif
    stepper1.moveTo(entryStation);

    setupDCCDecoder();
  }

  if (bHomePositionFound)
  {
    // You MUST call the NmraDcc.process() method frequently from the Arduino loop() function for correct library operation
    Dcc.process();
  }

  // Process the Stepper Library
  stepper1.run();

#ifdef STEPPER_ENABLE_PIN
  if(stepper1.isRunning() != lastIsRunningState)
  {
    lastIsRunningState = stepper1.isRunning();
    if(!lastIsRunningState)
    {
      stepper1.disableOutputs();
      Serial.println(F("Disable Stepper Outputs"));
    }
  }
#endif  
}
