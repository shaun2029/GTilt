/*
    GTilt - Game controller tilt control addition.
    Copyright Shaun Simpson 2015
*/

//Add the SPI library so we can communicate with the ADXL345 sensor
#include <SPI.h>
#include <EEPROM.h>
#include <avr/sleep.h>

//Assign the Chip Select signal to pin 10.
int CS=10;

//This is a list of some of the registers available on the ADXL345.
//To learn more about these and the rest of the registers on the ADXL345, read the datasheet!
char POWER_CTL = 0x2D;	//Power Control Register
char DATA_FORMAT = 0x31;
char DATAX0 = 0x32;	//X-Axis Data 0
char DATAX1 = 0x33;	//X-Axis Data 1
char DATAY0 = 0x34;	//Y-Axis Data 0
char DATAY1 = 0x35;	//Y-Axis Data 1
char DATAZ0 = 0x36;	//Z-Axis Data 0
char DATAZ1 = 0x37;	//Z-Axis Data 1

// Outputs
#define LEFT 3
#define RIGHT 2
#define UP 4
#define DOWN 5
#define POWER 7

// Input
#define BUTTON_1 6

// Mode limits
#define MIN_MODE 10
#define MAX_MODE 120
#define DEFAULT_MODE 30

// EEPROM addresses
#define NV_MAGIC_1 0
#define NV_MAGIC_2 1
#define NV_MODE 3

#define FILTERSIZE 4
#define FILTEROVERSAMPLE 2
#define VECTORBOOST 0

#define LOOPTIME 20

#define DEBUG
//#define DEBUG_READINGS

//This buffer will hold values read from the ADXL345 registers.
char values[10];
//These variables will be used to hold the x,y and z axis accelerometer values.
int x,y,z;

byte mode = 0;

void setup(){ 
  //Initiate an SPI communication instance.
  SPI.begin();
  //Configure the SPI connection for the ADXL345.
  SPI.setDataMode(SPI_MODE3);

  #ifdef DEBUG 
  //Create a serial connection to display the data on the terminal.
  Serial.begin(115200);
  #endif

  pinMode(POWER, OUTPUT);
  digitalWrite(POWER, HIGH);
  
  //Set up the Chip Select pin to be an output from the Arduino.
  pinMode(CS, OUTPUT);
  //Before communication starts, the Chip Select pin needs to be set high.
  digitalWrite(CS, HIGH);

  pinMode(LEFT, OUTPUT);
  digitalWrite(LEFT, LOW);
  pinMode(RIGHT, OUTPUT);
  digitalWrite(RIGHT, LOW);
  pinMode(UP, OUTPUT);
  digitalWrite(UP, LOW);
  pinMode(DOWN, OUTPUT);
  digitalWrite(DOWN, LOW);

  pinMode(BUTTON_1, INPUT_PULLUP);

  if ((EEPROM.read(NV_MAGIC_1) != 77) || (EEPROM.read(NV_MAGIC_2) != 77)) {
    mode = DEFAULT_MODE;
    EEPROM.write(NV_MAGIC_1, 77);
    EEPROM.write(NV_MAGIC_2, 58);
    EEPROM.write(NV_MODE, mode);
  }

  mode = EEPROM.read(NV_MODE);
  // Sanity
  if (mode > MAX_MODE) {
    mode = MAX_MODE;
    EEPROM.write(NV_MODE, mode);
  } else if (mode < MIN_MODE) {
    mode = MIN_MODE;
    EEPROM.write(NV_MODE, mode);
  }

  #ifdef DEBUG 
  Serial.print('MODE: ', mode);
  #endif  
  
  //Put the ADXL345 into +/- 4G range by writing the value 0x01 to the DATA_FORMAT register.
  writeRegister(DATA_FORMAT, 0x01);
  //Put the ADXL345 into Measurement Mode by writing 0x08 to the POWER_CTL register.
  writeRegister(POWER_CTL, 0x08);  //Measurement mode  
}

// Powers down the system
void GoToSleep() {
  Serial.end();
  
  //Kill the SPI communication instance.
  SPI.end();

  digitalWrite(POWER, LOW);
  digitalWrite(CS, LOW);

  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);
 
  byte old_ADCSRA = ADCSRA;                        // disable ADC //
  ADCSRA = 0;                                      // disable ADC //

  byte old_PRR = PRR;                              // disable Internal modules//
  PRR = 0xFF;                                      // disable Internal modules//
  
  while(true) {  
      MCUSR = 0;                                       // clear various "reset" flags// 
  
      // Sleep Activation //
      set_sleep_mode (SLEEP_MODE_PWR_DOWN);            //Sleep mode Selection//
      sleep_enable();                                  //Sleep Now//
  
      // turn off brown-out enable in software//
      MCUCR = bit (BODS) | bit (BODSE);                //Brown out settings
      MCUCR = bit (BODS);                              //Brown out set.
      sleep_cpu ();     //CPU is now sleeping
  }
}

void loop(){
  static int lastX[FILTERSIZE];
  static int lastY[FILTERSIZE];
  static int lastFilteredX = 0;
  static int lastFilteredY = 0;
  static int offsetX = 0;
  static int offsetY = 0;
  static bool firstLoop = true;
  static unsigned long powerOffTime = millis();
  static unsigned long centerTime = millis();
  
  static unsigned long loopCount = 0;
  loopCount += 1;

  // Get good reading
  if (firstLoop) {
    for (int i=0; i<10; i++) {
      readRegister(DATAX0, 6, values);
      delay(10);
    }
  }

  // Oversample
  for (int o = 0; o<FILTEROVERSAMPLE; o++) {
    //Reading 6 bytes of data starting at register DATAX0 will retrieve the x,y and z acceleration values from the ADXL345.
    //The results of the read operation will get stored to the values[] buffer.
    readRegister(DATAX0, 6, values);
  
    //The ADXL345 gives 10-bit acceleration values, but they are stored as bytes (8-bits). To get the full value, two bytes must be combined for each axis.
    //The X value is stored in values[0] and values[1].
    x = ((int)values[1]<<8)|(int)values[0];
    //The Y value is stored in values[2] and values[3].
    y = ((int)values[3]<<8)|(int)values[2];
    //The Z value is stored in values[4] and values[5].
    z = ((int)values[5]<<8)|(int)values[4];
  
    // Filter FIFO
    for (int i=FILTERSIZE-1; i > 0; i--) {
      lastX[i] = lastX[i-1];
      lastY[i] = lastY[i-1];
    }
    
    lastX[0] = x;
    lastY[0] = y;
    
    delay(LOOPTIME / FILTEROVERSAMPLE);
  }

  // Filtering
  for (int i=1; i < FILTERSIZE; i++) {
    x += lastX[i];
    y += lastY[i];
  }

  x /= FILTERSIZE;
  y /= FILTERSIZE;
  
  // Record Y offest at start
  if (firstLoop) {
    offsetY = -y;
    firstLoop = false;
  }

// VECTOR BOOST - Add synthetic input to improve direction change
//bool directionChange = (((lastX[0] >> 3 < lastX[1] >> 3) && (lastX[0] >> 3 > lastX[2] >> 3)));// || ((lastX[0] >> 4 > lastX[1] >> 4) && (lastX[0] >> 4 < lastX[2] >> 4)));

int vX = (lastX[0] + 360) - (lastX[1] + 360);

if (vX > 10) {
  x += mode;
  lastX[1] = x;
  lastX[2] = x;
  Serial.println("RIGHT");
} else if (vX < -10) {
  x -= mode;
  lastX[1] = x;
  lastX[2] = x;
  Serial.println("LEFT");
}

int vY = (lastY[0] + 360) - (lastY[1] + 360);

if (vY > 10) {
  y += mode;
  lastY[1] = y;
  lastY[2] = y;
  Serial.println("DOWN");
} else if (vY < -10) {
  y -= mode;
  lastY[1] = y;
  lastY[2] = y;
  Serial.println("UP");
}

  if (digitalRead(BUTTON_1) == LOW) {
    
    // Power off if held for 3 seconds
    unsigned long timeout = millis() + 3000;
    
    while (digitalRead(6) == LOW) {
      delay(100);
      if (millis() > timeout) {
        GoToSleep();
      }
    }
    
    mode = abs(x + offsetX);
    if (mode < MIN_MODE) mode = MIN_MODE;
    if (mode > MAX_MODE) mode = MAX_MODE;
    EEPROM.write(NV_MODE, mode);

    #ifdef DEBUG 
    Serial.print('SET MODE: ', mode);
    #endif  
    
    if (x + offsetX > 0) {
      digitalWrite(LEFT, LOW);
      digitalWrite(RIGHT, HIGH);
      digitalWrite(UP, LOW);
      digitalWrite(DOWN, LOW);
  
      delay(1000);
  
      digitalWrite(LEFT, HIGH);
      digitalWrite(RIGHT, LOW);
      digitalWrite(UP, LOW);
      digitalWrite(DOWN, LOW);
  
      delay(1000);
    } else {
      digitalWrite(LEFT, HIGH);
      digitalWrite(RIGHT, LOW);
      digitalWrite(UP, LOW);
      digitalWrite(DOWN, LOW);
  
      delay(1000);
  
      digitalWrite(LEFT, LOW);
      digitalWrite(RIGHT, HIGH);
      digitalWrite(UP, LOW);
      digitalWrite(DOWN, LOW);
  
      delay(1000);
    }
  }

  // If the device has not been moved for 60 seconds power it down
  if ((lastFilteredX >> 3 != x >> 3) || (lastFilteredY >> 3 != y >> 3))  {
    powerOffTime = millis();
  }

  // Test if it is time to power down
  if (millis() - powerOffTime > 30000) {
    GoToSleep();
  }

  // If the device has not been moved assume recentering
  if ( (abs(lastFilteredX) > 20) || (abs(lastFilteredY) > 45) || (lastFilteredX >> 3 != x >> 3) || (lastFilteredY >> 3 != y >> 3))  {
    centerTime = millis();
  }

  // If the device has not been moved assume it is centered
  if (millis() - centerTime > 5000) {
    offsetY = -y;
    centerTime = millis();
  }

  lastFilteredX = x;
  lastFilteredY = y;

  int cycle = loopCount % 1;
  int cycleMode = mode / 4;
  cycleMode += (mode * cycle) / 2;

  if (x + offsetX > cycleMode) {  
    digitalWrite(LEFT, HIGH);
  } else {
    digitalWrite(LEFT, LOW);
  } 

  if (x + offsetX < -cycleMode) {  
    digitalWrite(RIGHT, HIGH);
  } else {
    digitalWrite(RIGHT, LOW);
  } 

  if (y + offsetY > cycleMode) {  
    digitalWrite(UP, HIGH);
  } else {
    digitalWrite(UP, LOW);
  } 
  
  if (y + offsetY < -cycleMode) {  
    digitalWrite(DOWN, HIGH);
  } else {
    digitalWrite(DOWN, LOW);
  } 
  
  #ifdef DEBUG_READINGS
  //Print the results to the terminal.
  Serial.print(x, DEC);
  Serial.print(',');
  Serial.print(y + offsetY, DEC);
  Serial.print(',');
  Serial.println(z, DEC);      
  #endif  
  
  delay(LOOPTIME / FILTEROVERSAMPLE); 
}

//This function will write a value to a register on the ADXL345.
//Parameters:
//  char registerAddress - The register to write a value to
//  char value - The value to be written to the specified register.
void writeRegister(char registerAddress, char value){
  //Set Chip Select pin low to signal the beginning of an SPI packet.
  digitalWrite(CS, LOW);
  //Transfer the register address over SPI.
  SPI.transfer(registerAddress);
  //Transfer the desired register value over SPI.
  SPI.transfer(value);
  //Set the Chip Select pin high to signal the end of an SPI packet.
  digitalWrite(CS, HIGH);
}

//This function will read a certain number of registers starting from a specified address and store their values in a buffer.
//Parameters:
//  char registerAddress - The register addresse to start the read sequence from.
//  int numBytes - The number of registers that should be read.
//  char * values - A pointer to a buffer where the results of the operation should be stored.
void readRegister(char registerAddress, int numBytes, char * values){
  //Since we're performing a read operation, the most significant bit of the register address should be set.
  char address = 0x80 | registerAddress;
  //If we're doing a multi-byte read, bit 6 needs to be set as well.
  if(numBytes > 1)address = address | 0x40;
  
  //Set the Chip select pin low to start an SPI packet.
  digitalWrite(CS, LOW);
  //Transfer the starting register address that needs to be read.
  SPI.transfer(address);
  //Continue to read registers until we've read the number specified, storing the results to the input buffer.
  for(int i=0; i<numBytes; i++){
    values[i] = SPI.transfer(0x00);
  }
  //Set the Chips Select pin high to end the SPI packet.
  digitalWrite(CS, HIGH);
}
