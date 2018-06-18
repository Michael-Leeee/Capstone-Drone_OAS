
// I2C Sonar Definitions
#define SCL_PIN 5              //Default SDA is Pin5 PORTC for the UNO -- you can set this to any tristate pin
#define SCL_PORT PORTC 
#define SDA_PIN 4              //Default SCL is Pin4 PORTC for the UNO -- you can set this to any tristate pin
#define SDA_PORT PORTC
#define I2C_TIMEOUT 100        //Define a timeout of 100 ms -- do not wait for clock stretching longer than this time
#define I2C_MAXWAIT 0
//#define I2C_CPUFREQ (F_CPU/8)//Useful if you plan on doing any clock switching
#define I2C_FASTMODE 1         //Run in fast mode (400 kHz)
//#define I2C_SLOWMODE 1         //If you do not define the mode it will run at 100kHz with this define set to 1 it will run at 25kHz
#include <SoftI2CMaster.h>     //You will need to install this library

/*
 * Maxbotix Sensor Address Allocation.
 * Sensor will only accept EVEN address values. Invalid Address Values: 0,80,164,170, Any odd numbers 
 * Sensor 1 Address: 200
 * Sensor 2 Address: 202
 * Sensor 3 Address: 204
 * Sensor 4 Address: 206
 * Sensor 5 Address: 208
 * Sensor 6 Address: 210
 */


// LED Ring Definitions
#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
  #include <avr/power.h>
#endif
#define PIN 4
// Parameter 1 = number of pixels in strip
// Parameter 2 = Arduino pin number (most are valid)
// Parameter 3 = pixel type flags, add together as needed:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
//   NEO_RGBW    Pixels are wired for RGBW bitstream (NeoPixel RGBW products)
Adafruit_NeoPixel ring = Adafruit_NeoPixel(16, PIN, NEO_GRB + NEO_KHZ800);



char* sensor_list[] = {"RearLeft","FrontLeft","FrontRight","RearRight"};
int address_list[] = {202,200,204,206};
unsigned long current_time;
int sensor1_readings[5]; // Create a List of Readings to get the median readings
int sensor2_readings[5];
int sensor3_readings[5];
int sensor4_readings[5];

void setup() {
  // Initialize both serial and I2C bus
  Serial.begin(57600); 
  i2c_init();

  address_polling();
  //default_address_change(224,206); //Only uncomment when you want to change the specific address of the maxboxtix sensor connected
  delay(500);

  ring.begin();
  ring.show(); // Initialize all pixels to 'off'
  /*
  for (int i=0;i<4;i++) {
    update_color(ring.Color(127, 127, 127),i, 50); delay(1000);     
    update_color(ring.Color(255, 0, 0),i, 50); delay(1000);  //0-2m     
    update_color(ring.Color(255, 30, 0),i, 50); delay(1000);  //2-3m
    update_color(ring.Color(127, 90, 0),i, 50); delay(1000);  //3-4m
    update_color(ring.Color(60, 90, 0),i, 50); delay(1000);  //4-5m
    update_color(ring.Color(0, 255, 0),i, 50); delay(1000);   //5-7 // Green Color
  }*/

  
}

void loop() {
  // Read all 4 sensors and return readings
  delay(600);
  for (int i=0; i<4; i++) { 
    boolean error =0;
    int sensor_reading;
    current_time = micros();
    error = start_sensor(address_list[i]);
    Serial.print("Time taken for start_sensor is ");
    Serial.println(get_time_taken(current_time));
    if (!error) {
      //delay(100); // Delay here is important for the reading to be received
      current_time = micros();
      sensor_reading = read_sensor(address_list[i]);
      Serial.print(sensor_list[i]);Serial.print(" Sensor Reading is ");
      Serial.print(sensor_reading);Serial.println("cm");
      Serial.print("Time taken for sensor reading is ");
      Serial.print(get_time_taken(current_time));Serial.println("ms");
    }
    if (sensor_reading==0) {  //Error Reading Color is White
      update_color(ring.Color(127, 127, 127),i, 50);       
    }
    else if (sensor_reading<200) {  //Red: 0-2m 
      update_color(ring.Color(255, 0, 0),i, 50);      
    }
    else if (sensor_reading>=200 && sensor_reading<300) {  //Amber: 2-3m
      update_color(ring.Color(255, 30, 0),i, 50);
    }
    else if (sensor_reading>=300 && sensor_reading<400) {  //Yellow: 3-4m
      update_color(ring.Color(127, 90, 0),i, 50);
    }
    else if (sensor_reading>=400 && sensor_reading<500) {  //Yellow-Green: 4-5m
      update_color(ring.Color(0, 255, 0),i, 50);
    }
    else if (sensor_reading>=500 && sensor_reading<766) { //765 is the maximum number you can get
      update_color(ring.Color(0, 0, 200),i, 50);  //Green: 5-7m
    }
  }
  Serial.println("================================");
}



////////////////////////////////////////////////////////////////
// Function: Poll all possible addresses to find a sensor //
////////////////////////////////////////////////////////////////
void address_polling(){
  boolean error = 0;  //Create a bit to check for catch errors as needed.
  int range = 0;
  Serial.println("Polling addresses...");
 
  //Walk through all possible addresses and check for a device that can receive the range command and will
  //    return two bytes.
  for (byte i=2; i!=0; i+=2){   //start at 2 and count up by 2 until wrapping to 0. Checks all addresses (2-254) except 0 (which cannot be used by a device)
    error = 0;
    error = start_sensor(i);    //Start the sensor and collect any error codes.
    if (!error){                //If you had an error starting the sensor there is little point in reading it.
      //delay(100);
      range = read_sensor(i);   //reading the sensor will return an integer value -- if this value is 0 there was an error
      Serial.println(i);
      if (range != 0){
        Serial.print("Device found at:");Serial.print(i);Serial.print(" Reported value of:");Serial.println(range);
      }  
    }
    else{
      Serial.print("Couldn't start:");Serial.println(i);
    }
  }
  Serial.println("Address polling complete.");
}


//////////////////////////////////////////////
// Function: Change the default address //
//////////////////////////////////////////////
void default_address_change(byte current_address, byte new_address){
  boolean error = 0;  //Create a bit to check for catch errors as needed.
  int range;
  
  Serial.println("Checking for reading at the default given address");
  
  error = start_sensor(current_address);    //Start the sensor and collect any error codes.
  if (!error){                
    delay(100);
    range = read_sensor(current_address);   //reading the sensor will return an integer value -- if this value is 0 there was an error
    Serial.print("R:");Serial.println(range);
  }
  else {
    Serial.println("Error found in reading sensor readings at given address");
  }
  
  //Change the address from current_address to new_address
  error = 0;
  error = change_address(current_address,new_address);  //Change the address
  delay(200);    //Wait 125ms for the sensor to save the new address and reset
  Serial.print("Error is "); Serial.println(error);
  
  Serial.println("Checking for reading at the new address");
  
  //Take a range reading at the new address
  error = 0;
  error = start_sensor(new_address);     //Same as above but at the new address
  if (!error){
    delay(100);
    range = read_sensor(new_address);
    Serial.print("N:");Serial.println(range);
  }  
  Serial.print("Sensor has been successful changed to "); Serial.println(new_address); 
 
}


/////////////////////////////////////////
// Function: Change the sensor address //
/////////////////////////////////////////
//Uses the I2C library to change the address of a sensor at a given address
//Collects and reports an error bit where: 1 = there was an error or 0 = there was no error.
//INPUTS: byte oldaddress = the current address of the sensor that we want to change
//INPUTS: byte newddress  = the address that we want to change the sensor to
//OUPUTS: bit  errorlevel = reports if the function was successful in changing the address: 1 = the function had an
//      error, 0 = the function was successful
boolean change_address(byte oldaddress,byte newaddress){
  //note that the new address will only work as an even number (odd numbers will round down)
  boolean errorlevel = 0;
  oldaddress = oldaddress & B11111110;  //Do a bitwise 'and' operation to force the last bit to be zero -- we are writing to the address.
  errorlevel = !i2c_start(oldaddress) | errorlevel; //Start communication at the new address and track error codes
  errorlevel = !i2c_write(170) | errorlevel;        //Send the unlock code and track the error codes
  errorlevel = !i2c_write(165) | errorlevel;        //Send the unlock code and track the error codes
  errorlevel = !i2c_write(newaddress) | errorlevel; //Send the new address
  i2c_stop();
  return errorlevel;
}


///////////////////////////////////////////////////
// Function: Start a range reading on the sensor //
///////////////////////////////////////////////////
//Uses the I2C library to start a sensor at the given address
//Collects and reports an error bit where: 1 = there was an error or 0 = there was no error.
//INPUTS: byte bit8address = the address of the sensor that we want to command a range reading
//OUPUTS: bit  errorlevel = reports if the function was successful in taking a range reading: 1 = the function
//  had an error, 0 = the function was successful
boolean start_sensor(byte bit8address){
  boolean errorlevel = 0;
  bit8address = bit8address & B11111110;               //Do a bitwise 'and' operation to force the last bit to be zero -- we are writing to the address.
  errorlevel = !i2c_start(bit8address) | errorlevel;   //Run i2c_start(address) while doing so, collect any errors where 1 = there was an error.
  errorlevel = !i2c_write(81) | errorlevel;            //Send the 'take range reading' command. (notice how the library has error = 0 so I had to use "!" (not) to invert the error)
  i2c_stop();
  return errorlevel;
}


///////////////////////////////////////////////////////////////////////
// Function: Read the range from the sensor at the specified address //
///////////////////////////////////////////////////////////////////////
//Uses the I2C library to read a sensor at the given address
//Collects errors and reports an invalid range of "0" if there was a problem.
//INPUTS: byte  bit8address = the address of the sensor to read from
//OUPUTS: int   range = the distance in cm that the sensor reported; if "0" there was a communication error
int read_sensor(byte bit8address){
  boolean errorlevel = 0;
  int range = 0;
  byte range_highbyte = 0;
  byte range_lowbyte = 0;
  bit8address = bit8address | B00000001;  //Do a bitwise 'or' operation to force the last bit to be 'one' -- we are reading from the address.
  errorlevel = !i2c_start_wait(bit8address) | errorlevel;
  range_highbyte = i2c_read(0);           //Read a byte and send an ACK (acknowledge)
  range_lowbyte  = i2c_read(1);           //Read a byte and send a NACK to terminate the transmission
  i2c_stop();
  range = (range_highbyte * 256) + range_lowbyte;  //compile the range integer from the two bytes received.
  if(errorlevel){
    return 0;
  }
  else{
    return range;
  }
}


///////////////////////////////////////////////////////////////////////
// Function: Change LED ring color to indicate proximity             //
///////////////////////////////////////////////////////////////////////
// for 16 pixels to divide to 4 sectors

void update_color(uint32_t c, int orientation, uint8_t wait) {
  int max_pixel_index = 4*orientation + 4;
  for(uint16_t i=4*orientation; i<max_pixel_index; i++) {
    ring.setPixelColor(i, c);
  }
  ring.show();
  //delay(wait);
}

///////////////////////////////////////////////////////////////////////
// Function: Get Time Taken for each Reading                         //
///////////////////////////////////////////////////////////////////////
unsigned long get_time_taken(unsigned long start_time) {
  unsigned long current_time = micros();
  current_time -= start_time;
  return current_time;
}


