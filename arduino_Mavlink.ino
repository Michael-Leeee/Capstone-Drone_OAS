

/*
 * Each Mavlink packet has a length of 17 bytes.
 * 6 bytes header + 9 bytes payload + 2 bytes checksum = 17
 * 
 * ###### 6 bytes Header #######
 * 0. Packet Start Sign header: always 0xFE
 * 1. Payload length: Eg. 9
 * 2. Packet Sequence:
 * 3. System ID: ID of Sending System.For different MAVs on same network =1(For copters)
 * 4. Component ID: ID of Sending subsystem.
 * 5. Message ID: Defines that the payload means and how it should be correctly decoded.
 */

/*
 * Things to refer to:
 * http://mavlink.org/messages/common#DISTANCE_SENSOR
 * https://github.com/patrickpoirier51/POC
 * https://discuss.ardupilot.org/t/avoidance-experiments-with-the-poc-and-benewake-tfmini/25277
 * https://discuss.ardupilot.org/t/radar-distance-sensor-screen/18983
 * 
 */

/*
 * Static distance is used to test if the DISTANCE_SENSOR Mavlink message is working properly.
 * Pixhawk will absorb this Mavlink message and display on the Mission Planner radar screen.
 */
#include "mavlink.h"
#include "mavlink_msg_distance_sensor.h"

#include <SoftwareSerial.h>
SoftwareSerial mavSerial(10,11); //RX, TX
//RX is digital pin 10 (connect to TX of other device)
//TX is digital pin 11 (connect to RX of other device)
//Orange Wire pixhawk TX, RED wire pixhawk RX

const int MAV_DISTANCE_SENSOR_ULTRASOUND = 0;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(57600);
  mavSerial.begin(57600);
  Serial.println("Mavlink Initializing..");

}

void loop() {
  // put your main code here, to run repeatedly:
  //command_heartbeat();
  command_distance_1();
  command_distance_2();
  command_distance_3();
  command_distance_4();
}


///////////////////////////////////////////////////////////////////
// Function: To send Heartbeat to Pixhawk to check connection    //
///////////////////////////////////////////////////////////////////
void command_heartbeat() {
  //< ID 1 for this system
  int sysid = 1;                   
  //< The component sending the message.
  int compid = 0;    
  
  // Define the system type, in this case an Octocoptor
  uint8_t system_type = MAV_TYPE_OCTOROTOR;
  uint8_t autopilot_type = 0;
  
  uint8_t system_mode = 0; 
  uint32_t custom_mode = 0;                
  uint8_t system_state = 0;
  
  // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  
  // Pack the message
  mavlink_msg_heartbeat_pack(sysid,compid, &msg, system_type, autopilot_type, system_mode, custom_mode, system_state);
  
  // Copy the message to the send buffer
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  
  // Send the message 
  //delay(1);
  mavSerial.write(buf, len);
  Serial.println("Heartbeat sent!");  
}


void command_distance_1() {

// READ THE DISTANCE SENSOR
  float dist1 = 210; //in centimeters

  //MAVLINK DISTANCE MESSAGE
  int sysid = 1;                   
  //< The component sending the message.
  int compid = 0;    

  uint32_t time_boot_ms = 0; /*< Time since system boot*/
  uint16_t min_distance = 20; /*< Minimum distance the sensor can measure in centimeters*/
  uint16_t max_distance = 765; /*< Maximum distance the sensor can measure in centimeters*/
  uint16_t current_distance = dist1; /*< Current distance reading*/
  uint8_t type = MAV_DISTANCE_SENSOR_ULTRASOUND; /*< Type from MAV_DISTANCE_SENSOR enum.*/
  uint8_t id = 1; /*< Onboard ID of the sensor*/
  uint8_t orientation = 1; /*(0=forward, each increment is 45degrees more in clockwise direction), 24 (upwards) or 25 (downwards)*/
// Consumed within ArduPilot by the proximity class

  uint8_t covariance = 0; /*< Measurement covariance in centimeters, 0 for unknown / invalid readings*/
  // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // Pack the message
 mavlink_msg_distance_sensor_pack(sysid,compid,&msg,time_boot_ms,min_distance,max_distance,current_distance,type,id,orientation,covariance);

  // Copy the message to the send buffer
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  // Send the message (.write sends as bytes) 
  //delay(1);
  mavSerial.write(buf, len);
  Serial.println("Sensor 1 Distance Mavlink message is sent!");
}

void command_distance_2() {

// READ THE DISTANCE SENSOR
  float dist1 = 220;//in centimeters

  //MAVLINK DISTANCE MESSAGE
  int sysid = 1;                   
  //< The component sending the message.
  int compid = 0;    

  uint32_t time_boot_ms = 0; /*< Time since system boot*/
  uint16_t min_distance = 20; /*< Minimum distance the sensor can measure in centimeters*/
  uint16_t max_distance = 765; /*< Maximum distance the sensor can measure in centimeters*/
  uint16_t current_distance = dist1; /*< Current distance reading*/
  uint8_t type = MAV_DISTANCE_SENSOR_ULTRASOUND; /*< Type from MAV_DISTANCE_SENSOR enum.*/
  uint8_t id = 2; /*< Onboard ID of the sensor*/
  uint8_t orientation = 3; /*(0=forward, each increment is 45degrees more in clockwise direction), 24 (upwards) or 25 (downwards)*/
// Consumed within ArduPilot by the proximity class

  uint8_t covariance = 0; /*< Measurement covariance in centimeters, 0 for unknown / invalid readings*/
  // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // Pack the message
 mavlink_msg_distance_sensor_pack(sysid,compid,&msg,time_boot_ms,min_distance,max_distance,current_distance,type,id,orientation,covariance);

  // Copy the message to the send buffer
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  // Send the message (.write sends as bytes) 
  //delay(1);
  mavSerial.write(buf, len);
  Serial.println("Sensor 2 Distance Mavlink message is sent!");
}

void command_distance_3() {

// READ THE DISTANCE SENSOR
  float dist1 = 230;//in centimeters

  //MAVLINK DISTANCE MESSAGE
  int sysid = 1;                   
  //< The component sending the message.
  int compid = 0;    

  uint32_t time_boot_ms = 0; /*< Time since system boot*/
  uint16_t min_distance = 20; /*< Minimum distance the sensor can measure in centimeters*/
  uint16_t max_distance = 765; /*< Maximum distance the sensor can measure in centimeters*/
  uint16_t current_distance = dist1; /*< Current distance reading*/
  uint8_t type = MAV_DISTANCE_SENSOR_ULTRASOUND; /*< Type from MAV_DISTANCE_SENSOR enum.*/
  uint8_t id = 3; /*< Onboard ID of the sensor*/
  uint8_t orientation = 5; /*(0=forward, each increment is 45degrees more in clockwise direction), 24 (upwards) or 25 (downwards)*/
// Consumed within ArduPilot by the proximity class

  uint8_t covariance = 0; /*< Measurement covariance in centimeters, 0 for unknown / invalid readings*/
  // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // Pack the message
 mavlink_msg_distance_sensor_pack(sysid,compid,&msg,time_boot_ms,min_distance,max_distance,current_distance,type,id,orientation,covariance);

  // Copy the message to the send buffer
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  // Send the message (.write sends as bytes) 
  //delay(1);
  mavSerial.write(buf, len);
  Serial.println("Sensor 3 Distance Mavlink message is sent!");
}

void command_distance_4() {

// READ THE DISTANCE SENSOR
  float dist1 = 280;//in centimeters

  //MAVLINK DISTANCE MESSAGE
  int sysid = 1;                   
  //< The component sending the message.
  int compid = 0;    

  uint32_t time_boot_ms = 0; /*< Time since system boot*/
  uint16_t min_distance = 20; /*< Minimum distance the sensor can measure in centimeters*/
  uint16_t max_distance = 765; /*< Maximum distance the sensor can measure in centimeters*/
  uint16_t current_distance = dist1; /*< Current distance reading*/
  uint8_t type = MAV_DISTANCE_SENSOR_ULTRASOUND; /*< Type from MAV_DISTANCE_SENSOR enum.*/
  uint8_t id = 4; /*< Onboard ID of the sensor*/
  uint8_t orientation = 7; /*(0=forward, each increment is 45degrees more in clockwise direction), 24 (upwards) or 25 (downwards)*/
// Consumed within ArduPilot by the proximity class

  uint8_t covariance = 0; /*< Measurement covariance in centimeters, 0 for unknown / invalid readings*/
  // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // Pack the message
 mavlink_msg_distance_sensor_pack(sysid,compid,&msg,time_boot_ms,min_distance,max_distance,current_distance,type,id,orientation,covariance);

  // Copy the message to the send buffer
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  // Send the message (.write sends as bytes) 
  //delay(1);
  mavSerial.write(buf, len);
  Serial.println("Sensor 4 Distance Mavlink message is sent!");
}
