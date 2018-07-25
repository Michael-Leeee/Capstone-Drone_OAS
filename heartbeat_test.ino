#include <SoftwareSerial.h>
#include "mavlink.h" // change to path of whatever directory your libraries are located in. Make sure to include the mavlink.h header file from the ardupilot mega folder and not the common folder

#define DEBUG

#define RXpin 10
#define TXpin 11
SoftwareSerial Serial1(RXpin, TXpin); // sets up serial communication on pins 3 and 2



void setup() {
  Serial1.begin(57600); // begin serial communication
  Serial.begin(57600);
}

void loop() {
  int sysid=1; // not sure about sysid, compid, apparently sysid can be anything from 1 to 244. compid I think is usually 0
  int compid=0;
  int type=MAV_TYPE_OCTOROTOR; // all these MAV_etc_etc values are defined in the header file common.h They are all needed to send a message

  uint8_t system_type=MAV_TYPE_OCTOROTOR;
  uint8_t autopilot_type=0;

  uint8_t system_mode= MAV_MODE_PREFLIGHT;
  uint8_t custom_mode = 0;
  uint8_t system_state=MAV_STATE_STANDBY;
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  mavlink_msg_heartbeat_pack(sysid,compid, &msg, type, autopilot_type, system_mode, custom_mode, system_state);  // packing function for heartbeat message. Not really sure how this works, copied from online

  uint16_t len=mavlink_msg_to_send_buffer(buf, &msg);

  delay(1000);
  Serial1.write(buf, len);
  _MavLink_receive();

}

void _MavLink_receive(){ //function called by arduino to read any MAVlink messages sent by serial communication from flight controller to arduino
  mavlink_message_t msg;
  mavlink_status_t status;

  while(Serial1.available())
  {
    uint8_t c= Serial1.read();
    if(mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status))
    {
      Serial.print("DEBUG msgid:");Serial.println(msg.msgid);
      switch(msg.msgid)
      {
        case MAVLINK_MSG_ID_HEARTBEAT:
        mavlink_heartbeat_t hb;
        mavlink_msg_heartbeat_decode(&msg,&hb);
        #ifdef DEBUG
          Serial.print(millis());
          Serial.print("\ncustom_mode: ");Serial.println(hb.custom_mode);
          Serial.print("Type: ");Serial.println(hb.type);
          Serial.print("autopilot: ");Serial.println(hb.autopilot);
          Serial.print("base_mode: ");Serial.println(hb.base_mode);
          Serial.print("system_status: ");Serial.println(hb.system_status);
          Serial.print("mavlink_version: ");Serial.println(hb.mavlink_version);

          Serial.println();
        #endif

        break;
      }
    }
  }
}

