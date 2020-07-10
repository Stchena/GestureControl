#include <Arduino.h>
#include <SPI.h>
#include <ESP8266WiFi.h>

// change this define to 1 for increased serial monitor cluttering
#define DEBUG_ON 0

volatile uint8_t master_ready = 1;
volatile uint8_t gesture_type = 0;
String gesture_name = "NON";

const char* ssid = "pennyisafreeloader"; // guest wifi
const char* pwd = "BigBangThe0ry"; // guest wifi pwd
const char* server = "migu.ovh"; // vps
const char* _getLink = "http://migu.ovh";

String prev_gesture = "NON";

WiFiClient client;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  SPI.begin();

  Serial.println("Connecting");
  WiFi.begin(ssid, pwd);
  while(WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println(".");
  }
  Serial.println("\nWiFiConnected");
}

void loop() {
  // put your main code here, to run repeatedly:
  // Get Gesture from STM32 via SPI
  if(master_ready)
    gesture_type = SPI.transfer(master_ready); // this sends data but at the same time receives
  if(gesture_type) {
    prev_gesture = gesture_name;
    switch (gesture_type)
		{
			case 1:			  gesture_name = "UP";	break;
			case 2:				gesture_name = "DOWN";	break;
			case 4:				gesture_name = "LEFT";	break;
			case 8:				gesture_name = "RIGHT"; 	break;
			case 16:			gesture_name = "FORWARD";	break;
			case 32:			gesture_name = "BACKWARD"; 	break;
			case 64:			gesture_name = "CLOCKWISE";	break;
			case 128:	    gesture_name = "COUNTER_CLOCKWISE"; 	break;
			default:
				 gesture_name = "UNRECOGNIZED";
				 break;
    }
    if(DEBUG_ON) {
      Serial.println("previous:");
      Serial.println(prev_gesture);
      Serial.println("current:");
      Serial.println(gesture_name);
    }
  }
  // Post Gesture to server via WiFi
  if(gesture_name != prev_gesture) {
    if(client.connect(server, 5000)) {
      Serial.println("Gesture is: ");
      Serial.println(gesture_name);
      
      client.print("POST /update HTTP/1.1\n");
      client.print("Host: migu.ovh\n");
      client.print("Connection: close\n");
      client.print("Content-Length: ");
      client.print(gesture_name.length());
      client.print("\n\n");
      client.print(gesture_name);
    }
    else Serial.println("Gesture Unchanged!");
  }
  client.stop();
}