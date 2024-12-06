#include <SPI.h>
#include <mcp2515.h>


struct can_frame canMsg;  //creates struct for data type for canmsg
MCP2515 mcp2515(10); //CS pin for SPI



unsigned long currentTime;
unsigned long previousTime;
unsigned long interval = 500; //time interval for data pushing
char buffer[100];
int i = 0;




void setup() {
  Serial.begin(9600);
  Serial.println("Just started");

  SPI.begin();

  mcp2515.reset(); //clear all preset first
  mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ);  //bitrate set
  mcp2515.setNormalMode();
  
  canMsg.can_id = 0x35;   //can_id for transmission, lower id, means higher line priority
  canMsg.can_dlc = 1; //byte size of message being sent

}

void loop() {

  currentTime = millis();

  if(currentTime - previousTime >= interval){
    // i++;
    canMsg.data[0] = 0xC9;

    if (mcp2515.sendMessage(&canMsg) != MCP2515::ERROR_OK) {
      Serial.println("Error sending message");
    } 
    else {
      Serial.println("Message sent successfully");
    }
    
    Serial.print(canMsg.data[0]);
    previousTime = millis();
  }
}
