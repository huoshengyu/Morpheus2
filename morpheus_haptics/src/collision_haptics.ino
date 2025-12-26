#include <ros.h>
// #include <geometry_msgs/Vector3.h>
#include <std_msgs/Float64.h>
#include <Adafruit_DRV2605.h>

Adafruit_DRV2605 drv;
ros::NodeHandle nh;

void messageCb(const std_msgs::Float64& message){
  Serial.println(message.data,4);
  if (message.data < 0.20){
    float voltage; 
    voltage = (0.18 - message.data) * 127 / 0.2;
    voltage = min(127, voltage);
    voltage = max(0, voltage);
    drv.setRealtimeValue(voltage);
    
  }
  else{
    drv.setRealtimeValue(0);
  }
  // digitalWrite(13, HIGH-digitalRead(13));   // blink the led
}
   
ros::Subscriber<std_msgs::Float64> sub("collision/nearest/distance", &messageCb );

void setup()
{
  drv.begin();
  drv.setMode(DRV2605_MODE_REALTIME);
  drv.writeRegister8(DRV2605_REG_OVERDRIVE,0xFF);
  drv.writeRegister8(DRV2605_REG_FEEDBACK,0x86);
  drv.useLRA();
  drv.selectLibrary(6);
  Serial.begin(57600);
  nh.getHardware() ->setBaud(57600);
  pinMode(13, OUTPUT);
  nh.initNode();
  nh.subscribe(sub);
}

void loop()
{
  nh.spinOnce();
  delay(100);
}