// ---------------------------------------------------------------------------
// Morpheus2: ROS2 control and feedback for manual teleoperation research
// Copyright (C) 2026 Brian Huo
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.
// ---------------------------------------------------------------------------

#include <ros.h>
// #include <geometry_msgs/msg/Vector3.h>
#include <std_msgs/msg/Float64.h>
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