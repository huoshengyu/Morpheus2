#include <Wire.h>
#include <SparkFun_I2C_Mux_Arduino_Library.h>
#include <Sparkfun_DRV2605L.h>

#define QWIIC_MUX_DEFAULT_ADDRESS 0x70
SFE_HMD_DRV2605L HMD[8];
QWIICMUX myMux;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  if (myMux.begin() == false)
  {
    while (1)
      ;
  } 

  digitalWrite(42, HIGH);     // now this is safe  disableotherchannel(0x71);
  digitalWrite(43, HIGH);     // now this is safe  disableotherchannel(0x71);
  digitalWrite(44, HIGH);     // now this is safe  disableotherchannel(0x71);
  digitalWrite(45, HIGH);     // now this is safe  disableotherchannel(0x71);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW); // Or HIGH, depending on desired state
  disableotherchannel(0x72);
  disableotherchannel(0x73);
  delay(200);
  for (int i = 0; i<8; i++){
    singleActuator(i);
    HMD[i].begin();
    HMD[i].writeDRV2605L(OVERDRIVECLAMP_REG,0xFF);
    HMD[i].Mode(0x05);
    HMD[i].MotorSelect(0x86);
    HMD[i].Library(6);
  }
}

void performI2CChannelOperation(uint8_t i){
  singleActuator(i);
}

void singleActuator(uint8_t i){
  if (i>7) return;
  Wire.beginTransmission(QWIIC_MUX_DEFAULT_ADDRESS);
  Wire.write(1<<i);
  Wire.endTransmission();
}
void disableActuator(uint8_t i){
  Wire.beginTransmission(QWIIC_MUX_DEFAULT_ADDRESS);
  Wire.write(0<<i);
  Wire.endTransmission();
}
void disableotherchannel(uint8_t address){
  Wire.beginTransmission(address);
  Wire.write(0);
  Wire.endTransmission();
}

void loop() {
  if (Serial.available() > 0) {
    char incoming = Serial.read();
    if (incoming == '2') {
      // Wait for data to become available
      while (!Serial.available()) {
        delay(10);
      }
      // Read position string until newline
      String pos_data = Serial.readStringUntil('\n');
      int commaIndex = pos_data.indexOf(',');
      if (commaIndex == -1) return;  // malformed string

      float x = pos_data.substring(0, commaIndex).toFloat();
      float z = pos_data.substring(commaIndex + 1).toFloat();

      // Activate based on x
      if (x < 0) {
        singleActuator(0); HMD[0].RTP(127);
        singleActuator(1); HMD[1].RTP(0); HMD[1].stop(); disableActuator(1);
      } else {
        singleActuator(1); HMD[1].RTP(127);
        singleActuator(0); HMD[0].RTP(0); HMD[0].stop(); disableActuator(0);
      }

      // Activate based on z
      if (z < 0) {
        singleActuator(3); 
        HMD[3].RTP(0); 
        HMD[3].stop(); 
        disableActuator(3);
        singleActuator(2); 
        HMD[2].RTP(127);
        
      } else {
        singleActuator(2); 
        HMD[2].RTP(0); 
        HMD[2].stop(); 
        disableActuator(2);
        singleActuator(3); 
        HMD[3].RTP(127);
      }
    }
  }
}