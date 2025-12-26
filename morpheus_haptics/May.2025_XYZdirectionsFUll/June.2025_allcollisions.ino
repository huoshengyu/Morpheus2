#include <Wire.h>
#include <SparkFun_I2C_Mux_Arduino_Library.h>
#include <Sparkfun_DRV2605L.h>
SFE_HMD_DRV2605L HMD[8];
QWIICMUX muxX;  // for MUX_X (0x70)
QWIICMUX muxZ;  // for MUX_Z (0x71)
#define MUX_X 0x70 // wrist and lower forearm address
#define MUX_Z 0x71 // upper forearm and upper arm address 
unsigned long lastActivationTime = 0;

int wristZ[] = {2, 3, 4, 6}; //wrist vibration motors
int lfz[]    = {0, 1, 5, 7};  //lower forearm vibration motors
int ufz[]    = {0, 3, 6, 7};//upper forearm vibration motors
int uaz[]    = {1, 2, 4, 5};  //upperarm vibration motors

int stepIndexZ = 0;
int revstepIndexZ = 3;
char lastMode = '0';  // '0' means no mode active yet

unsigned long activationTimesZ[8] = {0};
bool deactivatedZ[8] = {true, true, true, true, true, true, true, true};
unsigned long channelInterval = 30;
unsigned long channelDuration = 100;
uint8_t muxAddress = MUX_X;  // Declare global variable



void setup() {
  Serial.begin(9600);
  digitalWrite(42, HIGH);     // now this is safe  disableotherchannel(0x71);
  digitalWrite(43, HIGH);     // now this is safe  disableotherchannel(0x71);
  digitalWrite(44, HIGH);     // now this is safe  disableotherchannel(0x71);
  digitalWrite(45, HIGH);     // now this is safe  disableotherchannel(0x71);

  Wire.begin();

  if (!muxX.begin(MUX_X)) {
    Serial.println("Failed to init MUX_X");
    while (1);
  }
  if (!muxZ.begin(MUX_Z)) {
    Serial.println("Failed to init MUX_Z");
    while (1);
  }  
  for (int i = 0; i < 8; i++) {
      setupMotor(i, MUX_X);
      setupMotor(i, MUX_Z);
    }
  stopAllMotors();  // Clean initial shutdown
}


void selectChannels(uint8_t i, uint8_t muxAddress) {
  Wire.beginTransmission(muxAddress);
  Wire.write(1 << i);
  Wire.endTransmission();
}

void setupMotor(uint8_t i, uint8_t muxAddress) {
  selectChannels(i, muxAddress);
  HMD[i].begin();
  HMD[i].writeDRV2605L(OVERDRIVECLAMP_REG, 0xFF);
  HMD[i].Mode(0x05);
  HMD[i].MotorSelect(0x86);
  HMD[i].Library(6);
}

void singleActuator(uint8_t i,uint8_t muxAddress){
  if (i>7) return;
  Wire.beginTransmission(muxAddress);
  Wire.write(1<<i);
  Wire.endTransmission();
}

void disableotherchannel(uint8_t muxAddress){
  Wire.beginTransmission(muxAddress);
  Wire.write(0);
  Wire.endTransmission();
}
void deactivateOtherMotors(uint8_t activeCh, uint8_t muxAddr) {
  for (int i = 0; i < 8; i++) {
    if (i == activeCh) continue;

    singleActuator(i, muxAddr);
    HMD[i].RTP(0);
    HMD[i].stop();
    HMD[i].Mode(0x00);
  }
}
void stopAllMotors() {
  for (int i = 0; i < 8; i++) {
    // Select motor i on both MUX_X and MUX_Z to ensure full shutdown
    singleActuator(i, MUX_X);
    HMD[i].stop();
    HMD[i].Mode(0x00);  // Standby mode
    deactivatedZ[i] = true;
    singleActuator(i, MUX_Z);
    HMD[i].stop();
    HMD[i].Mode(0x00);
    deactivatedZ[i] = true;
  }

  // Finally, disable all mux outputs
  disableotherchannel(MUX_X);
  disableotherchannel(MUX_Z);
}

void disableActuator(uint8_t i,uint8_t muxAddress){
  Wire.beginTransmission(muxAddress);
  Wire.write(0<<i);
  Wire.endTransmission();
}

void simulActuate(uint8_t i, uint8_t j, uint8_t muxAddress) {
  Wire.beginTransmission(muxAddress);
  Wire.write((1 << i) | (1 << j));
  Wire.endTransmission();
}


void loop() {
  if (Serial.available() > 0) {
    char incoming = Serial.read();

    if (incoming >= '1' && incoming <= '4') {
      while (!Serial.available()) delay(10);

      String pos_data = Serial.readStringUntil('\n');
      int firstComma = pos_data.indexOf(',');
      int secondComma = pos_data.indexOf(',', firstComma + 1);
      if (firstComma == -1 || secondComma == -1) {
        Serial.println("Malformed data: " + pos_data);
        return;
      }

      float x = pos_data.substring(0, firstComma).toFloat();
      float y = pos_data.substring(firstComma + 1, secondComma).toFloat();
      float z = pos_data.substring(secondComma + 1).toFloat();

      Serial.print("X: "); Serial.print(z);
      Serial.print(" | Y: "); Serial.print(y);
      Serial.print(" | Z: "); Serial.println(x);
      if (incoming != lastMode) {
        stopAllMotors();

        lastMode = incoming;
      }
      handleFeedback(incoming, x, y, z);
    } else {
      stopAllMotors(); // Unknown command
    }
  }
}


void handleFeedback(char mode, float x, float y, float z) {
  if (mode == '1' || mode == '2'){
    disableotherchannel(MUX_Z);
    muxAddress = MUX_X;
  }
  if (mode == '3' || mode == '4'){
    disableotherchannel(MUX_X);
    muxAddress = MUX_Z;

  }

  float absX = abs(x), absY = abs(y), absZ = abs(z);
  bool x_in_range = (absX < 0.05 && absX >= 0.01);
  bool y_in_range = (absY < 0.05 && absY >= 0.01);
  bool z_in_range = (absZ < 0.05 && absZ >= 0.01);



  if (x_in_range || y_in_range || z_in_range) {
    float minDist = 1.0;
    char axis = 'N';
    if (x_in_range && absX < minDist) { minDist = absX; axis = 'Z'; }
    if (y_in_range && absY < minDist) { minDist = absY; axis = 'X'; }
    if (z_in_range && absZ < minDist) { minDist = absZ; axis = 'Y'; }

    int ch;
    if (axis == 'X') {
      ch = selectMotor(mode, 'X', y > 0);
      singleActuator(ch, muxAddress);
      HMD[ch].Mode(0x05);
      HMD[ch].RTP(127);
      deactivateOtherMotors(ch, muxAddress);  // ✅ NEW
    } else if (axis == 'Y') {
      ch = selectMotor(mode, 'Y', z > 0);
      singleActuator(ch, muxAddress);      
      HMD[ch].Mode(0x05);
      HMD[ch].RTP(127);
      deactivateOtherMotors(ch, muxAddress);  // ✅ NEW

    } else if (axis == 'Z') {
      handleZfeedback(mode, x);
    }
  } else {
    stopAllMotors();
  }
}


void handleZfeedback(char mode, float x) {
  if (mode == '1' || mode == '2'){
    disableotherchannel(MUX_Z);
    muxAddress = MUX_X;

  }
  if (mode == '3' || mode == '4'){
    disableotherchannel(MUX_X);
    muxAddress = MUX_Z;

  }

  int* seqZ; 
  
    // Select sequence based on incoming mode
  if (mode == '1') seqZ = wristZ;
  else if (mode == '2') seqZ = lfz;
  else if (mode == '3') seqZ = ufz;
  else if (mode == '4') seqZ = uaz;
  else return;  // invalid mode
  if (x > 0) {
    if (stepIndexZ < 4 && millis() - lastActivationTime >= channelInterval) {
      int ch = seqZ[stepIndexZ];
      singleActuator(ch, muxAddress);
      HMD[ch].Mode(0x05);
      HMD[ch].RTP(127);
      activationTimesZ[ch] = millis();
      deactivatedZ[ch] = false;
      Serial.print("▶️ Z+ activated: "); Serial.println(ch);
      stepIndexZ ++;
      lastActivationTime = millis();
      if (stepIndexZ >= 4) stepIndexZ = 0;  // Optional: reset to loop
      lastActivationTime = millis();
      delay(10);
    }

    for (int i = 0; i < 4; i++) {
      int ch = seqZ[i];
      if (!deactivatedZ[ch] && millis() - activationTimesZ[ch] >= channelDuration) {
        singleActuator(ch, muxAddress);
        HMD[ch].RTP(0);
        HMD[ch].stop();
        deactivatedZ[ch] = true;
        Serial.print("🛑 Z+ deactivated: "); Serial.println(ch);
      }
    }
  } else {
    if (revstepIndexZ > -1 && millis() - lastActivationTime >= channelInterval) {
      int ch = seqZ[revstepIndexZ];
      singleActuator(ch, muxAddress);
      HMD[ch].Mode(0x05);
      HMD[ch].RTP(127);
      activationTimesZ[ch] = millis();
      deactivatedZ[ch] = false;
      Serial.print("▶️ Z- activated: "); Serial.println(ch);
      revstepIndexZ --;
      if (revstepIndexZ < 0) revstepIndexZ = 3;  // Optional: reset to loop

      lastActivationTime = millis();
      delay(10);
    }

    for (int i = 3; i >= 0; i--) {
      int ch = seqZ[i];
      if (!deactivatedZ[ch] && millis() - activationTimesZ[ch] >= channelDuration) {
        singleActuator(ch, muxAddress);
        HMD[ch].RTP(0);
        HMD[ch].stop();
        deactivatedZ[ch] = true;
        Serial.print("🛑 Z- deactivated: "); Serial.println(ch);
      }
    }
  }
}
// === Mapping Motor Selection ===

int selectMotor(char mode, char axis, bool positive) {
  if (mode == '1') {
    if (axis == 'Y') return positive ? 2 : 4;
    if (axis == 'Z') return positive ? 3 : 6;
  } else if (mode == '2') {
    if (axis == 'Y') return positive ? 0 : 5;
    if (axis == 'Z') return positive ? 1 : 7;
  } else if (mode == '3') {
    if (axis == 'Y') return positive ? 0 : 6;
    if (axis == 'Z') return positive ? 3 : 7;
  } else if (mode == '4') {
    if (axis == 'Y') return positive ? 1 : 4;
    if (axis == 'Z') return positive ? 2 : 5;
  }
  return 0; // default
}