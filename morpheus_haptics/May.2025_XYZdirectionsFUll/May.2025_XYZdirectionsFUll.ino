#include <Wire.h>
#include <SparkFun_I2C_Mux_Arduino_Library.h>
#include <Sparkfun_DRV2605L.h>

SFE_HMD_DRV2605L HMD[8];
QWIICMUX mux;
QWIICMUX muxX;  // for MUX_X (0x70)
QWIICMUX muxZ;  // for MUX_Z (0x71)

#define MUX_X 0x70
#define MUX_Z 0x71

float lastX = 0; 
float lastZ = 0; 
float lastY = 0;
bool isActivated = false;
bool gotX = false;
bool gotZ = false;
bool gotY = false;
bool isXPhase = true; 
bool isZPhase = false; 
bool isYPhase = false;

int stepIndexX = 0;
int stepIndexZ = 0;
int stepIndexY = 0;

unsigned long lastActivationTime = 0;
unsigned long activationTimesX[4] = {0};
unsigned long activationTimesZ[4] = {0};
unsigned long activationTimesY[8] = {0};

bool deactivatedX[4] = {true, true, true, true};
bool deactivatedZ[4] = {true, true, true, true};
bool deactivatedY[8] = {true, true, true, true, true, true, true, true};

unsigned long channelInterval = 30;
unsigned long channelDuration = 50;

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


void setupMotor(uint8_t i, uint8_t muxAddress) {
  selectChannels(i, muxAddress);
  HMD[i].begin();
  HMD[i].writeDRV2605L(OVERDRIVECLAMP_REG, 0xFF);
  HMD[i].Mode(0x05);
  HMD[i].MotorSelect(0x86);
  HMD[i].Library(6);
}
void singleActuator(uint8_t i, uint8_t muxAddr) {
  if (muxAddr == MUX_X) {
    muxX.setPort(i);  // activates channel i on mux 0x70
  } else if (muxAddr == MUX_Z) {
    muxZ.setPort(i);  // activates channel i on mux 0x71
  } else {
    Serial.println("Invalid mux address");
  }
}

void stopAllMotors() {
  for (int i = 0; i < 8; i++) {
    singleActuator(i, MUX_X);
    HMD[i].begin();  // ✅ Reinitialize for MUX_X
    HMD[i].stop();
    HMD[i].Mode(0x00);

    singleActuator(i, MUX_Z);
    HMD[i].begin();  // ✅ Reinitialize for MUX_Z
    HMD[i].stop();
    HMD[i].Mode(0x00);
  }

  disableChannels(MUX_X);
  disableChannels(MUX_Z);
}
void selectChannels(uint8_t i, uint8_t muxAddress) {
  Wire.beginTransmission(muxAddress);
  Wire.write(1 << i);
  Wire.endTransmission();
}

void disableChannels(uint8_t muxAddress) {
  Wire.beginTransmission(muxAddress);
  Wire.write(0);
  Wire.endTransmission();
}

void simulActuate(uint8_t i, uint8_t j, uint8_t muxAddress, int strength) {
  // Set both channels on the correct mux
  Wire.beginTransmission(muxAddress);
  Wire.write((1 << i) | (1 << j));
  Wire.endTransmission();

  // Reinitialize and set active mode
  // HMD[i].begin();           // ✅ Necessary for current mux channel
  HMD[i].Mode(0x05);        // Active mode
  HMD[i].RTP(strength);

  // HMD[j].begin();           // ✅ Necessary for current mux channel
  HMD[j].Mode(0x05);
  HMD[j].RTP(strength);
}

void simulDeactivate(uint8_t i, uint8_t j, uint8_t muxAddress) {
  Wire.beginTransmission(muxAddress);
  Wire.write((1 << i) | (1 << j));
  Wire.endTransmission();

  // HMD[i].begin();           // ✅ Necessary
  HMD[i].RTP(0);
  HMD[i].stop();

  // HMD[j].begin();           // ✅ Necessary
  HMD[j].RTP(0);
  HMD[j].stop();

  disableChannels(muxAddress);
}
bool allDeactivated(bool* flags, int count) {
  for (int i = 0; i < count; i++) {
    if (!flags[i]) return false;
  }
  return true;
}

void loop() {
  unsigned long now = millis();

  // Handle serial input
  if (!isActivated && Serial.available()) {
    char incoming = Serial.read();

    if (incoming == '1') {
      // Serial.println("🛑 Stopping all actuators");
      for (int i = 0; i < 8; i++) {
        selectChannels(i, MUX_X); HMD[i].RTP(0); HMD[i].stop();
        selectChannels(i, MUX_Z); HMD[i].RTP(0); HMD[i].stop();
      }
      disableChannels(MUX_X);
      disableChannels(MUX_Z);
      isActivated = false;
      lastX = lastZ = lastY = 0;
    }

    else if (incoming == '0') {
      gotX = gotZ = gotY = false;
      lastX = lastZ = lastY = 0;

      unsigned long startWait = millis();
      while (!(gotX && gotZ && gotY) && (millis() - startWait < 1000)) {
        if (Serial.available()) {
          String rawLine = Serial.readStringUntil('\n');
          rawLine.trim();
          if(rawLine.startsWith("START") && rawLine.endsWith("END")) {
              // Remove header and terminator then parse the inner content
              rawLine.remove(0, 6);  // Remove "START " (assumes a space after START)
              rawLine.remove(rawLine.length()-4, 4);  // Remove " END"
              // Now parse for tokens "X:", "Y:", "Z:" as before
          } else {
              Serial.println("Invalid message format");
          }
          // if (line.startsWith("X:")) {
          //   lastX = line.substring(2).toFloat(); gotX = true;
          //   Serial.print("✅ Got X: "); Serial.println(lastX);
          // } else if (line.startsWith("Z:")) {
          //   lastZ = line.substring(2).toFloat(); gotZ = true;
          //   Serial.print("✅ Got Z: "); Serial.println(lastZ);
          // } else if (line.startsWith("Y:")) {
          //   lastY = line.substring(2).toFloat(); gotY = true;
          //   Serial.print("✅ Got Y: "); Serial.println(lastY);
          // }
          int xIndex = rawLine.indexOf("X:");
          int yIndex = rawLine.indexOf("Y:");
          int zIndex = rawLine.indexOf("Z:");
          if (xIndex != -1 && yIndex != -1 && zIndex != -1) {
              lastX = rawLine.substring(xIndex + 2, yIndex).toFloat();
              lastY = rawLine.substring(yIndex + 2, zIndex).toFloat();
              lastZ = rawLine.substring(zIndex + 2).toFloat();

              gotX = gotY = gotZ = true; // Ensure all data is properly received

              Serial.print("✅ Parsed X: "); Serial.println(lastX);
              Serial.print("✅ Parsed Y: "); Serial.println(lastY);
              Serial.print("✅ Parsed Z: "); Serial.println(lastZ);
          } else {
              Serial.println("❌ Error parsing data! Check formatting.");}
        }
      }

      if (abs(lastX) > 0.1 || abs(lastZ) > 0.1 || abs(lastY) > 0.1) {
        isActivated = true;
        isXPhase = (abs(lastX) > 0.1);
        isYPhase = !isXPhase && (abs(lastY) > 0.1);
        isZPhase = !isXPhase && !isYPhase && (abs(lastZ) > 0.1);
        stepIndexX = stepIndexZ = stepIndexY = 0;
        for (int i = 0; i < 4; i++) deactivatedX[i] = deactivatedZ[i] = true;
        for (int i = 0; i < 8; i++) deactivatedY[i] = true;
        lastActivationTime = millis();
        // Serial.println("🚀 Starting feedback sequence");
      } else {
        // Serial.println("❌ No valid input, not activating");
      }
    }
  }

  // Activation logic
  if (isActivated && millis() - lastActivationTime >= channelInterval) {
    if (isXPhase && stepIndexX < 4 && abs(lastX) > 0.1) {
      int i = (lastX > 0) ? stepIndexX : 3 - stepIndexX;
      simulActuate(i, 7 - i, MUX_X, 127);
      activationTimesX[i] = millis();
      deactivatedX[i] = false;
      stepIndexX++;
      lastActivationTime = millis();
      // delay(10);
    }

    else if (isYPhase && stepIndexY < 4) {
      int motorIndex = (lastY > 0) ? stepIndexY : 7 - stepIndexY;
      selectChannels(motorIndex, MUX_Z);
      HMD[motorIndex].RTP(127);
      activationTimesY[motorIndex] = millis();
      deactivatedY[motorIndex] = false;
      stepIndexY++;
      lastActivationTime = millis();
      // delay(10);
    }

    else if (isZPhase && stepIndexZ < 4 && abs(lastZ) > 0.1) {
      int i = (lastZ > 0) ? stepIndexZ : 3 - stepIndexZ;
      simulActuate(i, 7 - i, MUX_Z, 127);
      activationTimesZ[i] = millis();
      deactivatedZ[i] = false;
      stepIndexZ++;
      lastActivationTime = millis();
      // delay(10);
    }
  }

  // Deactivation logic
  for (int i = 0; i < 4; i++) {
    if (isXPhase && !deactivatedX[i] && millis() - activationTimesX[i] >= channelDuration) {
      simulDeactivate(i, 7 - i, MUX_X);
      deactivatedX[i] = true;
    }

    if (isZPhase && !deactivatedZ[i] && millis() - activationTimesZ[i] >= channelDuration) {
      simulDeactivate(i, 7 - i, MUX_Z);
      deactivatedZ[i] = true;
    }
  }

  for (int i = 0; i < 8; i++) {
    if (isYPhase && !deactivatedY[i] && millis() - activationTimesY[i] >= channelDuration) {
      selectChannels(i, MUX_Z);
      HMD[i].RTP(0);
      HMD[i].stop();
      deactivatedY[i] = true;
      // Serial.print("🛑 Deactivated Y motor "); Serial.println(i);
    }
  }

  // Phase transitions
  if (isXPhase && stepIndexX >= 4 && allDeactivated(deactivatedX, 4)) {
    if (abs(lastY) > 0.1) {
      isXPhase = false; isYPhase = true;
      stepIndexY = 0; lastActivationTime = millis();
      // Serial.println("🔁 Switching to Y phase");
    } else if (abs(lastZ) > 0.1) {
      isXPhase = false; isZPhase = true;
      stepIndexZ = 0; lastActivationTime = millis();
      // Serial.println("🔁 Switching to Z phase");
    } else {
      isActivated = false;
      // Serial.println("✅ X phase done. No Y or Z feedback required.");
    }
  }

  if (isYPhase && stepIndexY >= 4 && allDeactivated(deactivatedY, 8)) {
    disableChannels(MUX_Z); // Now safe to disable
    if (abs(lastZ) > 0.1) {
      isYPhase = false; isZPhase = true;
      stepIndexZ = 0; lastActivationTime = millis();
      // Serial.println("🔁 Switching to Z phase");
    } else {
      isActivated = false;
      // Serial.println("✅ Y phase done. No Z feedback required.");
    }
  }

  if (isZPhase && stepIndexZ >= 4 && allDeactivated(deactivatedZ, 4)) {
    isActivated = false;
    disableChannels(MUX_Z);
    // Serial.println("✅ Z phase done. Waiting for next command.");
  }
}
