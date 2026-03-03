#include "sbus.h"
#include "config.h"

// SBUS variables
uint8_t sbusPacket[SBUS_PACKET_SIZE];
uint16_t sbusChannels[18];
bool sbusFailsafe = false;
bool sbusLostFrame = false;

// RC Input variables
float rcThrottle = 0.0;
float rcRoll = 0.0;
float rcPitch = 0.0;
float rcYaw = 0.0;
bool rcArm = false;

void initSBUS() {
    Serial2.begin(SBUS_BAUDRATE, SERIAL_8E2, SBUS_PIN, -1, true);
    Serial.print("SBUS initialized on pin "); Serial.print(SBUS_PIN);
    Serial.print(" at "); Serial.print(SBUS_BAUDRATE); Serial.println(" baud");
    Serial.println("ARM Logic: Check Serial Monitor for Raw ARM value and adjust logic.");
}

bool readSBUS() {
    static uint8_t sbusIndex = 0;
    static unsigned long lastValidPacket = 0;

    while (Serial2.available()) {
        uint8_t inByte = Serial2.read();

        if (sbusIndex == 0 && inByte != 0x0F) {
            continue;
        }

        sbusPacket[sbusIndex] = inByte;
        sbusIndex++;

        if (sbusIndex == SBUS_PACKET_SIZE) {
            sbusIndex = 0;
            packetCount++;
            
            // Check end bytes
            if (sbusPacket[24] == 0x00 || sbusPacket[24] == 0x04 || 
                sbusPacket[24] == 0x14 || sbusPacket[24] == 0x24 || 
                sbusPacket[24] == 0x34) {
                
                validPacketCount++;
                
                // Decode channels
                sbusChannels[0]  = ((sbusPacket[1]    |sbusPacket[2]<<8)                 & 0x07FF);
                sbusChannels[1]  = ((sbusPacket[2]>>3 |sbusPacket[3]<<5)                 & 0x07FF);
                sbusChannels[2]  = ((sbusPacket[3]>>6 |sbusPacket[4]<<2 |sbusPacket[5]<<10) & 0x07FF);
                sbusChannels[3]  = ((sbusPacket[5]>>1 |sbusPacket[6]<<7)                 & 0x07FF);
                sbusChannels[4]  = ((sbusPacket[6]>>4 |sbusPacket[7]<<4)                 & 0x07FF);
                sbusChannels[5]  = ((sbusPacket[7]>>7 |sbusPacket[8]<<1 |sbusPacket[9]<<9)  & 0x07FF);
                sbusChannels[6]  = ((sbusPacket[9]>>2 |sbusPacket[10]<<6)                & 0x07FF);
                sbusChannels[7]  = ((sbusPacket[10]>>5|sbusPacket[11]<<3)                & 0x07FF);
                sbusChannels[8]  = ((sbusPacket[12]   |sbusPacket[13]<<8)                & 0x07FF);
                sbusChannels[9]  = ((sbusPacket[13]>>3|sbusPacket[14]<<5)                & 0x07FF);
                sbusChannels[10] = ((sbusPacket[14]>>6|sbusPacket[15]<<2|sbusPacket[16]<<10) & 0x07FF);
                sbusChannels[11] = ((sbusPacket[16]>>1|sbusPacket[17]<<7)                & 0x07FF);
                sbusChannels[12] = ((sbusPacket[17]>>4|sbusPacket[18]<<4)                & 0x07FF);
                sbusChannels[13] = ((sbusPacket[18]>>7|sbusPacket[19]<<1|sbusPacket[20]<<9)  & 0x07FF);
                sbusChannels[14] = ((sbusPacket[20]>>2|sbusPacket[21]<<6)                & 0x07FF);
                sbusChannels[15] = ((sbusPacket[21]>>5|sbusPacket[22]<<3)                & 0x07FF);

                // Check failsafe and lost frame
                sbusFailsafe = sbusPacket[23] & 0x08;
                sbusLostFrame = sbusPacket[23] & 0x04;

                lastValidPacket = millis();
                return true;
            }
        }
    }

    // Check timeout
    if (millis() - lastValidPacket > 50) {
        if (!sbusFailsafe) {
            Serial.println("SBUS TIMEOUT - Setting failsafe!");
        }
        sbusFailsafe = true;
    }

    return false;
}

void processSBUS() {
    if (sbusFailsafe) {
        rcThrottle = 0.0;
        rcRoll = 0.0;
        rcPitch = 0.0;
        rcYaw = 0.0;
        rcArm = false;
        return;
    }

    rcThrottle = map(sbusChannels[CH_THROTTLE], 172, 1811, 0, 100) / 100.0;
    rcRoll = map(sbusChannels[CH_ROLL], 172, 1811, -30, 30);
    rcPitch = map(sbusChannels[CH_PITCH], 172, 1811, -30, 30);
    rcYaw = map(sbusChannels[CH_YAW], 172, 1811, -100, 100);
    
    int rawArmValue = sbusChannels[CH_ARM];
    rcArm = (rawArmValue > 900 && rawArmValue < 1100);
    
    // Constrain values
    rcThrottle = constrain(rcThrottle, 0.0, 1.0);
    rcRoll = constrain(rcRoll, -30.0, 30.0);
    rcPitch = constrain(rcPitch, -30.0, 30.0);
    rcYaw = constrain(rcYaw, -100.0, 100.0);

    static unsigned long lastArmDebugTime = 0;
    if (millis() - lastArmDebugTime > 500) {
        lastArmDebugTime = millis();
        Serial.print("RC Input - T:"); Serial.print(rcThrottle, 3);
        Serial.print(" R:"); Serial.print(rcRoll, 2);
        Serial.print(" P:"); Serial.print(rcPitch, 2);
        Serial.print(" Y:"); Serial.print(rcYaw, 2);
        Serial.print(" ARM:"); Serial.print(rcArm);
        Serial.print(" (Raw ARM: "); Serial.print(sbusChannels[CH_ARM]);
        Serial.println(rcArm ? " - ARMED!)" : " - DISARMED)");
    }
}