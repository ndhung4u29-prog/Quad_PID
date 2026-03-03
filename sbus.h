#ifndef SBUS_H
#define SBUS_H

#include "config.h"

// SBUS variables
extern uint8_t sbusPacket[];
extern uint16_t sbusChannels[];
extern bool sbusFailsafe;
extern bool sbusLostFrame;

// Function declarations
void initSBUS();
bool readSBUS();
void processSBUS();

#endif