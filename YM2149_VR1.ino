/******************************
 * YM2149 Simple Hardware Driver V.R1
 * Wil Lindsay
 * Original Version Created August 8, 2010
 * Released GPL-3.0 2014
 *
 *  Created by Wil Lindsay   wil@straytechnologies.com
 *  Please see bottom of file for credits, copying and copyright information
 *
 *  This software is to drive the YM2149 for 2 channel audio output.
 *  The primary goal, is to have useful audio-out only function with as little parts count/ build cost as possible.
 *  MIDI parsing is inline / no external libraries needed for this program
 *
 * V.1 is using YM, 2Mhz XTAL, Arduino & 10 resistors.
 *
 * V.2 filter out <3byte messages,
 *     speed fixes in parsing
 *     add sensing for usb vs. true midi
 *     31250 midi speed accesable via "stop all" button on startup
 *
 * V.3  fix pitchbend on CH2,3
 *
 * V.R1 License info added for public release 27-Feb-2014
 *     More inline comments added for ease of use

 *******************************
 *
 * Pinout between ICs:
 *
 * YM2149  Arduino
 * D0        D8            Register Address
 * D1        D9
 * D2        D10
 * D3        D11
 *
 * D4        D2            Upper Address
 * D5        D3
 * D6        D4
 * D7        D5
 * A8        D6
 * A9        D7
 *
 * BC1       D12          Write/Address Mode Select
 *
 *
 *  serial rate is 38400 (selectable 31250 norm on startup ; this can be changed in the setup function below)
 *  midi CH 1 : SQA (mono)
 *  midi CH 2 : SQB (left)
 *  midi CH 3 : SQC (right)
 *  midi CH 4 : NOISE (L/R/Both)
 */

/*lookup table for notes; these could be changed for alterate tunings */
const byte lowLUT[105] = {
  0xD1,0xEE,0x17,0x4D,0x8E,0xD9,0x2F,0x8E,0xF7,0x67,0xE0,0x61,0xE8,0x77,0x0B,0xA6,0x47,0xEC,0x97,0x47,0xFB,0xB3,0x70,0x30,0xF4,0xBB,0x85,0x53,0x23,0xF6,0xCB,0xA3,0x7D,0x59,0x38,0x18,0xFA,0xDD,0xC2,0xA9,0x91,0x7B,0x65,0x51,0x3E,0x2C,0x1C,0x0C,0xFD,0xEE,0xE1,0xD4,0xC8,0xBD,0xB2,0xA8,0x9F,0x96,0x8E,0x86,0x7E,0x77,0x70,0x6A,0x64,0x5E,0x59,0x54,0x4F,0x4B,0x47,0x43,0x3F,0x3B,0x38,0x35,0x32,0x2F,0x2C,0x2A,0x27,0x25,0x23,0x21,0x1F,0x1D,0x1C,0x1A,0x19,0x17,0x16,0x15,0x13,0x12,0x11,0x10,0x0F,0x0E,0x0E,0x0D,0x0C,0x0B,0x0B,0x0A,0x09};
const byte uppLUT[105] = {
  0x0F,0x0E,0x0E,0x0D,0x0C,0x0B,0x0B,0x0A,0x09,0x09,0x08,0x08,0x07,0x07,0x07,0x06,0x06,0x05,0x05,0x05,0x04,0x04,0x04,0x04,0x03,0x03,0x03,0x03,0x03,0x02,0x02,0x02,0x02,0x02,0x02,0x02,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};

byte statusByte, dataByte1, dataByte2;
byte upperTReg, lowerTReg;
byte mixerVal;
int button=14;
/* channel use flags */
boolean SAFlag = 0;
boolean SBFlag = 0;
boolean SCFlag = 0;
boolean NAFlag = 0;
boolean NBFlag = 0;
boolean NCFlag = 0;
boolean NDFlag = 0;

byte AVolStored, BVolStored, CVolStored;
byte lastOn90, lastOn91, lastOn92, lastOn99;
int bendVal,pitchVal,newPitch;
boolean AEnvFlag, BEnvFlag, CEnvFlag;
byte envShape = 1; //inits envelope
byte noiseChannel = 0;

/*  REGISTERS */
static byte volRegA = 0x08; //mono
static byte volRegB = 0x09; //right
static byte volRegC = 0x0A;  //left
/* Mixer Channel Toggles */
static byte mixerReg = 0x07;
static byte toggleSQA = B00000001;  //mono
static byte onSQA = B11111110;
static byte toggleSQB = B00000010;  //right
static byte onSQB = B11111101;
static byte toggleSQC = B00000100;  //left
static byte onSQC = B11111011;
static byte toggleNA = B00001000;   //mono  A
static byte onNA = B11110111;
static byte toggleNB = B00010000;   //right B
static byte toggleNC = B00100000;   //left  C
static byte toggleND = B00110000;   //Left & Right B & C

void setup(){
  DDRD = DDRD | B11111100;  //PINS D2-D7 OUTPUT
  DDRB = DDRB | B00111111;  //PINS D8-D13 OUTPUT
  DDRC = DDRC | B00000000;  //Analog pins as inputs _includes button

  if(PINC & B00000001 == B00000001){ //midi module attached if button is pressed on startup
      Serial.begin(31250);//(31250) vs (38400);  //true MIDI speed for 5-pin DIN

}else{
      Serial.begin(38400);//defaults to FTDI speed
  }
  initYM();

  mixerVal = B10111111;  // running mixer byte lower 6 bits:
  // NoiseC NoiseB NoiseA SQC SQB SQA
  // starts w/ all channels off
  allStop();
  writeReg(0xF,1); //blink LED on
}

void loop(){
  if(Serial.available() > 0) {
    statusByte = Serial.read();
    if (((statusByte>>4)== 0x08)||((statusByte>>4)== 0x09)||((statusByte>>4)== 0x0B)||((statusByte>>4)== 0x0E)){ //******
      while(Serial.available() < 1){
        //wait for dataByte1 to arrive on Serial;
      }
      dataByte1 = Serial.read();
      while(Serial.available() < 1){
        //wait for dataByte2 to arrive;
      }
      dataByte2 = Serial.read();
    }//endif filter useful status
    switch (statusByte){
    case 0x80:      // noteOff CH1
      if (lastOn90 == dataByte1){
        mixerVal =  mixerVal | toggleSQA;
        writeReg(mixerReg, mixerVal);
        writeReg(volRegA, B00000000 ); //vol 0 and turn off envelope
        if (NAFlag ){
          writeReg(volRegA,AVolStored); // return vol to noise channel
        }
        SAFlag = 0;
      }
      break;
    case 0x81:      // noteOff CH3 LEFT
      if (lastOn91 == dataByte1){
        mixerVal =  mixerVal | toggleSQC;
        writeReg(mixerReg, mixerVal);
        writeReg(volRegC,B00000000); //vol 0
        if (NCFlag){ //if noise on...
          writeReg(volRegC,CVolStored); // return vol to noise level
        }
        SCFlag = 0;
      }
      break;
    case 0x82:      // noteOff CH2  RIGHT
      if (lastOn92 == dataByte1){
        mixerVal =  mixerVal | toggleSQB;
        writeReg(mixerReg, mixerVal);
        writeReg(volRegB,B00000000); //vol 0
        if (NBFlag){ //if noise on...
          writeReg(volRegB,BVolStored); // return vol to noise level
        }
        SBFlag = 0;
      }
      break;
    case 0x89:     //CH 10 noise noteOff
      if (lastOn99 == dataByte1){
        switch (noiseChannel){
        case 0:
          mixerVal =  mixerVal | toggleNA;
          NAFlag = 0;
          break;
        case 1:
          mixerVal =  mixerVal | toggleNC;
          NCFlag = 0;
          break;
        case 2:
          mixerVal =  mixerVal | toggleNB;
          NBFlag = 0;
          break;
        case 3:
          mixerVal =  mixerVal | toggleND;
          NDFlag = 0;
          break;
        default:
          break;
        }
        writeReg(mixerReg, mixerVal);
        writeReg(volRegA,B00000000); //vol 0
        if (SAFlag){ //if Squarewave is on...
          writeReg(volRegA,AVolStored); // return vol to SQWave level
        }
        if (SCFlag){ //if Squarewave is on...
          writeReg(volRegC,CVolStored); // return vol to SQWave level
        }
        if (SBFlag){ //if Squarewave is on...
          writeReg(volRegB,BVolStored); // return vol to SQWave level
        }
      }
      break;
    case 0x90:      //noteOn   CH1
      if (dataByte2 == 0){ //volocity zero == noteOff
        if (lastOn90 == dataByte1){
          mixerVal =  mixerVal | toggleSQA;
          writeReg(mixerReg, mixerVal);
          writeReg(volRegA, B00000000 ); //vol 0 and turn off envelope
          if (NAFlag ){
            writeReg(volRegA,AVolStored); // return vol to noise channel
          }
          SAFlag = 0;
        }
      }
      else {
        if(dataByte1 >23){// no note below midi 23
          lastOn90 = dataByte1;
          upperTReg = uppLUT[dataByte1-23];
          lowerTReg = lowLUT[dataByte1-23];
          writeReg(0x1,upperTReg);    //tone
          writeReg(0x0,lowerTReg);
          if(!NAFlag){   // If Noise is on
            AVolStored = dataByte2 >>3; //store noise level
          }
          if (AEnvFlag){ //if envelope is on
            writeReg(volRegA, B00010000 );//turn envelope mode ON
            writeReg (0x0D,envShape);// trigger envelope
          }
          else {
            writeReg(volRegA,dataByte2 >>3);  //velocity 0-15 only as volume
          }
          mixerVal =  mixerVal & onSQA;//^ toggleSQA;//toggle on
          writeReg(mixerReg, mixerVal);
          SAFlag = 1;
        }
      }
      break;
    case 0x91:      //noteOn   CH3 LEFT
      if (dataByte2 == 0){
        if (lastOn91 == dataByte1){
          mixerVal =  mixerVal | toggleSQC;
          writeReg(mixerReg, mixerVal);
          writeReg(volRegC,B00000000); //vol 0
          if (NCFlag){ //if noise on...
            writeReg(volRegC,CVolStored); // return vol to noise level
          }
          SCFlag = 0;
        }
      }
      else
      {
        if(dataByte1 >23){// no note below midi 23
          lastOn91 = dataByte1;
          upperTReg = uppLUT[dataByte1-23];
          lowerTReg = lowLUT[dataByte1-23];
          writeReg(0x5,upperTReg);    //tone
          writeReg(0x4,lowerTReg);
          if(!NCFlag){   // If Noise is on
            CVolStored = dataByte2 >>3; //store noise level
          }
          if (CEnvFlag){ //if envelope is on
            writeReg(volRegC, B00010000 );//turn envelope mode ON
            writeReg (0x0D,envShape);// trigger envelope
          }
          else {
            writeReg(volRegC,dataByte2 >>3);  //velocity 0-15 only as volume
          }
          mixerVal =  mixerVal & onSQC;
          writeReg(mixerReg, mixerVal);
          SCFlag = 1;
        }
      }
      break;
    case 0x92:      //noteOn   CH2 RIGHT
      if (dataByte2 == 0){  //velocit zero = noteOff
        if (lastOn92 == dataByte1){
          mixerVal =  mixerVal | toggleSQB;
          writeReg(mixerReg, mixerVal);
          writeReg(volRegB,B00000000); //vol 0
          if (NBFlag){ //if noise on...
            writeReg(volRegB,BVolStored); // return vol to noise level
          }
          SBFlag = 0;
        }
      }
      else
      {
        if(dataByte1 >23){// no note below midi 23
          lastOn92 = dataByte1;
          upperTReg = uppLUT[dataByte1-23];
          lowerTReg = lowLUT[dataByte1-23];
          writeReg(0x3,upperTReg);    //tone
          writeReg(0x2,lowerTReg);
          if(!NBFlag){   // If Noise is on
            BVolStored = dataByte2 >>3; //store noise level
          }
          if (BEnvFlag){ //if envelope is on
            writeReg(volRegB, B00010000 );//turn envelope mode ON
            writeReg (0x0D,envShape);
          }
          else {
            writeReg(volRegB,dataByte2 >>3);  //velocity 0-15 only as volume
          }
          mixerVal =  mixerVal & onSQB;
          writeReg(mixerReg, mixerVal);
          SBFlag = 1;
        }
      }
      break;
    case 0x99:
      if (dataByte2 == 0){ // velocit zero = noteOff
        if (lastOn99 == dataByte1){
          switch (noiseChannel){
          case 0:
            mixerVal =  mixerVal | toggleNA;
            NAFlag = 0;
            break;
          case 1:
            mixerVal =  mixerVal | toggleNC;
            NCFlag = 0;
            break;
          case 2:
            mixerVal =  mixerVal | toggleNB;
            NBFlag = 0;
            break;
          case 3:
            mixerVal =  mixerVal | toggleND;
            NDFlag = 0;
            break;
          default:
            break;
          }
          writeReg(mixerReg, mixerVal);
          writeReg(volRegA,B00000000); //vol 0
          if (SAFlag){ //if Squarewave is on...
            writeReg(volRegA,AVolStored); // return vol to SQWave level
          }
          if (SCFlag){ //if Squarewave is on...
            writeReg(volRegC,CVolStored); // return vol to SQWave level
          }
          if (SBFlag){ //if Squarewave is on...
            writeReg(volRegB,BVolStored); // return vol to SQWave level
          }
        }
      }
      else
      {
        lastOn99 = dataByte1;
        writeReg(0x6,31-(dataByte1/4));
        if(!SAFlag){   // If Squarewave is on
          AVolStored = dataByte2 >>3; //store SQ level
        }
        if(!SBFlag){   // If Squarewave is on
          BVolStored = dataByte2 >>3; //store SQ level
        }
        if(!SCFlag){   // If Squarewave is on
          CVolStored = dataByte2 >>3; //store SQ level
        }
        switch (noiseChannel){
        case 0: //on CH A only
          if (AEnvFlag){ //if envelope is on
            writeReg(volRegA, B00010000 );//turn envelope mode ON
            writeReg (0x0D,envShape);
          }
          else {
            writeReg(volRegA,dataByte2 >>3);  //velocity 0-15 only as volume
          }
          if (!NAFlag){
            mixerVal =  mixerVal ^ toggleNA;
            writeReg(mixerReg, mixerVal);
            NAFlag = 1;
          }
          break;
        case 1: //on CH C only
          if (CEnvFlag){ //if envelope is on
            writeReg(volRegC, B00010000 );//turn envelope mode ON
            writeReg (0x0D,envShape);
          }
          else {
            writeReg(volRegC,dataByte2 >>3);  //velocity 0-15 only as volume
          }
          if (!NCFlag){
            mixerVal =  mixerVal ^ toggleNC;
            writeReg(mixerReg, mixerVal);
            NCFlag = 1;
          }
          break;
        case 2: //on CH B only
          if (BEnvFlag){ //if envelope is on
            writeReg(volRegB, B00010000 );//turn envelope mode ON
            writeReg (0x0D,envShape);
          }
          else {
            writeReg(volRegB,dataByte2 >>3);  //velocity 0-15 only as volume
          }
          if (!NBFlag){
            mixerVal =  mixerVal ^ toggleNB;
            writeReg(mixerReg, mixerVal);
            NBFlag = 1;
          }
          break;
        case 3: //on CH A & B only
          if (CEnvFlag){ //if envelope is on
            writeReg(volRegC, B00010000 );//turn envelope mode ON
            writeReg (0x0D,envShape);
          }
          else {
            writeReg(volRegC,dataByte2 >>3);  //velocity 0-15 only as volume
          }
          if (BEnvFlag){ //if envelope is on
            writeReg(volRegB, B00010000 );//turn envelope mode ON
            writeReg (0x0D,envShape);
          }
          else {
            writeReg(volRegB,dataByte2 >>3);  //velocity 0-15 only as volume
          }
          if (!NDFlag){
            mixerVal =  mixerVal ^ toggleND;
            writeReg(mixerReg, mixerVal);
            NDFlag = 1;
          }
          break;
        default:
          break;
        }
      }
      break;
    case 0xB0:     //controller change (CC)
    case 0xB1:
    case 0xB2:    //on any active channel are the same
    case 0xB9:
      switch (dataByte1){
      case 0x16:   //CC22 Envelope ON/OFF CH1
        if (dataByte2 > 0x3F){//ON
          AEnvFlag = 1;
        }
        else {
          AEnvFlag = 0;
        }
        break;
      case 0x17:   //CC23 Envelope ON/OFF CH2
        if (dataByte2 > 0x3F){//ON
          CEnvFlag = 1;
        }
        else {
          CEnvFlag = 0;
        }
        break;
      case 0x18:   //CC24 Envelope ON/OFF  CH3
        if (dataByte2 > 0x3F){//ON
          BEnvFlag = 1;
        }
        else {
          BEnvFlag = 0;
        }
        break;
      case 0x19:   //CC25 Rough envelope frequency
        writeReg(0x0C,dataByte2*2);
        break;
      case 0x1A:   //CC26 Fine envelope Frequency
        writeReg(0x0B,dataByte2*2);
        break;
      case 0x1B:    //CC27 Envelope Shape
        envShape = dataByte2>>3;
        writeReg(0x0D,envShape);
        break;
      case 0x1C:    //CC28 Noise Channel select
        mixerVal =  mixerVal |toggleNA | toggleND;
        writeReg(mixerReg, mixerVal);
        noiseChannel = (dataByte2)>>5; // 0=A,1=C,2=B,3=A&B
        break;
      default:
        break;
      }
      break;
    case 0xE0:      //Pitch Bend Ch1 CENTER
        bendVal = 8192 - ((dataByte2<<7) + dataByte1) ;//get bend val
        if (bendVal == 0){
          upperTReg = uppLUT[lastOn90-23];
          lowerTReg = lowLUT[lastOn90-23];
        }
        else{
          pitchVal = (uppLUT[lastOn90-23]<<8)+lowLUT[lastOn90-23];//get current pitch
          if (bendVal < 0){
            newPitch = map(bendVal,-8191,-1,0,pitchVal);
          }
          if (bendVal > 0){
            newPitch = map(bendVal,1,8192,pitchVal,4095);
          }
          upperTReg = newPitch >> 8;
          lowerTReg = newPitch & 0xFF;
        }
      writeReg(0x1,upperTReg);    //tone
      writeReg(0x0,lowerTReg);
      break;
    case 0xE1:      //Pitch Bend Ch3 LEFT
      bendVal = 8192 - ((dataByte2<<7) + dataByte1) ;//get bend val
      if (bendVal == 0){
        upperTReg = uppLUT[lastOn91-23];
        lowerTReg = lowLUT[lastOn91-23];
      }
      else{
          pitchVal = (uppLUT[lastOn91-23]<<8)+lowLUT[lastOn91-23];//get current pitch
          if (bendVal < 0){
            newPitch = map(bendVal,-8191,-1,0,pitchVal);
          }
          if (bendVal > 0){
            newPitch = map(bendVal,1,8192,pitchVal,4095);
          }
          upperTReg = newPitch >> 8;
          lowerTReg = newPitch & 0xFF;
      }
      writeReg(0x5,upperTReg);    //tone
      writeReg(0x4,lowerTReg);
      break;
    case 0xE2:      //Pitch Bend Ch2 RIGHT
      bendVal = 8192 - ((dataByte2<<7) + dataByte1) ;//get bend val
      if (bendVal == 0){
        upperTReg = uppLUT[lastOn92-23];
        lowerTReg = lowLUT[lastOn92-23];
      }
      else{
          pitchVal = (uppLUT[lastOn92-23]<<8)+lowLUT[lastOn92-23];//get current pitch
          if (bendVal < 0){
            newPitch = map(bendVal,-8191,-1,0,pitchVal);
          }
          if (bendVal > 0){
            newPitch = map(bendVal,1,8192,pitchVal,4095);
          }
          upperTReg = newPitch >> 8;
          lowerTReg = newPitch & 0xFF;
      }
      writeReg(0x3,upperTReg);    //tone
      writeReg(0x2,lowerTReg);
      break;
    case 0xE9:
      bendVal = 8192 - ((dataByte2<<7) + dataByte1) ;//get bend val
      if (bendVal == 0){
        newPitch = 31-(lastOn99/4);
      }
      else{
        pitchVal = 31-(lastOn99/4);//get current pitch
        newPitch = pitchVal+(bendVal/512);//current - (0 to 1024)
        if (newPitch > 31){
          newPitch = 31;
        }
        if (newPitch < 0){
          newPitch = 0;
        }
      }
      writeReg(0x6,newPitch);    //bent noise
      break;
    default:
      break;
    }
  }
  if(PINC & B00000001 == B00000001){ //kill all button
    allStop();
  }
}

void BlinkLed(byte num){
  for (byte i=0;i<num;i++) {
    writeReg(0xF,1);  //LED_ON
    delay(150);
    writeReg(0xF,0); //LED_OFF
    delay(150);
  }
}

void writeReg(byte w_reg,byte w_val){
  /*
order of events:
   set lower reg with w_reg
   set upper reg to addr mode
   set BC1 to ADDRESS MODE
   set BDIR to ACTIVE
   delay
   setBDIR to INACTIVE
   set w_val high bytes
   set w_Val  low bytes
   toggle BC1 to write mode
   set BDIR to ACTIVE
   delay
   setBDIR to INACTIVE
   */
  byte valHIGH = 0;
  byte valLOW = 0;
  valHIGH = w_val >> 4; //shift to only HIGH 4 bits
  valHIGH = valHIGH << 2; //shift back past TX & RX
  valLOW = w_val & B00001111; //zero out upper 4 bits
  PORTB = B000000;  //BDIR & BC1 = INACTIVE, lower reg clear
  PORTB = w_reg;    //write ADDR to lower register
  PORTD = B01000000;     // write upper register to ADDR MODE
  PORTB = PORTB ^ B00110000;  //toggle BC1 to ADDR MODE& BDIR to ACTIVE
  //delay(10);
  PORTB = B000000; //BC1 & BDIR to INACTIVE (BC1 to WRITE MODE), lower clear
  PORTD = valHIGH;   //set w_val high bytes
  PORTB = valLOW;  //set w_Val  low bytes
  PORTB = PORTB ^ B00100000; //toggle BDIR to ACTIVE
  //delay(10);
  PORTB = PORTB ^ B00100000; //toggle BDIR to INACTIVE
}

void allStop(){
  mixerVal = B10111111;
  writeReg(0x07,mixerVal);
  writeReg(volRegA,B00000000); //vol 0
  writeReg(volRegB,B00000000); //vol 0
  writeReg(volRegC,B00000000); //vol 0
  Serial.flush();
}

void initYM(){
  delay(2000);
  writeReg(0x7,B11000000); //LED port on
  for(int i=0;i<3;i++){
    writeReg(0xF,1);  //LED on
    delay(250);
    writeReg(0xF,0);  //LED off
    delay(250);
  }
  writeReg(0x0C, 10 );  //Init envelope freq
  writeReg(0x0B, 10 );  //Init envelope freq
  writeReg(0x0D, 1 );   //Init envelope shape
}


/*
*    Copyright 2014 Wil Lindsay
 *
 *    This program is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 *    This file is dependent on the ardumidi library which is a part of ttymidi.
 *    As such, ardumidi is also distributed under the GNU General Public License
 *    but is not distributed as part of this package.
 *
 ****************************************************************************************
 */
