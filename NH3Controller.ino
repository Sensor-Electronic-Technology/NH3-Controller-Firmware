//#include <Ethernet.h>
#include <EEPROM.h>
#include <Ethernet2.h>
#include <Q2HX711.h>
#include <Mudbus.h>
#include <LiquidCrystal.h>
#include "Helpers.h"

#define tankStart 70
#define tankStop  77

#define HARDCODE 0
#define DEBUG 0

#define WARNING 150
#define ALARM   100

#define scaleDT1 2
#define scaleSCK1 3
#define scaleDT2 4
#define scaleSCK2 5
#define scaleDT3 6
#define scaleSCK3 7
#define scaleDT4 8
#define scaleSCK4 9

#define resetTime 604800000
#define readTime 100
#define writeTime 500

#define outputPeriod 3000
#define runTime 50


volatile boolean firstRun = true;
volatile int heaterDuty1 = 100;
volatile float Duty1 = 100.0;
volatile float pTerm1 = 0.0;
volatile float err1 = 0.0;
volatile float iTerm1 = 0.0;
volatile float dTerm1 = 0.0;
volatile float lastErr1 = 0.0;
volatile int heaterDuty2 = 100;
volatile float Duty2 = 100.0;
volatile float pTerm2 = 0.0;
volatile float err2 = 0.0;
volatile float iTerm2 = 0.0;
volatile float dTerm2 = 0.0;
volatile float lastErr2 = 0.0;
volatile int heaterDuty3 = 100;
volatile float Duty3 = 100.0;
volatile float pTerm3 = 0.0;
volatile float err3 = 0.0;
volatile float iTerm3 = 0.0;
volatile float dTerm3 = 0.0;
volatile float lastErr3 = 0.0;
volatile int heaterDuty4 = 100;
volatile float Duty4 = 100.0;
volatile float pTerm4 = 0.0;
volatile float err4 = 0.0;
volatile float iTerm4 = 0.0;
volatile float dTerm4 = 0.0;
volatile float lastErr4 = 0.0;
volatile float heatfWeight = 0.005;
volatile float tankTemp1 = tempSP;
volatile float tankTemp2 = tempSP;
volatile float tankTemp3 = tempSP;
volatile float tankTemp4 = tempSP;
volatile float lineTemp1 = tempSP;
volatile float lineTemp2 = tempSP;
volatile int aValue = 1024;
volatile unsigned long heatReadTime = 0;
volatile unsigned long lastRunTime = 50;
volatile unsigned long lastOutputTime = 50;
volatile unsigned long lastReadTime = 0;
volatile unsigned long lastWriteTime = 25;

bool tankAlarms[4]={0,0,0,0};
bool tankWarnings[4]={0,0,0,0};
int gasWeight,heaterWeight;
bool calMode=false;
byte mac[] = {0x90, 0xA2, 0xDA, 0x0F, 0x22, 0xEE};
volatile float fWeight = 0.1;  
float scale[4];
int lastAddr=0;
float scaleRaw[4]={5404053,2453958,0,0};
//float scaleRaw[4]={0,0,0,0};
//int calCount=20;

//IPAddress ip(172, 20, 1, 221 );
IPAddress ip(172, 21, 100, 29 );
IPAddress subnet(255, 255, 0, 0);
Mudbus Mb;
LiquidCrystal lcds[4]{LiquidCrystal(30,31,32,33,34,35),LiquidCrystal(36,37,38,39,40,41),LiquidCrystal(42,43,44,45,46,47),LiquidCrystal(24,25,26,27,28,29)};
Q2HX711 hx711[4]={Q2HX711(scaleDT1, scaleSCK1),Q2HX711(scaleDT2, scaleSCK2),Q2HX711(scaleDT3, scaleSCK3),Q2HX711(scaleDT4, scaleSCK4)};

struct tankCal
{
  long zeroRawCal,highRawCal,zero,high;
  int tare,gasWeight,heaterWeight,combined;
}tanks[4];
//tankCal tanks[4];


void setup(){
  intitialize();
  updateScales();
  printValues();
}


void loop()
{
  Mb.Run();
  readTemps();
  setHeaterDuty();
  setOutputs();
  
  if(millis()>resetTime){
    resetFunc();
  }//End check for reset

   if(Mb.C[0]==true)
    {
      updateCal();
      Mb.C[0]=false;
    }//End check for input
  
  if (millis() >= (lastReadTime + readTime)){
    lastReadTime = lastReadTime + readTime;
    updateScales();
    
  }

  if (millis() >= (lastWriteTime + writeTime)){
    updateMod();
    updateLCD();
    //printValues();
    lastWriteTime = lastWriteTime + writeTime;
  }
}

void printValues()
{
  Serial.println("addr:"+(String)lastAddr);
  Serial.println("Tank 1: "+(String)scale[0]);
  Serial.println("Tank 2: "+(String)scale[1]);

  Serial.println("zeroRawCal: "+(String)tanks[0].zeroRawCal);
  Serial.println("highRawCal: "+(String)tanks[0].highRawCal);
  Serial.println("zero: "+(String)tanks[0].zero);
  Serial.println("high: "+(String)tanks[0].high);
  Serial.println("tare: "+(String)tanks[0].tare);
  Serial.println("gasWeight: "+(String)tanks[0].gasWeight);
}

void updateScales()
{
  for(int i=0;i<4;i++)
  {
    float newVal=0;
    scaleRaw[i]=hx711[i].read() - 8000000.0;
    if(scaleRaw[i]<=tanks[i].zeroRawCal){
      newVal=tanks[i].zero;
    }else{
      newVal=tanks[i].zero+(scaleRaw[i]-tanks[i].zeroRawCal)*(tanks[i].high-tanks[i].zero)/(tanks[i].highRawCal-tanks[i].zeroRawCal);
    }//end check for 0
    scale[i]=scale[i]+(newVal-tanks[i].tare-scale[i])*fWeight;
    if(scale[i]<=ALARM){
      tankAlarms[i]=true;
      tankWarnings[i]=true;
    } else if(scale[i]<=WARNING){
      tankWarnings[i]=true;
    }else{
      tankAlarms[i]=false;
      tankWarnings[i]=false;
    }
  }//End
  
}//End updateWeights()

void updateLCD()
{
  for(int i=0;i<4;i++)
  {
    lcds[i].setCursor(0,1);
     if (digitalRead(calPIN) || calMode) {
      lcds[i].print("  " + String(long(scaleRaw[i])) + "         ");
    }else if ((scale[i] >= 32767)) {
      lcds[i].print("Modbus Reg Ovflw");
    }else {
      lcds[i].print("  " + String(scale[i]) + "            ");
    }//
  }//End print to LCDs
}//End

void updateCal()
{
    int tank=Mb.R[82];       
    Mb.C[0]=LOW; 
    tanks[tank-1].zeroRawCal=convert(Mb.R[tankStart],Mb.R[tankStart+1]);
    tanks[tank-1].highRawCal=convert(Mb.R[tankStart+2],Mb.R[tankStart+3]);
    tanks[tank-1].zero=convert(Mb.R[tankStart+4],Mb.R[tankStart+5]);
    tanks[tank-1].high=convert(Mb.R[tankStart+6],Mb.R[tankStart+7]);
    tanks[tank-1].combined=convert(Mb.R[tankStart+8],Mb.R[tankStart+9]);
    tanks[tank-1].gasWeight=convert(Mb.R[tankStart+10],Mb.R[tankStart+11]);
    tanks[tank-1].tare=(tanks[tank-1].combined-tanks[tank-1].gasWeight);
    int addr=0;
    for(int i=0;i<4;i++){
      addr+=EEPROM_write(addr,tanks[i]);
    }//End write 
}//End update calibration

/**
 * updateMod()
 * Check if cal mode.  If cal return raw scal values, If not send calibrated values
 */
void updateMod()
{
    unsigned int *p;
    if(Mb.C[1]==false)
    {
      calMode=false;
      p=extract16bit((long)round(scale[0]));
      Mb.R[0]=p[0];
      Mb.R[1]=p[1];
      p=extract16bit((long)round(scale[1]));
      Mb.R[2]=p[0];
      Mb.R[3]=p[1];
      p=extract16bit((long)round(scale[2]));
      Mb.R[4]=p[0];
      Mb.R[5]=p[1];
      p=extract16bit((long)round(scale[3]));
      Mb.R[6]=p[0];
      Mb.R[7]=p[1];
    }else{
      calMode=true;
      p=extract16bit((long)scaleRaw[0]);
      Mb.R[0]=p[0];
      Mb.R[1]=p[1];
      p=extract16bit((long)scaleRaw[1]);
      Mb.R[2]=p[0];
      Mb.R[3]=p[1];
      p=extract16bit((long)scaleRaw[2]);
      Mb.R[4]=p[0];
      Mb.R[5]=p[1];
      p=extract16bit((long)scaleRaw[3]);
      Mb.R[6]=p[0];
      Mb.R[7]=p[1];
    }//End check for cal mode 
      p=extract16bit((long)tanks[0].zeroRawCal);
      Mb.R[8]=p[0];
      Mb.R[9]=p[1];
      p=extract16bit((long)tanks[0].highRawCal);
      Mb.R[10]=p[0];
      Mb.R[11]=p[1];
      p=extract16bit((long)tanks[0].zero);
      Mb.R[12]=p[0];
      Mb.R[13]=p[1];
      p=extract16bit((long)tanks[0].high);
      Mb.R[14]=p[0];
      Mb.R[15]=p[1];
      p=extract16bit((long)tanks[0].combined);
      Mb.R[16]=p[0];
      Mb.R[17]=p[1];
      p=extract16bit((long)tanks[0].gasWeight);
      Mb.R[18]=p[0];
      Mb.R[19]=p[1];

      p=extract16bit((long)tanks[1].zeroRawCal);
      Mb.R[20]=p[0];
      Mb.R[21]=p[1];
      p=extract16bit((long)tanks[1].highRawCal);
      Mb.R[22]=p[0];
      Mb.R[23]=p[1];
      p=extract16bit((long)tanks[1].zero);
      Mb.R[24]=p[0];
      Mb.R[25]=p[1];
      p=extract16bit((long)tanks[1].high);
      Mb.R[26]=p[0];
      Mb.R[27]=p[1];
      p=extract16bit((long)tanks[1].combined);
      Mb.R[28]=p[0];
      Mb.R[29]=p[1];
      p=extract16bit((long)tanks[1].gasWeight);
      Mb.R[30]=p[0];
      Mb.R[31]=p[1];

      p=extract16bit((long)tanks[2].zeroRawCal);
      Mb.R[32]=p[0];
      Mb.R[33]=p[1];
      p=extract16bit((long)tanks[2].highRawCal);
      Mb.R[34]=p[0];
      Mb.R[35]=p[1];
      p=extract16bit((long)tanks[2].zero);
      Mb.R[36]=p[0];
      Mb.R[37]=p[1];
      p=extract16bit((long)tanks[2].high);
      Mb.R[38]=p[0];
      Mb.R[39]=p[1];
      p=extract16bit((long)tanks[2].combined);
      Mb.R[40]=p[0];
      Mb.R[41]=p[1];
      p=extract16bit((long)tanks[2].gasWeight);
      Mb.R[42]=p[0];
      Mb.R[43]=p[1];

      p=extract16bit((long)tanks[3].zeroRawCal);
      Mb.R[44]=p[0];
      Mb.R[45]=p[1];
      p=extract16bit((long)tanks[3].highRawCal);
      Mb.R[46]=p[0];
      Mb.R[47]=p[1];
      p=extract16bit((long)tanks[3].zero);
      Mb.R[48]=p[0];
      Mb.R[49]=p[1];
      p=extract16bit((long)tanks[3].high);
      Mb.R[50]=p[0];
      Mb.R[51]=p[1];
      p=extract16bit((long)tanks[3].combined);
      Mb.R[52]=p[0];
      Mb.R[53]=p[1];
      p=extract16bit((long)tanks[3].gasWeight);
      Mb.R[54]=p[0];
      Mb.R[55]=p[1];
      Mb.R[56]=tanks[0].tare;
      Mb.R[57]=tanks[1].tare;
      Mb.R[58]=tanks[2].tare;
      Mb.R[59]=tanks[3].tare;
      Mb.R[60] = round(tankTemp1 * 100);
      Mb.R[61] = round(tankTemp2 * 100);
      Mb.R[62] = round(tankTemp3 * 100);
      Mb.R[63] = round(tankTemp4 * 100);
      Mb.R[64] = round(lineTemp1 * 100);
      Mb.R[65] = round(lineTemp2 * 100);
      Mb.R[66] = heaterDuty1;
      Mb.R[67] = heaterDuty2;
      Mb.R[68] = heaterDuty3;
      Mb.R[69] = heaterDuty4;
      
      for(int i=0;i<8;i++){
        if(i<4){
          Mb.C[i+2]=tankWarnings[i]; 
        }else{
          Mb.C[i+2]=tankAlarms[i-3]; 
        }//End 
      }//End set warnings/alarms
      //free(p);
}//End updateMod()

void intitialize()
{
    analogReference(EXTERNAL);
    pinMode(SS_SD_CARD, OUTPUT);
    pinMode(SS_ETHERNET, OUTPUT);
    digitalWrite(SS_ETHERNET, LOW);  // Ethernet ACTIVE
    digitalWrite(SS_ETHERNET, HIGH);  // SD Card Deactivated
    
    pinMode(scaleDT1, INPUT);
    pinMode(scaleSCK1, OUTPUT);
    pinMode(scaleDT2, INPUT);
    pinMode(scaleSCK2, OUTPUT);
    pinMode(scaleDT3, INPUT);
    pinMode(scaleSCK3, OUTPUT);
    pinMode(scaleDT4, INPUT);
    pinMode(scaleSCK4, OUTPUT);
    pinMode(calPIN, INPUT);
    pinMode(heatPin1, OUTPUT);  digitalWrite(heatPin1, LOW);
    pinMode(heatPin2, OUTPUT);  digitalWrite(heatPin2, LOW);
    pinMode(heatPin3, OUTPUT);  digitalWrite(heatPin3, LOW);
    pinMode(heatPin4, OUTPUT);  digitalWrite(heatPin4, LOW);
    
    pinMode(tankTempPin1, INPUT);
    pinMode(tankTempPin2, INPUT);
    pinMode(tankTempPin3, INPUT);
    pinMode(tankTempPin4, INPUT);
    pinMode(lineTempPin1, INPUT);
    pinMode(lineTempPin2, INPUT);
    
    int addr=0;
    for(int i=0;i<4;i++)
    {
      addr+=EEPROM_read(addr,tanks[i]);
    }//
    lastAddr=addr; 
    
    lcds[0].begin(16, 2);
    lcds[1].begin(16, 2);
    lcds[2].begin(16, 2);
    lcds[3].begin(16, 2);
    
    Ethernet.begin(mac, ip, subnet);

    //Clear all mbus reg/coils
    for (int i = 0 ; i <255; i  ++) {
      if(i<125){
        Mb.R[i] = 0;
        Mb.C[i]=false;
      }else{
        Mb.C[i]=false;
      }
    }//End loop

    lcds[0].print(" NH3 Tank 1");
    lcds[1].print(" NH3 Tank 2");
    lcds[2].print(" NH3 Tank 3");
    lcds[3].print(" NH3 Tank 4");

        //initialize temperature values
    aValue = analogRead(tankTempPin1);
    tankTemp1 = (((float)(aValue) * 500.0) / 1024.0);
    aValue = analogRead(tankTempPin2);
    tankTemp2 = (((float)(aValue) * 500.0) / 1024.0);
    aValue = analogRead(tankTempPin3);
    tankTemp3 = (((float)(aValue) * 500.0) / 1024.0);
    aValue = analogRead(tankTempPin4);
    tankTemp4 = (((float)(aValue) * 500.0) / 1024.0);
    aValue = analogRead(lineTempPin1);
    lineTemp1 = (((float)(aValue) * 500.0) / 1024.0);
    aValue = analogRead(lineTempPin2);
    lineTemp2 = (((float)(aValue) * 500.0) / 1024.0);
    
    for (int i = 0; i < 1000; i++){
      readTemps();
      delay(10);
    }//End intial temp read
    heatReadTime = 100;
    #if DEBUG
      Serial.begin(38400);
    #endif
}//End init

volatile unsigned long lastHeatReadTime = 0;
void readTemps() 
{
  if (millis() >= (lastHeatReadTime + heatReadTime)) {
    lastHeatReadTime = lastHeatReadTime + heatReadTime;
    aValue = analogRead(tankTempPin1);
    tankTemp1 = tankTemp1 + ((((float)(aValue) * 500.0) / 1024.0) - tankTemp1) * heatfWeight;
    aValue = analogRead(tankTempPin2);
    tankTemp2 = tankTemp2 + ((((float)(aValue) * 500.0) / 1024.0) - tankTemp2) * heatfWeight;
    aValue = analogRead(tankTempPin3);
    tankTemp3 = tankTemp3 + ((((float)(aValue) * 500.0) / 1024.0) - tankTemp3) * heatfWeight;
    aValue = analogRead(tankTempPin4);
    tankTemp4 = tankTemp4 + ((((float)(aValue) * 500.0) / 1024.0) - tankTemp4) * heatfWeight;
    aValue = analogRead(lineTempPin1);
    lineTemp1 = lineTemp1 + ((((float)(aValue) * 500.0) / 1024.0) - lineTemp1) * heatfWeight;
    aValue = analogRead(lineTempPin2);
    lineTemp2 = lineTemp2 + ((((float)(aValue) * 500.0) / 1024.0) - lineTemp2) * heatfWeight;
  }//End if 
}//End readTemps()

volatile unsigned long lastDutyTime = 525;
#define dutyTime 10000

void setHeaterDuty()
{
  if(firstRun){
    lastErr1 = err1; err1 = tempSP - tankTemp1;
    lastErr2 = err2; err2 = tempSP - tankTemp2;
    lastErr3 = err3; err3 = tempSP - tankTemp3;
    lastErr4 = err4; err4 = tempSP - tankTemp4;
    firstRun = false;
  }
  if (millis() >= (lastDutyTime + dutyTime))
  {
    lastDutyTime = lastDutyTime + dutyTime;
    
    lastErr1 = err1; err1 = tempSP - tankTemp1;
    lastErr2 = err2; err2 = tempSP - tankTemp2;
    lastErr3 = err3; err3 = tempSP - tankTemp3;
    lastErr4 = err4; err4 = tempSP - tankTemp4;

    iTerm1 = iTerm1 + err1 * Ki;
    iTerm2 = iTerm2 + err2 * Ki;
    iTerm3 = iTerm3 + err3 * Ki;
    iTerm4 = iTerm4 + err4 * Ki;

    pTerm1 = Kp * err1;
    pTerm2 = Kp * err2;
    pTerm3 = Kp * err3;
    pTerm4 = Kp * err4;

    dTerm1 = dTerm1 + ((Kd * (err1 - lastErr1)) - dTerm1) * 0.001;
    dTerm2 = dTerm2 + ((Kd * (err2 - lastErr2)) - dTerm2) * 0.001;
    dTerm3 = dTerm3 + ((Kd * (err3 - lastErr3)) - dTerm3) * 0.001;
    dTerm4 = dTerm4 + ((Kd * (err4 - lastErr4)) - dTerm4) * 0.001;

    Duty1 = pTerm1 + iTerm1 + dTerm1; if (Duty1 > 100.0) {
      Duty1 = 100.0;
    } else if (Duty1 < 0.0){
      Duty1 = 0.0;
    }
    Duty2 = pTerm2 + iTerm2 + dTerm2; if (Duty2 > 100.0) {
      Duty2 = 100.0;
    }else if (Duty2 < 0.0){
      Duty2 = 0.0;
    }
    Duty3 = pTerm3 + iTerm3 + dTerm3; if (Duty3 > 100.0) {
      Duty3 = 100.0;
    } else if (Duty3 < 0.0) {
      Duty3 = 0.0;
    }
    Duty2 = pTerm4 + iTerm4 + dTerm4; if (Duty4 > 100.0) {
      Duty4 = 100.0;
    }else if (Duty4 < 0.0) {
      Duty4 = 0.0;
    }
    if (tankTemp1 > tempHLimit) {
      heaterDuty1 = 0;
    } else if (tankTemp1 < tempLLimit) {
      heaterDuty1 = maxDuty;
    }else if (tankTemp1 < tempHLimit) {
      heaterDuty1 = (int(Duty1) * maxDuty) / 100; if ((heaterDuty1 > hiDuty) || (heaterDuty1 > maxDuty)) {
        heaterDuty1 = maxDuty;
      }else if (heaterDuty1 < lowDuty) {
        heaterDuty1 = 0;
      }
    }//End outer
    if (tankTemp2 > tempHLimit) {
      heaterDuty2 = 0;
    }else if (tankTemp2 < tempLLimit) {
      heaterDuty2 = maxDuty;
    }else if (tankTemp2 < tempHLimit) {
      heaterDuty2 = (int(Duty2) * maxDuty) / 100; 
      if ((heaterDuty2 > hiDuty) || (heaterDuty2 > maxDuty)) {
        heaterDuty2 = maxDuty;
      }else if (heaterDuty2 < lowDuty) {
        heaterDuty2 = 0;
      }//End else if
    }//End outer
    if (tankTemp3 > tempHLimit) {
      heaterDuty3 = 0;
    }else if (tankTemp3 < tempLLimit) {
      heaterDuty3 = maxDuty;
    }else if (tankTemp3 < tempHLimit) {
      heaterDuty3 = (int(Duty3) * maxDuty) / 100;
      if ((heaterDuty3 > hiDuty) || (heaterDuty3 > maxDuty)) {
        heaterDuty3 = maxDuty;
      }else if (heaterDuty3 < lowDuty) {
        heaterDuty3 = 0;
      }//End outer
    }if (tankTemp4 > tempHLimit) {
      heaterDuty4 = 0;
    }else if (tankTemp4 < tempLLimit) {
      heaterDuty4 = maxDuty;
    }else if (tankTemp4 < tempHLimit) {
      heaterDuty4 = (int(Duty4) * maxDuty) / 100; 
      if ((heaterDuty4 > hiDuty) || (heaterDuty4 > maxDuty)) {
        heaterDuty4 = maxDuty;
      }else if (heaterDuty4 < lowDuty){
        heaterDuty4 = 0;
      }
    }//End outer

#if DEBUG
    Serial.print(tankTemp1, 2); Serial.print("\t"); Serial.print(Duty1); Serial.print("\t"); Serial.print(heaterDuty1); Serial.print("\t"); Serial.print(pTerm1); Serial.print("+"); Serial.print(iTerm1); Serial.print("+"); Serial.print(dTerm1); Serial.print("\t"); Serial.print(tankTemp2, 2); Serial.print("\t"); Serial.print(Duty2); Serial.print("\t"); Serial.print(heaterDuty2); Serial.print("\t"); Serial.print(pTerm2); Serial.print("+"); Serial.print(iTerm2); Serial.print("+"); Serial.println(dTerm2);
#endif
  }
}

void setOutputs() 
{
  if (millis() >= (lastRunTime + runTime)) {
    lastRunTime = lastRunTime + runTime;
    if (millis() >= (lastOutputTime + outputPeriod)) {
      lastOutputTime = lastOutputTime + outputPeriod;
      if (heaterDuty1 > 0) {
        digitalWrite(heatPin1, HIGH);
      } else {
        digitalWrite(heatPin1, LOW);
      }
      if (heaterDuty2 > 0) {
        digitalWrite(heatPin2, HIGH);
      } else {
        digitalWrite(heatPin2, LOW);
      }
      if (heaterDuty3 > 0) {
        digitalWrite(heatPin3, HIGH);
      } else {
        digitalWrite(heatPin3, LOW);
      }
      if (heaterDuty4 > 0) {
        digitalWrite(heatPin4, HIGH);
      } else {
        digitalWrite(heatPin4, LOW);
      }
    }
    if (millis() >= (lastOutputTime + (outputPeriod / 100)*heaterDuty1)) {
      digitalWrite(heatPin1, LOW);
    }
    if (millis() >= (lastOutputTime + (outputPeriod / 100)*heaterDuty2)) {
      digitalWrite(heatPin2, LOW);
    }
    if (millis() >= (lastOutputTime + (outputPeriod / 100)*heaterDuty3)) {
      digitalWrite(heatPin3, LOW);
    }
    if (millis() >= (lastOutputTime + (outputPeriod / 100)*heaterDuty4)) {
      digitalWrite(heatPin4, LOW);
    }
  }//End outer if
}//End setOutputs
