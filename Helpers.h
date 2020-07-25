template <class T> int EEPROM_write(int addr,const T& value)
{
  const byte* p=(const byte*)(const void*)&value;
  int newAddr;
  for(newAddr=0;newAddr<sizeof(value);newAddr++)
  {
    EEPROM.write(addr++,*p++);
  }
  return newAddr;
}//End write any value/type

template <class T> int EEPROM_read(int addr, T& value)
{
   byte* p = (byte*)(void*)&value;
   int newAddr;
   for (newAddr = 0; newAddr < sizeof(value); newAddr++)
       *p++ = EEPROM.read(addr++);
   return newAddr;
}//End read any value/type

float convert(int raw1,int raw2)
{
    int raw[2];
    raw[1]=raw1;
    raw[0]=raw2;
    long result=0;
    memcpy(&result,&raw,sizeof(long));
    return (float)result;
}//End convert

/**
 * convertToFloat
 *  Converts 2 integers to float, read in from mbus
 */
float convertToFloat(int raw1,int raw2)
{
  unsigned int raw[2];
  raw[1]=raw1;
  raw[0]=raw2;
  float result;
  memcpy(&result,&raw,sizeof(float));
  return result;
}//End convert to float

/*
 * extract16bit(long)
 *   converting 32bit long into 2 16bit integers(cannot be negative)
 */

 unsigned int* extract16bit(long x)
{
    static unsigned int result[2];
    result[0]=(unsigned int)(x>>16);
    result[1]=(unsigned int)(x&0x0000FFFF);
    return result;
}//convert long into 2 16bit registers

uint16_t* extract16bit(long x)
{
    //static unsigned int result[2];
    
    uint16_t *result=(uint16_t *)malloc(2*sizeof(uint16_t));
    result[0]=(uint16_t)(x>>16);
    result[1]=(uint16_t)(x&0x0000FFFF);
    return result;
}//convert long into 2 16bit registers

void(* resetFunc) (void) = 0; //declare reset function @ address 0

#define calPIN 48

#define scaleDT1 2
#define scaleSCK1 3
#define scaleDT2 4
#define scaleSCK2 5
#define scaleDT3 6
#define scaleSCK3 7
#define scaleDT4 8
#define scaleSCK4 9


#define SS_SD_CARD 4
#define SS_ETHERNET 10

//Heater Constants

#define heatPin1 A10
#define heatPin2 A11
#define heatPin3 A12
#define heatPin4 A13
#define tankTempPin1 A0
#define tankTempPin2 A1
#define tankTempPin3 A2
#define tankTempPin4 A3
#define lineTempPin1 A4
#define lineTempPin2 A5
#define tempSP 27.0
#define tempHLimit 28
#define tempLLimit 22.5

#define Kp 150.0
#define Ki 0.001
#define Kd 150000
#define maxDuty 100
#define hiDuty 98
#define lowDuty 2

#define resetTime 604800000
#define readTime 100
#define writeTime 500

#define outputPeriod 3000
#define runTime 50

template <class T> int EEPROM_write(int addr,const T& value)
{
  const byte* p=(const byte*)(const void*)&value;
  int newAddr;
  for(newAddr=0;newAddr<sizeof(value);newAddr++)
  {
    EEPROM.write(addr++,*p++);
  }
  return newAddr;
}//End write any value/type

template <class T> int EEPROM_read(int addr, T& value)
{
   byte* p = (byte*)(void*)&value;
   int newAddr;
   for (newAddr = 0; newAddr < sizeof(value); newAddr++)
       *p++ = EEPROM.read(addr++);
   return newAddr;
}//End read any value/type

float convert(int raw1,int raw2)
{
    int raw[2];
    raw[1]=raw1;
    raw[0]=raw2;
    long result=0;
    memcpy(&result,&raw,sizeof(long));
    return (float)result;
}//End convert

/**
 * convertToFloat
 *  Converts 2 integers to float, read in from mbus
 */
float convertToFloat(int raw1,int raw2)
{
  unsigned int raw[2];
  raw[1]=raw1;
  raw[0]=raw2;
  float result;
  memcpy(&result,&raw,sizeof(float));
  return result;
}//End convert to float

/*
 * extract16bit(long)
 *   converting 32bit long into 2 16bit integers(cannot be negative)
 */

 unsigned int* extract16bit(long x)
{
    static unsigned int result[2];
    result[0]=(unsigned int)(x>>16);
    result[1]=(unsigned int)(x&0x0000FFFF);
    return result;
}//convert long into 2 16bit registers
/*uint16_t* extract16bit(long x)
{
    //static unsigned int result[2];
    
    uint16_t *result=(uint16_t *)malloc(2*sizeof(uint16_t));
    result[0]=(uint16_t)(x>>16);
    result[1]=(uint16_t)(x&0x0000FFFF);
    return result;
}//convert long into 2 16bit registers
*/
void(* resetFunc) (void) = 0; //declare reset function @ address 0
