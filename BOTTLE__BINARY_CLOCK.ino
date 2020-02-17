//************************************************************************************************************//
#include <SPI.h>
#include <NTPClient.h>
#include <ESP8266WiFi.h>
#include <time.h>
#include "timezone.h"

#define blank_pin       D6  // BLANK PIN - TPIC6B595
#define latch_pin       D4  // LATCH PIN - TPIC6B595
#define clock_pin       D5  // CLOCK PIN - TPIC6B595
#define data_pin        D7  // DATA PIN - TPIC6B595

#define ROW0            D8
#define ROW1            D0

//MAPPING TO PORT

#define Blank_Pin_Bit     12  // GPIO12
#define Latch_Pin_Bit     2   // GPIO2
#define Clock_Pin_Bit     14  // GPIO14
#define Data_Pin_Bit      13  // GPIO13

const char *WIFI_NETWORK_NAME = "XXXXXXXX"; // Change to your wifi network name
const char *WIFI_PASSWORD     = "YYYYYYYY";   // Change to your wifi password

const char *TIME_SERVER       = "asia.pool.ntp.org";
int myTimeZone = VST;   // change this to your time zone (see in timezone.h)

time_t now;

char H1_Number, H0_Number, M1_Number, M0_Number;

char s0, s1, m0, m1, h0, h1;
char prves0, prves1, prvem0, prvem1, prveh0, prveh1;

unsigned long samplingtimes0 = 0;
unsigned long samplingtimes1 = 0;
unsigned long samplingtimem0 = 0;
unsigned long samplingtimem1 = 0;
unsigned long samplingtimeh0 = 0;
unsigned long samplingtimeh1 = 0;

//************************************************************************************************************//

byte red[4][2];
byte green[4][2];
byte blue[4][2];

int row=0;
int BAM_Bit, BAM_Counter=0; 

struct Color
{
  unsigned char red, green, blue;

  Color(int r, int g, int b) : red(r), green(g) , blue(b){}
  Color() : red(0), green(0), blue(0) {}
};

const Color redcolor        = Color(0x0F, 0x00, 0x00);
const Color orangecolor     = Color(0x0F, 0x0F, 0x00);
const Color yellowcolor     = Color(0x0F, 0x09, 0x00);
const Color greencolor      = Color(0x00, 0x0F, 0x00);
const Color tealcolor       = Color(0x00, 0x0F, 0x04);
const Color bluecolor       = Color(0x00, 0x00, 0x0F);
const Color purplecolor     = Color(0x0F, 0x00, 0x0F);
const Color whitecolor      = Color(0x0F, 0x0F, 0x0F);
const Color blackcolor      = Color(0x00, 0x00, 0x00);

//************************************************************************************************************//

#define BAM_RESOLUTION 4  
#define COLOUR_WHEEL_LENGTH 256

uint8_t colourR[COLOUR_WHEEL_LENGTH];
uint8_t colourG[COLOUR_WHEEL_LENGTH];
uint8_t colourB[COLOUR_WHEEL_LENGTH];
int16_t ColPos = 0;
uint16_t colourPos;
uint8_t R, G, B;

#define myPI      3.14159265358979323846
#define myDPI     1.2732395
#define myDPI2    0.40528473

byte H0[4][2] = {{1, 0},{1, 1},{0, 1},{0, 0}};  // Hour - Ten digit
byte H1[4][2] = {{1, 2},{1, 3},{0, 3},{0, 2}};  // Hour - Unit digit
byte M0[4][2] = {{1, 4},{1, 5},{0, 5},{0, 4}};  // Minute - Ten digit
byte M1[4][2] = {{1, 6},{1, 7},{0, 7},{0, 6}};  // Minute - Unit digit

void LED(int X, int Y, int R, int G, int B);
void timer1_ISR(void);
void clearfast();
void fill_colour_wheel(void);
void get_colour(int16_t p, uint8_t *R, uint8_t *G);
void get_next_colour(uint8_t *R, uint8_t *G);
void increment_colour_pos(uint8_t i);
void DrawDot(byte number, byte coordinates[4][2], Color frontcolor, Color backcolor);
void Effect_M0(Color frontcolor, Color backcolor);
void Effect_M1(Color frontcolor, Color backcolor);
void Effect_H0(Color frontcolor, Color backcolor);
void Effect_H1(Color frontcolor, Color backcolor);

void setup()
{
WiFi.begin(WIFI_NETWORK_NAME, WIFI_PASSWORD);

while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
  }

configTime(3600*myTimeZone, 0, TIME_SERVER);

while (now < EPOCH_1_1_2019)
  {
    now = time(nullptr);
    delay(500);
  }

SPI.setDataMode(SPI_MODE0);
SPI.setBitOrder(MSBFIRST);
SPI.setFrequency(4000000);
noInterrupts();
  
pinMode(latch_pin, OUTPUT);
pinMode(data_pin, OUTPUT);
pinMode(clock_pin, OUTPUT);

pinMode(ROW0, OUTPUT);
pinMode(ROW1, OUTPUT);

SPI.begin();

timer1_isr_init();
timer1_attachInterrupt(timer1_ISR);
timer1_enable(TIM_DIV16, TIM_EDGE, TIM_SINGLE);
timer1_write(100);
interrupts();
fill_colour_wheel();

clearfast();

if (WiFi.status() == WL_CONNECTED)
  {
    for (int x=0; x<2; x++)
    {
      for (int y=0; y<8; y++)
        {
          LED(x, y, 15, 15, 15);
          delay(500);
        }
      }
    for (int x=0; x<2; x++)
      {
        for (int y=0; y<8; y++)
        {
          LED(x, y, 0, 0, 0);
          delay(500);
        }
      }
   }
clearfast();
}

void loop()
{    
    struct tm *timeinfo;
    time(&now);
    timeinfo = localtime(&now);
    int year        = timeinfo->tm_year + 1900;
    int month       = timeinfo->tm_mon + 1;
    int day         = timeinfo->tm_mday;
    int hour        = timeinfo->tm_hour;
    int mins        = timeinfo->tm_min;
    int sec         = timeinfo->tm_sec;
    int day_of_week = timeinfo->tm_wday;

    H0_Number = ((hour/10) %10) + 48;
    H1_Number = (hour%10) + 48;

    M0_Number = ((mins/10) %10) + 48;
    M1_Number = (mins%10) + 48;
    
    Effect_M0(bluecolor, whitecolor);
    Effect_M1(purplecolor, whitecolor);
    Effect_H0(greencolor, whitecolor);
    Effect_H1(redcolor, whitecolor); 
}         

void LED(int X, int Y, int R, int G, int B)
{
  X = constrain(X, 0, 1);
  Y = constrain(Y, 0, 7);
  
  R = constrain(R, 0, 15);
  G = constrain(G, 0, 15); 
  B = constrain(B, 0, 15);

  for (byte BAM = 0; BAM < BAM_RESOLUTION; BAM++) 
  {
    bitWrite(red[BAM][X], Y, bitRead(R, BAM));

    bitWrite(green[BAM][X], Y, bitRead(G, BAM));
    
    bitWrite(blue[BAM][X], Y, bitRead(B, BAM));
  } 
}

void rowScan(byte row)
{  
  if (row == 0)
  {
    digitalWrite(ROW0,HIGH);
    delayMicroseconds(1);
  }
  else 
  {
    digitalWrite(ROW0,LOW);
    delayMicroseconds(1);
  }
      
  
  if (row == 1)
  {
    digitalWrite(ROW1,HIGH);
    delayMicroseconds(1);
  }
  else
  {
    digitalWrite(ROW1,LOW);
    delayMicroseconds(1);
  }
}

void ICACHE_RAM_ATTR timer1_ISR(void)
{
  
digitalWrite(blank_pin, HIGH);  // Set BLANK PIN high - TPIC6B595   

if(BAM_Counter==8)
BAM_Bit++;
else
if(BAM_Counter==24)
BAM_Bit++;
else
if(BAM_Counter==56)
BAM_Bit++;

BAM_Counter++;

switch (BAM_Bit)
{
    case 0:
      SPI.transfer(blue[0][row]);   
      SPI.transfer(green[0][row]);      
      SPI.transfer(red[0][row]);
      break;
    case 1:
      SPI.transfer(blue[1][row]);    
      SPI.transfer(green[1][row]);  
      SPI.transfer(red[1][row]);        
      break;
    case 2:     
      SPI.transfer(blue[2][row]); 
      SPI.transfer(green[2][row]); 
      SPI.transfer(red[2][row]);     
      break;
    case 3:
      SPI.transfer(blue[3][row]); 
      SPI.transfer(green[3][row]); 
      SPI.transfer(red[3][row]);    
  if(BAM_Counter==120){
  BAM_Counter=0;
  BAM_Bit=0;
  }
  break;
}

rowScan(row);

digitalWrite(latch_pin, HIGH);    // Set LATCH PIN high - TPIC6B595
delayMicroseconds(3);
digitalWrite(latch_pin, LOW);     // Set LATCH PIN low - TPIC6B595
delayMicroseconds(3);
digitalWrite(blank_pin, LOW);     // Set BLANK PIN low - TPIC6B595
delayMicroseconds(3);
row++;
if(row==2)
row=0;
timer1_write(100);     //Interrupt will be called every 100 x 0.2us = 20us
pinMode(blank_pin, OUTPUT);
}

void clearfast ()
{
    memset(red, 0, sizeof(red[0][0]) * 4 * 2);
    memset(green, 0, sizeof(green[0][0]) * 4 * 2);
    memset(blue, 0, sizeof(blue[0][0]) * 4 * 2);
        
}

void fillTable(byte R, byte G, byte B)
{
  for (byte x=0; x<2; x++)
    {
      for (byte y=0; y<8; y++)
      {
        LED(x, y, R, G, B);
      }
    }
}

void fillTable_colorwheelRGB(int potentio, byte R, byte G, byte B)
{
  for (byte x=0; x<2; x++)
    {      
      for (byte y=0; y<8; y++)
      {
        get_colour(potentio + 36*(y+2*x), &R, &G, &B);
        LED(x, y, R, G, B);      
      }
    }
}  

//************************************************************************************************************//

//FAST SINE APPROX
float mySin(float x){
  float sinr = 0;
  uint8_t g = 0;

  while(x > myPI){
    x -= 2*myPI; 
    g = 1;
  }

  while(!g&(x < -myPI)){
    x += 2*myPI;
  }

  sinr = myDPI*x - myDPI2*x*myAbs(x);
  sinr = 0.225*(sinr*myAbs(sinr)-sinr)+sinr;

  return sinr;
}

//FAST COSINE APPROX
float myCos(float x){
  return mySin(x+myPI/2);
}

float myTan(float x){
  return mySin(x)/myCos(x);
}

//SQUARE ROOT APPROX
float mySqrt(float in){
  int16_t d = 0;
  int16_t in_ = in;
  float result = 2;
  
  for(d = 0; in_ > 0; in_ >>= 1){
    d++;
  }
  
  for(int16_t i = 0; i < d/2; i++){
    result = result*2;
  }
  
  for(int16_t i = 0; i < 3; i++){
    result = 0.5*(in/result + result);
  }
  
  return result;
}

//ABSOLUTE VALUE
float myAbs(float in){
  return (in)>0?(in):-(in);
} 

void fill_colour_wheel(void) 
{
  float red, green, blue;
  float c, s;
  int32_t phase = 0;
  int16_t I = 0;

  while (phase < COLOUR_WHEEL_LENGTH) 
  {
    s = (1 << BAM_RESOLUTION)*mySin(myPI*(3 * phase - I*COLOUR_WHEEL_LENGTH) / (2 * COLOUR_WHEEL_LENGTH));
    c = (1 << BAM_RESOLUTION)*myCos(myPI*(3 * phase - I*COLOUR_WHEEL_LENGTH) / (2 * COLOUR_WHEEL_LENGTH));

    red = (I == 0 ? 1 : 0)*s + (I == 1 ? 1 : 0)*c;
    green = (I == 1 ? 1 : 0)*s + (I == 2 ? 1 : 0)*c;
    blue = (I == 2 ? 1 : 0)*s + (I == 0 ? 1 : 0)*c;

    colourR[phase] = red;
    colourG[phase] = green;
    colourB[phase] = blue;

    if (++phase >= (1 + I)*COLOUR_WHEEL_LENGTH / 3) 
      I++;
  }
}

void get_colour(int16_t p, uint8_t *R, uint8_t *G, uint8_t *B)
{
  if (p >= COLOUR_WHEEL_LENGTH)
    p -= COLOUR_WHEEL_LENGTH;

  *R = colourR[p];
  *G = colourG[p];
  *B = colourB[p];
}

void get_next_colour(uint8_t *R, uint8_t *G, uint8_t *B)
{
  if (++ColPos >= COLOUR_WHEEL_LENGTH)
    ColPos -= COLOUR_WHEEL_LENGTH;

  *R = colourR[ColPos];
  *G = colourG[ColPos];
  *B = colourB[ColPos];
}

void increment_colour_pos(uint8_t i)
{
  colourPos += i;
  while (colourPos >= COLOUR_WHEEL_LENGTH)
  {
    colourPos -= COLOUR_WHEEL_LENGTH;
  }
}


void DrawDot(byte number, byte coordinates[4][2], Color frontcolor, Color backcolor)
{ 
    
    for (int i = 0; i < 4; i++)
      {
        if (bitRead(number, i))
        {
        LED(coordinates[i][0], coordinates[i][1], frontcolor.red, frontcolor.green, frontcolor.blue);
        }
        else
        {
        LED(coordinates[i][0], coordinates[i][1], backcolor.red, backcolor.green, backcolor.blue);  
        }
      }
}



void Effect_M1(Color frontcolor, Color backcolor)
{ 
  if ( (unsigned long) (micros() - samplingtimem1) > 555  )
  {     
    m1 = M1_Number;
    if (m1 != prvem1)  
      {      
        DrawDot(M1_Number, M1, frontcolor, backcolor);              
        prvem1 = m1;
      }
      samplingtimem1 = micros(); 
    }
}   

void Effect_M0(Color frontcolor, Color backcolor)
{ 
  if ( (unsigned long) (micros() - samplingtimem0) > 255  )
  {
    m0 = M0_Number;
    if (m0!=prvem0)  
      {         
        DrawDot(M0_Number, M0, frontcolor, backcolor);  
        prvem0 = m0;
      }
      samplingtimem0 = micros(); 
    }
} 

void Effect_H1(Color frontcolor, Color backcolor)
{ 
  if ( (unsigned long) (micros() - samplingtimeh1) > 555  )
  {

    h1 = H1_Number;
    if (h1 != prveh1)  
      {          
        DrawDot(H1_Number, H1, frontcolor, backcolor); 
        prveh1 = h1;
      }
      samplingtimeh1 = micros(); 
    }
}       

void Effect_H0(Color frontcolor, Color backcolor)
{ 
  if ( (unsigned long) (micros() - samplingtimeh0) > 455  )
  {    
    h0 = H0_Number;    
    if (h0 != prveh0)  
      {   
        DrawDot(H0_Number, H0, frontcolor, backcolor); 
        prveh0 = h0;
      }
      samplingtimeh0 = micros(); 
    }
} 
