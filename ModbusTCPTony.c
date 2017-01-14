#include <SPI.h>
#include <Ethernet.h>
#include <EEPROM.h>
#include "Mudbus.h"
void(* resetFunc) (void) = 0; //declare reset function @ address 0
Mudbus Mb;
//Function codes 1(read coils), 3(read registers), 5(write coil), 6(write register)
//signed int Mb.R[0 to 125] and bool Mb.C[0 to 128] MB_N_R MB_N_C
//Port 502 (defined in Mudbus.h) MB_PORT

#include <Wire.h>                   // Comes with Arduino IDE
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd1(0x24, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  // Set the 1st LCD I2C address
LiquidCrystal_I2C lcd2(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  // Set the 2nd LCD I2C address
LiquidCrystal_I2C lcd3(0x25, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  // Set the 2nd LCD I2C address

volatile unsigned int count = 0;
volatile unsigned int halfsec = 0;
unsigned int sec = 0;
int Ai[] = {A0, A1, A2, A3, A4, A5, A6, A7, A8, A9, A10, A11, A12};
int flag = 0; //Form Flag
float dataf[13] = {};
#define modepin A0 //680 510 0
int mac1, mac2, mac3, mac4, mac5, mac6;
int Ip1, Ip2, Ip3, Ip4, Ip5;
int Gateway1, Gateway2, Gateway3, Gateway4;
int Subnet1, Subnet2, Subnet3, Subnet4;
int RemoteResetM=20;
int ResetValue=30;

float mapMAX;
float mapMIN;
float RealMAX;
float RealMIN;



//ModBus Add

int Mac1addM = 50;
int Mac2addM = 51;
int Mac3addM = 52;
int Mac4addM = 53;
int Mac5addM = 54;
int Mac6addM = 55;

int Ip1addM = 56;
int Ip2addM = 57;
int Ip3addM = 58;
int Ip4addM = 59;

int Gateway1addM = 60;
int Gateway2addM = 61;
int Gateway3addM = 62;
int Gateway4addM = 63;

int Subnet1addM = 64;
int Subnet2addM = 65;
int Subnet3addM = 66;
int Subnet4addM = 67;

int mapMAXaddM = 120;
int mapMINaddM = 121;
int RealMAXaddM = 122;
int RealMINaddM = 123;


//EEROM Add
int MapMAXaddr = 11;
int MapMINaddr = 13;
int RealMAXaddr = 15;
int RealMINaddr = 17;

int Mac1addr = 19;
int Mac2addr = 20;
int Mac3addr = 21;
int Mac4addr = 22;
int Mac5addr = 23;
int Mac6addr = 24;

int Ip1addr = 25;
int Ip2addr = 26;
int Ip3addr = 27;
int Ip4addr = 28;

int Gateway1addr = 29;
int Gateway2addr = 30;
int Gateway3addr = 31;
int Gateway4addr = 32;

int Subnet1addr = 33;
int Subnet2addr = 34;
int Subnet3addr = 35;
int Subnet4addr = 36;

int mapMAXtempEROM, mapMINtempEROM, RealMAXtempEROM , RealMINtempEROM ;
int Mac1tempEROM, Mac2tempEROM, Mac3tempEROM, Mac4tempEROM, Mac5tempEROM, Mac6tempEROM;
int Ip1tempEROM, Ip2tempEROM, Ip3tempEROM, Ip4tempEROM;
int Gateway1EROM, Gateway2EROM, Gateway3EROM, Gateway4EROM;
int Subnet1EROM, Subnet2EROM, Subnet3EROM, Subnet4EROM;

//EEPROM.write(addr, value);
//EEPROM.read(addr);

void setup()
{
  EEROM_Read();
  lcd1.begin(16, 2);
  lcd2.begin(16, 2);
  lcd3.begin(16, 2);
  // NOTE: Cursor Position: (CHAR, LINE) start at 0
  lcd1.setCursor(0, 0);
  lcd1.print("NSRRC Position");
  lcd2.setCursor(0, 0);
  lcd2.print("Meter TCP MODBUS");
  lcd3.setCursor(0, 0);
  lcd3.print("SYSTEM KMC Corp.");
uint8_t mac []  = { mac1, mac2, mac3, mac4, mac5, mac6 };
uint8_t Ip[]      = { Ip1, Ip2, Ip3, Ip4 };
uint8_t Gateway[] = { Gateway1, Gateway2, Gateway3, Gateway4};
uint8_t Subnet[]  = { Subnet1, Subnet2, Subnet3, Subnet4};
  Ethernet.begin(mac, Ip, Gateway, Subnet);
  TCCR1A = 0x00;
  TCCR1B &= ~_BV(CS12);
  TCCR1B &= ~_BV(CS11);
  TCCR1B |= _BV(CS10);
  TIMSK1 |= _BV(TOIE1);
  TCNT1 = 0;
  Serial.begin(9600);
  for (int i = 1; i < 13; i++) 
  {
    pinMode(Ai[i], INPUT);
  }
  pinMode(7, INPUT);
  pinMode(8, OUTPUT);

  lcdclearall();

  Mb.Run();
  
  Mb.R[mapMAXaddM] = mapMAX;
  Mb.R[mapMINaddM] = mapMIN;
  Mb.R[RealMAXaddM] = RealMAX;
  Mb.R[RealMINaddM] = RealMIN;

  Mb.R[Mac1addM] = mac1;
  Mb.R[Mac2addM] = mac2;
  Mb.R[Mac3addM] = mac3;
  Mb.R[Mac4addM] = mac4;
  Mb.R[Mac5addM] = mac5;
  Mb.R[Mac6addM] = mac6;

  Mb.R[Ip1addM] = Ip1;
  Mb.R[Ip2addM] = Ip2;
  Mb.R[Ip3addM] = Ip3;
  Mb.R[Ip4addM] = Ip4;

  Mb.R[Gateway1addM] = Gateway1;
  Mb.R[Gateway2addM] = Gateway2;
  Mb.R[Gateway3addM] = Gateway3;
  Mb.R[Gateway4addM] = Gateway4;

  Mb.R[Subnet1addM] = Subnet1;
  Mb.R[Subnet2addM] = Subnet2;
  Mb.R[Subnet3addM] = Subnet3;
  Mb.R[Subnet4addM] = Subnet4;
}

void loop()
{
  Mb.Run();
  //Analog inputs 0-1023
  for (int i = 0; i < 13; i++) {  //Read Position Meter ChannelA-L 12Channel
    Mb.R[i] = analogRead(Ai[i]);
  }
  
ModbusWriteSetting();

  EEROM_Write();
  for (int i = 1; i < 13; i++) {
    float dataread = Mb.R[i];
    dataf[i] = mapfloat(dataread, RealMIN, RealMAX, mapMIN / 10, mapMAX / 10);
    Mb.R[i + 100] = dataf[i] * 10;
  }
  //float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
  int modeai = analogRead(modepin);

  if (modeai < 10) //680 510 0
    infmode();
  else if (modeai < 600)
    mappingmode();
  else
    realmode();
if(Mb.R[RemoteResetM]==ResetValue)
  resetFunc();

}

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void lcdprint(int lcdx, int line, int point, float data, int form)
{
  String Space;
  if (data >= 100)
    Space = "";
  else if (data >= 10)
    Space = " ";
  else if (data >= 1)
    Space = "  ";
  else
    Space = "  ";

  if (form == 0) {
    if (data > 0 && data < 1000)
      Space += " ";
    if (data == 0)
      Space = "   ";
  }

  switch (lcdx)
  {
    case 1:
      lcd1.setCursor(point, line);
      lcd1.print(Space);
      lcd1.print(data, form);
      break;

    case 2:
      lcd2.setCursor(point, line);
      lcd2.print(Space);
      lcd2.print(data, form);
      break;

    case 3:
      lcd3.setCursor(point, line);
      lcd3.print(Space);
      lcd3.print(data, form);
      break;
  }

}

void lcdclearall() {
  lcd1.clear();
  lcd2.clear();
  lcd3.clear();
}


void formprint() {
  lcd1.clear();
  lcd1.setCursor(0, 0);
  lcd1.print("A:      C:");
  lcd1.setCursor(0, 1);
  lcd1.print("B:      D:");
  lcd2.setCursor(0, 0);
  lcd2.print("E:      G:");
  lcd2.setCursor(0, 1);
  lcd2.print("F:      H:");
  lcd3.setCursor(0, 0);
  lcd3.print("I:      K:");
  lcd3.setCursor(0, 1);
  lcd3.print("J:      L:");
}
void realmode() {
  if (flag != 1) {

    lcdclearall();
    lcd1.setCursor(0, 0);
    lcd1.print("Real Value Mode:");
    delay(1000);
    lcdclearall();
    formprint();
    flag = 1;
  }
  
  int i = 0;
  lcdprint(1, 0, 2, Mb.R[1 + i], 0);
  lcdprint(1, 1, 2, Mb.R[2 + i], 0);
  lcdprint(1, 0, 10, Mb.R[3 + i], 0);
  lcdprint(1, 8, 10, Mb.R[4 + i], 0);
  lcdprint(2, 0, 2, Mb.R[5 + i], 0);
  lcdprint(2, 1, 2, Mb.R[6 + i], 0);
  lcdprint(2, 0, 10, Mb.R[7 + i], 0);
  lcdprint(2, 8, 10, Mb.R[8 + i], 0);
  lcdprint(3, 0, 2, Mb.R[9 + i], 0);
  lcdprint(3, 1, 2, Mb.R[10 + i], 0);
  lcdprint(3, 0, 10, Mb.R[11 + i], 0);
  lcdprint(3, 8, 10, Mb.R[12 + i], 0);
  
  working();
  //void lcdprint(int lcdx,int line,int point,float data)

}



void mappingmode() {
  if (flag != 2) {
    lcdclearall();
    lcd1.setCursor(0, 0);
    lcd1.print("Mapping Mode:");

    //lcd1.setCursor(0,0);
    //lcd1.print("Modbus Mapping:");
    //lcd1.setCursor(0,1);
    //lcd1.print("CovIN=10 OUT=1/10");
    //  lcd2.setCursor(0,0);
    //  lcd2.print("MaxValAdd:400400");


    lcd2.setCursor(0, 0);
    lcd2.print("RealMAX:");
    lcd2.print(RealMAX, 0);
    lcd2.setCursor(0, 1);
    lcd2.print("MapMAX:");
    lcd2.print(mapMAX, 0);

    //lcd3.setCursor(0,0);
    //lcd3.print("MinValAdd:400401");

    lcd3.setCursor(0, 0);
    lcd3.print("RealMIN:");
    lcd3.print(RealMIN, 0);
    lcd3.setCursor(0, 1);
    lcd3.print("MapMIN:");
    lcd3.print(mapMIN, 0);

    delay(2000);
    formprint();
    flag = 2;

  }
  int i = 100;
  lcdprint(1, 0, 2, dataf[1], 1);
  lcdprint(1, 1, 2, dataf[2], 1);
  lcdprint(1, 0, 10, dataf[3], 1);
  lcdprint(1, 8, 10, dataf[4], 1);
  lcdprint(2, 0, 2, dataf[5], 1);
  lcdprint(2, 1, 2, dataf[6], 1);
  lcdprint(2, 0, 10, dataf[7], 1);
  lcdprint(2, 8, 10, dataf[8], 1);
  lcdprint(3, 0, 2, dataf[9], 1);
  lcdprint(3, 1, 2, dataf[10], 1);
  lcdprint(3, 0, 10, dataf[11], 1);
  lcdprint(3, 8, 10, dataf[12], 1);
  working();
}

void infmode() {

  if (flag != 3) {
    lcdclearall();
    lcd1.setCursor(0, 0);
    lcd1.print("Information Mode:");
    delay(1000);
    lcdclearall();
  }

  lcd1.setCursor(0, 0);
  lcd1.print("FE-01-A1 Station");
  lcd1.setCursor(0, 1);
  lcd1.print("I");
  lcd1.print(Ethernet.localIP());

  lcd2.setCursor(0, 0);
  lcd2.print("MAC:");
  lcd2.print(mac1, HEX);
  lcd2.print("-");
  lcd2.print(mac2, HEX);
  lcd2.print("-");
  lcd2.print(mac3, HEX);
  lcd2.print("-");
  lcd2.print(mac4, HEX);
  lcd2.print("-");
  lcd3.setCursor(0, 0);
  lcd3.print(mac5, HEX);
  lcd3.print("-");
  lcd3.print(mac6, HEX);
  lcd3.setCursor(7, 0);
  lcd3.print("KMC Corp.");


  lcd2.setCursor(0, 1);
  lcd2.print("G");
  lcd2.print(Gateway1);
  lcd2.print(".");
  lcd2.print(Gateway2);
  lcd2.print(".");
  lcd2.print(Gateway3);
  lcd2.print(".");
  lcd2.print(Gateway4);

  lcd3.setCursor(0, 1);
  lcd3.print("S");
  lcd3.print(Subnet1);
  lcd3.print(".");
  lcd3.print(Subnet2);
  lcd3.print(".");
  lcd3.print(Subnet3);
  lcd3.print(".");
  lcd3.print(Subnet4);
  flag = 3;
if(count%100==0)
    lcdclearall();
    
}

void working() {
  char FF = 0xFF;
  if (sec % 2 == 0)
    FF = 0xFF;
  else
    FF = 0x20;
  lcd1.setCursor(7, 0);
  lcd1.write(FF);
  lcd2.setCursor(7, 0);
  lcd2.write(FF);
  lcd3.setCursor(7, 0);
  lcd3.write(FF);
  lcd1.setCursor(15, 0);
  lcd1.write(FF);
  lcd2.setCursor(15, 0);
  lcd2.write(FF);
  lcd3.setCursor(15, 0);
  lcd3.write(FF);
  lcd1.setCursor(7, 1);
  lcd1.write(FF);
  lcd2.setCursor(7, 1);
  lcd2.write(FF);
  lcd3.setCursor(7, 1);
  lcd3.write(FF);
  lcd1.setCursor(15, 1);
  lcd1.write(FF);
  lcd2.setCursor(15, 1);
  lcd2.write(FF);
  lcd3.setCursor(15, 1);
  lcd3.write(FF);

}



ISR (TIMER1_OVF_vect)
{
  count++;
  if (count == 244) {             // overflow frequency = 16 MHz/65536 = 244Hz
    halfsec++;
    PORTB ^= _BV(5);              // Toggle LED, PB5 = Arduino pin 13
    count = 0;
  }
  if (halfsec == 2) {
    if (sec > 1000) {
      sec = 0;
    }
    else {
      sec++;
      halfsec = 1;
    }
  }
}

void EEROM_Read() {
  mapMAX = EEPROM.read(MapMAXaddr) * 256 + EEPROM.read(MapMAXaddr + 1);
  mapMIN = EEPROM.read(MapMINaddr) * 256 + EEPROM.read(MapMINaddr + 1);
  RealMAX = EEPROM.read(RealMAXaddr) * 256 + EEPROM.read(RealMAXaddr + 1);
  RealMIN = EEPROM.read(RealMINaddr) * 256 + EEPROM.read(RealMINaddr + 1);
  mapMAXtempEROM = mapMAX;
  mapMINtempEROM = mapMIN;
  RealMAXtempEROM = RealMAX;
  RealMINtempEROM = RealMIN;

  mac1 = EEPROM.read(Mac1addr);
  mac2 = EEPROM.read(Mac2addr);
  mac3 = EEPROM.read(Mac3addr);
  mac4 = EEPROM.read(Mac4addr);
  mac5 = EEPROM.read(Mac5addr);
  mac6 = EEPROM.read(Mac6addr);
  Mac1tempEROM = mac1;
  Mac2tempEROM = mac2;
  Mac3tempEROM = mac3;
  Mac4tempEROM = mac4;
  Mac5tempEROM = mac5;
  Mac6tempEROM = mac6;
  Ip1 = EEPROM.read(Ip1addr);
  Ip2 = EEPROM.read(Ip2addr);
  Ip3 = EEPROM.read(Ip3addr);
  Ip4 = EEPROM.read(Ip4addr);
  Ip1tempEROM = Ip1;
  Ip2tempEROM = Ip2;
  Ip3tempEROM = Ip3;
  Ip4tempEROM = Ip4;
  Gateway1 = EEPROM.read(Gateway1addr);
  Gateway2 = EEPROM.read(Gateway2addr);
  Gateway3 = EEPROM.read(Gateway3addr);
  Gateway4 = EEPROM.read(Gateway4addr);
  Gateway1EROM = Gateway1;
  Gateway2EROM = Gateway2;
  Gateway3EROM = Gateway3;
  Gateway4EROM = Gateway4;
  Subnet1 = EEPROM.read(Subnet1addr);
  Subnet2 = EEPROM.read(Subnet2addr);
  Subnet3 = EEPROM.read(Subnet3addr);
  Subnet4 = EEPROM.read(Subnet4addr);
  Subnet1EROM = Subnet1;
  Subnet2EROM = Subnet2;
  Subnet3EROM = Subnet3;
  Subnet4EROM = Subnet4;

}

void EEROM_Write() {
        if (mapMAXtempEROM != mapMAX) {
          mapMAXtempEROM = mapMAX;
          EEPROM.write(MapMAXaddr, mapMAXtempEROM / 256);
          EEPROM.write(MapMAXaddr + 1, mapMAXtempEROM % 256);
        }
      
        if (mapMINtempEROM != mapMIN) {
          mapMINtempEROM = mapMIN;
          EEPROM.write(MapMINaddr, mapMINtempEROM / 256);
          EEPROM.write(MapMINaddr + 1, mapMINtempEROM % 256);
        }
      
        if (RealMAXtempEROM != RealMAX) {
          RealMAXtempEROM = RealMAX;
          EEPROM.write(RealMAXaddr, RealMAXtempEROM / 256);
          EEPROM.write(RealMAXaddr + 1, RealMAXtempEROM % 256);
        }
      
        if (RealMINtempEROM != RealMIN) {
          RealMINtempEROM = RealMIN;
          EEPROM.write(RealMINaddr, RealMINtempEROM / 256);
          EEPROM.write(RealMINaddr + 1, RealMINtempEROM % 256);
        }
      
        if (Mac1tempEROM != mac1) {
          Mac1tempEROM = mac1;
          EEPROM.write(Mac1addr, mac1);
        }
        if (Mac2tempEROM != mac2) {
          Mac2tempEROM = mac2;
          EEPROM.write(Mac2addr, mac2);
        }
        if (Mac3tempEROM != mac3) {
          Mac3tempEROM = mac3;
          EEPROM.write(Mac3addr, mac3);
        }
        if (Mac4tempEROM != mac4) {
          Mac4tempEROM = mac4;
          EEPROM.write(Mac4addr, mac4);
        }
        if (Mac5tempEROM != mac5) {
          Mac5tempEROM = mac5;
          EEPROM.write(Mac5addr, mac5);
        }
        if (Mac6tempEROM != mac6) {
          Mac6tempEROM = mac6;
          EEPROM.write(Mac6addr, mac6);
        }
        if (Ip1tempEROM != Ip1) {
          Ip1tempEROM = Ip1;
          EEPROM.write(Ip1addr, Ip1);
        }
        if (Ip2tempEROM != Ip2) {
          Ip2tempEROM = Ip2;
          EEPROM.write(Ip2addr, Ip2);
        }
        if (Ip3tempEROM != Ip3) {
          Ip3tempEROM = Ip3;
          EEPROM.write(Ip3addr, Ip3);
        }
        if (Ip4tempEROM != Ip4) {
          Ip4tempEROM = Ip4;
          EEPROM.write(Ip4addr, Ip4);
        }
      
        if (Gateway1EROM != Gateway1) {
          Gateway1EROM = Gateway1;
          EEPROM.write(Gateway1addr, Gateway1);
        }
        if (Gateway2EROM != Gateway2) {
          Gateway2EROM = Gateway2;
          EEPROM.write(Gateway2addr, Gateway2);
        }
        if (Gateway3EROM != Gateway3) {
          Gateway3EROM = Gateway3;
          EEPROM.write(Gateway3addr, Gateway3);
        }
        if (Gateway4EROM != Gateway4) {
          Gateway4EROM = Gateway4;
          EEPROM.write(Gateway4addr, Gateway4);
        }
        if (Subnet1EROM != Subnet1) {
          Subnet1EROM = Subnet1;
          EEPROM.write(Subnet1addr, Subnet1);
        }    if (Subnet2EROM != Subnet2) {
          Subnet2EROM = Subnet2;
          EEPROM.write(Subnet2addr, Subnet2);
        }    if (Subnet3EROM != Subnet3) {
          Subnet3EROM = Subnet3;
          EEPROM.write(Subnet3addr, Subnet3);
        }    if (Subnet4EROM != Subnet4) {
          Subnet4EROM = Subnet4;
          EEPROM.write(Subnet4addr, Subnet4);
        }

}

void ModbusWriteSetting(){

  mapMAX = Mb.R[mapMAXaddM];
  mapMIN = Mb.R[mapMINaddM];
  RealMAX = Mb.R[RealMAXaddM];
  RealMIN = Mb.R[RealMINaddM];

  mac1 = Mb.R[Mac1addM];
  mac2 = Mb.R[Mac2addM] ;
  mac3  = Mb.R[Mac3addM];
  mac4 = Mb.R[Mac4addM];
  mac5 = Mb.R[Mac5addM] ;
  mac6 = Mb.R[Mac6addM] ;

  Ip1 =  Mb.R[Ip1addM] ;
  Ip2 = Mb.R[Ip2addM];
  Ip3 = Mb.R[Ip3addM] ;
  Ip4  = Mb.R[Ip4addM] ;

  Gateway1 = Mb.R[Gateway1addM];
  Gateway2 = Mb.R[Gateway2addM];
  Gateway3 = Mb.R[Gateway3addM];
  Gateway4 = Mb.R[Gateway4addM];

  Subnet1 = Mb.R[Subnet1addM] ;
  Subnet2 = Mb.R[Subnet2addM] ;
  Subnet3 = Mb.R[Subnet3addM] ;
  Subnet4 = Mb.R[Subnet4addM] ;
}
