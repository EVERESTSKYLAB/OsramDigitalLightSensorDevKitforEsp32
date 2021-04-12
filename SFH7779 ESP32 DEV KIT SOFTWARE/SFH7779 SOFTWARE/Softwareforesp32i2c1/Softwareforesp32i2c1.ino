#include <Wire.h> // The library needed to run I2C commands
#define ALS_I2C_ADRESS 0x39     // defines address of SFH 7779, SFH 7779 operates in slave mode. Slave address is 0111001 (0x39h)
#define I2CSpeed 400000L        // defines i2c speed to 4000khz
// hardware specific defines for I2C initialization
const int SCL_SENS_pin1 = 33;    // declares pin 33 as Clock pin
const int SDA_SENS_pin1 = 32;    // declares pin 32 as Data pin
const int INT_Pin = 14;    
// hardware specific I2C initialisation
TwoWire I2C_to_Sens = TwoWire(0);

byte Prox_M ;
byte Prox_l;
byte Vis_M;
byte Vis_l;
byte IR_M;
byte IR_l;

int Vis_Val;
int IR_Val;
int Prox_Val;
// -------------- Main programs -----------------------


void Read_Sensors() 
{
  delay(100);
 
   Prox_M = 0;
   Prox_l = 0;
   Vis_M = 0;
   Vis_l = 0;;
   IR_M = 0;
   IR_l = 0;   
  
   Wire.beginTransmission(0x39);
   Wire.write(0x44); 
   Wire.endTransmission(); 
   Wire.requestFrom(0x39,6);
   Prox_M = Wire.read();
   Prox_l = Wire.read();
   Vis_l = Wire.read();
   Vis_M = Wire.read();
   IR_l = Wire.read();
   IR_M = Wire.read();    

 /* uint8_t iterator = 0;
  uint8_t * read_buffer;
  I2C_to_Sens.beginTransmission(0x39);
  I2C_to_Sens.write(0x44);
  I2C_to_Sens.endTransmission();
  I2C_to_Sens.requestFrom(0x39,6);
  while (iterator  < 0X39 && I2C_to_Sens.available()) {
    read_buffer[iterator] = I2C_to_Sens.read();
    iterator ++;
    }
  //return iterator;

 */
   Serial.print ("   Prox=  ");
   Prox_l = 256*(128*bitRead(Prox_l,7)+ 64*bitRead(Prox_l,6)+ 32*bitRead(Prox_l,5)+ 16*bitRead(Prox_l,4)+ 8*bitRead(Prox_l,3)+ 4*bitRead(Prox_l,2) + 2*bitRead(Prox_l,1)+ 1*bitRead(Prox_l,0));
   Serial.print(Prox_M);
   Serial.print(Prox_l);
   Prox_Val = Prox_l + 256*Prox_M;//PS = (PS + Content* 256); // combining low+high byte to decimal value
   
 
   Vis_Val =(32768*bitRead(Vis_M,7) + 16386*bitRead(Vis_M,6) + 8192*bitRead(Vis_M,5) + 4096*bitRead(Vis_M,4) + 2048*bitRead(Vis_M,3) + 1024*bitRead(Vis_M,2) + 512*bitRead(Vis_M,1) + 256*(256*bitRead(Vis_M,0)) + 128*bitRead(Vis_l,7)+ 64*bitRead(Vis_l,6)+ 32*bitRead(Vis_l,5)+ 16*bitRead(Vis_l,4)+ 8*bitRead(Vis_l,3)+ 4*bitRead(Vis_l,2) + 2*bitRead(Vis_l,1)+ 1*bitRead(Vis_l,0));
   Serial.print ("   Vis=  ");
   Serial.print(Vis_Val);

   IR_Val = 32768*bitRead(IR_M,7) + 16386*bitRead(IR_M,6) + 8192*bitRead(IR_M,5) + 4096*bitRead(IR_M,4) + 2048*bitRead(IR_M,3) + 1024*bitRead(IR_M,2) + 256*(512*bitRead(IR_M,1) + 256*bitRead(IR_M,0) + 128*bitRead(IR_l,7)+ 64*bitRead(IR_l,6)+ 32*bitRead(IR_l,5)+ 16*bitRead(IR_l,4)+ 8*bitRead(IR_l,3)+ 4*bitRead(IR_l,2) + 2*bitRead(IR_l,1)+ 1*bitRead(IR_l,0));
   Serial.print ("   IR=  ");
   Serial.print(IR_Val);

 //LUX CALCULATION
// Lux Calculation based on ALS Gain =128 and VIS_Int_Time and ALS_Int_Time = 100 ms
// Lux value in front of sensor, no cover glass
    int GAIN_VIS=128;
    int GAIN_IR=128;
    int t_INT_ALS=100;
    float LUX;
    float LUX1;
    float REF;
    REF=IR_Val/Vis_Val;
    Serial.print("\t\t");
    
    if (REF<0.109){
    LUX = (1.534 * Vis_Val/128 - 3.759 * IR_Val/128);
    Serial.print(LUX);
    //Serial.print("   LUXES");
    }
    else if (REF<0.429){
    LUX = (1.339 * Vis_Val/128 - 1.972 * IR_Val/128);
    Serial.print(LUX);
    //Serial.print("   LUXES");
    }
    else if (REF<1.378){
    LUX = (0.701 * Vis_Val/128 - 0.483 * IR_Val/128);
    Serial.print(LUX);
    //Serial.print("   LUXES");
    }
    else if (REF<2.175){
    LUX = (1.402 * Vis_Val/128 - 0.57 * IR_Val/128);
    Serial.print(LUX);
    //Serial.print("   LUXES");
    }
    else if (REF<3.625){
    LUX = (0.284 * Vis_Val/128 - 0.6424 * IR_Val/128);
    Serial.print(LUX);
    //Serial.print("   LUXES");
    }
    else{
    LUX = 5.608 * (Vis_Val/128);
    }
    LUX = LUX * (100/t_INT_ALS); 
    Serial.print(LUX);
    Serial.print("   LUXES");
    Serial.println(); 
}
void setup()
{
  Wire.begin(SDA_SENS_pin1, SCL_SENS_pin1);
  Wire.begin();
  Wire.setClock(I2CSpeed);
  Serial.begin(115200);
  Wire.beginTransmission(0x39); //Begin sending to the device with address ADD (defined above)
  Wire.write(0x42); 
  Wire.write(0x03); 
  Wire.endTransmission(); 
  Wire.beginTransmission(0x39);
  Wire.write(0x41); 
  Wire.write(0x06);  
  Wire.endTransmission();

}

void loop()   
{
   Read_Sensors();

}
  
  
