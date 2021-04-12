#include "Wire.h" // The library needed to run I2C commands


// defines for the SFH7779 I2C adress and interrupt pin number
#define ALS_I2C_ADRESS 0x39 // address of SFH 7779
#define INT_Pin 14

// hardware specific defines for I2C initialisation
#define SCL_SENS_Pin 33
#define SDA_SENS_Pin 32

#define I2CSpeed 400000L

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

const int LEDA_7779_Pin = 19;
const int ledPin_INT1_7779 = 14;

int ledState_LED_A_7779 = HIGH; 
int ledState_INT1_7779 = HIGH; 
byte val=0x44;

//-- Main programs -----------------------


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
/*
   val++;
   if(val== 0x49)
   {
    val=0x44;
   }
    Serial.print(val);
    }
  */ 
  
   Wire.requestFrom(0x39,6);

   while (Wire.available()){
      char c = Wire.read();
      Serial.print(c);
   } 
   

   
   Prox_M = Wire.read();
   Prox_l = Wire.read();
   Vis_l = Wire.read();
   Vis_M = Wire.read();
   IR_l = Wire.read();
   IR_M = Wire.read();    
  
   Wire.endTransmission();  

 
   Serial.print ("   Prox=  ");
   Serial.print(Prox_M);
   Serial.print(Prox_l);
 
   Vis_Val = 32768*bitRead(Vis_M,7) + 16386*bitRead(Vis_M,6) + 8192*bitRead(Vis_M,5) + 4096*bitRead(Vis_M,4) + 2048*bitRead(Vis_M,3) + 1024*bitRead(Vis_M,2) + 512*bitRead(Vis_M,1) + 256*bitRead(Vis_M,0) + 128*bitRead(Vis_l,7)+ 64*bitRead(Vis_l,6)+ 32*bitRead(Vis_l,5)+ 16*bitRead(Vis_l,4)+ 8*bitRead(Vis_l,3)+ 4*bitRead(Vis_l,2) + 2*bitRead(Vis_l,1)+ 1*bitRead(Vis_l,0);
   Serial.print ("   Vis=  ");
   Serial.print(Vis_Val);

   IR_Val = 32768*bitRead(IR_M,7) + 16386*bitRead(IR_M,6) + 8192*bitRead(IR_M,5) + 4096*bitRead(IR_M,4) + 2048*bitRead(IR_M,3) + 1024*bitRead(IR_M,2) + 512*bitRead(IR_M,1) + 256*bitRead(IR_M,0) + 128*bitRead(IR_l,7)+ 64*bitRead(IR_l,6)+ 32*bitRead(IR_l,5)+ 16*bitRead(IR_l,4)+ 8*bitRead(IR_l,3)+ 4*bitRead(IR_l,2) + 2*bitRead(IR_l,1)+ 1*bitRead(IR_l,0);
   Serial.print ("   IR=  ");
   Serial.print(IR_Val);

 
Serial.println(); 


/*  
 *   
 *0x44 read data read LSB of proximity measurement data
  0x45 read data read MSB of proximity measurement data
  0x46 read data read LSB of ambient light measurement of VIS diode
  0x47 read data read MSB of ambient light measurement of VIS diode
  0x48 read data read LSB of ambient light measurement of IR diode
  0x49 read data read MSB of ambient light measurement of IR diode
  Wire.endTransmission();
   */
}







void setup()
{
  Wire.begin();
  Wire.setClock(400000);
  Serial.begin(9600);
  
  pinMode(LEDA_7779_Pin, OUTPUT);
  pinMode(ledPin_INT1_7779, OUTPUT);
  
  
 //pinMode(LED_A_7779_PIN, OUTPUT);
 
  // initialize digital pin LED_BUILTIN to select multiplex Levels (6 total).

  
  
  Wire.beginTransmission(0x39); //Begin sending to the device with address ADD (defined above)
  Wire.write(0x42); 
  Wire.write(0x03F); 
  Wire.endTransmission(); 
  Wire.beginTransmission(0x39);
  Wire.write(0x41); 
  Wire.write(0x06);  
  Wire.endTransmission();

}



/*
 * 0x42 0x3F set LED pulse current to 200mA and ALS gain to x128
0x41 0x06 activate ALS & PS with a measurement repetition time of 100ms
Wait 100ms
0x44 read data read LSB of proximity measurement data
0x45 read data read MSB of proximity measurement data
0x46 read data read LSB of ambient light measurement of VIS diode
0x47 read data read MSB of ambient light measurement of VIS diode
0x48 read data read LSB of ambient light measurement of IR diode
0x49 read data read MSB of ambient light measurement of IR diode
 */

    
void loop()
{   

  sfh_address = 0x39; 
  ledState_LED_A_7779 = LOW;
  digitalWrite(LEDA_7779_Pin, ledState_LED_A_7779);
  ledState_INT1_7779 = LOW;
  digitalWrite(ledPin_INT1_7779, ledState_INT1_7779);

 
  Read_Sensors();

}
  
  
