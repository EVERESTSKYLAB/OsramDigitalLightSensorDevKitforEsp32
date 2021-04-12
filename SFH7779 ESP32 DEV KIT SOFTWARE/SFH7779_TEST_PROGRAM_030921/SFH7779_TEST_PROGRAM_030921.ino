//EVEREST SKYLAB March 4th, 2021
//Roberto Gonzalez
//Code for Digital Light Sensor SFH7779 for EVALKIT ESP32 DEV KIT 4
//On the ESP32 DEV KIT C4 can find 2 ports i2c recommended, for SFH7779 will be used i2c port 1
/*
Basic Program:
Initialization data
Set up port pin, speed and clock
0x42 0x3F set LED pulse current to 200mA and ALS gain to x128             //SET PROXIMITY LED CURRENT TO 200MA AND ALS GAIN TO 128
0x41 0x06 activate ALS & PS with a measurement repetition time of 100ms   //ACTIVATE ALS (INT AND REP TIME) AND PS (REP TIME)
Read_Sensors()
{
Wait 100ms
0x44 read data read LSB of proximity measurement data
0x45 read data read MSB of proximity measurement data
Print Value of PS
0x46 read data read LSB of ambient light measurement of VIS diode
0x47 read data read MSB of ambient light measurement of VIS diode
Print Value of ALS VIS
0x48 read data read LSB of ambient light measurement of IR diode
0x49 read data read MSB of ambient light measurement of IR diode
Print Value of ALS IR
}
*/

#include "Wire.h"               // The library needed to run I2C commands
                                // defines for the SFH7779 I2C adress and interrupt pin number
#define ALS_I2C_ADRESS 0x39     // defines address of SFH 7779, SFH 7779 operates in slave mode. Slave address is 0111001 (0x39h)
#define I2CSpeed 400000L        // defines i2c speed to 4000khz
// hardware specific defines for I2C initialization
const int SCL_SENS_pin1 = 33;    // declares pin 33 as Clock pin
const int SDA_SENS_pin1 = 32;    // declares pin 32 as Data pin
const int INT_Pin = 14;          // declares the Pin 14 as an interrupt INT_Pin
byte Prox_MSB;        //  declares BYTE MSB of Proximity Measurement Data
byte Prox_LSB;        //  declares BYTE LSB of Proximity Measurement Data
byte Vis_MSB;         //  declares BYTE MSB of Ambient Light Measurement of VIS diode
byte Vis_LSB;         //  declares BYTE LSB of Ambient Light Measurement of VIS diode
byte IR_MSB;          //  declares BYTE MSB of Ambient light measurement of IR diode
byte IR_LSB;          //  declares BYTE LSB of Ambient light measurement of IR diode
int Prox_Value;       //  declares BYTE FOR Total VALUE OF Proximity Measurement Data
int Vis_Value;        //  declares BYTE FOR Total VALUE OF Ambient Light Measurement of VIS diode
int IR_Value;         //  declares BYTE FOR Total VALUE OF Ambient light measurement of IR diode

void Read_Sensors() 
{
   //Sensors in Operation
   delay(100); // Wait 100ms
   //Initialization of Proximity sensor, VIS ALS and IR ALS
   Prox_MSB = 0;      //  SET to "0" BYTE MSB of Proximity Measurement Data
   Prox_LSB = 0;      //  SET to "0" BYTE LSB of Proximity Measurement Data
   Vis_MSB = 0;       //  SET to "0" BYTE MSB of Ambient Light Measurement of VIS diode
   Vis_LSB = 0;       //  SET to "0" BYTE LSB of Ambient Light Measurement of VIS diode
   IR_MSB = 0;        //  SET to "0" BYTE MSB of Ambient light measurement of IR diode
   IR_LSB = 0;        //  SET to "0" BYTE LSB of Ambient light measurement of IR diode


   Wire.beginTransmission(0x39);    //Sensor address of  SFH7779 is 0x39, SFH 7779 operates in slave mode. Slave address is 0111001 (0x39h)
   Wire.write(0x44);                //write 0x44 to Proximity Sensor Data Register From Master to Slave 0x39
   Wire.endTransmission();          //End of transmission
   Wire.requestFrom(0x39,6);        //Request from address 0x39 next 6 bytes
   
   //The block read/write mode of the SFH 7779 can be used to read all output registers in cyclic manner.
   //The proximity sensor delivers output values within the range from 0 up to 4095 (12 bit, linear). The integrated proximity measurement operates at 940 nm
   
   //3. PS value: reading data (MSB and LSB): 
   //The two byte PS value is accessible via the output registers (0x44 (LSB) and
   //0x45 (MSB)). After reading the two 8-bit words, the communication can be ended by the master with a not acknowledge “NA” 
   //and the stop command “P”. The two byte PS output readings of the SFH 7779 can then be converted to a final decimal PS value via Eq. (4):
   //DATA16 bit, decimal=  DATA LSB + 256 DATA MSB             Eq.(4)
   
   Prox_LSB = Wire.read();            //PS_DATA_LSBs register (0x44h) LSB of the PS output measurement 
   //Serial.print(0x39, Prox_LSB);  
   Prox_MSB = Wire.read();            //PS_DATA_MSBs register (0x45h) MSB of the PS output measurement        
   //Serial.print(Prox_MSB); 
   //4. ALS value: reading data (MSB and LSB):
   //The sensor’s two 16 bit ALS measurement values are composed of 2 bytes each (LSB & MSB). The bytes are accessible via the two output registers
   //(0x46 to 0x49). After addressing the LSB (least significant byte) resp. the MSB (most significant byte) output register, the communication direction
   //has got to be changed from the slave to the master by repeating the address and the R/W byte with a changed R/W bit. After reading LSB and
   //MSB, the communication is ended by the master with a not acknowledge “NA” and the stop condition “P”. The conversion of the two byte output
   //data into 16 bit values can easily be done by again using Eq. (4). Finally the true lux value can be obtained from the two ALS data (ALS_VIS, ALS_IR) by
   //using the simple instructions according to Eq. (1). After finishing the measurement, the SFH 7779 mode may be changed to
   //STAND-BY via the mode_control register.
 
   Vis_LSB = Wire.read();             //ALS_VIS_DATA_LSBs register (0x46h) LSB of the ALS VIS output measurement 
   //Serial.print(Vis_l);
   Vis_MSB = Wire.read();             //ALS_VIS_DATA_MSBs register (0x47h) MSB of the ALS VIS output measurement 
   //Serial.print(Vis_M);
   IR_LSB = Wire.read();              //ALS_IR_DATA_LSBs register (0x48h) LSB of the ALS IR output measurement
   //Serial.print(IR_l);
   IR_MSB = Wire.read();              //ALS_IR_DATA_MSBs register (0x49h) MSB of the ALS IR output measurement
   //Serial.print(IR_M);
   Wire.endTransmission();  

   
   //LED drive current and timing during one proximity measurement cycle (two options are possible: normal and two-pulse mode)
   //Functionality of the PS. The SFH 7779 uses a single 200 μs LED pulse.
   //Measurement repetition time in the free running mode can be selected to be 10 ms, 50 ms,100 ms or 400 ms (register 0x41). Two options are available: normal mode with
   //single IR-LED pulse and two-pulse mode with two consecutive pulses where the persistence number increases twice as fast (see Section G, Registers). PS data,
   //PS related interrupt and persistence are updated after every pulse.
   
   //The proximity sensor delivers output values within the range from 0 up to 4095 (12 bit, linear). The integrated proximity measurement operates at 940 nm
   Prox_LSB = 128*bitRead(Prox_LSB,7)+ 64*bitRead(Prox_LSB,6)+ 32*bitRead(Prox_LSB,5)+ 16*bitRead(Prox_LSB,4)+ 8*bitRead(Prox_LSB,3)+ 4*bitRead(Prox_LSB,2) + 2*bitRead(Prox_LSB,1)+ 1*bitRead(Prox_LSB,0);
   Serial.print ("   Prox_LSB=  ");
   Serial.print(Prox_LSB);
   //The proximity sensor delivers output values within the range from 0 up to 4095 (12 bit, linear). The integrated proximity measurement operates at 940 nm
   Prox_MSB = 2048*bitRead(Prox_MSB,3) + 1024*bitRead(Prox_MSB,2) + 512*bitRead(Prox_MSB,1) + 256*bitRead(Prox_MSB,0);
   Serial.print ("   Prox_MSB=  ");
   Serial.print(Prox_MSB);
   //The proximity sensor delivers output values within the range from 0 up to 4095 (12 bit, linear). The integrated proximity measurement operates at 940 nm
   //Prox_Value = 2048*bitRead(Prox_MSB,3) + 1024*bitRead(Prox_MSB,2) + 512*bitRead(Prox_MSB,1) + 256*bitRead(Prox_MSB,0) + 128*bitRead(Prox_LSB,7)+ 64*bitRead(Prox_LSB,6)+ 32*bitRead(Prox_LSB,5)+ 16*bitRead(Prox_LSB,4)+ 8*bitRead(Prox_LSB,3)+ 4*bitRead(Prox_LSB,2) + 2*bitRead(Prox_LSB,1)+ 1*bitRead(Prox_LSB,0);
   Prox_Value = Prox_LSB + 256*Prox_MSB;//PS = (PS + Content* 256); // combining low+high byte to decimal value
   Serial.print ("   Prox_Val=  ");
   Serial.println(Prox_Value);
   
   //The two ambient light sensors deliver output values in the range from 0 to 65535 (16 bit).
   Vis_Value = 32768*bitRead(Vis_MSB,7) + 16386*bitRead(Vis_MSB,6) + 8192*bitRead(Vis_MSB,5) + 4096*bitRead(Vis_MSB,4) + 2048*bitRead(Vis_MSB,3) + 1024*bitRead(Vis_MSB,2) + 512*bitRead(Vis_MSB,1) + 256*bitRead(Vis_MSB,0) + 128*bitRead(Vis_LSB,7)+ 64*bitRead(Vis_LSB,6)+ 32*bitRead(Vis_LSB,5)+ 16*bitRead(Vis_LSB,4)+ 8*bitRead(Vis_LSB,3)+ 4*bitRead(Vis_LSB,2) + 2*bitRead(Vis_LSB,1)+ 1*bitRead(Vis_LSB,0);
   Serial.print ("   Vis=  ");
   Serial.print(Vis_Value);
   Serial.print(" Counts ");
   
   //The two ambient light sensors deliver output values in the range from 0 to 65535 (16 bit).
   IR_Value = 32768*bitRead(IR_MSB,7) + 16386*bitRead(IR_MSB,6) + 8192*bitRead(IR_MSB,5) + 4096*bitRead(IR_MSB,4) + 2048*bitRead(IR_MSB,3) + 1024*bitRead(IR_MSB,2) + 512*bitRead(IR_MSB,1) + 256*bitRead(IR_MSB,0) + 128*bitRead(IR_LSB,7)+ 64*bitRead(IR_LSB,6)+ 32*bitRead(IR_LSB,5)+ 16*bitRead(IR_LSB,4)+ 8*bitRead(IR_LSB,3)+ 4*bitRead(IR_LSB,2) + 2*bitRead(IR_LSB,1)+ 1*bitRead(IR_LSB,0);
   Serial.print ("   IR=  ");
   Serial.print(IR_Value);
   Serial.print(" Counts ");
   Serial.println(); 

//LUX CALCULATION
// Lux Calculation based on ALS Gain =128 and VIS_Int_Time and ALS_Int_Time = 100 ms
// Lux value in front of sensor, no cover glass
    int GAIN_VIS=128;
    int GAIN_IR=128;
    int t_INT_ALS=100;
    float LUX; 
    float REF;
    REF=IR_Value/Vis_Value;
    Serial.print("\t\t");
    if (REF<0.109){
    LUX = (1.534 * Vis_Value/128 - 3.759 * IR_Value/128);
    Serial.print(LUX);
    Serial.print("   LUXES");
    }
    else if (REF<0.429){
    LUX = (1.339 * Vis_Value/128 - 1.972 * IR_Value/128);
    Serial.print(LUX);
    Serial.print("   LUXES");
    }
    else if (REF<1.378){
    LUX = (0.701 * Vis_Value/128 - 0.483 * IR_Value/128);
    Serial.print(LUX);
    Serial.print("   LUXES");
    }
    else if (REF<2.175){
    LUX = (1.402 * Vis_Value/128 - 0.57 * IR_Value/128);
    Serial.print(LUX);
    Serial.print("   LUXES");
    }
    else if (REF<3.625){
    LUX = (0.284 * Vis_Value/128 - 0.6424 * IR_Value/128);
    Serial.print(LUX);
    Serial.print("   LUXES");
    }
    else{
    LUX = 5.608 * (Vis_Value/128);
    LUX = LUX * (100/t_INT_ALS); 
    Serial.print(LUX);
    Serial.print("   LUXES");
    }
       

/*  0x44 read data read LSB of proximity measurement data
  0x45 read data read MSB of proximity measurement data
  0x46 read data read LSB of ambient light measurement of VIS diode
  0x47 read data read MSB of ambient light measurement of VIS diode
  0x48 read data read LSB of ambient light measurement of IR diode
  0x49 read data read MSB of ambient light measurement of IR diode
  Wire.endTransmission();*/
Serial.println();
}

void setup()
{

  //Set up SDA and SCL ports, port speed and Clock
  //Interrupt pin (INT): open-drain output (like SDA and SCL)
  //SFH7779 Designed for the I2C Fast mode (400 kb/s), Standard mode (Sm) ≤ 100 kbit/s, Fast mode (Fm) ≤ 400 kbit/s 
  Serial.begin(9600);
  Serial.println("I2C Scanner");
  Serial.println("SDA Pin = "+String(SDA));
  Serial.println("SCL Pin = "+String(SCL));
  Wire.begin(SDA_SENS_pin1, SCL_SENS_pin1);
  //Wire.begin();
  Wire.setClock(100000);

                


//By embedding the SFH 7779 in an I²C-bus network and after applying VDD = 2.5 V, the communication can start with this I²C-bus conversation:
// 1. Activation of the ALS and PS:
//The default mode of the sensor is STAND-BY and the SFH 7779 needs to be activated by the master (e.g. microcontroller). Each I²C bus communication begins with a start command “S” of the 
//Master (SDA line is changing from “1” to “0” during SCL line stays “1”) followed by the address of the slave (0x39). After the 7 bit slave address the read (1) or write (0) R/W bit 
//of the master will follow. The R/W bit controls the communication direction between the master and the addressed slave. The slave is responding to a proper communication with
//an acknowledge command. Acknowledge “A” (or not acknowledge “NA”) is performed from the receiver by pulling the SDA line down (or leave in “1” state). For the activation of the sensor 
//the master needs to write an activation command (e.g. 0x09 to activate ALS and the PS with T_int_ALS = 100 ms and repetition time of 400 ms and 100 ms for the PS) into the corresponding 
//mode_control register (0x41). Each command needs to be acknowledged by the slave. After activation the master ends the communication with a STOP command “P” (SDA line is changing 
//from LOW to HIGH during SCL line stays HIGH). Additionally the ALS gain is set to 64 and PS current to 200 mA by writing 0x2B into the ALS_PS_Control register (0x42).

// SET ALS_PS_CONTROL REGISTER AND ACTIVATE ALS AND PS MODE_CONTROL REGISTER 
    //The maximum detection range depends — among others — on target properties like size and reflectivity and on the IR-LED pulse current. To reach a maximum
    //detection range the recommended value for the LED drive current is 200 mA.
    //As indicated, the typical maximum detection range for the SFH 7779 is in the range of beyond 100 mm (by using 200 mA LED current (Kodak White and setting a threshold
    //level for the interrupt alert at 7 counts).
    //E.g touching the sensor with a human finger produces enough PS counts (typ. hundreds of counts above any typical threshold setting at 200 mA IR-LED
    //current), making the sensor capable of handling touch events (see also Section I, Zero-distance detection).             
    //SET PROXIMITY LED CURRENT TO 200MA AND ALS GAIN TO 128
          //ALS_PS_CONTROL register (0x42h)
          //ALS and PS Control of set the PS output mode, the ALS gain and the LED current. In the „Infrared DC
          //level output“ PS mode (bit <6> = 1) the sensor measures the infrared DC ambient level. The proximity
          //value of the reflected signal is not available in this mode.
          //Set Proximity LED Current to 200mA (0x03) and ALS Gain to 128 (0xF) 
          Wire.beginTransmission(0x39);   //SFH7779 Address 0x39: Begins sending to the slave with address 0x39
          Wire.write(0x42);               //Write 0x42  to register  ALS_PS_Control  
          Wire.write(0x3F);               //Write 0x3F (command) to set LED pulse current to 200mA and ALS GAIN to x128//LED Current 1:0 11 11 200 mA//1111 ALS VIS: x128; ALS IR: x128
  
    //ACTIVATE ALS (INT AND REP TIME) AND PS (REP TIME)
          //MODE_CONTROL register (0x41h)
          //CONTROL of PS and ALS operating modes and time settings.
          //Repetition time is the time between two separate measurements. Integration time is the duration for one
          //measurement. ALS high sensitivity modes are 1010 and 1011 with an increased integration time of
          //400ms. In PS operating mode: „normal mode“ only one PS measurement is performed during one PS
          //repetition time. In PS operating mode „twice mode“ two independent PS measurement are performed
          //within one PS repetition time. Both measurements are independent and can trigger the interrupt. This
          //feature can be used to decrease the interrupt update time if the persistence function (register 0x43h) is
          //used.
          Wire.beginTransmission(0x39);     //SFH7779 Address 0x39: Begins sebding to slave with address 0x39 
          //0x41 0x06 activate: ALS with Repetition Time=100ms/Integration Time=100ms and PS  Repetition Time=100ms
          Wire.write(0x41);                 //Write 0x41 to Mode_Control Register. See Operating modes and time settings on the spec
          // write 0x06 to register to Mode_Control Register in order to Activate ALS (T_int_ALS=100ms, T_rep_ALS=400ms) and PS (T_rep_PS=100ms)
          Wire.write(0x06);                 //Write 0x06 (command) to Activate ALS & PS with a ALS interuption/repetition time=100ms and PS repetition time=100ms// 0 normal //0110 100 ms (= tint_ALS) 100 ms
          Wire.endTransmission();


//2. Sensor in operation:
//After activation, the sensor will change from STAND-BY to FREERUNNING mode. After a delay of e.g. 100 ms (depending on tINT_ALS
//setting) the first measurement values are available and can be read via the I²C-bus. 
}

    
void loop()   
{

//2. Sensor in operation:
//After activation, the sensor will change from STAND-BY to FREERUNNING mode. After a delay of e.g. 100 ms (depending on tINT_ALS
//setting) the first measurement values are available and can be read via the I²C-bus. 

    Read_Sensors();
  
}
  
  
