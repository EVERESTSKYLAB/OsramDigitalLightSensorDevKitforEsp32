/*
  ESP32_BLINK_LED_001
  GOAL OF TH SKETCH: With this Sketch you will test all the leds visable on the BOARD
  DIGITAL LIGHT SENSOR SFH5721M SFH7779 AND PICCOLO
    Rewrite of classic Blink sketch for ESP32
  Using LED on pin 4,11,13,15,16,17,18,25
  Please add to Preferences:  Get into --> File --> Preferences
  Additional Boards Manager URLs: https://github.com/espressif/arduino-esp32.git
   
  ROBERTO GONZALEZ HUERTA
*/
 
//LED RGB up close to SFH5721
int LEDGI = 4;
int RED5721 = 16;
int LEDBI = 18;
//LED Blue Skylab Logo
int LEDBLI = 17;
//LED WHITE PICCOLO PWM/DAC
int LEDPICCOLOPWM = 25;
//LED RGB down close to SFH7779
int GREEN7779 = 27;
int RED7779 = 13;
int BLUE7779 = 12;


void setup()
{
   //LED RGB up close to SFH5721
    pinMode(LEDGI, OUTPUT);
    pinMode(RED5721, OUTPUT);
    pinMode(LEDBI, OUTPUT);
    //LED Blue Skylab Logo
    pinMode(LEDBLI, OUTPUT);
    //LED WHITE PICCOLO PWM/DAC
    pinMode(LEDPICCOLOPWM, OUTPUT);
    //LED RGB down close to SFH7779
    pinMode(GREEN7779, OUTPUT);
    pinMode(RED7779, OUTPUT);
    pinMode(BLUE7779, OUTPUT);
        
    // Serial monitor setup
    Serial.begin(115200);

}
 
void loop()
{
    Serial.print("RGB ON SFH5721");
    //LED RGB up close to SFH5721
    digitalWrite(LEDGI, LOW);
    digitalWrite(RED5721, LOW);
    digitalWrite(LEDBI, LOW);
    Serial.print("SKYLAB BLUE LOGO");
    //SKYLAB BLUE LOGO
    digitalWrite(LEDBLI, HIGH);
    Serial.print("LED WHITE ON PICCOLO SECTION");
    // WHITE LED PICCOLO
    digitalWrite(LEDPICCOLOPWM, LOW);
    Serial.print("RGB ON SFH7779");
    //LED RGB down close to SFH7779
    digitalWrite(GREEN7779, HIGH);
    digitalWrite(RED7779, LOW);
    digitalWrite(BLUE7779, HIGH);
          
    delay(500);
    
    Serial.print("RGB ON SFH5721");
    //LED RGB up close to SFH5721
    digitalWrite(LEDGI, HIGH);
    digitalWrite(RED5721, HIGH);
    digitalWrite(LEDBI, HIGH);
    Serial.print("SKYLAB BLUE LOGO");
    //BLUE LOGO
    digitalWrite(LEDBLI, HIGH);
    Serial.print("LED WHITE ON PICCOLO SECTION");
    // WHITE LED PICCOLO
    digitalWrite(LEDPICCOLOPWM, HIGH);
    //LED RGB down close to SFH7779
    digitalWrite(GREEN7779, HIGH);
    digitalWrite(RED7779, HIGH);
    digitalWrite(BLUE7779, HIGH);
    
     
    
    delay(500);
}
