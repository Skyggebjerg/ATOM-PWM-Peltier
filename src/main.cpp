#include <Arduino.h>
#include <M5Atom.h>
#include <M5_KMeter.h>

M5_KMeter sensor;

/*
* ESP32 PWM Control
*/
 
#define LED_GPIO   21 //32
#define PWM1_Ch    0
#define PWM1_Res   8
#define PWM1_Freq  1000

uint8_t addr1 = 0x60;

int PWM1_DutyCycle = 0;
int DIR = 25;

float temperature;
float temperature2;
long tidsbavl = millis();

void setup()
{
  pinMode(DIR, OUTPUT);
  
  ledcAttachPin(LED_GPIO, PWM1_Ch);
  ledcSetup(PWM1_Ch, PWM1_Freq, PWM1_Res);
  Wire.begin(26, 32, 400000L); //Wire.begin((int)SDA, (int)SCL, 400000L);
      // prepare sensor. ( default I2C addr : 0x66 )
    //sensor.begin();

        // change I2C addr.
    // ( When the unit is turned back on, it will operate with the new I2C address. )
  //sensor.changeAddr(0x60);
  //sensor.begin(&Wire, 0x66);
        // set sleep time. (5 second)
    //sensor.setSleepTime(5);
    Serial.begin(115200);

      ledcWrite(PWM1_Ch, 255);
     
}
 
void loop()
{

    // Sensor deep sleep.
    // (Number of seconds set by the setSleepTime function)
    //sensor.sleep();
  
    //delay(10);

    // data read from unit.
    sensor.begin(&Wire, 0x66); // blue plot - top sensor
    if (sensor.update()) {

        // get sensor temperature.
        temperature = sensor.getTemperature();

        // get unit internal temperature.
        //float internaltemp = sensor.getInternalTemp();
        //Serial.printf("%3.2f  /  %3.2f \n", temperature, temperature2);
        //Serial.printf("%3.2f  /  %3.2f \n", temperature, internaltemp);
       //if (temperature > 95) digitalWrite(DIR, HIGH);
       //       if (temperature < 95) digitalWrite(DIR, LOW);
       // if (temperature >= 80) ledcWrite(PWM1_Ch, 0);
       // if (temperature <= 40) ledcWrite(PWM1_Ch, 255);
    } else {
        Serial.printf("error %d \n", sensor.getError());
    }
//float temp1 = temperature; 
    sensor.begin(&Wire, 0x60); // red plot - bottom sensor
    if (sensor.update()) {

        // get sensor temperature.
        temperature2 = sensor.getTemperature();

        // get unit internal temperature.
        //float internaltemp = sensor.getInternalTemp();
      if (millis()>=tidsbavl+250){
        Serial.printf("%3.2f  /  %3.2f \n", temperature, temperature2);
        tidsbavl=millis();
    }
       if (temperature2 > 96.5) digitalWrite(DIR, HIGH);
              if (temperature2 < 96.5) digitalWrite(DIR, LOW);
       // if (temperature >= 80) ledcWrite(PWM1_Ch, 0);
       // if (temperature <= 40) ledcWrite(PWM1_Ch, 255);
    } else {
        Serial.printf("error %d \n", sensor.getError());
    }

  //while(PWM1_DutyCycle < 255)
  //{
  //  ledcWrite(PWM1_Ch, PWM1_DutyCycle++);
  //  delay(10);
  //}
  //delay(10000);
/*   while(PWM1_DutyCycle > 0)
  {
    ledcWrite(PWM1_Ch, PWM1_DutyCycle--);
    delay(20);
  }
  delay(1000); */
}