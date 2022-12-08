#include <Arduino.h>
#include <M5Atom.h>
#include <M5_KMeter.h>
#include <PID_v1.h>
#include <SimpleKalmanFilter.h>


#include <RunningMedian.h>

RunningMedian samples = RunningMedian(10);

//Define Variables we'll be connecting to
double Setpoint, Input, Output;
double CoolSetpoint;

//Specify the links and initial tuning parameters
// From 50 to 70: double Kp=120, Ki=9, Kd=2.25;
// From 70 to 90: double Kp=60, Ki=9, Kd=2.25;

double Kp=60, Ki=9, Kd=2.25; // 45 obs: 50-70: Kp=40, Ki=0.4, Kd=0;
double CoolKp=120, CoolKi=9, CoolKd=2.25;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT); //PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
PID coolPID(&Input, &Output, &CoolSetpoint, CoolKp, CoolKi, CoolKd, REVERSE);

M5_KMeter sensor;

/*
* ESP32 PWM Control
*/
 
#define LED_GPIO   21 //32
#define PWM1_Ch    0
#define PWM1_Res   8
#define PWM1_Freq  5

uint8_t addr1 = 0x60;

int PWM1_DutyCycle = 0;
int DIR = 25;
float estimated_value;
float temperature;
float temperature2;
long tidsbavl = millis();
SimpleKalmanFilter simpleKalmanFilter(2, 1, 1);
bool coagstarted = false;
long coagtimestamp;
bool coolstarted = false;
long cooltimestamp;
bool annealing = false;
bool elengation = false;


void setup()
{
  pinMode(DIR, OUTPUT);

  //Input = analogRead(PIN_INPUT);
  Setpoint = 95;
  CoolSetpoint = 50;

    myPID.SetMode(AUTOMATIC);

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
     digitalWrite(DIR, LOW);
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
 //estimated_value = simpleKalmanFilter.updateEstimate(temperature);
        // get unit internal temperature.
        //float internaltemp = sensor.getInternalTemp();
        //Serial.printf("%3.2f  /  %3.2f \n", temperature, temperature2);
        //Serial.printf("%3.2f  /  %3.2f \n", temperature, internaltemp);
       //if (temperature > 95) digitalWrite(DIR, HIGH);
       //       if (temperature < 95) digitalWrite(DIR, LOW);
       // if (temperature >= 80) ledcWrite(PWM1_Ch, 0);
       // if (temperature <= 40) ledcWrite(PWM1_Ch, 255);
    } /* else {
        Serial.printf("error %d \n", sensor.getError());
    } */
//float temp1 = temperature; 
    sensor.begin(&Wire, 0x60); // red plot - bottom sensor
    if (sensor.update()) {

        // get sensor temperature.
        temperature2 = sensor.getTemperature();

        // get unit internal temperature.
        //float internaltemp = sensor.getInternalTemp();
      //if (millis()>=tidsbavl+250){
      //  Serial.printf("%3.2f/%3.2f\n", temperature, temperature2); // blue (top), red (bottom)
      //  tidsbavl=millis();
      //}
       //Input = estimated_value;
       // Median and average calculation
       samples.add(temperature2); // add both bottom and lid temperatures
       samples.add(temperature);
       //samples.add(temperature);
          float l = samples.getLowest();
          float m = samples.getMedian();
          float a = samples.getAverage();
          float h = samples.getHighest();
          Input = a; // Use average as input for PID
          myPID.Compute();
          Serial.printf("%3.2f/%3.2f/%3.2f\n", temperature, temperature2, a); // print raw and runningmedian
       //Input = temperature2;
/*        if (coolstarted) {
        coolPID.Compute();
       }
       else {
        myPID.Compute();
       }
         myPID.Compute();
        ledcWrite(PWM1_Ch, Output); // analogWrite(PIN_OUTPUT, Output);
        delay(100); */
        if (a >= 95 && coagstarted == false) {
          coagtimestamp = millis();
          coagstarted = true;
        }
        if (coagstarted) {
          if (millis() - coagtimestamp >=  30000) {
            coagstarted = false;
            coolstarted = true;
            cooltimestamp = millis();

          }
        }
        if (coolstarted) {
            if (a <= 50) {
            cooltimestamp = millis();
            annealing = true;
            coolstarted = false;
            digitalWrite(DIR, LOW);
            Setpoint = 50;
            myPID.Compute();
            //ledcWrite(PWM1_Ch, Output);  
            }
            else {
            digitalWrite(DIR, HIGH); // cooling started by reversing DIR 
            Output = 255;
            //ledcWrite(PWM1_Ch, 255); // full power on cooling              
            }
            //digitalWrite(DIR, HIGH); // cooling started by reversing DIR 
            //ledcWrite(PWM1_Ch, 255); // full power on cooling
            //delay(2000); // let cooling be effective before continue
            //coolPID.Compute();
        }
/*         else {
            myPID.Compute();
            ledcWrite(PWM1_Ch, Output); // analogWrite(PIN_OUTPUT, Output);
       } */
/*        if (coolstarted && a <= 50) {
        cooltimestamp = millis();
        annealing = true;
        coolstarted = false;
        digitalWrite(DIR, LOW);
        Setpoint = 50;
        myPID.Compute();
       } */

        ledcWrite(PWM1_Ch, Output); 
        delay(100);

        //Serial.println(Output);
       //if (temperature2 > 96.5) digitalWrite(DIR, HIGH);
         //     if (temperature2 < 96.5) digitalWrite(DIR, LOW);
       // if (temperature >= 80) ledcWrite(PWM1_Ch, 0);
       // if (temperature <= 40) ledcWrite(PWM1_Ch, 255);
    } /* else {
        Serial.printf("error %d \n", sensor.getError());
    } */

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