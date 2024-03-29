#include <Arduino.h>
#include <M5Atom.h>
#include <M5_KMeter.h>
#include <PID_v1.h>
#include <RunningMedian.h>

RunningMedian samples = RunningMedian(5);

//Define Variables we'll be connecting to
double Setpoint, Input, Output;
double CoolSetpoint;
double ElongSetpoint; 
double AnnealSetpoint;
double ScaledOutput;

int cycle;

//Specify the links and initial tuning parameters
// From 50 to 70: double Kp=120, Ki=9, Kd=2.25;
// From 70 to 90: double Kp=60, Ki=9, Kd=2.25;

double Kp=6, Ki=0.4, Kd=0.1; // i:0.4;*****45 obs: 6,0.4,0.1; ny: 4,1,0.1;
double CoolKp=6, CoolKi=0.4, CoolKd=0.1; //CoolKp=120, CoolKi=9, CoolKd=2.25;
double ElongKp=6, ElongKi=0.4, ElongKd=0.1; //ElongKp=120, ElongKi=9, ElongKd=2.25; 10,1,1
double AnnealgKp=1, AnnealKi=0, AnnealKd=0;

PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT); //PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
PID ElongPID(&Input, &Output, &ElongSetpoint, ElongKp, ElongKi, ElongKd, DIRECT);
PID coolPID(&Input, &Output, &CoolSetpoint, CoolKp, CoolKi, CoolKd, DIRECT); // ***************************************
PID AnnealPID(&Input, &Output, &AnnealSetpoint, AnnealgKp, AnnealKi, AnnealKd, DIRECT);

M5_KMeter sensor;
 
#define LED_GPIO   21 //32
#define PWM1_Ch    0
#define PWM1_Res   8
#define PWM1_Freq  5 // 5

uint8_t addr1 = 0x60;

int PWM1_DutyCycle = 0;
int DIR = 25;
float estimated_value;
float temperature;
float temperature2;
long tidsbavl = millis();
bool coagstarted = false;
long coagtimestamp;
long coagtime;
bool coolstarted = false;
long cooltimestamp;
bool annealing = false;
long annealtimestamp;
bool elongation = false;
bool elongationstart = false;
long elongtimestamp;
bool boilingstart = false;
int numcycles = 40; //Number of thermocycles. Should be 40
bool roomtemp = false;


void setup()
{
  pinMode(DIR, OUTPUT);

  Setpoint = 93; //95, tapesensor is 6 degress lower/watersensor 4 degrees lower
  CoolSetpoint = 60; //tapesensor OK/OK
  AnnealSetpoint = 60; //tapesensor OK/OK
  ElongSetpoint = 60; //70, tapesensor is 2 degrees lower/watersensor 1 degree lower

  myPID.SetMode(AUTOMATIC);
  coolPID.SetMode(AUTOMATIC);
  ElongPID.SetMode(AUTOMATIC);
  AnnealPID.SetMode(AUTOMATIC);

  ledcAttachPin(LED_GPIO, PWM1_Ch);
  ledcSetup(PWM1_Ch, PWM1_Freq, PWM1_Res);
  Wire.begin(26, 32, 400000L); //Wire.begin((int)SDA, (int)SCL, 400000L);
    Serial.begin(115200);

      ledcWrite(PWM1_Ch, 255);
     digitalWrite(DIR, LOW);

     boilingstart = true; //Start the loop once for test
     cycle = 1;
}
 
void loop()
{
// ************************************************************************ Cycle LOOP Start ***********************************************************************
    
    for (cycle = 1; cycle <= numcycles;) { 

    // data read from unit.
    //sensor.begin(&Wire, 0x66); // blue plot - top sensor
    //sensor.update();
        // get sensor temperature.
    //temperature = sensor.getTemperature();
    
    sensor.begin(&Wire, 0x66); // red plot - bottom sensor
    //sensor.changeAddr(0x66);
    sensor.update();
        // get sensor temperature.
    temperature2 = sensor.getTemperature();

       // Median and average calculation
       samples.add(temperature2); // add both bottom and lid temperatures
       //samples.add(temperature);
          float l = samples.getLowest();
          float m = samples.getMedian();
          float a = samples.getAverage();
          float h = samples.getHighest();
        //  Input = a; // Use average as input for PID
        //  myPID.Compute();
        //  Serial.printf("%3.2f/%3.2f\n", temperature2, a); // print raw and runningmedian

/*         if (a >= 95 && coagstarted == false) {
          coagtimestamp = millis();
          coagstarted = true;
        } */

          
          //boilingstart = true;

          

        if (boilingstart) {
            Setpoint = 94;  
            Input = a; // Use average as input for PID
            myPID.Compute();
          if (a >= 94) {
          boilingstart = false;
          coagtimestamp = millis();
          coagstarted = true;
        }
        }

        if (coagstarted) {
          Input = a;
          myPID.Compute();
          if(cycle == 0) coagtime = 180000; // long denaturing during first cycle: 300000 = 5 minutes ; 3 min
          if(cycle > 0) coagtime = 25000; // 30 seconds ; 15 secs; 25 secs
          if (millis() - coagtimestamp >=  coagtime) {
            coagstarted = false;
            coolstarted = true;
            //cooltimestamp = millis();

          }
        }

        if (coolstarted) {
            if (a <= 60) { // actually it is supposed to be 60, but we need to go lower because the temp sensor is too close to peltier
             //annealtimestamp = millis();
             elongtimestamp = millis();
             
             //annealing = true;
             coolstarted = false;
             elongation = true;
             //Output = 0;
             //digitalWrite(DIR, LOW); // normal mode

            }
            else {
             //digitalWrite(DIR, HIGH); // cooling mode started by reversing DIR 
            Setpoint = 60;
            //CoolSetpoint = 55; // actually it is supposed to be 60, but we need to go lower because the temp sensor is too close to peltier
            Input = a;
            myPID.Compute();
            //coolPID.Compute();
            // Output = 255;          
            }
        }

        if (annealing) {
            //Setpoint = 55;
            //if (millis() - annealtimestamp <  10000) {
              CoolSetpoint = 60; // actually it is supposed to be 60, but we need to go lower because the temp sensor is too close to peltier
              Input = a;
              coolPID.Compute();
            //}
            /* if (temperature2 > 55) {
              
              digitalWrite(DIR, HIGH); //cool down actively
              Output = 255;
            }
            if (temperature2 < 55) {
              digitalWrite(DIR, LOW);
              Output = 64;
            } */
            //myPID.Compute();
            
            /* if (millis() - annealtimestamp >=  10000) {
              AnnealSetpoint = 55;
              //Output = 0;
              digitalWrite(DIR, LOW); // normal mode
              AnnealPID.Compute();
            } */
            if (millis() - annealtimestamp >=  90000) {
             annealing = false;
             elongationstart = true;
             digitalWrite(DIR, LOW); //set direction to normal heating 
            }
        }

        if (elongationstart) {
          
            ElongPID.SetMode(AUTOMATIC); // reset parm?
            ElongSetpoint = 60;
            Input = a;
            ElongPID.Compute();
          if (a <= 55) {
            elongtimestamp = millis();
            elongation = true;
            elongationstart = false;
          }
        }

        if (elongation) {
          //digitalWrite(DIR, LOW); //set direction to normal heating
          //ElongPID.SetMode(AUTOMATIC);
          Setpoint = 60;
          //ElongSetpoint = 55;
          Input = a;
          myPID.Compute();
          // ElongPID.Compute();
          if(cycle == numcycles) {
            if (millis() - elongtimestamp >= 90000) {
              elongation = false;
              roomtemp = true;
            }
          }
          else {
            if (millis() - elongtimestamp >= 90000) { //120000
            elongation = false;
            boilingstart = true;
            cycle++; //end of cycle. Only restart new cycle if numcycles is not reached
            }
          }
        }
        
        if (roomtemp) { //run forever. The FOR Loop never ends because cycle is not increased
              //digitalWrite(DIR, HIGH); // cooling mode started by reversing DIR
              //CoolSetpoint = 21; // room temperature
              //Setpoint = 21;
              //Input = a;
              //myPID.Compute();
              //coolPID.Compute();
              Output = 127; // In principle the PWM will be zero and no heating or cooling will take place
        }

        if (Output >= 128)
          {
            digitalWrite(DIR, LOW); //heating
            ScaledOutput = (Output - 128) * 2;
          }
        else
          {
            digitalWrite(DIR, HIGH); //cooling
            ScaledOutput = (abs(Output - 127)) * 2;
          }

        ledcWrite(PWM1_Ch, ScaledOutput); 
        Serial.printf("%3.2f/%3.2f/%3.2f/%2d\n", temperature2, a, Output, cycle);
        delay(100);

    }
        // ************************************************************** Cycle LOOP END **********************************************************************************

 // do nothing

}