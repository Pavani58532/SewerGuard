#define BLYNK_TEMPLATE_ID "TMPL3eBStZBML"
#define BLYNK_TEMPLATE_NAME "sewer guard"
#define BLYNK_AUTH_TOKEN "jv4ehZ6NaHLE9xhsKhusNg-QM7TQN7oc"
#define BLYNK_PRINT Serial
#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>

#define  trig  D4
#define  echo  D5
const int flowPin = D2;
volatile int pulseCount; 
float flowRate;  
unsigned long oldTime; 
float calibrationFactor = 4.5; 
void ICACHE_RAM_ATTR pulseCounter();

#define ADC_VREF_mV    3300.0 // in millivolt
#define ADC_RESOLUTION 1024.0
#define PIN_LM35       A0 // The ESP8266 pin ADC0 connected to LM35

const int mq2Pin = A0; 


long duration;
int distance;

// You should get Auth Token in the Blynk App.

char auth[] ="jv4ehZ6NaHLE9xhsKhusNg-QM7TQN7oc";

// Your WiFi credentials.
// Set password to "" for open networks.
char ssid[] = " ";//id
char pass[] = " ";//password

BlynkTimer timer;

void setup()
{
  // Debug console
  pinMode(trig, OUTPUT);  // Sets the trigPin as an Output
  pinMode(echo, INPUT);   // Sets the echoPin as an Input
  pinMode(flowPin, INPUT_PULLUP);

  pulseCount = 0;
  flowRate = 0.0;
  oldTime = 0;
  attachInterrupt(digitalPinToInterrupt(flowPin), pulseCounter, FALLING);


  Serial.begin(9600);
  Blynk.begin(auth, ssid, pass);

  // Setup a function to be called every second
  timer.setInterval(1000L, sendSensor);
}

void loop()
{
  Blynk.run();
  timer.run();
}
void sendSensor()
{
  digitalWrite(trig, LOW);   // Makes trigPin low
  delayMicroseconds(2);       // 2 micro second delay

  digitalWrite(trig, HIGH);  // tigPin high
  delayMicroseconds(10);      // trigPin high for 10 micro seconds
  digitalWrite(trig, LOW);   // trigPin low

  duration = pulseIn(echo, HIGH);   //Read echo pin, time in microseconds
  distance = duration * 0.034 / 2;   //Calculating actual/real distance
  
  Serial.print("Distance = ");        //Output distance on arduino serial monitor
  Serial.println(distance);



  unsigned long currentTime = millis();
  unsigned long elapsedTime = currentTime - oldTime;
oldTime = currentTime;
flowRate = ((1000.0 / elapsedTime) * pulseCount) / calibrationFactor;
pulseCount = 0;
Serial.print("Debit Air: ");
Serial.print(flowRate);
Serial.println(" L/min");

 Blynk.virtualWrite(V0, distance);
 Blynk.virtualWrite(V1, flowRate);
 
if((distance>0 && distance<32) && (flowRate>0.1 && flowRate<1.5)){
    Serial.println("Blockage Detected");
    Blynk.logEvent("Block_detection","ultrasonic below 30 and flowrate below 1.5");
  }

  int mq2Value = analogRead(mq2Pin);
  Serial.print("MQ2 Sensor Value: ");
  Serial.println(mq2Value);
   Blynk.virtualWrite(V3,mq2Value );
  if(mq2Value>10000){
    Serial.println("GAS ALERT");
    Blynk.logEvent("Gas_alert","gas level exceeded above 10000ppm");
  }

int adcVal = analogRead(PIN_LM35);
  // convert the ADC value to voltage in millivolt
  float milliVolt = adcVal * (ADC_VREF_mV / ADC_RESOLUTION);
  // convert the voltage to the temperature in °C
  float temperature_C = milliVolt/10;
 
  // print the temperature in the Serial Monitor:
  Serial.print("Temperature: ");
  Serial.print(temperature_C);   // print the temperature in °C
  Serial.print("°C");
  Blynk.virtualWrite(V2,temperature_C );
  if(temperature_C<10||temperature_C>21){
    Serial.println("TEMPERATURE ALERT");
    Blynk.logEvent("Temp_alert","Temperature not suitable");
  }

  delay(3000);
}
void pulseCounter(){
pulseCount++;}
