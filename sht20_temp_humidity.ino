#include <Wire.h>
#include "DFRobot_SHT20.h"
#include <NTPClient.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_TSL2561_U.h>
#include <TH02_dev.h>
#include "Arduino.h"
#include <WiFiUdp.h>
#include <Time.h>
#include <TimeLib.h>
#include <SPI.h>
#include <SD.h>
#include <DS1307RTC.h>
#include <TH02_dev.h>

#define TH02_EN 1 
const int pin_scl= 2;
const int pin_sda =3;
DFRobot_SHT20    sht20;
#define SensorPin A0            //pH meter Analog output to Arduino Analog Input 0
#define Offset 0.00            //deviation compensate
#define LED 13
#define samplingInterval 20
#define printInterval 800
#define ArrayLenth  40    //times of collection
int pHArray[ArrayLenth];   //Store the average value of the sensor feedback
int pHArrayIndex=0;
const int chipSelect = 53;
 int light;
 Sd2Card card;
Adafruit_TSL2561_Unified tsl = Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT, 12345);
WiFiUDP ntpUDP;
#include "RTClib.h"
RTC_DS1307 rtc;
File myFile;
int DATE_RTC,MONTH_RTC,YEAR_RTC,HOUR_RTC,MIN_RTC,SEC_RTC;
char daysOfTheWeek[7][12] = {"Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"};
// You can specify the time server pool and the offset (in seconds, can be
// changed later with setTimeOffset() ). Additionaly you can specify the
// update interval (in milliseconds, can be changed using setUpdateInterval() ).
NTPClient timeClient(ntpUDP, "in.pool.ntp.org", 3600, 60000);

void setup()
{
  Serial1.begin(9600);
     Serial.begin(9600);
    while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

while (! rtc.begin()) 
  {
    Serial.print("Couldn't find RTC");
   // while (1);
  }
   if (! rtc.isrunning()) 
  {
   Serial.print("RTC is NOT running!");
  }else {
  rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));  
  } 
  
Serial.println("Initializing SD Card...");
if (!card.init(SPI_HALF_SPEED, chipSelect)) {
    Serial.println("initialization failed. Things to check:");
    Serial.println("* is a card inserted?");
    Serial.println("* is your wiring correct?");
    Serial.println("* did you change the chipSelect pin to match your shield or module?");
    while (1);
  } else {
    Serial.println("Wiring is correct and a card is present.");
  }

File myFile = SD.open("Book1.csv", FILE_WRITE);
 

  // if the file is available, write to it:
  myFile.println("Date,Time,soilMoisture,pH,soil_temp,Soil_humd,Air_temp.Air_humd,intensity");

myFile.close();
   
   // ope.beginn the file. note that only one file dan be open at a time,
// so you have to close this one before opening another.
// if the file opened okay, write to it:
if (myFile) {
  Serial.print("Writing to test.txt...");
 
  myFile.close();
  Serial.print("done");

} else {

Serial.println("error opening test.txt");
}  

   // Serial.println("SHT20 Example!");
   // Serial.println("pH meter experiment!");
  
    sht20.initSHT20();                                  // Init SHT20 Sensor
    delay(100);
    sht20.checkSHT20();
     if(!tsl.begin())
  {
    /* There was a problem detecting the TSL2561 ... check your connections */
    Serial.print("Ooops, no TSL2561 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }// Check SHT20 Sensor
   configureSensor();
   TH02.begin(pin_scl,pin_sda);  
}

void loop()
{ 
  myFile = SD.open("Book1.csv", FILE_WRITE);
  printDateTime(); 
  int sl=0;
const int AirValue = 650;   //you need to replace this value with Value_1
const int WaterValue = 250;  //you need to replace this value with Value_2
int intervals = (AirValue - WaterValue)/3;
int soilMoistureValue = 0;
         Serial.print(", ");
          myFile.print(", ");
          Serial.print(sl++);   //Print Serial Number
          myFile.print(sl);
          Serial.print(", ");
          myFile.print(", ");
             
soilMoistureValue = analogRead(A8);
Serial.print(soilMoistureValue);

myFile.print(soilMoistureValue);
 Serial.print(" ");
 myFile.print(",");
 String S1;
if(soilMoistureValue > WaterValue && soilMoistureValue < (WaterValue + intervals))
{
  S1="Very Wet";
}
else if(soilMoistureValue > (WaterValue + intervals) && soilMoistureValue < (AirValue - intervals))
{
   S1="Wet";
}
else if(soilMoistureValue < AirValue && soilMoistureValue > (AirValue - intervals))
{
   S1="Dry";
}
Serial.print(" ");//put Sensor insert into soil
//int SoilMoisture=map(soilMoistureValue,270,1100,100,0);

 //Serial.print(SoilMoisture);

  static unsigned long samplingTime = millis();
  static unsigned long printTime = millis();
  static float pHValue,voltage;
  if(millis()-samplingTime > samplingInterval)
  {
      pHArray[pHArrayIndex++]=analogRead(SensorPin);
      if(pHArrayIndex==ArrayLenth)pHArrayIndex=0;
      voltage = avergearray(pHArray, ArrayLenth)*5.0/1024;
      pHValue = 3.5*voltage+Offset;
      samplingTime=millis();
  }
  if(millis() - printTime > printInterval)   //Every 800 milliseconds, print a numerical, convert the state of the LED indicator
  {
    //Serial.print("Voltage:");
      //  Serial.print(voltage,2);
        //Serial.print("    pH value: ");
    Serial.print(pHValue,2);
    Serial.print(" ");
    myFile.print(pHValue,2);
    myFile.print(",");
        digitalWrite(LED,digitalRead(LED)^1);
        printTime=millis();
  }  
   int humd = sht20.readHumidity();                  // Read Humidity
    int temp = sht20.readTemperature();               // Read Temperature
    //Serial.print("Time:");
    //Serial.print(millis());
   // Serial.print(" Temperature:");
    Serial.print(temp, 1);
    Serial.print("C");
    Serial.print(" ");
    myFile.print(temp,1);
    myFile.print(",");
  //  Serial.print(" Humidity:");
    Serial.print(humd, 1);
    Serial.print("%");
    Serial.print(" ");
    myFile.print(humd,1);
    myFile.print(",");
   int temper = TH02.ReadTemperature(); 
   Serial.print(temper);
   myFile.print(temper);
   myFile.print(",");
   Serial.print("C\r\t");
   
   int humidity = TH02.ReadHumidity();
   Serial.print(humidity);
   myFile.print(humidity);
   myFile.print(",");
   Serial.print("%\r\t");
    sensors_event_t event;
  tsl.getEvent(&event);
 
  /* Display the results (light is measured in lux) */
  if (int(event.light)>0)
  {
     light=map(event.light,2,10725,10,100);
    Serial.print(event.light); Serial.print(" lux  ");
     Serial.print(light);
     myFile.print(light);
     myFile.print(","); 
  }
   myFile.close();
  Serial.print("||");
  String data=String(S1)+String(pHValue,2)+String(temp)+String(humd)+String(temper)+String(humidity)+String(int(light));
  int len =data.length();
  Serial.print(len);
  Serial.println(data);
 
  
  Serial1.println(data);
  //delay(2000);
  if (Serial1.available()) {
    String inByte = Serial1.readStringUntil('\n');
    Serial.println(inByte);
    int t=inByte.length();
    Serial.println(t);
    if(t==14){
     char date[20];
   for(int i=0;i<14;i++){
    date[i]=inByte[i];
    }
    
   
    MONTH_RTC=date[4]-48;
    YEAR_RTC=(date[0]-48)*1000+(date[1]-48)*100+(date[2]-48)*10+(date[3]-48);
    
     DATE_RTC=(date[5]-48);
    
    HOUR_RTC=(date[6]-48)*10+(date[7]-48);
    MIN_RTC=(date[8]-48)*10+(date[9]-48);
    SEC_RTC=(date[10]-48)*10+(date[11]-48);
    Serial.print(" DATE_RTC:");
    Serial.print(DATE_RTC);
    Serial.print("\n");
    Serial.print("MONTH_RTC:");
    Serial.print(MONTH_RTC);
    Serial.print("\n");
    Serial.print("YEAR_RTC:");
    Serial.print(YEAR_RTC);
     Serial.print("\n");
     Serial.print("HOUR_RTC:");
    Serial.print(HOUR_RTC);
     Serial.print("\n");
    Serial.print("MIN_RTC:");
    Serial.print(MIN_RTC);
     Serial.print("\n");
     Serial.print("SEC_RTC:");
    Serial.println(SEC_RTC);
    rtc.adjust(DateTime(YEAR_RTC,MONTH_RTC,DATE_RTC,HOUR_RTC,MIN_RTC,SEC_RTC));  
    Serial.println("sensorValue ,Temperature:,Humidity:,PM1.0:,PM2.5:,PM 10: ");
    
     delay(1000);
   }
  }
  Serial.println();
  delay(1000);
  /*float temper = TH02.ReadTemperature(); 
  // Serial.println("Temperature: ");   
   Serial.print(temper);
   Serial.print("C");
   Serial.print(" ");
   float humidity = TH02.ReadHumidity();
   //Serial.println("Humidity: ");
   Serial.print(humidity);
   Serial.print("%\r");*/

 
}
double avergearray(int* arr, int number){
  int i;
  int max,min;
  double avg;
  long amount=0;
  if(number<=0){
    Serial.println("Error number for the array to avraging!/n");
    return 0;
  }
  if(number<5){   //less than 5, calculated directly statistics
    for(i=0;i<number;i++){
      amount+=arr[i];
    }
    avg = amount/number;
    return avg;
  }else{
    if(arr[0]<arr[1]){
      min = arr[0];max=arr[1];
    }
    else{
      min=arr[1];max=arr[0];
    }
    for(i=2;i<number;i++){
      if(arr[i]<min){
        amount+=min;        //arr<min
        min=arr[i];
      }else {
        if(arr[i]>max){
          amount+=max;    //arr>max
          max=arr[i];
        }else{
          amount+=arr[i]; //min<=arr<=max
        }
      }//if
    }//for
    avg = (double)amount/(number-2);
  }//if
  return avg;
}
void configureSensor(void)
{
  /* You can also manually set the gain or enable auto-gain support */
  // tsl.setGain(TSL2561_GAIN_1X);      /* No gain ... use in bright light to avoid sensor saturation */
  // tsl.setGain(TSL2561_GAIN_16X);     /* 16x gain ... use in low light to boost sensitivity */
  tsl.enableAutoRange(true);            /* Auto-gain ... switches automatically between 1x and 16x */
  
  /* Changing the integration time gives you better sensor resolution (402ms = 16-bit data) */
  tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_13MS);      /* fast but low resolution */
  // tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_101MS);  /* medium resolution and speed   */
  // tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_402MS);  /* 16-bit data but slowest conversions */

  /* Update these values depending on what you've set above! */  
  Serial.println("------------------------------------");
  Serial.print  ("Gain:         "); Serial.println("Auto");
  Serial.print  ("Timing:       "); Serial.println("13 ms");
  Serial.println("------------------------------------");
}
void printDateTime()
{
    DateTime now = rtc.now();

    char t_buf[20];
    char d_buf[20];

    sprintf(d_buf, "%04u/%02u/%02u",now.year(), now.month(), now.day());
    Serial.print(d_buf);
    myFile.print(d_buf);
    //Serial.write(d_buf);
    Serial.print(", ");
    myFile.print(", ");
   // Serial.write(", ");
    sprintf(t_buf, "%02u:%02u:%02u",now.hour(), now.minute(), now.second());
    Serial.print(t_buf);
    myFile.print(t_buf);
    //Serial.write(t_buf);
    //Serial.write(", ");
   
}
