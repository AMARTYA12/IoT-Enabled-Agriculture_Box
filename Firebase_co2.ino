#include <FirebaseESP8266.h>
#include <FirebaseESP8266HTTPClient.h>
#include <FirebaseFS.h>
#include <FirebaseJson.h>

#include <NTPClient.h>

#include <WiFiUdp.h>

#include <ESP8266WiFi.h>                                                    // esp8266 library
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>

                                               // firebase library
                                                           // dht11 temperature and humidity sensor library

#define FIREBASE_HOST "https://fir-temp-a0573.firebaseio.com"                          // the project name address from firebase id
#define FIREBASE_AUTH "XYN4Pt1LCORQjHIKex2ZNjIaFxZpZjsXvlf4Jxix"            // the secret key generated from firebase

#define WIFI_SSID "amartya"                                             // input your home or public wifi name 
#define WIFI_PASSWORD "12345678"                                    //password of wifi ssid
WiFiUDP ntpUDP;
// You can specify the time server pool and the offset (in seconds, can be

// changed later with setTimeOffset() ). Additionaly you can specify the
// update interval (in milliseconds, can be changed using setUpdateInterval() ).
NTPClient timeClient(ntpUDP, "in.pool.ntp.org");
//Week Days
String weekDays[7]={"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

//Month names
String months[12]={"January", "February", "March", "April", "May", "June", "July", "August", "September", "October", "November", "December"};

 int t=0;
int sensorValue; 
String SoilMoisture,pH,temp,hum,luxo,Air_tem,Air_hum;                                                 
String path="/DHT";
FirebaseData firebaseData;
void setup() {
   pinMode(BUILTIN_LED, OUTPUT); 
  Serial.begin(9600);      
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);                                     //try to connect with wifi
  Serial.print("Connecting to ");
  Serial.print(WIFI_SSID);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
   
  }
  Serial.println();
/*  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH)
      type = "sketch";
    else // U_SPIFFS
      type = "filesystem";

    // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
    Serial.println("Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });
  ArduinoOTA.begin();*/
  Serial.print("Connected to ");
  Serial.println(WIFI_SSID);
  Serial.print("IP Address is : ");
  Serial.println(WiFi.localIP());
 

      
  //print local IP address
Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);   
timeClient.begin();// connect to firebase
 timeClient.setTimeOffset(19800);
//  dht.begin();                                                               //Start reading dht sensor
}

void loop() { 
  // ArduinoOTA.handle();

  if(t<10){
  timeClient.update();

  unsigned long epochTime = timeClient.getEpochTime();
 // Serial.print("Epoch Time: ");
  //Serial.println(epochTime);
  
  String formattedTime = timeClient.getFormattedTime();
  //Serial.print("Formatted Time: ");
  //Serial.println(formattedTime);  

  int currentHour = timeClient.getHours();
  Serial.print("Hour: ");
  Serial.println(currentHour);  

  int currentMinute = timeClient.getMinutes();
  Serial.print("Minutes: ");
  Serial.println(currentMinute); 
   
  int currentSecond = timeClient.getSeconds();
  //Serial.print("Seconds: ");
  //Serial.println(currentSecond);  

  String weekDay = weekDays[timeClient.getDay()];
  //Serial.print("Week Day: ");
  //Serial.println(weekDay);    

  //Get a time structure
  struct tm *ptm = gmtime ((time_t *)&epochTime); 
  
  int monthDay = ptm->tm_mday;
  //Serial.print("Month day: ");
  //Serial.println(monthDay);

  int currentMonth = ptm->tm_mon+1;
  Serial.print("Month: ");
  Serial.println(currentMonth);

  String currentMonthName = months[currentMonth-1];
  //Serial.print("Month name: ");
  //Serial.println(currentMonthName);

  int currentYear = ptm->tm_year+1900;
  Serial.print("Year: ");
  Serial.println(currentYear);

  //Print complete date:
  String currentDate = String(currentYear) + String(currentMonth) +String(monthDay);
 
  
  String Str=String(currentDate)+String(formattedTime);

  Serial.println(Str);
   t++;
 delay(5000);
  }
while(Serial.available()>0){
  String S1=Serial.readStringUntil('\n');
  int len=S1.length();
 // Serial.println(len);
   SoilMoisture=S1.substring(0,3);
  
  pH=S1.substring(3,6);
   temp=S1.substring(7,9);
  hum=S1.substring(9,11);
   Air_tem=S1.substring(11,13);
   Air_hum=S1.substring(13,15);
   luxo=S1.substring(15,17);
 
}
  delay(2000);
  
   Firebase.setString(firebaseData, path+"/Soil",SoilMoisture);
   
   
   Firebase.setString(firebaseData, path+"/pH",pH);
  
   Firebase.setString(firebaseData, path+"/temp",temp);
   
   Firebase.setString(firebaseData, path+"/hum",hum);
    Firebase.setString(firebaseData, path+"/Air_hum",Air_hum);
     Firebase.setString(firebaseData, path+"/Air_tem",Air_tem);
   Firebase.setString(firebaseData, path+"/luxo",luxo);
  
  Serial.println(SoilMoisture);
  Serial.println(pH);
  Serial.println(temp);
  Serial.println(hum);
  Serial.println(luxo);
  Serial.println(Air_tem);
  Serial.println(Air_hum);
 // Firebase.getString("/DHT11/CO2", fireHumid);                                  //setup path and send readings
 // Firebase.pushString("/DHT11/Temperature", fireTemp);                                //setup path and send readings
   

}
