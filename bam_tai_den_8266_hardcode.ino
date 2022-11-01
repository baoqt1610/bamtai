//khai bao button
#include <EasyButton.h>

// Arduino pin where the button is connected to.
#define BUTTON_PIN 0

// Instance of the button.
EasyButton button(BUTTON_PIN);

// Callback function to be called when the button is pressed.
void onPressedForDuration()
{
  Serial.println("Button pressed, start webserver");
  updateOTA();
}
//het khai bao button


//khai bao update OTA
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <ESP8266HTTPUpdateServer.h>
const char* host = "Bam Tai";
const char* updatePath = "/update";
const char* updateUsername = "admin";
const char* updatePassword = "admin";

ESP8266WebServer webServer(80);
ESP8266HTTPUpdateServer httpUpdater;
//-----------------------------------------//
const char MainPage[] PROGMEM = R"=====(
  <!DOCTYPE html> 
  <html>
   <head> 
       <title>BAM TAI BANG DEN</title> 
       <style> 
          body{
            text-align: center;
          }
       </style>
       <meta name="viewport" content="width=device-width,user-scalable=0" charset="UTF-8">
   </head>
   <body> 
      <div>
        <img src='https://dienthongminhesmart.000webhostapp.com/firmware_ota.jpg' height='200px' width='330px'>
      </div>
      <div>
        <button onclick="window.location.href='/update'">UPLOAD FIRMWARE</button><br><br>
        
      </div>
      <script>
      </script>
   </body> 
  </html>
)=====";
// het khai bao update OTA


//khai bao dimmer
#include <RBDdimmer.h>//

#define outputPin  D5 
#define zerocross  D1 // for boards with CHANGEBLE input pins

dimmerLamp dimmer(outputPin, zerocross); //initialase port for dimmer for ESP8266, ESP32, Arduino due boards
//dimmerLamp dimmer(outputPin); //initialase port for dimmer for MEGA, Leonardo, UNO, Arduino M0, Arduino Zero

int TH = 0;
int TH_current = 0;
//het khai bao dimmer

#define BLYNK_TEMPLATE_ID "TMPLavBYkElh"
#define BLYNK_DEVICE_NAME "Bam Tai"
#define BLYNK_AUTH_TOKEN "Fbx2AulLFP0IN213o8ODO2X_1J0CFeXF"

#define BLYNK_PRINT Serial
#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>
////#include <SoftwareSerial.h>
////const byte RX = D5;
////const byte TX = D6;
//SoftwareSerial mySerial(RX,TX);
unsigned long t1,t2  = 0;


char auth[] = BLYNK_AUTH_TOKEN;

char* ssid[] = {"Xiaomi_3D3B","Bao Tri","Ngoc Duc","Foco"}; //list a necessary wifi networks
char* pass[] = {"0909051109","0901486129","0914099379","0909051109"}; //list a passwords



//WiFiEventHandler wifiConnectHandler;
//WiFiEventHandler wifiDisconnectHandler;

 
BlynkTimer timer;

 

String myString; // complete message from arduino, which consistors of snesors data
char rdata; // received charactors
double pTai, pInv, pFTai, pFInv, P_CL; // sensors
int vol;




void myTimerEvent()
{

  //Blynk.virtualWrite(V5, millis() / 1000);
  
}
 
 
 
void setup()
{


  // Debug console
  Serial.begin(9600);
  while (!Serial);
//  mySerial.begin(9600);
//  while (!mySerial);

  MultyWiFiBlynkBegin(); 

  //wifiConnectHandler = WiFi.onStationModeGotIP(onWifiConnect);
  //wifiDisconnectHandler = WiFi.onStationModeDisconnected(onWifiDisconnect);
   
 
    timer.setInterval(1000L,sensorvalue1); 
     timer.setInterval(1000L,sensorvalue2); 
       timer.setInterval(1000L,sensorvalue3);
        timer.setInterval(1000L,sensorvalue4); 
            timer.setInterval(1000L,sensorvalue5); 
//                  timer.setInterval(1000L,sensorvalue6);

   dimmer.begin(NORMAL_MODE, OFF); //dimmer initialisation: name.begin(MODE, STATE) 


 // khoi tao button.
  button.begin();
  // Add the callback function to be called when the button is pressed for at least the given time.
  button.onPressedFor(2000, onPressedForDuration);
 // het khoi tao button.
}
 
void loop()

{
  


  
   if (Serial.available() == 0 )

   {
  Blynk.run();
  timer.run(); // Initiates BlynkTimer
  MDNS.update();
  webServer.handleClient();
    // Continuously read the status of the button.
  button.read();
   }
//  

  //kiem tra ket noi wifi hang 5 phut
  if ((millis() - t1) > 600000)
  {
    if (WiFi.status() != WL_CONNECTED) {      //neu ket noi thanh cong thi in ra connected!
      
         MultyWiFiBlynkBegin();
      }

    

    t1 = millis();
  }




  //het code kiem tra wifi


  
 if (Serial.available() > 0 )
  {

    rdata = Serial.read();
    myString = myString + rdata;

    if (rdata == '\n')
    {

      String l = getValue(myString, ',', 0);
      String m = getValue(myString, ',', 1);
      String n = getValue(myString, ',', 2);
      String i = getValue(myString, ',', 3);



      pTai = l.toFloat();
      pInv = m.toFloat();
      vol =  n.toInt();
      TH =   map(i.toInt(), 0, 90, 10, 90);
      
      P_CL = pInv - pTai;

      Serial.println(myString);

      myString = "";

    }

  }
  //



  if ((TH > 0) and (TH != TH_current))
  {
    dimmer.setState(ON);
    dimmer.setPower(TH); // name.setPower(0%-100%)

    TH_current = TH;
  }
  else if (TH <= 0)
  {
    dimmer.setState(OFF);

  }
  // end new code



}



void sensorvalue1()
{
  double sdata = pTai;

  Blynk.virtualWrite(V0, sdata);

}
void sensorvalue2()
{
  double sdata = pInv;

  Blynk.virtualWrite(V1, sdata);

}

void sensorvalue3()
{
  double sdata = P_CL;

  Blynk.virtualWrite(V2, sdata);

}
void sensorvalue4()
{
  double sdata = vol;

  Blynk.virtualWrite(V3, sdata);
}

void sensorvalue5()
{
  double sdata = TH;

  Blynk.virtualWrite(V4, sdata);
}

//void sensorvalue6()
//{
//double sdata = TH_current;
//
//  Blynk.virtualWrite(V5, sdata);
// }
//

String getValue(String data, char separator, int index)
{
  int found = 0;
  int strIndex[] = { 0, -1 };
  int maxIndex = data.length() - 1;

  for (int i = 0; i <= maxIndex && found <= index; i++) {
    if (data.charAt(i) == separator || i == maxIndex) {
      found++;
      strIndex[0] = strIndex[1] + 1;
      strIndex[1] = (i == maxIndex) ? i + 1 : i;
    }
  }
  return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}

void MultyWiFiBlynkBegin() {
  int ssid_count=0;
  int ssid_mas_size = sizeof(ssid) / sizeof(ssid[0]);
  do {
    Serial.println("Trying to connect to wi-fi " + String(ssid[ssid_count]));
    WiFi.begin(ssid[ssid_count], pass[ssid_count]);    
    int WiFi_timeout_count=0;
    while (WiFi.status() != WL_CONNECTED && WiFi_timeout_count<50) { //waiting 10 sec
      delay(200);
      Serial.print(".");
      ++WiFi_timeout_count;
    }
    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("Connected to WiFi! Now I will check the connection to the Blynk server");
      Blynk.config(auth);
      Blynk.connect(5000); //waiting 5 sec
    }
    ++ssid_count; 
  }
  while (!Blynk.connected() && ssid_count<ssid_mas_size);
  if (!Blynk.connected() && ssid_count==ssid_mas_size) {
    Serial.println("I could not connect to blynk =( Ignore and move on. but still I will try to connect to wi-fi " + String(ssid[ssid_count-1]));
  }
    
}


//void onWifiDisconnect(const WiFiEventStationModeDisconnected& event) {
//  Serial.println("Disconnected from Wi-Fi, trying to connect...");
//  WiFi.disconnect();
//  MultyWiFiBlynkBegin();
//}


void updateOTA()

{
  
    Serial.println("Booting programs...");
  WiFi.mode(WIFI_STA);
  MultyWiFiBlynkBegin();
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  MDNS.begin(host);
  MDNS.addService("http", "tcp", 80);

  httpUpdater.setup(&webServer, updatePath, updateUsername, updatePassword);
  webServer.on("/",[]{
    String s = MainPage;
    webServer.send(200,"text/html",s);
  });
  webServer.begin();
  Serial.println("Web Server is started!");
  
  
  }
