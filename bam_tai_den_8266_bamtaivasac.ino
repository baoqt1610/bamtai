//khai bao sac acquy


int ptInv, ptsac;

//khai bao lay gio
#include <NTPClient.h>
#include <WiFiUdp.h>

const long utcOffsetInSeconds = 25200;

//----------------------------------------Define NTP Client to get time
// Define NTP Client to get time
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org");

//Week Days
String weekDays[7]={"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

//Month names
String months[12]={"January", "February", "March", "April", "May", "June", "July", "August", "September", "October", "November", "December"};

//het khai bao lay gio


#define relay6  D2
//int bat = 1; int tat = 0;      //dieu khien relay muc cao
int bat = 1; int tat = 0;   //dieu khien relay muc thap
 

//khai bao relay 6
int gio_mo_relay6 = 16;
int phut_mo_relay6 = 0;
int gio_tat_relay6 = 6;
int phut_tat_relay6 = 0;
byte trang_thai_relay6;  
unsigned long t_bat_dau_relay6 = 0;

 
//het khai bao relay

//khai bao thu vien wifi manager
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>         //https://github.com/tzapu/WiFiManager
#include <EEPROM.h>
//het khai bao thu vien wifi manager

//update code through Github

#include <ESP8266HTTPClient.h>
#include <ESP8266httpUpdate.h>
#include <WiFiClientSecure.h>
#include <CertStoreBearSSL.h>
BearSSL::CertStore certStore;
#include <time.h>
 


#define URL_fw_Bin "https://raw.githubusercontent.com/baoqt1610/bamtai/main/bam_tai.bin"
const char* host = "raw.githubusercontent.com";
const int httpsPort = 443;

// DigiCert High Assurance EV Root CA
const char trustRoot[] PROGMEM = R"EOF(
-----BEGIN CERTIFICATE-----
MIIDrzCCApegAwIBAgIQCDvgVpBCRrGhdWrJWZHHSjANBgkqhkiG9w0BAQUFADBh
MQswCQYDVQQGEwJVUzEVMBMGA1UEChMMRGlnaUNlcnQgSW5jMRkwFwYDVQQLExB3
d3cuZGlnaWNlcnQuY29tMSAwHgYDVQQDExdEaWdpQ2VydCBHbG9iYWwgUm9vdCBD
QTAeFw0wNjExMTAwMDAwMDBaFw0zMTExMTAwMDAwMDBaMGExCzAJBgNVBAYTAlVT
MRUwEwYDVQQKEwxEaWdpQ2VydCBJbmMxGTAXBgNVBAsTEHd3dy5kaWdpY2VydC5j
b20xIDAeBgNVBAMTF0RpZ2lDZXJ0IEdsb2JhbCBSb290IENBMIIBIjANBgkqhkiG
9w0BAQEFAAOCAQ8AMIIBCgKCAQEA4jvhEXLeqKTTo1eqUKKPC3eQyaKl7hLOllsB
CSDMAZOnTjC3U/dDxGkAV53ijSLdhwZAAIEJzs4bg7/fzTtxRuLWZscFs3YnFo97
nh6Vfe63SKMI2tavegw5BmV/Sl0fvBf4q77uKNd0f3p4mVmFaG5cIzJLv07A6Fpt
43C/dxC//AH2hdmoRBBYMql1GNXRor5H4idq9Joz+EkIYIvUX7Q6hL+hqkpMfT7P
T19sdl6gSzeRntwi5m3OFBqOasv+zbMUZBfHWymeMr/y7vrTC0LUq7dBMtoM1O/4
gdW7jVg/tRvoSSiicNoxBN33shbyTApOB6jtSj1etX+jkMOvJwIDAQABo2MwYTAO
BgNVHQ8BAf8EBAMCAYYwDwYDVR0TAQH/BAUwAwEB/zAdBgNVHQ4EFgQUA95QNVbR
TLtm8KPiGxvDl7I90VUwHwYDVR0jBBgwFoAUA95QNVbRTLtm8KPiGxvDl7I90VUw
DQYJKoZIhvcNAQEFBQADggEBAMucN6pIExIK+t1EnE9SsPTfrgT1eXkIoyQY/Esr
hMAtudXH/vTBH1jLuG2cenTnmCmrEbXjcKChzUyImZOMkXDiqw8cvpOp/2PV5Adg
06O/nVsJ8dWO41P0jmP6P6fbtGbfYmbW0W5BjfIttep3Sp+dWOIrWcBAI+0tKIJF
PnlUkiaY4IBIqDfv8NZ5YBberOgOzW6sRBc4L0na4UU+Krk2U886UAb3LujEV0ls
YSEY1QSteDwsOoBrp+uvFRTp2InBuThs4pFsiv9kuXclVzDAGySj4dzp30d8tbQk
CAUw7C29C79Fv1C5qfPrmAESrciIxpg0X40KPMbp1ZWVbd4=
-----END CERTIFICATE-----
)EOF";
X509List cert(trustRoot);


extern const unsigned char caCert[] PROGMEM;
extern const unsigned int caCertLen;

//het khai bao update code through Github




#define BLYNK_TEMPLATE_ID "TMPL6Coq8bexg"
#define BLYNK_TEMPLATE_NAME "Bam Tai"
#define BLYNK_AUTH_TOKEN "xTyRuJEuxkYfZL5keNW-tcXeWRlRYsjM"
#include <Arduino.h>
#define BLYNK_PRINT Serial
#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>
unsigned long t1,t2,t3  = 0;


char auth[] = BLYNK_AUTH_TOKEN;


 WiFiClientSecure client;

 
BlynkTimer timer;

 

String myString; // complete message from arduino, which consistors of snesors data
char rdata; // received charactors
double pTai, pInv, pFTai, pFInv, P_CL; // sensors
int vol;


void myTimerEvent()
{

  //Blynk.virtualWrite(V5, millis() / 1000);
  
}
 
// doc lenh update tu Blynk

 
void setup()
{



//khoi tao time

  timeClient.begin();
  // Set offset time in seconds to adjust for your timezone, for example:
  // GMT +1 = 3600
  // GMT +8 = 28800
  // GMT -1 = -3600
  // GMT 0 = 0
  timeClient.setTimeOffset(utcOffsetInSeconds);

//het khoi tao time


  // Debug console
  Serial.begin(9600);
  while (!Serial);
//  mySerial.begin(9600);
//  while (!mySerial);
  // khoi tao wifi manager
  EEPROM.begin(512);
  WiFiManager wifiManager;
  //wifiManager.resetSettings();    //Uncomment this to wipe WiFi settings from EEPROM on boot.  Comment out and recompile/upload after 1 boot cycle.
  wifiManager.autoConnect("ESP8266");
  //if you get here you have connected to the WiFi
  Serial.println("connected...ok :)");
  Blynk.begin(auth, WiFi.SSID().c_str(), WiFi.psk().c_str());
  //MultyWiFiBlynkBegin(); 
//het code khoi tao wifi manager
  //wifiConnectHandler = WiFi.onStationModeGotIP(onWifiConnect);
  //wifiDisconnectHandler = WiFi.onStationModeDisconnected(onWifiDisconnect);
   
 
    timer.setInterval(1000L,sensorvalue1); 
    timer.setInterval(1000L,sensorvalue2); 
      timer.setInterval(1000L,sensorvalue3);
        timer.setInterval(1000L,sensorvalue4); 
      
//                  timer.setInterval(1000L,sensorvalue6);






pinMode(relay6,OUTPUT);
digitalWrite(relay6,tat);






}
 
void loop()

{

  // het dieu khien den
  
   if (Serial.available() == 0 )

   {
  Blynk.run();
  timer.run(); // Initiates BlynkTimer
 
   }
//  

  //kiem tra ket noi wifi hang 1 phut
  if ((millis() - t1) > 60000)
  {
    
    
    if (WiFi.status() != WL_CONNECTED) {      //neu ket noi thanh cong thi in ra connected!
      
         ketnoilaiwifivablynk();
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
      //pInv = m.toFloat();
      vol =  n.toInt();
      ptInv =  m.toInt();
      ptsac =  i.toInt();
      //TH =   map(i.toInt(), 0, 100, 10, 90);
      
      //P_CL = pInv - pTai;

      Serial.println(myString);

      myString = "";

      // get date & time  
      timeClient.update();
    
      unsigned long epochTime = timeClient.getEpochTime();
      
      int hr, mn;
      hr = timeClient.getHours();
      mn = timeClient.getMinutes();
      Serial.print("gio hien tai: ");
      Serial.println(hr);
      Serial.print("phut hien tai: ");
      Serial.println(mn);
      
          
     //end get date & time





     
    //bat dau code relay 6

     if (trang_thai_relay6 ==0)
     {
      if (hr == gio_mo_relay6 and mn == phut_mo_relay6)
      {
        bat_relay6();
        
        }
      
      }

      else 
      if (hr == gio_tat_relay6 and mn == phut_tat_relay6)
      {
        tat_relay6();
        
        }
    

     //het code relay6
      
    
    }

  


  }
  //

 


}



void sensorvalue1()
{
  double sdata = pTai;

  Blynk.virtualWrite(V0, sdata);

}

void sensorvalue4()
{
  double sdata = vol;

  Blynk.virtualWrite(V3, sdata);
}


void sensorvalue2()
{
  double sdata = ptInv;

  Blynk.virtualWrite(V4, sdata);
}


void sensorvalue3()
{
  double sdata = ptsac;

  Blynk.virtualWrite(V2, sdata);
}

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

//void MultyWiFiBlynkBegin() {
//  int ssid_count=0;
//  int ssid_mas_size = sizeof(ssid) / sizeof(ssid[0]);
//  do {
//    Serial.println("Trying to connect to wi-fi " + String(ssid[ssid_count]));
//    WiFi.begin(ssid[ssid_count], pass[ssid_count]);    
//    int WiFi_timeout_count=0;
//    while (WiFi.status() != WL_CONNECTED && WiFi_timeout_count<50) { //waiting 10 sec
//      delay(200);
//      Serial.print(".");
//      ++WiFi_timeout_count;
//    }
//    if (WiFi.status() == WL_CONNECTED) {
//      Serial.println("Connected to WiFi! Now I will check the connection to the Blynk server");
//      Blynk.config(auth);
//      Blynk.connect(5000); //waiting 5 sec
//    }
//    ++ssid_count; 
//  }
//  while (!Blynk.connected() && ssid_count<ssid_mas_size);
//  if (!Blynk.connected() && ssid_count==ssid_mas_size) {
//    Serial.println("I could not connect to blynk =( Ignore and move on. but still I will try to connect to wi-fi " + String(ssid[ssid_count-1]));
//  }
//    
//}


void ketnoilaiwifivablynk()
{
  Serial.println("Disconnected from Wi-Fi, trying to connect...");
  WiFi.disconnect();
  WiFi.begin(WiFi.SSID().c_str(), WiFi.psk().c_str());
   if (WiFi.status() == WL_CONNECTED) {
      Serial.println("Connected to WiFi! Now I will check the connection to the Blynk server");
      Blynk.config(auth);
      Blynk.connect(5000); //waiting 5 sec
   }
  
}

BLYNK_WRITE(V5)
{
  if (param.asInt()) {  // Assumes if 1 then follow through..
  Serial.println("Nhan lenh update firmware");
  FirmwareUpdate();
 }
}


void setClock() {
   // Set time via NTP, as required for x.509 validation
  configTime(3 * 3600, 0, "pool.ntp.org", "time.nist.gov");
  Serial.print("Waiting for NTP time sync: ");
  time_t now = time(nullptr);
  while (now < 8 * 3600 * 2) {
    delay(500);
    Serial.print(".");
    now = time(nullptr);
  }
  /*
  Serial.println("");
  struct tm timeinfo;
  gmtime_r(&now, &timeinfo);
  Serial.print("Current time: ");
  Serial.print(asctime(&timeinfo));
  */
}
  



void FirmwareUpdate()
{  
  Serial.println("Start");
  WiFi.mode(WIFI_STA);
  if (WiFi.status() != WL_CONNECTED) {
  ketnoilaiwifivablynk();
    }
  
  setClock();
 
  client.setTrustAnchors(&cert);
  if (!client.connect(host, httpsPort)) {
    Serial.println("Connection failed");
    return;
  }
  else 
  {
    Serial.println("Connection sucessfull, start to update code");
    t_httpUpdate_return ret = ESPhttpUpdate.update(client, URL_fw_Bin);
        
    switch (ret) {
      case HTTP_UPDATE_FAILED:
        Serial.printf("HTTP_UPDATE_FAILD Error (%d): %s\n", ESPhttpUpdate.getLastError(), ESPhttpUpdate.getLastErrorString().c_str());
        break;

      case HTTP_UPDATE_NO_UPDATES:
        Serial.println("HTTP_UPDATE_NO_UPDATES");
        break;

      case HTTP_UPDATE_OK:
        Serial.println("HTTP_UPDATE_OK");
        break;
    }
    
    }

  
 }  

//cap nhat cai dat relay 1

// //cap nhat trang thai relay1
// BLYNK_WRITE(V10)
// {
//   trang_thai_relay1 = param.asInt();
//   if (trang_thai_relay1)
//   {bat_relay1();}
//   else {tat_relay1();}
   
//   }

// //nhan pt den mo relay 1
// BLYNK_WRITE(V19)
// {
//   ptden_mo_relay1 = param.asInt();
//   Serial.print("da cap nhat pt den mo relay1: ");
//   Serial.println(ptden_mo_relay1);
     
//   }

// //cap nhat cong suat tat relay1
// BLYNK_WRITE(V20)
// {
//   CS_tat_relay1 = param.asInt();
//   Serial.print("da cap nhat cong suat tat relay1: ");
//   Serial.println(CS_tat_relay1);
    
//   }
// //het cap nhat cai dat relay 1

// //cap nhat cai dat relay 2

// //cap nhat trang thai relay2 
// BLYNK_WRITE(V17)
// {
//   trang_thai_relay2 = param.asInt();
//   if (trang_thai_relay2)
//   {bat_relay2();}
//   else {tat_relay2();}
  
//   }
// //nhan pt den mo relay 2
// BLYNK_WRITE(V22)
// {
//   ptden_mo_relay2 = param.asInt();
//   Serial.print("da cap nhat pt den mo relay2: ");
//   Serial.println(ptden_mo_relay2);
     
//   }

// //cap nhat cong suat tat relay2
// BLYNK_WRITE(V23)
// {
//   CS_tat_relay2 = param.asInt();
//   Serial.print("da cap nhat cong suat tat relay2: ");
//   Serial.println(CS_tat_relay2);
    
//   }

// //het cap nhat relay 2



// //cap nhat cai dat relay 3

// //cap nhat trang thai relay3 
// BLYNK_WRITE(V1)
// {
//   trang_thai_relay3 = param.asInt();
//   if (trang_thai_relay3)
//   {bat_relay3();}
//   else {tat_relay3();}
  
//   }
// //nhan pt den mo relay 3
// BLYNK_WRITE(V16)
// {
//   ptden_mo_relay3 = param.asInt();
//   Serial.print("da cap nhat pt den mo relay3: ");
//   Serial.println(ptden_mo_relay3);
     
//   }

// //cap nhat cong suat tat relay3
// BLYNK_WRITE(V25)
// {
//   CS_tat_relay3 = param.asInt();
//   Serial.print("da cap nhat cong suat tat relay3: ");
//   Serial.println(CS_tat_relay3);
    
//   }

// //het cap nhat relay 3


// //cap nhat cai dat relay 4

// //cap nhat trang thai relay4 
// BLYNK_WRITE(V12)
// {
//   trang_thai_relay4 = param.asInt();
//   if (trang_thai_relay4)
//   {bat_relay4();}
//   else {tat_relay4();}
  
//   }
// //nhan pt den mo relay 4
// BLYNK_WRITE(V26)
// {
//   ptden_mo_relay4 = param.asInt();
//   Serial.print("da cap nhat pt den mo relay4: ");
//   Serial.println(ptden_mo_relay4);
     
//   }

// //cap nhat cong suat tat relay4
// BLYNK_WRITE(V6)
// {
//   CS_tat_relay4 = param.asInt();
//   Serial.print("da cap nhat cong suat tat relay4: ");
//   Serial.println(CS_tat_relay4);
    
//   }

// //het cap nhat relay 4


// //cai dat relay 5
// //cap nhat gio mo relay5
// BLYNK_WRITE(V15)
// {
//   gio_mo_relay5_nhan = param.asInt();
//   gio_mo_relay5 = int(gio_mo_relay5_nhan/(60*60));
//   phut_mo_relay5 = int((gio_mo_relay5_nhan - gio_mo_relay5 * 60 * 60)/60) ;  

  
//   Serial.print("da cap nhat gio mo relay5: ");
//   Serial.println(gio_mo_relay5);

  
//   Serial.print("da cap nhat phut mo relay5: ");
//   Serial.println(phut_mo_relay5);

//   }
// //cap nhat trang thai relay5  
// BLYNK_WRITE(V13)
// {
//   trang_thai_relay5 = param.asInt();
//   if (trang_thai_relay5)
//   {bat_relay5();}
//   else {tat_relay5();}
   
//   }

// //cap nhat thoi gian bat relay 5
// BLYNK_WRITE(V8)
// {
//   thoi_gian_mo_relay5 = param.asInt();
//   Serial.print("da cap nhat thoi gian mo relay5: ");
//   Serial.println(thoi_gian_mo_relay5);
    
//   }
// // het cap nhat relay 5


//cai dat relay 6
//cap nhat gio mo relay6

//cap nhat trang thai relay6  
BLYNK_WRITE(V2)
{
  trang_thai_relay6 = param.asInt();
  if (trang_thai_relay6)
  {bat_relay6();}
  else {tat_relay6();}
   
  }

//cap nhat thoi gian bat relay 6

// het cap nhat relay 6



void tat_relay6()
{
digitalWrite(relay6,tat);
trang_thai_relay6 = 0;
t_bat_dau_relay6 = 0;
Blynk.virtualWrite(V2, trang_thai_relay6);
Serial.println("Da tat relay6");
}

void bat_relay6()
{
digitalWrite(relay6,bat);
trang_thai_relay6 = 1;
t_bat_dau_relay6 = millis();
Blynk.virtualWrite(V2, trang_thai_relay6);
Serial.println("Da bat relay6");
}

//het khai bao update code through Github
