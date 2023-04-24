#include "esp32-hal-cpu.h"

int Vsample[500];
int Isample[500];

////khai bao lay gio
//#include <NTPClient.h>
//#include <WiFiUdp.h>
//
//const long utcOffsetInSeconds = 25200;
//
////----------------------------------------Define NTP Client to get time
//// Define NTP Client to get time
//WiFiUDP ntpUDP;
//NTPClient timeClient(ntpUDP, "pool.ntp.org");
//
////Week Days
//String weekDays[7]={"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};
//
////Month names
//String months[12]={"January", "February", "March", "April", "May", "June", "July", "August", "September", "October", "November", "December"};
//
////het khai bao lay gio

int Freq = 0;

//khai bao sac acquy

int maxsac = 100; 
#define pinPWM1 14 
int DRX = 0;

//het khai bao sac acquy

//khai bao relay
#define relay1  23
#define relay2  22
#define relay3  21
#define relay4  19 //chan TX
#define relay5  18
#define relay6  17
#define relay7  16
#define relay8  4

int bat = 1; int tat = 0;      //dieu khien relay muc cao
//int bat = 0; int tat = 1;   //dieu khien relay muc thap
 
//khai bao relay 1
int ptden_mo_relay1; 
int CS_tat_relay1;
int trang_thai_relay1;  
unsigned long t_bat_dau_relay1 = 0;

//khai bao relay 2
int ptden_mo_relay2; 
int CS_tat_relay2;
int trang_thai_relay2;  
unsigned long t_bat_dau_relay2 = 0;

//khai bao relay 3
int ptden_mo_relay3; 
int CS_tat_relay3;
int trang_thai_relay3;  
unsigned long t_bat_dau_relay3 = 0;

//khai bao relay 4
int ptden_mo_relay4; 
int CS_tat_relay4;
int trang_thai_relay4;  
unsigned long t_bat_dau_relay4 = 0;

//khai bao relay 5
int gio_mo_relay5_nhan; 
int gio_mo_relay5;
int phut_mo_relay5;
int thoi_gian_mo_relay5;
int trang_thai_relay5;  
unsigned long t_bat_dau_relay5 = 0;

//khai bao relay 6
int gio_mo_relay6_nhan; 
int gio_mo_relay6;
int phut_mo_relay6;
int thoi_gian_mo_relay6;
int trang_thai_relay6;  
unsigned long t_bat_dau_relay6 = 0;

//khai bao relay 7
int trang_thai_relay7;  
unsigned long t_bat_dau_relay7 = 0;

//khai bao relay 8
int trang_thai_relay8;  
unsigned long t_bat_dau_relay8 = 0;

//khai bao thu vien wifi manager
#include <DNSServer.h>
#include <WebServer.h>
#include <WiFiManager.h>         //https://github.com/tzapu/WiFiManager
#include <EEPROM.h>
//het khai bao thu vien wifi manager

//khai bao blynk
#define BLYNK_TEMPLATE_ID "TMPL6Coq8bexg"
#define BLYNK_TEMPLATE_NAME "Bam Tai"
#define BLYNK_AUTH_TOKEN "xTyRuJEuxkYfZL5keNW-tcXeWRlRYsjM"
#include <Arduino.h>
#define BLYNK_PRINT Serial
#include <WiFi.h>
#include <BlynkSimpleEsp32.h>
unsigned long t1,t2,t3  = 0;


char auth[] = BLYNK_AUTH_TOKEN;



//het khai bao blynk

//update code through Github

#include <HTTPClient.h>
#include <HTTPUpdate.h>
#include <WiFiClientSecure.h>


#include <time.h>
 

#define URL_fw_Bin "https://raw.githubusercontent.com/baoqt1610/bamtai/main/bam_tai.bin"
const char* host = "raw.githubusercontent.com";
const int httpsPort = 443;

//DigiCert root certificate has expiry date of 10 Nov 2031
const char * rootCACertificate = \
"-----BEGIN CERTIFICATE-----\n" \
"MIIDrzCCApegAwIBAgIQCDvgVpBCRrGhdWrJWZHHSjANBgkqhkiG9w0BAQUFADBh\n" \
"MQswCQYDVQQGEwJVUzEVMBMGA1UEChMMRGlnaUNlcnQgSW5jMRkwFwYDVQQLExB3\n" \
"d3cuZGlnaWNlcnQuY29tMSAwHgYDVQQDExdEaWdpQ2VydCBHbG9iYWwgUm9vdCBD\n" \
"QTAeFw0wNjExMTAwMDAwMDBaFw0zMTExMTAwMDAwMDBaMGExCzAJBgNVBAYTAlVT\n" \
"MRUwEwYDVQQKEwxEaWdpQ2VydCBJbmMxGTAXBgNVBAsTEHd3dy5kaWdpY2VydC5j\n" \
"b20xIDAeBgNVBAMTF0RpZ2lDZXJ0IEdsb2JhbCBSb290IENBMIIBIjANBgkqhkiG\n" \
"9w0BAQEFAAOCAQ8AMIIBCgKCAQEA4jvhEXLeqKTTo1eqUKKPC3eQyaKl7hLOllsB\n" \
"CSDMAZOnTjC3U/dDxGkAV53ijSLdhwZAAIEJzs4bg7/fzTtxRuLWZscFs3YnFo97\n" \
"nh6Vfe63SKMI2tavegw5BmV/Sl0fvBf4q77uKNd0f3p4mVmFaG5cIzJLv07A6Fpt\n" \
"43C/dxC//AH2hdmoRBBYMql1GNXRor5H4idq9Joz+EkIYIvUX7Q6hL+hqkpMfT7P\n" \
"T19sdl6gSzeRntwi5m3OFBqOasv+zbMUZBfHWymeMr/y7vrTC0LUq7dBMtoM1O/4\n" \
"gdW7jVg/tRvoSSiicNoxBN33shbyTApOB6jtSj1etX+jkMOvJwIDAQABo2MwYTAO\n" \
"BgNVHQ8BAf8EBAMCAYYwDwYDVR0TAQH/BAUwAwEB/zAdBgNVHQ4EFgQUA95QNVbR\n" \
"TLtm8KPiGxvDl7I90VUwHwYDVR0jBBgwFoAUA95QNVbRTLtm8KPiGxvDl7I90VUw\n" \
"DQYJKoZIhvcNAQEFBQADggEBAMucN6pIExIK+t1EnE9SsPTfrgT1eXkIoyQY/Esr\n" \
"hMAtudXH/vTBH1jLuG2cenTnmCmrEbXjcKChzUyImZOMkXDiqw8cvpOp/2PV5Adg\n" \
"06O/nVsJ8dWO41P0jmP6P6fbtGbfYmbW0W5BjfIttep3Sp+dWOIrWcBAI+0tKIJF\n" \
"PnlUkiaY4IBIqDfv8NZ5YBberOgOzW6sRBc4L0na4UU+Krk2U886UAb3LujEV0ls\n" \
"YSEY1QSteDwsOoBrp+uvFRTp2InBuThs4pFsiv9kuXclVzDAGySj4dzp30d8tbQk\n" \
"CAUw7C29C79Fv1C5qfPrmAESrciIxpg0X40KPMbp1ZWVbd4=\n" \
"-----END CERTIFICATE-----\n";


//extern const unsigned char caCert[] PROGMEM;
//extern const unsigned int caCertLen;

//het khai bao update code through Github






//khai bao dimmer
#include <RBDdimmer.h>//

#define outputPin  26 
#define zerocross  25 // for boards with CHANGEBLE input pins

dimmerLamp dimmer(outputPin, zerocross); //initialase port for dimmer for ESP8266, ESP32, Arduino due boards
//dimmerLamp dimmer(outputPin); //initialase port for dimmer for MEGA, Leonardo, UNO, Arduino M0, Arduino Zero

int TH = 0;
int TH_current = 0;
//het khai bao dimmer


const int coi = 10;
int ptden = 0;

long SupplyVoltage = 3300;
bool adc_conversion_working = false;
boolean Measure_Supply_volts = true;
bool working;

const byte adcPin = 36; // = 14 (pins_arduino.h) cam bien dong inverter
const byte adcPinV = 39; // = 14 (pins_arduino.h) cam bien ap

int Number_of_Samples = 1480*4;
int Number_of_PSamples = 200; //so mau lay P
double offsetI = 1342+448;
double sampleI;
double filteredI;
double sqI;
double sumI;


float ICAL = 67.5;
double iTai, Vol, pTai, pf1;

//khai bao cam bien voltage

 const int VCAL = 1188;

 int sampleV;
 double filteredV;          //Filtered_ is the raw analog value minus the DC offset
 double offsetV = 1344+449;                          //Low-pass filter output
 double sqV;
 double sumV;
 double sumV1;
 double sumI1;
 double instP,sumP,sumP1; 
double lastFilteredV;
double phaseShiftedV; 
double PHASECAL = 5;
double pf;

double I_RATIO = ICAL *((SupplyVoltage/1000.0) / (4096));
double V_RATIO = VCAL *((SupplyVoltage/1000.0) / (4096));

//het khai bao bien cam bien vol

int n = 0;
int m = 0;
boolean startnewsample = true;

unsigned long previousMillis, millis_coitruoc = 0;

unsigned long Vref_SET_TIME;
unsigned long currentMillis = millis();
boolean settleVref = HIGH;

void setup() {

  // Debug console
  Serial.begin(115200);
  while (!Serial);

 
 Freq = getCpuFrequencyMhz();
  Serial.print("CPU Freq = ");
  Serial.print(Freq);
  Serial.println(" MHz");
  Freq = getXtalFrequencyMhz();
  Serial.print("XTAL Freq = ");
  Serial.print(Freq);
  Serial.println(" MHz");
  Freq = getApbFrequency();
  Serial.print("APB Freq = ");
  Serial.print(Freq);
  Serial.println(" Hz");
 
 
 
 // pinMode(LED_BUILTIN, OUTPUT);
//khoi tao time

//  timeClient.begin();
//  // Set offset time in seconds to adjust for your timezone, for example:
//  // GMT +1 = 3600
//  // GMT +8 = 28800
//  // GMT -1 = -3600
//  // GMT 0 = 0
//  timeClient.setTimeOffset(utcOffsetInSeconds);
//
//het khoi tao time

//khoi tao sac acquy


ledcSetup(0, 2000, 10);
  /* Attach the LED PWM Channel to the GPIO Pin */
ledcAttachPin(pinPWM1,2);
ledcWrite(2, DRX);

//het khoi tao sac acqui

//khoi tao relay
pinMode(relay1,OUTPUT);
digitalWrite(relay1,tat);

pinMode(relay2,OUTPUT);
digitalWrite(relay2,tat);

pinMode(relay3,OUTPUT);
digitalWrite(relay3,tat);

pinMode(relay4,OUTPUT);
digitalWrite(relay4,tat);


pinMode(relay5,OUTPUT);
digitalWrite(relay5,tat);


pinMode(relay6,OUTPUT);
digitalWrite(relay6,tat);


pinMode(relay7,OUTPUT);
digitalWrite(relay7,tat);

pinMode(relay8,OUTPUT);
digitalWrite(relay8,tat);


Blynk.virtualWrite(V11, maxsac);

trang_thai_relay1 = 0;
//Blynk.virtualWrite(V10, trang_thai_relay1);
ptden_mo_relay1 = 0; 
Blynk.virtualWrite(V19, ptden_mo_relay1);
CS_tat_relay1 = 0; 
Blynk.virtualWrite(V20, CS_tat_relay1);

trang_thai_relay2 = 0;
Blynk.virtualWrite(V17, trang_thai_relay2);
ptden_mo_relay2 = 0; 
Blynk.virtualWrite(V22, ptden_mo_relay2);
CS_tat_relay2 = 0; 
Blynk.virtualWrite(V23, CS_tat_relay2);

trang_thai_relay3 = 0;
Blynk.virtualWrite(V1, trang_thai_relay3);
ptden_mo_relay3 = 0; 
Blynk.virtualWrite(V16, ptden_mo_relay3);
CS_tat_relay3 = 0; 
Blynk.virtualWrite(V25, CS_tat_relay3);

trang_thai_relay4 = 0;
//Blynk.virtualWrite(V12, trang_thai_relay4);
ptden_mo_relay4 = 0; 
Blynk.virtualWrite(V26, ptden_mo_relay4);
CS_tat_relay4 = 0; 
Blynk.virtualWrite(V6, CS_tat_relay4);

trang_thai_relay5 = 0;
Blynk.virtualWrite(V13, trang_thai_relay5);
gio_mo_relay5_nhan = 0; 
Blynk.virtualWrite(V15, gio_mo_relay5_nhan);
thoi_gian_mo_relay5 = 0; 
Blynk.virtualWrite(V8, thoi_gian_mo_relay5);

trang_thai_relay6 = 0;
Blynk.virtualWrite(V2, trang_thai_relay6);
gio_mo_relay6_nhan = 0; 
Blynk.virtualWrite(V7, gio_mo_relay6_nhan);
thoi_gian_mo_relay6 = 0; 
Blynk.virtualWrite(V9, thoi_gian_mo_relay6);

trang_thai_relay7 = 0;

trang_thai_relay8 = 0;

//het khoi tao relay

analogReadResolution(12);
pinMode(adcPin,INPUT);
pinMode(adcPinV,INPUT);


   dimmer.begin(NORMAL_MODE, OFF); //dimmer initialisation: name.begin(MODE, STATE) 



Serial.println("ADC without blocking");

  // khoi tao wifi manager + blynk
  EEPROM.begin(512);
  WiFiManager wifiManager;
//  WiFiManager.setConfigPortalTimeout(60);
  //wifiManager.resetSettings();    //Uncomment this to wipe WiFi settings from EEPROM on boot.  Comment out and recompile/upload after 1 boot cycle.
  wifiManager.autoConnect("ESP32");
  //if you get here you have connected to the WiFi
  Serial.println("connected...ok :)");
  Blynk.begin(auth, WiFi.SSID().c_str(), WiFi.psk().c_str());
//  //MultyWiFiBlynkBegin(); 
//het code khoi tao wifi manager
  //wifiConnectHandler = WiFi.onStationModeGotIP(onWifiConnect);
  //wifiDisconnectHandler = WiFi.onStationModeDisconnected(onWifiDisconnect);
   
 
//    timer.setInterval(1000L,sensorvalue1); 
////     timer.setInterval(1000L,sensorvalue2); 
////       timer.setInterval(1000L,sensorvalue3);
//        timer.setInterval(1000L,sensorvalue4); 
//            timer.setInterval(1000L,sensorvalue5); 
//                  timer.setInterval(1000L,sensorvalue6);
//het khoi tao wifi manager va blynk

}

void loop() {

  Blynk.run();
  //timer.run(); // Initiates BlynkTimer

//lay 500 mau
int i;
int j;

for (j = 1;j<=20;j=j+1) {
    
    for (i = 1;i<=500;i=i+1) {
            Isample[i] = analogRead(adcPin);  
            Vsample[i] = analogRead(adcPinV);
    } 
     
    for (i = 1;i<=500-15;i=i+1) {
            sampleI = Isample[i+15];
            sampleV = Vsample[i];
            offsetI = (offsetI + (sampleI-offsetI)/4096);
            filteredI = sampleI - offsetI;
        
            // Root-mean-square method current
            // 1) square current values
            sqI = filteredI * filteredI;
            // 2) sum
            sumI += sqI;
            sumI1 += sqI;
            
            offsetV = offsetV + ((sampleV-offsetV)/4096);
            filteredV = sampleV - offsetV;
        
            sqV= filteredV * filteredV;                 //1) square voltage values
            sumV += sqV;                                //2) sum
            sumV1 += sqV;
            instP = filteredV * filteredI;          //Instantaneous Power
            sumP +=instP;
            sumP1 +=instP;    

            //neu da du so mau 

            if (i == 500-15)

            {
               float pReal = V_RATIO * I_RATIO * sumP1 / (500-15);
               double iTai1 = I_RATIO * sqrt(sumI1 / (500-15));
               double Vol1 = V_RATIO * sqrt(sumV1 / (500-15));
               double appPower = iTai1 * Vol1;
               double pf = pReal/ appPower;
    

               Serial.print("P thuc: ");
               Serial.println(pReal);
               
               Serial.print("Dong: ");
               Serial.println(iTai1);
               Serial.print("Vol: ");
               Serial.println(Vol1);
               Serial.print("Power factor: ");
               Serial.println(pf);
            
    
    //code dieu khien den moi
    
      
//code dieu khien den moi
    
      if (pReal < -15)
    
      {
           if ((DRX < maxsac) and (maxsac > 0))
            {
              
             DRX = DRX + 1;
//             Serial.println("tang sac");
             if (DRX > 100)
                {DRX = 100;}
            
            }
           
           if ((DRX > maxsac) and (maxsac > 0))
            {
              
             DRX = DRX - 1;
//             Serial.println("giam sac");
             if (DRX < 0)
                {DRX = 0;}
            
            }

            else if ((DRX == maxsac) or (maxsac == 0))
            {
               TH = TH + 1;
//               Serial.println("tang den");
               if (TH > 100)
                   {TH = 100;} 
             }
            
            }
                
      else 
      {
         if (TH > 0)
         {
          TH = TH - 1;
//           Serial.println("giam den");
          if (TH < 0)
          {TH = 0;}
         }
         
         else if ((TH == 0) and (DRX > 0))
         {
           DRX = DRX - 1;
//            Serial.println("giam sac");
           if (DRX < 0)
           {DRX = 0;}
          }
      }

  //dieu khien sac
  //tat/ mo sac len
  if (DRX > 0)
  {
    if (trang_thai_relay7 == 0)
    {bat_relay7();}
    }
  else 

  {
    
    if (trang_thai_relay7 != 0)
      {
      if ((millis() - t_bat_dau_relay7) > 1000) 
        {tat_relay7();}
      }
   }
  
  int ptsac = map(DRX, 0, 100, 0, 1024);
  ledcWrite(2, ptsac);
//  Serial.print("% sac: ");
//   Serial.println(DRX);
  
  //het dieu  khien sac



  int ptden = map(TH, 0, 100, 10, 90);
  if ((ptden > 10) and (ptden != TH_current))
  {
//    Serial.print("bat den: ");
//    Serial.println(ptden);
    dimmer.setState(ON);
    dimmer.setPower(ptden); // name.setPower(0%-100%)

    TH_current = ptden;
  }
  else if (ptden <= 10)
  {
    dimmer.setState(OFF);
//    Serial.println("tat den: ");

  }
//   Serial.print("% den: ");
//   Serial.println(TH);
    //het code dieu khien den moi
    
//    //coi cho coi hu
//    //
//    if (millis_coitruoc ==0) 
//    
//    {
//      if             ((ptden > 20) and (ptden <50)) {coi1tieng();}
//        
//      else if       ((ptden >= 50)and (ptden <80))  {coi2tieng();}
//      else if       (ptden >= 80)                   {coi3tieng();}
//      
//      millis_coitruoc = millis(); 
//      
//      }
//    else if (millis_coitruoc !=0)
//    
//    {
//    if (millis() - millis_coitruoc > 60000)
//    
//    {
//    
//      if             ((ptden > 20) and (ptden <50)) {coi1tieng();}
//     
//      else if       ((ptden >= 50)and (ptden <80))  {coi2tieng();}
//      else if       (ptden >= 80)                   {coi3tieng();}
//      
//      millis_coitruoc = millis(); 
//      
//    }
//    
//    }
//    //
//    //het code coi
    





    sumP1 = 0; 
    sumI1 = 0; 
    sumV1 = 0; 
       
       }      

   
//    Serial.print(filteredI);
//    Serial.print(" ");
//    Serial.println(filteredV);
//  

    } // dong vong lap for i

if (j == 20)
{
  //tinh vua cap nhat blynk
  
   pTai = V_RATIO * I_RATIO * sumP / ((500-15)* j);
   iTai = I_RATIO * sqrt(sumI / ((500-15) * j));
   Vol = V_RATIO * sqrt(sumV / ((500-15) * j));
   double appPower1 = iTai * Vol;
   pf1 = pTai/ appPower1;
   Serial.print("P thuc: ");
   Serial.println(pTai);         
   Serial.print("Dong: ");
   Serial.println(iTai);
   Serial.print("Vol: ");
   Serial.println(Vol);
   Serial.print("Power factor: ");
   Serial.println(pf1);
            
   sumI = 0;
   sumV = 0;
   sumP = 0;
   sendPtai();
   sendVol();
   sendptDen();
   sendptsac();
   sendpf();
   sendiTai();
   
  
  
  //het code tinh va cap nhat blynk
  }

} // dong vong lap for j
}

void tatcoi()

{
  digitalWrite(coi,LOW);
  Serial.println(" tat coi");
  }

void coi1tieng()

{
  digitalWrite(coi,HIGH);
  delay(200);
  digitalWrite(coi,LOW);
  Serial.println(" coi 1 tieng");
  }

void coi2tieng()

{
  digitalWrite(coi,HIGH);
  delay(200);
  digitalWrite(coi,LOW);
  delay(200);
  digitalWrite(coi,HIGH);
  delay(200);
  digitalWrite(coi,LOW);
  Serial.println(" coi 2 tieng");
  }

void coi3tieng()

{
  digitalWrite(coi,HIGH);
  delay(200);
  digitalWrite(coi,LOW);
  delay(200);
  digitalWrite(coi,HIGH);
  delay(200);
  digitalWrite(coi,LOW);
  delay(200);
  digitalWrite(coi,HIGH);
  delay(200);
  digitalWrite(coi,LOW);
  Serial.println(" coi 3 tieng");
  }


  void sendPtai()
{
  double sdata = pTai;

  Blynk.virtualWrite(V0, sdata);

}

void sendVol()
{
  double sdata = Vol;

  Blynk.virtualWrite(V3, sdata);
}

void sendptDen()
{
  double sdata =   TH;

  Blynk.virtualWrite(V4, sdata);
}


void sendptsac()
{
  double sdata =   DRX;

  Blynk.virtualWrite(V8, sdata);
}

void sendpf()
{
  double sdata = pf1;

  Blynk.virtualWrite(V10, sdata);
}

void sendiTai()
{
  double sdata = iTai;

  Blynk.virtualWrite(V12, sdata);
}


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
  
  //setClock();
 
  WiFiClientSecure client;
  client.setCACert(rootCACertificate);
//  httpUpdate.setLedPin(LED_BUILTIN, LOW);
  t_httpUpdate_return ret = httpUpdate.update(client, URL_fw_Bin);

  switch (ret) {
  case HTTP_UPDATE_FAILED:
    Serial.printf("HTTP_UPDATE_FAILD Error (%d): %s\n", httpUpdate.getLastError(), httpUpdate.getLastErrorString().c_str());
    break;

  case HTTP_UPDATE_NO_UPDATES:
    Serial.println("HTTP_UPDATE_NO_UPDATES");
    break;

  case HTTP_UPDATE_OK:
    Serial.println("HTTP_UPDATE_OK");
    break;
  }
    
    

  
 }


void tat_relay1()
{
digitalWrite(relay1,HIGH);
trang_thai_relay1 = 0;
t_bat_dau_relay1 = 0;
//Blynk.virtualWrite(V10, trang_thai_relay1);
Serial.println("Da tat relay1");
}

void bat_relay1()
{
digitalWrite(relay1,bat);
trang_thai_relay1 = 1;
t_bat_dau_relay1 = millis();
//Blynk.virtualWrite(V10, trang_thai_relay1);
Serial.println("Da bat relay1");
}

void tat_relay2()
{
digitalWrite(relay2,tat);
trang_thai_relay2 = 0;
t_bat_dau_relay2 = 0;
//Blynk.virtualWrite(V17, trang_thai_relay2);
Serial.println("Da tat relay2");
}

void bat_relay2()
{
digitalWrite(relay2,bat);
trang_thai_relay2 = 1;
t_bat_dau_relay2 = millis();
//Blynk.virtualWrite(V17, trang_thai_relay2);
Serial.println("Da bat relay2");
}


void tat_relay3()
{
digitalWrite(relay3,tat);
trang_thai_relay3 = 0;
t_bat_dau_relay3 = 0;
//Blynk.virtualWrite(V1, trang_thai_relay3);
Serial.println("Da tat relay3");
}

void bat_relay3()
{
digitalWrite(relay3,bat);
trang_thai_relay3 = 1;
t_bat_dau_relay3 = millis();
//Blynk.virtualWrite(V1, trang_thai_relay3);
Serial.println("Da bat relay3");
}

void tat_relay4()
{
digitalWrite(relay4,tat);
trang_thai_relay4 = 0;
t_bat_dau_relay4 = 0;
//Blynk.virtualWrite(V12, trang_thai_relay4);
Serial.println("Da tat relay4");
}

void bat_relay4()
{
digitalWrite(relay4,bat);
trang_thai_relay4 = 1;
t_bat_dau_relay4 = millis();
//Blynk.virtualWrite(V12, trang_thai_relay4);
Serial.println("Da bat relay4");
}

void tat_relay5()
{
digitalWrite(relay5,tat);
trang_thai_relay5 = 0;
t_bat_dau_relay5 = 0;
//Blynk.virtualWrite(V13, trang_thai_relay5);
Serial.println("Da tat relay5");
}

void bat_relay5()
{
digitalWrite(relay5,bat);
trang_thai_relay5 = 1;
t_bat_dau_relay5 = millis();
//Blynk.virtualWrite(V13, trang_thai_relay5);
Serial.println("Da bat relay5");
}


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



void tat_relay7()
{
digitalWrite(relay7,tat);
trang_thai_relay7 = 0;
t_bat_dau_relay7 = 0;
//Serial.println("Da tat relay7");
}

void bat_relay7()
{
digitalWrite(relay7,bat);
trang_thai_relay7 = 1;
t_bat_dau_relay7 = millis();
//Serial.println("Da bat relay7");
}

void tat_relay8()
{
digitalWrite(relay8,tat);
trang_thai_relay8 = 0;
t_bat_dau_relay8 = 0;
Serial.println("Da tat relay8");
}

void bat_relay8()
{
digitalWrite(relay8,bat);
trang_thai_relay8 = 1;
t_bat_dau_relay8 = millis();
Serial.println("Da bat relay8");
}
