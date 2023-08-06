//khai bao thu vien wifi manager
#include <DNSServer.h>
#include <WebServer.h>
#include <WiFiManager.h>         //https://github.com/tzapu/WiFiManager
#include <EEPROM.h>
//het khai bao thu vien wifi manager

//khai bao blynk
#define BLYNK_TEMPLATE_ID "TMPL6VdMjq1zY"
#define BLYNK_TEMPLATE_NAME "bam tai"
#define BLYNK_AUTH_TOKEN "lbCLOFP5bMfQIn7qwQFAgIkjLD9ZpaDv"
#include <Arduino.h>
#define BLYNK_PRINT Serial
#include <WiFi.h>
#include <BlynkSimpleEsp32.h>


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






int ptsac;
bool nhan = 0; 
unsigned long t1,t2,t3,t_bat_dau_cho  = 0;
const int so_mau = 500; 
const int lech_pha = 12; 
int Vsample[so_mau];
int Isample[so_mau];

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

int dl;


#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

RF24 radio(4, 5); // CE, CSN
const byte diachi[][6] = {"12345", "10000"}; //0, 1

//SSID of your network
char ssid[] = "Chau Phuong";
//password of your WPA Network
char pass[] = "chauphuong";


void setup() 
{
  Serial.begin(9600);

  // khoi tao wifi manager + blynk
//  EEPROM.begin(512);
//  WiFiManager wifiManager;
//  wifiManager.setConfigPortalTimeout(60);
//  //wifiManager.resetSettings();    //Uncomment this to wipe WiFi settings from EEPROM on boot.  Comment out and recompile/upload after 1 boot cycle.
//  wifiManager.autoConnect("ESP32");
//  //if you get here you have connected to the WiFi
//  Serial.println("connected...ok :)");
//  Blynk.begin(auth, WiFi.SSID().c_str(), WiFi.psk().c_str());

  WiFi.begin(ssid, pass);
  delay(5000);
  if (WiFi.status() == WL_CONNECTED) {
      Serial.println("Connected to WiFi! Now I will check the connection to the Blynk server");
      
      Blynk.config(auth);
      Blynk.connect(1000); //waiting 0.5 sec
   }
 
  if (!radio.begin()) 
  {
    Serial.println("Module không khởi động được...!!");
    while (1) {}
  }   
  radio.openWritingPipe(diachi[1]);
  //Chỉ có thể mở 1 đường ghi
  //Lệnh openWritingPipe có số đường truyền mặc định là 0
  //Mở 1 kênh có địa chỉ 10000 trên đường truyền 0
  //kênh này chỉ ghi data trên địa chỉ 10000   
  radio.openReadingPipe(1, diachi[0]);
  //Có thể mở 6 đường đọc cùng lúc
  //Nhưng đường 0 mặc định dùng cho ghi
  //Lệnh openReadingPipe có thể mở đường truyền từ 1-5
  //Đọc data của địa chỉ 12345 trên đường truyền 1
     
  radio.setPALevel(RF24_PA_MAX); //Cài bộ khuyết địa công suất ở mức MIN
  radio.setChannel(80);
  radio.setDataRate(RF24_250KBPS);    
  if (!radio.available())
  {
    Serial.println("Chưa kết nối được với RX...!!");
    Serial.println("CHỜ KẾT NỐI.......");
  }



  analogReadResolution(12);
  pinMode(adcPin,INPUT);
  pinMode(adcPinV,INPUT);




}

void loop() 
{
  if (WiFi.status() == WL_CONNECTED) {      //neu ket noi thanh cong thi in ra connected!

       Blynk.run();
      }
//    //kiem tra ket noi wifi hang 10 phut
  if ((millis() - t1) > 600000)
  {
    
    
    if (WiFi.status() != WL_CONNECTED) {      //neu ket noi thanh cong thi in ra connected!
            Serial.println("Ket noi lai wifi");
            WiFi.begin(ssid, pass);
            delay(1000);
             if (WiFi.status() == WL_CONNECTED) {
            Serial.println("Connected to WiFi! Now I will check the connection to the Blynk server");
            
            Blynk.config(auth);
           Blynk.connect(1000); //waiting 0.5 sec
         }
  
   }
  
    t1 = millis();
  }
//
//


  //het code kiem tra wifi


 //lay 500 mau
int i;
int j;

for (j = 1;j<=20;j=j+1) {
    
    for (i = 1;i<=so_mau;i=i+1) {
            Isample[i] = analogRead(adcPin);  
            Vsample[i] = analogRead(adcPinV);
    } 
     
    for (i = 1;i<=so_mau-lech_pha;i=i+1) {
            sampleI = Isample[i+lech_pha];
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

            if (i == so_mau-lech_pha)

            {
               float pReal = V_RATIO * I_RATIO * sumP1 / (so_mau-lech_pha);
               double iTai1 = I_RATIO * sqrt(sumI1 / (so_mau-lech_pha));
               double Vol1 = V_RATIO * sqrt(sumV1 / (so_mau-lech_pha));
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
              //gui qua bam tai 
              if (pReal < -15)
               {dl = 1;}
              else {dl = 0;} 
            //dl = random(0,1);
    
    radio.stopListening(); //Ngưng nhận
    
    radio.write(&dl, sizeof(dl));
    Serial.print("GT gửi: "); Serial.print(dl); Serial.print("   ");
    delay(10);
      
//    radio.startListening(); //Bắt đầu nhận
//    while(!radio.available());
//    int nutnhan; 
//    radio.read(&nutnhan, sizeof(nutnhan));
//    Serial.print("Nhận nút nhấn: "); Serial.println(nutnhan);
//    delay(10);

//
//      
        radio.startListening(); //Bắt đầu nhận
        t_bat_dau_cho = millis();
        while ((millis()- t_bat_dau_cho)< 10)
        {
         if(radio.available())
         
         {
          radio.read(&ptsac, sizeof(ptsac));
          Serial.print("Do rong xung: "); Serial.println(ptsac);
          Serial.println("co nhan duoc");
          nhan = 1;
          break;
          }
          
          }
                    Serial.println("chay qua day");

          if (nhan ==0)
          {Serial.println("Khong nhan duoc");}
////                //het code gui qua bam tai 
////
  

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
  
   pTai = V_RATIO * I_RATIO * sumP / ((so_mau-lech_pha)* j);
   iTai = I_RATIO * sqrt(sumI / ((so_mau-lech_pha) * j));
   Vol = V_RATIO * sqrt(sumV / ((so_mau-lech_pha) * j));
   double appPower1 = iTai * Vol;
   pf1 = pTai/ appPower1;
  //  Serial.print("P thuc: ");
  //  Serial.println(pTai);         
  //  Serial.print("Dong: ");
  //  Serial.println(iTai);
  //  Serial.print("Vol: ");
  //  Serial.println(Vol);
  //  Serial.print("Power factor: ");
  //  Serial.println(pf1);
            
   sumI = 0;
   sumV = 0;
   sumP = 0;
    sendPtai();
    sendVol();

    sendptsac();

   

   
  
  //het code tinh va cap nhat blynk
  }

} // dong vong lap for j

/////////////////////////////////////////  
   
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

void sendptsac()
{
  double sdata =   ptsac;

  Blynk.virtualWrite(V4, sdata);
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
