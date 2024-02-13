//khai bao on 
float R1 = 96.4; 
float R2 = 19.68;
float Vcal= 114;
float Ical = 100.0;
float p_cal1 = 0.0;
float p_cal2 = 0.0; 

double psac_accu = 0.0;
double phoa_accu = 0.0;
double ptai_accu = 0.0;
double ptai_accu_truoc = 0.0;
double WH = 0.0;

//float ptai_accu = 0; 
int ngay_hien_tai = 0;

//float phoa_accu = 0;  
#define relaysac  27  //chan FAN1
#define relayhoaluoi  12 //chan FAN2


const int doap = 33; // = 14 (pins_arduino.h) cam bien dong inverter
const int dodong = 35;

float ap = 0.0; 
float dong =0.0; 

int relaysac_status = 0; 
int relayhoaluoi_status = 0; 

int bat = 1; 
int tat = 0; 

int phut, gio, ngay; 
 
//het khai bao rely 
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

String mystring; 

char auth[] = BLYNK_AUTH_TOKEN;



//het khai bao blynk

//update code through Github

#include <HTTPClient.h>
#include <HTTPUpdate.h>
#include <WiFiClientSecure.h>


#include <time.h>
const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = 25200;
const int   daylightOffset_sec = 3600;
 

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
decltype(millis()) t1,t2,t3,t_bat_dau_cho, t_relay, t_mo_sac = 0;
decltype(millis()) t_do  = 0;
decltype(millis()) tg;
decltype(millis()) tg1;

int quadong_status = 0;
const int so_mau = 500; 
int lech_pha = 7; 
int lech_pha2 = 6; 
int Vsample[so_mau];
int Isample[so_mau];
int I2sample[so_mau];

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

double offsetI2 = 1342+448;
double sampleI2;
double filteredI2;
double sqI2;
double sumI2, sumI21,pReal2;


float ICAL = 67.5;
double iTai, Vol, pTai, pf1, iTai2, pTai2;

//khai bao cam bien voltage

 const int VCAL = 1188;

 int sampleV;
 double filteredV;          //Filtered_ is the raw analog value minus the DC offset
 double offsetV = 1344+449;                          //Low-pass filter output
 double sqV;
 double sumV;
 double sumV1;
 double sumI1;
 double instP,sumP,sumP1,instP2,sumP2,sumP21; 
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

decltype(millis()) previousMillis, millis_coitruoc = 0;

decltype(millis()) Vref_SET_TIME;
decltype(millis()) currentMillis = millis();
boolean settleVref = HIGH;

int dl;
int dem_tang_INV=0;
//bien dieu khien 
int saconh= 7; 
int saconm= 0;
int sacoffh = 16;
int sacoffm = 0;

int hoaonh= 17; 
int hoaonm= 0;
int hoaoffh = 7;
int hoaoffm = 0;

float minV = 11.0;
float maxV = 14.5;
float maxI = 50.0; 
float maxP = 600.0; 

//

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

RF24 radio(4, 5); // CE, CSN
const byte diachi[][6] = {"12345", "10000"}; //0, 1

//SSID of your network
char ssid[] = "Xiaomi_3D3B";
//password of your WPA Network
char pass[] = "0909051109";

// khai bao sac PWM

const int ledPin = 14;  // 16 corresponds to GPIO16

// setting PWM properties
const int freq = 5000;
const int ledChannel = 0;
const int resolution = 8;
int DRX = 0; 
int maxsac = 50;
// het khai bao sac PWM

void setup() 
{

  //Khoi tao relay 

  pinMode(relaysac,OUTPUT);
  digitalWrite(relaysac,tat);

  pinMode(relayhoaluoi,OUTPUT);
  digitalWrite(relayhoaluoi,tat);



  //het khoi tao relay
  Serial.begin(9600);
  Serial.println("version 1.2");
 WiFi.begin(ssid, pass);
   Blynk.config(auth);
  // Init and get the time
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  printLocalTime();

  //het khoi tao thoi gian

  // khoi tao wifi manager + blynk
//  EEPROM.begin(512);
//  WiFiManager wifiManager;
//  wifiManager.setConfigPortalTimeout(60);
//  //wifiManager.resetSettings();    //Uncomment this to wipe WiFi settings from EEPROM on boot.  Comment out and recompile/upload after 1 boot cycle.
//  wifiManager.autoConnect("ESP32");
//  //if you get here you have connected to the WiFi
//  Serial.println("connected...ok :)");
//  Blynk.begin(auth, WiFi.SSID().c_str(), WiFi.psk().c_str());

 
 
  if (!radio.begin()) 
  {
    Serial.println("Module không khởi động được...!!");
    //while (1) {}
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
     radio.printDetails(); 
 
  if (!radio.available())
  {
    Serial.println("Chưa kết nối được với RX...!!");
    Serial.println("CHỜ KẾT NỐI.......");
  }



  analogReadResolution(12);
  pinMode(adcPin,INPUT);
  pinMode(adcPinV,INPUT);
  pinMode(doap,INPUT);
  pinMode(dodong,INPUT);
//khoi tao PWM

// configure LED PWM functionalitites
  ledcSetup(ledChannel, freq, resolution);
  
  // attach the channel to the GPIO to be controlled
  ledcAttachPin(ledPin, ledChannel);


//het khoi tao PWM




}

void loop() 
{
 

  if (WiFi.status() == WL_CONNECTED) {      //neu ket noi thanh cong thi in ra connected!

       Blynk.run();
      }
  if ((millis() - t1) > 6000)
  {
    
    
    if (WiFi.status() != WL_CONNECTED) {      //neu ket noi thanh cong thi in ra connected!
              Serial.println("Mat ket noi wifi, dang ket noi lai");
              WiFi.begin(ssid, pass);
   //Blynk.config(auth);
         }
  
   
  
      

    

    t1 = millis();
  }
//
//

  //het code kiem tra wifi

kiemtra();

dieukhiensacvahoaluoi();
baove();

 //lay 500 mau
int i;
int j;

for (j = 1;j<=20;j=j+1) {
    
    for (i = 1;i<=so_mau;i=i+1) {
            Isample[i] = analogRead(adcPin);  
            I2sample[i] = analogRead(dodong);  
            Vsample[i] = analogRead(adcPinV);
            
    } 
     
    for (i = 1;i<=so_mau-lech_pha2;i=i+1) {
            sampleI = Isample[i+lech_pha];
            sampleI2 = I2sample[i+lech_pha2];
            sampleV = Vsample[i];
            offsetI = (offsetI + (sampleI-offsetI)/4096);
            filteredI = sampleI - offsetI;
        
            offsetI2 = (offsetI2 + (sampleI2-offsetI2)/4096);
            filteredI2 = sampleI2 - offsetI2;
        
            // Root-mean-square method current
            // 1) square current values
            sqI = filteredI * filteredI;
            sqI2 = filteredI2 * filteredI2;
            // 2) sum
            sumI += sqI;
            sumI1 += sqI;

            sumI2 += sqI2;
            sumI21 += sqI2;

            
            offsetV = offsetV + ((sampleV-offsetV)/4096);
            filteredV = sampleV - offsetV;
        
            sqV= filteredV * filteredV;                 //1) square voltage values
            sumV += sqV;                                //2) sum
            sumV1 += sqV;
            instP = filteredV * filteredI;          //Instantaneous Power
            sumP +=instP;
            sumP1 +=instP;    

            instP2 = filteredV * filteredI2;          //Instantaneous Power
            sumP2 +=instP2;
            sumP21 +=instP2;    

            //neu da du so mau 

            if (i == so_mau-lech_pha2)

            {
               float pReal = (V_RATIO * I_RATIO * sumP1 / (so_mau-lech_pha));
               pReal = pReal; // dieu chinh tang len 100w

               
               pReal2 = (V_RATIO * I_RATIO * sumP2 / (so_mau-lech_pha2));

               if (pReal2>0)
                {
                  pReal2=pReal2 + p_cal1;
                }
                else
                {
                  pReal2 = pReal2 + p_cal2; // dieu chinh tang len 100w  
                }
               
                

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
//code dieu khien
          //phat mot it len luoi 100w
             if (pReal < -100)
    
            {
              TH_thua();      
              }
                      
            else 
            {
              TH_thieu();

            }

      // //het dieu khien sac
                 
            //dl = random(0,1);
    //mo sac len
    

    int sac = map(DRX, 0, 100, 0, 255);
    ledcWrite(ledChannel, sac);


    radio.stopListening(); //Ngưng nhận
    
    radio.write(&dl, sizeof(dl));
    Serial.print("GT gửi: "); Serial.print(dl); Serial.print("   ");
    
     
        radio.startListening(); //Bắt đầu nhận
        t_bat_dau_cho = millis();
        while ((millis()- t_bat_dau_cho)< 100)
        {
         if(radio.available())
         
         {
          radio.read(&ptsac, sizeof(ptsac));
          Serial.print("Do rong xung nhan duoc: "); Serial.println(ptsac);
          //Serial.println("co nhan duoc");
          //Blynk.virtualWrite(V1, ptsac);
          radio.flush_rx();


        nhan = 1;
          break;
          }
          else 
          {nhan =0;}

          
          }
                    //Serial.println("chay qua day");

          if (nhan ==0)
          {Serial.println("Khong nhan duoc");}
////                //het code gui qua bam tai 
////
      radio.stopListening(); //Ngưng nhận
      delay(10);
    

    sumP1 = 0; 
    sumI1 = 0; 
    sumV1 = 0; 

    
    sumP2 = 0; 
    sumI2 = 0; 
       
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

   pTai2 = V_RATIO * I_RATIO * sumP21 / ((so_mau-lech_pha2)* j);

          if (pTai2>0)
          {
            pTai2=pTai2 + p_cal1;
          }
          else
          {
            pTai2 = pTai2 - p_cal2; // dieu chinh tang len 100w  
          }
   
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

   
   sumI21 = 0;
   sumP21 = 0;

  tinhWH();

    sendPtai();
    sendVol();
    sendptsac();
    sendptsacpin();
    sendap();
    
    sendPsac();
    sendPsac_congdon();
    sendPhoa_congdon();
    sendPtai_cong_don();

    

   // sendPsac_congdon()

   

   
  
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

void sendptsacpin()
{
 double sdata =   DRX;
 

  Blynk.virtualWrite(V2, sdata);
}

void sendap()
{
  double sdata =   ap;
  

  Blynk.virtualWrite(V6, sdata);
}

// void senddong()
// {
//   double sdata =   pTai2;

//   Blynk.virtualWrite(V12, sdata);
// }

void sendPsac()
{
  
  double sdata = pTai2; 
   Blynk.virtualWrite(V12, sdata);
}

void sendPsac_congdon()
{
  
  double sdata = psac_accu; 
  Blynk.virtualWrite(V8, sdata);
}

void sendPhoa_congdon()
{
  
  double sdata = phoa_accu;
  
  Blynk.virtualWrite(V10, sdata);
}



void sendPtai_cong_don()
{
  
  double sdata = ptai_accu; 
  

  Blynk.virtualWrite(V5, sdata);
}


// BLYNK_WRITE(V5)
// {
//   if (param.asInt()) {  // Assumes if 1 then follow through..
//   Serial.println("Nhan lenh update firmware");
// FirmwareUpdate();
//  }
// }



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

 void tangsac()
{
  DRX = DRX + 5; 
  Serial.println("tang sac");

  if (DRX >= maxsac)
   {DRX = maxsac;}
}

void giamsac()
{

  DRX = DRX - 5; 
  Serial.println("giam sac");
  
  //chinh ve max sac
  if (DRX > maxsac)
  {DRX = maxsac;}

  if (DRX < 0)
   {DRX = 0;}


}

void tangINV()
{
  if (dl ==1)
  
  {
    dem_tang_INV = 0;
  }

  dl = 0; 
  dem_tang_INV = dem_tang_INV + 1;  
  
  if (dem_tang_INV > 200)
  {

    dem_tang_INV = 200;
  }

  Serial.println("tang INV");
   

}


void giamINV()
{
  
  dl = 1; 
  Serial.println("giam INV");
}

 

 void TH_thua()
{

//sac chua max

if (DRX < maxsac)
  {tangsac();}
else
// sac da max roi ma van thua 
  {
//giam inverter
    giamINV();

  }


}

 void TH_thieu()
{
  //TH chua tang INV het muc 
 
 tangINV();
 
 if (dem_tang_INV > 50)
   
  {
    giamsac();

  }

}

void kiemtra()
{

  // int Rshunt = 50; 
  // int Vshunt = 75;
    int a;
    int tongap=0;
  
    
    for (a = 1;a<=10;a=a+1) {
              tongap = tongap + analogRead(doap); 
             // tongdong = tongdong + analogRead(dodong); 
                          
      }

    ap = (((tongap)/10.0)/4096.0 * 3.3 * Vcal/100.0)/ R2 * (R1+R2) ;
    //dong = (((tongdong)/10.0)/4096.0 * 3300.0 * Ical) /75.0 * 50.0 ;  
    //Blynk.virtualWrite(V1, ap);
   
  
}

void baove()
{
  //kiemtra();
  if (relaysac_status ||  relayhoaluoi_status) 
  {
    
    
   if (ap > maxV)
   {
      if (relaysac_status)
      {
       tat_sac(); 
      }
    }

    else if (ap < minV)
    {
      if (relayhoaluoi_status) 
      {
        tat_hoa_luoi();
      }
    }
double p_sosanh; 
if (pReal2 <0)
{p_sosanh = pReal2 * -1.0;}

else 

{p_sosanh = pReal2;} 

  if (p_sosanh > maxP)

  {
      if (relayhoaluoi_status )
      {
        tat_hoa_luoi();
        Blynk.virtualWrite(V1, "Tat hoa luoi do qua dong");
 

      }

      if (relaysac_status)
      {
       tat_sac(); 
       Blynk.virtualWrite(V1, "Tat sac do qua dong");
      }
 
      

  }

 

  }

}



void tat_sac()
{
  
  if (relaysac_status == bat)
  {
    if ((millis() - t_mo_sac) > 1000) //chong bat tat sac lien tuc
    {
      digitalWrite(relaysac,tat);
      relaysac_status = tat; 
      Blynk.virtualWrite(V1, "Tat sac");
      t_mo_sac = 0; 
    }
  }
}


void mo_sac()
{
  if (relaysac_status == tat)
  {
    kiemtra();
    if (ap < maxV)
    {
    digitalWrite(relaysac,bat);
    relaysac_status = bat; 
    t_mo_sac = millis();

    Blynk.virtualWrite(V1, "Mo sac");
    }
    else 
    {
      Blynk.virtualWrite(V1, "Pin da day");

    }
  }
}


void tat_hoa_luoi()
{
  
  digitalWrite(relayhoaluoi,tat);
  relayhoaluoi_status = tat; 
  Blynk.virtualWrite(V1, "Tat hoa luoi");
}


void mo_hoa_luoi()
{
  // kiemtra();
  // if (ap > minV)
  // {
    digitalWrite(relayhoaluoi,bat);
    relayhoaluoi_status = bat; 
    Blynk.virtualWrite(V1, "Mo hoa luoi");
  // }
  // else 
  // {
  //   Blynk.virtualWrite(V1, "Pin thap ap");
  // }
}

void dieukhiensacvahoaluoi()
{

if ((t_relay - millis()) > 1000)
  {
    printLocalTime();

  if (relayhoaluoi_status == 0)
  { 
    if ((hoaonh == gio) and (hoaonm == phut))
    {
      mo_hoa_luoi();
    }
  }
  else 
  {
    if ((hoaoffh == gio) and (hoaoffm == phut))
    {
      tat_hoa_luoi();
    }
  }  
  //dieu khien sac

  if (DRX > 0)
    {

      mo_sac();
    }

    else
    {
      tat_sac();
    }


  //het dieu khien sac

  }
}


void printLocalTime(){
  struct tm timeinfo;
  if(!getLocalTime(&timeinfo)){
    Serial.println("Failed to obtain time");
    return;
  }
  // Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
  // Serial.print("Day of week: ");
  // Serial.println(&timeinfo, "%A");
  // Serial.print("Month: ");
  // Serial.println(&timeinfo, "%B");
  // Serial.print("Day of Month: ");
  // Serial.println(&timeinfo, "%d");
  // Serial.print("Year: ");
  // Serial.println(&timeinfo, "%Y");
  Serial.print("Hour: ");
  Serial.println(&timeinfo, "%H");
  // Serial.print("Hour (12 hour format): ");
  // Serial.println(&timeinfo, "%I");
  Serial.print("Minute: ");
  Serial.println(&timeinfo, "%M");
  // Serial.print("Second: ");
  // Serial.println(&timeinfo, "%S");

  // Serial.println("Time variables");
  char timeHour[3];
  strftime(timeHour,3, "%H", &timeinfo);
  gio = atoi(timeHour);

  char timeMin[3];
  strftime(timeMin,3, "%M", &timeinfo);
  phut = atoi(timeMin);

  char timeDay[3];
  strftime(timeDay,3, "%d", &timeinfo);
  ngay = atoi(timeDay);


}


//lay gia tri cai dat 
BLYNK_WRITE(V1) {
// Called when the datastream V1 value changes

// Assign incoming value from pin V1 to a variable
// String pinValue = param.asStr()
String pinValue = param.asString();
String lenh = getValue(pinValue, ',', 0);

if (lenh =="sacon")
  {
    String l1 = getValue(pinValue, ',', 1);
    String l2 = getValue(pinValue, ',', 2);
    saconh = l1.toInt();
    saconm = l2.toInt();
    mystring = "Sac on " + l1 + ":" + l2 ;
    
    Blynk.virtualWrite(V1, mystring);
  }

else if (lenh =="sacoff")
  {
    String l1 = getValue(pinValue, ',', 1);
    String l2 = getValue(pinValue, ',', 2);
    sacoffh = l1.toInt();
    sacoffm = l2.toInt();
    
    mystring = "Sac off " + l1 + ":" + l2 ;
    
    Blynk.virtualWrite(V1, mystring);
    
  }
else if (lenh =="hoaon")
  {
    String l1 = getValue(pinValue, ',', 1);
    String l2 = getValue(pinValue, ',', 2);
    hoaonh = l1.toInt();
    hoaonm = l2.toInt();

    
    mystring = "Hoa on " + l1 + ":" + l2 ;
    
    Blynk.virtualWrite(V1, mystring);
    
  }
else if (lenh =="hoaoff")
  {
    String l1 = getValue(pinValue, ',', 1);
    String l2 = getValue(pinValue, ',', 2);
    hoaoffh = l1.toInt();
    hoaoffm = l2.toInt();

    
    mystring = "Hoa off " + l1 + ":" + l2 ;
    
    Blynk.virtualWrite(V1, mystring);
  }
else if (lenh =="maxsac")
  {
    String l1 = getValue(pinValue, ',', 1);
    maxsac = l1.toInt();
    
    mystring = "Max sac = " + l1;
    Blynk.virtualWrite(V1, mystring);
    }
else if (lenh =="maxI")
  {
    String l1 = getValue(pinValue, ',', 1);
    maxI = l1.toInt();
    
    mystring = "Max I = " + l1;
    Blynk.virtualWrite(V1, mystring);
   
  }
else if (lenh =="maxV")
  {
    String l1 = getValue(pinValue, ',', 1);
    maxV = l1.toFloat();
    
    mystring = "Max V = " + l1;
    Blynk.virtualWrite(V1, mystring);
    
  }
else if (lenh =="minV")
  {
    String l1 = getValue(pinValue, ',', 1);
    minV = l1.toFloat();
   
    mystring = "Min V = " + l1;
    Blynk.virtualWrite(V1, mystring);
   }
else if (lenh =="R1")
  {
    String l1 = getValue(pinValue, ',', 1);
    R1 = l1.toFloat();
   
    mystring = "R1 = " + l1;
    Blynk.virtualWrite(V1, mystring);
   }

else if (lenh =="R2")
  {
    String l1 = getValue(pinValue, ',', 1);
    R2 = l1.toFloat();
   
    mystring = "R2 = " + l1;
    Blynk.virtualWrite(V1, mystring);
   }

else if (lenh =="Vcal")
  {
    String l1 = getValue(pinValue, ',', 1);
    Vcal = l1.toFloat();
   
    mystring = "V cal = " + l1;
    Blynk.virtualWrite(V1, mystring);
   }

else if (lenh =="Ical")
  {
    String l1 = getValue(pinValue, ',', 1);
    Ical = l1.toFloat();
   
    mystring = "I cal = " + l1;
    Blynk.virtualWrite(V1, mystring);
   }

else if (lenh =="lp1")
  {
    String l1 = getValue(pinValue, ',', 1);
    lech_pha = l1.toInt();
   
    mystring = "LP1 = " + l1;
    Blynk.virtualWrite(V1, mystring);
   }
else if (lenh =="lp2")
  {
    String l1 = getValue(pinValue, ',', 1);
    lech_pha2 = l1.toInt();
   
    mystring = "LP2 = " + l1;
    Blynk.virtualWrite(V1, mystring);
   }

else if (lenh =="pcal1")
  {
    String l1 = getValue(pinValue, ',', 1);
    p_cal1 = l1.toFloat();
   
    mystring = "LP2 = " + l1;
    Blynk.virtualWrite(V1, mystring);
   }

else if (lenh =="pcal2")
  {
    String l1 = getValue(pinValue, ',', 1);
    p_cal2 = l1.toFloat();
   
    mystring = "LP2 = " + l1;
    Blynk.virtualWrite(V1, mystring);
   }

else if (lenh =="fw")
  {
 
    
    mystring = "Update FW";
    Blynk.virtualWrite(V1, mystring);
    FirmwareUpdate();
   }
else if (lenh =="ml")
  {
 
    
    Blynk.virtualWrite(V1, millis());
    
   }


else 
  {
    mystring = "lenh sai cau truc";
    Blynk.virtualWrite(V1, mystring);


  }

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

void tinhWH()
{
//    //tinh P cong don

tg = millis()- t_do;
t_do = millis(); 


if (relaysac_status ==bat)

  {psac_accu = psac_accu + (pTai2 * tg/3600000.0); }

else if (relayhoaluoi_status == bat)
  {phoa_accu = phoa_accu + (pTai2 * tg/3600000.0); }





ptai_accu = ptai_accu + pTai * tg / 3600000.0;


if (ngay != ngay_hien_tai) 
{
//reset het P
psac_accu = 0.0;
phoa_accu = 0.0;
ptai_accu = 0.0;
} 
ngay_hien_tai = ngay;


//    //het code P cong don
}