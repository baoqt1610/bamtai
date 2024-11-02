//khai bao dieu khien den

//khai bao dimmer
#include <RBDdimmer.h>//

#define outputPin  26 
#define zerocross  25 // for boards with CHANGEBLE input pins

dimmerLamp dimmer(outputPin, zerocross); //initialase port for dimmer for ESP8266, ESP32, Arduino due boards
//dimmerLamp dimmer(outputPin); //initialase port for dimmer for MEGA, Leonardo, UNO, Arduino M0, Arduino Zero

int TH = 0;
int TH_current = 0;
//het khai bao dimmer



int ptden = 0;


//het khai bao dieu khien den




//khai bao on 
float R1 = 97.4; 
float R2 = 19.68;
float Vcal= 108;
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
#define BLYNK_TEMPLATE_ID "TMPL6qvbtmaFy"
#define BLYNK_TEMPLATE_NAME "Ho Long"
#define BLYNK_AUTH_TOKEN "L1H5YsNAzfO2bSYy9SJEv7ElLrB7K-M1"
#include <Arduino.h>
#define BLYNK_PRINT Serial
#include <WiFi.h>
#include <BlynkSimpleEsp32.h>

String mystring; 

char auth[] = BLYNK_AUTH_TOKEN;



//het khai bao blynk

//update code through Github


#include <SPIFFS.h>
#include "Update.h"
#include <WiFiClientSecure.h>


// Define server details and file path
#define HOST "raw.githubusercontent.com"
#define PATH "/baoqt1610/bamtai/main/bam_tai.bin"
#define PORT 443

// Define the name for the downloaded firmware file
#define FILE_NAME "bam_tai.bin"


//het khai bao update code through Github


#include <time.h>
const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = 25200;
const int   daylightOffset_sec = 0;




int ptsac;
bool nhan = 0; 
unsigned long t1,t2,t3,t_bat_dau_cho, t_relay, t_mo_sac, t_reset = 0;

//decltype(millis()) t1,t2,t3,t_bat_dau_cho, t_relay, t_mo_sac = 0;
decltype(millis()) t_do  = 0;
//decltype(millis()) t_reset  = 0;
decltype(millis()) tg;
decltype(millis()) tg1;

int quadong_status = 0;
const int so_mau = 500; 
int lech_pha = 7; 
int lech_pha2 = 6; 
uint16_t Vsample[so_mau];
uint16_t Isample[so_mau];
uint16_t I2sample[so_mau];

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

byte dl;
int dem_tang_INV=0;
//bien dieu khien 
int saconh= 7; 
int saconm= 0;
int sacoffh = 16;
int sacoffm = 0;

int hoaonh= 16; 
int hoaonm= 0;
int hoaoffh = 7;
int hoaoffm = 0;

float minV = 12.0;
float maxV = 14.8;
float maxI = 50.0; 
float maxP = 1500.0; 

//

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

RF24 radio(4, 5); // CE, CSN
const byte diachi[][6] = {"12345", "10000"}; //0, 1

//SSID of your network
// char ssid[] = "Xiaomi_3D3B";
// //password of your WPA Network
// char pass[] = "0909051109";

// khai bao sac PWM

const int ledPin = 14;  // 16 corresponds to GPIO16

// setting PWM properties
const int freq = 5000;
const int ledChannel = 0;
const int resolution = 8;
int DRX = 0; 
int maxsac = 100;
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
  Serial.println("version 22Sep24");
//  WiFi.begin(ssid, pass);
//    Blynk.config(auth);
  // Init and get the time
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  printLocalTime();

  //het khoi tao thoi gian

  // khoi tao wifi manager + blynk
 EEPROM.begin(512);
 WiFiManager wifiManager;
 wifiManager.setConfigPortalTimeout(60);
 //wifiManager.resetSettings();    //Uncomment this to wipe WiFi settings from EEPROM on boot.  Comment out and recompile/upload after 1 boot cycle.
 wifiManager.autoConnect("ESP32");
 //if you get here you have connected to the WiFi
 Serial.println("connected...ok :)");
 Blynk.begin(auth, WiFi.SSID().c_str(), WiFi.psk().c_str());

 


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

//khoi tao update github

 //het khoi tao github update
   dimmer.begin(NORMAL_MODE, OFF); //dimmer initialisation: name.begin(MODE, STATE) 


}

void loop() 
{
 

  if (WiFi.status() == WL_CONNECTED) {      //neu ket noi thanh cong thi in ra connected!

       Blynk.run();
      }
  if ((millis() - t1) > 60000)
  {
    
    
    if (WiFi.status() != WL_CONNECTED) {      //neu ket noi thanh cong thi in ra connected!
              Serial.println("Mat ket noi wifi, dang ket noi lai");
            
              Blynk.config(auth);
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
            //delayMicroseconds(10) ;
            
            //delayMicroseconds(10) ;
            Vsample[i] = analogRead(adcPinV);
            //delayMicroseconds(10) ;
            
    } 
     
    for (i = 1;i<=so_mau-lech_pha;i=i+1) {
            sampleI = Isample[i+lech_pha];
            
            sampleV = Vsample[i];
            // Serial.print(sampleV);
            // Serial.print(",");
            // Serial.println(sampleI);
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
               float pReal = (V_RATIO * I_RATIO * sumP1 / (so_mau-lech_pha));
               pReal = pReal * 80 /100; // dieu chinh tang len 100w

               
              Serial.print("P thuc: ");
              Serial.println(pReal);
               
              //  Serial.print("Dong: ");
              //  Serial.println(iTai1);
              //  Serial.print("Vol: ");
              //  Serial.println(Vol1);
              //  Serial.print("Power factor: ");
              //  Serial.println(pf);
            
    
                  //code dieu khien den moi
              //gui qua bam tai 
//code dieu khien
          //phat mot it len luoi 100w
             if (pReal < -15)
    
            {
                TH = TH + 1;
                Serial.println("tang den");
                 if (TH > 100)
                   {TH = 100;}   
              }
                      
            else 
            {
             TH = TH - 1;
             Serial.println("giam den");
            if (TH < 0)
            {TH = 0;}

            }

             int ptden = map(TH, 0, 100, 15, 90);
              if ((ptden > 15) and (ptden != TH_current))
              {
                Serial.print("bat den: ");
                Serial.println(ptden);
                dimmer.setState(ON);
                dimmer.setPower(ptden); // name.setPower(0%-100%)

                TH_current = ptden;
              }
              else if (ptden <= 15)
              {
                dimmer.setState(OFF);
                Serial.println("tat den: ");

              }
              //Serial.print("% den: ");
              //Serial.println(TH);
                //het code dieu khien den moi
    

 

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
   pTai = pTai * 80 /100;
   
   iTai = I_RATIO * sqrt(sumI / ((so_mau-lech_pha) * j));
   Vol = V_RATIO * sqrt(sumV / ((so_mau-lech_pha) * j));
   
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
  double sdata =  TH;

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
  if (!SPIFFS.begin(true)) {
    Serial.println("SPIFFS Mount Failed");
    return;
  }

  //connectToWiFi();
  getFileFromServer();
  performOTAUpdateFromSPIFFS();

    
    

  
 }

 void tangsac()
{
  DRX = DRX + 1; 
 // Serial.println("tang sac");

  if (DRX > maxsac)
    {DRX = maxsac;}

  else if (DRX < 0)
   {DRX = 0;}

}

void giamsac()
{

  DRX = DRX - 1; 
 // Serial.println("giam sac");
  
  //chinh ve max sac
  if (DRX > maxsac)
  {DRX = maxsac;}

  else if (DRX < 0)
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

//  Serial.println("tang INV");
   

}


void giamINV()
{
  
  dl = 1; 
  //Serial.println("giam INV");
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
       giamsac(); 
       //Blynk.virtualWrite(V1, "Tat sac do qua dong");
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
  if ((relaysac_status == tat) && (relayhoaluoi_status == tat))
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
  //Serial.print("Hour: ");
  //Serial.println(&timeinfo, "%H");
  // Serial.print("Hour (12 hour format): ");
  // Serial.println(&timeinfo, "%I");
  //Serial.print("Minute: ");
  //Serial.println(&timeinfo, "%M");
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

  // Serial.print("Gio: ");
  // Serial.println(gio);
  // Serial.print("Phut: ");
  // Serial.println(phut);




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

else if (lenh =="maxP")
  {
    String l1 = getValue(pinValue, ',', 1);
    maxP = l1.toFloat();
   
    mystring = "MaxP = " + l1;
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

else if (lenh =="mohoaluoi")
  {
 
    
    mo_hoa_luoi();
        
   }
else if (lenh =="tathoaluoi")
  {
 
    
    tat_hoa_luoi();
        
   }

else if (lenh =="mosac")
  {
 
    
    mo_sac();
        
   }
else if (lenh =="tatsac")
  {
 
    
    tat_sac();
        
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


if ((gio==6) && (phut ==0)) 
{
//reset het P
psac_accu = 0.0;
} 
if ((gio==8) && (phut ==0)) 
{
//reset het P
phoa_accu = 0.0;
ptai_accu = 0.0;
}

//    //het code P cong don
}

void khoidonglaiRF24() 
{
  //const unsigned int interval = 60000;
  
  int kt = millis() - t_reset;
  if ( kt > 3600000)
  {  radio.powerDown();
    //Serial.println(" 1 Tx1 problem end");
    radio_init();
    radio.powerUp();
    // Serial.println("Da reset RF24");
    // Blynk.virtualWrite(V1, "RF24");
    t_reset = millis(); 

  }
  
}


void radio_init()
{
  pinMode(4, OUTPUT);
  digitalWrite(4, HIGH);
  pinMode(5, OUTPUT);
  digitalWrite(5, HIGH);
  delay(10);
 
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
  //radio.setAutoAck(false);
  radio.setPALevel(RF24_PA_MAX); //Cài bộ khuyết địa công suất ở mức MIN
  radio.setChannel(80);
  radio.setDataRate(RF24_250KBPS);   
  radio.printDetails(); 
 
  if (!radio.available())
  {
    Serial.println("Chưa kết nối được với RX...!!");
    Serial.println("CHỜ KẾT NỐI.......");
  }

}



void getFileFromServer() {
  WiFiClientSecure client;
  client.setInsecure(); // Set client to allow insecure connections

  if (client.connect(HOST, PORT)) { // Connect to the server
    Serial.println("Connected to server");
    client.print("GET " + String(PATH) + " HTTP/1.1\r\n"); // Send HTTP GET request
    client.print("Host: " + String(HOST) + "\r\n"); // Specify the host
    client.println("Connection: close\r\n"); // Close connection after response
    client.println(); // Send an empty line to indicate end of request headers

    File file = SPIFFS.open("/" + String(FILE_NAME), FILE_WRITE); // Open file in SPIFFS for writing
    if (!file) {
      Serial.println("Failed to open file for writing");
      return;
    }

    bool endOfHeaders = false;
    String headers = "";
    String http_response_code = "error";
    const size_t bufferSize = 1024; // Buffer size for reading data
    uint8_t buffer[bufferSize];

    // Loop to read HTTP response headers
    while (client.connected() && !endOfHeaders) {
      if (client.available()) {
        char c = client.read();
        headers += c;
        if (headers.startsWith("HTTP/1.1")) {
          http_response_code = headers.substring(9, 12);
        }
        if (headers.endsWith("\r\n\r\n")) { // Check for end of headers
          endOfHeaders = true;
        }
      }
    }

    Serial.println("HTTP response code: " + http_response_code); // Print received headers

    // Loop to read and write raw data to file
    while (client.connected()) {
      if (client.available()) {
        size_t bytesRead = client.readBytes(buffer, bufferSize);
        file.write(buffer, bytesRead); // Write data to file
      }
    }
    file.close(); // Close the file
    client.stop(); // Close the client connection
    Serial.println("File saved successfully");
  }
  else {
    Serial.println("Failed to connect to server");
  }
}

void performOTAUpdateFromSPIFFS() {
  // Open the firmware file in SPIFFS for reading
  File file = SPIFFS.open("/" + String(FILE_NAME), FILE_READ);
  if (!file) {
    Serial.println("Failed to open file for reading");
    return;
  }

  Serial.println("Starting update..");
  size_t fileSize = file.size(); // Get the file size
  Serial.println(fileSize);

  // Begin OTA update process with specified size and flash destination
  if (!Update.begin(fileSize, U_FLASH)) {
    Serial.println("Cannot do the update");
    return;
  }

  // Write firmware data from file to OTA update
  Update.writeStream(file);

  // Complete the OTA update process
  if (Update.end()) {
    Serial.println("Successful update");
  }
  else {
    Serial.println("Error Occurred:" + String(Update.getError()));
    return;
  }

  file.close(); // Close the file
  Serial.println("Reset in 4 seconds....");
  delay(4000);
  ESP.restart(); // Restart ESP32 to apply the update
}