#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

RF24 radio(9, 10); // CE, CSN
const byte diachi[][6] = {"12345", "10000"}; //0, 1


const int so_mau = 80; 
const int lech_pha = 7; 
int Vsample[so_mau];
int Isample[so_mau];

int DRX; 
const int maxsac = 100; 
int dem_tang_INV = 0;
int relaysac_status; 
const int tat = 0; 
const int bat = 1; 
unsigned long t_mo_sac = 0;
unsigned long t_bat_dau_cho = 0;
int ptsac,nhan;
const int relaysac = 8;
const int pwmsac = 6; 
int dl; 
//het code measure supply voltage


//
#include <SendOnlySoftwareSerial.h>
SendOnlySoftwareSerial mySerial(2);  // Tx pin

//#include <SoftwareSerial.h>
//SoftwareSerial mySerial(3,2);  // Tx pin


String myString=""; // complete message from arduino, which consistors of snesors data
String cdata; 



const byte adcPin = A1; // = 14 (pins_arduino.h) cam bien dong inverter
const byte adcPinV = A0; // = 14 (pins_arduino.h) cam bien ap


double offsetI;
double sampleI;
double filteredI;
double sqI;
double sumI;
double sumI1, sumV1, sumP1;


float ICAL = 65.3; // CAM BIEN CU
//float ICAL = 22; // CAM BIEN MOI
double iTai, Vol;

//khai bao cam bien voltage

 const int VCAL = 234;

 int sampleV;
 double filteredV;          //Filtered_ is the raw analog value minus the DC offset
 double offsetV;                          //Low-pass filter output
 double sqV;
 double sumV;
 double instP,sumP, pTai; 

//het khai bao bien cam bien vol

double I_RATIO = ICAL *((5000/1000.0) / (1024));
double V_RATIO = VCAL *((5000/1000.0) / (1024));


void setup() {
  


pinMode(adcPin,INPUT);
pinMode(adcPin,INPUT);
//het khoi tao den moi


Serial.begin(9600); 
mySerial.begin(9600); 

Serial.println("ADC without blocking");

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
  if (!radio.available())
  {
    Serial.println("Chưa kết nối được với RX...!!");
    Serial.println("CHỜ KẾT NỐI.......");
  }



offsetI = 512;
offsetV = 512;

}

void loop() {



//STEP 2: get samples

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
            offsetI = (offsetI + (sampleI-offsetI)/1024);
            filteredI = sampleI - offsetI;
        
            // Root-mean-square method current
            // 1) square current values
            sqI = filteredI * filteredI;
            // 2) sum
            sumI += sqI;
            sumI1 += sqI;
            
            offsetV = offsetV + ((sampleV-offsetV)/1024);
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
               
              //  Serial.print("Dong: ");
              //  Serial.println(iTai1);
              //  Serial.print("Vol: ");
              //  Serial.println(Vol1);
              //  Serial.print("Power factor: ");
              //  Serial.println(pf);
            
    
    //code dieu khien den moi
    
      
//code dieu khien den moi
    
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
    analogWrite(pwmsac, sac);

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
          //Blynk.virtualWrite(V1, ptsac);
          


          nhan = 1;
          break;
          }
          
          }
             

    sumP1 = 0; 
    sumI1 = 0; 
    sumV1 = 0; 
       
       }      

   
  //  Serial.print(filteredI);
  //  Serial.print(" ");
  //  Serial.println(filteredV);
 

    } // dong vong lap for i

if (j == 20)
{
  //tinh vua cap nhat blynk
  
   pTai = V_RATIO * I_RATIO * sumP / ((so_mau-lech_pha)* j);
   iTai = I_RATIO * sqrt(sumI / ((so_mau-lech_pha) * j));
   Vol = V_RATIO * sqrt(sumV / ((so_mau-lech_pha) * j));
   double appPower1 = iTai * Vol;
   double pf1 = pTai/ appPower1;
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
   
    //gui du lieu qua esp 
      cdata = cdata + pTai + "," + ptsac + "," + Vol + "," + DRX; // comma will be used a delimeter
      // Serial.println(cdata);
      mySerial.println(cdata);
      // Serial.println("qua day");
      cdata = "";

  
    
  
  //het code tinh va cap nhat blynk
  }

} // dong vong lap for j



//het get samples

} //het loop


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

// void kiemtra()
// {

//   // int Rshunt = 50; 
//   // int Vshunt = 75;
//     int a;
//     int tongap=0;
  
    
//     for (a = 1;a<=10;a=a+1) {
//               tongap = tongap + analogRead(doap); 
//              // tongdong = tongdong + analogRead(dodong); 
                          
//       }

//     ap = (((tongap)/10.0)/4096.0 * 3.3 * Vcal/100.0)/ R2 * (R1+R2) ;
//     //dong = (((tongdong)/10.0)/4096.0 * 3300.0 * Ical) /75.0 * 50.0 ;  
//     //Blynk.virtualWrite(V1, ap);
   
  
// }


void tat_sac()
{
  
  if (relaysac_status == bat)
  {
    if ((millis() - t_mo_sac) > 1000) //chong bat tat sac lien tuc
    {
      digitalWrite(relaysac,tat);
      relaysac_status = tat; 
      
      t_mo_sac = 0; 
    }
  }
}


void mo_sac()
{
  if (relaysac_status == tat)
  {

    digitalWrite(relaysac,bat);
    relaysac_status = bat; 
    t_mo_sac = millis();

    
  }
}



void dieukhiensacvahoaluoi()
{


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

 
