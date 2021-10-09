
//~~~~~~~~~~~~ Liberary

// For PWM
#include <TimerOne.h>

// For GSM
#include <SoftwareSerial.h>

// Ethernet
#include <Ethernet.h>

// Temp
#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"

// Spo2
#include "spo2_algorithm.h"

// Display Deff
#include <TechShopBDScreen.h>
#include <TechShopBDTouch.h>

// Define State
#define off false
#define on true

// GSM SS pinMode
SoftwareSerial gprsSerial(9, 10);

// Ethernet Connection
bool connectionOk = false;

// Net On/off
bool SW_Ethernet = off;
//bool SW_GSM_SMS = off;
bool SW_GSM_NET = on;

// Screen And Touch Config
TechShopBDScreen myGLCD(ILI9325D_16, 38, 39, 40, 41);
TechShopBDTouch myTouch(42, 43, 44, 45, 46);

// Touch On/off
bool Enter_Touch = off;
bool Back_Touch = off;
bool Option_Touch = off;
bool Refresh_Touch = off;
bool EnableEthCheck = off;
// Declare which fonts we will be using
extern uint8_t BigFont[];

int x, y;
//End of Display Deff

#define MAX_BRIGHTNESS 255

// temp
float temperatureF;

// init Var
int g = 0;
// ............. Ecg
int pulse[1000];
int count = 1;
int cc = 0;
int Define = 300;

//~~~~~~~~~~~~~~~~~~~ Ethernet Data
byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED}; // RESERVED MAC ADDRESS
EthernetClient client;
IPAddress server(192, 168, 0, 100);
////////////////////////////////////////

//~~~~~~~~~~~~~~~~~~~~~~ temp  Function Define
MAX30105 particleSensorTemp;
MAX30105 particleSensor;

// ~~~~~~~~~~~~~~~~~~~~~ HeatBeet Sensor Calculation Variable
const byte RATE_SIZE = 4; //Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE];    //Array of heart rates
byte rateSpot = 0;
long lastBeat = 0; //Time at which the last beat occurred

float beatsPerMinute;
int beatAvg;

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// spo2

#if defined(ARDUINO_AVR_UNO)
//Arduino Uno doesn't have enough SRAM to store 100 samples of IR led data and red led data in 32-bit format
//To solve this problem, 16-bit MSB of the sampled data will be truncated. Samples become 16-bit data.
uint16_t irBuffer[100];  //infrared LED sensor data
uint16_t redBuffer[100]; //red LED sensor data
#else
uint32_t irBuffer[100];  //infrared LED sensor data
uint32_t redBuffer[100]; //red LED sensor data
#endif

int32_t bufferLength;  //data length
int32_t spo2;          //SPO2 value
int8_t validSPO2;      //indicator to show if the SPO2 calculation is valid
int32_t heartRate;     //heart rate value
int8_t validHeartRate; //indicator to show if the heart rate calculation is valid

// .....................................................................................Setup data
void setup()
{

  // GSM Freq Band
  gprsSerial.begin(19200);

  //init Property
  Serial.begin(9600);
  pinMode(13, INPUT); // Setup for leads off detection LO +
  pinMode(12, INPUT); // Setup for leads off detection LO -

  // Display init
  myGLCD.InitLCD();
  myGLCD.clrScr();

  myTouch.InitTouch();
  myTouch.setPrecision(PREC_MEDIUM);

  myGLCD.setFont(BigFont);
  myGLCD.setBackColor(0, 0, 0);

  myGLCD.print("Initializing Program", CENTER, 100);

  Serial.println("Initializing Program");

  Serial.println();

  EtherINIT();

  //Temp Sensor Data Get and detect ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

  if (!particleSensorTemp.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
    Serial.println("MAX30105 was not found. Please check wiring/power. ");
    while (1);
  }
 
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
    Serial.println("MAX30105 was not found. Please check wiring/power. ");
    while (1);
  }
  //particleSensor.setup(0); //Configure sensor. Turn off LEDs

  ///////////////////////////////////////////////////////////////////////////////////
  
  
  // heatBet Rate Define
  //particleSensor.setup(); //Configure sensor with default settings
  particleSensor.setPulseAmplitudeRed(0x0A); //Turn Red LED to low to indicate sensor is running
  particleSensor.setPulseAmplitudeGreen(0);  //Turn off Green LED

  // spo2

  // Serial.println(F("Attach sensor to finger with rubber band. Press any key to start conversion"));
  //while (Serial.available() == 0) ; //wait until user presses a key
  Serial.read();
  
  byte ledBrightness = 60; //Options: 0=Off to 255=50mA
  byte sampleAverage = 4;  //Options: 1, 2, 4, 8, 16, 32
  byte ledMode = 2;        //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  byte sampleRate = 100;   //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 411;    //Options: 69, 118, 215, 411
  int adcRange = 4096;     //Options: 2048, 4096, 8192, 16384

  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); //Configure sensor with these settings

  // Display init
 
  
  // Display The page
  myGLCD.clrScr();
  Start_Page();

  Timer1.initialize(500); // initialize timer1, and set a 1/2 second period
  Timer1.attachInterrupt(Touch_AT_DAT);
  
}

// Loop Code
void loop()
{
 
  
  TempDataGet();
  
  
  Heart_Beet_Rate();
  
  
  // Spo2 Data
  spo2DataGet();
  
  
  // ECG data
  ECGDataGet();
 
  
  for(int iiiii = 1 ; iiiii <= 10000; iiiii++){
    if(iiiii == 10000 ){
      EnableEthCheck = true;
    }
  }

  if(EnableEthCheck == true){
    EnableEthCheck = false;
    EtherINIT();
    
  }
}

//Touch Setting

void EtherINIT()
{
  //Ethernet Data Get ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  Ethernet.begin(mac);
  Serial.println("Initializing Ethernet.");
  Serial.print("IP Address        : ");
  Serial.println(Ethernet.localIP());
  if (client.connect(server, 80))
  {
    Serial.println("Connected");
    client.println();
    client.println();
    client.stop();
    SW_Ethernet = on;
  }
  else
  {
    // you didn't get a connection to the server:
    SW_Ethernet = off;
    Serial.println("\nConnection failed\n");
  }

  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
}

// Other Function

void Heart_Beet_Rate()
{
  // HeatBet Sensor And DB entry
  for (g = 1; g < 3; g++)
  {
    HeatBeetSensor();
    Touch_AT_DAT();
  }
  //////////////////////////////////
  // insert Net HeatBeetSensor
  if (SW_Ethernet == true)
  {
    if (client.connect(server, 80))
    {
      //Serial.println("\nConnected\n");
      // Make a HTTP request:
      client.print("GET /HealthMonitor/SensorDataGetPHP/heartbeet.php?");
      client.print("BPM=");
      client.print(beatsPerMinute);
      client.print("&BPMAVG=");
      client.print(beatAvg);

      client.println(" HTTP/1.1");
      client.print("Host: ");
      client.println(server);
      client.println("Connection: close");
      client.println();
      client.println();
      client.stop();
    }
  }
  else
  {
    String str = "heartbeet.php?BPM=" + (String)beatsPerMinute + "&BPMAVG=" + (String)beatAvg;
    gsm(str);    // Waiting For Config
  }
}


  // ECG Data
  void ECGDataGet()
  {

    if ((digitalRead(13) == 1) || (digitalRead(12) == 1))
    {
      Serial.println('!');
    }
    else
    {
      for (cc = 0; cc <= Define; cc++)
      {
        Serial.println(analogRead(A0));
        pulse[cc] = analogRead(A0);
        count++;
      }
      if (count >= Define)
      {
        for (cc = 0; cc < Define; cc++)
        {
          if (SW_Ethernet == true)
          {
            if (client.connect(server, 80))
            {

              client.print("GET /HealthMonitor/SensorDataGetPHP/ecg.php?");
              client.print("ecg=");
              client.print(pulse[cc]);
              client.println(" HTTP/1.1");
              client.print("Host: ");
              client.println(server);
              client.println("Connection: close");
              client.println();
              client.println();
              client.stop();
              count = 0;
            }
          }
          else
          {
            String str = "ecg.php?ecg=" + (String)pulse[cc];
            gsm(str);  // Waiting For Config
            delay(1);
          }
          Serial.println("Number of Data Insert ");
          Serial.print(cc);
        }
      }

      delay(1);
    }
  }

  // Spo2 data
  void spo2DataGet()
  {

    bufferLength = 100; //buffer length of 100 stores 4 seconds of samples running at 25sps

    //read the first 100 samples, and determine the signal range
    for (byte i = 0; i < bufferLength; i++)
    {
      while (particleSensor.available() == false) //do we have new data?
        particleSensor.check();                   //Check the sensor for new data

      redBuffer[i] = particleSensor.getRed();
      irBuffer[i] = particleSensor.getIR();
      particleSensor.nextSample(); //We're finished with this sample so move to next sample

      Serial.print(F("red="));
      Serial.print(redBuffer[i], DEC);
      Serial.print(F(", ir="));
      Serial.println(irBuffer[i], DEC);
    }

    //calculate heart rate and SpO2 after first 100 samples (first 4 seconds of samples)
    maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);
    int f = 0;
    //Continuously taking samples from MAX30102.  Heart rate and SpO2 are calculated every 1 second
    do
    {
      //dumping the first 25 sets of samples in the memory and shift the last 75 sets of samples to the top
      for (byte i = 25; i < 100; i++)
      {
        redBuffer[i - 25] = redBuffer[i];
        irBuffer[i - 25] = irBuffer[i];
      }

      //take 25 sets of samples before calculating the heart rate.
      for (byte i = 75; i < 100; i++)
      {
        while (particleSensor.available() == false) //do we have new data?
          particleSensor.check();                   //Check the sensor for new data

        //Blink onboard LED with every data read

        redBuffer[i] = particleSensor.getRed();
        irBuffer[i] = particleSensor.getIR();
        particleSensor.nextSample(); //We're finished with this sample so move to next sample

        //send samples and calculation result to terminal program through UART
        Serial.print(F("red="));
        Serial.print(redBuffer[i], DEC);
        Serial.print(F(", ir="));
        Serial.print(irBuffer[i], DEC);

        Serial.print(F(", HR="));
        Serial.print(heartRate, DEC);

        Serial.print(F(", HRvalid="));
        Serial.print(validHeartRate, DEC);

        Serial.print(F(", SPO2="));
        Serial.print(spo2, DEC);

        Serial.print(F(", SPO2Valid="));
        Serial.println(validSPO2, DEC);

        ///......................................................
        if (SW_Ethernet==true)
        {
          if (client.connect(server, 80))
          {
            //Serial.println("-> Connected");
            // Make a HTTP request:
            client.print("GET /HealthMonitor/SensorDataGetPHP/spo.php?");
            client.print("HR=");
            client.print(heartRate);
            client.print("&HRvalid=");
            client.print(validHeartRate);
            client.print("&SPO2=");
            client.print(spo2);
            client.print("&SPO2Valid=");
            client.print(validSPO2);
            client.println(" HTTP/1.1");
            client.print("Host: ");
            client.println(server);
            client.println("Connection: close");
            client.println();
            client.println();
            client.stop();
          }
        }
        else
        {
          String str = "spo.php?HR=" + (String)heartRate +"&HRvalid=" + (String)validHeartRate +  "&SPO2="  + (String)spo2 + "&SPO2Valid=" + (String)validSPO2;
          gsm(str);     // Waiting For Config
        }

        //After gathering 25 new samples recalculate HR and SP02
        maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);
      }
      
       
    }while (f == 80000);
  }

  // Get Temp Data From Sensor

  void TempDataGet()
  {
    //Touch_AT_DAT();

    float temperature = particleSensorTemp.readTemperature();
    Serial.print("temperatureC=");
    Serial.print(temperature, 4);

    temperatureF = particleSensorTemp.readTemperatureF(); //Because I am a bad global citizen

    Serial.print(" temperatureF=");
    Serial.print(temperatureF, 4);
    //Send Data  To server

    //Insert net
    if (SW_Ethernet == true)
    {
      if (client.connect(server, 80))
      {
        //Serial.println("-> Connected");
        // Make a HTTP request:
        client.print("GET /HealthMonitor/SensorDataGetPHP/temp.php?");
        client.print("tempC=");
        client.print(temperature);
        client.print("&tempF=");
        client.print(temperatureF);
        client.println(" HTTP/1.1");
        client.print("Host: ");
        client.println(server);
        client.println("Connection: close");
        client.println();
        client.println();
        client.stop();
      }
    }
    else
    {
      Serial.println("Sending GSM data");
      String str = "temp.php?tempC=" + (String)temperature + "&tempF=" + temperatureF;
      gsm(str);
    }

    Serial.println();
    delay(100);

    Touch_AT_DAT();
  }

  void HeatBeetSensor()
  {
    Touch_AT_DAT();
    long irValue = particleSensor.getIR();

    if (checkForBeat(irValue) == true)
    {
      //We sensed a beat!
      long delta = millis() - lastBeat;
      lastBeat = millis();

      beatsPerMinute = 60 / (delta / 1000.0);

      if (beatsPerMinute < 255 && beatsPerMinute > 20)
      {
        rates[rateSpot++] = (byte)beatsPerMinute; //Store this reading in the array
        rateSpot %= RATE_SIZE;                    //Wrap variable

        //Take average of readings
        beatAvg = 0;
        for (byte xs = 0; xs < RATE_SIZE; xs++)
          beatAvg += rates[xs];
        beatAvg /= RATE_SIZE;
      }
    }

    Serial.print("IR=");
    Serial.print(irValue);
    Serial.print(", BPM=");
    Serial.print(beatsPerMinute);
    Serial.print(", Avg BPM=");
    Serial.print(beatAvg);

    if (irValue < 50000)
      Serial.print(" No finger?");

    Serial.println();

    // Ethernet data insert at loop
  }

  // Display Function
  void Start_Page()
  {
    // The 1st Render Page
    myGLCD.clrScr();
    myGLCD.setColor(255, 255, 255);
    myGLCD.print("Welcome", CENTER, 80);
    myGLCD.print("To", CENTER, 100);
    myGLCD.print("Health Assista", CENTER, 120);

    delay(1000);
    Option_Page();
  }

  // Option Page
  void Option_Page()
  {

    myGLCD.clrScr();
    myGLCD.setColor(255, 255, 255);
    myGLCD.print("Health Assista", CENTER, 10);

    // 1st Row
    // Button MyHealth
    myGLCD.setColor(255, 255, 255);
    myGLCD.fillRoundRect(30, 40, 150, 80); //( x1 = x-axis start point , y1 = y-axis start point,x2 = x-axis end point , y2 = y-axis end point  )
    myGLCD.setColor(255, 255, 255);
    myGLCD.print("Health", 45, 55); // ("Text",x,y)
    myGLCD.setColor(0, 0, 255);

    // Button Doctors
    myGLCD.setColor(255, 255, 255);
    myGLCD.fillRoundRect(170, 40, 310, 80); //( x1 = x-axis start point , y1 = y-axis start point,x2 = x-axis end point , y2 = y-axis end point  )
    myGLCD.setColor(255, 255, 255);
    myGLCD.print("Doctors", 180, 55); // ("Text",x,y)
    myGLCD.setColor(0, 0, 255);

    // 2nd Row
    // Button Help
    myGLCD.setColor(255, 255, 255);
    myGLCD.fillRoundRect(30, 100, 150, 140); //( x1 = x-axis start point , y1 = y-axis start point,x2 = x-axis end point , y2 = y-axis end point  )
    myGLCD.setColor(255, 255, 255);
    myGLCD.print("Help", 60, 115); // ("Text",x,y)
    myGLCD.setColor(0, 0, 255);

    // Button About
    myGLCD.setColor(255, 255, 255);
    myGLCD.fillRoundRect(170, 100, 310, 140); //( x1 = x-axis start point , y1 = y-axis start point,x2 = x-axis end point , y2 = y-axis end point  )
    myGLCD.setColor(255, 255, 255);
    myGLCD.print("About", 200, 115); // ("Text",x,y)
    myGLCD.setColor(0, 0, 255);

    // Button Emergency
    myGLCD.setColor(255, 255, 255);
    myGLCD.fillRoundRect(70, 160, 260, 200); //( x1 = x-axis start point , y1 = y-axis start point,x2 = x-axis end point , y2 = y-axis end point  )
    myGLCD.setColor(255, 255, 255);
    myGLCD.print("Emergency", 90, 175); // ("Text",x,y)
    myGLCD.setColor(0, 0, 255);

    if (connectionOk)
    {
      myGLCD.print("Connected To Ether", CENTER, 200);
    }

    // Touch config
    Back_Touch = off;
    Enter_Touch = off;
    Option_Touch = on;
  }

  // Health Monitoring Section
  void Health_Page()
  {

    if(temperatureF > 105  ){
      temperatureF = 0;
    }
    if(heartRate > 200  ){
      heartRate = 0;
    }
    if(spo2 > 100 ){
      spo2 = 0;
    }
    
    myGLCD.setColor(255, 255, 255);
    myGLCD.print("Temperature", 20, 20);
    myGLCD.print(" : ", 200, 20);
    myGLCD.print((String)temperatureF, 230, 20);

    myGLCD.print("Pulse", 20, 60);
    myGLCD.print(" : ", 200, 60);
    myGLCD.print((String)heartRate, 230, 60);

    myGLCD.print("Oxygen SPO2", 20, 100);
    myGLCD.print(" : ", 200, 100);
    myGLCD.print((String)spo2, 230, 100);
    myGLCD.print(" % ", 275, 100);

     myGLCD.print("Blood Sugar", 20, 140);
     myGLCD.print(" : ", 200, 140);
     myGLCD.print("0.0", 230, 140);

    // Button Setting
    Back_Button();
    Refresh_Button();

    // Touch config
    Back_Touch = on;
    Enter_Touch = off;
    Option_Touch = off;
    Refresh_Touch = on;
  }

  void Doctor_Page()
  {

    myGLCD.clrScr();
    myGLCD.setColor(255, 255, 255);
    myGLCD.print("Doctor", CENTER, 20);

    myGLCD.setColor(255, 255, 255);
    myGLCD.print("Dr. G.A.Siddique", 10, 70);
    myGLCD.print("Mbl No:01823887976", 10, 100);
    myGLCD.print("Siddique Medical", 10, 120);
    myGLCD.print("Service", 160, 140);

    // Button Setting
    Back_Button();

    // Touch config
    Back_Touch = on;
    Enter_Touch = off;
    Option_Touch = off;
  }

  void Help_Page()
  {

    myGLCD.clrScr();
    myGLCD.setColor(255, 255, 255);
    myGLCD.print("Helpline", CENTER, 10);

    myGLCD.setColor(255, 255, 255);
    myGLCD.print("Hasnaine - Engineer", 10, 70);
    myGLCD.print("Mbl No:01558960325", 10, 90);

    // Button Setting
    Back_Button();

    // Touch config
    Back_Touch = on;
    Enter_Touch = off;
    Option_Touch = off;
  }

  void About_Page()
  {

    myGLCD.clrScr();
    myGLCD.setColor(255, 255, 255);
    myGLCD.print("About", CENTER, 10);

    myGLCD.setColor(255, 255, 255);
    myGLCD.print("Hasnaine-Engineer", 10, 70);
    myGLCD.print("Arnob - Engineer", 10, 90);
    myGLCD.print("Eity - Designer", 10, 110);
    myGLCD.print("Mostofa - R&D", 10, 130);
    //myGLCD.setBackColor(222,222, 111);

    // Button Setting
    Back_Button();

    // Touch config
    Back_Touch = on;
    Enter_Touch = off;
    Option_Touch = off;
  }

  void Emergency_Page()
  {

    myGLCD.clrScr();
    myGLCD.setColor(255, 255, 255);
    myGLCD.print("Emergency", CENTER, 10);

    myGLCD.setColor(255, 255, 255);
    myGLCD.print("Siddique M.Services", 10, 70);
    myGLCD.print("Road No-22,", 10, 90);
    myGLCD.print("Mirpur-6", 10, 110);
    myGLCD.print("Contact No", 10, 130);
    myGLCD.print("02-9035681", 10, 150);
    //myGLCD.setBackColor(222,222, 111);

    // Button Setting
    Back_Button();

    // Touch config
    Back_Touch = on;
    Enter_Touch = off;
    Option_Touch = off;
  }

  // End of Page Section

  //# Button Section
  //#Back Button
  void Back_Button()
  {

    myGLCD.setColor(255, 255, 255);
    myGLCD.fillRoundRect(200, 190, 300, 230); //( x1 = x-axis start point , y1 = y-axis start point,x1 = x-axis end point , y1 = y-axis end point  )
    myGLCD.setColor(255, 255, 255);
    myGLCD.print("Back", 210, 205); // ("Text",x,y)
    myGLCD.setColor(0, 0, 255);
  }
  //# End of Button Section

  //#Back Button
  void Refresh_Button()
  {

    myGLCD.setColor(255, 255, 255);
    myGLCD.fillRoundRect(20, 190, 150, 230); //( x1 = x-axis start point , y1 = y-axis start point,x1 = x-axis end point , y1 = y-axis end point  )
    myGLCD.setColor(255, 255, 255);
    myGLCD.print("Refresh", 30, 205); // ("Text",x,y)
    myGLCD.setColor(0, 0, 255);
  }
  //# End of Button Section

  //# Touch Section
  void Back_Touch_Render()
  {

    if (myTouch.dataAvailable())
    {
      myTouch.read();
      x = myTouch.getX();
      y = myTouch.getY();

      // TRender For BackButton
      if ((y >= 170) && (y <= 250))
      {
        if ((x >= 180) && (x <= 320))
        {
          Back_Touch = off;
          Enter_Touch = on;
          Refresh_Touch = off;
          // Should Nav to Option
          Option_Page();
        }
      }
    }
  }

  void Refresh_Touch_Render()
  {

    if (myTouch.dataAvailable())
    {
      myTouch.read();
      x = myTouch.getX();
      y = myTouch.getY();

      // TRender For BackButton
      if ((y >= 190) && (y <= 230))
      {
        if ((x >= 20) && (x <= 150))
        {
          Back_Touch = off;
          Enter_Touch = on;
          Refresh_Touch = off;
          // Should Nav to Option
          Health_Page();
        }
      }
    }
  }

  void Enter_touch_Render()
  {

    if (myTouch.dataAvailable())
    {
      myTouch.read();
      x = myTouch.getX();
      y = myTouch.getY();

      // TRender For BackButton
      if ((y >= 170) && (y <= 250))
      {
        if ((x >= 180) && (x <= 320))
        {
          myGLCD.clrScr();
          Back_Touch = on;
          Enter_Touch = off;

          // Should Nev to option
          Option_Page();
        }
      }
    }
  }

  void Option_touch_Render()
  {

    if (myTouch.dataAvailable())
    {
      myTouch.read();
      x = myTouch.getX();
      y = myTouch.getY();

      // TRender For button1 Health_Page
      if ((y >= 40) && (y <= 80))
      {
        if ((x >= 40) && (x <= 150))
        {
          myGLCD.clrScr();
          Back_Touch = on;
          Enter_Touch = off;

          Health_Page();
        }
      }

      // TRender For button2 Doctor
      if ((y >= 40) && (y <= 80))
      {
        if ((x >= 180) && (x <= 310))
        {
          myGLCD.clrScr();
          Back_Touch = on;
          Enter_Touch = off;
          Doctor_Page();
        }
      }

      // TRender For button3 Help
      if ((y >= 100) && (y <= 140))
      {
        if ((x >= 40) && (x <= 150))
        {
          myGLCD.clrScr();
          Back_Touch = on;
          Enter_Touch = off;

          Help_Page();
        }
      }

      // TRender For button4 About
      if ((y >= 100) && (y <= 140))
      {
        if ((x >= 180) && (x <= 310))
        {
          myGLCD.clrScr();
          Back_Touch = on;
          Enter_Touch = off;

          About_Page();
        }
      }

      // TRender For button5
      if ((y >= 160) && (y <= 200))
      {
        if ((x >= 80) && (x <= 270))
        {
          myGLCD.clrScr();
          Back_Touch = on;
          Enter_Touch = off;

          Emergency_Page();
        }
      }
    }
  }
  //# End of Touch Section
  //# End of Display Function

  void Touch_AT_DAT()
  {

    if (Enter_Touch)
    {
      Enter_touch_Render();
    }
    if (Back_Touch)
    {
      Back_Touch_Render();
    }
    if (Option_Touch)
    {
      Option_touch_Render();
    }
    if (Refresh_Touch)
    {
      Refresh_Touch_Render();
    }
  }

  // --------------------------------
  // Fucking GSM
  // --------------------------------
  void gsm(String str)
  {
    String link = "AT+HTTPPARA=\"URL\",\"http://acc.brothersonlinebd.com/SensorDataGetPHP/";
    // String Data = link + "\"";
    //int a = 500;
    String Data = link + str + "\"";

    // GSM System Config
    Serial.println("GSM Data Sending");
    gprsSerial.println("AT+CGDCONT?");
    delay(100);
    gprsSerial.println("AT+CREG?");
    delay(100);
    gprsSerial.println("AT+CGACT?");
    delay(100);
    gprsSerial.println("AT+CREG?");
    delay(100);
    gprsSerial.println("AT+SAPBR=2,1");
    delay(100);
    gprsSerial.println("AT+SAPBR=1,1");
    delay(100);
    gprsSerial.println("AT+HTTPINIT");
    delay(100);
    gprsSerial.println(Data);
    delay(100);
    gprsSerial.println("AT+HTTPPARA=\"CID\",1");
    delay(100);
    gprsSerial.println("AT+HTTPACTION=0");
    delay(100);
    //Serial.println(gprsSerial.write("AT+HTTPREAD"));
    delay(100);
  }
