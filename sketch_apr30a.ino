

#include <Wire.h> //I2C needed for sensors
#include "MPL3115A2.h" //Pressure sensor
#include "HTU21D.h" //Humidity sensor
#include <SoftwareSerial.h> //Needed for GPS
#include <TinyGPS++.h> //GPS parsing

//this is for RTC
int clockAddress = 0x68;  // This is the I2C address
int command = 0;  // This is the command char, in ascii form, sent from the serial port     
long previousMillis = 0;  // will store last time Temp was updated
byte second, minute, hour, dayOfWeek, dayOfMonth, month, year;

byte decToBcd(byte val)
{
  return ( (val/10*16) + (val%10) );
}

// Convert binary coded decimal to normal decimal numbers
byte bcdToDec(byte val)
{
  return ( (val/16*10) + (val%16) );
}

// Gets the date and time from the ds1307 and prints result
char* getDateDs1307(int flag) {
  //if flag == 0 : date output
  //if flag == 1 : time output
  // Reset the register pointer
  Wire.beginTransmission(clockAddress);
  Wire.write(byte(0x00));
  Wire.endTransmission();

  Wire.requestFrom(clockAddress, 7);

  // A few of these need masks because certain bits are control bits
  second     = bcdToDec(Wire.read() & 0x7f);
  minute     = bcdToDec(Wire.read());

  // Need to change this if 12 hour am/pm
  hour       = bcdToDec(Wire.read() & 0x3f);  
  dayOfWeek  = bcdToDec(Wire.read());
  dayOfMonth = bcdToDec(Wire.read());
  month      = bcdToDec(Wire.read());
  year       = bcdToDec(Wire.read());

  char sza[32];
  if (flag==0)
    sprintf(sza, "%02d-%02d-%02d",year,month,dayOfMonth);
  if (flag==1)
    sprintf(sza, "%02d:%02d:%02d",hour,minute,second);
  return(sza);
}
//end of RTC


TinyGPSPlus gps;

static const int RXPin = 5, TXPin = 4; //GPS is attached to pin 4(TX from GPS) and pin 5(RX into GPS)
SoftwareSerial ss(RXPin, TXPin); 
SoftwareSerial mySerial(3, 2);  
MPL3115A2 myPressure; //Create an instance of the pressure sensor
HTU21D myHumidity; //Create an instance of the humidity sensor

const byte STAT1 = 7;
const byte STAT2 = 8;
const byte GPS_PWRCTL = 6; //Pulling this pin low puts GPS to sleep but maintains RTC and RAM
String apn = "hutch";                       //APN
String apn_u = "";                     //APN-Username
String apn_p = "";                     //APN-Password
String url = "http://203.115.24.227:8888/detevts/vts.ashx";  //URL for HTTP-POST-REQUEST
// analog I/O pins
const byte REFERENCE_3V3 = A3;
const byte LIGHT = A1;
const byte BATT = A2;
const byte WDIR = A0;
String msg = String("");

int SmsContentFlag = 0;
 
long lastSecond; //The millis counter to see when a second rolls by
byte seconds; //When it hits 60, increase the current minute
byte seconds_2m; //Keeps track of the "wind speed/dir avg" over last 2 minutes array of data
byte minutes; //Keeps track of where we are in various arrays of data
byte minutes_10m; //Keeps track of where we are in wind gust/dir over last 10 minutes array of data

float lati;
float lon;

float humidity = 0; // [%]
float tempf = 0; // [temperature F]
float rainin = 0; // [rain inches over the past hour)] -- the accumulated rainfall in the past 60 min
volatile float dailyrainin = 0; // [rain inches so far today in local time]

float pressure = 0;


float batt_lvl = 11.8; //[analog value from 0 to 1023]
float light_lvl = 455; //[analog value from 0 to 1023]

void setup()
{
  Serial.begin(9600);

  ss.begin(9600); //Begin listening to GPS over software serial at 9600. This should be the default baud of the module.
mySerial.begin(9600);
  pinMode(STAT1, OUTPUT); //Status LED Blue
  pinMode(STAT2, OUTPUT); //Status LED Green
   lati=0.000000;
   lon=0.000000;
  pinMode(GPS_PWRCTL, OUTPUT);
  digitalWrite(GPS_PWRCTL, HIGH); //Pulling this pin low puts GPS to sleep but maintains RTC and RAM
  
 // pinMode(WSPEED, INPUT_PULLUP); // input from wind meters windspeed sensor
 // pinMode(RAIN, INPUT_PULLUP); // input from wind meters rain gauge sensor
  
  pinMode(REFERENCE_3V3, INPUT);
  pinMode(LIGHT, INPUT);

  //Configure the pressure sensor
  myPressure.begin(); // Get sensor online
  myPressure.setModeBarometer(); // Measure pressure in Pascals from 20 to 110 kPa
  myPressure.setOversampleRate(7); // Set Oversample to the recommended 128
  myPressure.enableEventFlags(); // Enable all three pressure and temp event flags 

  //Configure the humidity sensor
  myHumidity.begin();

  seconds = 0;
  lastSecond = millis();

  

}

void loop()
{
  //Keep track of which minute it is
  if(millis() - lastSecond >= 1000)
  {
    digitalWrite(STAT2, HIGH); //Blink stat LED
    
    lastSecond += 1000;

   
    
  

    //Report all readings every second
    printWeather();

    digitalWrite(STAT2, LOW); //Turn off stat LED
  }

  smartdelay(800); //Wait 1 second, and gather GPS data
  
}

//While we delay for a given amount of time, gather GPS data
static void smartdelay(unsigned long ms)
{
  unsigned long start = millis();
  ss.listen();
  do 
  {
    while (ss.available())
      gps.encode(ss.read());
  } while (millis() - start < ms);
}


//Calculates each of the variables that wunderground is expecting
void calcWeather()
{
 
  //Calc humidity
  humidity = myHumidity.readHumidity();
  //float temp_h = myHumidity.readTemperature();
  //Serial.print(" TempH:");
  //Serial.print(temp_h, 2);

  //Calc tempf from pressure sensor
  tempf = myPressure.readTemp();
 


  //Calc pressure
  pressure = myPressure.readPressure();

  //Calc dewptf

  //Calc light level
  light_lvl = get_light_level();

  //Calc battery level
  batt_lvl = get_battery_level();
  
}

//Returns the voltage of the light sensor based on the 3.3V rail
//This allows us to ignore what VCC might be (an Arduino plugged into USB has VCC of 4.5 to 5.2V)
float get_light_level()
{
  float operatingVoltage = analogRead(REFERENCE_3V3);

  float lightSensor = analogRead(LIGHT);
  
  operatingVoltage = 3.3 / operatingVoltage; //The reference voltage is 3.3V
  
  lightSensor = operatingVoltage * lightSensor;
  
  return(lightSensor);
}

//Returns the voltage of the raw pin based on the 3.3V rail
//This allows us to ignore what VCC might be (an Arduino plugged into USB has VCC of 4.5 to 5.2V)
//Battery level is connected to the RAW pin on Arduino and is fed through two 5% resistors:
//3.9K on the high side (R1), and 1K on the low side (R2)
float get_battery_level()
{
  float operatingVoltage = analogRead(REFERENCE_3V3);

  float rawVoltage = analogRead(BATT);
  
  operatingVoltage = 3.30 / operatingVoltage; //The reference voltage is 3.3V
  
  rawVoltage = operatingVoltage * rawVoltage; //Convert the 0 to 1023 int to actual voltage on BATT pin
  
  rawVoltage *= 4.90; //(3.9k+1k)/1k - multiple BATT voltage by the voltage divider to get actual system voltage
  
  return(rawVoltage);
}




//Prints the various variables directly to the port
//I don't like the way this function is written but Arduino doesn't support floats under sprintf
void printWeather()
{
  calcWeather(); //Go calc all the various sensors

   Serial.println();
  Serial.print(gps.location.lng(), 6);//[0]
  Serial.print(",");
  Serial.print(gps.location.lat(), 6);//[1]
  Serial.print(",");
  Serial.print(gps.altitude.meters());//[2]
  Serial.print(",");
  Serial.print(gps.satellites.value());//[3]
   Serial.print(",");
 Serial.print(gps.course.deg());//[3]
  Serial.print(",");
 Serial.print(gps.speed.kmph());//[3]
 lati=gps.location.lat(); 
 lon=gps.location.lng(); 
 
 
  
 gsm_sendhttp();
}

void gsm_sendhttp() {
  mySerial.listen();
  mySerial.println("AT");
  runsl();//Print GSM Status an the Serial Output;
   Serial.println("1");
  delay(5000);
 
 mySerial.println("AT+CMEE=2");
    runsl();
 // mySerial.println("AT+CBAND=EGSM_DCS_MODE");
    //runsl();
   // delay(100);
  mySerial.println("AT+CBAND=EGSM_PCS_BAND");
    runsl();
    delay(5000);
mySerial.println("AT+CPIN?");
  runsl();
   Serial.println("6");
  delay(100);
 
  mySerial.println("AT+CREG=1");
  runsl();
   Serial.println("6");
  delay(100);
  mySerial.println(" AT+CREG?");
  runsl();
   Serial.println("6");
  delay(100);
  
  mySerial.println("AT+CGATT?");
  runsl();
   Serial.println("6");
  delay(100);
  
   mySerial.println("AT+CSQ");
  runsl();
   Serial.println("6");
  delay(100);
  mySerial.println("AT+SAPBR=3,1,Contype,GPRS");
  runsl();
   Serial.println("2");
  delay(100);
  mySerial.println("AT+SAPBR=3,1,APN," + apn);
  runsl();
   Serial.println("3");
  delay(100);
  mySerial.println("AT+SAPBR =1,1");
  runsl();
   Serial.println("6");
  delay(100);
  mySerial.println("AT+SAPBR=2,1");
  runsl();
  delay(1000);

  
  mySerial.println("AT+HTTPINIT");
  runsl();
   Serial.println("8");
  delay(100);
  mySerial.println("AT+HTTPPARA=CID,1");
  runsl();
   Serial.println("9");
  delay(1000);
  
 
//if(!name1.startsWith(",")){
  mySerial.println("AT+HTTPPARA=URL,"+url+"?p1=A1&distan="+String(lati,6)+"*"+String(lon,6)
 +"&alti="+gps.altitude.meters()+"&tempi="+tempf+"&humi="+humidity+"&dirr="+gps.course.deg()+"&spd="+gps.speed.kmph());
  //+"?p1=B1&tempi="+DHT.temperature+"&humi="+DHT.humidity+"&distan="+name1+"*"+description+"&alti="+altt+"&spd="+spd+"&dirr="+dirr);

  runsl();
   Serial.println(lati,6);
  delay(1000);
  //}
   
  
   

 runsl();
   Serial.println("13");
  delay(1000);
  mySerial.println("AT+HTTPACTION=0");
  runsl();
    Serial.println("14");
  delay(3000);
  mySerial.println("AT+HTTPREAD");
  runsl();

    Serial.println("15");
  delay(100);
  mySerial.println("AT+HTTPTERM");
  runsl(); 
    Serial.println("16");
}

//Print GSM Status
void runsl() {
  while (mySerial.available()) {
    Serial.write(mySerial.read());
  }

}

