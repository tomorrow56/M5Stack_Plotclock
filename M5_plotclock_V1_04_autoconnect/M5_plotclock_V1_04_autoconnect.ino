/***************************************************
  This is an example for our Adafruit 16-channel PWM & Servo driver
  Servo test - this will drive 8 servos, one after the other on the
  first 3 pins of the PCA9685
  https://github.com/adafruit/Adafruit-PWM-Servo-Driver-Library
****************************************************/
/***************************************************
 Plotclock
 cc - by Johannes Heberlein 2014
 v 1.01
 thingiverse.com/joo   wiki.fablab-nuernberg.de
 units: mm; microseconds; radians
 origin: bottom left of drawing surface
 time library see http://playground.arduino.cc/Code/time 
****************************************************/
#include <M5Stack.h>
#include "M5StackUpdater.h"

#include <WiFi.h>
// #include <Time.h>        // http://playground.arduino.cc/Code/time 
#include <TimeLib.h>        // https://forum.arduino.cc/index.php?topic=415296.0
#include <DNSServer.h>      // https://github.com/zhouhan0126/DNSServer---esp32
#include <WebServer.h>      // https://github.com/zhouhan0126/WebServer-esp32
#include <WiFiManager.h>   // https://github.com/zhouhan0126/WIFIMANAGER-ESP32

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Use M5Stack Logo
extern const unsigned char gImage_logoM5[];

// delete or mark the next line as comment when done with calibration  
//#define CALIBRATION

// When in calibration mode, adjust the following factor until the servos move exactly 90 degrees
#define SERVOFAKTOR 500 // default = 620

// Zero-position of left and right servo
// When in calibration mode, adjust the NULL-values so that the servo arms are at all times parallel
// either to the X or Y axis
#define SERVOLEFTNULL 1700  // default = 1900
#define SERVORIGHTNULL 1000  // defaule = 984

#define SERVOPINLIFT  0
#define SERVOPINLEFT  1
#define SERVOPINRIGHT 2

// lift positions of lifting servo
#define LIFT0 1095 // on drawing surface
#define LIFT1 900  // between numbers
#define LIFT2 725  // going towards sweeper

// speed of liftimg arm, higher is slower
#define LIFTSPEED 3000  // usec

#define LINESPEED 15    // msec
#define CURVESPEED 10    // msec
#define CHARINTERVAL 200    // msec

// length of arms
#define L1 35   // default = 35
#define L2 55.1 // default = 55.1 measure = 44.5
#define L3 13.2 // default = 13.2 measure = 14.5

// origin points of left and right servo 
#define O1X 22
#define O1Y -25
#define O2X 47
#define O2Y -25

String DisplayFileName = "";

// you can also call it with a different address you want
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x72);

int servoLift = LIFT2;

volatile double lastX = 75;
volatile double lastY = 47.5;

int last_min = 0;

double currentX;
double currentY;

const char* ntpServer = "ntp.nict.jp";
const long  gmtOffset_sec = 9 * 3600;  // JST = UTC + 9
const int   daylightOffset_sec = 0;
int hh, mm, ss;
int retry = 5;

void setup(){ 
  M5.begin();

  if(digitalRead(BUTTON_A_PIN) == 0){
    Serial.println("Will Load menu binary");
    updateFromFS(SD);
    ESP.restart();
  }

  M5.Lcd.setBrightness(128);
  M5.Lcd.setCursor(0, 0);
  M5.Lcd.setTextColor(WHITE);
  M5.Lcd.setTextSize(2);

  //Serial.begin(115200);
  Serial.println("PCA9685 PlotClock");
  M5.Lcd.println("PCA9685 PlotClock");

  //WiFiManager
  //Local intialization. Once its business is done, there is no need to keep it around
  WiFiManager wifiManager;

  //if it does not connect it starts an access point with the specified name
  //here  "AutoConnectAP"
  //and goes into a blocking loop awaiting configuration
 
  M5.Lcd.println(" AP name: M5_PlotClock");
  M5.Lcd.println(" IP adrs: 192.168.4.1");

  // Display QRCode
  M5.Lcd.qrcode("http://192.168.4.1", 150, 70, 150, 6);
  // M5.Lcd.qrcode(const char *string, uint16_t x = 50, uint16_t y = 10, uint8_t width = 220, uint8_t version = 6);

  //fetches ssid and pass from eeprom and tries to connect
  //if it does not connect it starts an access point with the specified name
  //here  "AutoConnectAP"
  //and goes into a blocking loop awaiting configuration
  wifiManager.autoConnect("M5_PlotClock");

  M5.Lcd.setTextColor(GREEN);
 
  //if you get here you have connected to the WiFi
  IPAddress ipadr = WiFi.localIP();
  Serial.println("connected!");
  Serial.println(WiFi.SSID());
  Serial.println(ipadr);


  M5.Lcd.fillScreen(TFT_BLACK);
  M5.Lcd.setCursor(0, 0);

  M5.Lcd.println("SSID: " + WiFi.SSID());
  M5.Lcd.println("IP adrs: " + (String)ipadr[0] + "." + (String)ipadr[1] + "." + (String)ipadr[2] + "." + (String)ipadr[3]);

  //init and get the time
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

  struct tm timeinfo;
// int tm_sec;   /* 秒 － [0, 60/61] */
// int tm_min;   /* 分 － [0, 59] */
// int tm_hour;  /* 時 － [0, 23] */
// int tm_mday;  /* 日 － [1, 31] */
// int tm_mon;   /* 1月からの月数 － [0, 11] */
// int tm_year;  /* 1900年からの年数 */
// int tm_wday;  /* 日曜日からの日数 － [0, 6] */
// int tm_yday;  /* 1月1日からの日数 － [0, 365] */
// int tm_isdst; /* 夏時間フラグ */

  for(int i = 0; i < retry; i++){
    if(!getLocalTime(&timeinfo)){
      M5.Lcd.setTextColor(RED);
      Serial.println("Failed to obtain time");
      M5.Lcd.println("Failed to obtain time");
      if(i == retry - 1){
        return;
      }
    }else{
      M5.Lcd.setTextColor(GREEN);
      Serial.println("Connected to NTP Server!");
      M5.Lcd.println("Connected to NTP Server!");
      break;
    }
  }

  hh = timeinfo.tm_hour;
  mm = timeinfo.tm_min;
  ss = timeinfo.tm_sec;

  //disconnect WiFi as it's no longer needed
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);

  pwm.begin();
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates
  delay(100);

  // Set current time only the first to values, hh,mm are needed
  // If set the current time to 14:27:00, December 14th, 2015
  // setTime(14, 27, 00, 14, 12, 2015);
  setTime(hh, mm, ss, 0, 0, 0);
//  Serial.print((String)(yyyy + 1900));
  Serial.printf("%02d:%02d:%02d", hh, mm, ss);
  M5.Lcd.printf("%02d:%02d:%02d", hh, mm, ss);

  drawTo(75.2, 47);
  lift(0);

  delay(1000);

  M5.Lcd.setTextColor(WHITE);

  M5.Lcd.fillScreen(TFT_BLACK);
  M5.Lcd.setBrightness(0);

//  DisplayFileName = "/jpg/M5Logo.jpg";
//  M5.Lcd.drawJpgFile(SD, DisplayFileName.c_str(), 0, 0, 320, 240);

  // Use M5Stack Logo
  M5.Lcd.pushImage(0, 0, 320, 240, (uint16_t *)gImage_logoM5);
}

void loop(){ 

#ifdef CALIBRATION
  // Servohorns will have 90° between movements, parallel to x and y axis
  drawTo(-3, 29.2);
  delay(500);
  drawTo(74.1, 28);
  delay(500);
#else 
  int i = 0;
  if (last_min != minute()){
   M5.Lcd.setBrightness(127);
   lift(0);
    hour();
    while ((i+1)*10 <= hour()){
      i++;
    }

    number(3, 3, 111, 1);
    number(5, 25, i, 0.9);
    number(19, 25, (hour()-i*10), 0.9);
    number(28, 25, 11, 0.9);

    i=0;
    while ((i+1)*10 <= minute()){
      i++;
    }
    number(34, 25, i, 0.9);
    number(48, 25, (minute()-i*10), 0.9);
    lift(2);

//    return to home
    drawTo(74.2, 47.5);
//    lift(1);
    lift(0);
    last_min = minute();

//    servo detach
    setServoPulse(SERVOPINLIFT, 0);
    setServoPulse(SERVOPINLEFT, 0);
    setServoPulse(SERVOPINRIGHT, 0);

    M5.Lcd.setBrightness(0);
  }
#endif
} 

/*************************************************** 
 you can use this function if you'd like to set the pulse length in microsecond
***************************************************/
void setServoPulse(uint8_t n, double pulse_us){
  double pulselength;
  
  pulselength = 1000000;   // 1,000,000 us per second
  pulselength /= 60;   // 60 Hz
//  Serial.print(pulselength); Serial.println(" us per period"); 
  pulselength /= 4096;  // 12 bits of resolution
//  Serial.print(pulselength); Serial.println(" us per bit"); 
  pulse_us /= pulselength;
//  Serial.println(pulse_us);

  pwm.setPWM(n, 0, pulse_us);
}

/*************************************************** 
 Draw Line from p to q
***************************************************/
void drawFromTo(double pX, double pY, double qX, double qY){
  double dx, dy;
  double absx, absy;
  int i;
  int accuracy = 1;
  int loop;

  dx = qX - pX;
  dy = qY - pY;

  absx = abs(dx / accuracy);
  absy = abs(dy / accuracy);

  if(absx >= absy){
    loop = absx;
  }else{
    loop = absy;
  }

  for (i = 0; i <= loop; i++) {
    drawTo(pX + ((dx / loop) * i), pY + ((dy / loop) * i));
    delay(LINESPEED);
  }
  drawTo(qX, qY);
}

/*************************************************** 
 Writing numeral with bx by being the bottom left originpoint. Scale 1 equals a 20 mm high font.
 The structure follows this principle: move to first startpoint of the numeral, lift down, draw numeral, lift up
***************************************************/
void number(float bx, float by, int num, float scale){
  float bxx;
  float byy;
  float radius;
  int start;
  int ende;
  float sqee;

  double nextX;
  double nextY;

  switch (num){
  case 0:
    drawTo(bx + 12 * scale, by + 6 * scale);
    lift(0);
    bxx = bx + 7 * scale;
    byy = by + 10 * scale;
    radius = 10 * scale;
    start = -0.8;
    ende = 6.7;
    sqee = 0.5;
    nextX = sqee * radius * cos(start) + bxx;
    nextY = radius * sin(start) + byy;

    drawFromTo(bx + 12 * scale, by + 6 * scale, nextX, nextY);
    bogenGZS(bx + 7 * scale, by + 10 * scale, 10 * scale, -0.8, 6.7, 0.5);
    lift(1);
    delay(CHARINTERVAL);
    break;
  case 1:
    drawTo(bx + 3 * scale, by + 15 * scale);
    lift(0);
    drawFromTo(bx + 3 * scale, by + 15 * scale, bx + 10 * scale, by + 20 * scale);
    drawFromTo(bx + 10 * scale, by + 20 * scale, bx + 10 * scale, by + 0 * scale);
    lift(1);
    delay(CHARINTERVAL);
    break;
  case 2:
    drawTo(bx + 2 * scale, by + 12 * scale);
    lift(0);
    bxx = bx + 8 * scale;
    byy = by + 14 * scale;
    radius = 6 * scale;
    start = 3;
    ende = -0.8;
    sqee = 1;
    nextX = sqee * radius * cos(start) + bxx;
    nextY = radius * sin(start) + byy;

    drawFromTo(bx + 2 * scale, by + 12 * scale, nextX, nextY);
    bogenUZS(bx + 8 * scale, by + 14 * scale, 6 * scale, 3, -0.8, 1);
    drawFromTo(currentX, currentY, bx + 1 * scale, by + 0 * scale);
    drawFromTo( bx + 1 * scale, by + 0 * scale, bx + 12 * scale, by + 0 * scale);
    lift(1);
    delay(CHARINTERVAL);
    break;
  case 3:
    drawTo(bx + 2 * scale, by + 17 * scale);
    lift(0);
    bxx = bx + 5 * scale;
    byy = by + 15 * scale;
    radius = 5 * scale;
    start = 3;
    ende = -2;
    sqee = 1;
    nextX = sqee * radius * cos(start) + bxx;
    nextY = radius * sin(start) + byy;

    drawFromTo(bx + 2 * scale, by + 17 * scale, nextX, nextY);
    bogenUZS(bx + 5 * scale, by + 15 * scale, 5 * scale, 3, -2, 1);

    bxx = bx + 5 * scale;
    byy = by + 5 * scale;
    radius = 5 * scale;
    start = 1.57;
    ende = -3;
    sqee = 1;
    nextX = sqee * radius * cos(start) + bxx;
    nextY = radius * sin(start) + byy;
    
    drawFromTo(currentX, currentY, nextX, nextY);
    bogenUZS(bx + 5 * scale, by + 5 * scale, 5 * scale, 1.57, -3, 1);
    delay(CHARINTERVAL);
    lift(1);
    break;
  case 4:
    drawTo(bx + 10 * scale, by + 0 * scale);
    lift(0);
    drawFromTo(bx + 10 * scale, by + 0 * scale, bx + 10 * scale, by + 20 * scale);
    drawFromTo(bx + 10 * scale, by + 20 * scale, bx + 2 * scale, by + 6 * scale);
    drawFromTo(bx + 2 * scale, by + 6 * scale, bx + 12 * scale, by + 6 * scale);
    lift(1);
    delay(CHARINTERVAL);
    break;
  case 5:
    drawTo(bx + 2 * scale, by + 5 * scale);
    lift(0);
    bxx = bx + 5 * scale;
    byy = by + 6 * scale;
    radius = 6 * scale;
    start = -2.5;
    ende = 2;
    sqee = 1;
    nextX = sqee * radius * cos(start) + bxx;
    nextY = radius * sin(start) + byy;

    drawFromTo(bx + 2 * scale, by + 5 * scale, nextX, nextY);
    bogenGZS(bx + 5 * scale, by + 6 * scale, 6 * scale, -2.5, 2, 1);
    drawFromTo(currentX, currentY, bx + 5 * scale, by + 20 * scale);
    drawFromTo(bx + 5 * scale, by + 20 * scale, bx + 12 * scale, by + 20 * scale);
    lift(1);
    delay(CHARINTERVAL);
    break;
  case 6:
    drawTo(bx + 2 * scale, by + 10 * scale);
    lift(0);
    bxx = bx + 7 * scale;
    byy = by + 6 * scale;
    radius = 6 * scale;
    start = 2;
    ende = -4.4;
    sqee = 1;
    nextX = sqee * radius * cos(start) + bxx;
    nextY = radius * sin(start) + byy;

    drawFromTo(bx + 2 * scale, by + 10 * scale, nextX, nextY);
    bogenUZS(bx + 7 * scale, by + 6 * scale, 6 * scale, 2, -4.4, 1);
    drawFromTo(currentX, currentY, bx + 11 * scale, by + 20 * scale);
    delay(CHARINTERVAL);
    lift(1);
    break;
  case 7:
    drawTo(bx + 2 * scale, by + 20 * scale);
    lift(0);
    drawFromTo(bx + 2 * scale, by + 20 * scale, bx + 12 * scale, by + 20 * scale);
    drawFromTo(bx + 12 * scale, by + 20 * scale, bx + 2 * scale, by + 0);
    lift(1);
    delay(CHARINTERVAL);
    break;
  case 8:
    drawTo(bx + 5 * scale, by + 10 * scale);
    lift(0);
    bxx = bx + 5 * scale;
    byy = by + 15 * scale;
    radius = 5 * scale;
    start = 4.7;
    ende = -1.6;
    sqee = 1;
    nextX = sqee * radius * cos(start) + bxx;
    nextY = radius * sin(start) + byy;

    drawFromTo(bx + 5 * scale, by + 10 * scale, nextX, nextY);
    bogenUZS(bx + 5 * scale, by + 15 * scale, 5 * scale, 4.7, -1.6, 1);

    bxx = bx + 5 * scale;
    byy = by + 5 * scale;
    radius = 5 * scale;
    start = -4.7;
    ende = 2;
    sqee = 1;
    nextX = sqee * radius * cos(start) + bxx;
    nextY = radius * sin(start) + byy;

    drawFromTo(currentX, currentY, nextX, nextY);
    bogenGZS(bx + 5 * scale, by + 5 * scale, 5 * scale, -4.7, 2, 1);
    lift(1);
    delay(CHARINTERVAL);
    break;

  case 9:
    drawTo(bx + 9 * scale, by + 11 * scale);
    lift(0);
    bxx = bx + 7 * scale;
    byy = by + 15 * scale;
    radius = 5 * scale;
    start = 4;
    ende = -0.5;
    sqee = 1;
    nextX = sqee * radius * cos(start) + bxx;
    nextY = radius * sin(start) + byy;
   
    drawFromTo(bx + 9 * scale, by + 11 * scale, nextX, nextY);
    bogenUZS(bx + 7 * scale, by + 15 * scale, 5 * scale, 4, -0.5, 1);
    drawFromTo(currentX, currentY, bx + 5 * scale, by + 0);
   lift(1);
   delay(CHARINTERVAL);
    break;

  case 111:

    lift(0);
    drawFromTo(70, 46, 65, 43);
    drawFromTo(65, 43, 65, 49);
    drawFromTo(65, 49, 5, 49);

    drawFromTo(5, 49, 5, 45);
    drawFromTo(5, 45, 65, 45);
    drawFromTo(65, 45, 65, 40);
    drawFromTo(65, 40, 5, 40);

    drawFromTo(5, 40, 5, 35);
    drawFromTo(5, 35, 64, 35);
    drawFromTo(65, 35, 65, 30);
    drawFromTo(65, 30, 5, 30);

    drawFromTo(5, 30, 5, 25);
    drawFromTo(5, 25, 65, 25);
    drawFromTo(65, 25, 65, 20);
    drawFromTo(65, 20, 5, 20);

    drawFromTo(5, 20, 60, 44);
    drawFromTo(60, 44, 75.2, 47);

    lift(2);
    delay(CHARINTERVAL);

    break;

  case 11:
    drawTo(bx + 5 * scale, by + 15 * scale);
    lift(0);
    bogenGZS(bx + 5 * scale, by + 15 * scale, 0.1 * scale, 1, -1, 1);
    lift(1);
    delay(CHARINTERVAL);
    drawTo(bx + 5 * scale, by + 5 * scale);
    lift(0);
    bogenGZS(bx + 5 * scale, by + 5 * scale, 0.1 * scale, 1, -1, 1);
    lift(1);
    delay(CHARINTERVAL);
    break;
  }
}

void lift(char lift){
  switch (lift) {
  case 0: // on drawing surface
    if (servoLift >= LIFT0) {
      while (servoLift >= LIFT0){
        servoLift--;
        setServoPulse(SERVOPINLIFT, (double)servoLift);
        delayMicroseconds(LIFTSPEED);
      }
    }else{
      while (servoLift <= LIFT0){
        servoLift++;
        setServoPulse(SERVOPINLIFT, (double)servoLift);
        delayMicroseconds(LIFTSPEED);
      }
    }
    break;

  case 1: // between numbers
    if (servoLift >= LIFT1){
      while (servoLift >= LIFT1){
        servoLift--;
        setServoPulse(SERVOPINLIFT, (double)servoLift);
        delayMicroseconds(LIFTSPEED);
      }
    }else{
      while (servoLift <= LIFT1){
        servoLift++;
        setServoPulse(SERVOPINLIFT, (double)servoLift);
        delayMicroseconds(LIFTSPEED);
      }
    }
    break;

  case 2: // going towards sweeper
    if (servoLift >= LIFT2){
      while (servoLift >= LIFT2){
        servoLift--;
        setServoPulse(SERVOPINLIFT, (double)servoLift);
        delayMicroseconds(LIFTSPEED);
      }
    }else{
      while (servoLift <= LIFT2){
        servoLift++;
        setServoPulse(SERVOPINLIFT, (double)servoLift);
        delayMicroseconds(LIFTSPEED);
      }
    }
    break;
  }
}

void bogenUZS(float bx, float by, float radius, int start, int ende, float sqee){
  float inkr = -0.05;
  float count = 0;
  do{
    currentX = sqee * radius * cos(start + count) + bx;
    currentY = radius * sin(start + count) + by;
    drawTo(currentX, currentY);
    count += inkr;
    delay(CURVESPEED);
  }
  while ((start + count) > ende);
}

void bogenGZS(float bx, float by, float radius, int start, int ende, float sqee){
  float inkr = 0.05;
  float count = 0;
  do{
    currentX = sqee * radius * cos(start + count) + bx;
    currentY = radius * sin(start + count) + by;
    drawTo(currentX, currentY);
    count += inkr;
    delay(CURVESPEED);
  }
  while ((start + count) <= ende);
}

void drawTo(double pX, double pY){
  double dx, dy, c;
  int i;
  // dx dy of new point
  dx = pX - lastX;
  dy = pY - lastY;
  //path lenght in mm, times 4 equals 4 steps per mm
  c = floor(4 * sqrt(dx * dx + dy * dy));

  if (c < 1){
    c = 1;
  }

  for (i = 0; i <= c; i++){
    // draw line point by point
    set_XY(lastX + (i * dx / c), lastY + (i * dy / c));
  }
  lastX = pX;
  lastY = pY;
}

double return_angle(double a, double b, double c){
  // cosine rule for angle between c and a
  return acos((a * a + c * c - b * b) / (2 * a * c));
}

void set_XY(double Tx, double Ty){
  delay(1);
  double dx, dy, c, a1, a2, Hx, Hy;
  // calculate triangle between pen, servoLeft and arm joint
  // cartesian dx/dy
  dx = Tx - O1X;
  dy = Ty - O1Y;
  // polar lemgth (c) and angle (a1)
  c = sqrt(dx * dx + dy * dy); // 
  a1 = atan2(dy, dx); //
  a2 = return_angle(L1, L2, c);

  setServoPulse(SERVOPINLEFT, (double)(floor(((a2 + a1 - M_PI) * SERVOFAKTOR) + SERVOLEFTNULL)));

  // calculate joinr arm point for triangle of the right servo arm
  a2 = return_angle(L2, L1, c);
  Hx = Tx + L3 * cos((a1 - a2 + 0.621) + M_PI); //36,5°
  Hy = Ty + L3 * sin((a1 - a2 + 0.621) + M_PI);
  // calculate triangle between pen joint, servoRight and arm joint
  dx = Hx - O2X;
  dy = Hy - O2Y;

  c = sqrt(dx * dx + dy * dy);
  a1 = atan2(dy, dx);
  a2 = return_angle(L1, (L2 - L3), c);

  setServoPulse(SERVOPINRIGHT, (double)(floor(((a1 - a2) * SERVOFAKTOR) + SERVORIGHTNULL)));
}
