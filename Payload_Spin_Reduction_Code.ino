// PAYLOAD SPIN REDUCTION
//
// Developed by J. Michael Ducklow
// Mayberry Galactic
// Last update: 7/6/20/2020 Monday
//
// This Ardunio code runs hardware that reduces payload box spin (yaw). The spin control mechanism is desgined to 
// maintain a stable payload orinetation relative to the horizon durng high altitude balloon (HAB)
// flight asscent.  An accelerometer / gyroscope sensor board (MPU6050) measures periodic change in yaw called spin 
// rate. The spin rate becomes input to stepper-motor control routine, applying torque opposite of yaw movement. With 
// feedback between the accelerometer (MPU6050) and the stepper motor, a steady azimuth and stable flight platform 
// can be established.
// 
// An onboard 1.44" TFT display is incorporated into the design to aid in debugging software.
//
// A independent camara aiming system benefits from a stable flight platform. A camara designed to take solar 
// pictures (sunspots, total eclipse) can take more sun images as less time is needed for tilting and panning 
// a camera to adjust for spin. A spherical arrary with 32 photosensors detects the proximity of sun's location 
// using the highestphotocell current measurement. A pan/tilt platform driven by stepper motors positions a camera 
// at a roughapproximately of the sun's position. Fine adjustment of the pan/tilt is permfored by photosensors on 
// the pan/tilt mechanism. The camera control system is a seperate sketch but is mentioned here as yaw control is
// fundemental to the camera positioning system.
//   
// ATTRIBUTIONS

// (1)The MPU6050 accelerometer and gyroscope control code uses a software library developed by Jeff Rowberg dated 6/21/2012 
// and most recently updated on 7/8/2019.  It is referred to by him as IC2dev with DMP, verison 6.  It was taken from Github.
// Rowberg's code provides yaw/pitch/roll angles (in degrees) and is calculated from the quaternions coming from the MPU6050's FIFO. 
// Yaw control is the only interest for this sketch; Pitch and roll data is ignored. Rowberg's code is gratefully acknowledged.
// It used here under his declared MIT license free of charge with recognition that its used "AS IS" without warranty of anykind. 

// (2) The driver software for the UNLN2003 stepper board and a 4-wire 28-byj-48 stepper motor is based on code by Bret Stateham
// published online in March of 2016.  There is no stated license for the code. It appears to be in the public domain.

// (3) The driver software for the TFT 1.44" display is from Adafruit. It used here under his declared MIT license free of charge
// with recognition that its used "AS IS" without warranty of anykind. 
//
// Pin Usage
//
// Power - Ardunio Vcc 5.0V powers the MPU6050 board and the 1.44" TFT Display
// Gnd   - Ardunio GND grounds the MPU6050 board and the 1.44" TFT Display
// 2     - Interrupt for MPU6050 WHITE
// 3       BLUE / YELLOW stepper coil
// 4       PINK / ORANGE stepper coil
// 5       YELLOW / BLUE stepper coil
// 6       ORANGE / PINK stepper coil
// 8     - Adafruit ST7735 TFT Display DC
// 9     - Adafruit ST7735 TFT Display Reset TFT
// 10    - Adafruit ST7735 TFT Display Clock
// 13    - Adafruit ST7735 

// A5    - Serial Clock (SCL) for the MPU6050 board YELLOW
// A4    - Serial Data (SDA) for the MPU6050 board BLUE
//
// ISSUES & NEEDED ADJUSTMENTS
//
// (1) MPU Locking
// (2) Confirm float == float lockup and reset solved
// (3) Steady azimuth to a set reference point
// (4) No spin no firing messeage
// (5) Change large font for firing message
// (6) Adjustments to firing amount based on spin rate
//
// Library Calls
//
#include <Adafruit_ST7735.h>             // Hardware-specific library for ST7735
#include "MPU6050_6Axis_MotionApps20.h"  // Rowberg library code
#include <avr/wdt.h>

#define INTERRUPT_PIN  2  // pin 2 on Arduino Uno used by MPU6050 to regulate yaw drift
#define Blue           3  // IN1 on the ULN2003 Board, BLUE end of the Blue/Yellow stepper motor coil
#define Pink           4  // IN2 on the ULN2003 Board, PINK end of the Pink/Orange stepper motor coil
#define Yellow         5  // IN3 on the ULN2003 Board, YELLOW end of the Blue/Yellow stepper motor coil
#define Orange         6  // IN4 on the ULN2003 Board, ORANGE end of the Pink/Orange stepper motor coil     
#define TFT_DC         8
#define TFT_RST        9  // Or set to -1 and connect to Arduino RESET pin
#define TFT_CS        10

Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);
MPU6050 mpu;              // class default I2C address is 0x68

int stepper_motor_pins[] {Blue,Pink,Yellow,Orange};

// Define "full" stepper motor step sequence for maximum torque.  Firing of pins that control motor coil states in a array of arrays

int fullStepCount = 4;
int fullSteps[][4] = {
    {HIGH,HIGH,LOW,LOW},
    {LOW,HIGH,HIGH,LOW},
    {LOW,LOW,HIGH,HIGH},
    {HIGH,LOW,LOW,HIGH}
  };
  
bool          dmpReady            = false;    // set true if DMP init was successful
uint8_t       mpuIntStatus;          // holds actual interrupt status byte from MPU
uint8_t       devStatus;             // return status after each device operation (0 = success, !0 = error)
uint16_t      packetSize;            // expected DMP packet size (default is 42 bytes)
uint8_t       fifoBuffer[64];        // FIFO storage buffer
Quaternion    q;                     // [w, x, y, z]         quaternion container
VectorFloat   gravity;               // [x, y, z]            gravity vector
float         ypr[3];                // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
float         yaw_reference1      = 0.0;
float         yaw_reference2      = 0.0;
float         azimuth_target      = 0.0;
float         angle_of_deflection;
float         start_time  = millis();
float         lapse_time  = start_time;
float         lapse_time2         = 0.0;
float         cycle_time          = 0.0;
float         spin_rate           = 0.0;
byte          display_row         = 0;
byte          motor_step_delay_period = 3;   // microseconds
bool          clockwise;
bool          counter_clockwise;
bool          spin;
int           fire_cycles;

volatile bool mpuInterrupt = false;  // tracks MPU interrupt pin high/low


// =================================================================
// ===                 SHORT ROUTINES DEFINED                    ===
// =================================================================

void dmpDataReady()        {mpuInterrupt = true;}  // interupt detection
void long_pause()          {delay(1000);}
void pause()               {delay( 200);}
void very_short_pause()    {delay( 100);}
void get_laspe_time()      {lapse_time2 = lapse_time; lapse_time = millis() - start_time;}
void set_new_display_row() {display_row = display_row + 10; tft.setCursor(0, display_row);}
void (* resetFunc) (void) = 0;                     //declare reset function @ address 0


// =================================================================
// ===                       GET YAW ANGLE                       ===
// =================================================================

void get_yaw_angle() { 
   yaw_reference2 = yaw_reference1;
   if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {                        // get the Latest packet from FIFO
      mpu.dmpGetQuaternion(&q, fifoBuffer);          
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      yaw_reference1 = ypr[0] * 57.297;                                  // convert Euler angles to degrees 
      if (abs(yaw_reference1-yaw_reference2) < 0.01) {resetFunc;}        // if yaw angle is equal, MPU is likely locked; need to reset ardunio
      if (yaw_reference1 < 0) {yaw_reference1 = yaw_reference1 + 360.0;} // convert negative degrees to 0 to 360 positive degrees
   }
}


// =================================================================
// ===                       DISPLAY STATS                       ===
// =================================================================

void display_stats() {

  if (display_row >= 20) {display_row = 0; tft.fillScreen(ST77XX_BLACK);}    
  tft.setCursor(0, display_row);
  tft.print("Yaw Angle :  "); tft.print(yaw_reference1);
  set_new_display_row(); 
  tft.print("Dflt Angle : "); tft.print(angle_of_deflection);
  set_new_display_row(); 
  
  if (spin_rate < 0.75 && spin_rate > -0.75) {                    // Target Range of 'No Spin'
     tft.setTextColor(ST77XX_YELLOW);
     tft.print("NO SPIN");
     tft.setTextColor(ST77XX_WHITE);
     spin = false;
     long_pause(); }
     
  else {
     tft.print("Fire Cycles : "); tft.print(fire_cycles);
     set_new_display_row(); 
     spin = true;
     tft.print("Spin Rate : "); 
     set_new_display_row();
     tft.setTextSize(2);
     tft.print(spin_rate,2);
     tft.setTextSize(1);
     set_new_display_row(); set_new_display_row(); set_new_display_row(); 
     if (angle_of_deflection >= 0.0) {
        tft.setTextColor(ST77XX_GREEN);
        tft.print("Counter Clockwise"); 
        tft.setTextColor(ST77XX_WHITE);
        counter_clockwise = true; clockwise = false;}
     else {
        tft.setTextColor(ST77XX_ORANGE);
        tft.print("Clockwise"); 
        tft.setTextColor(ST77XX_WHITE);
        clockwise = true; counter_clockwise = false;}
   }    
 }


// =================================================================
// ===                     CALCULATE SPIN RATE                   ===
// =================================================================

void calculate_spin_rate_and_direction() {
   angle_of_deflection = yaw_reference2 - yaw_reference1;
   if (angle_of_deflection == 0) {angle_of_deflection = 1;}
   cycle_time = (lapse_time - lapse_time2) / 1000;  // units in seconds (float)
   spin_rate = angle_of_deflection / cycle_time;
   spin_rate = abs(spin_rate);
}


// ================================================================
// ===                    FIRE MOTOR ROUTINE                    ===
// ================================================================

void fire_stepper_motor(int steps[][4]) {

int step_motor_array_i;
int decelerate_pause = 1;

angle_of_deflection = abs(angle_of_deflection);
if (angle_of_deflection > 180.0 && spin_rate > 120.0) {spin_rate = 5.0;}

 // stepper motor takes 8.4 seconds for 360 degrees; 7.2 rpm; full 360 degrees takes 513 stepper cycles
if (spin_rate <    1.9)                      {fire_cycles = 2;} 
if (spin_rate >=   1.9 && spin_rate <   5.1) {fire_cycles = spin_rate * (4.275/5);}
if (spin_rate >=   5.1 && spin_rate <  10.1) {fire_cycles = spin_rate * (4.275/4);}
if (spin_rate >=  10.1 && spin_rate <  30.1) {fire_cycles = spin_rate * (4.275/3);}
if (spin_rate >=  30.1 && spin_rate < 100.1) {fire_cycles = spin_rate * (4.275/2);}
if (spin_rate >= 100.1)                      {fire_cycles = spin_rate * (4.275/1);}

for (int j=1; j <=  fire_cycles; j++) { 
   if (clockwise) {
      for (step_motor_array_i=0; step_motor_array_i < 4; step_motor_array_i++) {
         if (fire_cycles < 8) {delay(decelerate_pause); decelerate_pause = decelerate_pause + 5; }
         for(int pin=0; pin < 4; pin++) {
            digitalWrite(stepper_motor_pins[pin],steps[step_motor_array_i][pin]);  // fire pins
            delay(motor_step_delay_period);   
         }     
       }
   }
   if (counter_clockwise) {
      for (step_motor_array_i=3; step_motor_array_i > -1; step_motor_array_i--) {
         if (fire_cycles < 8) {delay(decelerate_pause); decelerate_pause = decelerate_pause + 5; }
            for(int pin=0; pin < 4; pin++) {
            digitalWrite(stepper_motor_pins[pin],steps[step_motor_array_i][pin]);  // fire pins
            delay(motor_step_delay_period);
         }       
      }
   }
}
}


// ================================================================
// ===                            SETUP                         ===
// ================================================================


void setup(void) {
  
// ================================================================
// ===     MPU6050 SENSOR, TFT SCREEN, STEPPER MOTOR SETUP      ===
// ================================================================

Wire.begin();            
Wire.setClock(400000);                                      // 400kHz I2C clock
pinMode(INTERRUPT_PIN, INPUT);
tft.initR(INITR_144GREENTAB);                               // Initial ST7735R chip

tft.fillScreen(ST77XX_BLACK);
tft.setTextWrap(false);
tft.setTextColor(ST77XX_RED);
tft.setTextSize(1);
tft.setCursor(0,  0); tft.println("** Setup Messages **");
tft.setTextColor(ST77XX_BLUE);
set_new_display_row(); tft.println("Initializing MPU6050");

mpu.initialize();                                           // initialize device

set_new_display_row(); tft.println("Testing Connections");
set_new_display_row(); tft.println(mpu.testConnection() ? "MPU6050 connected" : "MPU6050 connect failed");
set_new_display_row(); tft.println("Initializing DMP");  

devStatus = mpu.dmpInitialize();                            // load and configure the DMP
if (devStatus == 0) {                                       // make sure it worked (returns 0 means good)
   mpu.CalibrateAccel(6);                                   // Calibration Time: generate offsets and calibrate our MPU6050
   mpu.CalibrateGyro(6);
   set_new_display_row(); tft.println("Enabling MPU");
   mpu.setDMPEnabled(true);                                 // turn on the DMP, now that it's ready
   attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
   mpuIntStatus = mpu.getIntStatus();                       // enable Arduino interrupt detection
   set_new_display_row(); tft.println("MPU Ready");
   dmpReady = true;
   packetSize = mpu.dmpGetFIFOPacketSize();                 // get expected DMP packet size for later comparison
   get_yaw_angle();
   azimuth_target = yaw_reference1;                         // set the azimuth target for the stable position angle
   
} else {
   set_new_display_row(); tft.println("MPU Setup Failed");
   set_new_display_row(); tft.println("Code is: ");
   tft.println(devStatus);                                  // 1 = initial memory load failed;  2 = DMP configuration updates failed
 }

 for (int pin = 0; pin < 4; pin++) {                        //set up stepper motor firing pins
       pinMode(stepper_motor_pins[pin], OUTPUT);
       digitalWrite(stepper_motor_pins[pin], LOW);
 }
 set_new_display_row(); tft.println("Motor Setup Done");
 display_row = 0;
 long_pause();
 tft.fillScreen(ST77XX_BLACK);
 tft.setTextColor(ST77XX_WHITE);
}

// ================================================================
// ===                          MAIN LOOP                       ===
// ================================================================

void loop() {
  wdt_enable(WDTO_8S);     //set watch-dog timer; Reset Ardurnio if watch dog timer is not reset within 8 seconds 
  get_laspe_time();
  get_yaw_angle();
  very_short_pause();
  get_laspe_time();
  get_yaw_angle();
  calculate_spin_rate_and_direction();
  display_stats();
  if (spin) {fire_stepper_motor(fullSteps); }
  wdt_reset(); // reset the watch-dog timer
}
