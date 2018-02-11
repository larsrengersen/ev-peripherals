#include <DallasTemperature.h> // Source https://www.milesburton.com/Dallas_Temperature_Control_Library
#include <OneWire.h> // includes the OneWire Library for DS18B20 temp. sensors
#include <Wire.h>  // Comes with Arduino IDE
#include <LiquidCrystal_I2C.h>

/*-----( Declare Constants and Pin Numbers )-----*/
// Data wire is plugged into port 12 on the Arduino for battery, controller, motor
#define ONE_WIRE_BUS 12    // NOTE: No ";" on #define  

/*-----( Declare objects )-----*/
// Setup a oneWire instance to communicate with any OneWire devices 
OneWire oneWire(ONE_WIRE_BUS);

// Pass address of our oneWire instance to Dallas Temperature. 
DallasTemperature sensors(&oneWire);

// set the LCD address to 0x27 for a 20 chars 4 line display
// Set the pins on the I2C chip used for LCD connections:
//                    addr, en,rw,rs,d4,d5,d6,d7,bl,blpol
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  // Set the LCD I2C address


// Assigns PWM pump pins
int BatteryPumpPin = 6;
int ControllerPumpPin = 7;
int MotorPumpPin = 8;

/*-----( Declare Variables )-----*/
// Assign the addresses of your 1-Wire temp sensors.
DeviceAddress Battery = { <insert address here> }; 
DeviceAddress Controller = { <insert address here> }; 
DeviceAddress Motor = { <insert address here> }; 

// Temp sensors
int intTempC;
int mesTempC;

// Flow sensor stuff define
// Battery
volatile int  flow_frequency_b;  // Measures flow meter pulses
unsigned int  l_min_b;          // Litres/min                      
int flowmeter_b = 2;  // Battery Flow Meter Pin number
unsigned long currentTime_b;
unsigned long cloopTime_b;

void flow_b ()                  // Interrupt function
{ 
   flow_frequency_b++;
}

// Controller
volatile int  flow_frequency_c;  // Measures flow meter pulses
unsigned int  l_min_c;          // Litres/min                      
int flowmeter_c = 3;  // Controller Flow Meter Pin number
unsigned long currentTime_c;
unsigned long cloopTime_c;

void flow_c ()                  // Interrupt function
{ 
   flow_frequency_c++;
}

// Motor
volatile int  flow_frequency_m;  // Measures flow meter pulses
unsigned int  l_min_m;          // Litres/min                      
int flowmeter_m = 18;  // Controller Flow Meter Pin number
unsigned long currentTime_m;
unsigned long cloopTime_m;

void flow_m ()                  // Interrupt function
{ 
   flow_frequency_m++;
}

// end flow stuff
 
void setup() /****** SETUP: RUNS ONCE ******/
{ 
Serial.begin(9600); // Set communication speed (Baud rate)
 lcd.begin(20,4); // Initializes the interface to the LCD screen, and specifies the dimensions (width and height) of the display } 
 
 //------- Initialize the Temperature measurement library--------------
  sensors.begin();
// set the resolution to 10 bit (Can be 9 to 12 bits .. lower is faster)
  sensors.setResolution(Battery, 10);
  sensors.setResolution(Controller, 10);
  sensors.setResolution(Motor, 10);

// Initialize the pump pins
pinMode(BatteryPumpPin, OUTPUT);
pinMode(ControllerPumpPin, OUTPUT);
pinMode(MotorPumpPin, OUTPUT);

// Initialize the flowmeters
// Battery
  pinMode(flowmeter_b, INPUT);
  attachInterrupt(0, flow_b, RISING); // Setup Interrupt 
                                     // see http://arduino.cc/en/Reference/attachInterrupt
   sei();                            // Enable interrupts  
   currentTime_b = millis();
   cloopTime_b = currentTime_b;  

// Controller
  pinMode(flowmeter_c, INPUT);
  attachInterrupt(1, flow_c, RISING); // Setup Interrupt 
                                     // see http://arduino.cc/en/Reference/attachInterrupt
   sei();                            // Enable interrupts  
   currentTime_c = millis();
   cloopTime_c = currentTime_c;  

// Motor
  pinMode(flowmeter_m, INPUT);
  attachInterrupt(5, flow_m, RISING); // Setup Interrupt 
                                     // see http://arduino.cc/en/Reference/attachInterrupt
   sei();                            // Enable interrupts  
   currentTime_m = millis();
   cloopTime_m = currentTime_m;  

}

//
void loop()   /****** LOOP: RUNS CONSTANTLY ******/
{
// Temperature sensing  
 sensors.requestTemperatures(); // Send the command to get temperatures

//Calculate pump speed
//Battery
int batteryTemp;
int batteryWrite;
int batteryPWM;
batteryTemp = currentTemperature(Battery);
Serial.print(batteryTemp);
Serial.println(" battery temp in calc. loop");
if (batteryTemp < 0)
{
  batteryPWM = 50;
  analogWrite(BatteryPumpPin, 127); //battery pump speed 50%  
}
else if (batteryTemp > 40)
{
  analogWrite(BatteryPumpPin, 255); //battery pump speed 100%
  batteryPWM = 100;
}
else
{
  batteryWrite = ((((1.25 * batteryTemp) + 50)) * 2.55);
  batteryPWM = (((1.25 * batteryTemp) + 50));
  Serial.print(batteryWrite);
  analogWrite(BatteryPumpPin, batteryWrite);
}
//--end battery

//Controller
int controllerTemp;
int controllerWrite;
int controllerPWM;
controllerTemp = currentTemperature(Controller);
Serial.print(controllerTemp);
Serial.println(" controller temp in calc. loop");
if (controllerTemp < 0)
{
  controllerPWM = 50;
  analogWrite(ControllerPumpPin, 127); //controller pump speed 50%  
}
else if (controllerTemp > 50)
{
  analogWrite(ControllerPumpPin, 255); //controller pump speed 100%
  controllerPWM = 100;
}
else
{
  controllerWrite = ((controllerTemp + 50) * 2.55);
  controllerPWM = (controllerTemp + 50);
  Serial.print(controllerWrite);
  analogWrite(ControllerPumpPin, controllerWrite);
}
//--end controller

//Motor
int motorTemp;
int motorWrite;
int motorPWM;
motorTemp = currentTemperature(Motor);
Serial.print(motorTemp);
Serial.println(" motor temp in calc. loop");
if (motorTemp < 0)
{
  motorPWM = 50;
  analogWrite(MotorPumpPin, 127); //motor pump speed 50%  
}
else if (motorTemp > 60)
{
  analogWrite(MotorPumpPin, 255); //controller pump speed 100%
  motorPWM = 100;
}
else
{
  motorWrite = ((((0.83 * motorTemp) + 50)) * 2.55);
  motorPWM = (((0.83 * motorTemp) + 50));
  Serial.print(motorWrite);
  analogWrite(MotorPumpPin, motorWrite);
}
//--end motor control calculations

// Flowmeter calculate section
// Battery
   currentTime_b = millis();
   // Every second, calculate and print litres/hour
   if(currentTime_b >= (cloopTime_b + 1000))
   {     
      cloopTime_b = currentTime_b;              // Updates cloopTime
      // Pulse frequency (Hz) = 7.5Q, Q is flow rate in L/min. (Results in +/- 3% range)
      l_min_b = (flow_frequency_b / 7.5); // Pulse frequency / 7.5Q = flow rate in L/min 
      flow_frequency_b = 0;                   // Reset Counter
   }

// Controller
   currentTime_c = millis();
   // Every second, calculate and print litres/hour
   if(currentTime_c >= (cloopTime_c + 1000))
   {     
      cloopTime_c = currentTime_c;              // Updates cloopTime
      // Pulse frequency (Hz) = 7.5Q, Q is flow rate in L/min. (Results in +/- 3% range)
      l_min_c = (flow_frequency_c / 7.5); // Pulse frequency / 7.5Q = flow rate in L/min 
      flow_frequency_c = 0;                   // Reset Counter
   }

// Motor
   currentTime_m = millis();
   // Every second, calculate and print litres/hour
   if(currentTime_m >= (cloopTime_m + 1000))
   {     
      cloopTime_m = currentTime_m;              // Updates cloopTime
      // Pulse frequency (Hz) = 7.5Q, Q is flow rate in L/min. (Results in +/- 3% range)
      l_min_m = (flow_frequency_m / 7.5); // Pulse frequency / 7.5Q = flow rate in L/min 
      flow_frequency_m = 0;                   // Reset Counter
   }

// Print temperatures, flow and duty cycle on the LCD
// NOTE: Line number and character number start at 0 not 1

  lcd.setCursor(0,0); //Start at character 0 on line 0
  lcd.print("T=");
  if(currentTemperature(Battery) < 100)
  {
    lcd.setCursor(3,0);
    displayTemperature(Battery);  
  }
  else
  {
    lcd.setCursor(2,0);
    displayTemperature(Battery);  
  } 
//Column 2 controller
lcd.setCursor(7,0);
lcd.print("T=");
  if(currentTemperature(Controller) < 100)
  {
    lcd.setCursor(10,0);
    displayTemperature(Controller);  
  }
  else
  {
    lcd.setCursor(9,0);
    displayTemperature(Controller);  
  } 

//Column 3 motor
lcd.setCursor(15,0);
lcd.print("T=");
  if(currentTemperature(Motor) < 100)
  {
    lcd.setCursor(18,0);
    displayTemperature(Motor);  
  }
  else
  {
    lcd.setCursor(17,0);
    displayTemperature(Motor);  
  } 

//Row 2: PWM%
//Column 1 battery
 if (batteryPWM < 100)
 {
  lcd.setCursor(2,2);
  lcd.print(batteryPWM); // Prints battery pump duty cycle
  lcd.print("%"); 
 }
 else
 {
 lcd.setCursor(1,2);
 lcd.print(batteryPWM); // Prints battery pump duty cycle
 lcd.print("%");
 }

//Column 2 controller
 if (controllerPWM < 100)
 {
  lcd.setCursor(9,2);
  lcd.print(controllerPWM); // Prints controller pump duty cycle
  lcd.print("%"); 
 }
 else
 {
 lcd.setCursor(8,2);
 lcd.print(controllerPWM); // Prints controller pump duty cycle
 lcd.print("%");
 }

//Column 3 motor
 if (motorPWM < 100)
 {
  lcd.setCursor(17,2);
  lcd.print(motorPWM); // Prints motor pump duty cycle
  lcd.print("%"); 
 }
 else
 {
 lcd.setCursor(16,2);
 lcd.print(motorPWM); // Prints motor pump duty cycle
 lcd.print("%");
 }

//Row 3: Flow
//Column 1 battery
  lcd.setCursor(0,3);
  lcd.print("F="); 
  lcd.setCursor(3,3);
  lcd.print(l_min_b, DEC);

//Column 2 controller
  lcd.setCursor(7,3);
  lcd.print("F="); 
  lcd.setCursor(10,3);
  lcd.print(l_min_c, DEC);

//Column 3 motor
  lcd.setCursor(15,3);
  lcd.print("F="); 
  lcd.setCursor(18,3);
  lcd.print(l_min_m, DEC);
 
delay(500);
   
}//--(end main loop )---

/*-----( Declare User-written Functions )-----*/
void displayTemperature(DeviceAddress deviceAddress)
{ 
float tempC = sensors.getTempC(deviceAddress);
if (tempC == -127.00) // Measurement failed or no device found
{
lcd.print("Err");
} 
else
{
intTempC = 0.5 + tempC * 10;
lcd.print(intTempC / 10);
} 
}

int currentTemperature(DeviceAddress deviceAddress)
{ 
float mesTempC = sensors.getTempC(deviceAddress);
if (mesTempC == -127) // Measurement failed or no device found
{
return mesTempC= 100;
} 
else
return mesTempC;
}  
//*********( THE END )***********
