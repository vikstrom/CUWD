
#include "IRremote.h"
#include <utility/twi.h>
#include <Wire.h>

// RTC
const int MCP7940 = 0x6F; // Address of MCP7940 see data sheets
const int TIME_REG = 0;	// Sets the register to timekeeping register
const int CONTROL_REG = 0x7;	// Control register
const int ALARM0_REG = 0xA; 	// Alarm 0 register
//int MCP7940_REG = 0;	// Active register
 
// Initializes all values: 
byte second = 0;
byte minute = 0;
byte hour = 0;
byte weekday = 0;
byte date = 0;
byte month = 0;
byte year = 0;
byte controlRegister = 0;



//Defining pins
#define EN_pin_SMPS A2	//Enable pin for 5V reg. Keep low when sleeping
#define MTP_RTC A3		//Alarm pin from RTC-chip, active LOW, require PULL UP
#define SDA_pin A4		//I2C SDA pin, require PULL UP
#define SCL_pin A5		//I2C SCL pin, require PULL UP
#define RECV_PIN 0		//IR signal pin, input
//#define IR_PWR 3		//Power for IR module, this allowes the uC to easily switch this on and off 
						//during sleep to save some power.
#define Motor_PWM_pin 5 //PWM signal pin for motorcontroller
#define VCC_MEAS_PIN A1	//Pin to monitor level of RTC_VCC net. For enabling the SMPS if it goes to low.

IRrecv irrecv(RECV_PIN);	//Initiate IR Receiver
decode_results results;		//For storing received codes
uint16_t lastCode = 0;		//Storing last code reveived

#define BUTTON_POWER 0xD827	//Codes for buttons on IR-remote
#define BUTTON_A 0xF807
#define BUTTON_B 0x7887
#define BUTTON_C 0x58A7
#define BUTTON_UP 0xA05F
#define BUTTON_DOWN 0x00FF
#define BUTTON_LEFT 0x10EF
#define BUTTON_RIGHT 0x807F
#define BUTTON_CIRCLE 0x20DF

uint16_t BUTTON_CURRENT = 0;


//// Menu items ////

// alarmSet menu items
uint16_t alarmIsSet = 0;
uint16_t alarmSetMenuActive = 0;	// Zero indicates the menu is not active
uint16_t alarmSetHoursActive = 1;
uint16_t alarmSetMinutesActive = 0;
uint16_t alarmSetDaysActive = 0;
uint16_t alarmDate = 0;
uint16_t alarmDay = 0;
uint16_t alarmMonth = 0;
uint16_t alarmHours = 0;
uint16_t alarmMinutes = 0;

const uint16_t MINUTE_CHANGE = 5;
const uint16_t MAX_MINUTE = 55;
const uint16_t MAX_HOUR = 23;

// Calibration
uint16_t calibrationMenuActive = 0;
uint16_t calibrationPullActive = 0;
unsigned long calibrationStartTime = 0;
unsigned long calibrationTime = 1000;	// Standard 1s

// Motor
uint16_t motorRunning = 0;
unsigned long motorStartTime = 0;
uint16_t motorAlarmTest = 0;


void resetMenu()
{
	alarmSetMenuActive = 0;
	calibrationMenuActive = 0;
	calibrationPullActive = 0;
	stopMotor();
}


void handleIr()
{
	uint16_t resultCode = (results.value & 0xFFFF);	//Get value and mask

	switch (resultCode) 
	{
	    case BUTTON_POWER:
	    	Serial.println("POWER button");
	    	alarmOnOff();
	      break;

	    case BUTTON_A:
	     	Serial.println("A button");
	     	if(alarmSetMenuActive)
	     	{
	     		writeToRTC(ALARM0_REG);
	     		Serial.println("Write complete!");
	     		readFromRTC(ALARM0_REG);
	     		Serial.println("Current Alarm: ");
	     		printTime();
	     	}
	     	else if(!alarmSetMenuActive)
	     	{
		     	readFromRTC(TIME_REG);
		     	setAlarmMenu();
		    }
	      break;

	    case BUTTON_B:
	     	Serial.println("B button");
	     	calibration();
	      break;

	    case BUTTON_C:
	     	Serial.println("C button");
	     	testAlarm();
	      break;

	    case BUTTON_UP:
	     	Serial.println("UP button");
	     	if(alarmSetMenuActive)
	     	{
	     		if(alarmSetHoursActive && alarmHours < MAX_HOUR)
	     			alarmHours++;
	     			else if(alarmSetHoursActive && alarmHours == MAX_HOUR)
	     			{
	     				alarmHours = 0;
	     				alarmDate++;
	     			}

	     		if(alarmSetMinutesActive && alarmMinutes < (MAX_MINUTE - 5))
	     			alarmMinutes = alarmMinutes + MINUTE_CHANGE;
	     			else if(alarmSetMinutesActive && alarmMinutes == MAX_MINUTE)	
	     				alarmMinutes = 0;
	     		
	     		setAlarmMenu();
	     	}
	      break;

	     case BUTTON_DOWN:
	     	Serial.println("DOWN button");
	     	if(alarmSetMenuActive)
	     	{
	     		if(alarmSetHoursActive && alarmHours > 0)
	     			alarmHours--;
	     			else if(alarmSetHoursActive && alarmHours == 0)
	     			{
	     				alarmHours = MAX_HOUR;
	     				alarmDate--;
	     			}

	     		if(alarmSetMinutesActive && alarmMinutes > 0)
	     			alarmMinutes = alarmMinutes - MINUTE_CHANGE;
	     			else if(alarmSetMinutesActive && alarmMinutes == 0)
	     				alarmMinutes = MAX_MINUTE;
	     		
	     		setAlarmMenu();
	     	}
	      break;

	    case BUTTON_LEFT:
	     	Serial.println("LEFT button");
		     if(alarmSetMenuActive)
		     {	
		     	if(alarmSetMinutesActive)
		     	{
		     		alarmSetMinutesActive = 0;
		     		alarmSetHoursActive = 1;
		     	}
		     	else if(alarmSetHoursActive)
		     	{
		     		alarmSetHoursActive = 0;
		     		alarmSetDaysActive = 1;
		     	}
		    }
	      break;

	    case BUTTON_RIGHT:
	     	Serial.println("RIGHT button");
	     	if(alarmSetMenuActive)
		     {	
		     	if(alarmSetHoursActive)
		     	{
		     		alarmSetMinutesActive = 1;
		     		alarmSetHoursActive = 0;
		     	}
		    }
	      break;

	    case BUTTON_CIRCLE:
	     	Serial.println("CIRCLE button");
	     	if(calibrationMenuActive && !calibrationPullActive)
		    {
		    	startMotor();
		    	calibrationStartTime = millis();
		    	calibrationPullActive = 1;
		    	Serial.println("Calibration in progress...");
		    }
	     	else if(calibrationMenuActive && calibrationPullActive)
	     	{
	     		stopMotor();
	     		calibrationTime = millis() - calibrationStartTime;
	     		calibrationPullActive = 0;
	     		Serial.print("Calibration complete, time was:");
	     		Serial.println(calibrationTime);
	     		resetMenu();
	     	}
	      break;

	    case 0xFFFF:
	    	break;

	    default:
	      	Serial.println("Uknown code");
	      	break;
	}
	irrecv.resume();	//Receive next value
}

void startMotor()
{
	motorStartTime = millis();
/*	for(int i = 0; i < 13; i++)
	{
		analogWrite(Motor_PWM_pin, i);
		delay(10);
	}
/*	for(int i = 0; i < 100; i++)
	{
		analogWrite(Motor_PWM_pin, i);
		delay(2);
	}
*/	for(int i = 0; i < 200; i++)
	{
		analogWrite(Motor_PWM_pin, i);
		delay(1);
	}
}

void stopMotor()
{
	analogWrite(Motor_PWM_pin, 0);
}


void alarmOnOff()	// Sets alarm in RTC register
{
	resetMenu();
	readFromRTC(CONTROL_REG);
	if(controlRegister & B00010000)	// If bit 4 (ALM0) is set
	{	
		controlRegister = (controlRegister & B11101111);	// Mask all bits but bit 4 with "AND 1". Sets bit 4 = 0.
		writeToRTC(CONTROL_REG);
		Serial.println("Alarm turned off");
		alarmIsSet = 0;
	}
	else if ( !(controlRegister & B00010000) )
	{
		controlRegister = (controlRegister | B00010000);	// Mask all bits but bit 4 with "OR 0". Sets bit 4 = 1.
		writeToRTC(CONTROL_REG);
		Serial.println("Alarm turned on");
		readFromRTC(ALARM0_REG);
		printTime();
		alarmIsSet = 1;
	}

}

void setAlarmMenu()
{
	if(!alarmSetMenuActive)
	{
		resetMenu();
		alarmMinutes = minute;
		alarmHours = hour;
		alarmDate = date;
		alarmDay = weekday;
		alarmMonth = month;
	}
	alarmSetMenuActive = 1;	// Set to active menu
	Serial.println("Setting alarm...: ");
	Serial.print("Alarm day: ");
	Serial.println(alarmDate);
	Serial.print("Alarm time: ");
	Serial.print(alarmHours);
	Serial.print(":");
	Serial.println(alarmMinutes);
	Serial.println();
}

void calibration()
{
	resetMenu();
	calibrationMenuActive = 1;
	Serial.println("Let's calibrate!");
	Serial.println("Start and stop with CENTER");
}

void testAlarm()
{
	motorAlarmTest = 1;
	startMotor();
}

void readFromRTC(int MCP7940_REG) 
{
	
	switch (MCP7940_REG) {

	    case TIME_REG:
			Wire.beginTransmission(MCP7940);
			Wire.write(MCP7940_REG);
			Wire.endTransmission();
			Wire.requestFrom(MCP7940, 7);
			second = bcdToDec(Wire.read());
			minute = bcdToDec(Wire.read());
			hour = bcdToDec(Wire.read());
			weekday = bcdToDec(Wire.read());
			date = bcdToDec(Wire.read());
			month = bcdToDec(Wire.read());
			year = bcdToDec(Wire.read());
     	break;

	    case CONTROL_REG:
			Wire.beginTransmission(MCP7940);
			Wire.write(MCP7940_REG);
			Wire.endTransmission();
			Wire.requestFrom(MCP7940, 1);
			controlRegister = Wire.read();
    	break;

	    case ALARM0_REG:
			Wire.beginTransmission(MCP7940);
			Wire.write(MCP7940_REG);
			Wire.endTransmission();	
			Wire.requestFrom(MCP7940, 6);
			second = bcdToDec(Wire.read());
			minute = bcdToDec(Wire.read());
			hour = bcdToDec(Wire.read());
			weekday = bcdToDec(Wire.read());
			date = bcdToDec(Wire.read());
			month = bcdToDec(Wire.read());  
		break; 	

	   	default:
		break;
	}
}

void writeToRTC(int MCP7940_REG)
{
	switch (MCP7940_REG) {

	    case CONTROL_REG:
			Wire.beginTransmission(MCP7940);
			Wire.write(MCP7940_REG);
			Wire.write(controlRegister);
			Wire.endTransmission();
		break;

	    case ALARM0_REG:
			Wire.beginTransmission(MCP7940);
			Wire.write(MCP7940_REG);
			Wire.write(decToBcd(second));
			Wire.write(decToBcd(alarmMinutes));
			Wire.write(decToBcd(alarmHours));
			Wire.write(decToBcd(alarmDay) | B11110000);	//Also sets alarm comparator bits
			Wire.write(decToBcd(alarmDate));
			Wire.write(decToBcd(alarmMonth));
			Wire.endTransmission();
	    break;''

	    default:
	    break;
	      // do something
	}
}

void printTime() 
{	
	Serial.print(year);
	Serial.print("-");
	Serial.print(month);
	Serial.print("-");
	Serial.print(date);
	Serial.print("	");
	Serial.print(hour);
	Serial.print(":");
	Serial.println(minute);

}

byte decToBcd(byte val)	
{
 	return ((val/10*16) + (val%10));
}
byte bcdToDec(byte val) 
{
	return ((val/16*10) + (val%16));
}


void setup() 
{
	Serial.begin(9600);
	Wire.begin();
	irrecv.enableIRIn();	//Start receiver
	pinMode(EN_pin_SMPS, OUTPUT);
	pinMode(MTP_RTC, INPUT_PULLUP);
	pinMode(Motor_PWM_pin, OUTPUT);
	pinMode(RECV_PIN, INPUT_PULLUP);
	pinMode(VCC_MEAS_PIN, INPUT);
	//digitalWrite(EN_pin_SMPS, HIGH);	//Enable SMPS 5V
	readFromRTC(TIME_REG);
	printTime();
  }

void loop() 
{
	
	if(irrecv.decode(&results))
		handleIr();
	
	if(((millis() - motorStartTime) >= calibrationTime) && motorAlarmTest == 1)
	{	
		stopMotor();
		motorAlarmTest = 0;
	}
}

