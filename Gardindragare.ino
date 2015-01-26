
#include "IRremote.h"
#include <utility/twi.h>
#include <Wire.h>

// RTC
const int MCP7940 = 0x6F; // Address of MCP7940 see data sheets
const int TIME_REG = 0;	// Sets the register to timekeeping register
const int CONTROL_REG = 0x7;	// Control register
const int ALARM0_REG = 0xA; 	// Alarm 0 register
//int MCP7940_REG = 0;	// Active register

// Led driver SAA1064
const int SAA1064 = 0x76 >> 1;

 
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
#define MFP_RTC A3		//Alarm pin from RTC-chip, active LOW, require PULL UP
#define SDA_pin A4		//I2C SDA pin, require PULL UP
#define SCL_pin A5		//I2C SCL pin, require PULL UP
#define RECV_PIN 0		//IR signal pin, input
//#define IR_PWR 3		//Power for IR module, this allowes the uC to easily switch this on and off 
						//during sleep to save some power.
#define Motor_PWM_pin 5 //PWM signal pin for motorcontroller
#define VCC_MEAS_PIN A1	//Pin to monitor level of RTC_VCC net. For enabling the SMPS if it goes to low.

IRrecv irrecv(RECV_PIN);	//Initiate IR Receiver
decode_results results;		//For storing received codes
int lastCode = 0;		//Storing last code received

#define BUTTON_POWER 0xD827	//Codes for buttons on IR-remote
#define BUTTON_A 0xF807
#define BUTTON_B 0x7887
#define BUTTON_C 0x58A7
#define BUTTON_UP 0xA05F
#define BUTTON_DOWN 0x00FF
#define BUTTON_LEFT 0x10EF
#define BUTTON_RIGHT 0x807F
#define BUTTON_CIRCLE 0x20DF

int digit[10] = {63, 6, 91, 79, 102, 109, 125,7, 127, 111};	//{0,1,2,3,4,5,6,7,8,9}
	
// struct letterList {
// 	int A,C,E,F,L,N,O,R,S,T;
// };

//letterList letter = {119, 57, 121, 113, 56, 84, 65, 80, 109, 120};	//{A,C,E,F,L,N,O,R,S,T}
int letter[10] ={119, 57, 121, 113, 56, 84, 63, 80, 109, 120}; //{A,C,E,F,L,N,O,R,S,T}


//// Menu items ////

// alarmSet menu items
int alarmIsSet = 0;
int alarmSetMenuActive = 0;	// Zero indicates the menu is not active
int alarmSetHoursActive = 1;
int alarmSetMinutesActive = 0;
int alarmSetDaysActive = 0;
int alarmDate = 0;
int alarmDay = 0;
int alarmMonth = 0;
int alarmHours = 0;
int alarmMinutes = 0;

const int MINUTE_CHANGE = 1;	//Number of minutes to change for each click
const int MAX_MINUTE = 60 - MINUTE_CHANGE;
const int MAX_HOUR = 23;

// Calibration
int calibrationMenuActive = 0;
int calibrationPullActive = 0;
unsigned long calibrationStartTime = 0;
unsigned long calibrationTime = 1000;	// Standard 1s

// Motor
int motorRunning = 0;
unsigned long motorStartTime = 0;
int motorAlarmTest = 0;

//Time
long timeTimer = millis();

void resetMenu()
{
	alarmSetMenuActive = 0;
	calibrationMenuActive = 0;
	calibrationPullActive = 0;
	stopMotor();
}


void handleIr()
{
	int resultCode = (results.value & 0xFFFF);	//Get value and mask

	switch (resultCode) 
	{
	    case BUTTON_POWER:
	    	Serial.println("POWER button");
	    	alarmOnOff();	//Switches the alarm
	      break;

	    case BUTTON_A:
	     	Serial.println("A button");
	     	if(alarmSetMenuActive)	//If the user currently is setting the alarm
	     	{
	     		writeToRTC(ALARM0_REG);
	     		Serial.println("Write complete!");
	     		printMsg("SET", 1);
	     		delay(1000);
	     		readFromRTC(ALARM0_REG);
	     		printNum(hour,minute,0);
	     		Serial.print("Current Alarm: ");
	     		printTime();
	     	}
	     	else if(!alarmSetMenuActive)	//Else the user wants to start setting the alarm
	     	{
		     	readFromRTC(TIME_REG);
		     	setAlarmMenu();
		    }
	      break;

	    case BUTTON_B:
	     	Serial.println("B button");
	     	calibration();	//Starts the calibration
	      break;

	    case BUTTON_C:
	     	Serial.println("C button");
	     	Alarm(1);	//Test button
	      break;

	    case BUTTON_UP:
	     	Serial.println("UP button");
	     	if(alarmSetMenuActive)	//If user is setting the alarm
	     	{
	     		if(alarmSetHoursActive && alarmHours < MAX_HOUR)
	     			alarmHours++;
	     			else if(alarmSetHoursActive && alarmHours == MAX_HOUR)
	     			{
	     				alarmHours = 0;
	     				alarmDate++;
	     			}

	     		if(alarmSetMinutesActive && alarmMinutes < (MAX_MINUTE))
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
		     if(alarmSetMenuActive)	//If user is setting alarm, change value to be altered
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
	     	if(alarmSetMenuActive)	//Same thing as above
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

	motorRunning = 1;
}

void stopMotor()
{
	analogWrite(Motor_PWM_pin, 0);
	motorRunning = 0;
}


void alarmOnOff()	// Sets alarm in RTC register
{
	resetMenu();
	readFromRTC(CONTROL_REG);
	byte controlRegisterTemp = controlRegister;
	Serial.println(controlRegister);
	if(controlRegister & B00010000)	// If bit 4 (ALM0) is set
	{	
		controlRegister = (controlRegister & B11101111);	// Mask all bits but bit 4 with "AND 1". Sets bit 4 = 0.
		writeToRTC(CONTROL_REG);
		printMsg("OFF",1);
		Serial.println("Alarm turned off");
		Serial.println(controlRegister);
		alarmIsSet = 0;
	}
	else if ( ((controlRegister & B00010000) == 0))
	{
		controlRegister = (controlRegister | B00010000);	// Mask all bits but bit 4 with "OR 0". Sets bit 4 = 1.
		Serial.println(controlRegister);
		writeToRTC(CONTROL_REG);
		int checkMFP = 0;						//It's important to check if the MFP goes high when setting the register
		unsigned long timer = millis();			//And to stop it triggering the motor when we don't want to
		while(((millis() - timer) < 50) && (checkMFP == 0)) {
		    checkMFP = digitalRead(MFP_RTC);
			if(checkMFP) {
				controlRegister = controlRegisterTemp;
				writeToRTC(CONTROL_REG);
				printMsg("ERR",1);
				Serial.println("Error, set new alarm-time before turning on alarm");
			}
		}
		if (checkMFP == 0) {
		printMsg("ON",0);
		Serial.println("Alarm turned on");
		Serial.println(controlRegister);
		readFromRTC(ALARM0_REG);
		printTime();
		alarmIsSet = 1;
		}
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
	printNum(alarmHours,alarmMinutes,0);
	Serial.println("Setting alarm...: ");
	Serial.print("Alarm day: ");
	Serial.println(alarmDate);
	Serial.print("Alarm time: ");
	Serial.print(alarmHours);
	Serial.print(":");
	if(alarmMinutes <= 9)
	{
		Serial.print("0");
		Serial.println(alarmMinutes);
	}
	else
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
			Wire.write(decToBcd(0));	//Always write 0 to seconds when setting the alarm
			Wire.write(decToBcd(alarmMinutes));
			Wire.write(decToBcd(alarmHours));
			Wire.write(decToBcd(alarmDay | B11110000));	//Also sets alarm comparator bits
			Wire.write(decToBcd(alarmDate));
			Wire.write(decToBcd(alarmMonth));
			Wire.endTransmission();
	    break;

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
	if(minute <= 9)
	{
		Serial.print("0");
		Serial.println(minute);
	}
	else
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

void resetControlRegister()
{
	readFromRTC(CONTROL_REG);
	if(controlRegister != B00000000)	//Resetting control register so that the alarm output is not triggering the motor
	{
		controlRegister = B00000000;
		writeToRTC(CONTROL_REG);
		Serial.println("controlRegister byte is set to 0");
	}

}

void Alarm(int alarmTrig)
{	
	if(alarmTrig) {
		Serial.println("ALARM!");
		resetControlRegister();
		startMotor();
	}
}

void motorControl(int motorStatus)
{
	if(((millis() - motorStartTime) >= calibrationTime) && !calibrationPullActive)
	{	
		stopMotor();
	}

}

void printNum(int hour, int minute, int dot) 
{
	clearDisplay();
	int display[4] = {0,0,0,0};

	if (hour > 9) {
		display[2] = digit[hour/10];
		display[3] = digit[hour%10];

	}

	else {
		display[3] = digit[hour];
	}

	if (minute > 9) {
		display[0] = digit[minute/10];
		display[1] = digit[minute%10];
	}

	else {
		display[0] = digit[0];
		display[1] = digit[minute];
	}

	display[dot] = display[dot] + 128;	//Adding dot

	sendToDriver(display);	
}

void printMsg(String letters, int dot)
{	
	clearDisplay();

	int display[4] = {0,0,0,0};
		if(letters == "SET") {
		    display[2] = letter[8];
		    display[3] = letter[2];
		    display[0] = letter[9];
		}
	    else if(letters == "ON") {
	      	display[2] = letter[6];
	    	display[3] = letter[5];
	    }

	    else if(letters == "OFF") {
	    	display[2] = letter[6];
	    	display[3] = letter[3];
	    	display[0] = letter[3];
	    }

	    else if(letters == "ERR") {
	    	display[2] = letter[2];
	    	display[3] = letter[7];
	    	display[0] = letter[7];
	    }

	    else if(letters == "CAL") {
	    	display[2] = letter[1];
	    	display[3] = letter[0];
	    	display[0] = letter[4];
	    }

	    display[dot] = display[dot] + 128;

	    sendToDriver(display);
}

void sendToDriver(int display[])
{	
	Wire.beginTransmission(SAA1064);
	Wire.write(1);
	Wire.write(display[0]);	//Sending to display
	Wire.write(display[1]);
	Wire.write(display[2]);
	Wire.write(display[3]);
	Wire.endTransmission();
}

void clearDisplay()
{
	 Wire.beginTransmission(SAA1064);
	 Wire.write(1); // start with digit 1 (right-hand side)
	 Wire.write(0); // blank digit 1
	 Wire.write(0); // blank digit 2
	 Wire.write(0); // blank digit 3
	 Wire.write(0); // blank digit 4
	 Wire.endTransmission();
}

void initDisplay()
{
	 Wire.beginTransmission(SAA1064);
	 Wire.write(B00000000); // this is the instruction byte. Zero means the next byte is the control byte
	 Wire.write(B0110111); // control byte (dynamic mode on, digits 1+3 on, digits 2+4 on, 9mA segment current
	 Wire.endTransmission();
}

void printCurrentTime()
{	
		readFromRTC(TIME_REG);
		printNum(hour, minute, 0);
	
}

//////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
void setup() 
{
	Serial.begin(9600);
	Wire.begin();
	irrecv.enableIRIn();	//Start receiver
	pinMode(EN_pin_SMPS, OUTPUT);
	pinMode(MFP_RTC, INPUT);
	pinMode(Motor_PWM_pin, OUTPUT);
	pinMode(RECV_PIN, INPUT_PULLUP);
	pinMode(VCC_MEAS_PIN, INPUT);
	initDisplay();
	//digitalWrite(EN_pin_SMPS, HIGH);	//Enable SMPS 5V
	resetControlRegister();
	printTime();
	printCurrentTime();
	delay(2000);
	clearDisplay();

	
  }

void loop() 	//Polls the ir-input, runs the motorControl and checks if the alarm pin is high.
{
	
	if(irrecv.decode(&results))	//If there's something on the input
		handleIr();

	motorControl(motorRunning);	

	Alarm(digitalRead(MFP_RTC));

	//timeTimer = printCurrentTime(timeTimer);	//Check for timeout and if, set new timer.
	// int delaytime = 1000;
	// printMsg("CAL", 1);
	// delay(delaytime);
	// printMsg("SET", 1);
	// delay(delaytime);
	// printMsg("ON", 0);
	// delay(delaytime);
	// printMsg("OFF", 1);
	// delay(delaytime);
	// printCurrentTime();
	// delay(delaytime);

	

}

