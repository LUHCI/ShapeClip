/*
	ShapeClip Firmware
	This is logic code used in the ShapeClip paper.
		JH - John Hardy
		CW - Christian Weichel
		JV - John Vidler
		MS - Matthias Schitthelm
	
	v2.0 - JH - Second version, re-factored and ready for UIST + profiling.
	v1.6 - JV - Updated modes.
	v1.5 - JV - Serial comms added.
	v1.4 - JH - Height mode added.
	v1.3 - JH - Added modes.
	v1.2 - CW - Added LDR.h and LDR.cpp
	v1.1 - CW + MS - Added cool stepper.
	v1.0 - JH - First version, sync-pulse based control of RGB and Height
*/

/* External and 3rd party libraries. */ 
#include <Math.h>
#include <EEPROM.h>
#include <util/parity.h>
#include "CoolStepper.h" 
#include "LDR.h"
#include "Adafruit_NeoPixel.h"
#include "RunningVariance.h"
#include "WindowVariance.h"
#include "Kalman.h"

/* IO Pins */
#define PIN_LED   A5		// The on-board LED.
#define PIN_SWTOP 2			// The top switch (digital).
#define PIN_SWBOT 3			// The bottom switch (digital).
#define PIN_FORCE A0		// The force input (analog).
#define PIN_TOUCH 8  		// The touch output (digital).
#define PIN_RGB_OUT 9		// The RGB LED (digital).
#define PIN_MIN11 4			// Motor Coil 1 + (digital).
#define PIN_MIN12 5			// Motor Coil 1 - (digital).
#define PIN_MIN21 6			// Motor Coil 2 + (digital).
#define PIN_MIN22 7			// Motor Coil 2 - (digital).
#define PIN_RND   A4		// Random data source pin.
#define PIN_LDR1  A6		// The LDR1 input (analog).
#define PIN_LDR2  A7		// The LDR2 input (analog).

/* Motor characteristics */
#define MOTOR_ANGLE 20		// The degrees turned with each motor step.
#define MOTOR_SPEED 1500	// The motor speed in rotations per minute (RPMs).
#define MOTOR_TRAVEL 470	// The number of steps to move the nut from the top to the bottom.

/* Sync pulse mode settings. */
const float SYNC_SIGNAL_DROP_THRESHOLD = 3.0f; 				// Based on the behaviour of the LDR with a 10k resistor on a Dell LCD monitor.
const int PULSE_COUNT = 5;									// There are 5 pulses: high, low, r, g, b
const int PULSE_WIDTH = 200;								// The amount of time each pulse last (ms).
const int PULSE_WIDTH_ERR = 10;								// The amount of error in each pulse width start and end (~10ms due to 4ms timers in JS).
const int DATA_WIDTH  = PULSE_WIDTH * PULSE_COUNT;			// The total amount of time for an entire data pulse.
const int DATA_WIDTH_ERR = PULSE_WIDTH_ERR * PULSE_COUNT; 	// The total amount of error for an entire data pulse.
const int SAMPLE_OFFSET = 50;								// The amount of time (ms) to offset the LDR sampling start (relative to the sync drop).
const int SAMPLE_WINDOW = 90;								// The amount of time (ms) to sample the LDR for.

#define PULSE_STATE__NOT_CHECKED  -1	// The sync-pulse routine did not bother checking, too soon since previous check.
#define PULSE_STATE__NO_PULSE      0	// The sync-pulse check returned no pulse.. Probably off a screen.
#define PULSE_STATE__ACTIVE_PULSE  1	// The sync-pulse check returned a pulse! It's probably on a screen.

/** Sensor settings. */
#define ON_SCREEN_TIMEIN 1000			// The amount of time before height mode accepts an on screen value. One second in ms.
#define LDR_MIN_LIMIT 120				// The smallest acceptable delta between the min and max values of the LDR.
#define LDR_MAX_LIMIT 1000				// The largest acceptable delta between the min and max values of the LDR.
#define LDR_OFFSCREEN_DELTA_GAP 100 	// The largest gap between the LDR values at which it is considered off-screen.
#define SSMODE_LDR_SYMB_LOWER -450		// The cap for the lowest LDR symbol.
#define SSMODE_LDR_SYMB_UPPER 500		// The cap for the highest LDR symbol.

/* Application Behaviours */
#define MODE_PROFILING 			// Does the Clip accept basic serial commands: p,s,f (ping, start, finish).
#define PROFILING_RATE 50U		// The rate at which we stream data to serial in profiling mode.
#define SSMODE_8BIT				// Run screen-serial in 8 bit mode (recommended).
//#define SSMODE_16BIT			// Run screen-serial in 16 bit mode (not recommended or tested that much).

/* Serial Debugging modes. */
//#define HEIGHT_DEBUG			// Height mode: Enable serial print of debug messages.
//#define SYNCPULSE_DEBUG		// Sync pulse mode: Enable serial print of debug messages.
//#define SSMODE_CMDPRINT		// Screen serial mode: Enable serial print of command buffer interpretations.
#define SSMODE_STREAMPRINT	// Screen serial mode: Enable serial print of bitstream data for debugging the serial connection.

//#define SERIAL_INFO			// Should device values be streamed via serial? csv: ms, ldr1, ldr2, rgb, height, frc, haspulse, swbot
//#define SERIAL_SYNC_DEBUG		// Should sync pulse debug data be streamed via serial? csv: ms, r, g, b
//#define SERIAL_SYNC_DEBUG2	// Should the verbose sync pulse debug data be streamed via serial? csv: ms, slope, max, r, g, b, min, raw

/** Colour modes - how the RGB LED is controlled. */
#define RGBMODE_NONE     0		// The RGB LED displays nothing.
#define RGBMODE_SCREEN   1		// The RGB LED displays the last colour the pulses detected.
#define RGBMODE_NOSCREEN 2		// The RGB LED displays the "not on screen" pattern (i.e. no pulse detected).
#define RGBMODE_STARTUP  3		// The RGB LED displays the "setting up / calibrating" pattern.

/* Colour manipulation macros. */
#define RGB(r,g,b) 	(((r & 0xFF) << 16) | ((g & 0xFF) << 8)  | ((b & 0xFF) << 0))
#define RED(hex) 	hex >> 16 & 0xFF
#define GREEN(hex) 	hex >>  8 & 0xFF
#define BLUE(hex) 	hex >>  0 & 0xFF

/** EEPROM data settings. */
#define EEMODEADDR 0			// The address of the mode stored in EEPROM.
#define EEMODE_CHANGETIME 5000  // The number of ms to hold the switch before the clip mode is changed.
#define EEMODE_SYNCPULSE  45	// The Clip looks for a sync pulse on one of the LDRs.
#define EEMODE_HEIGHTONLY 37	// The Clip uses both LDRs to estimate height, no colour involved.
#define EEMODE_SERIAL     89	// The Clip can send / recieve serial data with a simple protocol.



/**
 * @brief Check to see if code should be called based on the timer.
 * This can be used to create a rudimentary timer.  It is called once per logic-loop.
 * If it returns true, then the timer handler code should be run.  If false, then it should not.
 * Based on: http://forum.arduino.cc/index.php/topic,5686.0.html
 * Usage: ulong timer = 0; if (cycleCheck(&timer, 10u) { log("called every 10 ms"); }
 * @param *iLastMs Pointer to a value which contains the last time this was checked.
 * @param iCycle The number of ms since the last time required to trigger.
 * @return True if the time since the last call has elasped and the code should run. False if not.
 */
boolean cycleCheck(unsigned long *iLastMs, unsigned int iCycle)  {
	unsigned long ms = millis();
	if(ms - *iLastMs >= iCycle)
	{
		*iLastMs = ms;
		return true;
	}
	else
	{
		return false;
	}
}

/**
 * @brief Reset a timer that is used with the cycleCheck function.
 * @param *iLastMs Pointer to a value which contains the last time this was checked.
 * This will update the iLastMs value to the current time.
 */
void cycleReset(unsigned long *iLastMs) {
	*iLastMs = millis();
}

/**
 * @brief Work out if a time value falls within a sample window. 
 * @param iRelativeTime The time to check.
 * @param iSampleOffset The sampling offset start (relative to 0 in iRelativeTime).
 * @param iWindowSize The sampling window size.
 * @return True if (iSampleOffset+iWindowSize) > iRelativeTime > iSampleOffset.
 */
boolean checkSample(int iRelativeTime, int iSampleOffset, int iWindowSize) {
	return (iRelativeTime > iSampleOffset) && ( iRelativeTime < (iSampleOffset + iWindowSize));
}


// ShapeClip v1 controls and live values.
Stepper motor(MOTOR_ANGLE, PIN_MIN11, PIN_MIN12, PIN_MIN21, PIN_MIN22);
Adafruit_NeoPixel pRGB = Adafruit_NeoPixel(1, PIN_RGB_OUT, NEO_GRB + NEO_KHZ800);
LDR ldr1(PIN_LDR1);
LDR ldr2(PIN_LDR2);

boolean bMotorCalibrated = false;	// Is the motor position value accurate?
int iMotorPosition = 0;				// The current number of steps in the motor.

int eClipMode = EEMODE_SYNCPULSE;	// Current Clip mode.
int eRGBMode = RGBMODE_NONE;		// Current RGB mode.
bool bOnScreen = false;				// Is the shape clip device on the screen (i.e. does it have a valid sync pulse)?

int iTargetR = 0;					// Target RED component.
int iTargetG = 0;					// Target GREEN component.
int iTargetB = 0;					// Target BLUE component.
int iTargetPos = 0;					// Target MOTOR height (steps).




/**
 * @brief Configure the chip pins to the proper states.
 */
void configure() {
	
	// On-board LED (output, off).
	pinMode(PIN_LED, OUTPUT);
	digitalWrite(PIN_LED, LOW);
	
	// LDR1 and LDR2 (input). NOT REQUIRED FOR ANALOG PINS - MAYBE HARMFUL
	//pinMode(PIN_LDR1, INPUT);
	//pinMode(PIN_LDR2, INPUT);
	
	// Top and bottom switches (input, active).
	pinMode(PIN_SWTOP, INPUT);
	digitalWrite(PIN_SWTOP, HIGH);
	pinMode(PIN_SWBOT, INPUT);
	digitalWrite(PIN_SWBOT, HIGH);
	
	// Touch sensor (output, low).
	pinMode(PIN_TOUCH, OUTPUT);
	digitalWrite(PIN_TOUCH, LOW);
	
	// Force sensor (input).
	pinMode(PIN_FORCE, INPUT);
	
	// Set up the motor.
	motor.setSpeed(MOTOR_SPEED);
	
	// Set up the RGB LED.
	pRGB.begin();
	pRGB.show();
	
	// On-board LED (on).
	digitalWrite(PIN_LED, HIGH);
}

/**
 * @brief This will send the motor DOWN until the SWBOT is triggered OR the full MOTOR_TRAVEL is used.
 * It will set the iMotorPosition value to 0 and not allow calls to moveMotor while operating.
 */
void zeroMotor() {
	
	// Zero flags.
	bMotorCalibrated = false;
	
	// Put us into setup mode (RGB LED).
	int eLastMode = eRGBMode;
	eRGBMode = RGBMODE_STARTUP;
	updateColour();
	
	// Until the motor is calibrated, step down each time.
	for(int iAvailableSteps = MOTOR_TRAVEL; iAvailableSteps >= 0; iAvailableSteps--) {
		motor.step(-1);

		//#ifdef SWBOT_MOTOR
		//if (digitalRead(PIN_SWBOT) == LOW) break;
		//#endif
	}
	
	// Set the motor position to 0 (known location).
	bMotorCalibrated = true;
	iMotorPosition = 0;
	
	// Shut the motor down.
	motor.shutdown();
	
	// Take us out of setup mode.
	eRGBMode = eLastMode;
	updateColour();
}

/**
 * @brief Move the motor one step towards the target position.
 */
void moveMotor() {
	
	// Only do this if the motor is calibrated.
	if (!bMotorCalibrated)
		return;
	
	// Compute the distance to travel.
	int iDelta = iTargetPos - iMotorPosition;
	
	// If we have to move over 1 step in either direction.
	if (abs(iDelta) > 1)
	{
		// Move one step in the correct direction and update the motor position.
		int step = iDelta >= 0 ? 1 : -1;
		motor.step(step);
		iMotorPosition += step;
	}
	
	// Otherwise shut the motor down.
	else
	{
		motor.shutdown();
	}
}

/**
 * @brief Display the correct colour on the RGB led based on the mode it is in.
 */
void updateColour() {
	
	// If we are in the normal mode.
	switch (eRGBMode)
	{
		case RGBMODE_NONE:
			pRGB.setPixelColor(0,  0, 0, 0);
			break;
		case RGBMODE_SCREEN:
			{
				// Go immediately to the new colour.
				// pRGB.setPixelColor(0,  iTargetR, iTargetG, iTargetB);
				
				// Get 1% closer to the target colour each iteration.
				uint32_t currentColor = pRGB.getPixelColor(0);
				int stepRed   = constrain(map(10, 0, 1000, RED(currentColor),   iTargetR), 0, iTargetR);
				int stepGreen = constrain(map(10, 0, 1000, GREEN(currentColor), iTargetG), 0, iTargetG);
				int stepBlue  = constrain(map(10, 0, 1000, BLUE(currentColor),  iTargetB), 0, iTargetB);
				
				// Set the step value.
				pRGB.setPixelColor(0,  stepRed, stepGreen, stepBlue);
			}
			break;
		case RGBMODE_NOSCREEN:
			pRGB.setPixelColor(0,  255, 0, 0);
			break;
		case RGBMODE_STARTUP:
			if (eClipMode == EEMODE_HEIGHTONLY)
				pRGB.setPixelColor(0,  255, 0, 0);
			else if (eClipMode == EEMODE_SYNCPULSE)
				pRGB.setPixelColor(0,  0, 255, 0);
			else
				pRGB.setPixelColor(0,  0, 0, 255);
			break;
	}
	
	// Push the new colour.
	pRGB.show();
}




/**
 * Setup the board and configure the pins.
 */
void setup() { 
	
	// Configure the board and pins.
	configure();
	
	// Start the serial port.
	Serial.begin(115200);
	
	// Wait two seconds.
	Serial.println("Boot v2.1");
	
	// Select a default clip mode from the set of possible modes, if one is not set.
	eClipMode = EEPROM.read(EEMODEADDR);
	switch( eClipMode )
	{
	  case EEMODE_SYNCPULSE:
		break;
	  case EEMODE_HEIGHTONLY:
		break;
	  case EEMODE_SERIAL:
		break;
	  default:
		eClipMode = EEMODE_HEIGHTONLY;
		EEPROM.write(EEMODEADDR, EEMODE_HEIGHTONLY);
	}

	// Debug Mode
	eClipMode = EEMODE_SERIAL;
	eRGBMode  = RGBMODE_SCREEN;
	detectModeChange( 1 );

	// Wait for a random time so that not all clips zero their motor at the same time.
	randomSeed(analogRead(PIN_RND));
	uint16_t randomWait = random(1000);
	delay(randomWait);
	
	// Bring the motor down until it hits the bottom.
	zeroMotor();
	
	// make sure we all start at the same time
	delay(1000 - randomWait);
}

/**
 * @brief Detect a mode change by bridging SWBOT.
 * Press SWBOT to change mode. Cycle through the modes every 1 second.
 * @param skip Set to 1 to emulate SWBOT being pressed.
 */
void detectModeChange( byte skip ) {
	
	// If SWBOT is not pressed, skip out.
	if( digitalRead(PIN_SWBOT) == HIGH && skip != 1 )
		return;
	
	// While we are still holding SWBOT down.
	while( digitalRead(PIN_SWBOT) == LOW )
	{
		// Swap mode.
		switch( eClipMode )
		{
			case EEMODE_HEIGHTONLY:
				pRGB.setPixelColor(0,  0, 255, 0);
				eRGBMode  = RGBMODE_SCREEN;
				eClipMode = EEMODE_SYNCPULSE;
				break;
			
			case EEMODE_SYNCPULSE:
				pRGB.setPixelColor(0,  0, 0, 255);
				eRGBMode  = RGBMODE_NONE;
				eClipMode = EEMODE_SERIAL;
				break;
			
			case EEMODE_SERIAL:
				pRGB.setPixelColor(0,  255, 0, 0);
				eRGBMode  = RGBMODE_SCREEN;
				eClipMode = EEMODE_HEIGHTONLY;
				break;
			
			default:
				pRGB.setPixelColor(0,  255, 0, 0);
				eRGBMode  = RGBMODE_NONE;
				eClipMode = EEMODE_HEIGHTONLY;
		}
		
		// Show the new colour for the mode and wait.
		pRGB.show();
		delay( 1000 );
	}
	
	// Now SWBOT has been released, flicker the colour 3 times.
	for( int i=0; i<3; i++ )
	{
		switch( eClipMode )
		{
			case EEMODE_HEIGHTONLY:
				pRGB.setPixelColor(0,  255, 0, 0);
				break;
			case EEMODE_SYNCPULSE:
				pRGB.setPixelColor(0,  0, 255, 0);
				break;
			case EEMODE_SERIAL:
				pRGB.setPixelColor(0,  0, 0, 255);
				break;
		}
		
		pRGB.show();
		delay( 100 );
		pRGB.setPixelColor(0,  0, 0, 0);
		pRGB.show();
		delay( 100 );
	}
	
	// Clear all target values on mode switch.
	iTargetR = 0;
	iTargetG = 0;
	iTargetB = 0;
	iTargetPos = 0;
	
	// Store new mode to EEPROM.
	EEPROM.write( EEMODEADDR, eClipMode );
}

/**
 * @brief Check the sync pulse and update relevant global variables.
 * This is automatically rate limited to 5ms samples.
 * It will update the iTargetR,G,B, iLDR1Max and iLDR1Min values if valid pulses are detected.
 * @return -1 if check not performed, 1 if we received an active pulse, 0 if we did not.
 */
int8_t checkPulse() {

	// Only allow this function to be called once every 5ms.
	static unsigned long tmr1 = 0;
	if (!cycleCheck(&tmr1, 5U))
		return PULSE_STATE__NOT_CHECKED;

	// Define static variables.
	static WindowVariance* pEstimatedMin = new WindowVariance(5);	// Running 
	static WindowVariance* pEstimatedMax = new WindowVariance(5);
	static int r=0; static int g=0; static int b=0;
	static int iLDR1Min = 0;
	static int iLDR1Max = 0;
	static float x1,x2,y1,y2 = 0;	// Required for slope differentials.
	static float fSlope = 0.0f;		// Required for slope differentials.
	static long iLastMinima = 0;	// The time of the last minimal (ms).

	// Read the LDR and reset the serial debug colour values.
	int iLDRValue = analogRead(PIN_LDR1);
	r=0; g=0; b=0;

	// Compute the slope (dLDR/dTime) by comparing it to the previous data.
	long iCurrentTime = millis();
	x1 = x2; y1 = y2;
	x2 = (float)iCurrentTime; y2 = (float)iLDRValue;
	fSlope = (y1-y2) / (x1-x2);

	// If we are hitting a big dip AND it has been over 50ms since the last one.
	int iDeltaTime = iCurrentTime - iLastMinima;
	boolean bSlopeDetected = fSlope < -SYNC_SIGNAL_DROP_THRESHOLD;
	boolean bWithinPulseRange = iDeltaTime > (DATA_WIDTH - DATA_WIDTH_ERR); // &&  delta < (DATA_WIDTH + DATA_WIDTH_ERR + PULSE_WIDTH + PULSE_WIDTH_ERR);

	// TODO: Reject pulses after a recent slope detection.
	if (bSlopeDetected && bWithinPulseRange)
	{
		// Log the start of the second pulse of the data-frame.
		iLastMinima = iCurrentTime;

		// If another minma happened BEFORE 500ms  (i.e. the amount of time between a high and a red/green)
		//  then we know that it is likely attached to the wrong pulse.
	}

	// Determine if this device has a sync pulse.
	boolean bActive = iDeltaTime < ( DATA_WIDTH + DATA_WIDTH_ERR ) ;

	// If an active sync pulse, sample the data.
	if (bActive)
	{
		// Make it relative to the start of the minima spike.
		int iRelativeDelta = iCurrentTime - iLastMinima;

		// Sample the min and max.  Min = immediately after the spike, Max = 4th pulse after the spike.
		if (checkSample(iRelativeDelta, (PULSE_WIDTH * 0) + SAMPLE_OFFSET, SAMPLE_WINDOW)) {		// Min.
			pEstimatedMin->push(iLDRValue);
			iLDR1Min = pEstimatedMin->mean();
			//ldr2.updateMin(iLDR1Min);
		}
		if (checkSample(iRelativeDelta, (PULSE_WIDTH * 4) + SAMPLE_OFFSET, SAMPLE_WINDOW)) {		// Max.
			pEstimatedMax->push(iLDRValue);
			iLDR1Max = pEstimatedMax->mean();
			//ldr2.updateMax(iLDR1Max);
		}

		// Sample the RGB pulses. R = 1st pulse after the spike, B = 3rd pulse after the spike.
		if (checkSample(iRelativeDelta, (PULSE_WIDTH * 1) + SAMPLE_OFFSET, SAMPLE_WINDOW)) {		// Red.
			r = iLDRValue;
			iTargetR = constrain(map(iLDRValue, iLDR1Min, iLDR1Max, 0, 255), 0, 255);
		}
		if (checkSample(iRelativeDelta, (PULSE_WIDTH * 2) + SAMPLE_OFFSET, SAMPLE_WINDOW)) {		// Green.
			g = iLDRValue;
			iTargetG = constrain(map(iLDRValue, iLDR1Min, iLDR1Max, 0, 255), 0, 255);
		}
		if (checkSample(iRelativeDelta, (PULSE_WIDTH * 3) + SAMPLE_OFFSET, SAMPLE_WINDOW)) {		// Blue.
			b = iLDRValue;
			iTargetB = constrain(map(iLDRValue, iLDR1Min, iLDR1Max, 0, 255), 0, 255);
		}

		// Ensure we are in a mode that displays a screen colour.
		//changeMode(SCREEN);
	}
	else
	{
		// Turn the LED blue (not on screen).
		//changeMode(NO_SCREEN);
	}

	// Debug graph. Used with SerialChart. 
	#ifdef SERIAL_SYNC_DEBUG
	Serial.print(millis()); Serial.print(","); 		// ms
	Serial.print(iTargetR); Serial.print(","); 		// Red
	Serial.print(iTargetG); Serial.print(","); 		// Green
	Serial.print(iTargetB); Serial.print("\n"); 	// Blue
	#endif
	
	#ifdef SERIAL_SYNC_DEBUG2
	// Print out the verbose serial debug as csv.
	Serial.print(millis()); Serial.print(","); 		// ms
	Serial.print(fSlope); Serial.print(","); 		// slope
	Serial.print(iLDR1Max); Serial.print(","); 		// Estimated maximum (white + backlight).
	Serial.print(r); Serial.print(","); 			// Red (estimated)
	Serial.print(g); Serial.print(","); 			// Green (estimated) 
	Serial.print(b); Serial.print(","); 			// Blue (estimated)
	Serial.print(iLDR1Min); Serial.print(","); 		// Estimated minimum (black + backlight).
	Serial.println(iLDRValue);						// ldr raw
	#endif

	// Return if we are active or not.
	return bActive ? PULSE_STATE__ACTIVE_PULSE : PULSE_STATE__NO_PULSE;
}

/**
 * @brief Handle the clip behaviour for sync pulse mode.
 * This will wait for a pulse, and then display colour and height changes
 *   as appropriate.
 */
void loopSyncPulseMode() {
	
	// Sync with the RGB pulse.
	int8_t ePulseState = checkPulse();
	
	// If we have a valid sync pulse, but were previously off screen...
	if (ePulseState == PULSE_STATE__ACTIVE_PULSE && bOnScreen == false)
	{
		// Update flags.
		bOnScreen = true;
		
		// Normal LED indicating it has a pulse.
		digitalWrite(PIN_LED, HIGH);
		
		// Interpolate the colours and update the RGB LED to match.
		eRGBMode = RGBMODE_SCREEN;
	}
	
	// Otherwise if we are not on the screen, but previously were...
	else if (ePulseState == PULSE_STATE__NO_PULSE && bOnScreen == true)
	{			
		// Update flags.
		bOnScreen = false;
		
		// Normal LED indicating it has no pulse.
		digitalWrite(PIN_LED, LOW);
		
		// No pulse - reset max and min for LDR2.
		ldr2.resetLimits();
	}
	
	
	// If our last known state was on the screen, do all the sampling.
	if (bOnScreen)
	{
		// Update the target height.
		uint16_t analogValue = ldr2.sample();
		
		// If we have enough range (i.e. our min max have not just been reset).
		if (ldr2.limitsInRange(LDR_MIN_LIMIT, LDR_MAX_LIMIT))
		{
			// Move the motor in the direction of the target position.
			iTargetPos = ldr2.mapMinMax(analogValue, 0, MOTOR_TRAVEL);
			moveMotor();
		}
		else
		{
			#ifdef SYNCPULSE_DEBUG
			Serial.print("[SP] LDR2 limits outside expected range.");
			Serial.println(analogValue);
			#endif
		}
	}
	else
	{
		// Reset LDR2 min max.
		ldr2.resetLimits();
	}
	
	static unsigned long tmrSample = 0;
	if (cycleCheck(&tmrSample, 10U))
	{
		#ifdef SYNCPULSE_DEBUG // csv: on screen, min, max, value
		Serial.print(bOnScreen ? 100 : 0); Serial.print(",");	
		Serial.print(ldr2.getMin()); Serial.print(",");
		Serial.print(ldr2.getMax()); Serial.print(",");
		Serial.print(ldr2.sample()); Serial.print("\n");
		#endif
	}
	
	//  Update the colour of the RGB LED.
	updateColour();
	
	// Read from the force sensor and serial stream all the outputs every 5ms.
	#ifdef SERIAL_INFO
	static unsigned long tmrSerial = 0;
	if (cycleCheck(&tmrSerial, 500U))
	{
		// Write out values.
		Serial.print(millis()); Serial.print(","); 									// ms
		Serial.print(analogRead(PIN_LDR1)); Serial.print(","); 						// ldr1
		Serial.print(ldr2.sample()); Serial.print(","); 							// ldr2
		Serial.print(RGB(iTargetR, iTargetG, iTargetB), HEX); Serial.print(" "); 	// RGB
		Serial.print(pRGB.getPixelColor(0), HEX); Serial.print(","); 				// RGB

		Serial.print(iMotorPosition); Serial.print(","); 							// current height
		//Serial.print(analogRead(PIN_FORCE)); Serial.print(","); 					// force
		Serial.print(iTargetPos); Serial.print(","); 								// target height
		Serial.print(bOnScreen ? "Y" : "N"); Serial.print(","); 					// has sync pulse
		Serial.print(digitalRead(PIN_SWBOT) == 0 ? "Y" : "N"); Serial.print("\n");	// swbot
	}
	#endif
}

/**
 * @brief Handle the clip behaviour for pure height mode.
 * This will read the state of the LDRs and use this to infer height.
 * As it sees a wider range of data (i.e. different brightness levels) it will
 *   become more accurate.
 */
void loopHeightMode() {
	
	// Determine historic variables for height sampling.
	static uint16_t min = 99999;
	static uint16_t max = 0;
	static unsigned long iOnScreenTime = 0;
	
	// Run at 10hz.
	static unsigned long tmrSample = 0;
	if (cycleCheck(&tmrSample, 10U))
	{
		// Push new data.
		int iSample1 = ldr1.sample();
		int iSample2 = ldr2.sample();
		
		// Compute the difference between the LDRs.
		uint16_t iDelta = abs(iSample1 - iSample2);
		
		// Compute the mean.
		uint16_t iMean  = (iSample1 + iSample2) * 0.5;
		
		// Start this frame by assuming we are not on the screen.
		bOnScreen = false;
		
		// If the LDRs are sensing the same thing, we are probably on the screen.
		boolean bDelta_Suggest = iDelta <= LDR_OFFSCREEN_DELTA_GAP;
		if (bDelta_Suggest)
		{
			// Update the ranges.
			if (iMean > max) max = iMean;
			if (iMean < min) min = iMean;
			uint16_t iExtentDelta = abs(max - min);
			
			// If the ranges are sensible.
			boolean bRange_Suggest = (iExtentDelta > LDR_MIN_LIMIT) && (iExtentDelta < LDR_MAX_LIMIT );
			if (bRange_Suggest)
			{
				// Update the time it thinks it was placed on a screen.
				if (iOnScreenTime == 0)
					iOnScreenTime = millis();
				
				// If it has been on the screen for over 1s.
				boolean bTime_Suggest  = (millis() - iOnScreenTime) > ON_SCREEN_TIMEIN;
				if (bTime_Suggest)
				{
					// Set the flag and update the motor height.
					bOnScreen = true;
					iTargetPos = constrain(map(iMean, min, max, 0, MOTOR_TRAVEL), 0, MOTOR_TRAVEL);
				}
				else
				{
					#ifdef HEIGHT_DEBUG
					Serial.println("[HM] not enough time on screen");
					#endif
				}
			}
			
			// If they are not, reset the screen time check so we need to wait another second.
			else
			{
				iOnScreenTime = 0;
				#ifdef HEIGHT_DEBUG
				Serial.print("[HM] too small range between min and max");
				Serial.println(abs(max - min));
				#endif
			}
		}
		
		// If the LDRs are sensing different things so we are not on the screen.
		else
		{
			// Reset min and maxes.
			min = 99999;
			max = 0;
			
			// And reset the screen time.
			iOnScreenTime = 0;
			#ifdef HEIGHT_DEBUG
			Serial.print("[HM] too small delta between LDRs");
			Serial.println(iDelta);
			#endif
		}
		
		#ifdef HEIGHT_DEBUG // csv: on screen, min, max, value
		Serial.print(bOnScreen ? 100 : 0); Serial.print(",");	
		Serial.print(min); Serial.print(",");
		Serial.print(max); Serial.print(",");
		Serial.print(iMean); Serial.print("\n");
		#endif
	}
	
	// If we are on the screen, update the colour and move the motor.
	if (bOnScreen)
	{
		moveMotor();
		updateColour();
	}
}



/**
 * @brief Handle the clip behaviour for screen serial mode.
 * This will wait for a pulse, and then display colour and height changes
 * as appropriate.
 */
void loopSerialMode() {
	
	
	// Move the motor and update the colour.
	moveMotor();
	updateColour();
	
	// Static variables for the serial mode.
	static bool flippedRead = false;						// Are we reading in "inverted" LDR mode (ie. LDR1 <> LDR2).
	static unsigned int bitsSeen = 0;						// The number of valid bits read.
	static unsigned char cmdBuffer[3] = { 0, 0, 0 };		// The command buffer.
	static int oldState = 0;								// The last 'state' symbol read by the LDR: SYMB_ZERO, SYMB_LOW, SYMB_HIGH
	#ifdef SSMODE_8BIT
	static uint16_t bSerialBuffer = 0x00;					// The buffer to write serial commands into. 16 bit (we need space for frame headers, parity, etc).
	#endif
	#ifdef SSMODE_16BIT
	static uint32_t bSerialBuffer = 0x00;					// The buffer to write serial commands into. 32 bit (we need space for frame headers, parity, etc).
	#endif
	static const unsigned char BitsSetTable256[256] = 		// A bit table to help quickly compute the parity of the data recieved.
	{
		#define B2(n) n,     n+1,     n+1,     n+2
		#define B4(n) B2(n), B2(n+1), B2(n+1), B2(n+2)
		#define B6(n) B4(n), B4(n+1), B4(n+1), B4(n+2)
		B6(0), B6(1), B6(1), B6(2)
	};
	
	// Different symbols.
	#define SYMB_LOW 0
	#define SYMB_ZERO 1
	#define SYMB_HIGH 2
	
	// Read the data from the LDRs into an array.
	int iLDRValue[] = { analogRead( PIN_LDR1 ), analogRead( PIN_LDR2 ) };
	int iInVal = iLDRValue[0] - iLDRValue[1];
	
	// Cacluate the midpoint.
	int m = ((iLDRValue[1] - iLDRValue[0]) / 2.0) +  iLDRValue[0];
	
	int state = SYMB_ZERO;
	int thresh = 100;
	if      (iLDRValue[0] < (m-thresh) && iLDRValue[1] > (m+thresh)) state = SYMB_LOW;
	else if (iLDRValue[1] < (m-thresh) && iLDRValue[0] > (m+thresh)) state = SYMB_HIGH;
	
	
	/* Create the state transition graph.
	static unsigned long tmr1 = 0;
	if (cycleCheck(&tmr1, 10U))
	{
		Serial.print(m); Serial.print(",");
		Serial.print(m-100); Serial.print(",");
		Serial.print(m+100); Serial.print(",");
		Serial.print(iLDRValue[0]); Serial.print(",");
		Serial.print(iLDRValue[1]); Serial.print(",");
		Serial.print((state * 100) - 100); Serial.print("\n");
	}
	*/
	
	/*
	// If we are possibly inverted, invert the values!
	if( flippedRead )
		iInVal = iLDRValue[1] - iLDRValue[0];
	
	// If we have not seen any valid bits in a while, invert it to check.
	if( bitsSeen > 80 )
	{
		#if (defined SSMODE_CMDPRINT || defined SSMODE_STREAMPRINT)
		Serial.println( "[SS] Inverted read mode!" );
		#endif
		flippedRead = !flippedRead;
		bitsSeen = 0;
	}
	*/
	
	// Convert the LDR value into a symbol.
	//int state = constrain( map( iInVal, -450, 500, 0, 4 ), 0, 4 );
	
	// If there has been a change in state.
	if( state != oldState )
	{
		// Increment the number of bits seen.
		bitsSeen++;
		
		// Overwrite the old state with the new one.
		oldState = state;
		
		// Handle it.
		switch( state )
		{
			// Low, so insert a 1 into the buffer.
			case SYMB_LOW:
				bSerialBuffer = (bSerialBuffer << 1);
				//bSerialBuffer = (bSerialBuffer << 1) & ~1;
				#if (defined SSMODE_CMDPRINT || defined SSMODE_STREAMPRINT)
				Serial.print( "0" );
				#endif
				break;
			
			// Returned to zero, do nothing.
			case SYMB_ZERO:
				//inputIndex++;
				#if (defined SSMODE_CMDPRINT || defined SSMODE_STREAMPRINT)
				Serial.print( "" );
				#endif
				break;
			
			// High, so insert a 0 into the buffer.
			case SYMB_HIGH:
				bSerialBuffer = (bSerialBuffer << 1) | 1;
				#if (defined SSMODE_CMDPRINT || defined SSMODE_STREAMPRINT)
				Serial.print( "1" );
				#endif
				break;
		}
		
		// Unpack the bit values from a frame.  Based on RS232.
		// |----------------|---------------|--------|--------|--------|--------|--------|--------|--------|--------|--------|-------\
		// | START FRAME 1  |  START BIT 0  |  DATA  |  DATA  |  DATA  |  DATA  |  DATA  |  DATA  |  DATA  |  DATA  | PARITY |  START FRAME 2
		// |----------------|---------------|--------|--------|--------|--------|--------|--------|--------|--------|--------|---------\
		
		// 8-bit mode
		#ifdef SSMODE_8BIT
		// 0 1 0 1111 1111 1 0
		byte eFrame    = (byte)((bSerialBuffer >> 0) & 0x01);	// Should = 0
		byte parity    = (byte)((bSerialBuffer >> 1) & 0x01);	// Should = 0 | 1
		byte frame     = (byte)((bSerialBuffer >> 2) & 0xFF);	// Payload
		byte startBit  = (byte)((bSerialBuffer >> 10) & 0x01);	// Should = 0
		byte sFrame    = (byte)((bSerialBuffer >> 11) & 0x01);	// Should = 0
		byte sFrame2    = (byte)((bSerialBuffer >> 12) & 0x01);	// Should = 1 = eFrame (they are the same bit)
		#endif
		
		// 16-bit mode
		/*
		#ifdef SSMODE_16BIT
		byte eFrame     = (byte)(bSerialBuffer & 0x01);
		byte parity     = (bSerialBuffer >> 1) & 0x01;
		uint16_t frame  = (byte)((bSerialBuffer >> 2) & 0xFFFF);
		byte startBit   = (bSerialBuffer >> 18) & 0x01;
		byte sFrame     = (bSerialBuffer >> 19) & 0x01;
		#endif
		*/
		
		// Check for valid frame... (see above).
		byte f2 = frame;
		bool bFrameParity = !parity_even_bit(frame);
		bool bParityCheck = (bFrameParity == 0 && parity == 0) || (bFrameParity == 1 && parity == 1);
		
		//bool bParityCheck = (BitsSetTable256[frame] % 2 == 0 && parity == 1) || (BitsSetTable256[frame] % 2 != 0 && parity == 0);
		if (eFrame == 0 && startBit == 0 && sFrame == 1 && sFrame2 == 0 && bParityCheck)// && bParityCheck)
		{
			Serial.println(" <-- valid frame ");
			bSerialBuffer = 0;
			
			Serial.print("  [");
			Serial.print(f2, BIN);
			Serial.print("]  ");
			Serial.println( f2, HEX );
			/*
			Serial.print( "\t" );
			
			Serial.print( "[" ); Serial.print( sFrame );
			Serial.print( " " ); Serial.print( startBit );
			
			Serial.print( "  " ); Serial.print( frame, BIN ); Serial.print( " " );
			
			Serial.print( " " ); Serial.print( parity );
			Serial.print( " " ); Serial.print( eFrame );
			Serial.print( "]\n" );
			*/
		
		}
		
		/*
		// If it looks like a valid frame... (see above).
		//if( sFrame == 0 && eFrame == 0 && startBit == 1 )
		if( sFrame == 0 && startBit == 1  &&  stop2 == 1 && stop1 == 0)
		{
			// If the parity of the data in the frame matches the parity bit.
			#ifdef SSMODE_8BIT
			if( (BitsSetTable256[frame] % 2 == 0 && parity == 1) || (BitsSetTable256[frame] % 2 != 0 && parity == 0) )
			#endif
			#ifdef SSMODE_16BIT
			int bits = BitsSetTable256[(frame & 0xFF)] + BitsSetTable256[((frame >> 8) & 0xFF)];
			if( (bits % 2 == 0 && parity == 1) || (bits % 2 != 0 && parity == 0) )
			#endif
			{
				// Reset the number of bits seen.
				bitsSeen = 0;
				
				#ifdef SSMODE_STREAMPRINT
				// Print out the frame we think is valid.
				Serial.print( "[" );
				Serial.print( frame, BIN );
				Serial.print( " " );
				Serial.print( frame, HEX );
				Serial.print( " " );
				Serial.print( (char)frame );
				Serial.print( "]\t" );
				Serial.println( bSerialBuffer, BIN );
				#endif
				
				// Fill the command buffer.
				cmdBuffer[0] = cmdBuffer[1];
				cmdBuffer[1] = cmdBuffer[2];
				cmdBuffer[2] = (unsigned char)(frame & 0xFF);
				
				#ifdef SSMODE_CMDPRINT
				// Print out the command buffer.
				Serial.print( "CMD: " );
				Serial.print( cmdBuffer[0], HEX );
				Serial.print( " " );
				Serial.print( cmdBuffer[1], HEX );
				Serial.print( " " );
				Serial.print( cmdBuffer[2], HEX );
				Serial.print( " | " );
				Serial.print( (unsigned char)cmdBuffer[0] );
				Serial.print( " " );
				Serial.print( (unsigned char)cmdBuffer[1] );
				Serial.print( " " );
				Serial.println( (unsigned char)cmdBuffer[2] );
				#endif
				
				// If we have an 'X' at the end, the command is valid.
				if( cmdBuffer[2] == 'X' )
				{
					switch( cmdBuffer[0] )
					{
						case 'R': iTargetR = cmdBuffer[1]; break;
						case 'G': iTargetG = cmdBuffer[1]; break;
						case 'B': iTargetB = cmdBuffer[1]; break;
						case 'H': iTargetPos = map( (unsigned)cmdBuffer[1], 0, 0xFF, 0, MOTOR_TRAVEL ); break;
					}
				}
				
				// Wipe the serial buffer ready for the next frame.
				bSerialBuffer = 0;
			}
		}
		else
		{
			Serial.print( bSerialBuffer, BIN );
			Serial.print( "\t" );
			
			Serial.print( "[" ); Serial.print( sFrame );
			Serial.print( " " ); Serial.print( startBit );
			
			Serial.print( " " ); Serial.print( frame, BIN ); Serial.print( " " );
			
			Serial.print( " " ); Serial.print( parity );
			Serial.print( " " ); Serial.print( stop2 );
			Serial.print( " " ); Serial.print( stop1 );
			Serial.print( "]\n" );
				
		}
		*/
	}
	

}

/**
 * Logic loop.
 * The function performs the following tasks:
 *   - 
 */
void loop() {
	
	// Listen for a change in mode.
	detectModeChange( 0 );
	
	// Detect which mode we are in.
	switch (eClipMode)
	{
		// If we are in EEMODE_HEIGHTONLY mode, just use both LDRs for height.
		case EEMODE_HEIGHTONLY:
			loopHeightMode();
			break;
			
		// If we are in EEMODE_SYNCPULSE mode, use the height and colour.
		case EEMODE_SYNCPULSE:
			loopSyncPulseMode();
			break;
			
		// If we are in EEMODE_SERIAL, use the simple binary protocol.
		case EEMODE_SERIAL:
			loopSerialMode();
			break;
			
		// Unknown mode.. should not get here.
		default:
			break;
	}
	
	// If we are in profiling mode.
	#ifdef MODE_PROFILING
	
	static bool bMeasuring = false;			// Are we measuring.
	static unsigned int iMeasureCount = 0;	// How many have we measured.
	
	// Process new commands.
	if (Serial.available())
	{
		byte data = Serial.read();
		switch (data)
		{
			case 'p':
				Serial.println("pong");
				break;
			case 's':
				Serial.print("start "); 
				Serial.println(iMeasureCount);
				bMeasuring = true;
				break;
			case 'f':
				Serial.print("stop ");
				Serial.println(iMeasureCount);
				bMeasuring = false;
				iMeasureCount ++;
				break;
		}
	}
	
	// Print out if measuring.
	if (bMeasuring)
	{
		// Rate limit the output stream.
		static unsigned long tmrSample = 0;
		if (cycleCheck(&tmrSample, PROFILING_RATE))
		{
			Serial.print(iMotorPosition); Serial.print(","); 	// Current MOTOR HEIGHT.
			Serial.print(iTargetPos); Serial.print(","); 		// Target MOTOR HEIGHT.
			Serial.print(iTargetR); Serial.print(","); 			// Target RED
			Serial.print(iTargetG); Serial.print(","); 			// Target GREEN
			Serial.print(iTargetB); Serial.print("\n"); 		// Target BLUE.
			
			//#define HRGB(h,r,g,b) 	(((h & 0xFF) << 24) | ((r & 0xFF) << 16) | ((g & 0xFF) << 8)  | ((b & 0xFF) << 0))
			//Serial.println(HRGB(iTargetPos, iTargetR, iTargetG, iTargetB), HEX); // Target HRGB.
		}
	}
	#endif
	
}









