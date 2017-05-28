/**
 * This file is part of RobotArm.
 *
 * RobotArm is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * RobotArm is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with RobotArm.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2017 Jérôme Labidurie <jerome@labidurie.fr>
 *
 * Fablab lannion http://fablab-lannion.org
 *
 */

#include <Servo.h>


// These constants won't change.  They're used to give names
#define PIN_LED 13

// number of axes, ie: servos
#define  NB_AXES 2
uint8_t  axes_pins[NB_AXES] = {9, 10};  /**< hw pins for each axe */
Servo    myservo[NB_AXES];              /**< array of servos object to control axes */
int16_t  axes_vals[NB_AXES] = {90, 90}; /**< array of servos current angles */
// number of joysticks axes to control movements
#define  NB_JOY 2
uint8_t  joy_pins[NB_JOY] = {A0, A1};   /**< pins of analogic inputs from joysticks */
uint16_t joy_bases[NB_JOY] = {0, 0};    /**< array for values of joysticks when they are in resting positions. ie: no touch, init in setup function */

// value of dead zone around resting position
#define HYSTERISIS 1

/** get sevo movement from joystick axis value
 * @param[in] joy joystick axis raw reading
 * @param[in] base joysick base (resting) value
 * @return increment (>0 or <0) value
 */
int16_t incServo (uint16_t joy, uint16_t base) {
	int16_t ret = 0;
	int16_t absVal = 0;

	if ( (joy > base-HYSTERISIS) && (joy < base+HYSTERISIS) ) {
		// no move, dead zone
		return ret;
	}
	absVal = base - joy ;
	absVal = abs ( absVal );
	// TODO: use non-linear function ?
	ret = map (absVal, 0, 500, 0, 15);

	return (joy < base)? ret : -ret ;
} // incServo

void dbgData (uint8_t servo, uint16_t val, int16_t angle, int inc) {
  // print the results to the serial monitor:
  Serial.print("servo(");
  Serial.print(servo);
  Serial.print(") val=");
  Serial.print(val);
  Serial.print(" ang=");
  Serial.print(angle);
  Serial.print(" inc=");
  Serial.println(inc);
}

/** get new servo angle from increment
 * @param[in] angle the current angle of the servo
 * @param inc increment to add/sub
 * @return the new angle to apply
 */
uint8_t getAngle (uint8_t angle, int inc) {
	int ret = angle;
	ret += inc;
	if (ret < 0) {
		ret = 0;
	}
	if (ret > 180) {
		ret = 180;
	}
	return ret;
}

void setup (void) {
	// setup hw
	pinMode (PIN_LED, OUTPUT);
	for (uint8_t a=0;a<NB_AXES;a++) {
		myservo[a].attach(axes_pins[a]);
	}

	// initialize serial communications at 9600 bps:
	Serial.begin(9600);

	// init of joy_bases[], servo should not be touched when arduino is starting
	Serial.println("Calibrating servos...");
	digitalWrite (PIN_LED, HIGH);
	for (uint8_t j=0; j<NB_JOY; j++) {
		for (uint8_t i=0;i<10;i++) {
			joy_bases[j] += analogRead(joy_pins[j]);
			delay(100);
		}
		joy_bases[j] /= 10;
		Serial.print ("Joy("); Serial.print(j); Serial.print ("):");
		Serial.println (joy_bases[j]);
	}
	digitalWrite (PIN_LED, LOW);
} // setup

void loop (void) {
	int inc;              /**< moving value for the servo */
	int sensorValue = 0;  /**< value read from the joy axis */

	// for each joystick axes
	for (uint8_t j=0; j<NB_JOY; j++) {
		// note: this code implies : one joy axis <=> on servo
		// read the analogin value from joystick
		sensorValue = analogRead( joy_pins[j] );
		// get servo movement
		inc = incServo (sensorValue, joy_bases[j] );
		axes_vals[j] = getAngle (axes_vals[j], inc);
		// move
		myservo[j].write(axes_vals[j]);
		if (inc != 0) { dbgData (j, sensorValue, axes_vals[j], inc); }
	}

	delay(10);
} // loop
