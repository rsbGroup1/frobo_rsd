#include <Stepper.h>
const int stepsPerRevolution = 200; // Steps per revolution
int en_a = 12; //Enable for A part of H bridge
int en_b = 13; //Enable for B part of H bridge
int supply_tipped = 4; //2nd leg for the tipped sensor
int supply_level = 5; //2nd leg for the level sensor
int is_tipped = 6 ;//sensor pin when tipped
int is_level = 7 ;// sensor pin when normal
int ramp = 67;
int ning = 11;
Stepper stepmotor(stepsPerRevolution, 8, 9, 10, 11);  //Stepper(int number_of_steps, int motor_pin_1, int motor_pin_2, int motor_pin_3, int motor_pin_4, int use_half_step);


void setup()
{

	Serial.begin(115200);
	pinMode(is_tipped, INPUT_PULLUP);  //Pullups required to avoid floating values
	pinMode(is_level, INPUT_PULLUP);
	pinMode(supply_tipped, OUTPUT);
	pinMode(supply_level, OUTPUT);
	pinMode(en_a, OUTPUT);  
	pinMode(en_b, OUTPUT);
	pinMode(8, OUTPUT);
	pinMode(9, OUTPUT);
	pinMode(10, OUTPUT);
	pinMode(11, OUTPUT);
	digitalWrite(supply_tipped, LOW);
	digitalWrite(supply_level, LOW);
}

void loop() {
	// read the sensor:
	if (Serial.available() > 0) 
	{
		int readByte = Serial.read();

		switch (readByte) 
		{
		case 'u':
			tip_load();
			break;
		case 'd':
			return_to_level();
			break;
		case 's':
			digitalWrite(en_a, LOW);
			digitalWrite(en_b, LOW);
		default:
			digitalWrite(en_a, LOW);
			digitalWrite(en_b, LOW);
		}
	} 
}

  
void tip_load()
{
	while (digitalRead(is_tipped) != LOW )
	{
		digitalWrite(en_a, HIGH);
		digitalWrite(en_b, HIGH);
		//stepmotor.setSpeed(80);
		//stepmotor.step(stepsPerRevolution);
		// x=x+10;
		// half_step(8,9,10,11);
		half_step(8,9,10,11);

		// Serial.println("Tipping"); 
		//Serial.println(x);
		digitalWrite(en_a, LOW);
		digitalWrite(en_b, LOW);  
	}
	Serial.println("d");
}
   
void return_to_level()
{
	while(digitalRead(is_level) != LOW)
	{
		digitalWrite(en_a, HIGH);
		digitalWrite(en_b, HIGH);
		//stepmotor.setSpeed(60);
		full_step(10,11,8,9);
		full_step(10,11,8,9);
		//stepmotor.step(-stepsPerRevolution);
		//   Serial.println("Reversing");
		digitalWrite(en_a, LOW);
		digitalWrite(en_b, LOW);  
		return;
	} 

	Serial.println("d");
	return;
}
   
void half_step(int pin1, int pin2, int pin3, int pin4)
{
	int timer_delay=3;

	digitalWrite(pin1, HIGH);
	digitalWrite(pin2, LOW);  //1
	digitalWrite(pin3, HIGH);
	digitalWrite(pin4, LOW);
	delay(timer_delay);
	//Serial.println(ramp);
	digitalWrite(pin1, LOW);
	digitalWrite(pin2, LOW);
	digitalWrite(pin3, HIGH);  //2
	digitalWrite(pin4, LOW);
	delay(timer_delay);
	// Serial.println(ramp);
	digitalWrite(pin1, LOW);
	digitalWrite(pin2, HIGH);  //3
	digitalWrite(pin3, HIGH);
	digitalWrite(pin4, LOW);
	delay(timer_delay);
	//Serial.println(ramp);
	digitalWrite(pin1, LOW);
	digitalWrite(pin2, HIGH);   //4
	digitalWrite(pin3, LOW);
	digitalWrite(pin4, LOW);
	delay(timer_delay);
	// Serial.println(ramp);
	digitalWrite(pin1, LOW);
	digitalWrite(pin2, HIGH);   //5
	digitalWrite(pin3, LOW);
	digitalWrite(pin4, HIGH);
	delay(timer_delay);
	// Serial.println(ramp);
	digitalWrite(pin1, LOW);
	digitalWrite(pin2, LOW);  //6
	digitalWrite(pin3, LOW);
	digitalWrite(pin4, HIGH);
	delay(timer_delay);
	// Serial.println(ramp);
	digitalWrite(pin1, HIGH);
	digitalWrite(pin2, LOW);   //7
	digitalWrite(pin3, LOW);
	digitalWrite(pin4, HIGH);
	delay(timer_delay);
	//Serial.println(ramp);
	digitalWrite(pin1, HIGH);
	digitalWrite(pin2, LOW);
	digitalWrite(pin3, LOW);
	digitalWrite(pin4, LOW);
	delay(timer_delay);
	// Serial.println(ramp);
	// if (ramp>0) {
	// ramp= ramp-ning;
	//if (ning>0){   
	// ning--;
	//}
	//}
	return;
}
     

void full_step(int pin1, int pin2, int pin3, int pin4) 
{
	int timer_delay=3;
	digitalWrite(pin1, HIGH);
	digitalWrite(pin2, LOW);
	digitalWrite(pin3, HIGH);
	digitalWrite(pin4, LOW);
	delay(timer_delay);
	Serial.println(ramp);

	digitalWrite(pin1, LOW);
	digitalWrite(pin2, HIGH);
	digitalWrite(pin3, HIGH);
	digitalWrite(pin4, LOW);
	delay(timer_delay+ramp);
	Serial.println(ramp);

	digitalWrite(pin1, LOW);
	digitalWrite(pin2, HIGH);
	digitalWrite(pin3, LOW);
	digitalWrite(pin4, HIGH);
	delay(timer_delay+ramp);
	Serial.println(ramp);

	digitalWrite(pin1, HIGH);
	digitalWrite(pin2, LOW);
	digitalWrite(pin3, LOW);
	digitalWrite(pin4, HIGH);
	delay(timer_delay+ramp);
	Serial.println(ramp);

	if (ramp>0) 
	{
		ramp= ramp-ning;
	}

	if (ning>0)
	{   
		ning--;
	}
	return;
}

void reformed_tip(int pin1, int pin2, int pin3, int pin4){
	// half_step
	// full_step
	//half_step
}
