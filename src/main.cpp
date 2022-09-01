#include <Arduino.h>
#include <stdint.h>
/*
CBRR - Firmware to control a Railroad inside a Cigar Box.

There are two tracks, which travel in opposite direction.
The trains can pass each other at the station.

Control Panel:
- Track selector button: Toggles between track 1 and 2.
- Mode select button: Toggles between manual and automatic mode.
- Speed knob: Manually controls the train speed. In automatic mode it controls the maximum train speed.

In Automatic mode a train will automatically start to drive for several laps.
The train slows down when it passes the station to continue for the next round.
On the last round the train will slow down and come to a halt at the station.

After some delay the other track will depart in the opposite direction.

This will continue until the 'Mode' button is pressed. When this happens, any running train will finish its round and stop at the station.

*/

#if 0 // Arduino Pin Mapping
// Arduino Pro Micro 32u8

//#define LED_BUILTIN_RX 17
//#define LED_BUILTIN_TX 30
//-----------------------
// analog_pin_to_channel
//-----------------------
// A0             PF7               ADC7
// A1             PF6               ADC6    
// A2             PF5               ADC5    
// A3             PF4               ADC4
// A4             PF1 (Not Used)    ADC1    
// A5             PF0 (Not Used)    ADC0    
// A6     D4      PD4               ADC8
// A7     D6      PD7               ADC10
// A8     D8      PB4               ADC11
// A9     D9      PB5               ADC12
// A10    D10     PB6               ADC13
// A11    D12     PD6 (Not Used)    ADC9

//-----------------------
// digital_pin_to_port
//-----------------------
// D0 - PD2(RXI)
// D1 - PD3(TXO)
// D2 - PD1
// D3 - PD0
// D4 - PD4
// D5 - PC6
// D6 - PD7
// D7 - PE6
//
// D8 - PB4
// D9 - PB5
// D10 - PB6
// D11 - PB7 (Not Used)
// D12 - PD6 (Not Used)
// D13 - PC7 (Not Used)
//
// D14 - MISO - PB3
// D15 - SCK - PB1
// D16 - MOSI - PB2
// D17 - SS - PB0 (RX LED)
//
// D18 - A0 - PF7
// D19 - A1 - PF6
// D20 - A2 - PF5
// D21 - A3 - PF4
// D22 - A4 - PF1 (Not Used)
// D23 - A5 - PF0 (Not Used)
//
// D24 / D4 - A6 - PD4
// D25 / D6 - A7 - PD7
// D26 / D8 - A8 - PB4
// D27 / D9 - A9 - PB5
// D28 / D10 - A10 - PB6
// D29 / D12 - A11 - PD6 (Not Used)
// D30 / TX Led - PD5 (TX LED)
#endif

#define BUTTON_TRACK 3 // 4433 PD6 pin 12
#define BUTTON_MODE 2  // 4433 PD7 pin 13
#define LED_TRACK1 9   // 4433 PD0 pin 2
#define LED_TRACK2 8   // 4433 PD1 pin 3
#define LED_MODE 7     // 4433 PD2 pin 4
#define KNOB_SPEED A10 // 4433 PC4 pin 27 ADC4

#define MOTOR_ENABLE_1 4 // 4433 PB0 pin 14
#define MOTOR_ENABLE_2 5 // 4433 PB2 pin 16
#define MOTOR_PWM 6      // 4433 PB1 pin 15
#define MOTOR_SENSE_1 A1 // 4433 PC1 pin 24 ADC1
#define MOTOR_SENSE_2 A0 // 4433 PC0 pin 23 ADC0

#define TRACK_SENSE_1 A3 // 4433 PC3 pin 26 ADC3
#define TRACK_SENSE_2 A2 // 4433 PC2 pin 25 ADC2

#define LED_ON LOW
#define LED_OFF HIGH
#define MOTOR_ENABLED LOW
#define MOTOR_DISABLED HIGH
#define BUTTON_PRESSED LOW

uint8_t _serialAvailable = false;

unsigned long last_millis = 0;

uint16_t msTimerTask = 0;  // Core task every 10mS
uint16_t msTimerRamp = 0;  // Speed ramp timer
uint16_t msTimerStop = 0;  // Stop time for train
uint16_t msTimerDrive = 0; // Drive progress estimator
uint16_t msTimerPrint = 0; // Print debug timer
uint16_t msTimerFree = 0;  // free-running timer

void updateTimers()
{
  unsigned long ms = millis();
  uint16_t diff_ms = (uint16_t)(ms - last_millis);
  last_millis = ms;

  msTimerTask += diff_ms;
  msTimerRamp += diff_ms;
  msTimerStop += diff_ms;
  msTimerDrive += diff_ms;
  msTimerPrint += diff_ms;
  msTimerFree += diff_ms;
}

#define MODE_SWITCH_TO_MANUAL 0
#define MODE_MANUAL 1
#define MODE_SWITCH_TO_AUTOMATIC 2
#define MODE_AUTOMATIC 3
uint8_t operation_mode = MODE_SWITCH_TO_MANUAL;

#define STATE_STOPPING 0
#define STATE_STOPPED 1
#define STATE_DRIVING 2
#define STATE_DRIVINGSENSE 3
uint8_t state = STATE_STOPPING;

union
{
  uint8_t allButtons;
  struct
  {
    uint8_t trackButton;
    uint8_t modeButton;
  };
} button_state = {0};

uint8_t exit_automatic = 0;
uint8_t active_motor = 1;
uint8_t rounds_to_go = 0;
uint16_t stop_time = 3000;
uint16_t max_speed = 255;
uint16_t accel_up = 0;
uint16_t accel_down = 0;
int16_t target_speed = 0;
uint8_t speedknob_value = 0;
int16_t motor_speed = 0;
int16_t motor_speed_pwm = 0;
int16_t motor_sense = 0;

// the setup function runs once when you press reset or power the board
void setup()
{
  // Disable Motors
  pinMode(MOTOR_ENABLE_1, OUTPUT);              // Open=Motor1 Enable; 1=Disable
  pinMode(MOTOR_ENABLE_2, OUTPUT);              // Open=Motor2 Enable; 1=Disable
  digitalWrite(MOTOR_ENABLE_1, MOTOR_DISABLED); // HIGH = off
  digitalWrite(MOTOR_ENABLE_2, MOTOR_DISABLED); // HIGH = off

  pinMode(MOTOR_PWM, OUTPUT);
  digitalWrite(MOTOR_ENABLE_2, HIGH); // HIGH = off/no pwm

  pinMode(MOTOR_SENSE_1, INPUT); // Analog
  pinMode(MOTOR_SENSE_2, INPUT); // Analog
  pinMode(TRACK_SENSE_1, INPUT); // Analog
  pinMode(TRACK_SENSE_2, INPUT); // Analog

  pinMode(LED_TRACK1, OUTPUT);         // LOW=on
  pinMode(LED_TRACK2, OUTPUT);         // LOW=on
  pinMode(LED_MODE, OUTPUT);           // LOW=on
  pinMode(KNOB_SPEED, INPUT);          // Analog
  pinMode(BUTTON_TRACK, INPUT_PULLUP); // LOW = pressed
  pinMode(BUTTON_MODE, INPUT_PULLUP);  // LOW = pressed
  // All leds off
  digitalWrite(LED_TRACK1, LED_OFF);
  digitalWrite(LED_TRACK2, LED_OFF);
  digitalWrite(LED_MODE, LED_OFF);

  // Prescaler on timer4 is set to 64 (488Hz PWM) by Arduino core.
  // Change prescaler from 64 into 4 (7812Hz PWM)
  // TCCR4B &= ~_BV(CS42);
  // Change prescaler from 64 into 2 (15625Hz PWM)
  TCCR4B &= ~_BV(CS42);
  TCCR4B &= ~_BV(CS40);

  // start serial port
  Serial.begin(9600);

  // Flash LEDs
  static uint8_t leds[] = {LED_TRACK2, LED_TRACK1, LED_MODE};
  for (size_t led = 0; led < sizeof(leds); led++)
  {
    digitalWrite(leds[led], LED_ON);
    delay(100);
    digitalWrite(leds[led], LED_OFF);
    delay(50);
  }

  last_millis = millis();
}

void read_speedknob_value()
{
  uint16_t speed = analogRead(KNOB_SPEED);
  // convert range 0..1023 to 0..255 with some deadband
  speed /= 3;
  speed = speed > 42 ? speed - 42 : 0;
  if (speed > 255)
    speed = 255;
  speedknob_value = (uint8_t)speed;
}

void control_motor_speed()
{
  int channel = (active_motor == 1) ? MOTOR_SENSE_1 : MOTOR_SENSE_2;
  motor_sense = (motor_sense + ((analogRead(channel) + 3) >> 2) + 1) >> 1; // Round-up before shifting

  // Control the motor by voltage (not current!) as measured over motor.
  // This will accurately compensate for back EMF.
  // Adjust motor_speed_pwm proportionally to the differnce between speed and motor_sense.
  if (motor_sense < motor_speed)
  {
    motor_speed_pwm += ((motor_speed - motor_sense) >> 1) + 1;
    if (motor_speed_pwm > 255)
      motor_speed_pwm = 255;
  }
  else if (motor_sense > motor_speed)
  {
    motor_speed_pwm -= ((motor_sense - motor_speed) >> 1) + 1;
    if (motor_speed_pwm < 0)
      motor_speed_pwm = 0;
  }

  analogWrite(MOTOR_PWM, 255 - motor_speed_pwm); // LOW = ON

  // MOTOR_ENABLE_n: Open = Motor Enable; HIGH = Disable
  if (active_motor == 1)
  {
    pinMode(MOTOR_ENABLE_1, INPUT);
    pinMode(MOTOR_ENABLE_2, OUTPUT);
    digitalWrite(MOTOR_ENABLE_2, HIGH);
  }
  else if (active_motor == 2)
  {
    pinMode(MOTOR_ENABLE_2, INPUT);
    pinMode(MOTOR_ENABLE_1, OUTPUT);
    digitalWrite(MOTOR_ENABLE_1, HIGH);
  }
}

void read_buttons()
{
  button_state.allButtons = 0;
  button_state.modeButton = (digitalRead(BUTTON_MODE) == BUTTON_PRESSED);
  button_state.trackButton = (digitalRead(BUTTON_TRACK) == BUTTON_PRESSED);
}

void show_status()
{
  // all LEDs off
  digitalWrite(LED_TRACK1, LED_OFF);
  digitalWrite(LED_TRACK2, LED_OFF);
  digitalWrite(LED_MODE, LED_OFF);

  // 'mode' LED
  if (operation_mode == MODE_AUTOMATIC)
  {
    // blink 'mode' LED when stopping (=switching to manual)
    if (exit_automatic == 0 || (msTimerFree & 0x80) == 0)
    {
      digitalWrite(LED_MODE, LED_ON);
    }
  }

  // Blink 'track' led when waiting for knob to return to zero
  // or when stopping in automatic mode
  if (operation_mode == MODE_SWITCH_TO_MANUAL ||
      operation_mode == MODE_SWITCH_TO_AUTOMATIC ||
      (operation_mode == MODE_AUTOMATIC && state == STATE_STOPPING))
  {
    if ((msTimerFree & 0x80) == 0)
      return;
  }

  // // Slow blink when idle
  // if (motor_speed == 0 && speedknob_value == 0)
  // {
  //   if ((msTimerFree & 0x300) == 0)
  //   {
  //     if ((msTimerFree & 0x3) != 0)
  //       return;
  //   }
  // }

  if (active_motor == 1)
  {
    digitalWrite(LED_TRACK1, LED_ON);
  }
  else if (active_motor == 2)
  {
    digitalWrite(LED_TRACK2, LED_ON);
  }
}

void stop_motor_and_wait_for_buttons()
{
  // Stop Motors
  target_speed = 0;
  motor_speed = 0;
  motor_speed_pwm = 0;
  control_motor_speed();

  delay(50); // debounce

  // Wait for all buttons to be released
  do
  {
    show_status();
    read_buttons();
    read_speedknob_value();
    updateTimers();
  } while (button_state.allButtons != 0);

  delay(100); // debounce
}

void switch_active_motor()
{
  active_motor = (active_motor == 2) ? 1 : 2;
}

void update_speed()
{
  if (motor_speed < target_speed)
  {
    if (msTimerRamp >= accel_up)
    {
      motor_speed++;
      msTimerRamp -= accel_up;
    }
  }
  else if (motor_speed > target_speed)
  {
    if (msTimerRamp >= accel_down)
    {
      motor_speed--;
      msTimerRamp -= accel_down;
    }
  }
  else
  {
    msTimerRamp = 0;
  }
}

//
// Wait for speed knob to return to zero
//
void handle_mode_switching()
{
  if (speedknob_value == 0)
  {
    if (operation_mode == MODE_SWITCH_TO_AUTOMATIC)
      operation_mode = MODE_AUTOMATIC;
    else
      operation_mode = MODE_MANUAL;
  }
}

//
// Manual Driving
//
void handle_mode_manual()
{
  accel_up = 7;
  accel_down = 3;
  target_speed = speedknob_value;

  // Switch only when not moving
  if (motor_speed != 0)
    return;

  // Switch to Automatic Driving
  if (button_state.modeButton)
  {
    operation_mode = MODE_SWITCH_TO_AUTOMATIC; // Automatic Driving
    stop_motor_and_wait_for_buttons();
    operation_mode = MODE_AUTOMATIC; // Automatic Driving
    state = STATE_STOPPED;
  }

  // Switch Active Motor
  if (button_state.trackButton)
  {
    switch_active_motor();
    stop_motor_and_wait_for_buttons();
    operation_mode = MODE_SWITCH_TO_MANUAL;
  }
}

//
// Automatic Driving
//
void handle_mode_automatic()
{
  // Switch to manual driving
  if (button_state.modeButton)
  {
    exit_automatic = 1;
  }

  // Act on switch to manual driving
  if (exit_automatic)
  {
    if (motor_speed == 0) // || speedknob_value == 0)
    {
      exit_automatic = 0;
      operation_mode = MODE_SWITCH_TO_MANUAL; // Manual Driving
      stop_motor_and_wait_for_buttons();
      return;
    }
  }

  // Allow track switch when stopped
  if (button_state.trackButton && motor_speed == 0)
  {
    switch_active_motor();
    stop_motor_and_wait_for_buttons();
  }

  random(); // Make truly random

  switch (state)
  {
  case STATE_STOPPING:
    target_speed = 0;
    if (motor_speed == 0)
    {
      switch_active_motor();
      state = STATE_STOPPED;
      stop_time = random(3000, 9500);
      msTimerStop = 0;
    }
    // Just in case...
    // when driving too fast, break hard
    if (motor_speed > 140)
    {
      accel_down = 2;
    }
    break;

  case STATE_STOPPED:
    target_speed = 0;
    if (msTimerStop > stop_time)
    {
      accel_up = 16;
      accel_down = 7;

      rounds_to_go = 2;
      uint8_t rnd = random(100);
      if (rnd < 20)
        rounds_to_go = 1;
      // else if (rnd > 90)
      //   rounds_to_go = 4;
      else if (rnd > 70)
        rounds_to_go = 3;

      state = STATE_DRIVING;
      max_speed = random(210, 255);
      msTimerDrive = 0;
    }
    break;

  case STATE_DRIVING:
  {
    target_speed = min(speedknob_value, max_speed);

    //
    //
    // Limit driving speed after fixed distance/seconds
    uint16_t distance = (14000 - target_speed * 32);
    if (active_motor == 1)
      distance -= 1000; // Adjust for: Motor 1 is slightly faster

    if (msTimerDrive >= distance)
    {
      msTimerDrive = distance;
      int16_t target = 140;
      if (rounds_to_go == 1 || exit_automatic) // Last round
        target = 120;

      if (target_speed > target)
      {
        target_speed = target;
      }
    }

    // Check light sensor
    uint16_t sense = analogRead(active_motor == 1 ? TRACK_SENSE_1 : TRACK_SENSE_2);
    static uint16_t last_sense_time = 0;
    if (sense > 700 && (msTimerFree - last_sense_time) > 300) // debounce light sensor by 300mS
    {
      last_sense_time = msTimerFree;
      state = STATE_DRIVINGSENSE;
    }
  }
  break;

  case STATE_DRIVINGSENSE:
    uint16_t sense = analogRead(active_motor == 1 ? TRACK_SENSE_1 : TRACK_SENSE_2);
    // Passed the sensor?
    if (sense < 400)
    {
      state = STATE_DRIVING;

      rounds_to_go--;
      if (rounds_to_go == 0 || exit_automatic)
      {
        state = STATE_STOPPING;
      }

      // Start next round
      // Aready driving, so give timer a headstart of 1500mS
      msTimerDrive = 1500;
    }
    break;
  }
}

void loop()
{
  updateTimers();
  show_status();

  if (msTimerTask >= 2)
  {
    msTimerTask -= 2;
    control_motor_speed(); // Can be outside msTimerTask

    read_speedknob_value();
    read_buttons();
    update_speed();

    switch (operation_mode)
    {
    case MODE_SWITCH_TO_MANUAL:
      handle_mode_switching(); // Switch to manual driving
      break;

    case MODE_MANUAL:
      handle_mode_manual(); // Manual Driving
      break;

    case MODE_AUTOMATIC:
      handle_mode_automatic(); // Automatic Driving
      break;
    }
  }

  if (msTimerPrint > 100)
  {
    msTimerPrint -= 100;
    _serialAvailable = Serial.availableForWrite();
    if (_serialAvailable)
    {
      //     // Serial.print("KNOB_SPEED: ");    Serial.println(analogRead(KNOB_SPEED));
      Serial.print("MOTOR_SENSE_1: ");
      Serial.println(analogRead(MOTOR_SENSE_1));
      Serial.print("MOTOR_SENSE_2: ");
      Serial.println(analogRead(MOTOR_SENSE_2));
      Serial.print("pwm: ");
      Serial.println(motor_speed_pwm);
      Serial.print("motor_speed: ");
      Serial.println(motor_speed);
      //     Serial.print("TRACK_SENSE_1: ");
      //     Serial.println(analogRead(TRACK_SENSE_1));
      //     Serial.print("TRACK_SENSE_2: ");
      //     Serial.println(analogRead(TRACK_SENSE_2));
      //     // Serial.print("BUTTON_TRACK: ");  Serial.println(digitalRead(BUTTON_TRACK) == BUTTON_PRESSED);
      //     // Serial.print("BUTTON_MODE:  ");  Serial.println(digitalRead(BUTTON_MODE) == BUTTON_PRESSED);
      //     Serial.print("mode: ");
      //     Serial.println(operation_mode);
      //     Serial.print("state: ");
      //     Serial.println(state);
      //     Serial.print("stopping: ");
      //     Serial.println(exit_automatic);
      //     Serial.print("rounds: ");
      //     Serial.println(rounds_to_go);

      //     Serial.println("---");
    }
    //   // digitalWrite(LED_BUILTIN_TX, _serialAvailable);
    //   digitalWrite(LED_BUILTIN_RX, !_serialAvailable);
  }

#if 0
  static long lastMillis = 0;
  static int loopcount = 0;

  long currentMillis = millis();
  loopcount++;
  if (currentMillis - lastMillis > 1000)
  {
    if (_serialAvailable)
    {
      Serial.print("Loops last second:");
      Serial.println(loopcount);
    }
    lastMillis = currentMillis;
    loopcount = 0;
  }
#endif
}
