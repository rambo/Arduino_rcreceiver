// Get this from https://github.com/pololu/Dual-VNH5019-Motor-Shield
#include "DualVNH5019MotorShield.h"
DualVNH5019MotorShield md;
// Get this from https://github.com/rambo/PinChangeInt_userData
#include "PinChangeInt_userData.h"

#define SERVO_MIDDLE 1500

// To hold data for reach RC input pin.
typedef struct {
    const uint8_t pin;
    volatile unsigned long start_micros;
    volatile unsigned long stop_micros;
    volatile boolean new_data;
    uint16_t servo_position; // microseconds
} RCInput;

// Initialize the inputs to an array
RCInput inputs[] = {
    { A4, 0, 0, false, SERVO_MIDDLE }, // Forward/backward
    { A5, 0, 0, false, SERVO_MIDDLE }, // Left/right
};
const uint8_t inputs_len = sizeof(inputs) / sizeof(RCInput);

// Speeds for the left & right motors
int16_t left_speed = 0;
int16_t right_speed = 0;

// Called whenever a control pulse ends
void rc_pulse_low(void* inptr)
{
    RCInput* input = (RCInput*)inptr;
    input->stop_micros = micros();
    input->new_data = true;
}

// Called whenever a control pulse starts
void rc_pulse_high(void* inptr)
{
    RCInput* input = (RCInput*)inptr;
    input->new_data = false;
    input->start_micros = micros();
}

// Calculates the servo position, called from the mainloop whenever there is new data
void calc_servo_position(void* inptr)
{
    RCInput* input = (RCInput*)inptr;
    input->servo_position = (uint16_t)(input->stop_micros - input->start_micros);
    input->new_data = false;
}

int16_t clip_md_speed(int16_t spd)
{
    if (spd > 400)
    {
        spd = 400;
    }
    if (spd < -400)
    {
        spd = -400;
    }
    return spd;
}

int16_t map_rc_to_speed(uint16_t servo_position)
{
    return clip_md_speed(servo_position - SERVO_MIDDLE);
}

void stopIfFault()
{
  if (md.getM1Fault())
  {
    Serial.println(F("M1 fault"));
    md.setSpeeds(0,0);
    while(1);
  }
  if (md.getM2Fault())
  {
    Serial.println(F("M2 fault"));
    md.setSpeeds(0,0);
    while(1);
  }
}


void setup()
{
    Serial.begin(115200);
    Serial.println(F("RC controlled dual VNH5019 Motor Shield"));
    md.init();
    
    // Attach pin change interrupts for the RCInputs
    for (uint8_t i=0; i < inputs_len; i++)
    {
        PCintPort::attachInterrupt(inputs[i].pin, &rc_pulse_high, RISING, &inputs[i]);
        PCintPort::attachInterrupt(inputs[i].pin, &rc_pulse_low, FALLING, &inputs[i]);
    }
    Serial.println(F("Booted"));
}

volatile unsigned long last_report_time = millis();
int16_t lrspeed = 0;
int16_t fbspeed = 0;
void loop()
{
    // Stop and hang if motor driver has fault
    stopIfFault();

    // Check inputs for new data
    for (uint8_t i=0; i < inputs_len; i++)
    {
        if (inputs[i].new_data)
        {
            calc_servo_position(&inputs[i]);
        }
    }

    lrspeed = map_rc_to_speed(inputs[1].servo_position);
    if (lrspeed != 0)
    {
        left_speed = lrspeed;
        right_speed = -1 * lrspeed;
    }
    else
    {
        left_speed = 0;
        right_speed = 0;
    }
    fbspeed = map_rc_to_speed(inputs[0].servo_position);
    left_speed += fbspeed;
    right_speed += fbspeed;


    // Set speeds
    left_speed = clip_md_speed(left_speed);
    right_speed = clip_md_speed(right_speed);
    md.setSpeeds(left_speed, right_speed);

    // Report positions every second
    if (millis() - last_report_time > 200)
    {
        for (uint8_t i=0; i < inputs_len; i++)
        {
            Serial.print(F("Channel "));
            Serial.print(i, DEC);
            Serial.print(F(" position "));
            Serial.print(inputs[i].servo_position, DEC);
            Serial.print(F(" speed "));
            Serial.println(map_rc_to_speed(inputs[i].servo_position), DEC);
        }
        Serial.print(F("Left motor speed "));
        Serial.println(left_speed, DEC);
        Serial.print(F("Right motor speed "));
        Serial.println(right_speed, DEC);
        last_report_time = millis();
    }
}


