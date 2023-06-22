/**
 * AUTHOR: Xingbo Wang, MASc
 * 
 * Systems Control Group
 * Department of Electrical and Computer Engineering
 * University of Toronto
 * 
 * For use with the SUGAR system
 * Code for the switch box arduino nano
 * 
 * Last Modified: 12 February 2020
 * Last Editor: Adan Moran-MacDonald (MASc 2020)
 */

// Define this to enable Serial outputs, to read the data we are
// sending to the SUGAR_box over the wire.
#define DEBUG_ENABLE 0

#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>
//#include <ABZEncoder.h>
#include <Wire.h>
#include <math.h>
#include <Event.h>
#include <Timer.h>


// PARAMETERS ------------------------------------------------------------------

// Sampling period
// We want this to be at most 5 ms, pushing for 2 ms
const float Ts_ms = 2;
const float Ts_sec = Ts_ms/1000.;

// HARDWARE CONSTANTS ----------------------------------------------------------

// A randomly chosen I2C address for this box
// The SUGAR_bot address must match this one for the two
// arduinos to communicate properly
const int I2C_address = 0x1B;

// Encoder PPR (pulses per revolution)
const long ENCODER_PPR = 1000;

// Encoder counting mode (either 1, 2, or 4)
const int ENCODER_COUNT = 4;

// Digital pin which is hooked up to the master switch
const int SWITCH_PIN = 5;

// A little header for error-checking data transferred between the two arduinos
const byte HEADER[] = {0xFA, 0xCE};

// The analog pin to which the potentiometer is attached
const int POT_PIN = A3;

// GLOBAL STATES ---------------------------------------------------------------

// Output of the encoder converted to radians
float encoder_pos_rad = 0;

// ...run through a LPF to remove jagged edges, or tries to at least
float encoder_pos_filtered_1 = 0;

// This is the filtered angle, to report.
// It corresponds to Xingbo's psi, Adan's qu.
float encoder_angle_rad = 0;

// We estimate the angular velocity with a washout-lowpass filter
// We need to store the last two states of both the input(position)
// and the output (velocity)
float encoder_vel_raw_1 = 0;
float encoder_vel_raw_2 = 0;
float encoder_pos_rad_1 = 0;
float encoder_pos_rad_2 = 0;

// The velocity is then converted to rad/s
// This corresponds to psi_dot <-> qu_dot
float encoder_vel_rad_s = 0;

// State of the main switch
byte main_switch_state = 0;

// State of the main switch as seen by the robot arduino
byte bot_switch_state = 0;

// Angle of the servo as reported by the robot arduino
// This corresponds to alpha <-> qa
float servo_pos_rad = 0;

// Angular velocity of the servo as reported by the robot arduino
// This corresponds to alpha_dot <-> qa_dot
float servo_vel_rad_s = 0;

// Goal angle of the servo, alpha_des <-> qa_des
float servo_goal_rad = 0;

// State of the link between this arduino and the robot arduino
bool link_alive = false;

// Number of timeouts when waiting for request/receive from robot arduino
unsigned int wire_timeouts = 0;

// The reading from the knob
float potentiometer_state = 0;

// Energy. More on this later..
float energy = 0;

// The following 8 chunks of data must be sent to the computer, but
// only one chunk per iteration (one period of Ts). This counter tells
// us which chunk we should be sending next.
// 1 Switch states (both bot and box)
// 2 Potentiometer state
// 3 Encoder angle
// 4 Encoder velocity
// 5 Servo position
// 6 Servo velocity
// 7 Servo goal position
// 8 Energy, as calculated by robot
unsigned int iteration_phase = 0;

// Need to use union types to send float data over I2C
union WireData{
  float d;
  byte b[4];
};

WireData tempData;
WireData tempData2;

byte data_buffer[32];
  
// INITIALIZATIONS -------------------------------------------------------------

// Timer object (TODO find out if interrupts delay the execution of the timer)
Timer timer;

// Create the Encoder which uses pins D2 and D3, both having interrupt capability
Encoder encoder(2,3);
//ABZEncoder* encoder = ABZEncoder::getInstance(2,3,13,ENCODER_PPR,ENCODER_COUNT);

void setup() {
  // Let's try to use a faster I2C transfer rate, 400kHz
  Wire.setClock(400000L);
  Wire.begin(I2C_address);
  Wire.onReceive(wireReceiveEvent);
  Wire.onRequest(wireRequestEvent);

  //set the switch pin to input
  pinMode(SWITCH_PIN, INPUT);

  //set the pot pin to input
  pinMode(POT_PIN, INPUT);

  // Using lower baud rates seem to cause timing issues.
  // at 9600, one byte of data is sent in about 1ms
  // our loop is running at 500Hz, or 2ms per iteration
  // Each iteration we send up to 32 bytes of data
  // we want a rate that is AT LEAST 50 times faster
  // making the transmission time about 600us
  // luckily the arduino supports up to 2,000,000 baud rate
  // See here http://arduino.stackexchange.com/questions/296/
  //                      how-high-of-a-baud-rate-can-i-go-without-errors
  // UPDATE: still this isn't good enough since the computer
  // interface doesn't handle high baud rates very well.
  // We will instead split the content of the transmission in
  // 8, and use a slower baud rate to transmit the data over
  // several iterations.
  Serial.begin(250000);
  // A real-time loop that executes once every Ts_ms (2) milliseconds
  timer.every(Ts_ms, rtloop);

}

// LOOPS -----------------------------------------------------------------------

// This function runs every Ts_ms milliseconds
void rtloop() {
  // Update the switch status
  main_switch_state = digitalRead(SWITCH_PIN);
  
  // Refresh data from the encoder  
  long encoder_pos_raw = encoder.read();
  
  encoder_pos_rad = encoderMap((float)(encoder_pos_raw), 2*M_PI);
  
  // Filter the raw data to remove some of the jagged edges
  // TODO: Xingbo wrote this filter. Where do the numbers come from? Replace these
  // with variables.
  encoder_angle_rad = 0.2929*(encoder_pos_rad + encoder_pos_rad_1) 
                          + 0.4142*encoder_pos_filtered_1;
  encoder_pos_filtered_1 = encoder_angle_rad;

  // Run the filter to calculate velocity
  // This is a washout filter at 40rad/s cascaded with a lowpass filter at 50rad/s
  encoder_vel_rad_s = 2.695*(encoder_pos_rad - encoder_pos_rad_2)
                    + 1.792*encoder_vel_raw_1 - 0.8023*encoder_vel_raw_2;
                    
  // Update the old variables
  encoder_vel_raw_2 = encoder_vel_raw_1;
  encoder_vel_raw_1 = encoder_vel_rad_s;
  encoder_pos_rad_2 = encoder_pos_rad_1;
  encoder_pos_rad_1 = encoder_pos_rad;

  // Read the potentiometer
  potentiometer_state = potMap(analogRead(POT_PIN), 1.4, 2.0);

  //at the beginning of the step, capture all data in the data_buffer
  if(iteration_phase >= 8){
    // switch state
    // Send the header first for error checking
    data_buffer[0] = HEADER[0];
    data_buffer[1] = HEADER[1];
    // switch states
    data_buffer[2] = main_switch_state;
    data_buffer[3] = bot_switch_state;

    // potentiometer state
    tempData2.d = potentiometer_state;
    for(int i = 0; i < 4; i++)
      data_buffer[4 + i] = tempData2.b[i];
  
    // encoder angle
    tempData2.d = encoder_angle_rad;
    for(int i = 0; i < 4; i++)
      data_buffer[8 + i] = tempData2.b[i];
      
    // encoder velocity
    tempData2.d = encoder_vel_rad_s;
    for(int i = 0; i < 4; i++)
      data_buffer[12 + i] = tempData2.b[i];
  
    // servo angle
    tempData2.d = servo_pos_rad;
    for(int i = 0; i < 4; i++)
      data_buffer[16 + i] = tempData2.b[i];
  
    // servo velocity
    tempData2.d = servo_vel_rad_s;
    for(int i = 0; i < 4; i++)
      data_buffer[20 + i] = tempData2.b[i];
  
    // servo reference angle
    tempData2.d = servo_goal_rad;
    for(int i = 0; i < 4; i++)
      data_buffer[24 + i] = tempData2.b[i];

    tempData2.d = energy;
    for(int i = 0; i < 4; i++)
      data_buffer[28 + i] = tempData2.b[i];

    iteration_phase = 0;
  }

#if DEBUG_ENABLE
  // Print twice per second.
  // This loop runs once every Ts_ms, so we need 500*Ts_ms iterations 
  // between each printout.
  if(iteration_phase % (int((500*Ts_ms))) == 0)
  {
    Serial.println("");
    Serial.println("-------------------------");
    Serial.print("Switch State = ");
    Serial.println(main_switch_state);
    Serial.print("Switch State (bot view) = ");
    Serial.println(bot_switch_state);
    Serial.print("Encoder angle = ");
    Serial.println(encoder_angle_rad);
    Serial.print("Encoder Velocity = ");
    Serial.println(encoder_vel_rad_s);
    Serial.print("Potentiometer Value = ");
    Serial.println(potentiometer_state);
    Serial.print("Servo Position = ");
    Serial.println(servo_pos_rad);
    Serial.print("Servo Velocity = ");
    Serial.println(servo_vel_rad_s);
    Serial.print("* Desired Servo Position = ");
    Serial.println(servo_goal_rad);
    Serial.print("Acrobot Energy = ");
    Serial.println(energy);
    Serial.println("-------------------------");
    Serial.println("");
  }
  
#else
  // Send some data over serial
  // Let's try removing some overhead and call Serial.write just once
  // some fancy pointer work here to send the right chunks of data
  Serial.write( (4*iteration_phase + data_buffer), 4);
#endif

  // Update the counter
  iteration_phase++;
  
  // Check if we are receiving requests from the master
  wire_timeouts++;
  // If we haven't received anything in over 10ms
  if(wire_timeouts > 5){
    // Something might be wrong, and we should raise a flag TODO
    link_alive = false;
  }
}

// Function to be called when the robot arduino requests data from this device
void wireRequestEvent() {
  // See http://forum.arduino.cc/index.php?topic=109335.0
  // There must only be one Wire.write() command or else I get that problem
  
  // First send the header, indicating the start of the data stream
  byte data_array[15];
  data_array[0] = HEADER[0];
  data_array[1] = HEADER[1];
  
  // Now send the requested data, which in this case is the state of the switch
  data_array[2] = main_switch_state;

  // the angle of the encoder
  tempData.d = encoder_angle_rad;
  data_array[3] = tempData.b[0];
  data_array[4] = tempData.b[1];
  data_array[5] = tempData.b[2];
  data_array[6] = tempData.b[3];

  // the anglular velocity of the encoder
  tempData.d = encoder_vel_rad_s;
  data_array[7] = tempData.b[0];
  data_array[8] = tempData.b[1];
  data_array[9] = tempData.b[2];
  data_array[10] = tempData.b[3];

  // and the potentiometer state
  tempData.d = potentiometer_state;
  data_array[11] = tempData.b[0];
  data_array[12] = tempData.b[1];
  data_array[13] = tempData.b[2];
  data_array[14] = tempData.b[3];
  
  Wire.write(data_array, 15);

  // Update the link status since we received a request
  link_alive = true;
  wire_timeouts = 0;
}


// The robot arduino sends us some data as well. Here is where we read it
void wireReceiveEvent(int howmany){
  // The state of the switch as seen by the robot
  bot_switch_state = Wire.read();

  // The angle of the hip joint as reported by the servo, raw
  // the resolution is 0.29 degrees or 0.0051 rad
  tempData2.b[0] = Wire.read();
  tempData2.b[1] = Wire.read();
  tempData2.b[2] = Wire.read();
  tempData2.b[3] = Wire.read();
  servo_pos_rad = tempData2.d;

  // Read the servo angular velocity data
  tempData2.b[0] = Wire.read();
  tempData2.b[1] = Wire.read();
  tempData2.b[2] = Wire.read();
  tempData2.b[3] = Wire.read();
  servo_vel_rad_s = tempData2.d;

  // Read the servo goal data
  tempData2.b[0] = Wire.read();
  tempData2.b[1] = Wire.read();
  tempData2.b[2] = Wire.read();
  tempData2.b[3] = Wire.read();
  servo_goal_rad = tempData2.d;

  // Read the energy data
  tempData2.b[0] = Wire.read();
  tempData2.b[1] = Wire.read();
  tempData2.b[2] = Wire.read();
  tempData2.b[3] = Wire.read();
  energy = tempData2.d;
}

void loop() {
  // Update the timer, do nothing else here
  timer.update();
}

// AUXILIARY STUFF ----------------------------------------------------------

float encoderMap(long x, float out_max)
{
  return (float)(x) * (out_max ) / (float)(ENCODER_PPR*ENCODER_COUNT);
}

float potMap(int x, float out_min, float out_max)
{
  return (out_max - out_min)*(float)x/(float)(1024) + out_min;
}

float sgn(float x)
{
  if(x == 0){
    return 0;
  }else{
    return x < 0 ? -1.0 : 1.0;
  }
}
