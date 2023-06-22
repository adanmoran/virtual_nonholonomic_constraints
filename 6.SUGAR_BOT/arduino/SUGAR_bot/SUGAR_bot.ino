/**
 * AUTHOR: Xingbo Wang, MASc
 *
 * Systems Control Group
 * Department of Electrical and Computer Engineering
 * University of Toronto
 *
 * For use with the SUGAR system
 * Code for the acrobot arduino nano
 *
 * Last Modified: 12 January 2020
 * Last Editor: Adan Moran-MacDonald
 */

#include <Wire.h>
#include <Event.h>
#include <Timer.h>
#include <math.h>
#include <RX24F.h>
#include "SUGAR_bot.h"

// PARAMETERS ------------------------------------------------------------------

// Sampling period
// We want this to be at most 5 ms, pushing for 2 ms
const double Ts_ms = 2;
const double Ts_sec = Ts_ms / 1000.;

// Robot parameters, in SI units
const double Mt = 0.2112;  // kg
const double Ml = 0.1979;  // kg
const double Jt = 0.00075; // kg*m^2
const double Jl = 0.00129; // kg*m^2
const double Rt = 0.148;   // m
const double Rl = 0.145;   // m
const double lt = 0.073;   // m
const double ll = 0.083;   // m
const double g = 9.8;      // m/s^2

// Robot Inverse Inertia Matrix
const auto M = AcrobotInertia(
	Mt, Ml,
	Rt, Rl,
	lt, ll,
	Jt, Jl);

// Energy quantities
const double E_still_exit = 0.15;
const double E_still_enter = 0.05;

// HARDWARE CONSTANTS ----------------------------------------------------------

// The I2C address of the switch box
const int Box_I2C_address = 0x1B;

// Hardware ID of the Dynamixel Servo, which defaults to 1 and is not changed
const int SERVO_ID = 1;

// Baud rate for communication with the Dynamixel Servo. This must be set to the same
// value both here and on the servo itself.
// Pushing for as high as we can here. I tried 1,000,000 but that causes problems
// lost packets, misreads, and whatnot
const long SERVO_BAUD_RATE = 500000L;

// The servo has a range of 0 - 300 degrees, the latter of which is this number in radians
const double MAX_SERVO_ANGLE_RAD = 5.23599;

// A little header for error-checking data transferred between the two arduinos
const byte HEADER[] = {0xFA, 0xCE};

// GLOBAL STATES ---------------------------------------------------------------

// State of the main switch as reported by the box arduino
byte main_switch_state = 0;

// The position of the encoder in radians as reported by the box arduino
double encoder_pos_rad = 0;

// The angular velocity of the encoder in radians/s as reported by the box arduino
double encoder_vel_rad_s = 0;
double encoder_vel_rad_s_1 = 0;

// The velocity of the encoder, passed through a LPF
double encoder_vel_rad_s_lpf = 0;
double encoder_vel_rad_s_lpf_1 = 0;

// The position of the hip joint servo
double servo_pos_rad = 0;
double servo_pos_rad_1 = 0;
double servo_pos_rad_2 = 0;

// The velocity of the hip joint servo, as calculated using a 2nd order filter
double servo_vel_rad_s = 0;
double servo_vel_rad_s_1 = 0;
double servo_vel_rad_s_2 = 0;

// The goal position of the hip joint servo
double servo_goal_rad = 0;

// The total mechanical energy of the robot
double energy = 0;
double energy_1 = 0;
double energy_filtered = 0;
double energy_filtered_1 = 0;

// Total mechanical energy at the angle psi = pi
double energy_discrete = 0;

// The desired energy level to stabilize
double E0 = 0;
double psi_last = 0;

// Declare the initial VHC state for the automaton.
// For some reason this is required for making the code compile.
void vhc_still_state(Configuration, Compensator);

// Assign the initial VHC state function to the automaton.
VHCState state = vhc_still_state;
unsigned int still_counter = 0;

// INITIALIZATIONS -------------------------------------------------------------

// Timer object
Timer timer;

void setup() {
  // Let's try to use a faster I2C transfer rate, 400kHz
  Wire.setClock(400000L);
  Wire.begin();

  // The Dynamixel RX-24F servo uses RS485 protocol
  // 500kHz should be fast enough for a loop rate of 500Hz
  // 2 is the direction pin, as required by RS485
  RX24F.begin(SERVO_BAUD_RATE, 2);

  // Compliance Margin and Compliance Slope set here. These
  // are set to the extreme according to Dynamixel's specifications.
  RX24F.setCSlope(SERVO_ID, 0x02, 0x02);
  RX24F.setCMargin(SERVO_ID, 0x00, 0x00);
  RX24F.setAngleLimit(SERVO_ID, 512 + 308, 512 - 308);

  // A real-time loop that executes once every Ts_ms (2) milliseconds
  // NOTE after further investigation into the timer library, this is NOT
  // guaranteed to be real time (...duh)
  timer.every(Ts_ms, rtloop);

  // Pin 13 is the built-in LED, used for debugging purposes
  pinMode(13, OUTPUT);
}

// Need to use union types to send float data over I2C
union WireData {
  double d;
  byte b[4];
};

WireData temp_data;

// LOOPS -----------------------------------------------------------------------

// This function runs every Ts_ms milliseconds
void rtloop() {
  
  // Start by assuming the switch is off, query the switch box for actualy state
  main_switch_state = 0;
  
  // Get the servo and encoder velocities
  update_velocities();
  
  // Update data as measured by the other arduino
  get_wire_data();
  
  // Update the main switch state and configuration variables
  update_configuration();
  
  // Rename the energy for ease-of-use in many places
  energy = configuration.E;
  // Filter the energy, as it changes very dramatically.
  // TODO: What are the magic numbers in this filter? 
  energy_filtered = 0.0378 * (energy + energy_1) + 0.9244 * energy_filtered_1;
  
  // Decide whether or not to update the discrete energy
  // TODO: This may not be necessary, energy_discrete is not used anywhere
  if(psi_last > (M_PI - 0.1) && configuration.psi < -(M_PI - 0.1)){
    energy_discrete = energy_filtered;
  }
  
  // Update the compensator
  // TODO: This is not used by the VNHC
  double dpsi_filtered = encoder_vel_rad_s_lpf;
  compensator.s = energy_filtered;
  compensator.xi = toXi(configuration.psi, dpsi_filtered);
  compensator.rho = toRho(configuration.psi, dpsi_filtered);
  
  // Check the state of the master switch
  bool switch_on = (main_switch_state == 1);
  RX24F.ledStatus(SERVO_ID, switch_on);
  
  if (switch_on) {
    RX24F.torqueStatus(SERVO_ID, true);
  
    // This calls the current state's corresponding function to set servo_goal_rad
    // and determine the next state, in Xingbo's VHC automaton
    // state(configuration, compensator);

    // TODO: Set the servo_goal_rad with the vnhc update function.
  
    // Send it off to the servo
    int pos = radiansToServo(servo_goal_rad);
    RX24F.move(1, pos);
  
  } else {
    RX24F.torqueStatus(SERVO_ID, false);
  }
  
  // Send data back to the other box
  send_wire_data();
  
  // Update the variables keeping track of previous states
  // Servo / Encoder data 
  servo_pos_rad_2 = servo_pos_rad_1;
  servo_pos_rad_1 = servo_pos_rad;
  servo_vel_rad_s_2 = servo_vel_rad_s_1;
  servo_vel_rad_s_1 = servo_vel_rad_s;
  encoder_vel_rad_s_1 = encoder_vel_rad_s;
  encoder_vel_rad_s_lpf_1 = encoder_vel_rad_s_lpf;
  // Configuration data
  psi_last = configuration.psi;
  // Energy data
  energy_1 = energy;
  energy_filtered_1 = energy_filtered;
}

void loop() {
  // Update the timer, do nothing else here
  timer.update();
}

// Loop Functions ---------------------------------------------------------

/**
 * Use the previously stored servo data to update the current velocities
 * of the servo and encoder measurements.
 *
 * Variables which are updated:
 * - servo_pos_rad
 * - servo_vel_rad_s
 * - encoder_vel_rad_s_lpf
 */
void update_velocities()
{
  // Update the servo position and calculate velocity
  servo_pos_rad = servoToRadians(RX24F.readPosition(SERVO_ID));

  // TODO: What are these magic constants? Check the servo's datasheet
  //       and replace them with constant variables
  servo_vel_rad_s = 11.58 * (servo_pos_rad - servo_pos_rad_2)
                    + 0.7799 * servo_vel_rad_s_1 - 0.1506 * servo_vel_rad_s_2;

  encoder_vel_rad_s_lpf = 0.1367 * (encoder_vel_rad_s + encoder_vel_rad_s_1)
                          + 0.7265 * encoder_vel_rad_s_lpf_1;
}

/**
 * Request the following info from the other Arduino:
 * - main switch state
 * - encoder position in radians
 * - encoder velocity in radians/s
 * - potentiometer position, scaled appropriately
 * 
 * Variables which are updated:
 * - main_switch_state
 * - encoder_pos_rad
 * - encoder_vel_rad_s
 * - E0
 */
void get_wire_data()
{
	Wire.requestFrom(Box_I2C_address, 15);

  // Attempt to read the header of this data stream
  byte header_test[2];
  header_test[0] = Wire.read();
  header_test[1] = Wire.read();

  bool header_found = ((header_test[0] == HEADER[0]) && 
					   (header_test[1] == HEADER[1]));

  // Decode the data only when we recognize the header
  if (header_found) {
    // read switch state
    main_switch_state = Wire.read();

    // read encoder position
    temp_data.b[0] = Wire.read();
    temp_data.b[1] = Wire.read();
    temp_data.b[2] = Wire.read();
    temp_data.b[3] = Wire.read();
    encoder_pos_rad = temp_data.d;

    // read encoder velocity
    temp_data.b[0] = Wire.read();
    temp_data.b[1] = Wire.read();
    temp_data.b[2] = Wire.read();
    temp_data.b[3] = Wire.read();
    encoder_vel_rad_s = temp_data.d;

    // read desired energy
    temp_data.b[0] = Wire.read();
    temp_data.b[1] = Wire.read();
    temp_data.b[2] = Wire.read();
    temp_data.b[3] = Wire.read();
    E0 = temp_data.d;

  } else {
    // If header is unrecognized, discard the rest of the data
    // NOTE: perhaps we can loop until we find the header in this mess,
    // but that will complicate other issues such as timing and how
    // the wire library handles buffered data
    while (Wire.available()) {
      Wire.read();
    } // endwhile
  } // endif
}

/**
 * Send the following back to the other Arduino box:
 * - main_switch_state
 * - servo_pos_rad
 * - servo_vel_rad_s
 * - servo_goal_rad
 * - energy
 */
void send_wire_data()
{
  Wire.beginTransmission(Box_I2C_address);

  // Let us only call Wire.write once to reduce overhead
  byte data_buffer[17];

  // Report back the switch state, for error checking or something
  data_buffer[0] = main_switch_state;

  // Report the servo position data
  temp_data.d = servo_pos_rad;
  for (int i = 0; i < 4; i++)
    data_buffer[1 + i] = temp_data.b[i];

  // Report the servo velocity data
  temp_data.d = servo_vel_rad_s;
  for (int i = 0; i < 4; i++)
    data_buffer[5 + i] = temp_data.b[i];

  // Report the servo goal data
  temp_data.d = servo_goal_rad;
  for (int i = 0; i < 4; i++)
    data_buffer[9 + i] = temp_data.b[i];

  // Report the estimated energy
  temp_data.d = energy;
//  temp_data.d = compensator.s;
  for (int i = 0; i < 4; i++)
    data_buffer[13 + i] = temp_data.b[i];

  Wire.write(data_buffer, 17);

  Wire.endTransmission();
}

/**
 * Update the configurations
 * This function updates the servo/encoder velocities, then downloads
 * the required data from the wire before updating the configuration object.
 */
void update_configuration()
{
  // Update the configuration with the current state
  configuration.psi = atan2(sin(encoder_pos_rad), cos(encoder_pos_rad));
  configuration.alpha = atan2(sin(servo_pos_rad), cos(servo_pos_rad));
  configuration.dpsi = encoder_vel_rad_s;
  configuration.dalpha = servo_vel_rad_s;
  configuration.E = compute_energy(configuration);
}

/**
 * Compute the mechanical energy of the acrobot
 * E(configuration) = qd' * M(q) * qd + V(q)
 */
double compute_energy(Configuration configuration)
{
  double E = (Jl * sq(configuration.dalpha)) / 2.0 + (Jl * sq(configuration.dpsi)) / 2.0 
	   + (Jt * sq(configuration.dpsi)) / 2.0 + (Ml * sq(Rt) * sq(configuration.dpsi)) / 2.0
	   + (Ml * sq(configuration.dalpha) * sq(ll)) / 2.0 + (Ml * sq(configuration.dpsi) * sq(ll)) / 2.0 
	   + (Mt * sq(configuration.dpsi) * sq(lt)) / 2.0 + Jl * configuration.dalpha * configuration.dpsi 
	   - Ml * g * ll * cos(configuration.alpha + configuration.psi) - Ml * Rt * g * cos(configuration.psi)
	   + Ml * configuration.dalpha * configuration.dpsi * sq(ll) - Mt * g * lt * cos(configuration.psi) 
	   + Ml * Rt * sq(configuration.dpsi) * ll * cos(configuration.alpha)
	   + Ml * Rt * configuration.dalpha * configuration.dpsi * ll * cos(configuration.alpha) 
	   + g * (Ml * (Rt + ll) + Mt * lt);
	   
  return E;
}

// VHC ----------------------------------------------------------------------

// Copied from my MATLAB/Simulink code

const double alpha_max_back = M_PI / 4.5;
const double alpha_max_front = M_PI / 2.25;
const double rho0 = 0.6;
const double s0 = 0.05;

double f_forward(double psi) {
  psi = atan2(sin(psi), cos(psi));
  return -alpha_max_front * (psi + M_PI / 4.0) * psi * 
    pow(psi + M_PI, 3.0) * pow(psi - M_PI, 3.0) * (20.0 / pow(M_PI, 9.0));
}

double f_backward(double psi) {
  psi = atan2(sin(psi), cos(psi));
  if (psi < 0) {
    return -alpha_max_back * pow(psi, 3.0) * pow(psi + M_PI, 3.0) * pow(2.0 / M_PI, 6.0);
  } else {
    return 0.0;
  }
}

double b1(double s) {
  return 0.5 + 0.5 * tanh(s / s0);
}

double b2(double s) {
  return 0.5 - 0.5 * tanh(s / s0);
}

double b4(double w) {
  return 40*w / ( 40*abs(w) + 1.0);
}

double toXi(double psi, double s) {
  return atan2(0.1*s, 0.7*psi);
}

double toRho(double psi, double s) {
  return sqrt(0.7*sq(psi) + 0.1*sq(s));
}

double Fa(double xi) {
  if (xi < 0)
    return 0;
  else if( xi <= M_PI/2)
    return exp(1.0 - 1.0 / (1 - sq(4.0 * xi / M_PI - 1))) * alpha_max_front;
  else
    return 0;
}

double Fr(double rho) {
  return sq(tanh(rho / rho0));
}
  
// VHC AUTOMATON ------------------------------------------------------------

void vhc_still_state(Configuration conf, Compensator comp) {
  if(still_counter < 25){
    servo_goal_rad = M_PI / 2.0;
  }else{
    servo_goal_rad = 0;
  }
  still_counter++;
  if(still_counter > 50){
    still_counter = 0;
  }

  if (comp.s > E_still_exit && still_counter > 35) {
    state = vhc_tap_state;
  } else {
    state = vhc_still_state;
  }
}

void vhc_tap_state(Configuration conf, Compensator comp) {
  servo_goal_rad = Fa(comp.xi) * Fr(comp.rho);

  if (comp.s < E_still_enter) {
    state = vhc_still_state;
    still_counter = 0;
  } else if (conf.psi < -M_PI/2.0 && psi_last > M_PI/2.0) {
    state = vhc_giant_state;
  } else {
    state = vhc_tap_state;
  }
}

void vhc_giant_state(Configuration conf, Compensator comp) {
  double m = -0.5 * (comp.s - E0);
  
  servo_goal_rad = b4(m) * ( b1(m) * f_forward(conf.psi) - b2(m) * f_backward(conf.psi));

  if(conf.dpsi < 0){
    state = vhc_tap_state;
  } else {
    state = vhc_giant_state;
  }
}

// AUXILIARY STUFF ----------------------------------------------------------

// Convert servo position reading to radians,
// with zero radians being completely parallel to the torso link
double servoToRadians(long x) {
  return (double)(x - 512) / (double)1024 * MAX_SERVO_ANGLE_RAD;
}

// Convert a joint angle in radians to a servo position,
// with zero being at 150 degrees, or 512
int radiansToServo(double rad) {
  return rad * (1024.) / (MAX_SERVO_ANGLE_RAD) + 512;
}
