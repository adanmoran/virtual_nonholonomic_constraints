/**
 * AUTHOR: Adan Moran-MacDonald, MASc
 *
 * Systems Control Group
 * Department of Electrical and Computer Engineering
 * University of Toronto
 *
 * Implementation of VNHC code for use with the SUGAR system.
 * Adapted from Xingbo Wang's work (MASc 2016).
 *
 * Last Modified: 12 February 2020
 * Last Editor: Adan Moran-MacDonald
 */

// Do this if you want to print everything out to Serial WITHOUT using the motor
// That is, if you set DEBUG_ENABLE, all Serial interaction with the
// RX24F servo is disabled so we can read Arduino debug data from serial
#define DEBUG_ENABLE 0

// Includes for our system to work
#include <Wire.h>
#include <Event.h>
#include <Timer.h>
#include <math.h>
#include <RX24F.h>
#include "SUGAR_VNHC.h"
using namespace SUGAR;

// PARAMETERS ------------------------------------------------------------------
// Default configuration object
Configuration configuration;

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
const double g = 9.81;      // m/s^2h

// Generate an acrobot object, which will contain the Acrobot inertia and
// potential functions
Acrobot acrobot(
  Mt, Ml,
  Rt, Rl,
  lt, ll,
  Jt, Jl,
  g
);

// Define the actuator limit for the acrobot, which must be in the range
// [0, M_PI/2] for feasibility with the physical acrobot.
// Note: small actuator limits may make it impossible for the acrobot
// to achieve full rotations, since it cannot overcome friction.
ActuatorLimit qa_max = M_PI/2;

// Generate a VNHC for the qa_max*tanh(scale*pu) function:
// Note: since we have very small momentum (the largest pu is around 0.2)
//, we set tanh_scale to be large so that the controller will gain energy.
// This is done heuristically
// at the moment, though hopefully there will be a theoretical value
// based on the masses / lengths of the links / actuator limits
// which we can employ later.
ScalingFactor tanh_scale = 20;
TanhVNHC tanh_vnhc(acrobot, qa_max, tanh_scale);

// Generate a VNHC for the sin(theta) function
SinuVNHC sinu_vnhc(acrobot, qa_max);

// Generate a VNHC for the qa_max*(2/pi)*arctan(scale*pu) function
// Note that the acrobot rotates when pu = sqrt(60m^2gl^3)
// If you take m = avg(ml,mt) and l = avg(Rt,Rl,ll,lt), MATLAB says that
// sqrt(60m^2gl^3) = 0.1866. 
// Since qmax = pi/2, qa=arctan(Ipu) can never reach pi/2,
// we want qa ~ pi/3 at pu = 0.1866.
// i.e we need pi/3 = arctan(I*0.1866) <-> I = tan(pi/3)/0.1866 ~ 10.
ScalingFactor I = 10;
ScalingFactor I_diss = -3;
ArctanVNHC arctan_in(acrobot, qa_max, I);
ArctanVNHC arctan_diss(acrobot,qa_max, I_diss);

// Assign the VNHC pointers
AcrobotVNHC* pVNHCin = &arctan_in;
AcrobotVNHC* pVNHCdiss = &arctan_diss;

// Define the supervisor
//Supervisor sup(arctan_in, arctan_diss);
//OscillationSupervisor osup(sup);
//RotationSupervisor rsup(sup);

enum MetaSupervisor
{
  INJECTION,
  DISSIPATION,
//  OSCILLATION,
//  ROTATION,
  ENERGIZATION
};
MetaSupervisor supervisor = INJECTION;

// Set the parameters for oscillation supervisors
double qu_des = PI/2; // the desired oscillation value. Must be in [0,pi].
double osc_hys = 0.1; // hysteresis on oscillation angle, in rad.
bool overcompensate = false; // if true, set qa = 0 any time we go within range of qu_des

// Set the parameters for rotation supervisors
//NOTE: for arctan and I = 10, rotations occur at (0,pu) for pu in [0.15, 0.195], as per Adan's thesis.
double pu_des = 0.16; // the desired rotation value.
double rot_hys = pu_des/20; // hysteresis on rotation momentum

// Set the parameter for the energization controller, which is to inject below some E and dissipate above
double E_des = 0.5997*(1-cos(qu_des)); // nominal energy of pendulum at (qu_des,0)
double E_hys = E_des/20; // 5% error for hysteresis

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

// The position of the encoder in radians as reported by the box arduino.
// This corresponds to qu <-> configuration.psi.
double encoder_pos_rad = 0;

// The angular velocity of the encoder in radians/s as reported by the box arduino.
// This corresponds to qu_dot <-> configuration.dpsi.
double encoder_vel_rad_s = 0;

// The position of the hip joint servo. 
// This corresponds to the qa <-> configuration.alpha.
double servo_pos_rad = 0;
// We keep two previous iterations of the servo position for the qa_dot velocity filter.
double servo_pos_rad_1 = 0;
double servo_pos_rad_2 = 0;

// The velocity of the hip joint servo, as calculated using a 2nd order filter.
// This corresponds to qa_dot <-> configuration.dalpha.
double servo_vel_rad_s = 0;
double servo_vel_rad_s_1 = 0;
double servo_vel_rad_s_2 = 0;

// The goal position of the hip joint servo, qa_desired.
double servo_goal_rad = 0;

// The total mechanical energy of the robot
double energy = 0;

// The desired energy level to stabilize. CURRENTLY NON_FUNCTIONAL.
double E0 = 0;

// INITIALIZATIONS -------------------------------------------------------------

// Timer object
Timer timer;

void setup() {
  // Let's try to use a faster I2C transfer rate, 400kHz
  Wire.setClock(400000L);
  Wire.begin();

#if DEBUG_ENABLE
  Serial.begin(9600);
  Serial.println("---------------");
  Serial.println(" Debug Enabled ");
  Serial.println("---------------");
  
#else // DEBUG_ENABLE
  // The Dynamixel RX-24F servo uses RS485 protocol
  // 500kHz should be fast enough for a loop rate of 500Hz
  // 2 is the direction pin, as required by RS485
  RX24F.begin(SERVO_BAUD_RATE, 2);

  // Compliance Margin and Compliance Slope set here. These
  // are set to the extreme according to Dynamixel's specifications.
  RX24F.setCSlope(SERVO_ID, 0x02, 0x02);
  RX24F.setCMargin(SERVO_ID, 0x00, 0x00);
  RX24F.setAngleLimit(SERVO_ID, 512 + 308, 512 - 308);
#endif // ifdef DEBUG_ENABLE
  
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
  
  // Check the state of the master switch
  bool switch_on = (main_switch_state == 1);
#if !DEBUG_ENABLE
  RX24F.ledStatus(SERVO_ID, switch_on);
#endif // ifdef DEBUG_ENABLE
  
  if (switch_on) {
    // Convert the current configuration object to a phase object, then
    // get its unactuated component
    Phase phase(acrobot.M(), configuration);
    UnactuatedPhase qpu = phase.unactuatedPhase();
    
    // Set the servo_goal_rad with the supervisor.
    // For some reason switch-case breaks this, so use if-statements.
    if(supervisor == INJECTION)
    {
      servo_goal_rad = pVNHCin->qa(qpu);
    } 
    else if (supervisor == DISSIPATION)
    {
      servo_goal_rad = pVNHCdiss->qa(qpu);
    }
    else if (supervisor == ENERGIZATION)
    {
      // Inject below desired energy, dissipate above,
      // do nothing in between.
      if (energy <= E_des - E_hys){
        servo_goal_rad = pVNHCin->qa(qpu);
      } else if ( energy >= E_des + E_hys) {
        servo_goal_rad = pVNHCdiss->qa(qpu);
      } else {
        servo_goal_rad = servo_goal_rad;
      }
    } // if (supervisor)
    
    // Send it off to the servo
    int pos = radiansToServo(servo_goal_rad);
    
#if DEBUG_ENABLE
    Serial.print("qu = ");
    Serial.print(phase.qu);
    Serial.print(" | qu_d = ");
    Serial.print(encoder_vel_rad_s);
    Serial.print(" | pu = ");
    Serial.print(phase.pu);
    Serial.print(" | qa_des = ");
    Serial.println(servo_goal_rad);
  } // if (switch_on)
  else {
  }
#else // ~DEBUG_ENABLE
    RX24F.torqueStatus(SERVO_ID, true);
    RX24F.move(1, pos);
  } // if (switch_on)
  else {
    RX24F.torqueStatus(SERVO_ID, false);
  }
#endif // ifdef DEBUG_ENABLE
  
  // Send data back to the other box
  send_wire_data();
  
  // Update the variables keeping track of previous states
  // Servo / Encoder data 
  servo_pos_rad_2 = servo_pos_rad_1;
  servo_pos_rad_1 = servo_pos_rad;
  servo_vel_rad_s_2 = servo_vel_rad_s_1;
  servo_vel_rad_s_1 = servo_vel_rad_s;
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
 */
void update_velocities()
{
  // Update the servo position and calculate velocity
#if DEBUG_ENABLE
  servo_pos_rad = 0;
#else // !DEBUG_ENABLE
  servo_pos_rad = servoToRadians(RX24F.readPosition(SERVO_ID));
#endif // ifdef DEBUG_ENABLE

  // TODO: What are these magic constants? Xingbo wrote them in without comment.
  // Check the Servo datasheet and replace them with constant variables.
  servo_vel_rad_s = 11.58 * (servo_pos_rad - servo_pos_rad_2)
                    + 0.7799 * servo_vel_rad_s_1 - 0.1506 * servo_vel_rad_s_2;
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
  configuration.psi = encoder_pos_rad; //atan2(sin(encoder_pos_rad), cos(encoder_pos_rad));
  configuration.alpha = servo_pos_rad; //atan2(sin(servo_pos_rad), cos(servo_pos_rad));
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
