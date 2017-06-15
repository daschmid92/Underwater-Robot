/*
  Flipper Wiggle Sparse PWM
  This program is used to control the flippers without real-time encoder data.
  This progrma is required, because the power delivered through motor box tethers causes
  so much interference with parallel lines that it is impossible to read the encoder
  when the motor is running.
  The timer "calibration" is done with the following steps:
    1. find the starting position of the flipper
    2. move the flipper in the forward direction for a certain duration
    3. find the new position of the flipper
    4. move the flipper in the backward diretion for a certain duraction
    5. find the new position of the flipper
  This will provide an estimate to the amount of time to drive the flipper for a specified
  range of motion in each direction.

  The PWM version of this program uses the Servo library to control the Simple Motor
  Controllers. I had difficulty reliably using serial communication to command the
  SMCs. The SMCs would halt the motor if they received bad serial data.

  Anselm J. Mak
  Created: 2016-02-05
  Updated: 2016-05-30

  Nicewell Su
  Updated: 2017-03-11
  
  Doug Schmidt
  updated: 2017-06-08
*/

#include <SPI.h> // if including <ArduinoRobot.h>, exlude this
#include <SoftwareSerial.h>
#include <Servo.h>
#include <Math.h>

// define pin numbers for the serial comm with Pololu motor drivers
#define txPin 18  //Pin #
#define rxPin 19  //Pin #

// for Mega
#define encoder_1 32  //Pin # Normally (24)
#define encoder_2 31  //Pin # Normally (23)
#define encoder_3 30  //Pin # Normally (22)
#define m1_PWM 11     //Pin #
#define m2_PWM 10     //Pin #
#define m3_PWM 9      //Pin #
#define MOSI_pin 51   //Pin #
#define MISO_pin 50   //Pin #
#define SPI_SCK 52    //Pin #
#define SS_pin 53     // Slave Select pin should be set as OUTPUT
#define BDCM_PWM 12   //Pin #
#define SPI_speed 120000    //Serial Peripheral Interface baud rate set for intial encoder reading, used in first_read()
#define serial_baud 250000  //Baud rate set for serial monitor for communications between arduino and computer
#define motor_baud 115200   //Motor driver baud rate with computer, not currently used
#define SPI_wait 1 // recommended 20 micro-sec delay between reads, not currently used
#define max_speed 90 // max deviation from 90 for Servo
#define max_PID 40 // maximum speed for PID control
#define max_prop 50 // maximum speed for driving propellers   ////Change this value to 90 for water motion      
#define max_wheel 60 // maximum speed for surface mode driving wheels using frictional omniwheel contact, used in surf_drive() to establish
#define BDCM_speed 85 // max deviation from 90 for BDCM Servo (Input of 90 for central propeller yields no motion, > 90 +z, <90 -z)
#define thresh 12 // "close enough" for flipper position target (1 degree ~ 12 encoder ticks)
#define thresh_oscillate 48 // determines whether to trim servo speeds by 1 or 2
#define flip_range 227 // encoder counts for calibrating for ~20 deg. motion
#define motor_stop_delay 20 // time for wires to clear or cross-talk
#define flip_duration 40 // indirectly controls range of flipper motion
#define servo_trim 1 // how much to adjust servo speed to correct for flipper centering
#define servo_bound 6 // max difference between forward and backward servo speeds
#define joy_pot_max 255 // maximum value of joystick output
#define deadband 20 // region that the joystick might rest
#define extra_dead 125 // mostly for right joystick to separate roll and pitch
#define RP_deadband 50 // region where the right joystick begins to work
#define yaw_surf_speed 30 // wheel rotation speed when changing yaw on a drive surface
#define step_tolerance 150 //Sets the threshold for flipper step function to determine acceptable flipper position error
#define timeout 300 // Number of allowable steps for flipper zeroing before step_through() exits while loop

////////////////////////////// VARIABLE DECLARATIONS //////////////////////////////
//SoftwareSerial motorSerial = SoftwareSerial(rxPin, txPin);
Servo motor1;
Servo motor2;
Servo motor3;
Servo BDCM;

char incoming[9]; // this is for storing incoming controller data
/*  
 * 0. 125 // make sure that the first thing received is always 125
   1. Left joystick, horizontal
   2. Left joystick, vertical
   3. Right joystick, horizontal
   4. Right joystick, vertical
   5. D-pad, horizontal
   6. D-pad, vertical
   7. A button
   8. B button
*/
unsigned int scaled_speed = max_speed * .5; // CHANGE SPEED HERE FOR DIFFERENT TESTS
unsigned int half_speed = max_speed >> 1; // max_speed/2
unsigned int flip_speed = 45; //  max_speed/2
unsigned int target_center = 2063;
unsigned char delta = 150; // this defines the range of motion (originally 150)//
unsigned int target;
unsigned int flip_pos;
unsigned int zero_pos1 = 0;
unsigned int zero_pos2 = 0;
unsigned int zero_pos3 = 0;


// the following track what driving mode the robot it currently in
boolean roll_state = false;
boolean pitch_state = false;
boolean yaw_state = false;
boolean wiggle_state = false; // if current state requires flipper motion
boolean m1_state = true;      // track last direction for motor for controlling oscillation
boolean m2_state = true;
boolean m3_state = true;
boolean surf_state = false;
boolean swim_state = false;
boolean freewater_state = true; // assuming the robot starts just in the water
boolean cal_mode = false; //  Becomes 'true' when "A" button is pressed, calibrating flippers
boolean flip_state = false; // mostly for just testing one
boolean A_toggle = false;
boolean B_toggle = false;
boolean enc1_reverse = false;
boolean enc2_reverse = false;
boolean enc3_reverse = false;

unsigned int loop_count = 0;
const float cos_60 = cos(60 * PI / 180);
const float sin_60 = sin(60 * PI / 180);
struct all_flip_pos {
  unsigned int m1;
  unsigned int m2;
  unsigned int m3;
};
struct flip_arcspeed { // units are milliseconds per encoder count
  float forward;
  float backward;
};
struct motor_speeds {
  unsigned int m1;
  unsigned int m2;
  unsigned int m3;
};
motor_speeds rough_speeds; // this is for storing all of the motor box speeds
flip_arcspeed m1_times; // this is for one motorbox
flip_arcspeed m2_times;
flip_arcspeed m3_times;
struct flip_servo_speed {
  unsigned char forward; // add to 90 for Servo.write()
  unsigned char backward; // subtract from 90 for Servo.write()
};

flip_servo_speed m1_servo;
flip_servo_speed m2_servo;
flip_servo_speed m3_servo;

struct all_flip_time {
  // all_flip_time is used as the return for calibrating all flippers at once
  flip_arcspeed m1;
  flip_arcspeed m2;
  flip_arcspeed m3;
};
struct dir_dist {
  char dir; // direction to rotate motor for shortest path to a target
  unsigned int dist; // shortest distance to target (in encoder counts)
};
struct vector_2 {
  char x_dir;
  char y_dir;
};

////////////////////////////// PROTOTYPE FUNCTIONS //////////////////////////////
char find_dir (unsigned char motor_num, unsigned int flip_pos, unsigned int target);
boolean cmd_pololu (unsigned char motor_num, char dir, unsigned char mag_speed);
boolean first_read (unsigned char encoder_num);
unsigned int read_position (unsigned char encoder_num);
//all_flip_pos new_zero();
all_flip_pos read_all(); // reads all flipper positions
flip_arcspeed cal_flipper (unsigned char box_num, unsigned int range);
all_flip_time cal_all_flip (unsigned char range);
unsigned int flipper_reset(unsigned char motor_num);
unsigned char enc_num_for_motor (unsigned char m_num);
boolean choppy_set_pos_one (unsigned char m_box, unsigned int target);
// chopp_set_pos_one sets one flipper to a target
boolean close_enough (unsigned int place, unsigned int target);
// returns TRUE if abs(place - target) < thresh
boolean stop_all(); // stops all motors
void wigglewaggle(unsigned char motor_num, unsigned char times);
// moves flipper back and forth for up to 255 times
boolean z_thrust (char dir);
boolean oscillate_1_correction (unsigned char motor_num);
// oscillates one flipper about a target (don't use this one)
dir_dist find_dir_dist (unsigned int flip_pos, unsigned int target);
void bound_times(); // don't use this one either
boolean oscillate_one (unsigned char motor_num, unsigned char num_flips, unsigned char thrust);
// don't use this one
boolean rough_set_pos_one (unsigned char m_box, unsigned int target);
// uses time calculation to get flipper towards target location
boolean set_pos_one_fast (unsigned char m_box, unsigned int target, unsigned char fast);
// uses time calc. to get flipper toward target location. can adjust speed
boolean one_oscillate (unsigned char motor_num, unsigned int target);
// one_onscillate is the best for oscillating one flipper starting from a target location
boolean all_oscillate (unsigned int target[3]);
// based on one_oscillate to flipp all flippers
flip_servo_speed bound_speeds(flip_servo_speed passed_in);
// use this to ensure that the motor box servo speeds don't get to far apart
boolean planar_swim (vector_2 target);
boolean surf_drive (vector_2 target);
boolean yaw (boolean dir); // rotate robot about vertical axis
boolean pitch (boolean rot_dir);
boolean roll (boolean rol_dir); // can implement rate later by adjusting flipper speeds
boolean reverse_enc (unsigned char motor_num);
// determines if the encoders are reversed on motor_num. sets enc#_reverse booleans
boolean rough_cal_all ();

////////////////////////////// SETUP //////////////////////////////
void setup() {
  Serial.begin(serial_baud); // communication to Arduino Serial Monitor
  Serial.println("Program start.");
  //motorSerial.begin(motor_baud); // communication to motor drivers
  //Serial1.begin(motor_baud);

  motor1.attach(m1_PWM);
  motor2.attach(m2_PWM);
  motor3.attach(m3_PWM);
  BDCM.attach(BDCM_PWM);
  pinMode(18, OUTPUT);
  pinMode(19, INPUT);
  pinMode(encoder_1, OUTPUT);
  pinMode(encoder_2, OUTPUT);
  pinMode(encoder_3, OUTPUT);
  pinMode(SS_pin, OUTPUT);
  digitalWrite(encoder_1, HIGH);
  digitalWrite(encoder_2, HIGH);
  digitalWrite(encoder_3, HIGH);
  pinMode(MOSI_pin, OUTPUT);
  pinMode(MISO_pin, INPUT);
  pinMode(SPI_SCK, OUTPUT);

  reset_servo_speeds();

  // target = target_center + delta;
   read_position(encoder_1); // first read usually garbage
   read_position(encoder_2);
   read_position(encoder_3);
  // flip_pos = read_position(encoder_1); // second read gives actual position
  // flip_pos = read_position(encoder_2);
  // flip_pos = read_position(encoder_3);

  // initialize brushless DC motor (BDCM) electronic speed controller (ESC)
  Serial.println("Initializing ESC (wait for 2 seconds)");
  BDCM.write(91); // set to center position

  cmd_pololu(1, 0, 0); // initialize PWM comm to pololus
  cmd_pololu(2, 0, 0); // initialize PWM comm to pololus
  cmd_pololu(3, 0, 0); // initialize PWM comm to pololus
  delay(2000); // allow pololus and ESC to initialize

  // figure out if any of the encoders are  reversed
  reverse_enc(1);
  reverse_enc(2);
  reverse_enc(3);
  rough_cal_all();
  /*
    // need to "configure" flipper timing for all flippers
    // befor configuration, try to get key centered in wheel
    // wigglewaggle(1, 2); // MAY NEED FOR LATER
    flip_arcspeed temp1, temp2, temp3;
    boolean good_cal = false;
    while (!(good_cal))
      temp1 = cal_flipper(1, flip_range); // find speed^-1 of motor 1
      if ((temp1.forward < 0.3) && (temp1.backward < 0.3)) good_cal = true;
    }
    good_cal = false;
    while (!(good_cal)) {
      temp2 = cal_flipper(1, flip_range); // find speed^-1 of motor 1
      if ((temp2.forward < 0.3) && (temp2.backward < 0.3)) good_cal = true;
    }
    good_cal = false;
    while (!(good_cal)) {
      temp3 = cal_flipper(1, flip_range); // find speed^-1 of motor 1
      if ((temp3.forward < 0.3) && (temp3.backward < 0.3)) good_cal = true;
    }

    m1_times.forward = (temp1.forward + temp2.forward + temp3.forward) / 3;
    m1_times.backward = (temp1.backward + temp2.backward + temp3.backward) / 3;
    // for now, base all speed calucaltions on one motor box
    m2_times.forward = m1_times.forward;
    m2_times.backward = m1_times.backward;
    m3_times.forward = m1_times.forward;
    m3_times.backward = m1_times.backward;
    delay(1000); // want to see calibration values
  */
  //choppy_set_pos_one(1, 2048);
  Serial.println("Initial Leg Positions:");
  Serial.print(read_pos_corrected(encoder_1,zero_pos1), DEC);
  Serial.print("  ");
  Serial.print(read_pos_corrected(encoder_2,zero_pos2), DEC);
  Serial.print("  ");
  Serial.println(read_pos_corrected(encoder_3,zero_pos3), DEC);
  Serial.println("Begin Main Loop.");
}

////////////////////////////// MAIN LOOP //////////////////////////////
void loop () {
  state_machine();
}
//////////////////////////// SUB-ROUTINES /////////////////////////////
boolean state_machine () {
loop_top:
  /*
    Serial.print(", enc: ");
    Serial.println(read_position(encoder_1), DEC);
  */
    loop_count ++;
    delay(10);
  
  /*
    PID_set_pos_one(2, 1024);
    delay(2500);
    PID_set_pos_one(2, 3072);
    delay(2500);
  */
  /*
    Potential program flow:
      read the serial input for controller changes continuously
      when one of the analog readings is high enough, run the appropriate
      subroutine and switch that state to true
  */

  // repeatedly check for updates from the controller (through Python and Serial)
  if (Serial.available() > 8) {
    for (unsigned char i = 0; i < 9; i++) {
      incoming[i] = Serial.read();
      // [125, leftHor_pos, leftVert_pos, rightHor_pos, rightVert_pos, dPad[0], dPad[1], A_button, B_button]
      // verticals are inverted, down is positive
    }
    // Debugging output

//    Serial.print("Debug number: ");
//    Serial.print(incoming[0]);
    Serial.print("Loop number: ");
    Serial.print(loop_count);
    Serial.print(", ");
    Serial.print("Left Joy: ");
    Serial.print(incoming[1], DEC);
    Serial.print(", ");
    Serial.print(incoming[2], DEC);
    Serial.print("| Right Joy: ");
    Serial.print(incoming[3], DEC);
    Serial.print(", ");
    Serial.print(incoming[4], DEC);
    Serial.print("| dPad_Hor: ");
    Serial.print(incoming[5], DEC);
    Serial.print(", ");
    Serial.print(incoming[6], DEC);
    Serial.print("| A: ");
    Serial.print(incoming[7], DEC);
    Serial.print("| B: ");
    Serial.println(incoming[8], DEC);

    if (incoming[0] != 125) {
      while (Serial.available()) {
        Serial.read();
      }
      // if the first value is not 125 as expected, dump the rest of the serial buffer
      // skip the rest of the following steps and return to top of the loop
      goto loop_top;
    }

    if ((abs(incoming[1]) > deadband) || (abs(incoming[2]) > deadband)) {
      // left joystick is being used to do planar motion
      if (freewater_state) {
        vector_2 temp;
        temp.x_dir = incoming[1];
        temp.y_dir = -incoming[2];
        Serial.println("planar_swim");
        planar_swim(temp);
      }
      else {
        vector_2 temp;
        temp.x_dir = incoming[1];
        temp.y_dir = -incoming[2];

        Serial.println("surf_drive");
        surf_drive(temp);
      }
    }

    if ((abs(incoming[3]) > extra_dead) && (freewater_state)) {
      // vertical of right joystick used to adjust pitch
      boolean dir;
      if (incoming[3] > 0) {
        dir = true;
        Serial.print("Clockwise ");
      }
      else {
        dir = false;
        Serial.print("Counter-clockwise ");
      }
      Serial.println("roll");
      roll(dir);
      roll_state = true;
    }
    if ((abs(incoming[4]) > extra_dead) && (freewater_state)) {
      // vertical of right joystick used to adjust pitch
      boolean dir;
      if (incoming[4] < 0) {
        // inverse vertical needs to be reversed
        Serial.println("Pitch up");
        dir = true;
      }
      else {
        Serial.println("Pitch down");
        dir = false;
      }
      pitch(dir);
      pitch_state = true;
    }

    if (abs(incoming[5]) > 0) {
      // horizontal d-pad is used for yaw
      boolean dir;
      if (incoming[5] < 0) {
        dir = true;
        //Serial.println("Counter-clockwise yaw");
      }
      else {
        //Serial.println("Clockwise yaw");
        dir = false;
      }
      if (freewater_state) {
        yaw(dir);
        yaw_state = true;
      }
      else {
        yaw_surf(dir);
      }
    }
    if ((abs(incoming[6]) > 0) && (freewater_state)) {
      // vertical d-pad is used for controlling the center propeller
      if (incoming[6] > 0) {
        z_thrust(-1);
        Serial.println("Up");
      }
      else {
        z_thrust(+1);
        Serial.println("Down");
      }
      // z_thrust(dir);
    }
    if ((abs(incoming[6]) == 0) && (freewater_state)) {
      // vertical d-pad is used for controlling the center propeller
      z_thrust(0);
    }

    if (incoming[7] == 1) {
      //A button is used to enter flipper calibration mode.  
      //Flippers will return to the '0' encoder position to ensure proper alignment
      A_toggle = true;
      Serial.println("A pressed.");
    }
    if (incoming[7] ==0 && A_toggle) {
      //A button is released
      cal_mode = !cal_mode;
      A_toggle = false;
      if (cal_mode){
        Serial.println("cal_mode on");
        zero_pos1 = flipper_reset(1);
        zero_pos2 = flipper_reset(2);
        zero_pos3 = flipper_reset(3);
        Serial.println("New zero positions: ");
        Serial.println(zero_pos1,DEC);
        Serial.println(zero_pos2,DEC);
        Serial.println(zero_pos3,DEC);
        }
      }

    if (incoming[8] == 1) {
      // B button is used to toggle between freewater and surface driving
      B_toggle = true;
        Serial.println("New zero positions: ");
        Serial.println(zero_pos1,DEC);
        Serial.println(zero_pos2,DEC);
        Serial.println(zero_pos3,DEC);
      // Serial.println("B pressed.");
    }
    if ((incoming[8] == 0) && B_toggle) {
      // if button B is released
      freewater_state = !freewater_state; // switch between freewater and surface modes
      B_toggle = false;

      if (freewater_state) Serial.println("Freewater mode");
      else {
        Serial.println("Surface drive mode");
        z_thrust(+1);
      }
    }
    if ((abs(incoming[1]) < deadband) && (abs(incoming[2]) < deadband) && (abs(incoming[3]) < deadband) && (abs(incoming[4]) < deadband) && (incoming[5] == 0) && (incoming[6] == 0)) {
      // if all josytick inputs and witihn deadband and none of d-pads are pressed
      //Serial.println("No motion");
      stop_all();
      if (freewater_state || !cal_mode){
      z_thrust(0);
      }
      yaw_state = false;
      roll_state = false;
      pitch_state = false;
      surf_state = false;
      swim_state = false;
      wiggle_state = false;
    }
  }
  if ((abs(incoming[5]) > 0) && freewater_state) {
    // horizontal d-pad is used for yaw
    boolean dir;
    if (incoming[5] < 0) {
      dir = true;
      //Serial.println("Counter-clockwise yaw");
    }
    else {
      //Serial.println("Clockwise yaw");
      dir = false;
    }
    if (freewater_state) {
      yaw(dir);
    }
    else {
      yaw_surf(dir);
    }
  }
}


//////////////////////Original above, new below/////////////

///////////////////////////////////////////////////////////////////////////////////////////
char find_dir (unsigned char motor_num, unsigned int flip_pos, unsigned int target) {
  /* gives: -1 or +1 depending on the closest distance to the target
    takes: 1. motor number
        2. flipper position
        3. target
  */
  unsigned dir = 0;
  if ((target > flip_pos) && (target - flip_pos < 2048)) dir = 1;
  else if ((target > flip_pos) && (target - flip_pos > 2048)) dir = -1;
  else if ((target < flip_pos) && (flip_pos - target < 2048)) dir = -1;
  else if ((target < flip_pos) && (flip_pos - target > 2048)) dir = 1;
  else dir = 0;
  //if (dir == 0) Serial.println("On target");

  return dir;
}
///////////////////////////////////////////////////////////////////////////////////////////
boolean cmd_pololu (unsigned char motor_num, char dir, unsigned char mag_speed) {
  /* gives: 1. true if successfully sent motor driving data
     takes: 1. motor box number
            2. direction, see below for directions
            3. speed or braking amount

     dir = 1 for forward
     dir = -1 for reverse
     dir = 0 for brake
  */
  /*
    if ((motor_num > 3) || (motor_num < 1)) {
    Serial.println("Invalid motor box number.");
    return false;
    }
  */
  if (mag_speed > max_speed) {
    Serial.print("Motor ");
    Serial.print(motor_num, DEC);
    //Serial.println(" exceeds max speed");
    Serial.println(" max speed");
    mag_speed = max_speed;
  }
  if (dir == 1) {
    // if forward
    //Serial.println("Forward.");
    switch (motor_num) {
      case 1:
        motor1.write(90 + mag_speed);
        m1_state = true;
        break;
      case 2:
        motor2.write(90 + mag_speed);
        m2_state = true;
        break;
      case 3:
        motor3.write(90 + mag_speed);
        m3_state = true;
        break;
      default:
        Serial.println("Invalid motor box number.");
        return false;
        break;
    }
  }
  else if (dir == -1) {
    // if reverse
    //Serial.println("Reverse.");
    switch (motor_num) {
      case 1:
        motor1.write(90 - mag_speed);
        m1_state = false;
        break;
      case 2:
        motor2.write(90 - mag_speed);
        m2_state = false;
        break;
      case 3:
        motor3.write(90 - mag_speed);
        m3_state = false;
        break;
      default:
        Serial.println("Invalid motor box number.");
        return false;
        break;
    }
  }
  else if (dir == 0) {
    // if braking
    switch (motor_num) {
      case 1:
        motor1.write(90);
        break;
      case 2:
        motor2.write(90);
        break;
      case 3:
        motor3.write(90);
        break;
      default:
        Serial.println("Invalid motor box number.");
        return false;
        break;
    }
    //Serial1.print(cmd_bytes);
    /*
      motorSerial.print(cmd_bytes[0]);
      motorSerial.print(cmd_bytes[1]);
      motorSerial.print(cmd_bytes[2]);
      motorSerial.print(cmd_bytes[3]);
    */
    return true;
  }
  else {
    Serial.println("Invalid direction.");
    return false;
  }
}
///////////////////////////////////////////////////////////////////////////////////////////
boolean first_read (unsigned char encoder_num) {
  // before running rest of program, start an encoder position read request
  SPI.begin();
  SPI.beginTransaction(SPISettings(SPI_speed, MSBFIRST, SPI_MODE0));

  digitalWrite(encoder_num, LOW); // pull select pin LOW to indicate start of transmission
  SPI.transfer(0x10); // send position request
  digitalWrite(encoder_num, HIGH); // pull select pin HIGH to indicate end of transmission

  SPI.end();
  return true;
}
///////////////////////////////////////////////////////////////////////////////////////////
unsigned int flipper_reset(unsigned char motor_num){
  unsigned char encoders[3] = {encoder_1,encoder_2,encoder_3};
  unsigned char encoder_num = encoders[motor_num-1];
  unsigned int zero_arr[3] = {zero_pos1,zero_pos2,zero_pos3};
  stop_all();
  Serial.println("Beginning Flipper Reset");
  Serial.print("Leg ");
  Serial.print(motor_num,DEC);
  Serial.print(" Pos: ");
  Serial.println(read_pos_corrected(encoder_1,zero_arr[motor_num-1]), DEC);
//  wigglewaggle(motor_num,30);  //Wiggle flipper 30 times
  step_through(motor_num,encoder_num,4096);
  
  // 10 second delay for manual adjustment
  Serial.println("Manually Adjust Flippers to Zero Position.");
  for (unsigned char k = 0; k<6; k++){ 
    Serial.print((6-k),DEC);
    Serial.println(" seconds remaining.");
    delay(1000);
  }
  unsigned int m_pos = read_pos_corrected(encoder_num,zero_arr[motor_num-1]);
  
  Serial.println("Final Flipper Pos");
  Serial.print("Leg 1 Pos: ");
  Serial.println(read_pos_corrected(encoder_1,zero_pos1), DEC);
  Serial.print("Leg 2 Pos: ");
  Serial.println(read_pos_corrected(encoder_2,zero_pos2), DEC);
  Serial.print("Leg 3 Pos: ");
  Serial.println(read_pos_corrected(encoder_3,zero_pos3), DEC);
  return m_pos;
}
///////////////////////////////////////////////////////////////////////////////////////////
void step_through (unsigned char motor_num, unsigned char encoder_num, unsigned int target){
//  unsigned char encoders[3] = {encoder_1,encoder_2,encoder_3};
//  unsigned char encoder_num = encoders[motor_num-1];
  unsigned int zero_arr[3] = {zero_pos1,zero_pos2,zero_pos3};
  double motor_speeds[3] = {0.35,0.5,0.4};
  bool loop = true;
  unsigned char x = 0;
  // Section to determine speed and set speed constants for each motor
  //  Takes 1 step of known duration and measures encoder differences to determine speed coeffecient (enc1 - enc0)/duration = rough speed
  while (loop){
    if (x > 300){
      Serial.println("Timeout error.");
      loop = false;
    }
    unsigned int enc = read_pos_corrected(encoder_num,zero_arr[motor_num-1]);
    if ((abs(enc - target) < step_tolerance) || (enc - 4096 + target) < step_tolerance){
      stop_all();
      Serial.print("Leg ");
      Serial.print(motor_num,DEC);
      Serial.print(" stopped at ");
      Serial.print(enc,DEC);
      Serial.print("     Target = ");
      Serial.println(target,DEC);
      loop = false;
    }
    cmd_pololu (motor_num, 1, motor_speeds[motor_num-1]*flip_speed); // drive motor forward/backward
    delay(motor_stop_delay/5); // for a certain number of milliseconds
    cmd_pololu (motor_num, 0, 0); // stop motor
    delay(1.5*motor_stop_delay); // wait to make sure there's no cross-interference & motor fully stopped
    x++;
  }
}
///////////////////////////////////////////////////////////////////////////////////////////
unsigned int read_position(unsigned char encoder_num) {
  /* Command 0x10 rd_pos (read position)
     1. Master sends rd_pos command, encoder responds with idle chars
     2. Continue sending nop_a5 command while encoder response is 0xA5
     3. If response was 0x10 (rd_pos), send nop_a5 and receive MSB position
        lower 4 bits of this byte are the upper 4 of the 12-bit position
     4. Send second nop_a5 cmomand and receive LSB position
  */
  /*
    Serial.print("Begin read_position for encoder_num: ");
    Serial.println(encoder_num, DEC);
  */

  // check to see if valid encoder number
  switch (encoder_num) {
    case encoder_1 :
      // do nothing and exit switch
      break;
    case encoder_2 :
      break;
    case encoder_3 :
      break;
    default :
      // if encoder_num does not maatch any of the encoder values
      Serial.println("Invalid encoder number.");
      return 9999;
  }

  // request a read
  first_read(encoder_num);

  SPI.begin();
  SPI.beginTransaction(SPISettings(SPI_speed, MSBFIRST, SPI_MODE0));

  unsigned char data_chunks[3] = {0, 0, 0};
  // get the first byte
  digitalWrite(encoder_num, LOW);
  data_chunks[0] = SPI.transfer(0x00); // send blank byte and check response
  digitalWrite(encoder_num, HIGH);
  //delay(SPI_wait);
  /*
    // debugging output
    Serial.println("Sent position request.");
    Serial.print("1-R: ");
    Serial.println(data_chunks[0]);
  */
  if (data_chunks[0] = 0x80) {

  }
  else {
    //while (data_chunks[0] == 0xA5) {
    unsigned char retry_count = 0;
    while (data_chunks[0] != 0x80) {
      // Serial.println("Send waiting byte.");
      digitalWrite(encoder_num, LOW);
      data_chunks[0] = SPI.transfer(0); // keep sending 0 until receive 0x10 response
      digitalWrite(encoder_num, HIGH);
      // delay(SPI_wait);
      // debugging output
      // Serial.println("wait");
      retry_count ++;
      if (retry_count > 3) {
        //Serial.println("Retry position request.");
        first_read(encoder_num);
      }
    }
  }

  // collect the two bytes of information
  digitalWrite(encoder_num, LOW);
  data_chunks[0] = SPI.transfer(0x00);
  digitalWrite(encoder_num, HIGH);
  //delay(SPI_wait);
  digitalWrite(encoder_num, LOW);
  data_chunks[1] = SPI.transfer(0x00);
  digitalWrite(encoder_num, HIGH);
  // delay(SPI_wait);

  // clear the register
  while (data_chunks[2] != 0xA5) {
    digitalWrite(encoder_num, LOW);
    data_chunks[2] = SPI.transfer(0); // keep sending 0 until receive 0x10 response
    digitalWrite(encoder_num, HIGH);
    // delay(SPI_wait);
    // debugging output
    // Serial.println(data_chunks[2]);
  }

  // debugging output

  unsigned int data = data_chunks[0] << 8 | data_chunks[1];
  /*
    Serial.print("e");
    Serial.print(encoder_num);
    Serial.print(": ");
    Serial.println(data);
  */
  /* no longer need to start next request, because sparse reading
    digitalWrite(encoder_num, LOW);
    SPI.transfer(0x10); // send next position request before leaving subroutine
    digitalWrite(encoder_num, HIGH);
  */
  SPI.end();
  if (data > 4096) {
    // max reading of encoder is 4096, so if data > 4096, must be garbage
    data = read_position (encoder_num); // read again
  }

  // flip the encoder reading if the encoders are backwards
  if ((encoder_num == encoder_1) && enc1_reverse) {
    data = 4096 - data;
  }
  if ((encoder_num == encoder_2) && enc2_reverse) {
    data = 4096 - data;
  }
  if ((encoder_num == encoder_3) && enc3_reverse) {
    data = 4096 - data;
  }

  return data;
}
///////////////////////////////////////////////////////////////////////////////////////////
unsigned int read_pos_corrected (unsigned char encoder_num, unsigned int zero_position) {
  // ~10% of the encoder readings are wrong, often the reading is 4096 minus the proper position
  // 4 samples seems to be good enough if we just toss the highest and lowest readings
  //Serial.println("Begin read_pos_corrected");
  unsigned int enc_samples[3];
  for (unsigned char i = 0; i < 3; i++) {
    enc_samples[i] = read_position(encoder_num);
    delay(2);
  }
  // need to sort array from smallest to largest to find median
  for (int i = 0; i < 5; i++) {
    for (int o = 0; o < 2; o++) {
      if (enc_samples[o] > enc_samples[o + 1]) {
        int t = enc_samples[o] - zero_position;
        enc_samples[o] = enc_samples[o + 1];
        enc_samples[o + 1] = t;
      }
    }
  }
  /*                            
    // debugging output
    Serial.print("Encoder readings: ");
    Serial.print(enc_samples[0]);
    Serial.print(", ");
    Serial.print(enc_samples[1]);
    Serial.print(", ");
    Serial.println(enc_samples[2]);
  */
  return enc_samples[1];
}
//////////////////////////////////////////////////////////////////////////////////////////
//all_flip_pos new_zero() {
//  //Takes current encoder readings and makes necessary adjustment to set current reading as 0 position 
//  all_flip_pos temp;
//  temp.m1 = read_pos_corrected(encoder_1);
//  temp.m2 = read_pos_corrected(encoder_2);
//  temp.m3 = read_pos_corrected(encoder_3);
//  return temp;
//}
///////////////////////////////////////////////////////////////////////////////////////////
all_flip_pos read_all() {
  /* gives: positions of all the flippers
     takes: nothing
     NOT COMPLETE YET
  */
  all_flip_pos temp;
  temp.m1 = read_pos_corrected(encoder_1,0);
  temp.m2 = read_pos_corrected(encoder_2,0);
  temp.m3 = read_pos_corrected(encoder_3,0);
  /*
    // debugging output
    Serial.print("e1: ");
    Serial.println(temp.m1, DEC);
    Serial.print("e2: ");
    Serial.println(temp.m2, DEC);
    Serial.print("e3: ");
    Serial.println(temp.m3, DEC);
  */
  return temp;
}

///////////////////////////////////////////////////////////////////////////////////////////
flip_arcspeed cal_flipper (unsigned char box_num, unsigned int range) {
  /* gives: struct including forward and reverse runtime for specified flipper range
     takes: 1. which box number to calibrate
            2. travel range of flipper motion desired
  */

  /*
    loop_count = 0;
    while (loop_count < 10) {
    cmd_pololu(1, 1, flip_speed);
    delay(100);
    cmd_pololu(1, -1, flip_speed);
    delay(100);
    loop_count++;
    }
    stop_all();
    delay(1);
  */
  Serial.print("Calibration for box number: ");
  Serial.println(box_num, DEC);
  unsigned char duration = 50;
  unsigned char encoder_num = enc_num_for_motor(box_num);
  unsigned int flip_pos_start = read_position(encoder_num);
  dir_dist dir_dist_for;
  dir_dist dir_dist_bak;
  dir_dist_for.dir = 0;
  dir_dist_for.dist = 0;
  dir_dist_bak.dir = 0;
  dir_dist_bak.dist = 0;

  cmd_pololu (box_num, 1, flip_speed); // drive motor forward
  delay(duration); // for a certain number of milliseconds
  cmd_pololu (box_num, 0, 0); // stop motor
  delay(motor_stop_delay + 100); // wait to make sure there's no cross-interference & motor fully stopped
  unsigned int flip_pos_top = read_position(encoder_num);
  cmd_pololu (box_num, -1, flip_speed);
  delay(duration);
  cmd_pololu (box_num, 0, 0);
  delay(motor_stop_delay); // wait to make sure there's no cross-interference
  unsigned int flip_pos_end = read_position(encoder_num);

  //int forward_dist, backward_dist;
  dir_dist_for = find_dir_dist(flip_pos_start, flip_pos_top);
  dir_dist_bak = find_dir_dist(flip_pos_top, flip_pos_end);
  // forward_dist = abs(flip_pos_top - flip_pos_start);
  // if (forward_dist > 2048) forward_dist -= 2048;
  // backward_dist = abs(flip_pos_top - flip_pos_end);
  // if (backward_dist > 2048) backward_dist -= 2048;
  float f_time = duration / float(dir_dist_for.dist);
  float b_time = duration / float(dir_dist_bak.dist);

  // debugging output
  Serial.print("Forward time: ");
  Serial.println(f_time, DEC);
  Serial.print("Backward time: ");
  Serial.println(b_time, DEC);

  flip_arcspeed temp;
  temp.forward = f_time;
  temp.backward = b_time;
  return temp;
}

///////////////////////////////////////////////////////////////////////////////////////////
all_flip_time cal_all_flip (unsigned char range) {
  /*  gives: struct including forward and reverse runtime for specified flipper range
      takes: 1. which box number to calibrate
             2. travel range of flipper motion desired
  */
  unsigned char duration = 30;
  all_flip_pos flippyflip_start;
  all_flip_pos flippyflip_top;
  all_flip_pos flippyflip_end;

  flippyflip_start = read_all();

  cmd_pololu (1, 1, half_speed); // drive all motors forward
  cmd_pololu (2, 1, half_speed);
  cmd_pololu (3, 1, half_speed);
  delay(duration); // for a certain number of milliseconds
  cmd_pololu (1, 0, half_speed); // stop all motors
  cmd_pololu (2, 0, half_speed);
  cmd_pololu (3, 0, half_speed);
  delay(1); // wait to make sure there's not cross-interference

  flippyflip_top = read_all();

  cmd_pololu (1, -1, half_speed);
  cmd_pololu (2, -1, half_speed);
  cmd_pololu (3, -1, half_speed);
  delay(duration);
  cmd_pololu (1, 0, half_speed);
  cmd_pololu (2, 0, half_speed);
  cmd_pololu (3, 0, half_speed);
  delay(1); // wait to make sure there's not cross-interference

  flippyflip_end = read_all();

  unsigned char f1_time = duration * range / (flippyflip_top.m1 - flippyflip_start.m1);
  unsigned char b1_time = duration * range / (flippyflip_top.m1 - flippyflip_end.m1);
  unsigned char f2_time = duration * range / (flippyflip_top.m2 - flippyflip_start.m2);
  unsigned char b2_time = duration * range / (flippyflip_top.m2 - flippyflip_end.m2);
  unsigned char f3_time = duration * range / (flippyflip_top.m3 - flippyflip_start.m3);
  unsigned char b3_time = duration * range / (flippyflip_top.m3 - flippyflip_end.m3);

  // debugging output
  Serial.print("M1 forward time: ");
  Serial.println(f1_time, DEC);
  Serial.print("M1 backward time: ");
  Serial.println(b1_time, DEC);

  all_flip_time temp;
  temp.m1.forward = f1_time;
  temp.m1.backward = b1_time;
  temp.m2.forward = f2_time;
  temp.m2.backward = b2_time;
  temp.m3.forward = f3_time;
  temp.m3.backward = b3_time;
  return temp;
}

///////////////////////////////////////////////////////////////////////////////////////////
unsigned char enc_num_for_motor (unsigned char m_num) {
  /* gives: encoder number tied to the motor driver number
      takes: motor driver number
  */
  if (m_num == 1) {
    return encoder_1;
  }
  else if (m_num == 2) {
    return encoder_2;
  }
  else if (m_num == 3) {
    return encoder_3;
  }
  else {
    Serial.println("Invalid motor number.");
    return 0;
  }
}
///////////////////////////////////////////////////////////////////////////////////////////
//Nicewell version 6.9.2017
boolean choppy_set_pos_one(unsigned char m_box, unsigned int target) {
  /* gives: true if flipper positions are set within tolerance
  takes: desired target positions for all the flippers
  */
  Serial.print("Begin choppy_set_pos_one for motor ");
  Serial.print(m_box, DEC);
  Serial.print(" to ");
  Serial.println(target, DEC);
  cmd_pololu(m_box, 0, 0); // stop motor
  unsigned int current_pos = read_position(enc_num_for_motor(m_box));

    boolean m1_state = false, // for keeping track of which ones are moving
      m1_reach = false; // for keeping track of whether target is reached
                // flip_times times = cal_flip_time(target - current_pos)*find_dir(m_box, current_pos, target);
    dir_dist motor_cmds = find_dir_dist(current_pos, target);
    unsigned long expected_time;
    unsigned long start;
  
    while (!(m1_reach)) {
      cmd_pololu(m_box, 0, 0); // stop motor
      delay(20); // wait for wire interference to stop
      current_pos = read_position(enc_num_for_motor(m_box));
      if (close_enough(current_pos, target)) {
        Serial.print("Motor ");
        Serial.print(m_box, DEC);
        Serial.print(" final pose: ");
        Serial.println(current_pos, DEC);
        return true; // exit if close enough
      }
      // otherwise continue with following
      motor_cmds = find_dir_dist(current_pos, target);
      if (motor_cmds.dir > 0) expected_time = m1_times.forward * motor_cmds.dist;
      else expected_time = m1_times.backward * motor_cmds.dist;
      /*
      // Debugging output
      Serial.print("Direction: ");
      Serial.println(motor_cmds.dir, DEC);
      Serial.print("Distance: ");
      Serial.println(motor_cmds.dist, DEC);
      Serial.print("Expected travel time: ");
      Serial.println(expected_time, DEC);
      */
  //////////////////////////////////////////////EDIT THIS SECTION///////////////////////////////////////
      unsigned int slow_down_range = 500;
      unsigned char close_speed;
      if (motor_cmds.dist < slow_down_range) {
        close_speed = flip_speed - (float)motor_cmds.dist / 15;
  //      Serial.print("Scaled speed: ");
  //      Serial.println(close_speed, DEC);
      }
      else close_speed = flip_speed;
 
      start = millis();
      cmd_pololu(m_box, motor_cmds.dir, close_speed);
 ///////////////////////////////////////////////////////////////////////////////////////////////////////
      while (!(m1_state)) {
        // loop until time is up
        if (millis() - start > expected_time) m1_state = true;//|| expected_time == 0
        int temp = millis();
  //      Serial.println("Wait until target reached.\n");
  //      Serial.print("Expected time: ");
  //      Serial.println(expected_time, DEC);
  //      Serial.print("Current time: ");
  //      Serial.println(temp - start, DEC);
      }
      m1_state = false; // reset for the next loop around
    }
}
///////////////////////////////////////////////////////////////////////////////////////////
boolean PID_set_pos_one (unsigned char motor_num, unsigned int target) {
  unsigned int zero_arr[3] = {zero_pos1, zero_pos2, zero_pos3};
 // for M2 : speedy min = 7, scale factor = 0.068, max_PID = 40
  unsigned int current_pos = read_pos_corrected (enc_num_for_motor(motor_num), zero_arr[motor_num + 1]);
  
  
  Serial.print("current position: ");
  Serial.println(current_pos);
  Serial.print("target position: ");
  Serial.println(target);
  

  dir_dist displacement = find_dir_dist(current_pos, target);
  unsigned int speedy =0;
  while (displacement.dist > thresh) {
    // calculate the speed to the target here
    

    speedy = displacement.dist * 0.068;

    
    Serial.print("Speed: ");
    Serial.println(speedy);
    
    if (speedy <= 7) {
      speedy = 7;
    }
    else if (speedy >= max_PID) {
      speedy = max_PID;
    }

    Serial.print("Speed after adjustment: ");
    Serial.println(speedy);
    cmd_pololu(motor_num, displacement.dir, speedy);
    current_pos = read_pos_corrected (enc_num_for_motor(motor_num), zero_arr[motor_num + 1]);
    displacement = find_dir_dist(current_pos, target);
  }
  cmd_pololu(motor_num, 0, 0);

  Serial.print("Final error: ");
  Serial.println(displacement.dist);
  return true;
  
}
///////////////////////////////////////////////////////////////////////////////////////////
boolean close_enough (unsigned int place, unsigned int target) {
  /* gives: true if abs(place - target) < threshold
      takes: 1. current encoder reading
             2. desired encoder position
  */
  dir_dist temp = find_dir_dist(place, target);
  if (temp.dist < thresh) {
    return true;
  }
  else {
    return false;
  }
}
///////////////////////////////////////////////////////////////////////////////////////////
boolean stop_all() {
  // stops all the motor boxes but not the central prop
  cmd_pololu(1, 0, 0);
  cmd_pololu(2, 0, 0);
  cmd_pololu(3, 0, 0);
  //z_thrust(90);
}
///////////////////////////////////////////////////////////////////////////////////////////
void wigglewaggle(unsigned char motor_num, unsigned char times) {
  unsigned char count = 0;
  while (count < times) {
    cmd_pololu(motor_num, 1, flip_speed);
    delay(2*flip_duration);
    cmd_pololu(motor_num, -1, flip_speed);
    delay(2*flip_duration);
    count++;
  }
  stop_all();
  delay(1);
}
///////////////////////////////////////////////////////////////////////////////////////////
boolean z_thrust (char dir) {
  // gives: true if successfully completed subroutine
  // takes: + for going up, - for down, 0 for stopping
  if (dir > 0) BDCM.write(91 + BDCM_speed);
  else if (dir < 0) BDCM.write(91 - BDCM_speed);
  else BDCM.write(91);
}
///////////////////////////////////////////////////////////////////////////////////////////
boolean oscillate_1_correction (unsigned char motor_num) {
  // takes: motor box number
  // gives: TRUE when complete
  unsigned int start_pos = read_position(enc_num_for_motor(motor_num));
  cmd_pololu(motor_num, 1, (unsigned char)(3 / m1_times.forward));
  delay(flip_duration);
  cmd_pololu(motor_num, -1, (unsigned char)(3 / m1_times.backward));
  delay(flip_duration);
  cmd_pololu(motor_num, 0, 0);
  delay(motor_stop_delay);
  unsigned int end_pos = read_position(enc_num_for_motor(motor_num));
  if (end_pos < start_pos) {
    // reduce foward speed (add to speed^-1)
    m1_times.forward += 0.005;
    // increase backward speed (subtract from speed^-1)
    m1_times.backward -= 0.005;
    bound_times();
  }
  else {
    // increase foward speed (add to speed^-1)
    m1_times.forward -= 0.01;
    // reduce backward speed (subtract from speed^-1)
    m1_times.backward += 0.01;
    bound_times();
  }
}
///////////////////////////////////////////////////////////////////////////////////////////
dir_dist find_dir_dist (unsigned int flip_pos, unsigned int target) {
  // gives: nothing
  // takes: target location
  // prints through the serial interface the direction to turn and the distance to the target
  //unsigned int flip_pos = read_position(enc_num_for_motor(motor_num));
  /*
    Serial.print("Begin find_dir_dist, motor_num: ");
    Serial.print(motor_num, DEC);
    Serial.print(", flip_pos: ");
    Serial.print(flip_pos, DEC);
    Serial.print(", target: ");
    Serial.println(target, DEC);
  */
  dir_dist temp;
  temp.dir = 0;
  temp.dist = 0;
  if ((target > flip_pos) && (target - flip_pos < 2048)) {
    temp.dir = 1;
    temp.dist = target - flip_pos;
  }
  else if ((target > flip_pos) && (target - flip_pos > 2048)) {
    temp.dir = -1;
    temp.dist = flip_pos + 4096 - target;
  }
  else if ((target < flip_pos) && (flip_pos - target < 2048)) {
    temp.dir = -1;
    temp.dist = flip_pos - target;
  }
  else if ((target < flip_pos) && (flip_pos - target > 2048)) {
    temp.dir = 1;
    temp.dist = target + 4096 - flip_pos;
  }
  else {
    temp.dir = 0;
    //Serial.println("On target, direction is 0.");
  }
  /*
    Serial.print("Direction: ");
    Serial.println(temp.dir, DEC);

    Serial.print("Distance: ");
    Serial.println(temp.dist, DEC);
  */
  return temp;
}
///////////////////////////////////////////////////////////////////////////////////////////
void bound_times() {
  if (m1_times.forward > 0.2) m1_times.forward = 0.2;
  if (m2_times.backward > 0.2) m1_times.backward = 0.2;
}
///////////////////////////////////////////////////////////////////////////////////////////
boolean oscillate_one (unsigned char motor_num, unsigned char num_flips, unsigned char thrust) {
  // gives: TRUE when complete
  // takes: 1. target center location of the flipping motion
  //        2. a value from 0 to 90 for the speed of oscillation
  // this routine oscillates a flipper based on TIME, not live encoder readings

  // make sure thrust is within bounds
  if (thrust > 90) {
    Serial.println("Thrust greater than 90.");
    return false;
  }

  // original intended flip duration is 60ms at the calibration speed of flip_speed
  unsigned int flip_time = 60 * flip_speed / thrust;
  Serial.print("Oscillate duration: ");
  Serial.println(flip_time, DEC);
  //delay(2000);
  for (unsigned char i = 0; i < num_flips; i++) {
    unsigned long t_begin = millis();
    cmd_pololu(motor_num, 1, thrust);
    while (millis() - t_begin < flip_time) {
      //Serial.println("FORWARD FLIPPING!!!");
    }
    t_begin = millis();
    cmd_pololu(motor_num, -1, thrust);
    while (millis() - t_begin < flip_time) {
      //Serial.println("BACKWARD FLIPPING!!!");
    }
  }
  cmd_pololu(motor_num, 0, 0);
  return true;
}
///////////////////////////////////////////////////////////////////////////////////////////
boolean oscillate_one_live(unsigned char motor_num, unsigned int target) {
  // this is a bang-bang controller for oscillating one flipper with live encoder readings
  unsigned int current_pos = read_position (enc_num_for_motor(motor_num));
  //unsigned int target = flip_center;
  /*
    Serial.print("Starting position: ");
    Serial.println(current_pos);
    Serial.print("Oscillating center: ");
    Serial.println(target);
  */
  cmd_pololu(motor_num, 1, 30);
  while (current_pos < target + flip_range / 2) {
    current_pos = read_position (enc_num_for_motor(motor_num));
  }
  cmd_pololu(motor_num, 0, 0);
  delay(1000);
  cmd_pololu(motor_num, -1, 30);
  while (current_pos > target - flip_range / 2) {
    current_pos = read_position (enc_num_for_motor(motor_num));
  }
  cmd_pololu(motor_num, 0, 0);
  delay(1000);
}
///////////////////////////////////////////////////////////////////////////////////////////
boolean rough_set_pos_one (unsigned char m_box, unsigned int target) {
  /* gives: true if flipper positions are set within tolerance
     takes: desired target positions for all the flippers
  */
  /*
    Serial.print("rough_set_pos_one to ");
    Serial.println(target, DEC);
  */
  cmd_pololu(m_box, 0, 0); // stop motor
  delay(motor_stop_delay);
  unsigned int current_pos = read_position(enc_num_for_motor(m_box));
  // flip_times times = cal_flip_time(target - current_pos)*find_dir(m_box, current_pos, target);
  dir_dist motor_cmds = find_dir_dist(current_pos, target);
  boolean m1_state = false;
  unsigned long expected_time;
  unsigned long start;

  // current_pos = read_position(enc_num_for_motor(m_box));
  if (close_enough(current_pos, target)) return true; // exit if close enough
  // otherwise continue with following
  motor_cmds = find_dir_dist(current_pos, target);
  /*
    if (motor_cmds.dir > 0) expected_time = m1_times.forward * motor_cmds.dist;
    else expected_time = m1_times.backward * motor_cmds.dist;
  */
  switch (m_box) {
    case 1:
      expected_time = motor_cmds.dist / rough_speeds.m1;
      break;
    case 2:
      expected_time = motor_cmds.dist / rough_speeds.m2;
      break;
    case 3:
      expected_time = motor_cmds.dist / rough_speeds.m3;
      break;
  }

  /*
    // Debugging output
    Serial.print("Direction: ");
    Serial.println(motor_cmds.dir, DEC);
    Serial.print("Distance: ");
    Serial.println(motor_cmds.dist, DEC);
  */
  Serial.print("Expected travel time: ");
  Serial.println(expected_time, DEC);

  start = millis();
  cmd_pololu(m_box, motor_cmds.dir, flip_speed);
  while (!(m1_state)) {
    // loop until time is up
    if (millis() - start > expected_time) m1_state = true;
    //Serial.println("Wait until target reached.");
  }
  cmd_pololu(m_box, 0, 0);
  return true;
}
///////////////////////////////////////////////////////////////////////////////////////////
boolean set_pos_one_fast (unsigned char m_box, unsigned int target, unsigned char fast) {
  /* gives: true if flipper positions are set within tolerance
     takes: 1. motor box number
            2. target location
            3. how fast (from 0 to 90)
  */
  /*
    Serial.print("set_pos_one_fast to ");
    Serial.print(target, DEC);
    Serial.print(", speed: ");
    Serial.println(fast, DEC);
  */
  cmd_pololu(m_box, 0, 0); // stop motor
  delay(motor_stop_delay);
  unsigned int current_pos = read_position(enc_num_for_motor(m_box));
  // flip_times times = cal_flip_time(target - current_pos)*find_dir(m_box, current_pos, target);
  dir_dist motor_cmds = find_dir_dist(current_pos, target);
  boolean m1_state = false;
  unsigned long expected_time;
  unsigned long start;

  if (close_enough(current_pos, target)) return true; // exit if close enough
  // otherwise continue with following
  motor_cmds = find_dir_dist(current_pos, target);
  if (motor_cmds.dir > 0)
    expected_time = (m1_times.forward * motor_cmds.dist * flip_speed / fast);
  else
    expected_time = (m1_times.backward * motor_cmds.dist * flip_speed / fast);
  // overshoot correction based on distance to target
  float scale = 0.02;
  //expected_time += (unsigned long)(motor_cmds.dist * scale);

  // usually seems to overshoot
  /*
    // Debugging output
    Serial.print("Direction: ");
    Serial.println(motor_cmds.dir, DEC);
    Serial.print("Distance: ");
    Serial.println(motor_cmds.dist, DEC);
    Serial.print("Expected travel time: ");
    Serial.println(expected_time, DEC);
  */
  start = millis();
  cmd_pololu(m_box, motor_cmds.dir, fast);
  while (!(m1_state)) {
    // loop until time is up
    if (millis() - start > expected_time) m1_state = true;
    //Serial.println("Wait until target reached.");
  }
  cmd_pololu(m_box, 0, 0);
  delay(motor_stop_delay);
  unsigned int temp_pos = read_position(enc_num_for_motor(m_box));
  int error = temp_pos - target;
  if (error < -1000) error += 4096;
  if (error > 1000) error -= 4096;
  Serial.print("set_pos_one_fast target: ");
  Serial.print(target, DEC);
  Serial.print(", error: ");
  Serial.println(error, DEC);
  return true;
}
///////////////////////////////////////////////////////////////////////////////////////////
boolean one_oscillate (unsigned char motor_num, unsigned int target) {
  /*gives: true when complete with subroutine
    takes: 1. which motor box to run
           2. the target start location of the flipper
    moves flipper forward then backward for a set time. adjusts speed afterwards to try
    get flipper to remain in the same bounds. the flipper should already be close
    to the target location before running subroutine
  */
  /*
    unsigned int start_pos = read_position(enc_num_for_motor(motor_num));
    if (!close_enough(start_pos, target)) {
    //choppy_set_pos_one(motor_num, target);
    //Serial.println("one_oscillate start position far from target");
    }
  */
  unsigned char f_speed, b_speed;
  switch (motor_num) {
    case 1:
      f_speed = m1_servo.forward;
      b_speed = m1_servo.backward;
      break;
    case 2:
      f_speed = m2_servo.forward;
      b_speed = m2_servo.backward;
      break;
    case 3:
      f_speed = m3_servo.forward;
      b_speed = m3_servo.backward;
      break;
    default:
      Serial.println("Invalid motor box number.");
      return false;
  }
  cmd_pololu(motor_num, 1, f_speed);
  delay(flip_duration);
  cmd_pololu(motor_num, 0, 0); // stop the motor
  delay(motor_stop_delay);
  cmd_pololu(motor_num, -1, b_speed);
  delay(flip_duration);
  cmd_pololu(motor_num, 0, 0); // stop the motor
  delay(motor_stop_delay);
  unsigned int end_pos = read_position(enc_num_for_motor(motor_num)); // find final pose
  /*
    Serial.print("Start: ");
    Serial.print(start_pos, DEC);
    Serial.print(", Stop: ");
    Serial.println(end_pos, DEC);
  */
  //dir_dist temp_dd = find_dir_dist(start_pos, end_pos);
  dir_dist temp_dd = find_dir_dist(target, end_pos);

  unsigned char loop_times;
  if (temp_dd.dist > thresh_oscillate) loop_times = 2;
  else loop_times = 1;

  for (unsigned char i = 0; i < loop_times; i++) {
    if (temp_dd.dir > 0) {
      switch (motor_num) {
        case 1:
          m1_servo.forward = f_speed - servo_trim;
          m1_servo.backward = b_speed + servo_trim;
          break;
        case 2:
          m2_servo.forward = f_speed - servo_trim;
          m2_servo.backward = b_speed + servo_trim;
          break;
        case 3:
          m3_servo.forward = f_speed - servo_trim;
          m3_servo.backward = b_speed + servo_trim;
          break;
      }
    }
    else if (temp_dd.dir < 0) {
      switch (motor_num) {
        case 1:
          m1_servo.forward = f_speed + servo_trim;
          m1_servo.backward = b_speed - servo_trim;
          break;
        case 2:
          m2_servo.forward = f_speed + servo_trim;
          m2_servo.backward = b_speed - servo_trim;
          break;
        case 3:
          m3_servo.forward = f_speed + servo_trim;
          m3_servo.backward = b_speed - servo_trim;
          break;
      }
    }
  }
  m1_servo = bound_speeds(m1_servo);
  /*
    Serial.print("f_speed: ");
    Serial.print(m1_servo.forward, DEC);
    Serial.print(", b_speed: ");
    Serial.println(m1_servo.backward, DEC);
  */
  return true;
}
///////////////////////////////////////////////////////////////////////////////////////////
boolean all_oscillate (unsigned int target[3]) {
  /*gives: true when complete with subroutine
    takes: 1. an array for all of the target locations
    moves flipper forward then backward for a set time. adjusts speed afterwards to try
    get flipper to remain in the same bounds. the flipper should already be close
    to the target location before running subroutine
  */
  // Serial.println("Begin all_oscillate");
  Serial.println("Starting Leg Positions:");
  Serial.print(read_position(encoder_1), DEC);
  Serial.print("  ");
  Serial.print(read_position(encoder_2), DEC);
  Serial.print("  ");
  Serial.println(read_position(encoder_3), DEC);
    cmd_pololu(1, 1, m1_servo.forward);
    cmd_pololu(2, 1, m2_servo.forward);
    cmd_pololu(3, 1, m3_servo.forward);
    delay(flip_duration);
    stop_all(); // stop the motor
    delay(motor_stop_delay);
    cmd_pololu(1, -1, m1_servo.backward);
    cmd_pololu(2, -1, m2_servo.backward);
    cmd_pololu(3, -1, m3_servo.backward);
    delay(flip_duration);
    stop_all(); // stop all the motor boxes
    delay(motor_stop_delay); // wait until interference gone
  Serial.println("Ending Leg Positions:");
  Serial.print(read_position(encoder_1), DEC);
  Serial.print("  ");
  Serial.print(read_position(encoder_2), DEC);
  Serial.print("  ");
  Serial.println(read_position(encoder_3), DEC);
    all_flip_pos all_end_pos = read_all(); // find final pose
    /*
      Serial.print("Stop: ");
      Serial.println(end_pos, DEC);
    */
  
    // dir_dist temp_dd = find_dir_dist(start_pos, end_pos);
    dir_dist temp_dd[3];
    temp_dd[0] = find_dir_dist(target[0], all_end_pos.m1);
    temp_dd[1] = find_dir_dist(target[1], all_end_pos.m2);
    temp_dd[2] = find_dir_dist(target[2], all_end_pos.m3);
  
    for (unsigned char j = 0; j < 3; j++) {
      unsigned char loop_times;
      if (temp_dd[j].dist > thresh_oscillate) loop_times = 2;
      else if  (temp_dd[j].dist > thresh) loop_times = 1;
      else loop_times = 0;
  
      for (unsigned char i = 0; i < loop_times; i++) {
        if (temp_dd[j].dir > 0) {
          switch (j) {
            case 0:
              m1_servo.forward -= servo_trim;
              m1_servo.backward += servo_trim;
              break;
            case 1 :
              m2_servo.forward -= servo_trim;
              m2_servo.backward += servo_trim;
              break;
            case 2:
              m3_servo.forward -= servo_trim;
              m3_servo.backward += servo_trim;
              break;
          }
        }
        else if (temp_dd[j].dir < 0) {
          switch (j) {
            case 0:
              m1_servo.forward += servo_trim;
              m1_servo.backward -= servo_trim;
              break;
            case 1:
              m2_servo.forward += servo_trim;
              m2_servo.backward -= servo_trim;
              break;
            case 2:
              m3_servo.forward += servo_trim;
              m3_servo.backward -= servo_trim;
              break;
          }
        }
      }
  }
  m1_servo = bound_speeds(m1_servo);
  m2_servo = bound_speeds(m2_servo);
  m3_servo = bound_speeds(m3_servo);
  
    // if the end position is near enough to the target, reset the servo speed
    for (unsigned char j = 0; j < 3; j++) {
      if (temp_dd[j].dist < thresh) {
        switch (j) {
          case 0:
            m1_servo.forward = flip_speed;
            m1_servo.backward = flip_speed;
            break;
          case 1:
            m2_servo.forward = flip_speed;
            m2_servo.backward = flip_speed;
            break;
          case 2:
            m3_servo.forward = flip_speed;
            m3_servo.backward = flip_speed;
            break;
        }
      }
    }
 
  /*
    Serial.print("M1: f_speed: ");
    Serial.print(m1_servo.forward, DEC);
    Serial.print(", b_speed: ");
    Serial.println(m1_servo.backward, DEC);
  */
  return true;
}
///////////////////////////////////////////////////////////////////////////////////////////
boolean all_oscillate_live (unsigned int target[3], unsigned char power[3]) {
  // takes the dircetion and power each flipper is supposed to be flipping

  // start by figuring out direction of m1
  if (m1_state) { // if m1 is travelling forward
    unsigned int m1_place = read_position(enc_num_for_motor(1));
    dir_dist m1_dir_dist = find_dir_dist(m1_place, target[0] + flip_range / 2);
    if ((m1_dir_dist.dir == 1) && (m1_dir_dist.dist > 0))
      // if flipper if rotating forward and has surpassed upper inflection point
      cmd_pololu(1, -1, power[0]); // flip direction of the flipper
  }
  if (!m1_state) { // if m1 is travelling backward
    unsigned int m1_place = read_position(enc_num_for_motor(1));
    dir_dist m1_dir_dist = find_dir_dist(m1_place, target[0] - flip_range / 2);
    if ((m1_dir_dist.dir == -1) && (m1_dir_dist.dist > 0))
      // if flipper if rotating forward and has surpassed upper inflection point
      cmd_pololu(1, 1, power[0]); // flip direction of the flipper
  }
  /*
    // do the same things for the other two flippers
    if (m2_state) { // if m1 is travelling forward
      unsigned int m2_place = read_position(enc_num_for_motor(2));
      dir_dist m2_dir_dist = find_dir_dist(m2_place, target[1] + flip_range / 2);
      if ((m2_dir_dist.dir == 1) && (m2_dir_dist.dist > 0))
        // if flipper if rotating forward and has surpassed upper inflection point
        cmd_pololu(2, -1, power[1]); // flip direction of the flipper
    }
    if (!m2_state) { // if m1 is travelling backward
      unsigned int m2_place = read_position(enc_num_for_motor(2));
      dir_dist m2_dir_dist = find_dir_dist(m2_place, target[1] - flip_range / 2);
      if ((m2_dir_dist.dir == -1) && (m2_dir_dist.dist > 0))
        // if flipper if rotating forward and has surpassed upper inflection point
        cmd_pololu(2, 1, power[1]); // flip direction of the flipper
    }

    if (m3_state) { // if m1 is travelling forward
      unsigned int m3_place = read_position(enc_num_for_motor(3));
      dir_dist m3_dir_dist = find_dir_dist(m3_place, target[2] + flip_range / 2);
      if ((m3_dir_dist.dir == 1) && (m3_dir_dist.dist > 0))
        // if flipper if rotating forward and has surpassed upper inflection point
        cmd_pololu(3, -1, power[2]); // flip direction of the flipper
    }
    if (!m3_state) { // if m1 is travelling backward
      unsigned int m3_place = read_position(enc_num_for_motor(3));
      dir_dist m3_dir_dist = find_dir_dist(m3_place, target[2] - flip_range / 2);
      if ((m3_dir_dist.dir == -1) && (m3_dir_dist.dist > 0))
        // if flipper if rotating forward and has surpassed upper inflection point
        cmd_pololu(3, 1, power[2]); // flip direction of the flipper
    }
  */
  return true;
}
///////////////////////////////////////////////////////////////////////////////////////////
unsigned int convert_joy_to_enc (char horiz, char vert) {
  // takes a horizontal and vertical component (basically a vector)
  // returns an encoder direction corresponding to aforementioned vector
  //Serial.println("Begin convert_joy_to_enc");
  double angle = atan2(vert, horiz);
  if (angle<0)
  {
  angle += 2*M_PI; // shift arctan results to only be positive
  }
  unsigned int enc_pos = 4096 * angle / (2 * M_PI);
  if (enc_pos == 4096) enc_pos = 0;
  //Serial.print("Joy to encoder value: ");                                                      
  //Serial.println(enc_pos);
  return enc_pos;
}
///////////////////////////////////////////////////////////////////////////////////////////
flip_servo_speed bound_speeds (flip_servo_speed passed_in) {
  /*gives: adjusted flip_servo_speed
    takes: flip_servo_speed
    if the servo speeds are too far from each other, pull them back withing range
  */
  if (passed_in.forward - passed_in.backward > servo_bound) {
    passed_in.forward -= servo_trim;
    passed_in.backward += servo_trim;
  }
  else if (passed_in.forward - passed_in.backward < -servo_bound) {
    passed_in.forward += servo_trim;
    passed_in.backward -= servo_trim;
  }
  // Serial.println("Speeds bounded.");
  return passed_in;
}
///////////////////////////////////////////////////////////////////////////////////////////
boolean planar_swim (vector_2 target) {
  int wheel_speeds[3] = {0, 0, 0};
  /* Wheel numbering and robot orientation:
          1
          /\
         /__\
       2      3
     x-axis points from center of robot to the left
     y-axis points from the center of the robot to the bottom
  */
  /*
    // debugging
    Serial.print("X: ");
    Serial.print(target.x_dir, DEC);
    Serial.print(", Y: ");
    Serial.println(target.y_dir, DEC);
  */
  wheel_speeds[0] = -target.y_dir;
  wheel_speeds[1] = target.x_dir * cos_60 + target.y_dir * sin_60;
  wheel_speeds[2] = target.x_dir * cos_60 + target.y_dir * sin_60;
  /*
    // debugging
    for (unsigned char i = 1; i < 4; i++) {
      Serial.print("Wheel ");
      Serial.print(i, DEC);
      Serial.print(" speed: ");
      Serial.println(wheel_speeds[i - 1]);
    }
  */
  // NEED TO SCALE THESE TO THE 0 to 180 INPUT FOR SERVOS
  for (char i = 0; i < 3; i++) {
    wheel_speeds[i] = wheel_speeds[i] * max_prop / 127;
  }
  /*
    // debugging
    for (unsigned char i = 1; i < 4; i++) {
      Serial.print("Wheel ");
      Serial.print(i, DEC);
      Serial.print(" speed: ");
      Serial.println(wheel_speeds[i - 1], DEC);
    }
  */
  char wheel_dir[3] = {0, 0, 0};
  for (char i = 0; i < 3; i++) {
    if (wheel_speeds[i] > 0) wheel_dir[i] = 1;
    else if (wheel_speeds[i] < 0) wheel_dir[i] = -1;
    else wheel_dir[i] = 0;
  }
  for (unsigned char i = 1; i < 4; i++) {
    /*
      // debugging
      Serial.print("Wheel ");
      Serial.print(i, DEC);
      Serial.print(" speed: ");
      Serial.println(abs(wheel_speeds[i - 1]));
    */
    cmd_pololu (i, wheel_dir[i - 1], abs(wheel_speeds[i - 1]));
  }
}
///////////////////////////////////////////////////////////////////////////////////////////
boolean surf_drive (vector_2 target) {
  int wheel_speeds[3] = {0, 0, 0};
  /* Wheel numbering and robot orientation:
          1
          /\
         /__\
       2      3
     x-axis points from center of robot to the left
     y-axis points from the center of the robot to the bottom
  */

  wheel_speeds[0] = target.x_dir;
  wheel_speeds[1] = -target.x_dir * cos_60 - target.y_dir * sin_60;
  wheel_speeds[2] = -target.x_dir * cos_60 + target.y_dir * sin_60;

  // debugging
  for (unsigned char i = 1; i < 4; i++) {
    Serial.print("Wheel ");
    Serial.print(i, DEC);
    Serial.print(" speed: ");
    Serial.println(wheel_speeds[i - 1], DEC);
  }

  // NEED TO SCALE THESE TO THE 0 to 180 INPUT FOR SERVOS
  for (char i = 0; i < 3; i++) {
    wheel_speeds[i] = wheel_speeds[i] * max_wheel / 127;
  }
  /*
    // debugging
    for (unsigned char i = 1; i < 4; i++) {
      Serial.print("Wheel ");
      Serial.print(i, DEC);
      Serial.print(" speed: ");
      Serial.println(wheel_speeds[i - 1], DEC);
    }
  */
  char wheel_dir[3] = {0, 0, 0};
  for (char i = 0; i < 3; i++) {
    if (wheel_speeds[i] > 0) wheel_dir[i] = 1;
    else if (wheel_speeds[i] < 0) wheel_dir[i] = -1;
    else wheel_dir[i] = 0;
  }

  for (unsigned char i = 1; i < 4; i++) {
    cmd_pololu (i, wheel_dir[i - 1], abs(wheel_speeds[i - 1]));
  }
}
///////////////////////////////////////////////////////////////////////////////////////////
boolean yaw (boolean dir) {
  /* gives: true if subroutine completes
     takes: 1. true: spinning counterclockwise if looking down on robot
               false: spinning clockwise if looking down on robot
            2. rate at which to tilt the robot (currently unused)
  */
  stop_all();
  unsigned int target;
  if (dir) target = 1024; // may need to switch these two
  else target = 3072;

  unsigned int target_arr[3] = {target, target, target};
  if (!(yaw_state)) {
    // if not already rotating yaw, the initialize flipper positions
    Serial.println("Set all flippers to starting positions.");
    for (unsigned char i = 1; i < 4; i++) {
      PID_set_pos_one(i, target);
    }
  }
  //one_oscillate(1, target_arr[0]);
  all_oscillate(target_arr);
}
///////////////////////////////////////////////////////////////////////////////////////////
boolean pitch (boolean rot_dir) {
  /* gives: true if subroutine completes
     takes: 1. true: tilting front upwards relative to robot
               false: for tilting front down relative to robot
            2. rate at which to tilt the robot (currently unused)
  */
  unsigned int target_arr[3];
  if (rot_dir) {
    target_arr[0] = 0;
    target_arr[1] = 2048;
    target_arr[2] = 2048;
  }
  else {
    target_arr[0] = 2048;
    target_arr[1] = 0;
    target_arr[2] = 0;
  }

  if (!(pitch_state)) {
    // if the robot was already changing pitch, do not reset the flippers
    for (unsigned char i = 1; i < 4; i++) {
      PID_set_pos_one(i, target_arr[i - 1]);
    }
  }

  all_oscillate(target_arr);
}
///////////////////////////////////////////////////////////////////////////////////////////
boolean roll (boolean rol_dir) {
  /* gives: true if subroutine completes
     takes: 1. true: tilting right, assuming robot coordinates
               false: for tilting left, assmuming robot coordinates
            2. rate at which to tilt the robot (currently unused)
     Wheel numbering and robot orientation:
          1
          /\
         /__\
       2      3
     x-axis points from center of robot to the left
     y-axis points from the center of the robot to the bottom
  */
  unsigned int target_arr[2];
  if (rol_dir) {
    target_arr[0] = 0; // do not need to use motor box 1 for roll
    target_arr[1] = 2048;
  }
  else {
    target_arr[0] = 2048;
    target_arr[1] = 0;
  }

  if (!(roll_state)) {
    // if the robot was already changing roll, do not reset the flippers
    for (unsigned char i = 2; i < 4; i++) {
      PID_set_pos_one(i, target_arr[i - 2]);
    }
  }

  two_oscillate(target_arr);
}
///////////////////////////////////////////////////////////////////////////////////////////
boolean two_oscillate (unsigned int target[2]) {
  /*gives: true when complete with subroutine
    takes: 1. an array for all of the target locations
    moves flipper forward then backward for a set time. adjusts speed afterwards to try
    get flipper to remain in the same bounds. the flipper should already be close
    to the target location before running subroutine

    This is only used for the freewater roll operation
  */
  // Serial.println("Begin all_oscillate");
  //cmd_pololu(1, 1, m1_servo.forward);
  cmd_pololu(2, 1, m2_servo.forward);
  cmd_pololu(3, 1, m3_servo.forward);
  delay(flip_duration);
  stop_all(); // stop the motor
  delay(motor_stop_delay);
  //cmd_pololu(1, -1, m1_servo.backward);
  cmd_pololu(2, -1, m2_servo.backward);
  cmd_pololu(3, -1, m3_servo.backward);
  delay(flip_duration);
  stop_all(); // stop all the motor boxes
  delay(motor_stop_delay); // wait until interference gone
  all_flip_pos all_end_pos = read_all(); // find final pose
  /*
    Serial.print("Stop: ");
    Serial.println(end_pos, DEC);
  */

  // dir_dist temp_dd = find_dir_dist(start_pos, end_pos);
  dir_dist temp_dd[2];
  temp_dd[0] = find_dir_dist(target[0], all_end_pos.m2);
  temp_dd[1] = find_dir_dist(target[1], all_end_pos.m3);

  for (unsigned char j = 0; j < 2; j++) {
    unsigned char loop_times;
    if (temp_dd[j].dist > thresh_oscillate) loop_times = 2;
    else if  (temp_dd[j].dist > thresh) loop_times = 1;
    else loop_times = 0;

    for (unsigned char i = 0; i < loop_times; i++) {
      if (temp_dd[j].dir > 0) {
        switch (j) {
          case 0 :
            m2_servo.forward -= servo_trim;
            m2_servo.backward += servo_trim;
            break;
          case 1:
            m3_servo.forward -= servo_trim;
            m3_servo.backward += servo_trim;
            break;
        }
      }
      else if (temp_dd[j].dir < 0) {
        switch (j) {
          case 0:
            m2_servo.forward += servo_trim;
            m2_servo.backward -= servo_trim;
            break;
          case 1:
            m3_servo.forward += servo_trim;
            m3_servo.backward -= servo_trim;
            break;
        }
      }
    }
  }
  // m1_servo = bound_speeds(m1_servo);
  m2_servo = bound_speeds(m2_servo);
  m3_servo = bound_speeds(m3_servo);
  /*
    // if the end position is near enough to the target, reset the servo speed
    for (unsigned char j = 0; j < 3; j++) {
      if (temp_dd[j].dist < thresh) {
        switch (j) {
          case 0:
            m1_servo.forward = flip_speed;
            m1_servo.backward = flip_speed;
            break;
          case 1:
            m2_servo.forward = flip_speed;
            m2_servo.backward = flip_speed;
            break;
          case 2:
            m3_servo.forward = flip_speed;
            m3_servo.backward = flip_speed;
            break;
        }
      }
    }
  */
  /*
    Serial.print("M1: f_speed: ");
    Serial.print(m1_servo.forward, DEC);
    Serial.print(", b_speed: ");
    Serial.println(m1_servo.backward, DEC);
  */
  return true;
}
///////////////////////////////////////////////////////////////////////////////////////////
boolean reverse_enc (unsigned char motor_num) {
  Serial.print("Finding encoder direction for enc ");
  Serial.println(motor_num, DEC);
  unsigned int start_pos = read_position(enc_num_for_motor(motor_num));
  cmd_pololu(motor_num, 1, flip_speed);
  delay(flip_duration);
  cmd_pololu(motor_num, 0, 0);
  delay(motor_stop_delay);
  unsigned int end_pos = read_position(enc_num_for_motor(motor_num));
  /*
    // debugging output
    Serial.print("Start pos: ");
    Serial.println(start_pos);
    Serial.print("  End pos: ");
    Serial.println(end_pos);
    delay(2000);
  */
  dir_dist DD = find_dir_dist(start_pos, end_pos);
  if (DD.dir < 0) {
    switch (motor_num) {
      case 1:
        enc1_reverse = true;
        Serial.println("Encoder 1 reversed");
        break;
      case 2:
        enc2_reverse = true;
        Serial.println("Encoder 2 reversed");
        break;
      case 3:
        enc3_reverse = true;
        Serial.println("Encoder 3 reversed");
        break;
      default:
        Serial.println("Invalid motor number for reverse_enc");
        return false;
    }
  }
  return true;
}
///////////////////////////////////////////////////////////////////////////////////////////
boolean yaw_surf (boolean dir) {
  if (dir) {
    cmd_pololu(1, 1, yaw_surf_speed);
    cmd_pololu(2, 1, yaw_surf_speed);
    cmd_pololu(3, 1, yaw_surf_speed);
  }
  else {
    cmd_pololu(1, -1, yaw_surf_speed);
    cmd_pololu(2, -1, yaw_surf_speed);
    cmd_pololu(3, -1, yaw_surf_speed);
  }
}
///////////////////////////////////////////////////////////////////////////////////////////
boolean reset_servo_speeds () {
  m1_servo.forward = flip_speed; // by default, set forward and backward speeds to 45
  m1_servo.backward = flip_speed;
  m2_servo.forward = flip_speed;
  m2_servo.backward = flip_speed;
  m3_servo.forward = flip_speed;
  m3_servo.backward = flip_speed;
}
///////////////////////////////////////////////////////////////////////////////////////////
boolean rough_cal_all () {
  /* takes: nothing
     gives: TRUE when routine completed

     rough_cal_all finds the encoder counts traveled for each of the motor boxes
     when driven at the rate flip_speed for a duration flip_duration
  */
  all_flip_pos start_poses = read_all();
  cmd_pololu(1, 1, flip_speed);
  cmd_pololu(2, 1, flip_speed);
  cmd_pololu(3, 1, flip_speed);
  delay(flip_duration);
  cmd_pololu(1, 0, 0);
  cmd_pololu(2, 0, 0);
  cmd_pololu(3, 0, 0);
  all_flip_pos end_poses = read_all();
  rough_speeds.m1 = find_dir_dist(start_poses.m1, end_poses.m1).dist;
  rough_speeds.m2 = find_dir_dist(start_poses.m2, end_poses.m2).dist;
  rough_speeds.m3 = find_dir_dist(start_poses.m3, end_poses.m3).dist;
  Serial.print("Rough Speeds M1: ");
  Serial.print(rough_speeds.m1);
  Serial.print(", M2: ");
  Serial.print(rough_speeds.m2);
  Serial.print(", M3: ");
  Serial.println(rough_speeds.m3);
  if ((rough_speeds.m1 > 600) || (rough_speeds.m2 > 600) || (rough_speeds.m3 >600)) Serial.println("Encoder readings");
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void wiggle_simult(int times){
  //Add motor_num = #
  unsigned char count = 0;
  while (count < times) {
    cmd_pololu(1, 1, flip_speed);
    cmd_pololu(2, 1, flip_speed);
    cmd_pololu(3, 1, flip_speed);
    delay(2*flip_duration);
    cmd_pololu(1, -1, flip_speed);
    cmd_pololu(2, -1, flip_speed);
    cmd_pololu(3, -1, flip_speed);
    delay(2*flip_duration);
    count++;
  }
  stop_all();
  delay(motor_stop_delay);
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

