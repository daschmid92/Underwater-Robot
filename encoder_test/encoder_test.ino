#include <SPI.h> // if including <ArduinoRobot.h>, exlude this
#include <Servo.h>
#include <Average.h>
#include <SoftwareSerial.h>

// encoder communication pins
#define encoder_1 32
#define encoder_2 31
#define encoder_3 30
#define MOSI_pin 51
#define MISO_pin 50
#define SPI_SCK 52
#define SS_pin 53 // SS pin should be set as OUTPUT
// PWM output pins for motor controllers (no longer using serial)
#define m1_PWM 11
#define m2_PWM 10
#define m3_PWM 9
#define BDCM_PWM 12
// other constants
#define max_speed 90 // max deviation from 90 for Servo
#define min_speed 13 // minimum speed to make motor move
#define SPI_speed 1200000 // for encoder comm
#define serial_baud 115200 // baud for comm between computer and Arduino
#define SPI_wait 1 // recommended 20 micro-sec delay between reads
#define BDCM_center 91 // value for no movement of BDCM for initializing and stopping
#define flip_center 2048 // ideal center of oscillation for basic flip
#define flip_range 40 // encoder counts for moving ~20 deg.
#define flip_speed 40 // power for adjusting flipping speed
#define flip_duration 20 // delay for moving flipper forward to find direction of encoder
#define thresh 11 // "close enough" for flipper position target, about 1 degree
#define servo_trim 1 // how much to adjust servo speed to correct for flipper centering
#define servo_bound 6 // max difference between forward and backward servo speeds
#define max_PID 40 // maximum speed for PID control

////////////////////////////// VARIABLE DECLARATIONS //////////////////////////////
Servo motor1;
Servo motor2;
Servo motor3;
Servo BDCM;

// sloppy use of global variables, because I got sick of passing them
unsigned int target_center = flip_center;
boolean enc1_reverse = true;
boolean enc2_reverse = false;
boolean enc3_reverse = false;
///////////////////////////////////////////////////////////////////////
struct flip_servo_speed {
  unsigned char forward; // add to 90 for Servo.write()
  unsigned char backward; // subtract from 90 for Servo.write()
};

flip_servo_speed m1_servo;
flip_servo_speed m2_servo;
flip_servo_speed m3_servo;
///////////////////////////////////////////////////////////////////////
struct all_flip_pos {
  unsigned int m1;
  unsigned int m2;
  unsigned int m3;
};
//////////////////////////////////////////////////////////////////////
struct flip_arcspeed { // units are milliseconds per encoder count
  float forward;
  float backward;
};
flip_arcspeed m1_times; // this is for one motorbox
flip_arcspeed m2_times;
flip_arcspeed m3_times;
//////////////////////////////////////////////////////////////////////
all_flip_pos read_all() {
  /* gives: positions of all the flippers
  takes: nothing
  NOT COMPLETE YET
  */
  all_flip_pos temp;
  temp.m1 = read_position(encoder_1);
  temp.m2 = read_position(encoder_2);
  temp.m3 = read_position(encoder_3);
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
/////////////////////////////////////////////////////////////////
// struct definitions to make passing information easier
struct dir_dist {
  char dir; // direction to rotate motor for shortest path to a target
  unsigned int dist; // shortest distance to target (in encoder counts)
};
dir_dist find_dir_dist(unsigned int flip_pos, unsigned int target);
//////////////////////////////////////////////////////////////////////
//Define Variables we'll be connecting to for PID control
double Setpoint, Input, Output;

//Define the aggressive and conservative Tuning Parameters
double aggKp=4, aggKi=0.2, aggKd=1;
double consKp=1, consKi=0.05, consKd=0.25;

//Specify the links and initial tuning parameters
//PID myPID(&Input, &Output, &Setpoint, consKp, consKi, consKd, DIRECT);
//////////////////////////////////////////////////////////////////////////



void setup() {
  // put your setup code here, to run once:
  Serial.begin(serial_baud); // communication to Arduino Serial Monitor
  Serial.println("Program start.");

  // attach software servos to appropriate pins
  motor1.attach(m1_PWM);
  motor2.attach(m2_PWM);
  motor3.attach(m3_PWM);
  BDCM.attach(BDCM_PWM);

  // set up SPI pins for communicating with encoders
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

  //initialize_BDCM();
  first_read(encoder_1);
  first_read(encoder_2);
  first_read(encoder_3);

  //Serial.println("Checking if encoders reversed...");
  fix_reverse_enc(1);
  fix_reverse_enc(2);
  fix_reverse_enc(3);

  //cmd_pololu(1, 1, 15);
  //motor1.write(90);
  //set_target_one(1, flip_center);
//  choppy_set_pos_one(2,4096);
  PID_set_pos_one (1,4096);
//  PID_ocsillate (1,4096);
  PID_set_pos_one (2,4096);
  PID_set_pos_one (3,4096);
  Serial.println("Begin Main Loop.");
  delay(2000);
  
}

////////////////////////////// MAIN LOOP //////////////////////////////
void loop() {
  
  Serial.println(read_pos_corrected(encoder_1));
  Serial.println(read_pos_corrected(encoder_2));
  Serial.println(read_pos_corrected(encoder_3));
//  oscillate_one(1);
//  unsigned int current_pos = read_position(encoder_1);
//  oscillate_one_live(1, current_pos);


//  for (int speedi = 0; speedi < max_speed; speedi += 10) {
//    cmd_pololu(1, 1, speedi);
//    Serial.print("Motor power: ");
//    Serial.println(speedi);
//    //delay(10000);
//    unsigned long start_milli = millis();
//    //Serial.println("here");
//    while (millis() - start_milli < 10000) {
//      read_position(encoder_1);
//    }
//  }
//  Serial.println("Restart");
//  delay(5000);

}

////////////////////////////// SUB-ROUTINES //////////////////////////////
boolean initialize_BDCM () {
  BDCM.write(BDCM_center);
  delay(2000);
}
//////////////////////////////////////////////////////////////////////////
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
    Serial.println("Exceeds motor speed range.");
    mag_speed = max_speed;
  }
  if (dir == 1) {
    // if forward
    //Serial.println("Forward.");
    switch (motor_num) {
      case 1:
        motor1.write(90 + mag_speed);
        break;
      case 2:
        motor2.write(90 + mag_speed);
        break;
      case 3:
        motor3.write(90 + mag_speed);
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
        break;
      case 2:
        motor2.write(90 - mag_speed);
        break;
      case 3:
        motor3.write(90 - mag_speed);
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
//////////////////////////////////////////////////////////////////////////
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
//////////////////////////////////////////////////////////////////////////
unsigned int read_position (unsigned char encoder_num) {
  /* Command 0x10 rd_pos (read position)
     1. Master sends rd_pos command, encoder responds with idle chars
     2. Continue sending nop_a5 command while encoder response is 0xA5
     3. If response was 0x10 (rd_pos), send nop_a5 and receive MSB position
        lower 4 bits of this byte ar the upper 4 of the 12-bit position
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
      // if encoder_num does not match any of the encoder values
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
  if (!(data_chunks[0] == 0xA5)) {

  }
  else {
    //while (data_chunks[0] == 0xA5) {
    unsigned char retry_count = 0;
    while (data_chunks[0] != 0x10) {
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
  while (!(data_chunks[2] == 0xA5)) {
    digitalWrite(encoder_num, LOW);
    data_chunks[2] = SPI.transfer(0); // keep sending 0 until receive 0x00 response
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


  // debugging output
  Serial.print("enc ");
  Serial.print(encoder_num);
  Serial.print(": ");
  Serial.println(data);

  return data;
}

//////////////////////////////////////////////////////////////////////////
unsigned int read_pos_corrected (unsigned char encoder_num) {
  // ~10% of the encoder readings are wrong, often the reading is 4096 minus the proper position
  // 4 samples seems to be good enough if we just toss the highest and lowest readings
  //Serial.println("Begin read_pos_corrected");
  unsigned int enc_samples[3];
  for (unsigned char i = 0; i < 3; i++) {
    enc_samples[i] = read_position(encoder_num);
    delay(2);
  }
  // need to sort array from smallest to largest to find median
  for (int i = 0; i < 3; i++) {
    for (int o = 0; o < 2; o++) {
      if (enc_samples[o] > enc_samples[o + 1]) {
        int t = enc_samples[o];
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
//////////////////////////////////////////////////////////////////////////
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
//////////////////////////////////////////////////////////////////////////
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
//////////////////////////////////////////////////////////////////////////
dir_dist find_dir_dist (unsigned int flip_pos, unsigned int target) {
  // gives: nothing
  // takes: target location
  // prints through the serial interface the direction to turn and the distance to the target
  //unsigned int flip_pos = read_pos_corrected(enc_num_for_motor(motor_num));
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
//////////////////////////////////////////////////////////////////////////
boolean fix_reverse_enc (unsigned char motor_num) {
  //Serial.print("Finding encoder direction for enc ");
  //Serial.println(motor_num, DEC);
  unsigned int start_pos = read_pos_corrected(enc_num_for_motor(motor_num));
  cmd_pololu(motor_num, 1, flip_speed);
  delay(flip_duration);
  cmd_pololu(motor_num, 0, 0);
  unsigned int end_pos = read_pos_corrected(enc_num_for_motor(motor_num));
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
        Serial.println("Invalid motor number for fix_reverse_enc");
        return false;
    }
  }
  else if (DD.dir == 0) {
    //Serial.println("Motor did not move, trying again...");
    fix_reverse_enc(motor_num);
  }
  else {
    switch (motor_num) {
      case 1:
        enc1_reverse = true;
        Serial.println("Encoder 1 okay");
        break;
      case 2:
        enc2_reverse = true;
        Serial.println("Encoder 2 okay");
        break;
      case 3:
        enc3_reverse = true;
        Serial.println("Encoder 3 okay");
        break;
      default:
        Serial.println("Invalid motor number for fix_reverse_enc");
        return false;
    }
  }
  return true;
}
//////////////////////////////////////////////////////////////////////////
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
//////////////////////////////////////////////////////////////////////////
boolean set_target_one( unsigned char motor_num, unsigned int target) {
  // not yet working smoothly
  unsigned int current_pos = read_pos_corrected (enc_num_for_motor(motor_num));
  /*
    // debuggingn output
    Serial.print("current position: ");
    Serial.println(current_pos);
    Serial.print("target position: ");
    Serial.println(target);
  */
  dir_dist displacement = find_dir_dist(current_pos, target);
  while (displacement.dist > thresh) {
    // calculate speed
    float p_const = 0.6; // proportional constant for P control
    float rate = min_speed +  p_const * max_speed * displacement.dist / 4096;
    // debugging output
    //Serial.println(rate, DEC);
    cmd_pololu(motor_num, displacement.dir, (unsigned char) rate);
    current_pos = read_pos_corrected (enc_num_for_motor(motor_num));
    displacement = find_dir_dist(current_pos, target);
  }
  cmd_pololu(motor_num, 0, 0);
}
//////////////////////////////////////////////////////////////////////////
boolean oscillate_one(unsigned char motor_num) {
  unsigned int current_pos = read_pos_corrected (enc_num_for_motor(motor_num));
  unsigned int target = flip_center;
  /*
    Serial.print("Starting position: ");
    Serial.println(current_pos);
    Serial.print("Oscillating center: ");
    Serial.println(target);
  */
  cmd_pololu(motor_num, 1, flip_speed);
  while (current_pos < target + flip_range / 2) {
    current_pos = read_pos_corrected (enc_num_for_motor(motor_num));
  }
  cmd_pololu(motor_num, 0, 0);
  delay(1000);
  cmd_pololu(motor_num, -1, flip_speed);
  while (current_pos > target - flip_range / 2) {
    current_pos = read_pos_corrected (enc_num_for_motor(motor_num));
  }
  cmd_pololu(motor_num, 0, 0);
  delay(1000);
}
/////////////////////////////////////////////////////////////////////////////
boolean stop_all() {
  // stops all the motor boxes but not the central prop
  cmd_pololu(1, 0, 0);
  cmd_pololu(2, 0, 0);
  cmd_pololu(3, 0, 0);
  //z_thrust(90);
}
/////////////////////////////////////////////////////////////////////////////
flip_servo_speed bound_speeds(flip_servo_speed passed_in) {
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
/////////////////////////////////////////////////////////////////////////////
boolean all_oscillate(unsigned int target[3]) {
  /*gives: true when complete with subroutine
  takes: 1. an array for all of the target locations
  moves flipper forward then backward for a set time. adjusts speed afterwards to try
  get flipper to remain in the same bounds. the flipper should already be close
  to the target location before running subroutine
  */
  // Serial.println("Begin all_oscillate");
  cmd_pololu(1, 1, m1_servo.forward);
//  cmd_pololu(2, 1, m2_servo.forward);
//  cmd_pololu(3, 1, m3_servo.forward);
  delay(40);//flip_duration : indirectly controls range of flipper motion
  stop_all(); // stop the motor
  delay(20); //motor_stop_delay : time for wires to clear or cross-talk
  cmd_pololu(1, -1, m1_servo.backward);
//  cmd_pololu(2, -1, m2_servo.backward);
//  cmd_pololu(3, -1, m3_servo.backward);
  delay(40);
  stop_all(); // stop all the motor boxes
  delay(20); // wait until interference gone     motor_stop_delay : time for wires to clear or cross-talk
  all_flip_pos all_end_pos = read_all(); // find final pose
                       /*
                       Serial.print("Stop: ");
                       Serial.println(end_pos, DEC);
                       */

                       // dir_dist temp_dd = find_dir_dist(start_pos, end_pos);
  dir_dist temp_dd[3];
  temp_dd[0] = find_dir_dist(target[0], all_end_pos.m1);
//  temp_dd[1] = find_dir_dist(target[1], all_end_pos.m2);
//  temp_dd[2] = find_dir_dist(target[2], all_end_pos.m3);

  for (unsigned char j = 0; j < 1; j++) {
    unsigned char loop_times;
    if (temp_dd[j].dist > 48) loop_times = 2; //thresh_oscillate : determines whether to trim servo speeds by 1 or 2
    else if (temp_dd[j].dist > 12) loop_times = 1; ///thresh : "close enough" for flipper position target about 1 degree
    else loop_times = 0;

    for (unsigned char i = 0; i < loop_times; i++) {
      if (temp_dd[j].dir > 0) {
        switch (j) {
        case 0:
          m1_servo.forward -= servo_trim;
          m1_servo.backward += servo_trim;
          break;
        case 1:
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
boolean oscillate_one_live(unsigned char motor_num, unsigned int target) {
  // this is a bang-bang controller for oscillating one flipper with live encoder readings
  unsigned int current_pos = read_position(enc_num_for_motor(motor_num));
  //unsigned int target = flip_center;
  /*
  Serial.print("Starting position: ");
  Serial.println(current_pos);
  Serial.print("Oscillating center: ");
  Serial.println(target);
  */
  cmd_pololu(motor_num, 1, 10);
  while (current_pos < target + 110) {
    current_pos = read_position(enc_num_for_motor(motor_num));
    Serial.print("(+)positive");
  }
  cmd_pololu(motor_num, 0, 0);
  delay(1000);
  cmd_pololu(motor_num, -1, 10);
  while (current_pos > target - 110) {
    current_pos = read_position(enc_num_for_motor(motor_num));
    Serial.print("(-)negative");
  }
  cmd_pololu(motor_num, 0, 0);
  delay(1000);
}
/////////////////////////////////////////////////////////////////////////
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

    unsigned int slow_down_range = 400;
    unsigned char close_speed;
    if (motor_cmds.dist < slow_down_range) {
      close_speed = flip_speed - (float)motor_cmds.dist / 15;
      Serial.print("Scaled speed: ");
      Serial.println(close_speed, DEC);
    }
    else close_speed = flip_speed;
 
    start = millis();
    cmd_pololu(m_box, motor_cmds.dir, close_speed);
    while (!(m1_state)) {
      // loop until time is up
      if (millis() - start >= expected_time) m1_state = true;
      int temp = millis();
      Serial.println("Wait until target reached.\n");
      Serial.print("Expected time: ");
      Serial.println(expected_time, DEC);
      Serial.print("Current time: ");
      Serial.println(temp - start, DEC);
    }
    m1_state = false; // reset for the next loop around
  }
}
//////////////////////////////////////////////////////////////////////////////////////////
boolean PID_set_pos_one (unsigned char motor_num, unsigned int target) {
  // for M2 : speedy min = 7, scale factor = 0.068
  unsigned int current_pos = read_position (enc_num_for_motor(motor_num));
  
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
    current_pos = read_position (enc_num_for_motor(motor_num));
    displacement = find_dir_dist(current_pos, target);
  }
  cmd_pololu(motor_num, 0, 0);

  Serial.print("Final error: ");
  Serial.println(displacement.dist);
  return true;
}
///////////////////////////////////////////////////////////////////////////////////////////
boolean PID_ocsillate (unsigned char motor_num, unsigned int target) {
  unsigned int current_pos = read_position (enc_num_for_motor(motor_num));
  
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
    current_pos = read_position (enc_num_for_motor(motor_num));
    displacement = find_dir_dist(current_pos, target);
  }
  cmd_pololu(motor_num, 0, 0);

  Serial.print("Final error: ");
  Serial.println(displacement.dist);
  return true;
}
