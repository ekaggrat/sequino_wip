#include <Arduino.h>

//steps per mm calculation
//----------------------------------------------------------------------------

// y motor
// motor steps per turn = 16300 (8 microsteps )( microstepping )


// x motor
// motor steps per turn = 16300 (8 microsteps ) ( microstepping )

// SPROCKET RATIO 1:3
// sprocket teeth 8


// IN REALITY STEPS TO COMPLETE ONE CIRCLE 194MM COMMANDED
//that makes 194 x 106.3 = 20622.2 steps ( full circle )
// steps for full belt = 20622.2 x 4.64 = 95687

// so actual ratio is 20622.2/(18x4) = 286.419
// so actual steps per mm = total belt steps / belt length
// 95687/314.3 = 304.44

// z steps
// z_gear ratio 20:58 1:2.9 // NEW 20:112 1:5.6
// steps per revolution is 16300x 3 = 48900 // 16300 X 5.6 = 91280
// radius = 52 mm
// steps per mm = 48900/circumfrance = 48900/326.72 = 149.66 for 16 steps = 299.33
// NEW STEPS PER MM = 48900/circumfrance = 91280/326.72 = 279.38 for 16 steps = 558.76
//------------------------------------------------------------------------------
// CONSTANTS
//------------------------------------------------------------------------------
//#define VERBOSE              (1)  // add to get a lot more serial output.

#define VERSION              (2)  // firmware version
#define BAUD                 (57600)  // How fast is the Arduino talking?
#define MAX_BUF              (64)  // What is the longest message Arduino can store?
#define STEPS_PER_TURN       (100)  // depends on your stepper motor.  most are 200.
#define STEPS_PER_MM         (STEPS_PER_TURN*1/1.25)  // (400*16)/1.25 with a M8 spindle
#define STEPS_PER_MM_X         31.25//500
#define STEPS_PER_MM_Y       31.25//500
#define STEPS_PER_MM_Z       34.92//558.76//299.33
#define MAX_FEEDRATE         (6000000)//(500000)//(1000000)INCREASEING THE VALUE SLOWS IT 
#define MIN_FEEDRATE         (1)
#define NUM_AXIES            (4)
#include <Servo.h>
#include <math.h>
#include <Time.h>

#include <TimeLib.h>

Servo servo_x;  // create servo object to control a servo
Servo servo_y;

boolean x_homestate = false;

boolean y_homestate = false;

boolean z_homestate = false;

# define y_stop A1
# define x_stop A0
# define z_stop 11

int home_y_position = 0;
int home_x_position = 0;
int home_z_position = 0;

int m2_pos = 169;
int m1_pos = 107;//112;
int c_pos = 85;

int h1_pos = 0;
int h2_pos = 24;


int x_one_seg = 5;
int ang_1 = 60;//45;
int ang_2 = 80;
int ang_3 = 30;

int start_offset = 0;
int start_offset_y = 20;
int y_back = 2;
float pen_height = 6.5;
int y_peak = 900;
int start_offset_z = 175;
//led fading

int brightness = 0;    // how bright the LED is
int fadeAmount = 5;    // how many points to fade the LED by
long previousMillis = 0;
long interval = 30;
int led = 12;

int xx = 0;
int zz = 0;


// time demo

long demo_previousMillis = 0;
long demo_interval = 500;
int execute_time = 0;
int time_count = 0;
int hit_delay  = 600;








//------------------------------------------------------------------------------
// STRUCTS
//------------------------------------------------------------------------------
// for line()
typedef struct {
  long delta;  // number of steps to move
  long absdelta;
  long over;  // for dx/dy bresenham calculations
}
Axis;


typedef struct {
  int step_pin;
  int dir_pin;
  int enable_pin;
  int limit_switch_pin;
}
Motor;


//------------------------------------------------------------------------------
// GLOBALS
//------------------------------------------------------------------------------
Axis a[NUM_AXIES];  // for line()
Axis atemp;  // for line()
Motor motors[NUM_AXIES];

char buffer[MAX_BUF];  // where we store the message until we get a ';'
int sofar;  // how much is in the buffer

// speeds
float fr = 0; // human version
long step_delay;  // machine version

float px, py, pz, pe; // position

// settings
char mode_abs = 1; // absolute mode? = 1 relative mode = 0

long line_number = 0;

//-----------------------------------------------------------------------------
// time code from old doodle
//-----------------------------------------------------------------------------

int disp_m1 = minute() / 10;
int disp_m2 = minute() % 10;
int hh = hour();
int disp_h = hh;

int disp_h1 = disp_h / 10;
int disp_h2 = disp_h % 10;

int disp_h12 = abs( hh - 12);
int disp_h1_12 = disp_h12 / 10;
int disp_h2_12 = disp_h12 % 10;



void sudo_time() {
  line(h2_pos, py, zz, 0);
  nine(0, 0);

  line(c_pos , py, zz, 0);
  colon(0, 0);

  line(m1_pos, py, zz, 0);
  zero(0, 0);

  line(m2_pos, py, zz, 0);
  three(0, 0);

  line(c_pos , py, zz, 0);

  delay(2000);
  line(m2_pos, py, zz, 0);
  erase(0, 0);
  four(0, 0);
  line(c_pos , py, zz, 0);



}


void starttime() // function to plot time the first time .. still needs improvement
{
  int hh = hour();
  if (hh > 12) {
    hh = hh - 12;
  }

  int hh1 = hh / 10;

  if (hh1 > 0 ) {
    update_h1();
    update_h2();

    line(c_pos , py, zz, 0);
    erase_line(0, 0);
    colon(0, 0);

    update_m1();
    update_m2();

  }
  else
    update_h2();

  line(c_pos, py, zz, 0);
  erase_line(0, 0);
  colon(0, 0);

  update_m1();
  update_m2();

}



void printtime() // main function handling the ploting of the time digits
{
  int hh = hour();

  int h1 = hh / 10;
  int h2 = hh % 10;




  if ( hh > 12) {
    hh = hh - 12;
    int h1_12 = hh / 10;
    int h2_12 = hh % 10;

    int chk_hour1_12 = abs(disp_h1_12 - h1_12);
    int chk_hour2_12 = abs(disp_h2_12 - h2_12);





    if ( chk_hour1_12 > 0) {

      update_h1();
      disp_h1_12 = h1_12;
    }


    if ( chk_hour2_12 > 0) {

      update_h2();
      disp_h2_12 = h2_12;
    }
  }

  if ( hh <= 12) {



    int chk_hour1 = abs(disp_h1 - h1);
    int chk_hour2 = abs(disp_h2 - h2);





    if ( chk_hour1 > 0) {

      update_h1();
      disp_h1 = h1;
    }


    if ( chk_hour2 > 0) {

      update_h2();
      disp_h2 = h2;
    }
  }

  int m = minute();
  int m1 = m / 10;
  int m2 = m % 10;
  int chk_min1 = abs(disp_m1 - m1);
  int chk_min2 = abs(disp_m2 - m2);



  if ( chk_min1 > 0) {
    update_m1();
    disp_m1 = m1;
  }
  if ( chk_min2 > 3) {
    update_m2();
    disp_m2 = m2;
  }
}

void update_h1() // update hour1 digit
{
  motor_enable();
  line(h1_pos, py, zz, 0);
  erase_line(0, 0);
  hour1();
}

void update_h2()// update hour2 digit
{
  motor_enable();
  line(h2_pos, py, zz, 0);
  erase(0, 0);
  hour2();
}

void update_m1()// update minute1 digit
{ motor_enable();
  line(m1_pos, py, zz, 0);
  erase(0, 0);
  min1();
}

void update_m2()// update minute2 digit
{
  motor_enable();
  line(m2_pos, py, zz, 0);
  erase(0, 0);
  min2();


}


void hour1()//calcuate hour 1
{
  int h = hour();
  if (h > 12) {
    h = h - 12;
  }
  int h1 = h / 10;
  Serial.println("h1");
  Serial.println(h1);

  digit(h1, 0, 0);

}


void blank()
{
}
void hour2()//calcuate hour 2
{
  int h = hour();
  if (h > 12) {
    h = h - 12;
  }
  int h2 = h % 10;
  Serial.println("h2");
  Serial.println(h2);

  digit(h2, 0, 0);

}

void min1()//calcuate minute 1
{
  int m = minute();
  int m1 = m / 10;
  Serial.println("m1");
  Serial.println(m1);
  digit(m1, 0, 0);

}

void min2()//calcuate minute 2
{
  int m = minute();
  int m2 = m % 10;
  Serial.println("m1");
  Serial.println(m2);
  digit(m2, 0, 0);

}

void digit(int timee, float xx, float zz)// function to figure out which digit to plot
{
  if (timee == 1) {

    one(xx, zz);
  }
  else if (timee == 2) {

    two(xx, zz);
  }
  else if (timee == 3) {

    three(xx, zz);
  }
  else  if (timee == 4) {

    four(xx, zz);
  }
  else if (timee == 5) {

    five(xx, zz);
  }
  else if (timee == 6) {

    sixx(xx, zz);
  }
  else if (timee == 7) {

    seven(xx, zz);
  }
  else if (timee == 8) {

    eight(xx, zz);
  }
  else if (timee == 9) {

    nine(xx, zz);
  }
  else if (timee == 0) {

    zero(xx, zz);
  }
}

void current_time() {
  Serial.print("Current time : ");
  Serial.println(hour());
  Serial.print(":");
  Serial.print(minute());


}

//-------------------------------------------------------------------------------
// end of time code
//-------------------------------------------------------------------------------







//------------------------------------------------------------------------------
// METHODS
//------------------------------------------------------------------------------


/**
   delay for the appropriate number of microseconds
   @input ms how many milliseconds to wait
*/
void pause(long ms) {
  delay(ms / 1000);
  delayMicroseconds(ms % 1000); // delayMicroseconds doesn't work for values > ~16k.
}


/**
   Set the feedrate (speed motors will move)
   @input nfr the new speed in steps/second
*/
void feedrate(float nfr) {
  if (fr == nfr) return; // same as last time?  quit now.

  if (nfr > MAX_FEEDRATE || nfr < MIN_FEEDRATE) { // don't allow crazy feed rates
    Serial.print(F("New feedrate must be greater than "));
    Serial.print(MIN_FEEDRATE);
    Serial.print(F("steps/s and less than "));
    Serial.print(MAX_FEEDRATE);
    Serial.println(F("steps/s."));
    return;
  }
  step_delay = MAX_FEEDRATE / nfr;
  fr = nfr;
}


/**
   Set the logical position
   @input npx new position x
   @input npy new position y
*/
void position(float npx, float npy, float npz, float npe) {
  // here is a good place to add sanity tests
  px = npx;
  py = npy;
  pz = npz;
  pe = npe;
}


/**
   Supports movement with both styles of Motor Shield
   @input newx the destination x position
   @input newy the destination y position
 **/



/**
   Uses bresenham's line algorithm to move both motors
   @input newx the destination x position
   @input newy the destination y position
 **/
void line(float newx, float newy, float newz, float newe) {



  a[0].delta = ((newx - px) + (newy - py)) * STEPS_PER_MM_X;
  a[1].delta = ((newx - px) - (newy - py)) * STEPS_PER_MM_Y;
  a[2].delta = (newz - pz) * STEPS_PER_MM_Z;
  a[3].delta = (newe - pe) * STEPS_PER_MM;

  long i, j, maxsteps = 0;

  for (i = 0; i < NUM_AXIES; ++i) {
    a[i].absdelta = abs(a[i].delta);
    if ( maxsteps < a[i].absdelta ) maxsteps = a[i].absdelta;
    // set the direction once per movement
    digitalWrite(motors[i].dir_pin, a[i].delta > 0 ? HIGH : LOW);
  }
  for (i = 0; i < NUM_AXIES; ++i) {
    a[i].over = maxsteps / 2;
  }

  long dt = MAX_FEEDRATE / 5000;
  long accel = 1;
  long steps_to_accel = dt - step_delay;
  if (steps_to_accel > maxsteps / 2 )
    steps_to_accel = maxsteps / 2;

  long steps_to_decel = maxsteps - steps_to_accel;

  //Serial.print("START ");
  // Serial.println(dt);
  // Serial.print("TOP ");
  // Serial.println(step_delay);

  // Serial.print("accel until ");
  // Serial.println(steps_to_accel);
  // Serial.print("decel after ");
  // Serial.println(steps_to_decel);
  // Serial.print("total ");
  // Serial.println(maxsteps);
#ifdef VERBOSE
  Serial.println(F("Start >"));
#endif

  for ( i = 0; i < maxsteps; ++i ) {
    for (j = 0; j < NUM_AXIES; ++j) {
      a[j].over += a[j].absdelta;
      if (a[j].over >= maxsteps) {
        a[j].over -= maxsteps;

        digitalWrite(motors[j].step_pin, HIGH);
        digitalWrite(motors[j].step_pin, LOW);
      }
    }

    if (i < steps_to_accel) {
      dt -= accel;
    }
    if (i >= steps_to_decel) {
      dt += accel;
    }
    delayMicroseconds(dt);
  }

#ifdef VERBOSE
  Serial.println(F("< Done."));
#endif

  position(newx, newy, newz, newe);

  //where();
}


// returns angle of dy/dx as a value from 0...2PI
static float atan3(float dy, float dx) {
  float a = atan2(dy, dx);
  if (a < 0) a = (PI * 2.0) + a;
  return a;
}



/**
   Look for character /code/ in the buffer and read the float that immediately follows it.
   @return the value found.  If nothing is found, /val/ is returned.
   @input code the character to look for.
   @input val the return value if /code/ is not found.
 **/
float parsenumber(char code, float val) {
  char *ptr = buffer;
  while (ptr && *ptr && ptr < buffer + sofar) {
    if (*ptr == code) {
      return atof(ptr + 1);
    }
    ptr = strchr(ptr, ' ') + 1;
  }
  return val;
}


/**
   write a string followed by a float to the serial line.  Convenient for debugging.
   @input code the string.
   @input val the float.
*/
void output(char *code, float val) {
  Serial.print(code);
  Serial.println(val);
}


/**
   print the current position, feedrate, and absolute mode.
*/
void where() {
  output("X", px);
  output("Y", py);
  output("Z", pz);
  //output("E", pe);
  output("F", fr);
  Serial.println(mode_abs ? "ABS" : "REL");
}


/**
   display helpful information
*/
void help() {
  Serial.print(F("GcodeCNCDemo6AxisV2 "));
  Serial.println(VERSION);
  Serial.println(F("Commands:"));
  Serial.println(F("G00/G01 [X/Y/Z/E(steps)] [F(feedrate)]; - linear move"));
  Serial.println(F("G04 P[seconds]; - delay"));
  Serial.println(F("G90; - absolute mode"));
  Serial.println(F("G91; - relative mode"));
  Serial.println(F("G92 [X/Y/Z/E(steps)]; - change logical position"));
  Serial.println(F("M18; - disable motors"));
  Serial.println(F("M100; - this help message"));
  Serial.println(F("M114; - report position and feedrate"));
  Serial.println(F("All commands must end with a newline."));
}


/**
   Read the input buffer and find any recognized commands.  One G or M command per line.
*/
void processCommand() {
  int cmd = parsenumber('G', -1);
  switch (cmd) {
    case  0:
    case  1:
      { // line
        feedrate(parsenumber('F', fr));
        line( parsenumber('X', (mode_abs ? px : 0)) + (mode_abs ? 0 : px),
              parsenumber('Y', (mode_abs ? py : 0)) + (mode_abs ? 0 : py),
              parsenumber('Z', (mode_abs ? pz : 0)) + (mode_abs ? 0 : pz),
              parsenumber('E', (mode_abs ? pe : 0)) + (mode_abs ? 0 : pe) );
        break;
      }
    case  2:
      { // line
        feedrate(parsenumber('F', fr));
        line( parsenumber('X', (mode_abs ? px : 0)) + (mode_abs ? 0 : px),
              parsenumber('X', (mode_abs ? py : 0)) + (mode_abs ? 0 : py),
              parsenumber('Z', (mode_abs ? pz : 0)) + (mode_abs ? 0 : pz),
              parsenumber('E', (mode_abs ? pe : 0)) + (mode_abs ? 0 : pe) );
        break;
      }
    case  4:
      pause(parsenumber('P', 0) * 1000);
      break; // dwell
    case 90:
      mode_abs = 1;
      break; // absolute mode
    case 91:
      mode_abs = 0;
      break; // relative mode
    case 92:  // set logical position
      position( parsenumber('X', 0),
                parsenumber('Y', 0),
                parsenumber('Z', 0),
                parsenumber('E', 0) );

      break;



    case 28:
      home_x();
      break;

    case 29:
      home_yn();
      break;


    case 27:
      home_z();
      break;

    case 30:
      home_all();
      break;


      break;
    case 40:
      z_step(10);
      break;



    case 100:
      x_step(1);
      break;

    case 101:
      x_step(10);
      break;


    case 200:
      x_step(-1);
      break;

    case 201:
      x_step(-10);
      break;

    default:
      break;
  }

  cmd = parsenumber('M', -1);
  switch (cmd) {
    case  17:
      motor_enable();
      break;
    case  18:
      motor_disable();
      break;
    case 100:
      help();
      break;
    case 114:
      where();
      break;
    case 50:
      p_step(1);
      break;
    case 51:
      p_step(-1);
      break;
    case 60:
      pen_up();
      break;
    case 61:
      pen_dn();
      break;

    case 80:
      y_alone_step(5);
      break;

    case 90:
      y_alone_step(-5);
      break;

    case 119:
      endstop_status();
      break;




    case 40:
      motor_enable_x();
      break;
    case 41:
      motor_disable_x();
      break;
    case 500:
      motor_enable_y();
      break;
    case 501:
      motor_disable_y();
      break;


    case 401:
      execute_time = 1;
      break;

    //digis

    case 0:
      zero(0, 0);
      break;


    case 1:
      one(0, 0);
      break;


    case 2:
      two(0, 0);
      break;

    case 3:
      three(0, 0);
      break;

    case 4:
      four(0, 0);
      break;

    case 5:
      five(0, 0);
      break;

    case 6:
      sixx(0, 0);
      break;

    case 7:
      seven(0, 0);
      break;

    case 8:
      eight(0, 0);
      break;

    case 9:
      nine(0, 0);
      break;
    case 10:
      erase(0, 0);
      break;

    case 12:
      colon(0, 0);
      break;
    case 14:
      erase_line(0, 0);
      break;

    case 20:// printing 1 to 9 in a sequence
      all_test(0, 0);
      break;

    case 21: // all digits in a single line
      all_digits(0, 0);
      break;
    case 22:// check starting time
      starttime();
      break;

    case 23:// return current time according to code
      current_time();
      break;






    default:
      break;
  }
}


/**
   prepares the input buffer to receive a new message and tells the serial connected device it is ready for more.
*/
void ready() {
  sofar = 0; // clear input buffer
  Serial.print(F(">"));  // signal ready to receive input
}

void home_where() {

  Serial.println(x_homestate);
  Serial.println(y_homestate);
}

void endstop_status() {
  int x_home = analogRead(x_stop);

  int y_home = analogRead(y_stop);
  int z_home = digitalRead(z_stop);

  Serial.print("x:");
  Serial.println(x_home);

  Serial.print("y:");
  Serial.println(y_home);

  Serial.print("z:");
  Serial.println(z_home);

  Serial.print("mode");
  Serial.println(mode_abs);

  Serial.print("x_home :");
  Serial.println(x_homestate);

  Serial.print("y_home :");
  Serial.println(y_homestate);

  Serial.print("z_home :");
  Serial.println(z_homestate);

}



// DIGITS

void block(int num_times) {
  int stp = 20;

  int i;
  for (i = 0; i < num_times; ++i) {

    line(px, py, pz + stp, 0);
    pen_dn();
    line(px, py, pz - stp, 0);
    pen_up();
  }
}


void start_block() {
  int stp = 20;



  line(px, py, pz + stp, 0);
  pen_dn();
  line(px, py, pz - stp, 0);
  pen_up();
  line(px, py, pz + stp, 0);
  pen_dn();
  line(px, py, pz - stp, 0);


}
void start_block2() {
  int stp = 10;



  line(px, py, pz + stp, 0);
  pen_dn();
  line(px, py, pz - stp, 0);
  pen_up();
  line(px, py, pz + stp, 0);
  pen_dn();
  line(px, py, pz - stp, 0);


}

// 1 approved OK core xy
void one(int xx, int zz) {
  line(px + xx, py, pz + zz, 0);
  line(px, py, pz + 75, 0);
  start_block();

  line(px, py, pz - 75, 0);

  pen_up();
}

// 2 approved ok core xy
void two(int xx, int zz) {
  line(px + xx, py, pz + zz, 0);

  line(px + 10, py, pz, 0);
  block(2);
  line(px + 10, py, pz, 0);
  block(2);
  line(px + 10, py, pz, 0);
  block(2);



  line(px - 40, py, pz + 60, 0);
  block(2);



  line(px + 10, py, pz + 10, 0);



  block(2);
  line(px + 10, py, pz, 0);
  block(2);
  line(px + 10, py, pz , 0);
  block(2);


  line(px + 10, py, pz - 10, 0);
  pen_half_up();
  start_block();
  line(px + 10, py, pz - 10, 0);
  //block(2);
  //pen_dn();
  line(px - 40, py, pz - 50, 0);
  pen_up();
}


//3 approved ok finally!! changed core xy

void three (int xx, int zz) {
  int x_shift = 35;
  //line(xx, xx, zz, 0);
  line(px + xx, py, pz + zz, 0);
  block(2);
  line(px + x_shift / 3, py, pz, 0);
  block(2);
  line(px + x_shift / 3, py, pz, 0);
  block(2);

  line(px , py , pz + 30, 0);
  block(2);
  line(px - x_shift / 3, py, pz, 0);
  block(2);


  line(px - x_shift / 3, py, pz + 45, 0);


  //pen_quat_dn();
  block(2);


  line(px + x_shift / 3, py, pz, 0);
  block(2);
  line(px + x_shift / 3, py, pz, 0);
  block(2);
  //pen_half_up();

  line(px + x_shift / 3, py, pz , 0);
  start_block();

  line(px  , py , pz - 75, 0);
  pen_up();


  line(px - (x_shift + 1), py, pz , 0);



}

//4 approved OK corexy
void four(int xx, int zz) {

  int x_shift = 35;

  line(px + xx, py, pz + zz, 0);
  line (px , py , pz + 75, 0);
  start_block();
  line(px , py , pz - 40, 0);
  pen_up();

  line(px + x_shift / 2, py, pz, 0);
  block(2);

  line (px + x_shift / 2, py, pz + 30, 0);
  start_block();
  line(px, py, pz - 65, 0);
  pen_up();


  line(px - x_shift, py, pz, 0);

}

//5 apprroved ok corexy

void five(int xx, int zz) {
  int x_shift = 35;

  line(px + xx, py, pz + zz, 0);
  line(px + x_shift, py, pz + 70, 0);
  block(2);
  line(px - x_shift / 2, py, pz, 0);
  block(2);

  line(px - x_shift / 2, py, pz, 0);
  start_block();
  line(px, py, pz - 20, 0);

  line(px + x_shift, py, pz - 20, 0);
  line(px, py, pz - 30, 0);
  pen_up();

  line(px - x_shift / 2, py, pz, 0);
  block(2);
  line(px - x_shift / 2, py, pz, 0);
  block(2);

  line(px - 2, py , pz, 0);//error




}



//6 approved ok CHANGED corexy error remove
void sixx(int xx, int zz) {
  int x_shift = 40;

  line(px + xx, py, pz + zz, 0);
  line(px + (2 * (x_shift / 3)), py, pz + 35, 0);
  block(2);

  line(px , py  , pz + 40, 0);
  start_block2();

  line(px - (2 * (x_shift / 3)), py, pz - 35, 0);
  //pen_half_up();//error
  line(px, py, pz - 40, 0);
  pen_up();

  line(px + x_shift, py, pz + 35, 0);
  start_block2();
  //pen_quat_dn();
  line(px, py, pz - 35, 0);
  pen_up();
  line(px - x_shift / 3, py, pz, 0);
  block(2);
  line(px - x_shift / 3, py, pz, 0);
  block(2);

  //line(px - ((x_shift / 2) - 10 ), py - ((x_shift / 2) - 10 ), pz, 0); // shift error no idea why
  line(px - x_shift / 3, py, pz, 0);

}

//7 ok corexy

void seven(int xx , int zz) {

  int x_shift = 38;
  line(px + xx, py, pz + zz, 0);

  line(px, py, pz + 75, 0);
  block(2);


  line(px + x_shift / 3, py, pz , 0);
  block(2);
  line(px +  x_shift / 3, py, pz, 0);
  block(2);
  line(px +  x_shift / 3, py, pz, 0);
  start_block();

  line(px - x_shift, py, pz - 75, 0);
  pen_up();
}



//8 approved ok corexy

void eight(int xx, int zz) {
  int x_shift = 35;

  line(px + xx, py, pz + zz, 0);
  line(px , py , pz + 65, 0);
  start_block();
  line(px, py, pz - 10, 0);

  line(px + x_shift, py, pz - 30, 0);
  line(px, py, pz - 25, 0);
  pen_up();

  line(px - (x_shift / 2), py, pz + 70, 0);
  block(3);
  line(px + (x_shift / 2), py, pz - 5, 0);
  start_block();

  line(px, py, pz - 10, 0);
  line(px - x_shift, py, pz - 30, 0);
  line(px, py, pz - 25, 0);
  pen_up();
  line(px + (x_shift / 2), py, pz, 0);
  block(2);
  line(px - (x_shift / 2), py, pz, 0);
  line(px - 5, py, pz, 0);//error
}

//9 approved OK corexy


void nine(int xx, int zz) {

  int x_shift = 35;

  line(px + xx, py, pz + zz, 0);
  line(px , py , pz + 75, 0);
  start_block();
  line(px, py, pz - 40, 0);
  pen_up();
  line(px + x_shift / 2, py, pz, 0);
  block(2);
  line(px, py, pz + 40, 0);
  block(2);
  line(px + x_shift / 2, py, pz, 0);
  start_block();

  line(px, py, pz - 75, 0);
  pen_up();

  line(px - x_shift / 2, py, pz, 0);
  block(2);
  line(px - x_shift / 2, py, pz, 0);
  block(2);


}

//0 approved ok corexy

void zero(int xx, int zz) {
  int x_shift = 36;

  line(px + xx, py, pz + zz, 0);

  line(px , py , pz + 75, 0);
  start_block();

  line(px , py , pz - 75, 0);
  pen_up();
  //pen_quat_up(); //error
  line(px + x_shift / 3 , py , pz + 75, 0);
  block(2);
  line(px + x_shift / 3 , py , pz, 0);
  block(2);
  line(px + x_shift / 3 , py, pz, 0);
  start_block();

  line(px , py , pz - 75, 0);
  pen_up();

  line(px - x_shift / 3 , py, pz, 0);
  block(2);
  line(px - x_shift / 3 , py, pz, 0);
  block(2);


  line(px - x_shift / 3 , py , pz, 0);
  line(px - 2 , py, pz, 0); //error
}

// colon APPROVED OK corexy
void colon(int xx, int zz) {
  line(px + xx, py, pz + zz, 0);
  line(px, py, pz + 55, 0);
  start_block();
  line(px, py, pz - 12, 0);
  pen_up();


  //pen_half_up();

  line(px, py, pz - 25, 0);
  block(2);

  line(px, py, pz - 18, 0);


}

// erase approved ok corexy

void erase(int xx , int zz) {
  int x_shift = 40;
  int z_shift = 95;
  line(px + xx, py, pz + zz, 0);
  line(px, py, pz - 5, 0);
  pen_dn();
  line(px, py, pz + z_shift, 0);
  pen_up();
  line(px + x_shift / 3, py, pz - z_shift, 0);
  pen_dn();
  line(px, py, pz + z_shift, 0);
  pen_up();
  line(px + x_shift / 3, py, pz - z_shift, 0);
  pen_dn();
  line(px, py, pz + z_shift, 0);
  pen_up();
  line(px + x_shift / 3, py, pz - z_shift, 0);
  pen_dn();
  line(px, py, pz + z_shift, 0);
  pen_up();
  line(px - x_shift, py, pz - (z_shift - 5), 0);




}

//erase single line ok corexy
void erase_line(int xx , int zz) {
  int z_shift = 95;
  line(px + xx, py, pz + zz, 0);

  line(px, py, pz - 5, 0);
  pen_dn();
  line(px, py, pz + z_shift, 0);
  pen_up();
  line(px, py, pz - (z_shift - 5), 0);


}

void all_test(int xx , int zz) {

  erase(0, 0);
  one(0, 0);
  erase_line(0, 0);
  two(0, 0);
  erase(0, 0);
  three(0, 0);
  erase(0, 0);
  four(0, 0);
  erase(0, 0);
  five(0, 0);
  erase(0, 0);
  sixx(0, 0);
  erase(0, 0);
  seven(0, 0);
  erase(0, 0);
  eight(0, 0);
  erase(0, 0);
  nine(0, 0);
  erase(0, 0);
  zero(0, 0);



}

// test absolute position with corexy

void all_digits(int xx , int zz) {

  //int m2_pos = 172;
  //int m1_pos = 112;
  //int c_pos = 85;

  //int h1_pos = 0;
  //int h2_pos = 24;



  // goto digit 1
  line(px + h1_pos, py, pz, 0);
  one(0, 0);

  // goto digit 2

  line( h2_pos, py, pz + zz, 0);
  zero(0, 0);

  // goto colon
  line( c_pos, py, pz + zz, 0);
  colon(0, 0);

  // goto digit 3

  line( m1_pos, py, pz + zz, 0);
  three(0, 0);

  // goto digit 4
  line( m2_pos, py, pz + zz, 0);
  sixx(0, 0);



}

/**
   set up the pins for each motor
   Pins fits a Ramps 1.4 board
*/
void motor_setup() {
  motors[0].step_pin = 5;//2;
  motors[0].dir_pin = 2;//5;
  motors[0].enable_pin = 8;
  motors[0].limit_switch_pin = 11;

  motors[1].step_pin = 6;//;
  motors[1].dir_pin = 3;//;//;//6;fliped pin with fade led
  motors[1].enable_pin = 8;
  motors[1].limit_switch_pin = 10;

  motors[2].step_pin = 7;//;
  motors[2].dir_pin = 4;//7;//;//6;fliped pin with fade led
  motors[2].enable_pin = 8;
  motors[2].limit_switch_pin = 10;



  int i;
  for (i = 0; i < NUM_AXIES; ++i) {
    // set the motor pin & scale
    pinMode(motors[i].step_pin, OUTPUT);
    pinMode(motors[i].dir_pin, OUTPUT);
    pinMode(motors[i].enable_pin, OUTPUT);
  }
}


void motor_enable() {
  int i;
  for (i = 0; i < NUM_AXIES; ++i) {
    digitalWrite(motors[i].enable_pin, LOW);
  }



}

void motor_enable_x() {

  digitalWrite(motors[0].enable_pin, LOW);

}
void motor_disable_x() {

  digitalWrite(motors[0].enable_pin, HIGH);

}

void motor_enable_y() {

  digitalWrite(motors[1].enable_pin, LOW);

}
void motor_disable_y() {

  digitalWrite(motors[1].enable_pin, HIGH);

}




void motor_disable() {
  int i;
  for (i = 0; i < NUM_AXIES; ++i) {
    digitalWrite(motors[i].enable_pin, HIGH);
  }
}




void onestep(int motor) {


  digitalWrite(motors[motor].step_pin, HIGH);
  digitalWrite(motors[motor].step_pin, LOW);
  delayMicroseconds(500);
}


void onestep_y(int motor) {


  digitalWrite(motors[motor].step_pin, HIGH);
  digitalWrite(motors[motor].step_pin, LOW);
  delayMicroseconds(195);
}

void onestep_f(int motor) {


  digitalWrite(motors[motor].step_pin, HIGH);
  digitalWrite(motors[motor].step_pin, LOW);
  delayMicroseconds(2000);
}

void home_all() {

  home_x();
  line(px + start_offset_y, py, pz, 0);
  home_yn();
  home_x();

}

void home_x() {

  x_homestate = false;

  int x_home = analogRead(x_stop);
  digitalWrite(motors[0].dir_pin, LOW);
  digitalWrite(motors[1].dir_pin, LOW);

  while (x_home < 900 ) {
    int x_home = analogRead(x_stop);

    if (x_homestate == true) {
      //motor_disable();
      break;
    }
    if (x_home > 900) {

      px = 0;
      py = 0;
      //line(start_offset, start_offset, pz, 0);

      px = 0;
      py = 0;

      x_homestate = true;
      break;
    }
    onestep(0);
    onestep(1);

  }
}
// y home needs to be checked with corexy
void home_yn() {

  y_homestate = false;

  int y_home = analogRead(y_stop);
  digitalWrite(motors[1].dir_pin, LOW);

  while (y_home < y_peak) {
    int y_home = analogRead(y_stop);

    if (y_homestate == true) {
      //motor_disable();
      break;
    }
    if (y_home > y_peak) {

      px = 0;
      py = 0;
      line(px, py + y_back, 0, 0);//5
      px = 0;
      py = 0;
      y_homestate = true;
      break;
    }

    onestep_f(1);

  }
}


void home_z() { // try if method

  digitalWrite(motors[2].dir_pin, HIGH);
  z_homestate = false;
  int z_home = digitalRead(z_stop);

  while (z_home == 1) {
    int z_home = digitalRead(z_stop);

    if (z_homestate == true) {
      // motor_disable();
      break;
    }
    if (z_home == 0) {
      pz = 0;
      line(px, py, -start_offset_z, 0);
      //py = 0;
      pz = 0;
      z_homestate = true;
      break;
    }
    onestep_f(2);

  }



}

float mapf(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}



void y_left(int angle) {
  servo_y.write(90 - angle);
  delay(hit_delay);
  servo_y.write(90);
}

void y_left_n(int angle) {

  servo_y.write(90 - angle);
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= hit_delay) {
    previousMillis = currentMillis;
    //delay(hit_delay);
    servo_y.write(90);

  }
}


void servo_xy(int u, int v) {
  servo_x.write(90 + u);
  servo_y.write(90 + v);
  delay(hit_delay);
  servo_x.write(90);
  servo_y.write(90);

}

// step x forward n
void x_step(int s) {

  line(px + (1 * s), py , pz, 0);

}
//pen stepping
void p_step(int s) {

  line(px, py - (1 * s) , pz, 0);
}

void pen_up() {
  p_step(-pen_height);
}

void pen_dn() {
  p_step(pen_height);
}

void pen_half_dn() {
  p_step(pen_height / 2);
}

void pen_half_up() {
  p_step(-pen_height / 2);
}

void pen_quat_dn() {
  p_step(pen_height / 4);
}

void pen_quat_up() {
  p_step(-pen_height / 4);
}

void z_step(int s) {
  line(px, py , pz + (10 * s) , 0);
}

void y_alone_step(int s) {
  line(px, py + s , pz , 0);
}



/**
   First thing this machine does on startup.  Runs only once.
*/
void setup() {
  Serial.begin(BAUD);  // open coms
  servo_y.attach(9);
  servo_x.attach(10);
  pinMode(x_stop, INPUT);
  servo_y.write(90);
  servo_x.write(90);

  pinMode(y_stop, INPUT);
  motor_setup();
  motor_disable();


  where();  // for debugging purposes
  help();  // say hello
  position(0, 0, 0, 0); // set starting position
  feedrate(100);  // set default speed
  ready();
  setTime(10, 36, 00, 12, 19, 2019);
  //home_all();
  // home_z();
  motor_enable();
  home_z();
  home_all();
  sudo_time();
  motor_disable();

}


/**
   After setup() this machine will repeat loop() forever.
*/
void loop() {
  // listen for serial commands
  // motor_enable(); // do something with the command





  while (Serial.available() > 0) { // if something is available
    char c = Serial.read(); // get it
    Serial.print(c);  // repeat it back so I know you got the message
    if (sofar < MAX_BUF - 1) buffer[sofar++] = c; // store it
    if (c == ';') {
      // entire message received
      buffer[sofar] = 0; // end the buffer so string functions work right
      Serial.print(F("\r\n"));  // echo a return character for humans
      processCommand();
      //starttime();
      //printtime();
      ready();




    }
  }
}
