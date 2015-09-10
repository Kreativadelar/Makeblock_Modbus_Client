#include <Mudbus.h>
#include <PID_v1.h>
#include <Ethernet.h>
#include <SPI.h>
#include <VarSpeedServo.h>

/*************************************************************************/
/*
 *      rotary_sm.ino   --  State machine implementation of rotary encoder driver
 *                          Interrupt driven; supports 2 types of interrupts:
 *                            * Polled interrupts; enable by defining TIMER2INT
 *                            * Pin change interrupts: enalbe by defining PINCHANGEINT
 *                              (Do NOT enable both at the same time)
 *
 *          This program is developed from the code at: 
 *              http://www.buxtronix.net/2011/10/rotary-encoders-done-properly.html
 *          Since this code was in the form of an arduino library, and I had unresolvable
 *          references that I could not figure out, I just modified it as a single sketch.
 *          The only library support required is if you want to use interrupt polling,
 *          and then the MsTimer2 library needs to be installed.
 */

/* Define PINCHANGEINT if you want to interrupt on any encoder pin change */
#define PINCHANGEINT
/* --- OR --- */
/* Define TIMER2INT if you want to use periodic interrupts to poll the encoder */
//#define TIMER2INT

/* Define ENABLEPULLUPS if there are no external pull-ups on encoder AB pins */
//#define ENABLEPULLUPS

/* Define to enable the ISR debug flag */
//#define ISRFLAG


/* You may need to install this library (MsTimer2) from the arduino site */
#ifdef TIMER2INT
#include    <MsTimer2.h>
#endif


#define DIR_NONE 0x00           // No complete step yet.
#define DIR_CW   0x10           // Clockwise step.
#define DIR_CCW  0x20           // Anti-clockwise step.

unsigned int state;
unsigned int A = 2;             // pins connected to the encoder (digital_pin 2)
unsigned int B = 3;             //              "                (digital_pin 3)
unsigned int ISRflag = 5;       //              "                (digital_pin 3)
         int count = 0;         // count each indent
         int old_count = 0;     // check for count changed


/*
 * The below state table has, for each state (row), the new state
 * to set based on the next encoder output. From left to right in,
 * the table, the encoder outputs are 00, 01, 10, 11, and the value
 * in that position is the new state to set.
 */

// State definitions state table (emits a code at 00 only)
// states are: (NAB) N = 0: clockwise;  N = 1: counterclockwiswe
#define R_START     0x3
#define R_CW_BEGIN  0x1
#define R_CW_NEXT   0x0
#define R_CW_FINAL  0x2
#define R_CCW_BEGIN 0x6
#define R_CCW_NEXT  0x4
#define R_CCW_FINAL 0x5

const unsigned char ttable[8][4] = {
    {R_CW_NEXT,  R_CW_BEGIN,  R_CW_FINAL,  R_START},                // R_CW_NEXT
    {R_CW_NEXT,  R_CW_BEGIN,  R_CW_BEGIN,  R_START},                // R_CW_BEGIN
    {R_CW_NEXT,  R_CW_FINAL,  R_CW_FINAL,  R_START | DIR_CW},       // R_CW_FINAL
    {R_START,    R_CW_BEGIN,  R_CCW_BEGIN, R_START},                // R_START
    {R_CCW_NEXT, R_CCW_FINAL, R_CCW_BEGIN, R_START},                // R_CCW_NEXT
    {R_CCW_NEXT, R_CCW_FINAL, R_CCW_FINAL, R_START | DIR_CCW},      // R_CCW_FINAL
    {R_CCW_NEXT, R_CCW_BEGIN, R_CCW_BEGIN, R_START},                // R_CCW_BEGIN
    {R_START,    R_START,     R_START,     R_START}                 // ILLEGAL
};


class Motor{

  public:
    Motor(uint8_t dir_pin, uint8_t speed_pin);
    void  run(int speed);
    void  stop();

  private:
    uint8_t _dir = 0;
    uint8_t _speed = 0;
};

Motor::Motor(uint8_t dir_pin, uint8_t speed_pin)
{
   _dir = dir_pin;
   _speed = speed_pin; 
}


void Motor::run(int speed)
{
    speed = speed > 255 ? 255 : speed;
    speed = speed < -255 ? -255 : speed;

    if(speed >= 0)
    {
        pinMode(_dir, OUTPUT);
        digitalWrite(_dir, HIGH);
        analogWrite(_speed, speed);
    }
    else
    {
        pinMode(_dir, OUTPUT);
        digitalWrite(_dir, LOW);
        analogWrite(_speed, -speed);
    }
}

void Motor::stop()
{
    Motor::run(0);
}


/**********************************************************************************/
/****************************   PID    ********************************************/
//Define Variables we'll be connecting to
double Setpoint, Input, Output;
unsigned long serialTime;   

double Kp=30, Ki=0, Kd=0.08;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

/**********************************************************************************/
/****************************   SERVO    ******************************************/
VarSpeedServo myservo;

int SERVO_PIN             = A1;
int SERVO_UPPER_POS       = 0;
int SERVO_LOWER_POS       = 60;
int SERVO_DEFUALT_SPEED   = 20;

/**********************************************************************************/
/****************************   TURN MOTOR   **************************************/

int M1_LIMIT_PIN          = A0;
int M1_SPEED_PIN          = 6; 
int M1_DIR_PIN            = 7;
int M1_UNCALIB_SPEED      = 50;

//int sensorValue = 0;  // variable to store the value coming from the sensor
//int motorOutput = 0;

Motor m1(M1_DIR_PIN, M1_SPEED_PIN);

/**********************************************************************************/
/*******************************    Modbus   **************************************/

Mudbus Mb;

/**********************************************************************************/

void setup() {
  // Axis 3
  myservo.attach(SERVO_PIN);                                  // attaches the servo on pin 9 to the servo object 
  myservo.write(SERVO_UPPER_POS, SERVO_DEFUALT_SPEED, true);  // tell servo to go to upper position  
  
  delay(1000);
  Serial.begin(9600);
  Serial.print("Attaching interrupts");
  /**********************************************************/
  pinMode( A, INPUT );
  pinMode( B, INPUT );

#ifdef ENABLEPULLUPS
    digitalWrite( A, HIGH );                // set pullups
    digitalWrite( B, HIGH );                //      "
#endif

#ifdef TIMER2INT
    MsTimer2::set( 1, T2_isr );             // interrupt polling:
    MsTimer2::start( );
#endif

#ifdef PINCHANGEINT
    attachInterrupt( 0, AB_isr, CHANGE );   // pin-change interrupts: 
    attachInterrupt( 1, AB_isr, CHANGE );
#endif

#ifdef ISRFLAG
    pinMode( ISRflag, OUTPUT );             // time the ISR
    digitalWrite( ISRflag, HIGH );          // set pull-up ON
#endif

    state = (digitalRead( A ) << 1) | digitalRead( B );     // Initialise state.
    old_count = 0;
  /**********************************************************/
  delay(1000);

  // PID
  Setpoint = 0;
  myPID.SetOutputLimits(-255, 255);
  myPID.SetMode(AUTOMATIC);

  // Axis 2
  pinMode(M1_LIMIT_PIN, INPUT_PULLUP);                        // Setup the limit switch with an internal pull-up :

  // Modbus
  byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xEF };  // Define MAC address
  byte ip[] = { 192, 168, 0, 2 };                      // Define IP address
  byte subnet[] = { 255, 255, 255, 0 };                 // Define Subnet mask
  Ethernet.begin(mac, ip, subnet);                      // Assign MAC, IP, and subnet mask
}

#ifdef PINCHANGEINT
void AB_isr( ) {
    // Grab state of input pins.
    unsigned char pinstate = (digitalRead( A ) << 1) | digitalRead( B );

    // Determine new state from the pins and state table.
    state = ttable[state & 0x07][pinstate];

    if( state & DIR_CW )    count++;
    if( state & DIR_CCW )   count--;
}
#endif


#ifdef TIMER2INT
void T2_isr( ) {

#ifdef ISRFLAG
    digitalWrite( ISRflag, HIGH );
#endif

    // Grab state of input pins.
    unsigned char pinstate = (digitalRead( A ) << 1) | digitalRead( B );

    // Determine new state from the pins and state table.
    state = ttable[state & 0x07][pinstate];

    if( state & DIR_CW )    count++;        // count up for clockwise
    if( state & DIR_CCW )   count--;        // count down for counterclockwise

#ifdef ISRFLAG
    digitalWrite( ISRflag, LOW );
#endif
}
#endif

void loop() {
  //static float deg = 0;
  //static double motorOutput = 0;
  //static double motorLimit = 255;
  //static bool calibDone = false;
  Mb.Run(); // start MbTcp subroutine

  // Controll
  static int Q_Axis_2_Pos           = 0;
  static int Q_Axis_2_Vel           = 0; 
  static int Q_Axis_2_Acc           = 0;
  static int Q_Axis_2_Decel         = 0;
  static int Q_Axis_2_Trans         = 0.85714;
  static int Q_Axis_2_Home_Off      = 0;
  
  static bool Q_Axis_2_Exec         = false;
  static bool Q_Axis_2_Reset        = false;
  static bool Q_Axis_2_Mode         = false;
  static bool Q_Axis_2_Forw         = false;
  static bool Q_Axis_2_Stop         = true;
  static bool Q_Axis_2_Rev          = false;
  static bool Q_Axis_2_Homing      = false;
  static bool Q_Axis_2_Auto_Homing  = false;

  // Status
  static int I_Axis_2_Pos            = 0;
  static int I_Axis_2_Vel            = 0;
  static int I_Axis_2_Trans          = 0;
  static int I_Axis_2_Home_off       = 0;
  static int I_Axis_2_ErrorId        = 0;

  static bool I_Axis_2_Ready         = false;
  static bool I_Axis_2_Busy          = false;
  static bool I_Axis_2_Stop          = false;
  static bool I_Axis_2_Pos_Done      = false;
  static bool I_Axis_2_Home          = false;
  static bool I_Axis_2_Mode          = false;
  static bool I_Axis_2_Error         = false;

/*
  static int  modbus_axis2_in_pos     = 0;
  static bool modbus_axis2_in_start   = false;
  static bool modbus_axis2_in_stop    = true;
  static int  I_Axis_2_Pos   = 255;     
  static bool modbus_axis2_in_read_PID    = false;   
  static int  modbus_axis2_in_P       = 30;
  static int  modbus_axis2_in_I       = 0;
  static int  modbus_axis2_in_D       = 0.88;
  static bool modbus_axis2_in_calib   = false;
  static int  modbus_axis2_out_pos;
  static int  modbus_axis2_out_speed;
  static bool modbus_axis2_out_calib  = false;
  */

/**********************************************************************************/
/****************************   Read Modbus   *************************************/  

  if(Mb.Active){
    // Axis 2
    Q_Axis_2_Pos           = Mb.R[8];
    Q_Axis_2_Vel           = Mb.R[9]; 
    Q_Axis_2_Acc           = Mb.R[10];
    Q_Axis_2_Decel         = Mb.R[11];
    Q_Axis_2_Trans         = Mb.R[12];
    Q_Axis_2_Home_Off      = Mb.R[13];
  
    Q_Axis_2_Exec         = Mb.C[8];
    Q_Axis_2_Reset        = Mb.C[9];
    Q_Axis_2_Mode         = Mb.C[10];
    Q_Axis_2_Forw         = Mb.C[11];
    Q_Axis_2_Stop         = Mb.C[12];
    Q_Axis_2_Rev          = Mb.C[13];
    Q_Axis_2_Homing       = Mb.C[14];
    Q_Axis_2_Auto_Homing  = Mb.C[15];
  }
/**********************************************************************************/
  // Axis 2
  // Read param
  I_Axis_2_Stop = Q_Axis_2_Stop;
  
  // Check if homing should be done
  if(!I_Axis_2_Home && Q_Axis_2_Auto_Homing && I_Axis_2_Mode){
    // Auto homing
    I_Axis_2_Vel = M1_UNCALIB_SPEED;
    if(digitalRead(M1_LIMIT_PIN) == LOW){
      I_Axis_2_Vel = 0;
      count = Q_Axis_2_Home_Off;//-175;
      //Setpoint = 0;
      I_Axis_2_Home = true;
    }   
  }else if(!I_Axis_2_Home && Q_Axis_2_Homing && !I_Axis_2_Mode){
      // Man homing
      I_Axis_2_Vel = 0;
      count = Q_Axis_2_Home_Off;//-175;
      I_Axis_2_Home = true;
  }else{
    // Auto mode
    if(!Q_Axis_2_Stop && Q_Axis_2_Exec && I_Axis_2_Mode){
      Setpoint = Q_Axis_2_Pos;
      
      // Calculate current position with encoder value
      if( old_count != count ) {
          I_Axis_2_Pos = float(count) * 0.85714;
          old_count = count;
      }
    
      // Get PID input and compute new output
      Input = I_Axis_2_Pos;
      myPID.Compute();
    
      // Send output to motor with current max speed settings
      I_Axis_2_Vel = (Output*-1); 
      //motorLimit = map(analogRead(sensorPin), 0, 974, 0, 255);
      
      if(I_Axis_2_Vel >= 0){
        I_Axis_2_Vel = constrain(I_Axis_2_Vel, 0, Q_Axis_2_Vel);
      }else{
        I_Axis_2_Vel = constrain(I_Axis_2_Vel, (Q_Axis_2_Vel*-1), 0);
      }
    }else if(!I_Axis_2_Stop && !I_Axis_2_Mode){
      // Man mode
      if(Q_Axis_2_Forw && !Q_Axis_2_Rev){
        I_Axis_2_Vel = constrain(I_Axis_2_Vel, 0, Q_Axis_2_Vel);
      }else if(!Q_Axis_2_Forw && Q_Axis_2_Rev){
        I_Axis_2_Vel = constrain(I_Axis_2_Vel, (Q_Axis_2_Vel*-1), 0);
      }else{
        I_Axis_2_Vel = 0;
      }
    }else{ 
      // Stop axis
      I_Axis_2_Vel = 0;
      // If axis is stoped set setpoint to current pos
      if(I_Axis_2_Stop){
        Setpoint = I_Axis_2_Pos;
      }
    }
  }

  m1.run(I_Axis_2_Vel);


/**********************************************************************************/
/****************************   Send Modbus   *************************************/  
  // Axis 2
  Mb.R[16] = I_Axis_2_Pos;
  Mb.R[17] = I_Axis_2_Vel;
  Mb.R[18] = I_Axis_2_Trans;
  Mb.R[19] = I_Axis_2_Home_off;
  Mb.R[20] = I_Axis_2_ErrorId;

  Mb.C[16] = I_Axis_2_Ready;
  Mb.C[17] = I_Axis_2_Busy;
  Mb.C[18] = I_Axis_2_Stop;
  Mb.C[19] = I_Axis_2_Pos_Done;
  Mb.C[20] = I_Axis_2_Home;
  Mb.C[21] = I_Axis_2_Mode;
  Mb.C[22] = I_Axis_2_Error;
/**********************************************************************************/
  /*
  // send-receive with processing if it's time (PID Front End)
  if(millis()>serialTime)
  {
    SerialReceive();
    SerialSend();
    serialTime+=500;
  }
  */
}




