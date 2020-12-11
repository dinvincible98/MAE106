#include "src/mpu.h"
#include <Wire.h>
int ret;

// pin route
#define pinpiston           2
#define pinservo            3
#define pinreedpiston       4
#define pinhall             6
#define pinled              13

// data: store the constants into the program storage rather than take up the memory space

// program configuration
#define SERIAL_COM_ON       true
#define TEST_MODE_ON        true
#define FAST_MODE_ON        true
#define USE_MAG             true

// start configuration
#define Dturn1              900
#define Dturn2              3000
#define DstopPoint          1350

// route configurations
#define DdirOffset          120
#define Dfinalmin           50
#define Dslowdowndis        1000
#define Dnextstepdir        10          // if direction is within +- Dnextstepdir Sstep goes to 2
#define DTshutdown          5000
#define Dvelmin             100         // minimum speed
#define Dvelnorm            200        // normal speed safe for turning(but no fire)
#define Dvelmax             200        // maximum speed but still efficient enough

// PID control parameters
#define Ka                  (0.5F)

// robot configurations
#define DTstart             10000       // time delay before any action
#define DTrun               60000       // time interval that robot can take actions
#define DTpistonfirelen     1200        // duration of piston firing, safe exit if reed swtich is not responsive
#define DTpistonretractlen  300
#define DTreeddelay         50
#define DencoderTic         (42.0F)
#define DrobotCenter        (80.7F)     // half of track width
#define Dsteermax           (22.5F)          // absolute max steering, after which is considered dangerous when traveling in normal speed
#define Dturnradius         500          // turn radius, after which robot begin turning
#define Dsteersmall         5           // steering is small so speed returns to normal and I control is on
#define SERVO_MID           (136.6)         // steering middle value which robot would go in a straight line
#define SERVO_MAX           60          // the maximum steering from steering middle



// global variables
// time related
unsigned long Tstart =      0;          // record the time before any execution of program
unsigned long Tcurrent =    0;          // current time, read from this variable instead of millis() to save space
unsigned long Tpiston =     0;          // the time piston started fire and started retraction
unsigned long Treed =       0;
unsigned long Tshutdown =   0;
unsigned long TreedSpeed =  0;
unsigned long TcurrentMicro=0;
unsigned long TlastMicro =  0;
unsigned long Tprint =      0;

// commands
bool fire =                 false;      // actuation of the piston
bool pistonCom =            false;
bool reedcorrected =        false;
bool hall =                 false;
float  steer =                0;
bool fastEncoder =          false;
bool stages =               0;

// lockup switch


// state slector

// robot datas
float vel =                 0;
float distance =            0;
float distanceRaw =         0;
float angularVel =          0;
float dir =                 0;
float dirRad();                         // return value from dir and converted when called;


// for IMU uses


// functions
void movement               (float Fdestination, float dirLocal, int Fmode);
void speedMode              (int Fmode);

void setup() {
    // initiations
    Tstart = millis();
    pinMode(pinpiston,OUTPUT);
    pinMode(pinservo,OUTPUT);
    pinMode(pinreedpiston,INPUT_PULLUP);
    pinMode(pinhall,INPUT_PULLUP);
    if (SERIAL_COM_ON)
    {
        Serial.begin(115200);
    }
    Wire.begin();
    ret = mympu_open(200);
    Tprint = Tstart;
    
    
    
}
void loop() {
    byte steps = 0;
    int mode = 0;
    float dirLocal;
    float destination = 0;
    // empty loop act as delay
    while(millis()<(Tstart+DTstart))
    {
        mympu_update();
    }
    Tstart = millis();
    Tcurrent = millis();
    while(Tcurrent<(Tstart+DTrun))
    {
        // main action loop
        // get values for each cycle
        Tcurrent = millis();
        mympu_update();
        dir = mympu.ypr[0] - DdirOffset;
        if (dir > 180) dir -= 360;
        else if (dir < -180) dir += 360;
        //main steps
        switch(steps)
        {
            case 0:
            {
                steps ++;
                break;                       // empty case
            }
            case 1:
            {
                //approach center line
                destination = Dturn1;
                dirLocal = 0;
                mode = 2;
                if (abs(dir) < Dnextstepdir) steps ++;
                break;
            }
            case 2:
            {
                // go the lone way
                destination = Dturn2 + Dturn1 - (Dturnradius * (1 - M_PI) / 2);
                dirLocal = 0;
                if ((destination - distance) > Dslowdowndis) mode = 3;
                else
                {
                    mode = 1;
                    steps ++;
                    Tshutdown = Tcurrent;
                }
                break;
            }
            case 3:
            {
                // approach the final horizontal line
                destination = Dturn2 + Dturn1 - (Dturnradius * (1 - M_PI) / 2);
                dirLocal = 90;
                if (abs(90-dir) < Dnextstepdir) steps ++;
                break;
            }
            case 4:
            {
                // stop at the final block
                destination = DstopPoint + Dturn2 + Dturn1 - (Dturnradius * (1 - M_PI));
                dirLocal = 90;
                if (destination - distance < Dfinalmin) steps ++;
                break;
            }
            default:{
                
                break;
            }
        }
        if (steps > 2) if (Tshutdown + DTshutdown > Tcurrent) mode = 0; // delay so the robot would slide into the box
        movement(destination,dirLocal,mode);
        if (steps > 4) break;
        if (SERIAL_COM_ON)
        {
            Serial.print(steps);
            Serial.print("    ");
            Serial.print(dir);
            Serial.print("    ");
            Serial.print(distance);
            Serial.print("    ");
            Serial.println(steer);
        }
        
    }
    Serial.print("complete");
    //make sure the robot is stoped
    digitalWrite(pinpiston,false);
    while(true){}                              // ensure there is no further action
}

// functions

void movement(float Fdestination, float dirLocal, int Fmode = 2){
    float distanceLocal = Fdestination - distance;
    speedCal();
    // calculations
    if (distanceLocal > Dturnradius) steer = 0;
    else steer = (dirLocal - dir) * Ka;
    // limit filter for steer command
    if (steer > Dsteermax) steer = Dsteermax;
    else if (steer < -Dsteermax) steer = -Dsteermax;
    // speed limit when turning
    if (abs(steer) > Dsteersmall){      // decrese speed when turning
        if (Fmode > 1) Fmode = 1;
    }
    //if (abs(steer) < 3) steer = 0;
    speedMode(Fmode);
    steering();
    return;
    
}

void steering(){
    // filter out the command over the limit
    if (steer < -SERVO_MAX){
        steer = -SERVO_MAX;
    }else if (steer > SERVO_MAX){
        steer = SERVO_MAX;
    }
    int steerOut = 0 - steer + SERVO_MID;
    analogWrite(pinservo,steerOut);
}


void speedMode(int Fmode = 0)                        // control actuation and the speed of the robot
{
    static bool Ifire =                 true;
    static bool Iretract =              true;
    static bool Ireed =                 false;
    static bool firstfire =             true;
    static unsigned long TfireDelay =   0;
    // mode selection = 0: slide; 1: turning; 2: normal; 3: fast;
    if (!FAST_MODE_ON) if (Fmode > 2) Fmode = 2;  // fastmode switch
    if (Fmode>3)
    {
        if (firstfire)
        {
            TfireDelay = Tcurrent;
            fire = true;
            firstfire = false;
        }
    }
    else firstfire = true;
    // mode selector
    switch(Fmode){
        case 0:
            fire = false;
            break;
        case 1:
            if (vel < Dvelmin) fire = true;
            break;
        case 2:
            if (vel < Dvelnorm) fire = true;
            break;
        case 3:
            if (vel < Dvelmax) fire = true;
            break;
        default:
        {
            if (fire) TfireDelay = Tcurrent;
            if (Tcurrent - TfireDelay > Fmode) fire = true;
        }
    }
    // fire command
    if (fire){
        if (Ifire){
            pistonCom = true;
            Tpiston = Tcurrent;
            Ifire = false;
        }
        if (Iretract){
            if (reedcorrected||(Tcurrent - Tpiston > DTpistonfirelen))
            {
                pistonCom = false;
                Tpiston = Tcurrent;
                Iretract = false;
            }
        }
        else if (Tcurrent - Tpiston > DTpistonretractlen)
        {
            Ifire = true;
            Iretract = true;
            fire = false;
        }
    }
    else
    {
        Ifire = true;
        Iretract = true;
        pistonCom = false;
    }
    if (pistonCom) digitalWrite(pinpiston,true);
    else digitalWrite(pinpiston,false);
    // reedswitch correction, give a adjustable delay to the reedswitch, to conterract the program delay and eliminate the need to adjust reedswitch's position
    if (Tcurrent - Treed > DTreeddelay){
        if (reedcorrected)
            reedcorrected = false;
        if (Ireed){
            reedcorrected = true;
            Ireed = false;
        }
        else if (!digitalRead(pinreedpiston))
            Treed = Tcurrent;
    }
    else
        Ireed = true;
}


void speedCal()
{
    static float speedLocal = 0;
    static bool Ihall = false;
    TcurrentMicro = micros();
    // count the events of magnet near
    
    hall = digitalRead(pinhall);

    
    if (hall)
    {
        if (Ihall)
        {
            float timeFrac = ((float)TcurrentMicro - (float)TlastMicro) / 1000000.0;
            speedLocal = DencoderTic / timeFrac;
            distanceRaw += speedLocal;
            // with rotation compenstion
            vel = speedLocal;
            distance += vel * timeFrac;
            // ending
            TlastMicro = TcurrentMicro;
            Ihall = false;
        }
    }
    else
    {
        Ihall = true;
    }
}
