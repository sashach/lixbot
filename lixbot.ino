
#include "IRremote.h"
#include "IRremoteInt.h"
#include "Servo.h"

#include <Ultrasonic.h>

#define  REPEAT_DELAY  50   // Delay before checking for another button / repeat

// Ultrasonic
int triggerPin = 6;
int echoPin = 7;

// IR
int receiverPin = 8;

// Servo
int hServoPin = 9;
int vServoPin = 10;

// Motor 1
int dir1PinA = 2;
int dir2PinA = 3;
int speedPinA = 11; // Needs to be a PWM pin to be able to control motor speed

// Motor 2
int dir1PinB = 4;
int dir2PinB = 5;
int speedPinB = 12; // Needs to be a PWM pin to be able to control motor speed

IRrecv irrecv(receiverPin);        // create instance of 'irrecv'
decode_results results;            // create instance of 'decode_results'


//#define RIGHT 1
//#define LEFT  -1

struct OneServo
{
  int pos, pin;
  bool sweep;
  int sweepDirection, sweepStep;
  int maxPos, minPos;
  Servo servo;

  enum
  {
    RIGHT = 1,
    LEFT = -1
  };

  OneServo(const int _pos, const int _minPos, const int _maxPos, const bool _sweep, const int _sweepDirection, const int _sweepStep = 1)
    :
    pos(_pos),
    sweep(_sweep),
    sweepDirection(_sweepDirection),
    sweepStep(_sweepStep),
    maxPos(_maxPos),
    minPos(_minPos)
  {
  }
  attach(const int _pin)
  {
    pin = _pin;
    servo.attach(pin);
    update();
    print();
  }
  invertSweep()
  {
    sweep = !sweep;
  }
  move()
  {
    pos += (sweepStep * sweepDirection);
    update();

    if (pos > maxPos)
      sweepDirection = LEFT;
    else if (pos < minPos)
      sweepDirection = RIGHT;
  }
  update()
  {
    servo.write(pos);
    delay(5);
  }
  update(const int _pos)
  {
    pos = _pos;
    update();
  }
  print()
  {
    Serial.print("Servo info: pin ");
    Serial.print(pin, DEC);
    Serial.print(" pos ");
    Serial.print(pos, DEC);
    Serial.print(" max:min ");
    Serial.print(maxPos, DEC);
    Serial.print(":");
    Serial.print(minPos, DEC);
    Serial.println();
  }
  process()
  {
    if (sweep)
    {
      move();
    }
  }
};

int hServoHome = 90 - 7;
int vServoHome = 90;

OneServo hOneServo(hServoHome, 10, 170, false, OneServo::RIGHT, 5);
OneServo vOneServo(vServoHome, 60, 120, false, OneServo::RIGHT);

Ultrasonic ultrasonic(triggerPin, echoPin);

struct OneMotor
{
  int dir1Pin;
  int dir2Pin;
  int speedPin;

  OneMotor(const int _dir1Pin, const int _dir2Pin, const int _speedPin)
    :
    dir1Pin(_dir1Pin),
    dir2Pin(_dir2Pin),
    speedPin(_speedPin)
  {
  }
  setPinMode()
  {
    pinMode(dir1Pin, OUTPUT);
    pinMode(dir2Pin, OUTPUT);
    pinMode(speedPin, OUTPUT);
  }
  void forward(const int speed = 255)
  {
    analogWrite(speedPin, speed);//Sets speed variable via PWM
    digitalWrite(dir1Pin, LOW);
    digitalWrite(dir2Pin, HIGH);
  }
  void stop()
  {
    analogWrite(speedPin, 0);
    digitalWrite(dir1Pin, LOW);
    digitalWrite(dir2Pin, HIGH);
  }
  void reverse(const int speed = 255)
  {
    analogWrite(speedPin, speed);
    digitalWrite(dir1Pin, HIGH);
    digitalWrite(dir2Pin, LOW);
  }

};

OneMotor leftMotor(dir1PinA, dir2PinA, speedPinA);
OneMotor rightMotor(dir1PinB, dir2PinB, speedPinB);

void setup()   /*----( SETUP: RUNS ONCE )----*/
{
  Serial.begin(9600);
  Serial.println("Lixbot V 1.0");

  irrecv.enableIRIn(); // Start the receiver

  hOneServo.attach(hServoPin);
  vOneServo.attach(vServoPin);

  leftMotor.setPinMode();
  rightMotor.setPinMode();

}/*--(end setup )---*/


void stop()
{
  leftMotor.stop();
  rightMotor.stop();
}


void goForward(const int speed = 255)
{
  hOneServo.update(hServoHome);
  vOneServo.update(vServoHome);

  leftMotor.forward(speed);
  rightMotor.forward(speed);
}

void goReverse(const int speed = 255)
{
  leftMotor.reverse(speed);
  rightMotor.reverse(speed);
}

void turnLeft(const int t = 200)
{
  leftMotor.forward();
  rightMotor.reverse();
  delay(t);
  stop();
}

void turnRight(const int t = 200)
{
  leftMotor.reverse();
  rightMotor.forward();
  delay(t);
  stop();
}

int oneMeasure()
{
  int maxMeasures = 10;
  int d[maxMeasures];
  for (int i = 0; i < maxMeasures; ++i)
  {
    d[i] = ultrasonic.Ranging(CM);

    Serial.print(d[i], DEC);
    Serial.print(" ");
  }
  Serial.println();

  int res = 0;
  for (int i = 0; i < maxMeasures; ++i)
  {
    res += d[i];
  }

  return res / maxMeasures;
}

int hScan()
{
  hOneServo.update(hOneServo.minPos);
  delay(200);

  int rightDist = oneMeasure();

  hOneServo.update(hOneServo.maxPos);
  delay(600);

  int leftDist = oneMeasure();

  hOneServo.update(hServoHome);
  delay(200);

  Serial.print("right - ");
  Serial.print(rightDist, DEC);
  Serial.print("; left - ");
  Serial.print(leftDist, DEC);  Serial.println();

  int res = 0;
  if (rightDist > 40 && rightDist > leftDist)
    res = 1;
  else if (leftDist > 40 && leftDist > rightDist)
    res = -1;

  return rightDist < leftDist;
}

bool goForwardMode = false;
int speed = 255;

void loop()   /*----( LOOP: RUNS CONSTANTLY )----*/
{
  if (irrecv.decode(&results))
  {
    Serial.println(results.value, HEX);

    int ButtonValue = translateIR();
    switch (ButtonValue)
    {
      case 1:
        hOneServo.invertSweep();
        break;
      case 3:
        vOneServo.invertSweep();
        break;
      case 2:
        goForwardMode = true;
        goForward(speed);
        break;
      case 5:
        goForwardMode = false;
        stop();
        break;
      case 8:
        goReverse(speed);
        break;
      case 4:
        if (goForwardMode)
        {
          turnLeft(300);
          goForward(speed);
        }
        break;
      case 6:
        if (goForwardMode)
        {
          turnRight(300);
          goForward(speed);
        }
        break;

      default:
        break;
    }

    Serial.println(ButtonValue, DEC);
    delay(REPEAT_DELAY);    // Adjust for repeat / lockout time
    irrecv.resume(); // receive the next value
  }

  hOneServo.process();
  vOneServo.process();

  if (goForwardMode)
  {
    int dist = ultrasonic.Ranging(CM);
    if (dist < 10)
    {
      stop();

      int scan = hScan();
      if (scan > 0)
      {
        turnRight(300);
        goForward(speed);
      }
      else if (scan < 0)
      {
        turnLeft(300);
        goForward(speed);
      }
      else
      {
        turnLeft(700);
        goForward(speed);
      }
    }
  }

  delay(50);
}/* --(end main loop )-- */


/*-----( Declare User-written Functions )-----*/
int translateIR() // returns value of "Car MP3 remote IR code received
{
  switch (results.value)
  {
    case 0xFFA25D:
      return 10;  // CH-
      break;

    case 0xFF629D:
      return 11; // CH
      break;

    case 0xFFE21D:
      return 12; // CH+
      break;

    case 0xFF22DD:
      return 13; // PREV
      break;

    case 0xFF02FD:
      return 14; // NEXT
      break;

    case 0xFFC23D:
      return 15; //  PLAY/PAUSE
      break;

    case 0xFFE01F:
      return 16; // VOL-
      break;

    case 0xFFA857:
      return 17; // VOL+
      break;

    case 0xFF906F:
      return 18; // EQ
      break;

    case 0xFF6897:
      return 0; // ZERO
      break;

    case 0xFF9867:
      return 100; // 100+
      break;

    case 0xFFB04F:
      return 200; // 200+
      break;

    case 0xFF30CF:
      return 1;  // 1 etc.. to 9
      break;

    case 0xFF18E7:
      return 2;
      break;

    case 0xFF7A85:
      return 3;
      break;

    case 0xFF10EF:
      return 4;
      break;

    case 0xFF38C7:
      return 5;
      break;

    case 0xFF5AA5:
      return 6;
      break;

    case 0xFF42BD:
      return 7;
      break;

    case 0xFF4AB5:
      return 8;
      break;

    case 0xFF52AD:
      return 9; // 9
      break;

    case 0xFFFFFFFF:
      return -2; // REPEAT: Button Held down longer than
      break;
    default:
      return -1; // Other Button  / Bad Code

  } //END case

} //END translateIR



