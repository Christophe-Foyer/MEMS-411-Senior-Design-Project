// Christophe Foyer 2017

//#include <Servo.h> 

//Servo myservo;
//Servo myservo2;

int servoPin = 9;
int servoPin2 = 10;

//set up some global variables for the light level

int lightLevel1; //left
int lightLevel2; //middle
int lightLevel3; //right

int servoPos = 0; //servo position
int servoPos2 = servoPos;
int servoPos1 = servoPos;
int servoMid = servoPos;
int servoRange = 90;

int ledPin1 = 7; //left
int ledPin2 = 6; //middle
int ledPin3 = 5; //right

int tol = 5; //tolerance out of 1024

int servoStep = 1;

void setup()
{
  //Setup serial
  Serial.begin(115200);

  //Setup Analog pins
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);

  pinMode(ledPin1, OUTPUT);
  pinMode(ledPin2, OUTPUT);
  pinMode(ledPin3, OUTPUT);

  //myservo.attach(servoPin);
  //myservo.attach(servoPin2);

  //Serial.println("Servopos (read):");
  //Serial.println(myservo.read());
  //Serial.println("ServoPos:");
  //Serial.println(servoPos);
  //delay(10000);
  
  //myservo.write(servoPos);

  
}


void loop()
{
  //Aquire photoresisor values
  lightLevel1 = analogRead(A0);
  lightLevel2 = analogRead(A1);
  lightLevel3 = analogRead(A2);

  //Set LEDs to off
  digitalWrite(ledPin1, LOW);
  digitalWrite(ledPin2, LOW);
  digitalWrite(ledPin3, LOW);

  //print lightlevels
  Serial.println("Light levels (left/middle/right)");
  Serial.println(lightLevel1);
  Serial.println(lightLevel2);
  Serial.println(lightLevel3);
  Serial.println("Light levels - tolerance (left/right");
  Serial.println(lightLevel1 - tol);
  Serial.println(lightLevel3 - tol);

  //Choose which way to actuate (might want to add deadzone later on
  if ((lightLevel1 - tol > lightLevel2) && (lightLevel3 - tol > lightLevel2))
  {
    Serial.println("case 1");
    if (lightLevel1 - tol > lightLevel3)
    {
      servoPos = servoPos - servoStep;
      digitalWrite(ledPin1, HIGH);
      Serial.println("Moving left");
    }
    else if (lightLevel3 - tol > lightLevel1) 
    {
      servoPos = servoPos + servoStep;
      digitalWrite(ledPin3, HIGH);
      Serial.println("Moving right");
    }
    else
    {
      //green led
      digitalWrite(ledPin2, HIGH);
    }
  }
  else
  {
    Serial.println("case 2");
    if (lightLevel1 - tol > lightLevel2-5) //check for rising edge
       { 
         servoPos = servoPos - servoStep;
         digitalWrite(ledPin1, HIGH);
         Serial.println("Moving left");
       }
    else if (lightLevel3 - tol > lightLevel2-5)
       {
         servoPos = servoPos + servoStep;
         digitalWrite(ledPin3, HIGH);
         Serial.println("Moving right");
       }
    else
       {
         //green led
         digitalWrite(ledPin2, HIGH);
       }
  }

  //Limit servopos

  if (servoMid - servoRange/2 > servoPos)
  { 
    servoPos = servoMid - servoRange/2;
  }
  else if (servoMid + servoRange/2 < servoPos  ) //check for rising edge
  { 
    servoPos = servoMid + servoRange/2;
  }

  //Move servo and wait
  servoPos2 = (2 * servoMid - servoPos);
  servoPos1 = servoPos;
  analogWrite(servoPin,servoPos1); //set new servo position
  analogWrite(servoPin2,servoPos2);
  //myservo.write(servoPos);
  //myservo2.write(servoPos2);
  //Serial.println("Servopos (read):");
  //Serial.println(myservo.read());
  Serial.println("ServoPos:");
  Serial.println(servoPos);
  Serial.println("ServoPos2:");
  Serial.println(servoPos2);
  delay(100); //wait x milliseconds

  
}
