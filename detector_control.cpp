//#include <algorithm>
#include <AFMotor.h>
#include <Ultrasonic.h>

Ultrasonic ultrasonicFL(47,46);
Ultrasonic ultrasonicFR(39,38);
Ultrasonic ultrasonicL(53,52);
Ultrasonic ultrasonicR(43,42);

int coordX = 0;
int coordY = 0;

int numTurnsR = 0;

int swtch = 48;
int encoder4 = 26;
int encoder3 = 24;
int encoder2 = 50;
int encoder1 = 22;

int deg4 = 0;
int deg3 = 0;

int d3 = 0;
int d4 = 0;

int speed4 = 250;
int speed3 = 243.8;

int speed2 = 200;
int speed1 = 200;

int lnth = 247;
int width = 267;

int piezoF = 8;
int piezoB = 13;
int threshold = 6;
int mines = 0;

AF_DCMotor motor3(3); // left motor
AF_DCMotor motor4(4); //right motor

AF_DCMotor motor2(2); //left arm
AF_DCMotor motor1(1); //right arm

int segG = 30;
int segF = 31;
int segA = 32;
int segB = 33;
int segE = 34;
int segD = 35;
int segC = 36;

void numDisp (int n)
{
  for (int i = 30; i <= 36; i++)
  {
    digitalWrite(i,LOW);
  }

  if (n == 0)
  {
    digitalWrite(segA, HIGH);
    digitalWrite(segF, HIGH);
    digitalWrite(segE, HIGH);
    digitalWrite(segD, HIGH);
    digitalWrite(segC, HIGH);
    digitalWrite(segB, HIGH);    
  }
  else if (n == 1)
  {
    digitalWrite(segB, HIGH);
    digitalWrite(segC, HIGH);
  }
  else if (n == 2)
  {
    digitalWrite(segA, HIGH);
    digitalWrite(segB, HIGH);
    digitalWrite(segG, HIGH);
    digitalWrite(segE, HIGH);
    digitalWrite(segD, HIGH);
  }
  else if (n == 3)
  {
    digitalWrite(segA, HIGH);
    digitalWrite(segB, HIGH);
    digitalWrite(segG, HIGH);
    digitalWrite(segC, HIGH);
    digitalWrite(segD, HIGH);
  }
  else if (n == 4)
  {
    digitalWrite(segF, HIGH);
    digitalWrite(segG, HIGH);
    digitalWrite(segB, HIGH);
    digitalWrite(segC, HIGH);    
  }
  else if (n == 5)
  {
    digitalWrite(segA, HIGH);
    digitalWrite(segF, HIGH);
    digitalWrite(segG, HIGH);
    digitalWrite(segC, HIGH);
    digitalWrite(segD, HIGH);
  }
  else if (n == 6)
  {
    digitalWrite(segA, HIGH);
    digitalWrite(segF, HIGH);
    digitalWrite(segG, HIGH);
    digitalWrite(segC, HIGH);
    digitalWrite(segD, HIGH);
    digitalWrite(segE, HIGH);
  }
  else if (n == 7)
  {
    digitalWrite(segA, HIGH);
    digitalWrite(segB, HIGH);
    digitalWrite(segC, HIGH);
  }
  else if (n == 8)
  {
    digitalWrite(segA, HIGH);
    digitalWrite(segB, HIGH);
    digitalWrite(segC, HIGH);
    digitalWrite(segD, HIGH);
    digitalWrite(segE, HIGH);
    digitalWrite(segF, HIGH);
    digitalWrite(segG, HIGH);
  }
  else if (n == 9)
  {
    digitalWrite(segA, HIGH);
    digitalWrite(segB, HIGH);
    digitalWrite(segF, HIGH);
    digitalWrite(segG, HIGH);
    digitalWrite(segC, HIGH);
    digitalWrite(segD, HIGH);
  }
}

int sort_desc(const void *cmp1, const void *cmp2)
{
  double a = *((double *)cmp1);
  double b = *((double *)cmp2);
  return b-a;
}// qsort(array_name, array_length, data_type_in_array(int), sort_desc);

void turn90L () //uses only motor 3's encoder
{
    int deg = 0;
    int deg90 = 495;//
    motor3.run(BACKWARD);
    motor4.run(FORWARD);
    
    int reading;
    int prevReading = digitalRead(encoder3);
    
    while((deg < deg90))
    {
      reading = digitalRead(encoder3);

      if(prevReading != reading)
        deg++;

      prevReading = reading;
    }

    numTurnsR -= 1;

    motor3.run(RELEASE);
    motor4.run(RELEASE);  
}

void turn90R () //uses only motor 3's encoder
{
    int deg = 0;
    int deg90 = 565;
    
    motor3.run(FORWARD);
    motor4.run(BACKWARD);
    
    int reading;
    int prevReading = digitalRead(encoder3);
    
    while((deg < deg90))
    {
      reading = digitalRead(encoder3);

      if(prevReading != reading)
        deg++;

      prevReading = reading;
      
    }

    motor3.run(RELEASE);
    motor4.run(RELEASE);  

    numTurnsR+=1;
}

void turn90RArm() //uses only motor 3's encoder
{
    int deg = 0;
    int deg90 = 500;
    
    motor3.run(FORWARD);
    motor4.run(BACKWARD);
    
    int reading;
    int prevReading = digitalRead(encoder3);
    
    while((deg < deg90))
    {
      reading = digitalRead(encoder3);

      if(prevReading != reading)
        deg++;

      prevReading = reading;
    }

    motor3.run(RELEASE);
    motor4.run(RELEASE);  

    numTurnsR+=1;
}

void forward (int n) //uses only motor 3's encoder
{
    int deg = 0;
    
    motor3.run(FORWARD);
    motor4.run(FORWARD);
    
    int reading;
    int prevReading = digitalRead(encoder3);
    
    while((deg < n))
    {
      reading = digitalRead(encoder3);

      if(prevReading != reading)
        deg++;

      prevReading = reading;

      if (analogRead(piezoF) > threshold)
      {
         mines++;
         numDisp(mines);
      }
      if (analogRead(piezoB) > threshold)
      {
        mines++;
        numDisp(mines);
      }
      
      getCoords();
    }

    motor3.run(RELEASE);
    motor4.run(RELEASE);  
}

void forwardSingle (int n, int m)
{
    int deg = 0;
    if (m == 3)
    {
      motor3.setSpeed(150);
      motor3.run(FORWARD);
    }
    else if (m == 4)
    {
      motor4.setSpeed(150);
      motor4.run(FORWARD);
    }
    
    int reading;
    int prevReading = digitalRead(encoder3);
    
    while((deg < n))
    {
      reading = digitalRead(encoder3);

      if(prevReading != reading)
        deg++;

      prevReading = reading;

      
      if (analogRead(piezoF) > threshold)
      {
         mines++;
         numDisp(mines);
      }
      if (analogRead(piezoB) > threshold)
      {
        mines++;
        numDisp(mines);
      }      
    }
    if (m == 3)
    {
      motor3.run(RELEASE);
      motor3.setSpeed(speed3);
    }
    else if (m == 4)
    {
      motor4.run(RELEASE);
      motor4.setSpeed(speed4); 
    }
}

void backward (int n) //uses only motor 3's encoder
{
    int deg = 0;
    
    motor3.run(BACKWARD);
    motor4.run(BACKWARD);
    
    int reading;
    int prevReading = digitalRead(encoder3);
    
    while((deg < n))
    {
      reading = digitalRead(encoder3);

      if(prevReading != reading)
        deg++;

      prevReading = reading;

      
      if (analogRead(piezoF) > threshold)
      {
         mines++;
         numDisp(mines);
      }
      if (analogRead(piezoB) > threshold)
      {
        mines++;
        numDisp(mines);
      }
      
      getCoords();
    }

    motor3.run(RELEASE);
    motor4.run(RELEASE);  
}

void backwardSingle (int n, int m)
{
    int deg = 0;
    if (m == 3)
    {
      motor3.setSpeed(200);
      motor3.run(BACKWARD);
    }
    else if (m == 4)
    {
      motor4.setSpeed(200);
      motor4.run(BACKWARD);
    }
    
    int reading;
    int prevReading = digitalRead(encoder3);
    
    while((deg < n))
    {
      reading = digitalRead(encoder3);

      if(prevReading != reading)
        deg++;

      prevReading = reading;
      
      if (analogRead(piezoF) > threshold)
      {
         mines++;
         numDisp(mines);
      }
      if (analogRead(piezoB) > threshold)
      {
        mines++;
        numDisp(mines);
      }
    }
    if (m == 3)
    {
      motor3.run(RELEASE);
      motor3.setSpeed(speed3);
    }
    else if (m == 4)
    {
      motor4.run(RELEASE);
      motor4.setSpeed(speed4); 
    }
}

void buttonPress ()
{
  while(digitalRead(swtch) == 0)
  {
  }
  delay(1000);
}

void endL()
{
  backward(140);
  delay(500);
  getCoords();
  turn90L();
  numTurnsR -= 1;
  delay(500);
  getCoords();
  forward(760);
  delay(500);
  getCoords();
  turn90L();
  numTurnsR -= 1;
  delay(500);
  getCoords();
}

void endR()
{
  backward(140);
  delay(500);
  getCoords();
  turn90R();
  numTurnsR += 1;
  delay(500);
  getCoords();
  forward(760);
  delay(500);
  getCoords();
  turn90R();
  numTurnsR += 1;
  delay(500);
  getCoords();
}

void endRArm()
{
  backward(140);
  delay(500);
  getCoords();
  turn90RArm();
  numTurnsR += 1;
  delay(500);
  getCoords();
  forward(760);
  delay(500);
  getCoords();
  turn90RArm();
  numTurnsR += 1;
  delay(500);
  getCoords();
}

void varEndL()
{
  backward(140);
  delay(500);
  //getCoords();
  turn90L();
  numTurnsR -= 1;
  delay(500);
  getCoords();
  
  fwdWall(10);
  
  delay(500);
  getCoords();
  turn90L();
  numTurnsR -= 1;
  delay(500);
  getCoords();
}

void varEndR()
{
   backward(140);
  //delay(500);
  //getCoords();
  turn90R();
  //numTurnsR += 1;
  delay(500);
  //getCoords();
  
  fwdWall(10);
  
  delay(500);
  getCoords();
  //turn90R();
  //numTurnsR += 1;
  delay(500);
  //getCoords();
}

void getCoords()
{
  double Xs [] = {0,0,0,0,0,0,0,0,0};
  double Ys [] = {0,0,0,0,0,0,0,0,0};
  if (numTurnsR % 4 == 0)
  {
    int n = 0;
    while (n < 9)
    {
      int reading = 0;
      int reading1 = ultrasonicFL.Ranging(CM);
      int reading2 = ultrasonicFR.Ranging(CM);
      if (reading1 < reading2)
        reading = lnth - reading1 - 12.5;
      else
        reading = lnth - reading2 - 12.5;       
      
      if (reading <= 450)
      {
        Ys[n] = reading;
        n++;
      }
    }

    int m = 0;
    while (m < 9)
    {
      int val = ultrasonicL.Ranging(CM);

      if (val <= 450)
      {
        Xs[m] = val + 9.5;
        m++;
      }
    }   
  }

  else if (numTurnsR % 4 == 2)
  {
    int n = 0;
    while (n < 9)
    {
      int reading = 0;
      int reading1 = ultrasonicFL.Ranging(CM);
      int reading2 = ultrasonicFR.Ranging(CM);
      if (reading1 < reading2)
        reading = reading1 + 12.5;
      else
        reading = reading2 + 12.5;       
      
      if (reading <= 450)
      {
        Ys[n] = reading;
        n++;
      }
    }

    int m = 0;
    while (m < 9)
    {
      int val = ultrasonicR.Ranging(CM);

      if (val <= 450)
      {
        Xs[m] = val + 8.5;
        m++;
      }
    }   
  }

  else if (numTurnsR % 4 == 1)
  {
    int n = 0;
    while (n < 9)
    {
      int reading = 0;
      int reading1 = ultrasonicFL.Ranging(CM);
      int reading2 = ultrasonicFR.Ranging(CM);
      if (reading1 < reading2)
        reading = width - reading1 - 12.5;
      else
        reading = width - reading2 - 12.5;       
      
      if (reading <= 450)
      {
        Xs[n] = reading;
        n++;
      }
    }

    int m = 0;
    while (m < 9)
    {
      int val = ultrasonicR.Ranging(CM);

      if (val <= 450)
      {
        Ys[m] = val + 9.5;
        m++;
      }
    }   
  }
  else if (numTurnsR % 4 == 3)
  {
    int n = 0;
    while (n < 9)
    {
      int reading = 0;
      int reading1 = ultrasonicFL.Ranging(CM);
      int reading2 = ultrasonicFR.Ranging(CM);
      if (reading1 < reading2)
        reading = reading1 + 12.5;
      else
        reading = reading2 + 12.5;       
      
      if (reading <= 450)
      {
        Xs[n] = reading;
        n++;
      }
    }

    int m = 0;
    while (m < 9)
    {
      int val = ultrasonicL.Ranging(CM);

      if (val <= 450)
      {
        Ys[m] = val + 9.5;
        m++;
      }
    }   
  }

  qsort(Xs,sizeof(Xs)/sizeof(Xs[0]),sizeof(Xs[0]),sort_desc);
  qsort(Ys,sizeof(Ys)/sizeof(Ys[0]),sizeof(Ys[0]),sort_desc);

  coordX = Xs[4]; 
  coordY = Ys[4];

  Serial.println(coordX);
  Serial.println(coordY);
  Serial.println(mines);
  delay(1000);
}

void getLW ()
{
  int Ls [] = {0,0,0,0,0,0,0,0,0};
  int Ws [] = {0,0,0,0,0,0,0,0,0};

  double l = 0, w = 0;
  
  int n = 0;
  while (n < 9)
  {
    int val = ultrasonicFL.Ranging(CM);
    if (val <= 450)
    {
      Ls[n] = val;
      n++;
    }
  }

  int m = 0;
  while (m < 9)
  {
    int value = ultrasonicR.Ranging(CM);
    if (value <= 450)
    {
      Ws[m] = value;
      m++;
    }    
  }
  qsort(Ls,sizeof(Ls)/sizeof(Ls[0]),sizeof(Ls[0]),sort_desc);
  qsort(Ws,sizeof(Ws)/sizeof(Ws[0]),sizeof(Ws[0]),sort_desc);
 
  l = Ls[4];
  w = Ws[4];
  
  lnth = (int) l + 28.5;
  width = (int) w + 24;
}

void serpentine ()
{
  int FL = ultrasonicFL.Ranging(CM);
  int FR = ultrasonicFR.Ranging(CM);
  while (FR > 7)
  {
    motor3.run(FORWARD);
    motor4.run(FORWARD);
  
    FL = ultrasonicFL.Ranging(CM);
    FR = ultrasonicFR.Ranging(CM);
  }

  motor3.run(RELEASE);
  motor4.run(RELEASE);
}

void fwdWall (int dist)
{
  int read3 = 0;
  int read4 = 0;
  int prev3 = digitalRead(encoder3);
  int prev4 = digitalRead(encoder4);
  
  int FL = ultrasonicFL.Ranging(CM);
  while(FL > dist)
  {
    FL = ultrasonicFL.Ranging(CM);
    getCoords();
    motor3.run(FORWARD);
    motor4.run(FORWARD);
    read3 = digitalRead(encoder3);
    read4 = digitalRead(encoder4);

    if (read3 != prev3)
    {
      d3++;
    }
    if (read4 != prev4)
    {
      d4++;
    }
    prev3 = read3;
    prev4 = read4;

    
   if (analogRead(piezoF) > threshold)
   {
     mines++;
     numDisp(mines);
   }
   if (analogRead(piezoB) > threshold)
   {
     mines++;
     numDisp(mines);
   }
   
   //getCoords();
  }
  motor3.run(RELEASE);
  motor4.run(RELEASE);
  delay(500);

  /*
  if ((d4 > d3) && (d4 - d3 > 45))
  {
    backwardSingle(d4-d3, 4);
    d3 = 0;
    d4 = 0;
  }
  else if ((d3 > d4) && (d3 - d4 > 45))
  {
    backwardSingle(d3 - d4, 3);
    d4 = 0;
    d4 = 0;
  }
  */
}

void armUp ()
{
  int deg = 0;

  int reading;
  int prev = digitalRead(encoder1);
  
  while (deg < 720)
  {
    motor1.run(BACKWARD);
    motor2.run(BACKWARD);
    reading = digitalRead(encoder1);

    if (reading != prev)
    {
      deg++;
    }

    prev = reading;    
  }

  motor1.run(RELEASE);
  motor2.run(RELEASE);
  delay(500); 
}

void armDown ()
{
  int deg = 0;

  int reading;
  int prev = digitalRead(encoder1);
  
  while (deg < 700)
  {
    motor1.run(FORWARD);
    motor2.run(FORWARD);
    reading = digitalRead(encoder1);

    if (reading != prev)
    {
      deg++;
    }

    prev = reading;    
  }

  motor1.run(RELEASE);
  motor2.run(RELEASE);
  delay(500); 
  
}

void firstCase ()
{
  fwdWall(8);
  armUp();
  endRArm();
  armDown();
}

void generalCase()
{
  int distance = ultrasonicL.Ranging(CM);
  while (distance >= 40)
  { 
    fwdWall(8);
    if (numTurnsR%4 == 0)
    {
      endR();
    }
    else if(numTurnsR%4 == 2)
    {
      endL();
    }
    if(numTurnsR%4 == 0)
    {
      distance = ultrasonicR.Ranging(CM);
    }
    else if (numTurnsR%4 == 2)
    {
      distance = ultrasonicL.Ranging(CM);
    }
  }
}

void readPiezo()
{
  
}

/*
void random()
{
  
  while (mines < 5)
  {
    fwdWall(20);
    
  }
}
*/

void setup() 
{
  // put your setup code here, to run once:
  for (int i = 30; i <= 36; i++)
  {
    pinMode(i,OUTPUT);
  }  
  numDisp(mines);
  Serial.begin(9600);
  motor3.setSpeed(speed3);
  motor4.setSpeed(speed4);
  motor3.run(RELEASE);
  motor4.run(RELEASE);

  motor2.setSpeed(speed2);
  motor1.setSpeed(speed1);
  motor2.run(RELEASE);
  motor1.run(RELEASE);

  d3 = 0;
  d4 = 0;
  pinMode(swtch, INPUT_PULLUP);
  //buttonPress();
}

void randomWalk(){
  for(int i = 0; i < 2; i++){
    if (analogRead(piezoF) > threshold || analogRead(piezoB) > threshold){
      mines++;
      numDisp(mines);
    }
    
    fwdWall(15);
    backward(720);
    turn90R();
    fwdWall(15);
    backward(720);
    turn90R();
    fwdWall(15);
    backward(720);
    turn90R();
    fwdWall(15);
    backward(720);
    turn90R();
    
    fwdWall(30);
    turn90R();
    fwdWall(30);
    turn90R();
    fwdWall(15);
    backward(720);
    turn90R();
    fwdWall(15);
    backward(720);
    turn90R();
    
    fwdWall(50);
    turn90R();
    fwdWall(50);
    turn90R();
    fwdWall(15);
    backward(720);
    turn90R();
    fwdWall(15);
    backward(720);
    turn90R();
    
    fwdWall(70);
    turn90R();
    fwdWall(70);
    turn90R();
    fwdWall(15);
    backward(720);
    turn90R();
    fwdWall(15);
    backward(720);
    turn90R();
    
    fwdWall(90);
    turn90R();
    fwdWall(90);
    turn90R();
    fwdWall(15);
    backward(720);
    turn90R();
    fwdWall(15);
    backward(720);
    turn90R();
    
    fwdWall(120);
    turn90R();
    fwdWall(120);
    turn90R();
    fwdWall(15);
    backward(720);
    turn90R();
    fwdWall(15);
    backward(720);
    turn90R();
    
    fwdWall(90);
    turn90R();
    fwdWall(90);
    turn90R();
    fwdWall(15);
    backward(720);
    turn90R();
    fwdWall(15);
    backward(720);
    turn90R();
    
    fwdWall(70);
    turn90R();
    fwdWall(70);
    turn90R();
    fwdWall(15);
    backward(720);
    turn90R();
    fwdWall(15);
    backward(720);
    turn90R();
    
    fwdWall(50);
    turn90R();
    fwdWall(50);
    turn90R();
    fwdWall(15);
    backward(720);
    turn90R();
    fwdWall(15);
    backward(720);
    turn90R();
    
    fwdWall(30);
    turn90R();
    fwdWall(30);
    turn90R();
    fwdWall(15);
    backward(720);
    turn90R();
    fwdWall(15);
    backward(720);
    turn90R();
    
    fwdWall(15);
    backward(720);
    turn90R();
    fwdWall(15);
    backward(720);
    turn90R();
    fwdWall(15);
    backward(720);
    turn90R();
    fwdWall(15);
    backward(720);
    turn90R();
    
    int FL = ultrasonicFL.Ranging(CM);
    int FR = ultrasonicFR.Ranging(CM);
  }
}

void real ()
{
 int n = ultrasonicFL.Ranging(CM);
 if (n > 15)
 {
   motor3.run(FORWARD);
   motor4.run(FORWARD);
 }
 else
 {
   motor3.run(RELEASE);
   motor4.run(RELEASE);
   delay(500);
   motor3.run(BACKWARD);
   motor4.run(BACKWARD);
   delay(1500);
   motor3.run(FORWARD);
   motor4.run(BACKWARD);
   delay(3500);
   motor3.run(RELEASE);
   motor4.run(RELEASE);
 }
}
  

void oriPos(){
  
}
void loop() 
{
  /*delay(5000);
  getLW();
  getCoords();
  
  randomWalk();
  oriPos();
  */
  real();
}
