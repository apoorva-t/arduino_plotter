// new sketch for drawing a line

#include <AFMotor.h>
#include <Stepper.h>
#include <Servo.h>
#include <Process.h>

#define MAX_LEN 200
#define MM_PER_SEGMENT 5
#define STEPSIZE 8 //MICROSTEPS

// Connect a stepper motor with 48 steps per revolution (7.5 degree)
// to motor port #2 (M3 and M4)
AF_Stepper motor1(200, 1);
AF_Stepper motor2(200, 2);

Servo penServo;
float oldX = 0.0, oldY = 0.0;
int pos = 0;

void setup() {
  Serial.begin(9600);           // set up Serial library at 9600 bps
  Serial.println("Stepper test!");

  motor1.setSpeed(20);  // 20 rpm  
  motor2.setSpeed(20); 
  
  penServo.write(180);
  penServo.attach(9);
  
}


void loop() {
  
  char gcLine[MAX_LEN];
  char c;
  int linenum = 0;
  
  while (1)
  {
    while (Serial.available() > 0)
    {
        c = Serial.read();
        if (c == '\n')
        {  
          //end of line
          gcLine[linenum++] = '\0';
          //penDown();
          drawStep(gcLine, linenum);
          //penUp();
          delay(500);
          linenum = 0;
          Serial.flush();
          Serial.print(gcLine);
        }
        else
        {
          gcLine[linenum] = c;
          linenum++;
        }
        //Serial.println("done");
        //delay(80);
    }
  }
}

void penDown()
{
   //servo down gradually to write
   pos = 180;
   for (int i = 0; i < 4; i++)
   {
     penServo.write(pos);
     delay(15);
     pos -= 4;
   }
}

void penUp()
{
  //servo up gradually
  pos = 165;
  for (int i = 0; i < 4; i++)
  {
    penServo.write(pos);
    delay(15);
    pos += 4;
  }
}

void drawStep(char* gcLine, int numC)
{
  int ind = 0;
  char tmpbuf[MAX_LEN];
  char cmd;
  char* pos;
  float newX=0.0, newY=0.0;
  float offX = 0.0, offY = 0.0;
//  while (ind < numC-1)
//  {
    cmd = gcLine[ind++];
    switch (cmd)
    {
      case 'G':
      {
      //tmpbuf[0] = gcLine[ind++];
      //tmpbuf[1] = gcLine[ind++];
      //tmpbuf[2] = '\0';
      int gNum = (int)parseNum('G', gcLine);
      switch (gNum) //switch (atoi(tmpbuf))
      {
        case 0:
        case 1:
        //move in straight line to X and Y
        newX = parseNum('X', gcLine);
        newX = (newX == -1.00) ? oldX : newX;
        newY = parseNum('Y', gcLine);
        newY = (newY == -1.00) ? oldY : newY;
        drawLine(newX, newY);
        oldX = newX;
        oldY = newY;
        break;
        case 2:
        newX = parseNum('X', gcLine);
        newX = (newX == -1.00) ? oldX : newX;
        newY = parseNum('Y', gcLine);
        newY = (newY == -1.00) ? oldY : newY;
        offX = parseNum('I', gcLine);
        offY = parseNum('J', gcLine);
        drawArc(newX, newY, offX, offY, true);
        oldX = newX;
        oldY = newY;
        break;
        case 3:
        newX = parseNum('X', gcLine);
        newX = (newX == -1.00) ? oldX : newX;
        newY = parseNum('Y', gcLine);
        newY = (newY == -1.00) ? oldY : newY;
        offX = parseNum('I', gcLine);
        offY = parseNum('J', gcLine);
        //Serial.println("Printing ccW\n");
        drawArc(newX, newY, offX, offY, false);
        oldX = newX;
        oldY = newY;
        break;
      }
      break;
      }
      case 'Z':
      {
      float pUp = parseNum('Z', gcLine);
      if (pUp == 0.0)
         penDown();
       else
         penUp();
       break;
      }
      default:
        //move in straight line to X and Y
        newX = parseNum('X', gcLine);
        newX = (newX == -1.00) ? oldX : newX;
        newY = parseNum('Y', gcLine);
        newY = (newY == -1.00) ? oldY : newY;
        drawLine(newX, newY);
        oldX = newX;
        oldY = newY;
        break;
    }
 // }
  
}

void drawArc(float newX, float newY, float offX, float offY, boolean cW)
{
  const float Pi = 3.14159;
  float cx = oldX + offX;
  float cy = oldY + offY;
  float radi = sqrt((newX-cx)*(newX-cx) + (newY-cy)*(newY-cy));
  
  float dx = oldX - cx;
  float dy = oldY - cy;
  
  float theta0 = atan2(dy, dx);
  theta0 = (theta0 < 0) ? (2*Pi + theta0) : theta0;
  dx = newX - cx;
  dy = newY - cy;
  float theta1 = atan2(dy, dx);
  theta1 = (theta1 < 0) ? (2*Pi + theta1) : theta1;
  
  if (cW == true)
  {
    float dTheta = (2*Pi + theta0 - theta1);
    dTheta = (dTheta > (2*Pi)) ? (dTheta - 2*Pi) : dTheta;
    float arcLen = dTheta*radi;
    
    int numSegs = floor(arcLen/MM_PER_SEGMENT);
    //Serial.print("\n Numsegs: ");
    //Serial.print(numSegs);
    //Serial.print("\t ArcLEN: ");
    //Serial.print(arcLen);
    //Serial.print("\n");
    for (int i = 0; i < numSegs; i++)
    {
      float newDTheta = (dTheta*i)/numSegs;
      float newTheta1 = theta0 - newDTheta;
      float tmpNewX = radi*cos(newTheta1) + cx;
      float tmpNewY = radi*sin(newTheta1) + cy;
      //printf("tmpX = %.3f, tmpY = %.3f", tmpNewX, tmpNewY);
      //Serial.print("tmpX: ");
      //Serial.print(tmpNewX);
      //Serial.print(" tmpY: ");
      //Serial.print(tmpNewY);
      //Serial.print("\n");
      drawLine(tmpNewX, tmpNewY);
      delay(50);
      oldX = tmpNewX;
      oldY = tmpNewY;
    }
    drawLine(newX, newY);
    //Serial.print(numSegs);
  }
  else // if cW == false
  {
    float dTheta = (2*Pi + theta1 - theta0);
    dTheta = (dTheta > (2*Pi)) ? (dTheta - 2*Pi) : dTheta;
    float arcLen = dTheta*radi;
    //Serial.println("arcLen is: ");
    //Serial.println(arcLen);
    int numSegs = floor(arcLen/MM_PER_SEGMENT);
    for (int i = 0; i < numSegs; i++)
    {
      float newDTheta = (dTheta*i)/numSegs;
      float newTheta1 = theta0 + newDTheta;
      float tmpNewX = radi*cos(newTheta1) + cx;
      float tmpNewY = radi*sin(newTheta1) + cy;
      //printf("tmpX = %.3f, tmpY = %.3f", tmpNewX, tmpNewY);
      //Serial.print("tmpX: ");
      //Serial.print(tmpNewX);
      //Serial.print(" tmpY: ");
      //Serial.print(tmpNewY);
      //Serial.print("\n");
      drawLine(tmpNewX, tmpNewY);
      delay(50);
      oldX = tmpNewX;
      oldY = tmpNewY;
    }
    drawLine(newX, newY);
  }
}

void drawLine(float newX, float newY)
{
  int stepCntX = 0, stepCntY = 0;
  float diffX = newX - oldX;
  float diffY = newY - oldY;
  int dirX = (diffX > 0) ? 1 : -1;
  int dirY = (diffY > 0) ? 1 : -1;
  
  diffX = abs(diffX);
  diffY = abs(diffY);
  
  float over = -1.00;
  
  float m = diffY/diffX;
  
  uint32_t uspers1 = motor1.usperstep/MICROSTEPS;
  //uint32_t uspers2 = motor2.usperstep/MICROSTEPS;
  
  if (diffX > diffY)
  {
    //one step in X dir
    for (float i=0; i < diffX; i++)
    {
      if (dirX==1)
      {
        //uint8_t ret1 = 0, ret2 = 0;
        motor1.onestep(FORWARD, MICROSTEP);
      }
      else
      {
        motor1.onestep(BACKWARD, MICROSTEP);
      }
      
      over += m;
      if (over >= 0)
      {
        over -= 1;
        if (dirY==1)
        {
           motor2.onestep(FORWARD, MICROSTEP);
           delay(uspers1/1000);
           for (int k=0; k < STEPSIZE-1; k++)
           {
             motor1.onestep((dirX==1) ? FORWARD : BACKWARD, MICROSTEP);
             motor2.onestep(FORWARD, MICROSTEP);
             delay(uspers1/1000);
           }          
        }
        else
        {
          motor2.onestep(BACKWARD, MICROSTEP);
          delay(uspers1/1000);
           for (int k=0; k < STEPSIZE-1; k++)
           {
             motor1.onestep((dirX==1) ? FORWARD : BACKWARD, MICROSTEP);
             motor2.onestep(BACKWARD, MICROSTEP);
             delay(uspers1/1000);
           }
        }
        stepCntX++;
        stepCntY++;
      }
      else
      {
        delay(uspers1/1000);
         for (int k=0; k < STEPSIZE-1; k++)
         {
             motor1.onestep((dirX==1) ? FORWARD : BACKWARD, MICROSTEP);
             delay(uspers1/1000);
         }
         stepCntX++;
      }
      //delay(motor1.usperstep/(MICROSTEPS*1000));
      delay(100);
    }
  }
  else // if diffY > diffX
  {
    for (float i=0; i < diffY; i++)
     {
       if (dirY==1)
       {
          motor2.onestep(FORWARD, MICROSTEP);
       }
       else
       {
          motor2.onestep(BACKWARD, MICROSTEP);
       }
      over += 1/m;
      if (over >= 0)
      {
        over -= 1;
        if (dirX==1)
        {
           motor1.onestep(FORWARD, MICROSTEP);
           delay(uspers1/1000);
           for (int k=0; k < STEPSIZE-1; k++)
           {
             motor1.onestep(FORWARD, MICROSTEP);
             motor2.onestep((dirY == 1) ? FORWARD : BACKWARD, MICROSTEP);
             delay(uspers1/1000);
           }
        }
        else
        {
          motor1.onestep(BACKWARD, MICROSTEP);
          delay(uspers1/1000);
           for (int k=0; k < STEPSIZE-1; k++)
           {
             motor1.onestep(BACKWARD, MICROSTEP);
             motor2.onestep((dirY==1) ? FORWARD : BACKWARD, MICROSTEP);
             delay(uspers1/1000);
           }
        }
        stepCntX++;
        stepCntY++;
      }
      else
      {
        delay(uspers1/1000);
         for (int k=0; k < STEPSIZE-1; k++)
         {
             motor2.onestep((dirY==1) ? FORWARD : BACKWARD, MICROSTEP);
             delay(uspers1/1000);
         }
         stepCntY++;
      }
      //delay(motor1.usperstep/(MICROSTEPS*1000));
      delay(100);
     }
  }
  if ((stepCntX % 2) != 0)
  {
    for (int k=0; k< STEPSIZE; k++)
    {
      motor1.onestep((dirX==1) ? FORWARD : BACKWARD, MICROSTEP);   
    }
  }
  if ((stepCntY % 2) != 0)
  {
    for (int k=0; k< STEPSIZE; k++)
    {
      motor2.onestep((dirY==1) ? FORWARD : BACKWARD, MICROSTEP);
    }
  }
  Serial.print("stepCntX:");
  Serial.print(stepCntX);
  Serial.print("\n");
  Serial.print("stepCntY:");
  Serial.print(stepCntY);
}

float parseNum(char findC, const char* str)
{
  char* cPos = strchr(str, findC);
  float retInt = -1.00;
  if (cPos != NULL)
  {
    retInt = atof(cPos+1);
  }
   return retInt; 
}


