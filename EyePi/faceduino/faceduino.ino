#define USE_USBCON
#include <ros.h>
#include <ram_animation_msgs/Face.h>
#include <ram_animation_msgs/animatorConstants.hpp>
#include "HT1632.h"

#define DATA 2
#define WR   3
#define CS   4
// use this line for single matrix
HT1632LEDMatrix matrix = HT1632LEDMatrix(DATA, WR, CS);
#define NLEVEL 6  //0-5

#define eyeWidth    7
#define eyeOriginRX 6
#define eyeOriginRY 7
#define eyeOriginLX 17
#define eyeOriginLY 7
#define eyeSize     3

#define paddleHeight 7

char emotionState = RamAnimatorConstants::EmotionSleepy;
char emotionLevel = 120;
char xPosition = 64;
char yPosition = 64;
char screenbrightness = 30;

int gCount;
int gBlinkCycle;
int blinkDuration = 0;
char crashAnimationState = 0;

unsigned long time;

void clrLine(int x1, int y1, int x2, int y2){
  for(int i=x1; i<=x2; i++){
    double y = (double)((double)((y2-y1)*(i-x1))/(x2-x1))+y1;
    if(y - (int)y > 0.1) y = y + 1.0;
    matrix.clrPixel(i, (int)y);
  }  
}

void setLine(int x1, int y1, int x2, int y2){
  for(int i=x1; i<=x2; i++){
    double y = (double)((double)((y2-y1)*(i-x1))/(x2-x1))+y1;
    if(y - (int)y > 0.1) y = y + 1.0;
    matrix.setPixel(i, (int)y);
  }  
}

void setEyeBall(int x, int y){
  matrix.fillCircle(eyeOriginRX + x, eyeOriginRY + y, eyeSize, 1);   
  matrix.fillCircle(eyeOriginLX + x, eyeOriginLY + y, eyeSize, 1);
}

void changeBrightness(int duration) {
  int8_t brightness;
  if(duration == 0) {
    brightness = 15;
  } else {
    if(gCount % duration < duration / 2) brightness = map(gCount % duration, 0,          duration/2 - 1, 0,  15);//(int8_t)( (gCount%(duration/2)) * 15 / (duration/2) + 1);
    else                                 brightness = map(gCount % duration, duration/2, duration - 1,  15,   0);//(int8_t)(-(gCount%(duration/2)) * 15 / (duration/2)  + 15);
  }
  matrix.setBrightness(brightness);
}

void crashAnimation(int state)
{
  switch(state){
      case 0:  //Draw random pixelation of screen
      
        for(int i = 0; i<200;i++)
        {
          matrix.clrPixel(int(random(0,matrix.width())),int(random(0,matrix.height())));  //Turn a random pixel off
          matrix.setPixel(int(random(0,matrix.width())),int(random(0,matrix.height())));  //Turn a random pixel on
          matrix.writeScreen();
        }
        crashAnimationState++;
        break;
        
      case 1: //Draw a blinking X on the screen
        matrix.clearScreen(); 
        matrix.drawLine(0, 0, matrix.width()-1, matrix.height()-1, 1);
        matrix.drawLine(matrix.width()-1, 0, 0, matrix.height()-1, 1);
        matrix.writeScreen();
        matrix.blink(true);
        delay(1500);
        matrix.blink(false);
        crashAnimationState++;
        break;
        
      case 2: 
        
        //Eyes wondering what the hell is happening with himself
        matrix.clearScreen();
        yPosition=0;
       
        for(int i = -2; i<3; i++) //move eyes right
        {
          matrix.clearScreen();
          xPosition = i;
          setEyeBall(xPosition,yPosition);
          matrix.writeScreen();
          delay(40);
        }
        delay(300);
        for(int i = 2; i>-3; i--) //move eyes left
        {
          matrix.clearScreen();
          xPosition = i;
          setEyeBall(xPosition,yPosition);
          matrix.writeScreen();    
          delay(40);      
        }
        delay(300);
        for(int i = -2; i<3; i++) //move eyes right
        {
          matrix.clearScreen();
          xPosition = i;
          setEyeBall(xPosition,yPosition);
          matrix.writeScreen();
          delay(40);
        }
        delay(300);
     
        xPosition = 0;
        yPosition = 0;
        /*setEyeBall(xPosition,yPosition);
        for(int i = 0; i < 6; i++) //act amazed
        {
          emotionState = RamAnimatorConstants::EmotionAmazed;
          emotionLevel = i;
          matrix.writeScreen();
          delay(200);
        }  */
        crashAnimationState++;
        break;
        
      case 3: //Draw 'ERROR!' on the screen     
        matrix.clearScreen(); 
        // draw some text!
        matrix.setTextSize(1);    // size 1 == 8 pixels high
        matrix.setTextColor(1);   // 'lit' LEDs

        matrix.setCursor(3, 0);   // start at top left, with one pixel of spacing
        matrix.print("ERR");
        matrix.setCursor(4, 8);   // next line, 8 pixels down
        matrix.print("OR!");
        matrix.writeScreen();
        matrix.writeScreen();
        delay(2000);
        crashAnimationState++;
        break;
        
      case 4:  //Draw pong animation
        for(int i = 0; i<4;i++)
        {
          matrix.clearScreen();
          pongDrawPaddle(i+2);
          pongDrawBall(18-i,11-i);
          matrix.writeScreen();
          delay(200);
        }
        for(int i = 0; i<7;i++)
        {
          matrix.clearScreen();
          pongDrawPaddle(6-i);  
          pongDrawBall(10-2*i,7-2*i);
          matrix.writeScreen();
          delay(200);
        }
        crashAnimationState++;
        break;
        
      case 5:
        
        //REBOOTING SCROLLING TEXT
        matrix.clearScreen(); 
        // draw some text!
        matrix.setTextSize(1);    // size 1 == 8 pixels high
        matrix.setTextColor(1);   // 'lit' LEDs

        for(int i = 24; i>-55; i--)
        {
          matrix.clearScreen();
          matrix.setCursor(i, 4);  
          matrix.print("RESETTING....");
          matrix.writeScreen();
          delay(40);
        }
        crashAnimationState++;
       
      }
}

void pongDrawPaddle(int yPos) //draws diagonal symmetric pong paddles
{
   matrix.drawLine(1,yPos,1,yPos+paddleHeight,1); //left paddle
   matrix.drawLine(2,yPos,2,yPos+paddleHeight,1);
   matrix.drawLine(matrix.width()-2,matrix.height()-yPos-1,matrix.width()-2,matrix.height()-yPos-paddleHeight-1,1); //right paddle
   matrix.drawLine(matrix.width()-3,matrix.height()-yPos-1,matrix.width()-3,matrix.height()-yPos-paddleHeight-1,1);
}

void pongDrawBall(int xPos, int yPos)
{
  matrix.setPixel(xPos,yPos);
}

void writeMatrix(int emotionState, int emotionLevel, int xPosition, int yPosition) {
  
  // Start and End position for Line or Rect
  int xR1 = eyeOriginRX + eyeWidth/2; //inside
  int xL1 = eyeOriginLX - eyeWidth/2; //inside
  int xR2 = eyeOriginRX - eyeWidth/2; //outside
  int xL2 = eyeOriginLX + eyeWidth/2; //outside

  // Reset the crash animation state after going back to normal faces
  if(emotionState != RamAnimatorConstants::EmotionCrash) crashAnimationState = 0; 

  //// eyeballs  
  switch(emotionState){
    case RamAnimatorConstants::EmotionNeutral:  //Neutral
      matrix.fillRect(0, 0, matrix.width(), matrix.height(), 0); // start with blank screen
      setEyeBall(xPosition, yPosition);
      break;  
  
     case RamAnimatorConstants::EmotionExcited:  //Happy
      matrix.fillRect(0, 0, matrix.width(), matrix.height(), 0); // start with blank screen
      setEyeBall(xPosition, yPosition);
      matrix.fillCircle(eyeOriginRX + xPosition, eyeOriginRY + yPosition - emotionLevel + 8, 3, 0);  // clear circle below Right eye
      matrix.fillCircle(eyeOriginLX + xPosition, eyeOriginLY + yPosition - emotionLevel + 8, 3, 0);  // clear circle below Left  eye
      break; 
   
    case RamAnimatorConstants::EmotionSleepy:  //Sleep
      matrix.fillRect(0, 0, matrix.width(), matrix.height(), 0); // start with blank screen
      setEyeBall(xPosition, yPosition);
      // Arc
      for (int i=0; i<=emotionLevel; i++){  
        matrix.drawLine(0, eyeOriginRY - 5 + i + yPosition, matrix.width(), eyeOriginRY - 5 + i + yPosition, 0);  // Upper arc
        matrix.drawLine(0, eyeOriginLY + 5 - i + yPosition, matrix.width(), eyeOriginLY + 5 - i + yPosition, 0);  // Bottom top arc
      }
      break;
   
    case RamAnimatorConstants::EmotionAmazed:  //Surprized 
      matrix.fillRect(0, 0, matrix.width(), matrix.height(), 0); // start with blank screen
      if(emotionLevel == 0) {
        setEyeBall(xPosition, yPosition);
      }
      // Change size
      else{
        matrix.fillCircle(eyeOriginRX + xPosition, eyeOriginRY + yPosition, emotionLevel + 1, 1);   
        matrix.fillCircle(eyeOriginLX + xPosition, eyeOriginLY + yPosition, emotionLevel + 1, 1);
      }
      break;
   
    case RamAnimatorConstants::EmotionSad:  //Sadness
      matrix.fillRect(0, 0, matrix.width(), matrix.height(), 0); // start with blank screen
      setEyeBall(xPosition, yPosition);
      // Upper arc
      for (int i=0; i<=emotionLevel; i++){ 
        clrLine(xR2 + xPosition, eyeOriginRY - 4 + i + yPosition, xR1 + xPosition, eyeOriginRY - 7 + i + yPosition);
        clrLine(xL1 + xPosition, eyeOriginLY - 7 + i + yPosition, xL2 + xPosition, eyeOriginLY - 4 + i + yPosition); 
      }
      break; 
   
    case RamAnimatorConstants::EmotionAngry:  //Anger
      matrix.fillRect(0, 0, matrix.width(), matrix.height(), 0); // start with blank screen
      setEyeBall(xPosition, yPosition);
      for (int i=0; i<=emotionLevel; i++){\
        clrLine(xR2 + xPosition, eyeOriginRY - 7 + i + yPosition, xR1 + xPosition, eyeOriginRY - 4 + i + yPosition); // Upper arc
        clrLine(xL1 + xPosition, eyeOriginLY - 4 + i + yPosition, xL2 + xPosition, eyeOriginLY - 7 + i + yPosition); // Upper arc
        matrix.drawLine(0, eyeOriginRY + 7 - i  + yPosition, matrix.width(), eyeOriginLY + 7 - i + yPosition, 0);    // Bottom arc  
      }
      break;
   
    case RamAnimatorConstants::EmotionFriendly:  //Friendly
      matrix.fillRect(0, 0, matrix.width(), matrix.height(), 0); // start with blank screen
      setEyeBall(xPosition, yPosition);
      // Modify dots    
      if(emotionLevel > 0){
        matrix.setPixel(19 + xPosition, 11 + yPosition);
        matrix.clrPixel(16 + xPosition, 11 + yPosition);
        matrix.setPixel(15 + xPosition, 4  + yPosition);
        matrix.clrPixel(18 + xPosition, 4  + yPosition);
   
        matrix.setPixel(4 + xPosition, 11 + yPosition);
        matrix.clrPixel(7 + xPosition, 11 + yPosition);
        matrix.setPixel(8 + xPosition, 4  + yPosition);
        matrix.clrPixel(5 + xPosition, 4  + yPosition);
      }  
      // Upper arc
      if(emotionLevel > 1){
        for (int i=0; i<=emotionLevel; i++){
          clrLine(xR2 + xPosition, eyeOriginRY - 7 + i + yPosition, xR1 + xPosition, eyeOriginRY - 4 + i + yPosition); // Upper arc
          clrLine(xL1 + xPosition, eyeOriginLY - 4 + i + yPosition, xL2 + xPosition, eyeOriginLY - 7 + i + yPosition); 
        }
        matrix.drawLine(0, eyeOriginRY - 6 + emotionLevel + yPosition, matrix.width(), eyeOriginRY - 6 + emotionLevel + yPosition, 0);  // Upper arc
      }
      break;
    
    case RamAnimatorConstants::EmotionCrash: //System crash animation
      crashAnimation(crashAnimationState);    
      break;

    default:
      setEyeBall(xPosition, yPosition);
      blinkDuration = 0;
      break;
  }
  
  // changeBrightness(screenbrightness*2);
  matrix.setBrightness(map(screenbrightness,0,127,0,15));

  // now, update the screen:
  matrix.writeScreen();
  
}

ros::NodeHandle  nh;

void messageCb( const ram_animation_msgs::Face& msg){ 
   emotionState = msg.emotionState;
   emotionLevel = msg.emotionLevel;
   xPosition = msg.eyePosX;
   yPosition = msg.eyePosY;
   screenbrightness = msg.screenBrightness;
}

ros::Subscriber<ram_animation_msgs::Face> sub("/ram/animation/generated/face", &messageCb );

void setup()
{ 
  pinMode(13, OUTPUT);
  matrix.begin(HT1632_COMMON_16NMOS); 
  nh.initNode();
  nh.subscribe(sub);
}

long customMap(long x, long in_min, long in_max, long out_min, long out_max){
  return round(((float)((x - in_min) * (out_max - out_min)) / (float)(in_max - in_min)) + out_min);
}

void loop()
{ 
  writeMatrix(emotionState, map(emotionLevel,0,127,0,5), customMap(xPosition,0,127,-3,3), customMap(yPosition,0,127,4,-4));
  nh.spinOnce();
  delay(1);
}

