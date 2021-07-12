#include <Arduino.h>
#include <Wire.h>

// --------------- Master ---------------
// Open The Command if want to upload on Slave

// void prosedur
//Master To Slave
#include <kinematic_RS.h>

#define Master Serial3
// Motor DC Pin Out
#define MRF 5       // Motor Right Front Direction
#define MRB 4       // Motor Right Back Off Direction
#define MLF 7       // Motor Left Front Direction
#define MLB 6       // Motor Left Back Off Direction
int RpwmOne = 100,
    LpwmOne = 100,
    RpwmTwo = 50,
    LpwmTwo = 50,
    Rpwm,
    Lpwm;
    
int leftSpeed,
    rightSpeed,
    leftSpeedVal,
    rightSpeedVal;

bool check = false;

//UVC Activation
#define UB 29         // UV Behind
#define UF 28         // UV Front
#define BUZZ 42       // Buzzer Relay PIN Before Turn the Lamp ON
#define ACTIVE_RL 30   // Activation Relay for Inverter


const int rellayOn = HIGH,
          rellayOff = LOW,
          BuzzerOn = HIGH,
          BuzzerOff = LOW,
          buzzerTime = 1000,
          buzzerCycle = 3;

// DHT and FAN
#define DHT 42        // DHT11 RELAY
#define FAN_RLY 31    // FAN RELAY


// Ultrasonic 
int reading = 0;

// Constrain Variable 
int max = 1988, //Stick Constrain
    min = 988;

int analogMax = 500, // Output Sensor Constrain
    analogMin = 120;

#define errorRules 5 
#define deltaErrorRules 5
String state;

float fuzzyError[errorRules],
      deltaError[deltaErrorRules],
      errorPosition,
      deltaErrorPosition,
      previousError;

float rule[10][10],
      rule00,
      rule01,
      rule02,
      rule03,
      rule04,
      
      rule10,
      rule11,
      rule12,
      rule13,
      rule14,
      
      rule20,
      rule21,
      rule22,
      rule23,
      rule24,

      rule30,
      rule31,
      rule32,
      rule33,
      rule34,

      rule40,
      rule41,
      rule42,
      rule43,
      rule44;
//Membership function Error Position
float midLeft = -100, //left
      rightLeft = -50,

      leftMostLeft = -100, //Most Left
      midMostLeft = -50,
      rightMostLeft = 0,

      leftCenter = -50, // Center
      midCenter = 0,
      rightCenter = 50,

      leftMostRight = 0, // Most Right
      midMostRight = 50,
      rightMostRight = 100,

      leftRight = 50, // Right
      midRight = 100;

float midNegative = -20,
      rightNegative = -10,

      leftMostNegative = -20,
      midMostNegative = -10,
      rightMostNegative = 0,

      leftZero = -10,
      midZero = 0,
      rightZero = 10,
      
      leftMostPositive = 0,
      midMostPositive = 10,
      rightMostPositive = 20,

      leftPositive = 10,
      midPositive = 20;

float LF = -30,
      LM = -20,
      LS = -10,
      FW = 0,
      RS = 10,
      RM = 20,
      RF = 30;

float speedMotor,
      decision,
      devider,
      yValue,
      yFirst,
      ySecond;

//Variable PID
float error,
      RsetPointPwm = 80,
      LsetPointPwm = 80,
      setPointSensor = 300,
      pValue,
      Kp = 0.4,
      Ki,
      Kd = 0.2,
      Ts = 0.16,
      lastError;

//Variabel Derivatif
float derivatifOne,
      derivatifTwo,  
      derivatifThre,
      derivatifValue,
      proporsionalDerivatif;


// Variable Sensor
int pointerValue,
    trackDetect,
    AnalogDetect,
    RightMarkerDetect,
    LeftMarkerDetect,
    value;

volatile int encoder_position = 0; //Encoder Variable
volatile int R_encoder_position = 0;
#define phi 3.14
#define wheelLength 36
#define wheelsRadius 7.62  //6" to cm
#define encoderPPR 600
#define encoderCPR (4*encoderPPR)
#define wheelCircumference ((wheelRadius*phi)/encoderPPR)
#define xEncoder 2.108012
int stopEncoderValue = 3800; //nilai encoder 3 meter
float yRight,
      yLeft,

      yRightTick,
      yLeftTick,

      avgEncoder,
      n = 1;

// Millis
unsigned long endTimeMillis,
              startTimeMillis,
              stopTimeMillis,
              loopTimer,
              loopTimerCheck,
              endEncoderMillis,
              startEncoderMillis,
              encoderTimer;

int radiusIcc,
      xPosition,
      lastXPosition,
      yPosition,
      lastYPosition,
      thetaPosition,
      lastThetaPosition,
      xPositionCm,
      yPositionCm,
      thetaPositionDegree;

// Variable Kinematic
float // MR,
      // ML,
      // previousValueLeft,
      // previousValueRight,
      // midValue,
      // Teta_ch,
      // xKinematic = 0,
      // yKinematic = 0,
      // deltaY,
      // previousX,
      // previousY,
      // deltaX,
      // Teta,
      // deltaPositionKinematic,

      leftPosition,
      previousLeftPosition,
      previousRightPosition,
      rightPosition,
      velocityRight,
      velocityLeft,
      angularSpeed;


// //Variable Voltage Divider
float voltageDividerOne,
      voltageDividerTwo,
      outputOne,
      outputTwo;

// Ultrasonic ....

// Receiver Control
const int CH1 = 32,
          CH2 = 33,
          CH3 = 34,
          CH4 = 35,
          CH5 = 36,
          CH6 = 37,
          CH7 = 38,
          CH8 = 39,
          CH9 = 40,
          CH10 = 41;

int ch1, //variable constrain
    ch2,
    ch3,
    ch4,
    ch5,
    ch6;

void goForward(){
  if(Lpwm>0){  
    analogWrite(MLF, Lpwm);
    analogWrite(MLB, 0);
  }
  else if(Lpwm<0){
    analogWrite(MLF, 0);
    analogWrite(MLB, -Lpwm);
  }
  else if(Lpwm==0){
    analogWrite(MLF, 0);
    analogWrite(MLB, 0);
  }

  if(Rpwm>0){
    analogWrite(MRF, Rpwm);
    analogWrite(MRB, 0);
  }
  else if(Rpwm<0){
    analogWrite(MRF, 0);
    analogWrite(MRB, -Rpwm);
  }
  else if(Rpwm==0){
    analogWrite(MRF, 0);
    analogWrite(MRB, 0);
  }
}
void Buzzer(){
    for(int i=0; i<=2; i++){
      digitalWrite(BUZZ, BuzzerOn);
      delay(100);
      digitalWrite(BUZZ, BuzzerOff);
      delay(50);
    }
  }

void buzzerUV(){
  for(int i=0; i<=2; i++){
    digitalWrite(BUZZ, BuzzerOn);
    delay(500);
    digitalWrite(BUZZ, BuzzerOff);
    delay(2000);
  }
}

void uvActivation()
{
  buzzerUV();
  digitalWrite(UB, rellayOn);
  digitalWrite(UF, rellayOn);
}

void uvDeActivation(){
  digitalWrite(UB, rellayOff);
  digitalWrite(UF, rellayOff);
}


void getSensor(){
  if(Master.available())
  {
    char c = Master.peek();
    if (c == '!'){
      Master.read();
      pointerValue = 1;
    }
    else if(c == '@'){
      Master.read();
      pointerValue = 2;
    }
    else if (c == '#'){
      Master.read();
      pointerValue = 3;
    }
    else if (c == '$'){
      Master.read();
      pointerValue = 4;
    }
    else if (c == '%'){
       Master.read();
       pointerValue = 5;
    }
    else if (c == '^'){
       Master.read();
       pointerValue = 6;
    }
    else {
      value = Master.parseInt();
      if (pointerValue == 1){
        trackDetect=value;
        trackDetect=constrain(trackDetect, 0, 1);
      }
      else if (pointerValue == 2){
        AnalogDetect=value;
        AnalogDetect= constrain(AnalogDetect, analogMin, analogMax);
      }
      else if (pointerValue == 3){
        RightMarkerDetect=value;
        RightMarkerDetect=constrain(RightMarkerDetect, 0, 1);
      }
      else if (pointerValue == 4){
        LeftMarkerDetect=value;
        LeftMarkerDetect=constrain(LeftMarkerDetect, 0, 1);
      }  
      else if(pointerValue == 5){
        R_encoder_position=value;
      }
      else if(pointerValue == 6){
        encoder_position=value;
      }
      check = true;
    } 
    // Serial.print("TP: ");
    // Serial.print(trackDetect);
    // Serial.print(" AO: ");
    // Serial.print(AnalogDetect);
    // Serial.print(" RM: ");
    // Serial.print(RightMarkerDetect);
    // Serial.print(" LM: ");
    // Serial.print(LeftMarkerDetect);
    // Serial.print(" R_Encoder: ");
    // Serial.print(R_encoder_position);
    // Serial.print(" L_Encoder: ");
    // Serial.println(encoder_position);
  }
}

float callErrorPosition(float x, float xOne, float Xtwo, String highLow){
  if (highLow == "High"){
    yFirst=0;
    ySecond=1;
  }
  else if(highLow == "Low"){
    yFirst=1;
    ySecond=0;
  }
  yValue=(((x-xOne)*(ySecond-yFirst))/(Xtwo-xOne))+yFirst;
  return yValue;
}

void fuzzyfication(){
  errorPosition = setPointSensor - AnalogDetect;
  
  if(errorPosition <= midLeft){
    fuzzyError[0] = 1;
  }
  else if(errorPosition > midLeft && errorPosition <= rightLeft){
    fuzzyError[0]=callErrorPosition(errorPosition, midLeft, rightLeft, "Low");
  }
  else if(errorPosition > rightLeft){
    fuzzyError[0]=0;
  }

  if(errorPosition <= leftMostLeft){
    fuzzyError[1]=0;
  }
  else if(errorPosition > leftMostLeft && errorPosition <= midMostLeft){
    fuzzyError[1]=callErrorPosition(errorPosition, leftMostLeft, midMostLeft, "High");
  }
  else if(errorPosition > midMostLeft && errorPosition <= rightMostLeft){
    fuzzyError[1]=callErrorPosition(errorPosition, midMostLeft, rightMostLeft, "Low");
  }
  else if(errorPosition > rightMostLeft){
    fuzzyError[1]=0;
  }

  if(errorPosition <= leftCenter){
    fuzzyError[2]=0;
  }
  else if(errorPosition > leftCenter && errorPosition <= midCenter){
    fuzzyError[2]=callErrorPosition(errorPosition, leftCenter, midCenter, "High");
  }
  else if(errorPosition > midCenter && errorPosition <= rightCenter){
    fuzzyError[2]=callErrorPosition(errorPosition, midCenter, rightCenter, "Low");
  }
  else if(errorPosition > rightCenter){
    fuzzyError[2]=0;
  }

  if (errorPosition <= leftMostRight){
    fuzzyError[3]=0;
  }
  else if (errorPosition > leftMostRight && errorPosition <= midMostRight){
    fuzzyError[3]=callErrorPosition(errorPosition, leftMostRight, midMostRight,"High");
  }
  else if(errorPosition > midMostRight && errorPosition <= rightMostRight){
    fuzzyError[3]=callErrorPosition(errorPosition, midMostRight, rightMostRight,"Low");
  }
  else if(errorPosition > rightMostRight){
    fuzzyError[3]=0;
  }

  if (errorPosition <= leftRight){
    fuzzyError[4]=0;
  }
  else if(errorPosition > leftRight && midRight){
    fuzzyError[4]=callErrorPosition(errorPosition, leftRight, midRight, "High");
  }
  else if(errorPosition > midRight){
    fuzzyError[4]=1;
  }
  
  //Delta Error Calculation

  deltaErrorPosition = errorPosition - previousError;

  if(deltaErrorPosition <= midNegative){
    deltaError[0]=1;
  }
  else if(deltaErrorPosition > midNegative && deltaErrorPosition <= rightNegative){
    deltaError[0]=callErrorPosition(deltaErrorPosition, midNegative, rightNegative, "Low");
  }
  else if(deltaErrorPosition > rightNegative){
    deltaError[0]=0;
  }

  if(deltaErrorPosition <= leftMostNegative){
    deltaError[1]=0;
  }
  else if(deltaErrorPosition > leftMostNegative && deltaErrorPosition <= midMostNegative){
    deltaError[1]=callErrorPosition(deltaErrorPosition, leftMostNegative, midMostNegative, "High");
  }
  else if(deltaErrorPosition > midMostNegative && deltaErrorPosition <= rightMostNegative){
    deltaError[1]=callErrorPosition(deltaErrorPosition, midMostNegative, rightMostNegative, "Low");
  }
  else if(deltaErrorPosition > rightMostNegative){
    deltaError[1]=0;
  }

  if(deltaErrorPosition <= leftZero){
    deltaError[2]=0;
  }
  else if(deltaErrorPosition > leftZero && deltaErrorPosition <= midZero){
    deltaError[2]=callErrorPosition(deltaErrorPosition, leftZero, midZero, "High");
  }
  else if(deltaErrorPosition > midZero && deltaErrorPosition <= rightZero){
    deltaError[2]=callErrorPosition(deltaErrorPosition, midZero, rightZero, "Low");
  }
  else if(deltaErrorPosition > rightZero){
    deltaError[2]=0;
  }

  if(deltaErrorPosition <= leftMostPositive){
    deltaError[3]=0;
  }
  else if(deltaErrorPosition > leftMostPositive && deltaErrorPosition <= midMostPositive){
    deltaError[3]=callErrorPosition(deltaErrorPosition, leftMostPositive, midMostPositive, "High");
  }
  else if(deltaErrorPosition > midMostPositive && deltaErrorPosition <= rightMostPositive){
    deltaError[3]=callErrorPosition(deltaErrorPosition, midMostPositive, rightMostPositive, "Low");
  }
  else if(deltaErrorPosition > rightMostPositive){
    deltaError[3]=0;
  }

  if(deltaErrorPosition <= leftPositive){
    deltaError[4]=0;
  }
  else if(deltaErrorPosition > leftPositive && deltaErrorPosition <= midPositive){
    deltaError[4]=callErrorPosition(deltaErrorPosition, leftPositive, midPositive, "High");
  }
  else if(deltaErrorPosition >= midPositive){
    deltaError[4]=1;
  }

  previousError = errorPosition;
}

void rules(){
  devider = 0;
  	for(int i=0; i<=4; i++){
	  	for(int j=0; j<=4; j++){
		  rule[i][j] = min(deltaError[i], fuzzyError[j]);
  		devider = devider + rule[i][j];}}

  rule00 = rule[0][0];
  rule01 = rule[0][1];
  rule02 = rule[0][2];
  rule03 = rule[0][3];
  rule04 = rule[0][4];

  rule10 = rule[1][0];
  rule11 = rule[1][1];
  rule12 = rule[1][2];
  rule13 = rule[1][3];
  rule14 = rule[1][4];

  rule20 = rule[2][0];
  rule21 = rule[2][1];
  rule22 = rule[2][2];
  rule23 = rule[2][3];
  rule24 = rule[2][4];

  rule30 = rule[3][0];
  rule31 = rule[3][1];
  rule32 = rule[3][2];
  rule33 = rule[3][3];
  rule34 = rule[3][4];

  rule40 = rule[4][0];
  rule41 = rule[4][1];
  rule42 = rule[4][2];
  rule43 = rule[4][3];
  rule44 = rule[4][4];

  decision = rule00 * RM + rule01 * RS + rule02 * LS + rule03 * LS + rule04 * LM +
             rule10 * LM + rule11 * LS + rule12 * LS + rule13 * RS + rule14 * RM +
             rule20 * LF + rule21 * LM + rule22 * FW + rule23 * RM + rule24 * RF +
             rule30 * LM + rule31 * LS + rule32 * RS + rule33 * RS + rule34 * RM +
             rule40 * RM + rule41 * RS + rule42 * RS + rule43 * LS + rule44 * LM;
}

void defuzzification(){
  speedMotor = decision/devider;
  rightSpeedVal = RpwmTwo + speedMotor;
  leftSpeedVal = LpwmTwo - speedMotor;
}


void goFuzzy(){
  if(leftSpeedVal>0){  
    analogWrite(MLF, leftSpeedVal);
    analogWrite(MLB, 0);
  }
  else if(leftSpeedVal<0){
    analogWrite(MLF, 0);
    analogWrite(MLB, -leftSpeedVal);
  }
  else if(leftSpeedVal==0){
    analogWrite(MLF, 0);
    analogWrite(MLB, 0);
  }

  if(rightSpeedVal>0){
    analogWrite(MRF, rightSpeedVal);
    analogWrite(MRB, 0);
  }
  else if(rightSpeedVal<0){
    analogWrite(MRF, 0);
    analogWrite(MRB, -rightSpeedVal);
  }
  else if(rightSpeedVal==0){
    analogWrite(MRF, 0);
    analogWrite(MRB, 0);
  }
}
void stop(){
    analogWrite(MLF, 0);
    analogWrite(MLB, 0);
    analogWrite(MRF, 0);
    analogWrite(MRB, 0);
  }

void fuzzy(){
  fuzzyfication();
  rules();
  defuzzification();
}

void encoderMode(){
  yRight = R_encoder_position;
  yLeft = encoder_position;

  yRightTick = (yRight/(6*xEncoder));
  yLeftTick = (yLeft/(6*xEncoder));
  avgEncoder = ((yRightTick+yLeftTick)/2);

  if (avgEncoder >= 300 * n)
  {
    n++;
    // previousX = xKinematic;
    // previousY = yKinematic;
    stop();
    uvActivation();
    delay(60000);
    uvDeActivation();
  }
  Serial.print("AvgDistance:  ");
  Serial.print(avgEncoder);
  Serial.print(" RightDistance: ");
  Serial.print(yRightTick);
  Serial.print(" LeftDistance: ");
  Serial.print(yLeftTick);
}

void checkEncoderTimer(){
  endEncoderMillis = millis();
  encoderTimer = (endEncoderMillis - startEncoderMillis);
  startEncoderMillis = millis();
  Serial.print(" encoderTimer: ");
  Serial.print(encoderTimer);
}

void checkLoopTimer(){
  endTimeMillis = millis();
  loopTimerCheck = endTimeMillis - startTimeMillis;
  startTimeMillis = millis();
  loopTimer = (float)loopTimerCheck/1000;
  Serial.print(" LT: ");
  Serial.print(loopTimer);
}

void forwardKinematic(){
  checkLoopTimer();

  leftPosition = (2*phi*wheelsRadius*encoder_position)/encoderCPR;
  rightPosition = (2*phi*wheelsRadius*R_encoder_position)/encoderCPR;

  checkEncoderTimer();
  
  velocityRight = (leftPosition - previousLeftPosition)*1000/encoderTimer;
  velocityLeft = (rightPosition - previousRightPosition)*1000/encoderTimer;
  Serial.print(" veloRight : ");
  Serial.print(velocityRight);
  Serial.print(" veloLeft: ");
  Serial.print(velocityLeft);

  previousLeftPosition = leftPosition;
  previousRightPosition = rightPosition;

  if ((velocityLeft != velocityRight) && ((velocityLeft > 0 && velocityRight > 0) || (velocityLeft < 0 && velocityRight < 0))){
    angularSpeed = (velocityLeft - velocityRight)/wheelLength;

    radiusIcc = (wheelLength * (velocityLeft + velocityRight)) / (2 * (velocityLeft - velocityRight));

    xPosition = - radiusIcc * sin(lastThetaPosition) + radiusIcc * sin(lastThetaPosition + angularSpeed * loopTimer) + lastXPosition;
    yPosition = radiusIcc * cos(lastThetaPosition) - radiusIcc * cos(lastThetaPosition + angularSpeed * loopTimer) + lastYPosition;
    thetaPosition = lastThetaPosition + angularSpeed * loopTimer;
  }
  else if ((velocityLeft == velocityRight)){
    xPosition = lastXPosition + velocityLeft * loopTimer * cos(lastThetaPosition);
    yPosition = lastYPosition + velocityLeft * loopTimer * sin(lastThetaPosition);
    thetaPosition = lastThetaPosition;  
  }
  else if ((velocityLeft != - velocityRight) && (((velocityLeft > 0) && (velocityRight < 0)) || ((velocityLeft < 0) && (velocityRight > 0)))){
    angularSpeed = (velocityLeft - velocityRight) / wheelLength;

    radiusIcc = (wheelLength * (velocityLeft + velocityRight))/(2 * (velocityLeft - velocityRight));

    xPosition = - radiusIcc * sin(lastThetaPosition) + radiusIcc * sin(lastThetaPosition + angularSpeed * loopTimer) + lastXPosition;
    yPosition = radiusIcc * cos(lastThetaPosition) - radiusIcc * cos(lastThetaPosition + angularSpeed * loopTimer) + lastYPosition;
    thetaPosition = lastThetaPosition + angularSpeed * loopTimer;
  }
  else if ((velocityLeft == -velocityRight)){
    angularSpeed = (velocityLeft - velocityRight) / wheelLength;
    xPosition = lastXPosition;
    yPosition = lastYPosition;
    thetaPosition = lastThetaPosition - angularSpeed * loopTimer;
  }
  else if (((velocityLeft == 0) && (velocityRight != 0)) || ((velocityLeft != 0) && (velocityRight == 0))){
    angularSpeed = (velocityLeft - velocityRight)/wheelLength;

    radiusIcc = 0.5;

    xPosition = - radiusIcc * sin(lastThetaPosition) + radiusIcc * sin(lastThetaPosition + angularSpeed * loopTimer) + lastXPosition;
    yPosition = radiusIcc * cos(lastThetaPosition) - radiusIcc * cos(lastThetaPosition + angularSpeed * loopTimer) + lastYPosition;
    thetaPosition = lastThetaPosition + angularSpeed * loopTimer;
  }

  lastXPosition = xPosition;
  lastYPosition = yPosition;
  lastThetaPosition = thetaPosition; 

  xPositionCm = xPosition;
  yPositionCm = yPosition;
  thetaPositionDegree = ((int)(thetaPosition*360/(2*phi))%360) >=0? ((int)(thetaPosition*360/(2*phi))%360): 360+((int)(thetaPosition*360/(2*phi))%360);

  Serial.print(" Xpostition : ");
  Serial.print(xPosition);
  Serial.print(" yPosition : ");
  Serial.print(yPosition);
  Serial.print(" ThetaPostDeg : ");
  Serial.println(thetaPositionDegree);
}


// void kinematic(){
//   encoderMode();
//   ML = wheelCircumference*((yLeftTick-previousValueLeft)/encoderPPR);
//   MR = wheelCircumference*((yRightTick-previousValueRight)/encoderPPR);

//   previousValueLeft = yLeftTick;
//   previousValueRight = yRightTick;

//   midValue=((MR+ML)/2);
//   Teta_ch=((MR-ML)/(2*wheelLength));

//   Teta = Teta+Teta_ch; //Orientation
 
//   yKinematic = yKinematic + (midValue*cos(Teta)); // Turn Right or Left
//   xKinematic = xKinematic + (midValue*sin(Teta)); // Forward

//   deltaX = xKinematic - previousX;
//   deltaY = yKinematic - previousY;
  
//   deltaPositionKinematic = sqrt(deltaX+deltaY);
  
//   //avgEncoder = ((yRightTick+yLeftTick)/2);
//   if(deltaPositionKinematic >= 300*n){
//     n++;
//     previousX = xKinematic;
//     previousY = yKinematic;
//     stop();
//     uvActivation();
//     delay(60000);
//     uvDeActivation();
//   }
//   Serial.print(" EncRight: ");
//   Serial.print(R_encoder_position);
//   Serial.print(" EncLeft :");
//   Serial.print(encoder_position);
//   Serial.print(" Teta: ");
//   Serial.print(Teta);
//   Serial.print(" DeltaPos: ");
//   Serial.print(deltaPositionKinematic);
//   Serial.print(" xValue: ");
//   Serial.print(xKinematic);
//   Serial.print(" yValue: ");
//   Serial.println(yKinematic);
// }

void getFuzzy(){
  getSensor();
  if(check == true){
    fuzzy();
    //encoderMode();
    goFuzzy();
    forwardKinematic();
    check = false;
    // Serial.print(" Decision: ");
    // Serial.print(decision);
    // Serial.print(" Devider: ");
    // Serial.print(devider);
    // Serial.print(" errorP: ");
    // Serial.print(errorPosition);
    // Serial.print(" DeltaEP: ");
    // Serial.print(deltaErrorPosition);
    // Serial.print(" Rpwm: ");
    // Serial.print(rightSpeedVal);
    // Serial.print(" Lpwm: ");
    // Serial.print(leftSpeedVal);
    // Serial.print(" speedMotor: ");
    // Serial.println(speedMotor);
  }
}

void uvActivationReset()
{
  digitalWrite(UB, rellayOff);
  digitalWrite(UF, rellayOff);
}

// Variable Pergerakan


void voltageDivider(){
  float aref = 5;
  float rOne = 10000;
  float rTwo = 5800;
  float analogValue = 1023;
  float rOne_two = 45000;
  float rTwo_two = 6800;

  voltageDividerOne = analogRead(A8);
  Serial.print("Output Analog 12V: ");
  Serial.print(voltageDividerOne);

  voltageDividerTwo = analogRead(A9);
  Serial.print(" Output Analog 24V: ");
  Serial.print(voltageDividerTwo);

  outputOne = (voltageDividerOne*aref *(rOne + rTwo))/(analogValue*rTwo);
  Serial.print(" Output 12V: ");
  Serial.print(outputOne);

  outputTwo = (voltageDividerTwo*aref*(rOne_two+rTwo_two))/(analogValue*rTwo_two);
  Serial.print(" Output 24V: ");
  Serial.println(outputTwo);
}

void outputRemote(){
  ch1 = pulseIn(CH1, HIGH); // 
  // Serial.print(" ch1: "); 
  // Serial.print(ch1);
  ch2 = pulseIn(CH2, HIGH); //
  // Serial.print(" ch2: ");
  // Serial.print(ch2);
  ch3 = pulseIn(CH3, HIGH); //
  // Serial.print(" ch3: ");
  // Serial.print(ch3);
  ch4 = pulseIn(CH4, HIGH); //
  // Serial.print(" ch4: ");
  //  Serial.print(ch4);
  ch5 = pulseIn(CH5, HIGH); //
  // Serial.print(" ch5: ");
  // Serial.print(ch5);
  ch6 = pulseIn(CH6, HIGH); //
  // Serial.print(" ch6: ");
  // Serial.println(ch6);
}

void rcMode(){
  int deadzone = 15;
  outputRemote();
  int throt = map(ch3, min, max, 0, 255);
  // Serial.print("Throt: ");
  // Serial.print(throt);
  
  int x = map(ch1, min, max, -throt, throt);
  if(x>-deadzone && x<deadzone) {
    x = 0;
  }
  // Serial.print(" x: ");
  // Serial.print(x);

  int y = map(ch2, min, max, -throt, throt);
  if(y>-deadzone && y<deadzone) {
    y = 0;
  }
  // Serial.print(" y: ");
  // Serial.print(y);

  int leftSpeed = y + x;
  int rightSpeed = y - x;
  leftSpeed = constrain(leftSpeed, -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);
  // Serial.print(" LS: ");
  // Serial.print(leftSpeed);
  // Serial.print(" RS: ");
  // Serial.print(rightSpeed);

  if (leftSpeed == 0){
    analogWrite(MLF, 0);
    analogWrite(MLB, 0);
  }
  else if (leftSpeed > 0){
    analogWrite(MLF, leftSpeed);
    analogWrite(MLB, 0);
  }
  else {
    analogWrite(MLF, 0);
    analogWrite(MLB, -leftSpeed);
  }

  // right Motor Setup
  if (rightSpeed == 0){
    analogWrite(MRF, 0);
    analogWrite(MRB, 0);
  }
  else if (rightSpeed > 0){
    analogWrite(MRF, rightSpeed);
    analogWrite(MRB, 0);
  }
  else {  
    analogWrite(MRF, 0);
    analogWrite(MRB, -rightSpeed);
  }
}


void PID(){
    // PID 

  getSensor();
  if(trackDetect == 1){
  error = setPointSensor - AnalogDetect;
  pValue = Kp * error; 
  derivatifValue = ((Kd/Ts)*(error-lastError));
  lastError=error;  
  proporsionalDerivatif = pValue + derivatifValue;
  Rpwm = RsetPointPwm + proporsionalDerivatif; // Proposional - Derivatif (PD)
  Lpwm = LsetPointPwm - proporsionalDerivatif;

  goForward();
  // Rpwm = RsetPointPwm + pValue; // Proporsional
  // Lpwm = LsetPointPwm - pValue;
  // Serial.print("PD: ");
  // Serial.print(proporsionalDerivatif);
  // Serial.print(" P: ");
  // Serial.print(pValue);
  // Serial.print(" D: ");
  // Serial.print(derivatifValue);
  // Serial.print(" Error: ");
  // Serial.print(error);
  // Serial.print(" trackDetect: ");
  // Serial.print(trackDetect);
  // Serial.print(" AnalogOutput: ");
  // Serial.print(AnalogDetect);
  // Serial.print(" Rpwm: ");
  // Serial.print(Rpwm);
  // Serial.print(" Lpwm: ");
  // Serial.println(Lpwm);
}
else if (trackDetect==0){
    stop();
}
}
void ultraSonic(){
  
  Wire.beginTransmission(113);
  Wire.write(byte(0x00));
  Wire.write(byte(0x50));
  Wire.endTransmission();

  delay(65);

  Wire.beginTransmission(113);
  Wire.write(byte(0x02));
  Wire.endTransmission();
  Wire.requestFrom(113, 2);
  
  if (2 <= Wire.available()){
    reading=Wire.read();
    reading=reading << 8;
    reading |= Wire.read();
    Serial.print("Ultrasonic: ");
    Serial.println(reading);
  }
  delay(250);
}


void setup() {
  Wire.beginTransmission(113);
  Master.begin(115200);
  Serial.begin(115200);
  pinMode(MRF, OUTPUT);
  pinMode(MRB, OUTPUT);
  pinMode(MLF, OUTPUT);
  pinMode(MLB, OUTPUT);

  pinMode(UB, OUTPUT);
  pinMode(UF, OUTPUT);
  pinMode(BUZZ, OUTPUT);
  pinMode(ACTIVE_RL, OUTPUT);

  pinMode(CH2, INPUT);
  pinMode(CH3, INPUT);
  pinMode(CH4, INPUT);
  pinMode(CH5, INPUT);
  pinMode(CH6, INPUT);
  //Buzzer();
  Buzzer();
  uvActivationReset();
}


void loop() {
  getFuzzy();
  //uvActivation();
  //ultraSonic();
  //PID();
  //getSensor();
  //voltageDivider();
  //rcMode();
  //kinematic();
}





// // ---------------- Slave ----------------

// // Open The Command if want to upload on Slave
// #include <Arduino.h>
// #include <Wire.h>
// // Slave to Master 
// #define Slave Serial3

// // Slave Read Magnetic Sensor 
// #define ForkRight 12
// #define ForkLeft 13
// #define RightMarker 7
// #define LeftMarker 6
// #define TrackPresent A2
// #define AnalogOut A0

// // Encoder
// enum {ENC_STOP, ENC_CLOCKWISE_ROTATION, ENC_COUNTERCLOCKWISE_ROTATION};
// const byte SINPin = 4;
// const byte COSPin = 2;
// const byte R_SINPin = 5;
// const byte R_COSPin = 3;
// volatile byte encoder_state = ENC_STOP;
// volatile int encoder_position = 0; 
// volatile int encoder_oldpos = 0;

// volatile byte R_encoder_state = ENC_STOP;
// volatile int R_encoder_position = 0; 
// volatile int R_encoder_oldpos = 0;

// void ForkRightDetect(){
//   digitalWrite(ForkRight, HIGH);
//   digitalWrite(ForkRight, LOW);
// }

// void ForkLeftDetect(){
//   digitalWrite(ForkLeft, HIGH);
//   digitalWrite(ForkLeft, LOW);
// }

// int trackDetect(){
//   int TrackPresentValue = digitalRead(TrackPresent);
//   Serial.print(" TP: ");
//   Serial.print(TrackPresentValue);

//   return TrackPresentValue;
// }

// int AnalogOutDetect(){
//   int AnalogOutValue = analogRead(AnalogOut);
//   Serial.print(" AO: ");
//   Serial.print(AnalogOutValue);
  
//   return AnalogOutValue;
// }

// int RightMarkerDetect() {
//   int RightMarkerValue = digitalRead(RightMarker);
//   Serial.print(" RM: ");
//   Serial.print(RightMarkerValue);

//   return RightMarkerValue;
// }

// int LeftMarkerDetect(){
//   int LeftMarkerValue = digitalRead(LeftMarker);
//   Serial.print(" LM: ");
//   Serial.println(LeftMarkerValue);

//  return LeftMarkerValue;
// }

// void encoder_isr() {
  
//   if  (digitalRead(SINPin) == LOW) {
//     // clockwise rotation
//     encoder_state=ENC_CLOCKWISE_ROTATION;
//     encoder_position++;
//   } else {
//     //counter-clockwise rotation
//     encoder_state=ENC_COUNTERCLOCKWISE_ROTATION;
//     encoder_position--;    
//   } 
// }
// void R_encoder_isr() {
  
//   if  (digitalRead(R_SINPin) == HIGH) {
//     // clockwise rotation
//     R_encoder_state=ENC_CLOCKWISE_ROTATION;
//     R_encoder_position++;
//   } else {
//     //counter-clockwise rotation
//     R_encoder_state=ENC_COUNTERCLOCKWISE_ROTATION;
//     R_encoder_position--;    
//   } 
// }
// void encoder(){
//   if (encoder_oldpos == encoder_position) encoder_state= ENC_STOP;

//   // output encoder incremental and status
//   Serial.print("Left Encoder position: ");
//   Serial.print(encoder_position);
//   //Serial.print(",Left Encoder state: ");
  
//   encoder_oldpos = encoder_position;

//   if (R_encoder_oldpos == R_encoder_position) R_encoder_state= ENC_STOP;

//   // output encoder incremental and status
//   Serial.print(" Right Encoder position: ");
//   Serial.println(R_encoder_position);
//   //Serial.println(",Right Encoder state: ");
  
//   R_encoder_oldpos = R_encoder_position;
// }
// void sendData(){
//   encoder();
//   //Magnetic Sensor
//   Slave.print("!");
//   delay(20);
//   Slave.print(trackDetect());
//   delay(20);

//   Slave.print("@");
//   delay(20);
//   Slave.print(AnalogOutDetect());
//   delay(20);
  
//   Slave.print("#");
//   delay(20);
//   Slave.print(RightMarkerDetect());
//   delay(20);
  
//   Slave.print("$");
//   delay(20);
//   Slave.print(LeftMarkerDetect());
//   delay(20);  

//  // Encoder
//   Slave.print("%");
//   Slave.print(R_encoder_position);

//   Slave.print("^");
//   Slave.print(encoder_position);
// }

// void setup() {

//   Slave.begin(115200);
//   Serial.begin(115200);

//   pinMode(ForkRight, OUTPUT);
//   pinMode(ForkLeft, OUTPUT);
//   pinMode(RightMarker, INPUT);
//   pinMode(LeftMarker, INPUT);
//   pinMode(TrackPresent, INPUT);
//   pinMode(AnalogOut, INPUT);
//   pinMode(COSPin, INPUT_PULLUP);
//   pinMode(SINPin, INPUT);
//   pinMode(R_COSPin, INPUT_PULLUP);
//   pinMode(R_SINPin, INPUT);
//   attachInterrupt(digitalPinToInterrupt(COSPin), encoder_isr, RISING);
//   attachInterrupt(digitalPinToInterrupt(R_COSPin), R_encoder_isr, RISING);
// }

// void loop() {
//   //encoder();
//   sendData();
// }