// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// clamp                digital_out   A               
// ---- END VEXCODE CONFIGURED DEVICES ----


#include "vex.h" //introduce the vex libraries
using namespace vex; //allow certain abbreviations



//define the controller
controller Controller1 = controller();


//define all of the motors
//front left
motor leftFrontBottom = motor(PORT15, ratio6_1,true);
motor leftFrontMid = motor(PORT7, ratio6_1,false);
motor leftFrontTop = motor(PORT3, ratio6_1,true);
motor_group leftFront(leftFrontBottom, leftFrontMid, leftFrontTop);

//front right
motor rightFrontBottom = motor(PORT1, ratio6_1,false);
motor rightFrontMid = motor(PORT8, ratio6_1,true);
motor rightFrontTop = motor(PORT16, ratio6_1,false);
motor_group rightFront(rightFrontBottom, rightFrontMid, rightFrontTop);


//back left
motor leftBackBottom = motor(PORT20, ratio6_1,true);
motor leftBackMid = motor(PORT13, ratio6_1,false);
motor leftBackTop = motor(PORT5, ratio6_1,true);
motor_group leftBack(leftBackBottom, leftBackMid, leftBackTop);

//back right
motor rightBackBottom = motor(PORT6, ratio6_1,false);
motor rightBackMid = motor(PORT18, ratio6_1,true);
motor rightBackTop = motor(PORT11, ratio6_1,false);
motor_group rightBack(rightBackBottom, rightBackMid, rightBackTop);

//other motors
motor intake = motor(PORT12, ratio6_1,true);
motor lift = motor(PORT19, ratio36_1,true);
motor hooks = motor(PORT2, ratio18_1,false);

//code for colour sensor
optical ringSort = optical(PORT17);

//code for odom sensor
motor ESP = motor(PORT4, ratio18_1, false);


//introduce Competition format
competition Competition;


//initialize constants
double pi = 3.141592654;


//odometry variables
float angle;
float x;
float y;

float deltaTrack = 0;

float angleStartOffset = 0;
float xStartOffset = 0;
float yStartOffset = 0;



//hook controls
bool manual = false;
float hookTarget = 0;
char myColour = ' ';


int colourDetect(){
  if(ringSort.hue() > 300 || ringSort.hue() < 30){
    if(myColour == 'r'){
      return 1;
    }
    else if(myColour == 'b'){
      return -1;
    }    
  }
  else if(ringSort.hue() < 270 & ringSort.hue() > 130){
    if(myColour == 'b'){
      return 1;
    }
    else if(myColour == 'r'){
      return -1;
    }    
  }  
  return 0;
}

//task that will call the print info function
int printInfo() {
  while (true) {
     //reset the cursor and screen
    Brain.Screen.clearScreen();
    int row = 1;
    Brain.Screen.setCursor(row,1);
    row++;

    //print motor stats
    Brain.Screen.print("Motor Name    Deg RPM  Watts  Volts  Amps"); //print headers
    Brain.Screen.newLine();


    Brain.Screen.print("Front Left:"); //print motor name
    Brain.Screen.setCursor(row,15); 
    Brain.Screen.print(int(leftFront.temperature(temperatureUnits::celsius))); //print the temperature of the motor
    Brain.Screen.setCursor(row,19); 
    Brain.Screen.print(int(leftFront.velocity(rpm))); //print motor speed
    Brain.Screen.setCursor(row,24);
    Brain.Screen.print((leftFront.power(watt))); //print motor power
    Brain.Screen.setCursor(row,31);
    Brain.Screen.print((leftFront.voltage(volt))); //print motor voltage 
    Brain.Screen.setCursor(row,38);
    Brain.Screen.print(leftFront.current(amp)); //print motor current   
    Brain.Screen.newLine();
    row++;

    Brain.Screen.print("Back Left:"); //print motor name
    Brain.Screen.setCursor(row,15); 
    Brain.Screen.print(int(leftBack.temperature(temperatureUnits::celsius))); //print the temperature of the motor
    Brain.Screen.setCursor(row,19); 
    Brain.Screen.print(int(leftBack.velocity(rpm))); //print motor speed
    Brain.Screen.setCursor(row,24);
    Brain.Screen.print(leftBack.power(watt)); //print motor power
    Brain.Screen.setCursor(row,31);
    Brain.Screen.print(leftBack.voltage(volt)); //print motor voltage 
    Brain.Screen.setCursor(row,38);
    Brain.Screen.print(leftBack.current(amp)); //print motor current    
    Brain.Screen.newLine();
    row++;




      Brain.Screen.print("Front Right:"); //print motor name
    Brain.Screen.setCursor(row,15); 
    Brain.Screen.print(int(rightFront.temperature(temperatureUnits::celsius))); //print the temperature of the motor
    Brain.Screen.setCursor(row,19); 
    Brain.Screen.print(int(rightFront.velocity(rpm))); //print motor speed
    Brain.Screen.setCursor(row,24);
    Brain.Screen.print(rightFront.power(watt)); //print motor power
    Brain.Screen.setCursor(row,31);
    Brain.Screen.print(rightFront.voltage(volt)); //print motor voltage   
    Brain.Screen.setCursor(row,38);
    Brain.Screen.print(rightFront.current(amp)); //print motor current    
    Brain.Screen.newLine();
    row++;

    Brain.Screen.print("Back Right:"); //print motor name
    Brain.Screen.setCursor(row,15); 
    Brain.Screen.print(int(rightBack.temperature(temperatureUnits::celsius))); //print the temperature of the motor
    Brain.Screen.setCursor(row,19); 
    Brain.Screen.print(int(rightBack.velocity(rpm))); //print motor speed
    Brain.Screen.setCursor(row,24);
    Brain.Screen.print(rightBack.power(watt)); //print motor power
    Brain.Screen.setCursor(row,31);
    Brain.Screen.print(rightBack.voltage(volt)); //print motor voltage
    Brain.Screen.setCursor(row,38);
    Brain.Screen.print(rightBack.current(amp)); //print motor current     
    Brain.Screen.newLine();
    row++;




    Brain.Screen.print("Lift:"); //print motor name
    Brain.Screen.setCursor(row,15); 
    Brain.Screen.print(int(lift.temperature(temperatureUnits::celsius))); //print the temperature of the motor
    Brain.Screen.setCursor(row,19); 
    Brain.Screen.print(int(lift.velocity(rpm))); //print motor speed
    Brain.Screen.setCursor(row,24);
    Brain.Screen.print(lift.power(watt)); //print motor power
    Brain.Screen.setCursor(row,31);
    Brain.Screen.print(lift.voltage(volt)); //print motor voltage 
    Brain.Screen.setCursor(row,38);
    Brain.Screen.print(lift.current(amp)); //print motor current    
    Brain.Screen.newLine();
    row++;

    Brain.Screen.print("Intake:"); //print motor name
    Brain.Screen.setCursor(row,15); 
    Brain.Screen.print(int(intake.temperature(temperatureUnits::celsius))); //print the temperature of the motor
    Brain.Screen.setCursor(row,19); 
    Brain.Screen.print(int(intake.velocity(rpm))); //print motor speed
    Brain.Screen.setCursor(row,24);
    Brain.Screen.print(intake.power(watt)); //print motor power
    Brain.Screen.setCursor(row,31);
    Brain.Screen.print(intake.voltage(volt)); //print motor voltage 
    Brain.Screen.setCursor(row,38);
    Brain.Screen.print(intake.current(amp)); //print motor current    
    Brain.Screen.newLine();
    row++;

    Brain.Screen.print("Hooks:"); //print motor name
    Brain.Screen.setCursor(row,15); 
    Brain.Screen.print(int(hooks.temperature(temperatureUnits::celsius))); //print the temperature of the motor
    Brain.Screen.setCursor(row,19); 
    Brain.Screen.print(int(hooks.velocity(rpm))); //print motor speed
    Brain.Screen.setCursor(row,24);
    Brain.Screen.print(hooks.power(watt)); //print motor power
    Brain.Screen.setCursor(row,31);
    Brain.Screen.print(hooks.voltage(volt)); //print motor voltage 
    Brain.Screen.setCursor(row,38);
    Brain.Screen.print(hooks.current(amp)); //print motor current    
    Brain.Screen.newLine();
    row++;


    for(int i = 1; i < 13; i++){ //create divider
    Brain.Screen.setCursor(i,43);     
    Brain.Screen.print("|");
    }


    Brain.Screen.setCursor(1,44); //print x value 
    Brain.Screen.print("X pos");
    Brain.Screen.setCursor(2,44);    
    Brain.Screen.print(x);


    Brain.Screen.setCursor(4,44);   //print y value 
    Brain.Screen.print("Y pos");
    Brain.Screen.setCursor(5,44);   
    Brain.Screen.print(y);


    Brain.Screen.setCursor(7,44);   //print angle 
    Brain.Screen.print("Theta");
    Brain.Screen.setCursor(8,44);   
    Brain.Screen.print(angle); 
    /*
    Brain.Screen.setCursor(9,44);   
    Brain.Screen.print(colourDetect());     
    Brain.Screen.setCursor(10,44);   
    Brain.Screen.print(ringSort.hue());     */  

    /*
    Controller1.Screen.clearScreen(); //clear the screen
    Controller1.Screen.setCursor(1, 1); //reset the cursor


    Controller1.Screen.print("Timer: "); //print the time remaining
    Controller1.Screen.print((90- int(Brain.Timer.value())));




    Controller1.Screen.setCursor(2, 1); //print the battery level
    Controller1.Screen.print("Battery: ");
    Controller1.Screen.print(int(Brain.Battery.capacity()));*/

    /*
    //buz the controller at 30, 15 and 5 seconds
    if(Brain.Timer.value() <61 && Brain.Timer.value() >59){
      Controller1.rumble("-");
    }
    else if(Brain.Timer.value() <76 && Brain.Timer.value() >74){
      Controller1.rumble(".");
    } 
    else if(Brain.Timer.value() <86 && Brain.Timer.value() >84){
      Controller1.rumble(".");
    }   */



    
    wait(0.05, sec); //  Brain refresh rate
 }
 
}

void setInitialPosition(float initialX, float initialY, float initialA){
  uint8_t sendBuffer[12];
    memcpy(&sendBuffer[0], &initialX, sizeof(float));
    memcpy(&sendBuffer[4],&initialY, sizeof(float));
    memcpy(&sendBuffer[8],&initialA, sizeof(float));
    vexGenericSerialTransmit(ESP.index(), sendBuffer, sizeof(sendBuffer));
    /*
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("Data sent!");
    */
    wait(0.022, sec);
}

//function to calculate robot position
int odom(){
    vexGenericSerialEnable(ESP.index(), 0); // Enable UART
    vexGenericSerialBaudrate(ESP.index(), 9600); // Match baud rate with ESP32
    for(int i = 0;i < 5;i++){
    
      setInitialPosition(x, y, -1*angle);

    }

    uint8_t buffer[4]; // Buffer to store the 4 bytes of the float
    float receivedNumber = 0.0; // Variable to store the received float

    float xPrev = x;
    float yPrev = y;
    float anglePrev = angle;
    float newAngle = angle;

    wait(0.2, sec);

    while (true) {
        
        
        // Check if 4 bytes of data are available
        int length = vexGenericSerialReceive(ESP.index(), buffer, sizeof(buffer));
        if (length == sizeof(buffer)) { // Expecting exactly 4 bytes
          // Reconstruct the float from the 4 bytes
          memcpy(&receivedNumber, buffer, sizeof(receivedNumber)); // Copy bytes into float
        }
        /*
        Brain.Screen.clearScreen();
        Brain.Screen.setCursor(1, 1);
        Brain.Screen.print("Received: %.2f", receivedNumber); // Print with 2 decimal places
        */

        if(receivedNumber <= 11000 && receivedNumber > 9000) { 
          newAngle = -1*(receivedNumber-10000 +angleStartOffset);
        } else if (receivedNumber > 11000 && receivedNumber <= 25000){
          x = receivedNumber-20000 + xStartOffset;
        } else if (receivedNumber > 25000){
          y = receivedNumber-30000 + yStartOffset;
        }
        

        deltaTrack = sqrt((x - xPrev)*(x - xPrev) + (y - yPrev)*(y - yPrev));
        if(leftFront.velocity(rpm) < 0){
          deltaTrack *= -1;
        }
    
        float deltaAngle = newAngle - anglePrev;
        if (deltaAngle > 180) {
            deltaAngle -= 360;
        } else if (deltaAngle < -180) {
            deltaAngle += 360;
        }

        angle += deltaAngle;
        

        yPrev = y;
        xPrev = x;
        anglePrev = angle;

        wait(0.02, sec);
    }
}



int autoHook() {
  while(0 == 0){

    if(manual == false){
      if(hooks.position(degrees) <  hookTarget){
        hooks.spin(forward, 12, volt);
        if(colourDetect() == -1){
          if(hooks.velocity(rpm) > 10){
            hooks.stop();
          }
          else{
            hooks.spin(forward, 12, volt);
          }
        }
      }
      else{
        hooks.stop();
      }
    }
    else{
      hookTarget = hooks.position(degrees);
    }

    if(hooks.position(deg) > 260){
      hooks.setPosition(hooks.position(degrees) - 260, deg);
      hookTarget = hookTarget - 260;
    }
    else if(hooks.position(deg) < 0){
      hooks.setPosition(hooks.position(degrees) + 260, deg);
      hookTarget = hookTarget + 260;
    }   


    wait(0.05, sec);

  }
}




//input: desired angle
//output: robot turns to the angle
void turnToAngle(float desiredAngle, float minAccuracy, float timeout, float additionalAction){
 
  //define speed control varialbes
  double kP =0.095;//.10, .095
  double kD =0.04725;//.04, .03
  double kI = 0.0038095238;//.005, .006
  float integralBound = 6;




  //define other variables
  double integral = 0;
  double error = desiredAngle - angle;
  double prevError = error;
  double motorSpeed = 0;     
  float clock = 0;
  float counter = 0;


  //repeated section
  while(counter < 6 && clock < timeout ){
  
    //update current position
    error = desiredAngle - angle;


    //give optimal speed
    motorSpeed = error * kP + kD *(error- prevError) + kI * integral;    

    /*if( fabs(leftFront.velocity(rpm)) > 400){
      if(motorSpeed > 0){
        motorSpeed = 8;

      }
      else{
        motorSpeed = -8;

      }
    }*/

    //state if the robot is within the target 
    if(!(angle +minAccuracy < desiredAngle  || angle -minAccuracy > desiredAngle)){
      counter= counter +1;   
    }
    else{
      //reset counter
      counter = 0;


    }


    //power the motors
    leftFront.spin(forward, motorSpeed , volt);
    leftBack.spin(forward, motorSpeed , volt);
    rightBack.spin(forward, -motorSpeed, volt); 
    rightFront.spin(forward, -motorSpeed, volt); 
    
    //increment timer
    if (fabs(error) < integralBound){
      integral = prevError + integral;
    }
    printf("Integral: %f\n", integral);

    if ((error>0 && prevError <0)||(error<0 && prevError>0)){
      integral = 0;
      printf("intreset\n");
    }

    prevError = error;

    wait(0.02, sec);
    clock   = clock + 0.02; 
  }


  //stop the robot
  leftBack.stop();
  rightBack.stop();
  leftFront.stop();
  rightFront.stop();


  Controller1.rumble(".");
}


//input: desired point
//output: robot turns to face the point
void turnToPoint(float desiredX, float desiredY , float minAccuracy, float timeout, float additionalAction){
  float deltaY = desiredY - y; //Find change in Y
  float deltaX = desiredX - x; //Find change in X
  float desiredAngle;


  if(deltaX==0){  //acount for the possibility of an undefined slope
    desiredAngle=0;
  }
  else{
    float slope = deltaY/deltaX;  //Calculate Slope
    desiredAngle = 90- atan(slope) * 180/pi; //turn slope into a bearing angle
    if(desiredAngle >= 0 && desiredAngle <= 90 && x > desiredX){  //account for arctan only return quadrant 1 and 4 value
      desiredAngle = desiredAngle + 180;
    }


    if(desiredAngle >= 90 && desiredAngle <=180 && x > desiredX){
      desiredAngle = desiredAngle - 180;
    }
  }


  while(desiredAngle > angle + 180){
    desiredAngle = desiredAngle - 360;
  }
  while(desiredAngle < angle - 180){
    desiredAngle = desiredAngle + 360;
  } 
    turnToAngle(desiredAngle, minAccuracy, timeout, additionalAction);  //turn to the angle function
} 


//input: desired angle
//output: robot turns to the angle
void driveForwardDistance(float length, float minAccuracy, float timeout, float additionalAction){


  //define speed control varialbes
  double kP =0.220;
  double kD =0.0039375;
  double kI = 0.00571428571;
  float integralBound = 8;


  //define other variables
  double integral = 0;
  double error = length;
  double prevError = error;
  double motorSpeed = 0;     
  float clock = 0;
  float counter = 0;

  //define max speed
  float maxSpeed = 6; 

  //repeated section
  while(counter < 20 && clock < timeout ){
  
    //update current position
    error = error - deltaTrack;


    //give optimal speed
    motorSpeed = error * kP + kD *(error- prevError) + kI * integral;    


    //state if the robot is within the target
    if(!(error + minAccuracy< 0  || error - minAccuracy > 0)){
      counter++;
    }   
    else{
      //reset count if outside range     
      counter = 0;
    }

    if(motorSpeed > maxSpeed){
      motorSpeed = maxSpeed;
    }
    else if(motorSpeed < -maxSpeed){
      motorSpeed = -maxSpeed;
    }

    //power the motors
    leftFront.spin(forward, motorSpeed , volt);
    leftBack.spin(forward, motorSpeed , volt);
    rightFront.spin(forward, motorSpeed, volt); 
    rightBack.spin(forward, motorSpeed, volt); 


    
    //increment timer

    if (fabs(error) < integralBound){
      integral = prevError + integral;
    }


    if ((error>0 && prevError <0)||(error<0 && prevError>0)){
      integral = 0;
    }




    if (additionalAction == 2){
        if(hooks.position(degrees) < 1850){
            hooks.spin(forward, 12, volt);
        }
        else{
          hooks.stop();
        }
    }


    if (additionalAction == 3){
        if(error < 2 && error > -2){
          clamp.set(true);
        }


    }



    prevError = error;

    printf("\nTime: %f\n", Brain.Timer.value());
    printf("Error: %f\n", error);
    printf("DeltaTrack: %f\n", deltaTrack);
    printf("Counter: %f\n", counter);
    printf("X: %f\n", x);


    wait(0.02, sec);
    clock   = clock + 0.02; 


  }


  //stop the robot
  leftBack.stop();
  rightBack.stop();
  leftFront.stop();
  rightFront.stop();
  Controller1.rumble(".");


}


//input: how far you want to drive and a set speed
//outuput: drives that distance at a constant speed
void driveSpeedDistance(float length, float speed, float timeout, float additionalAction){


  float clock = 0;
  double error = length; 


  if(speed > 0 && length < 0){
    speed = speed * -1;
  }
    if(length > 0){
    while(error > 0 && clock < timeout  ){


    error = error + deltaTrack;


      leftFront.spin(forward, speed , volt);
      leftBack.spin(forward, speed, volt);     
      rightFront.spin(forward, speed , volt);
      rightBack.spin(forward, speed, volt);     

      wait(0.05, sec);
      clock = clock + 0.05;
    }
  }
  else{
    while(error < 0 && clock < timeout ){


    error = error + deltaTrack;


      leftFront.spin(forward, speed , volt);
      leftBack.spin(forward, speed, volt);     
      rightFront.spin(forward, speed , volt);
      rightBack.spin(forward, speed, volt);       


      wait(0.05, sec);
      clock = clock + 0.05;
    }
  }
  leftBack.stop();
  rightBack.stop();
  leftFront.stop();
  rightFront.stop();
}




//input: the distqance you want to be from a point
//output: moves robot to that distance
//not: can only be used if looking at the point
void driveDistanceFrom(float length, float pointX, float pointY,float minAccuracy, float timeout,float additionalAction){

  float deltaX = pointX -x;  //find x component of distance
  float deltaY = pointY - y; //find y component of distance


  double distanceFromPoint = sqrt(deltaX*deltaX +deltaY*deltaY);


  double desiredDistance = distanceFromPoint - length;


  driveForwardDistance(desiredDistance, minAccuracy, timeout , additionalAction);
}


//input: desired coordinates
//output: robot turn to face the coordinates and drives to them
void goTo(float desiredX, float desiredY, float  minAccuracy, float timeout1, float timeout2, float additionalAction){


  turnToPoint( desiredX,  desiredY, minAccuracy, timeout1, 0);


  float deltaY = desiredY - y;
  float deltaX = desiredX - x; 
  double distanceFromPoint = sqrt(deltaX*deltaX +deltaY*deltaY);


  driveForwardDistance(distanceFromPoint, minAccuracy, timeout2 , additionalAction);
}




//code that runs before the match starts
void pre_auton(void) {
  vexcodeInit(); //required competition code
}


void blueAuton(void){
  myColour = 'b';
  manual = true;

  wait(10.5, sec);

  //grab mogo
      leftFront.spin(forward, -3 , volt);
      leftBack.spin(forward, -3, volt);     
      rightFront.spin(forward, -3 , volt);
      rightBack.spin(forward, -3, volt);   
      wait(0.75, sec);

  //score ring
  clamp.set(true);
      leftFront.spin(forward, -3 , volt);
      leftBack.spin(forward, -3, volt);     
      rightFront.spin(forward, -3 , volt);
      rightBack.spin(forward, -3, volt);   
      wait(0.5, sec);
      
      leftFront.stop();
      leftBack.stop();     
      rightFront.stop();
      rightBack.stop();   
  

  wait(1, sec);
  manual = false;
  task hookTask = task(autoHook);


  hookTarget = 500;
  wait(0.5, sec);

  //drive to ring 2
  intake.spin(forward, 12, volt);
  driveForwardDistance(27-y, 1,2.5, 0);
  wait(0.5, sec);
  hookTarget = 3000;
  wait(1.5, sec);
  intake.stop();
  //go to corner
  turnToAngle(angle + 135,1,5, 0);

  //go to corner
  driveForwardDistance(-24, 1,2.5, 0);

  //drop goal
  clamp.set(false);
  driveForwardDistance(30, 1,2.5, 0);

}

void redAuton(void){
  myColour = 'r';
  manual = true;

  wait(10.5, sec);

  //grab mogo
      leftFront.spin(forward, -3 , volt);
      leftBack.spin(forward, -3, volt);     
      rightFront.spin(forward, -3 , volt);
      rightBack.spin(forward, -3, volt);   
      wait(0.75, sec);

  //score ring
  clamp.set(true);
      leftFront.spin(forward, -3 , volt);
      leftBack.spin(forward, -3, volt);     
      rightFront.spin(forward, -3 , volt);
      rightBack.spin(forward, -3, volt);   
      wait(0.5, sec);
      
      leftFront.stop();
      leftBack.stop();     
      rightFront.stop();
      rightBack.stop();   
  

  wait(1, sec);
  manual = false;
  task hookTask = task(autoHook);


  hookTarget = 500;
  wait(0.5, sec);

  //drive to ring 2
  intake.spin(forward, 12, volt);
  driveForwardDistance(27-y, 1,2.5, 0);
  wait(0.5, sec);
  hookTarget = 3000;
  wait(1.5, sec);
  intake.stop();
  //go to corner
  turnToAngle(angle - 135,1,10, 0);

  //go to corner
  driveForwardDistance(-24, 1,2.5, 0);

  //drop goal
  clamp.set(false);
  driveForwardDistance(30, 1,2.5, 0);

}

//autonomous code
void autonomous(void) { 
  //reset the brain timer
  Brain.Timer.reset();

  angle = -90;
  x = 63;
  y= 27;


  //set motor stop type
  leftFront.setStopping(hold);
  rightFront.setStopping(hold);
  leftBack.setStopping(hold);
  rightBack.setStopping(hold);
  lift.setStopping(hold);
  hooks.setStopping(hold);

  //set current
  leftFront.setMaxTorque( 1.83, amp );
  leftBack.setMaxTorque( 1.83, amp );
  rightFront.setMaxTorque( 1.83, amp );
  rightBack.setMaxTorque( 1.83, amp );

  lift.setMaxTorque(1.00, amp);
  intake.setMaxTorque( 2.50, amp );
  hooks.setMaxTorque( 2.50, amp );




  //getting runing odom and printing the interface
  task printing = task(printInfo);
  task getOdom = task(odom);
  task hookTask = task(autoHook);

  //stop the hooks
  manual = false;
  hookTarget = 0;

  //wait for sensor to calibrate
  wait(3.5, sec);


  //drive into mogo
  leftFront.spin(forward, -3 , volt);
  leftBack.spin(forward, -3, volt);     
  rightFront.spin(forward, -3 , volt);
  rightBack.spin(forward, -3, volt);   
  wait(0.75, sec);

  //grab the goal
  clamp.set(true);
  leftFront.spin(forward, -3 , volt);
  leftBack.spin(forward, -3, volt);     
  rightFront.spin(forward, -3 , volt);
  rightBack.spin(forward, -3, volt);   
  wait(0.5, sec);
      
  //wait for goal to settle
  leftFront.stop();
  leftBack.stop();     
  rightFront.stop();
  rightBack.stop();   
  wait(0.5, sec);

  //score first ring
  hookTarget = 260*4;
  wait(1, sec);

  //go to 2nd ringf
  intake.spin(forward, 12, volt);
  driveForwardDistance(x-21, 1.5, 2.5, 0);

  //score the 2nd ring
  wait(1, sec);
  intake.stop();
  hookTarget = 260*6;  
  wait(2, sec);

  //clear the intake
  intake.spin(reverse, 12, volt);
  wait(0.5, sec);
  intake.stop();


  //raise the lift
  while( lift.position(deg) < 345){
    lift.spin(forward, 12, volt);
  }        
  lift.stop();
  intake.spin(forward, 12, volt);

  //go to 3rd ring
  turnToPoint(22, 36, 1.5, 3, 0);
  driveForwardDistance(20,1.5, 2, 0);

  //grab 3rd ring
  while (lift.position(deg) > 25) {
      lift.spin(reverse, 12, volt);
  }  
  lift.spin(reverse, 2, volt);
  wait(0.5, sec);
  lift.stop();

  //score the 3rd ring
  wait(1, sec);
  intake.stop();
  hookTarget = 260*6;  
  wait(2, sec);  

  //turn to corner
  intake.spin(reverse, 12, volt);
  turnToAngle(35,2, 2.5, 0);

  //go back into corner
  intake.stop();
  leftFront.spin(forward, -5 , volt);
  leftBack.spin(forward, -5, volt);     
  rightFront.spin(forward, -5 , volt);
  rightBack.spin(forward, -5, volt);   
  wait(1.5, sec);

  //leave corner
  clamp.set(false);
  lift.spin(forward, 5, volt);
  wait(0.5, sec);
  leftFront.spin(forward, 8 , volt);
  leftBack.spin(forward, 8, volt);     
  rightFront.spin(forward, 8 , volt);
  rightBack.spin(forward, 8, volt);  
  lift.spin(reverse, 5, volt);
  wait(0.75, sec);
  leftFront.spin(forward, 0 , volt);
  leftBack.spin(forward, 0, volt);     
  rightFront.spin(forward, 0 , volt);
  rightBack.spin(forward, 0, volt);  
  lift.stop(); 

  //blueAuton();
  //redAuton();

}


//driver control code
void usercontrol(void) {
  //reset the brain timer
  Brain.Timer.reset();

  angle = -90;
  x = 63;
  y= 27;
  


  //set motor stop type
  leftFront.setStopping(brake);
  rightFront.setStopping(brake);
  leftBack.setStopping(brake);
  rightBack.setStopping(brake);
  lift.setStopping(hold);
  hooks.setStopping(hold);



  //variables to track x drive current
  bool liftInUse = true;
  bool intakeInUse = true;
  bool hooksInUse = true;


  //getting runing odom and printing the interface

  //getting runing odom and printing the interface
  task printing = task(printInfo);
  task getOdom = task(odom);
  task hookTask = task(autoHook);
        


  leftFront.setMaxTorque( 1.83, amp );
  leftBack.setMaxTorque( 1.83, amp );
  rightFront.setMaxTorque( 1.83, amp );
  rightBack.setMaxTorque( 1.83, amp );

  lift.setMaxTorque(1.00, amp);
  intake.setMaxTorque( 2.50, amp );
  hooks.setMaxTorque( 2.50, amp );

  while (0==0) { //repeated tasks


  //Drive control code
  //split style arcade with 3/25 multiplier to convert percent to volts
  leftFront.spin(forward, (Controller1.Axis3.position(percent)  + Controller1.Axis1.position(percent) + Controller1.Axis4.position(percent)) * 3 / 25, volt); // Left Front Drive Code
  leftBack.spin(forward, (Controller1.Axis3.position(percent)  + Controller1.Axis1.position(percent) - Controller1.Axis4.position(percent))* 3 / 25, volt); // Left Back Drive Code
  rightFront.spin(forward,( Controller1.Axis3.position(percent)  - Controller1.Axis1.position(percent) - Controller1.Axis4.position(percent))* 3 / 25, volt); // Right Back Drive Code
  rightBack.spin(forward, (Controller1.Axis3.position(percent)  - Controller1.Axis1.position(percent) + Controller1.Axis4.position(percent))* 3 / 25, volt); // Right Back Drive Code

    //intake
    if (Controller1.ButtonR1.pressing()){
      intake.spin(forward, 100, percent);
      intakeInUse = true;
    }
    else if (Controller1.ButtonR2.pressing()) {
      intake.spin(reverse, 100, percent);
      intakeInUse = true;
    }
    else {
      intake.stop();
      intakeInUse = false;
    }

    //lift
    if (Controller1.ButtonL2.pressing() && lift.position(deg) < 250){
      lift.spin(forward, 12, volt);
      liftInUse = true;
    }    
    else if (Controller1.ButtonL2.pressing() && lift.position(deg) < 325){
      lift.spin(forward, 3, volt);
      liftInUse = true;
    }    
    else if (Controller1.ButtonL2.pressing() && lift.position(deg) < 350){
      lift.spin(forward, 2.05, volt);
      liftInUse = true;
    }        
    else if (Controller1.ButtonL2.pressing() && lift.position(deg) > 360){
      lift.spin(reverse, 0.75, volt);
      liftInUse = true;
    }       
    else if (lift.position(deg) > 25) {
      lift.spin(reverse, 12, volt);
      liftInUse = true;
    }
    else if (lift.position(deg) > 10) {
      lift.spin(reverse, 2, volt);
      liftInUse = true;
    }    
     
    else {
      liftInUse = false;

      lift.stop();
    }

    //hook
   

    if(Controller1.ButtonL1.pressing()){
      manual = false;
      hooksInUse = true;
      //1560 degreses total 260 each

      hookTarget = 260;
      if(hooks.position(degrees) > 245){
        hookTarget = 520;

      }
    
    }
    else if(Controller1.ButtonY.pressing()){
      manual = true;
      hooksInUse = true;
      hooks.spin(reverse, 12, volt);
    }
    else if (Controller1.ButtonA.pressing()){
      manual = true;
      hooksInUse = true;
      hooks.spin(forward, 12, volt);
    }
    else if (manual == true){
      hooks.stop();
      if(fabs(hooks.velocity(rpm)) > 5){
        hooksInUse = true;
      }
      else{
        hooksInUse= false;
      }      
      
    } 


    //mogo clamp code
    if(Controller1.ButtonB.pressing()){
      clamp.set(true);
    }
    else if (Controller1.ButtonX.pressing()){
      clamp.set(false);
    }   
    
    //allocate current
    if(hooksInUse && intakeInUse && liftInUse){
      //set current
      leftFront.setMaxTorque( 1.83, amp );
      leftBack.setMaxTorque( 1.83, amp );
      rightFront.setMaxTorque( 1.83, amp );
      rightBack.setMaxTorque( 1.83, amp );

      lift.setMaxTorque(1.00, amp);
      intake.setMaxTorque( 2.50, amp );
      hooks.setMaxTorque( 2.50, amp );
    }

    else if(hooksInUse && intakeInUse && !(liftInUse)){
      //set current
      leftFront.setMaxTorque( 1.85, amp );
      leftBack.setMaxTorque( 1.85, amp );
      rightFront.setMaxTorque( 1.85, amp );
      rightBack.setMaxTorque( 1.85, amp );

      lift.setMaxTorque(0.06, amp);
      intake.setMaxTorque( 2.50, amp );
      hooks.setMaxTorque( 2.50, amp );      
    }

    else if(hooksInUse && !(intakeInUse) && liftInUse){
      //set current
      leftFront.setMaxTorque( 1.91, amp );
      leftBack.setMaxTorque( 1.91, amp );
      rightFront.setMaxTorque( 1.91, amp );
      rightBack.setMaxTorque( 1.91, amp );

      lift.setMaxTorque(1.00, amp);
      intake.setMaxTorque( 0.10, amp );
      hooks.setMaxTorque( 2.50, amp );      
    }    

    else if(!(hooksInUse) && (intakeInUse) && liftInUse){
      //set current
      leftFront.setMaxTorque( 1.91, amp );
      leftBack.setMaxTorque( 1.91, amp );
      rightFront.setMaxTorque( 1.91, amp );
      rightBack.setMaxTorque( 1.91, amp );

      lift.setMaxTorque(1.00, amp);
      intake.setMaxTorque( 2.50, amp );
      hooks.setMaxTorque( 0.10, amp );      
    }       

    else if(!(hooksInUse) && !(intakeInUse) && (liftInUse)){
      //set current
      leftFront.setMaxTorque( 2.08, amp );
      leftBack.setMaxTorque( 2.08, amp );
      rightFront.setMaxTorque( 2.08, amp );
      rightBack.setMaxTorque( 2.08, amp );

      lift.setMaxTorque(1.00, amp);
      intake.setMaxTorque( 0.10, amp );
      hooks.setMaxTorque( 0.10, amp );      
    }  

    else if(!(hooksInUse) && (intakeInUse) && !(liftInUse)){
      //set current
      leftFront.setMaxTorque( 2.03, amp );
      leftBack.setMaxTorque( 2.03, amp );
      rightFront.setMaxTorque( 2.03, amp );
      rightBack.setMaxTorque( 2.03, amp );

      lift.setMaxTorque(0.06, amp);
      intake.setMaxTorque(2.5, amp );
      hooks.setMaxTorque( 0.10, amp );      
    }

    else if((hooksInUse) && !(intakeInUse) && !(liftInUse)){
      //set current
      leftFront.setMaxTorque( 2.03, amp );
      leftBack.setMaxTorque( 2.03, amp );
      rightFront.setMaxTorque( 2.03, amp );
      rightBack.setMaxTorque( 2.03, amp );

      lift.setMaxTorque(0.06, amp);
      intake.setMaxTorque(0.10, amp );
      hooks.setMaxTorque( 2.5, amp );      
    }    

    else if(!(hooksInUse) && !(intakeInUse) && !(liftInUse)){
      //set current
      leftFront.setMaxTorque( 2.1, amp );
      leftBack.setMaxTorque( 2.1, amp );
      rightFront.setMaxTorque( 2.1, amp );
      rightBack.setMaxTorque( 2.1, amp );

      lift.setMaxTorque(0.06, amp);
      intake.setMaxTorque(0.10, amp );
      hooks.setMaxTorque( 0.1, amp );      
    }      
    

      wait(10, msec); // Sleep the task for a short amount of time to prevent wasted resources.
  }
}




int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);


  // Run the pre-autonomous function.
  pre_auton();
  // Prevent main from exiting with 0an infinite loop.
  while (true) {
    wait(100, msec);
  }
}




