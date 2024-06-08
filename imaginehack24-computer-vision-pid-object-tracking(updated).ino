/*    
    PORTA Register
    ---------------------------
    (MSB) 0 0 0 0 0 0 0 0 (LSB)
    ---------------------------
    Byte 0: M1 (Pin 22)
    Byte 1: M2 (Pin 23)
    Byte 2: M3 (Pin 24)
    Byte 3: M4 (Pin 25)
    Byte 4: Unused
    Byte 5: Unused
    Byte 6: Unused
    Byte 7: Unused

    -------- ------------
   | Port A |  Register  |
    -------- ------------
   | Pin 22 |     PA0    |
   | Pin 23 |     PA1    |
   | Pin 24 |     PA2    |
   | Pin 25 |     PA3    |
   | Pin 26 |     PA4    |
   | Pin 27 |     PA5    |
   | Pin 28 |     PA6    |
   | Pin 29 |     PA7    |
    -------- ------------

    Copyright© Tung Tze Yang 2024
    For : IMAGINEHACK@Taylor's University 2024
    Edited : 8/6/2024 Tung Tze Yang 2024
*/
#define M1_PWM 6
#define M2_PWM 7
#define M3_PWM 8
#define M4_PWM 9

#define pi 3.141592654
#define rotation_speed 1.5
#define movement_speed 1.3

// defines ultrasnic pins numbers
const int trigPin = 46;
const int echoPin = 47;

// defines variables
long duration;
int distance;

double dt, timeOld = 0;
double integral = 0.00, previous = 0.00;
int error, xPos, basespeed;
double control;

//Manual-movement content
typedef
    union
    {
      struct
      {
        unsigned char joyStatus;          //byte0
        unsigned char joyLeftX;           //byte1
        unsigned char joyLeftY;           //byte2
        unsigned char joyRightX;          //byte3
        unsigned char joyRightY;          //byte4
        struct
        {
          unsigned char  dpad:4;          //byte5
          unsigned char  btnSquare:1;
          unsigned char  btnCross:1;
          unsigned char  btnCircle:1;
          unsigned char  btnTriangle:1;

          unsigned char  btnL1:1;         //byte6
          unsigned char  btnR1:1;
          unsigned char  btnL2:1;
          unsigned char  btnR2:1;
          unsigned char  btnShare:1;
          unsigned char  btnOptions:1;
          unsigned char  btnL3:1;         
          unsigned char  btnR3:1;
                
          unsigned char  chk:3;           //byte7. First 4 bits not changing, will be used as verification code
          unsigned char  btnMode:1;       //Mode button
          unsigned char  btnTouchPad:1;
          unsigned char  :1;              //unsed
          unsigned char  btnPs:1;         
          unsigned char  :1;              //unused
        }btn;
      }state;
      unsigned char rawdata[8];
}Controller;

Controller gamepad;
unsigned char rx_data[8];
unsigned int buf_overrun = 0;
unsigned long comm_timeout = 0;
unsigned int timeout_duration = 0;
unsigned int i = 0;

boolean btnL1Check = false;
boolean btnR1Check = false;
boolean btnL2Check = false;
boolean btnR2Check = false;
boolean btnPsCheck = false;
boolean stepperCheck1 = false;
boolean stepperCheck2 = false;
boolean stepperCheck3 = false;
boolean stepperCheck4 = false;
boolean btnTriangleCheck = false;
boolean btnCircleCheck = false;
boolean btnCrossCheck = false;
boolean btnSquareCheck = false;
boolean btnTouchpadCheck = false;

boolean autocheck = false;

int serial_retrieve(){
    if (Serial2.available() > 0){
    String input = Serial2.readStringUntil('\n');  // Read the string until a newline character is encountered
    Serial.println("Received: " + input);

    // Parse the string to extract X and Y values
    int spaceIndex = input.indexOf(' ');
    if (spaceIndex != -1) {
      // Extract X and Y substrings
      String xString = input.substring(0, spaceIndex);
      String yString = input.substring(spaceIndex + 1);

      // Convert X and Y substrings to integers
      int xPos = xString.substring(xString.indexOf('X') + 1).toInt();
      int yPos = yString.substring(yString.indexOf('Y') + 1).toInt();

      Serial.print("xPos: ");
      Serial.println(xPos);
      Serial.print("yPos: ");
      Serial.println(yPos);
      return xPos;
    }
  }
}

double pid(double error, double kp, double ki, double kd)
{
  double propotional = error;
  integral += error * dt;

  integral = constrain(integral, -200, 200);

  double derivative = (error - previous) / dt;
  previous = error;
  double output = (kp*propotional) + (ki*integral) + (kd*derivative);
  
  return output;  
}

void motorControl(double control){
  if (control > 0){
    PORTA = B00000000;
    //turn right, clockwise

  }else if (control < 0){
    PORTA = B00001111;
    //turn left, anticlockwise
  }else {
    PORTA = B00000110;

  }
  control = fabs(control);
  int control_int = (int) control;
  analogWrite(M1_PWM, (control_int + basespeed));
  analogWrite(M2_PWM, (control_int + basespeed));
  analogWrite(M3_PWM, (control_int + basespeed));
  analogWrite(M4_PWM, (control_int + basespeed));

}

void setup() {
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT);  // Sets the echoPin as an Input
    
  Serial.begin(115200);   //for monitoring/debugging
  Serial1.begin(115200);  // to FTDI USB Host
  Serial2.begin(115200);  //UART communication with raspberrypi 
  Serial.println("Serial Initialized");

    //Timer 2: Pins 9, 10
  TCCR2B &= ~ _BV (CS22); 
  TCCR2B |= _BV (CS20);
  
  //Timer 4: Pins 6, 7 and 8
  TCCR4B &= ~(_BV(CS42) | _BV(CS41) | _BV(CS40));
  TCCR4B |= _BV(CS40);
  
  DDRA = B00001111;
  PORTA = B00000000;

  timeout_duration = 500;   //set time out duration for serial comm. Max waiting time for 1 byte = 500us (Theoretically 87us)
}

void loop() {
  if (micros() > comm_timeout)
    i = 0;
  
  if (Serial1.available()>0){
    comm_timeout = micros() + timeout_duration;
    if (Serial1.available()>8)
      buf_overrun++;

    gamepad.rawdata[i] = Serial1.read();
    i++;
    if (i == 8)
    {
      i = 0;
      //debug_gamepad();
      process_gamepad();
    }  
  }
}

void process_gamepad(void)
{
  if ((gamepad.state.joyStatus == 1))
  {
    if(gamepad.state.btn.btnTouchPad && !btnTouchpadCheck)
    {
      if(autocheck == false)
      {
        autocheck = true;
      }
      
      else
      {
        autocheck = false;
      }
    }
    btnTouchpadCheck = gamepad.state.btn.btnTouchPad;
  
  
    if(autocheck)
    {
  
            int ly = -(gamepad.state.joyLeftY - 127);
            int lx = gamepad.state.joyLeftX - 128;
            int rx = (gamepad.state.joyRightX - 128) / rotation_speed;
            
            float theta = atan2(lx, ly);
            float mag = sqrt((lx * lx) + (ly * ly));
        
            float M1 = mag * cos(theta + pi / 4) - rx;
            float M2 = mag * -sin(theta + pi / 4) - rx;
            float M3 = mag * -cos(theta + pi / 4) - rx;
            float M4 = mag * sin(theta + pi / 4) - rx;
        
            M1 = M1 * movement_speed;
            M2 = M2 * movement_speed;
            M3 = M3 * movement_speed;
            M4 = M4 * movement_speed;
        
            M1 = round(M1);
            M2 = round(M2);
            M3 = round(M3);
            M4 = round(M4);
        
            M1 = limitValue(M1);
            M2 = limitValue(M2);
            M3 = limitValue(M3);
            M4 = limitValue(M4);
        
            if (M1 > 0) PORTA = PORTA & B11111110;
            else PORTA = PORTA | B00000001;
            
            if (M2 > 0) PORTA = PORTA & B11111101; 
            else PORTA = PORTA | B00000010;         
            
            if (M3 > 0) PORTA = PORTA & B11111011;
            else PORTA = PORTA | B00000100;
        
            if (M4 > 0) PORTA = PORTA & B11110111;  
            else PORTA = PORTA | B00001000;         
         
            analogWrite(M1_PWM, abs(int(M1)));
            analogWrite(M2_PWM, abs(int(M2)));
            analogWrite(M3_PWM, abs(int(M3)));
            analogWrite(M4_PWM, abs(int(M4)));      
      Serial.println("Manual Sequence Started !");    
    }
    else
    {
      
      double now = millis();
      dt = (now -timeOld)/1000.00;
      timeOld =  now;
    
      xPos = serial_retrieve();
      error = -xPos;
      
      if(abs(error) <= 60){
        digitalWrite(trigPin, LOW);
        delayMicroseconds(2);
        // Sets the trigPin on HIGH state for 10 micro seconds
        digitalWrite(trigPin, HIGH);
        delayMicroseconds(10);
        digitalWrite(trigPin, LOW);
        // Reads the echoPin, returns the sound wave travel time in microseconds
        duration = pulseIn(echoPin, HIGH);
        distance = duration * 0.034 / 2;
    
        basespeed = 7ss0;
    
        if (distance <= 20){
           basespeed = map(distance, 0, 20, 0,50);
           Serial.println(basespeed);
        }
    
        control = 0;
        motorControl(control);
    //    Serial.print("Control: ");
    //    Serial.println(control);
    
      }
      else {
        basespeed = 0;
        control = pid(error, 0.2,0.1,0.1);
        motorControl(control);
        Serial.print("Control: ");
        Serial.println(control);
      }
    
      Serial.print("Distance: ");
      Serial.println(distance);
      Serial.println("Auto Sequence Started !");          
    }
  }
}

inline int limitValue(int value) {
  return (value > 255) ? 255 : ((value < -255) ? -255 : value);
}

void debug_gamepad(void){
  
  //For F710, Gamepad status must be 1, chk must be 4, and mode must be off for data to be valid
  //For F310, Gamepad status must be 255, chk must be 0, and mode must be off for data to be valid
  if ((gamepad.state.joyStatus == 1) && (gamepad.state.btn.chk == 4) && (!gamepad.state.btn.btnMode)) 
  {
    Serial.print("LX=");
    Serial.println(gamepad.state.joyLeftX);
  
    Serial.print("LY=");
    Serial.println(gamepad.state.joyLeftY);
  
    Serial.print("RX=");
    Serial.println(gamepad.state.joyRightX);
  
    Serial.print("RY=");
    Serial.println(gamepad.state.joyRightY);

    Serial.print("dpad=");
    Serial.println(gamepad.state.btn.dpad);
  
    if (gamepad.state.btn.btnSquare)
      Serial.println("Square");
  
    if (gamepad.state.btn.btnTriangle)
      Serial.println("Triangle");

    if (gamepad.state.btn.btnCross)
      Serial.println("Cross");
  
    if (gamepad.state.btn.btnCircle)
      Serial.println("Circle");

    if (gamepad.state.btn.btnL1)
      Serial.println("L1");
  
    if (gamepad.state.btn.btnR1)
      Serial.println("R1");

    if (gamepad.state.btn.btnL2)
      Serial.println("L2");
  
    if (gamepad.state.btn.btnR2)
      Serial.println("R2");

    if (gamepad.state.btn.btnShare)
      Serial.println("Share");
  
    if (gamepad.state.btn.btnOptions)
      Serial.println("Options");

    if (gamepad.state.btn.btnL3)
      Serial.println("L3");
      
    if (gamepad.state.btn.btnR3)
      Serial.println("R3");      

    if (gamepad.state.btn.btnPs)
      Serial.println("PS button");
  
    if (gamepad.state.btn.btnTouchPad)
      Serial.println("TouchPad");
  } 
}
