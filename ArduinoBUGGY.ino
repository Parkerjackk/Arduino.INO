#include <WiFiNINA.h>
#include <PID_v1_bc.h>
#include <Encoder.h>

char ssid[] = "GROUPZ1";
char pass[] = "sillycunt";

WiFiClient client;
IPAddress server(192, 168, 169, 179);

int Lspeed, Rspeed;
double defSpeed = 255; //establishing the default speed at 255, the max value output by the motors

//SETTING PINS
//LEFT MOTOR
const int LS = A6;
const int LM1 = 3;
const int LM2 = 5;
//RIGHT MOTOR
const int RS = A5;
const int RM2 = 4;
const int RM1 = 2;
//IR SENSORS
const int LEFT_IR = A1;
const int RIGHT_IR = A0;
//US SENSORS
const int ECHO = A2;
const int TRIG = A3;

//DEFINING VARIABLE WHICH WILL BE USED IN CALCULATIONS.
double kp = 0.15;
double ki = 0.01;
double kd = 0.0165;
double input, distance, output;
double setPoint = 20;
bool direction = true;
String d;
String oldString = String(""); //initialising oldString variable with a value.

//creating a PID Object, "myPID" and setting the parameters it will take in.
PID myPID(&input, &output, &setPoint, kp, ki, kd, direction);

char cmd = 'C'; //used later on in the code to see if a alphabetical value is received by the server

//encoder pin & variables which will be used when calculating the velocity of the buggy and object.
const int encoder1 = 11; 
const float ticks_per_rev = 16;
float cmps;
float oldV;
float rps;
Encoder encoder(encoder1, encoder1); //creating an encoder object
const long encPulsesPerRev = 16; //declaring the amount of pulses per revolution

// Time and speed measurement
double velocity2;
String velocity;
unsigned long prevMillis = 0; //initialising the previous time to 0
const unsigned long interval = 1000;  //The speed will update every second, aka 1000 miliseconds
long prePos = 0; //initialising the prev. position to 0
//old & new distance initialised as 15cm
double oldd=20; 
double newd=20;

int timeD;
double distanced;
long duration;

//function found in the encoder.h library which calculates the speed by using the time & position
void encoderCalc() {
  unsigned long currentMillis = millis();

  if (currentMillis - prevMillis >= interval) {
    //Calculating time diff.
    timeD = currentMillis - prevMillis;
    prevMillis = currentMillis;

    // Read the current position from the encoder
    long currPos = encoder.read();

    rps = ((currPos - prePos) / float(encPulsesPerRev)) / (timeD / 1000);
    cmps = (rps * 0.325 * 100 * 2 * PI);

    newd = distance;
    distanced = newd - oldd;
    velocity2 = cmps + (distanced / timeD);
    oldd = newd;
    prePos = currPos; //at the end of the function, the current position is equal to the previous position for the next iteration of the function
  }
}

void forward() //left and right motor operate at defSpeed in a forward motion
{
  analogWrite(LS, defSpeed);
  analogWrite(RS, defSpeed);
  digitalWrite(LM1, HIGH);
  digitalWrite(RM1, HIGH);
  digitalWrite(LM2, LOW);
  digitalWrite(RM2, LOW);
}

void stop()//left and right motor operate at defSpeed in a backwards motion
{
  analogWrite(LS, defSpeed);
  analogWrite(RS, defSpeed);
  digitalWrite(LM1, LOW);
  digitalWrite(RM1, LOW);
  digitalWrite(LM2, LOW);
  digitalWrite(RM2, LOW);
}

void goLeft() //The left motor isnt provided power while their is power provided to the right motor in a forward motion, therefore rotating the buggy towards the left.
{
  analogWrite(LS, 0);
  analogWrite(RS, defSpeed);
  digitalWrite(LM1, LOW);
  digitalWrite(RM1, HIGH);
  digitalWrite(LM2, LOW);
  digitalWrite(RM2, LOW);
}

void goRight() //The left motor is powered in a forward motion, while the right isnt powered, therefore rotating the buggy towards the right
{
  analogWrite(LS, defSpeed);
  analogWrite(RS, 0);
  digitalWrite(LM1, HIGH);
  digitalWrite(RM1, LOW);
  digitalWrite(LM2, LOW);
  digitalWrite(RM2, LOW);
}

//Function created to use the IR sensors to follow a white line.
void lineFollowing() {
  int IR_LEFT = digitalRead(LEFT_IR);
  int IR_RIGHT = digitalRead(RIGHT_IR);

  if (IR_LEFT == 1 && IR_RIGHT == 1) // If the value read in by the IR sensors is high, aka black, the buggy continues to move forward as it means that 
  {
    stop();
  } 
  else if (IR_LEFT == 1 && IR_RIGHT == 0) //If the value read in by the left sensor is high while the value read in by the right sensor is low, the buggy will turn right
  {
    goRight();
  } 
  else if (IR_LEFT == 0 && IR_RIGHT == 1) //If the value from the left sensor is low and the right is high, the buggy will turn left
  {
    goLeft();
  }

  else if (IR_LEFT == 0 && IR_RIGHT == 0) //If both values are low, the buggy will go forward.
  {
    forward();
  }
}

double distanceTest(){
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  duration = pulseIn(ECHO, HIGH); //value is read in from the echo pin
  distance = duration * 0.034 / 2; //distance is calculated
  return distance;
}

void BronzeTest() {
  distance = distanceTest(); //call distance test to get a distance value.

  if (distance >= 20) //if the distance is equal or greater to 20cm, the buggy will continue to follow the line
  {
    lineFollowing();
  } else //otherwise, it will stop and the distance will be sent to the client
    {
    stop();
    Server.write("Object detected!!")
    Serial.println(distance);
    delay(500);
    }
}

void SilverFollow() {
  int IR_LEFT = digitalRead(LEFT_IR);
  int IR_RIGHT = digitalRead(RIGHT_IR);
  defSpeed = 230 * output;
  if (defSpeed > 230) defSpeed = 230;
  else if (defSpeed < 0) defSpeed = 0;

  //line following
  if (distance > 50){
  stop();
  } 
  if (distance > 20 && distance < 50) {
   lineFollowing();
  }

  else if (distance < 20){
    stop();}
}

//Function created for the silver challeng
void silverChallenge() {
  distance = distanceTest();
  //Calculated the distance as previously done above, this distance will be used as the input for the PID calculations.
  input = distance;
  myPID.Compute(); //PID is then computed as it has all the values it needs
  SilverFollow(); //Runs another function

  encoderCalc(); //Runs the encoderCalc function
  
  if (cmps != oldV) //If the old velocity is not equal to the new velocity, the velocity is computed and sent to the server. 
  {
    velocity = String(cmps);
    d = String(velocity2, 4);
    d = d + "S" + String(velocity);
    client.write(d.c_str(), d.length());
    oldV = cmps; // the new velocity then becomes the old velocity
  }
}

void setup() {
  Serial.begin(9600);
    //initialise the connection to the WiFi connection & server
  WiFi.begin(ssid, pass);
  client.connect(server, 5200);
  //print the IP address of the buggy
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Adress: ");
  Serial.println(ip);
  //Initialise all pins to either input or output
  pinMode(LEFT_IR, INPUT); 
  pinMode(RIGHT_IR, INPUT);  
  pinMode(LM1, OUTPUT);
  pinMode(LM2, OUTPUT);
  pinMode(RM1, OUTPUT);
  pinMode(RM2, OUTPUT);
  pinMode(ECHO, INPUT);
  pinMode(TRIG, OUTPUT);
  prevMillis = millis();
  //setup PID object as explained by library
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(0, 1);

}

void loop() {

  char c = client.read(); //reads in a value sent by the server
  if (isAlpha(c)) //If the value is alphanumeric, we store it in cmd and print out the value
  {
    cmd = c;
    Serial.println(cmd);
  } else // otherwise, it does nothing
    ;
  if (cmd == 'G') //if the letter 'A' is received, the arduino will run the code that makes it follow as line, the bronze code.
  { BronzeTest(); }
  if (cmd == 'S') //if the letter 'C' is received, the arduino will STOP
   { stop(); }
  if (cmd == 'A') //finally, if the letter 'N' is received, the arduino will run the code that stored in the silverChallenge function.
  {
    silverChallenge();
    delay(50);
  }
}
