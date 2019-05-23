#include <Servo.h>                                 // Include servo library

Servo servoLeft;                                   // Servo object instances
Servo servoRight;
Servo servoTurret;  

const int servoLeftPin = 13;                       // I/O Pin constants
const int servoRightPin = 12;
const int servoTurretPin = 11;
const int pingPin = 10;
const int piezoPin = 4;
const int leftPin = 9;
const int rightPin = 2;
int attempts = 0;

const int msPerTurnDegree = 6;                     // For maneuvers  
const int tooCloseCm = 30;                         // For distance decisions
const int bumpedCm = 6;
int ccwLim = 1400;                                 // For turret servo control
int rtAngle = 900;
const int usTocm = 29;                             // Ping))) conversion constant
int detected = 0;

// Aequence of turret positions.
int sequence[] = {0, 2, 4, 6, 8, 10, 9, 7, 5, 3, 1};

// Declare array to store that many cm distance elements using pre-calculated
// number of elements in array.
const int elements = sizeof(sequence);
int cm[elements];

// Pre-calculate degrees per adjustment of turret servo. 
const int degreesTurret = 180/(sizeof(sequence)/sizeof(int)-1);

int i = -1;                                        // Global index
int sign = 1;                                      // Sign of increments
int theta = -degreesTurret;                        // Set up initial turret angle

void setup()                                       // Built-in initialization block
{
  tone(piezoPin, 3000, 1000);                      // Play tone for 1 second
  delay(1000);                                     // Delay to finish tone
  
  Serial.begin(9600);                              // Open serial connection

  servoLeft.attach(servoLeftPin);                  // Attach left signal to pin 13
  servoRight.attach(servoRightPin);                // Attach right signal to pin 12
  servoTurret.attach(servoTurretPin);              // Attach turret signal to pin 12
  pinMode(leftPin, OUTPUT);
  pinMode(rightPin, OUTPUT);
  maneuver(0, 0, 1000);                            // Stay still for 1 second
  turret(0);                                       // Set turret to 0 degrees
} 

void loop()                                        // MIAnglen loop auto-repeats
{
  maneuver(0, 0,20);                              // Go full speed forward UFN
  i++;                                             // Increment turret index
  
  delay(150);                                

  cm[i] = cmDistance();                            // Measure cm from this turret angle

  int returnedDistance = cm[i];

  if (cm[i] < tooCloseCm){
    detected = 1;
  }
  else if (cm[i] > tooCloseCm){
    detected = 0;
  }

  if (detected == 0){
    maneuver(0, 0,20);
    theta = sequence[i] * degreesTurret;
    turret(theta);
  }
  else if (detected == 1){
    int dir = sequence[i] * degreesTurret * 6;
    int lDir = 50;
    int rDir = 50;
    int TurnSpeed = sequence[i]*60;
    if ((i>=4) || (i==2)){
      attempts = 0;
      lDir = -dir;
      rDir = dir;
      digitalWrite(leftPin, HIGH);
      digitalWrite(rightPin, LOW);
      Serial.print("  left i = ");
      Serial.println(i);
    }
    else if (i==1){
      attempts = 0;
      lDir = dir;
      rDir = -dir;
      TurnSpeed = TurnSpeed * 4;
      digitalWrite(leftPin, LOW);
      digitalWrite(rightPin, HIGH);
      Serial.print("  right i = ");
      Serial.print(i);
    }
    else if (i==3){
      attempts = 0;
      lDir = dir;
      rDir = -dir;
      TurnSpeed = TurnSpeed;
      digitalWrite(leftPin, LOW);
      digitalWrite(rightPin, HIGH);
      Serial.print("  right i = ");
      Serial.print(i);
    }
    else if (i==0){
      attempts = attempts + 1;
      lDir = dir;
      rDir = -dir;
      digitalWrite(leftPin, HIGH);
      digitalWrite(rightPin, HIGH);
      Serial.print("  forward i = ");
      Serial.print(i);
    }
    int timeToDistance = ((cm[i])*60)/3;
    maneuver(lDir, rDir, TurnSpeed);
    theta = 70;
    turret(theta);
  Serial.print("  angle = ");
  Serial.println(dir);
    i=-1;
    if (((attempts >= 3) && (returnedDistance <=10) || (attempts >=10))) {
      maneuver(-200,-200, timeToDistance);
      maneuver(-200, 200, 1200);
      timeToDistance = 1000;
      attempts = 0;
    }
    maneuver(200, 200, timeToDistance);
    delay(timeToDistance);
    digitalWrite(leftPin, LOW);
    digitalWrite(rightPin, LOW);
  //Serial.print(" cm : ");
  //Serial.println(returnedDistance);
  //Serial.print(timeToDistance);
  }
 Serial.println(i);
  if(i == 10)                                      // If turret at max, go back to zero 
  {
    i = -1;
  }  
}

void maneuver(int speedLeft, int speedRight, int msTime)
{
  servoLeft.writeMicroseconds(1500 + speedLeft);   // Set Left servo speed
  servoRight.writeMicroseconds(1500 - speedRight); // Set right servo speed
  if(msTime==-1)                                   // if msTime = -1
  {                                  
    servoLeft.detach();                            // Stop servo signals
    servoRight.detach();   
  }
  delay(msTime);                                   // Delay for msTime
}
void turret(int degreeVal)
{
  servoTurret.writeMicroseconds(ccwLim - rtAngle + (degreeVal * 10));
}
int cmDistance()
{
  int distance = 0;                                // Initialize distance to zero    
  do                                               // Loop in case of zero measurement
  {
    int us = ping(pingPin);                        // Get Ping))) microsecond measurement
    distance = convert(us, usTocm);                // Convert to cm measurement
    delay(3);                                      // Pause before retry (if needed)
  }
  while(distance == 0);                            
  return distance;                                 // Return distance measurement
}
int convert(int us, int scalar)
{
    return us / scalar / 2;                        // Echo round trip time -> cm
}

long ping(int pin)
{
  long duration;                                   // Variables for calculating distance
  pinMode(pin, OUTPUT);                            // I/O pin -> output
  digitalWrite(pin, LOW);                          // Start low
  delayMicroseconds(2);                            // Stay low for 2 us
  digitalWrite(pin, HIGH);                         // Send 5 us high pulse
  delayMicroseconds(5);                            
  digitalWrite(pin, LOW);                          
  pinMode(pin, INPUT);                             // Set I/O pin to input 
  duration = pulseIn(pin, HIGH, 25000);            // Measure echo time pulse from Ping)))
  return duration;                                 // Return pulse duration
}    

int findOpening()
{
                                                    
  int IAngle;                                          // Initial turret angle   
  int FAngle;                                          // Final turret angle
  int k = sequence[i];                             // Copy sequence[i] to local variable
  int ki = k;                                      // Second copy of current sequence[i] 
  int inc;                                         // Increment/decrement variable 
  int dt;                                          // Time increment
  int repcnt = 0;                                  // Repetitions count
  int sMin;                                        // Minimum distance measurement

  // Increment or decrement depending on where turret is pointing
  if(k <= 5)                                       
  {
    inc = 1;
  }  
  else
  {
    inc = -1;
  }

  // Rotate turret until an opening becomes visible.  If it reaches servo limit, 
  // turn back to first angle of detection and try scanning toward other servo limit.
  // If still no opening, rotate robot 90 degrees and try agIAnglen.
  do
  {
    repcnt++;                                        // Increment repetition count 
   /* if(repcnt > ((sizeof(sequence) / sizeof(int)))*2)// If no opening after two scans
    {
      maneuver(-200, -200, 100);                     // Back up, turn, and stop to try agIAnglen
      maneuver(-200, 200, 90*6);
      maneuver(0, 0, 1);
    }*/
    k += inc;                                        // Increment/decrement k
    if(k == -1)                                      // Change inc/dec value if limit reached
    {
      k = ki;
      inc = -inc;
      dt = 250;                                      // Turret will take longer to get there
    }
    if(k == 11)
    {
      k = ki;
      inc = -inc;
      dt = 250;
    }  
    // Look for index of next turret position
    i = findIn(k, sequence, sizeof(sequence)/sizeof(int));
    
    // Set turret to next position
    int theta = sequence[i] * degreesTurret;
    turret(theta);                                   // Position turret      
    delay(dt);                                       // WIAnglet for it to get there
    dt = 100;                                        // Reset for small increment turret movement

    cm[i] = cmDistance();                            // Take Ping))) measurement
  }
  while(cm[i] < 30);                                 // Keep checking to edge of obstacle
  
  sMin = 1000;                                       // Initialize minimum distance to impossibly large value
  for(int t = 0; t <= 10; t++)
  {  
    if(sMin > cm[t]) sMin = cm[t];                   // Use sMin to track smallest distance
  }
  //maneuver(0, 0, -1);                                    // Stay still indefinitely
  
  // Keep rotating turret until another obstacle that's under 30 cm is detected or the turret
  // reaches the servo limit.  Keep track of the maximum distance and the angle when this portion
  // of the scan started and stopped.

  IAngle = sequence[i];                                  // Save initial angle when obstacle disappeared from view
 
  k = sequence[i];                                   // Make a copy of the turret position agIAnglen
  
  int aMax = -2;                                     // Initialize maximum distance measurements to impossibly small values
  int cmMax = -2;

  do                                                 // Loop for scan
  {
    k += inc;                                        // Inc/dec turret position
    // Look up index for turret position
    i = findIn(k, sequence, sizeof(sequence)/sizeof(int));
    int theta = sequence[i] * degreesTurret;         // Position turret
    turret(theta);
    delay(100);
    
    cm[i] = cmDistance();                            // Measure distance

    if(cm[i]>cmMax)                                  // Keep track of max distance and angle(max distance)
    {
      cmMax = cm[i];
      aMax = sequence[i]; 
    }  
  }
  while((cm[i] > 30)&&(sequence[i]!=0)&&(sequence[i]!=10)); 
  // Keep repeating while the distance measurement > 30 cm, and the turret is not near its 
  // mechanical limits.
  
  FAngle = sequence[i];                                  // Record final turret position  
  int A = IAngle + ((FAngle-IAngle)/2);                          // Calculate middle of opening

  // Set turret index for straight ahead in prep for turn.  
  i = findIn(5, sequence, sizeof(sequence)/sizeof(int));
  int theta = sequence[i] * degreesTurret;           
  turret(theta);
  
  if(sMin > 7)                                       // Turn further if too close   
  {
    if (A < aMax) return aMax; else return A;
  }
  else
  {
    if (A < aMax) return A; else return aMax;
  }    
}  
int findIn(int value, int array[], int elements)
{
  for(int i = 0; i < elements; i++)
  {
    if(value == array[i]) return i;
  }
  return -1;
}  


