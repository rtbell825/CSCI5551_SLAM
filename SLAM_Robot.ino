#include <Servo.h>
#include <MatrixMath.h>

Servo turnPin;
int turnZero = 85; 
Servo driver;
int driveZero = 80;
int driveStatus = 0;
bool isMoving = false;
Servo sensorArray;
int sensorZero = 90;
int rangePin = 3;

const short MAX_SIGHT_RECORD = 36;
const short MAX_FEATURE_COUNT = 6;
const short MAX_FEATURE_ARRAY = 2; // use 5 when only using line extraction
const short MAX_LANDMARK_COUNT = 6;
const short MAX_POSE_COUNT = 2;
const short ROBOT_BODY_LENGTH = 18; // In centimeters
const short ROBOT_CM_PER_SEC = 3; // 7.4 in ideal conditions
const short LANDMARK_OBS_THRESH = 2;
typedef struct point {
  short x;
  short y;
} Point;
Point sightRecord[MAX_SIGHT_RECORD];
int currentSightRecord = 0;
typedef struct feature {
  Point e0;
  Point e1;
} Feature;
Feature featureList[MAX_FEATURE_COUNT];
short numFeatures = 0;
short detectedFeatures = 0;
Point featureIndexList[MAX_FEATURE_COUNT];
Point landmarks[MAX_LANDMARK_COUNT];
short numLandmarks = 0;
typedef struct featureArray {
  short s;
  Feature featureArray[MAX_FEATURE_ARRAY];
} FeatureArray;
typedef struct pose {
  short x;
  short y;
  short heading;
  short numLandmarks;
  short landmarkList[MAX_FEATURE_COUNT];
} Pose;
Pose poseList[MAX_POSE_COUNT];
short poseCount = 0;
bool firstPose = true;

// SLAM Structures
short Sigma[2*MAX_LANDMARK_COUNT + 3][2*MAX_LANDMARK_COUNT + 3];
short covarTopLeft[3][3];
short covarTopRight[3][2*MAX_LANDMARK_COUNT];
short covarBotLeft[2*MAX_LANDMARK_COUNT][3];
short covarBotRight[2*MAX_LANDMARK_COUNT][2*MAX_LANDMARK_COUNT];
short motionJacobian[3][3];

short robotXPos = 0;
short robotYPos = 0;
// 0 - 360
short robotTheta = 0;

void printArray(int A[], int sizeArr)
{
    int i;
    for (i = 0; i < sizeArr; i++) {
        Serial.print(A[i]);
        Serial.print(" ");
    }
    Serial.println();
}

void printSightRecord()
{
    int i = 0;
    for (i = 0; i < currentSightRecord; i++) {
        Serial.print("(");
        Serial.print(sightRecord[i].x);
        Serial.print(", ");
        Serial.print(sightRecord[i].y);
        Serial.print(")");
        Serial.println();
    }
    Serial.println();
}

void printFeatureList()
{
    Serial.print("Num Features = ");
    Serial.println(numFeatures);
    int i = 0;
    for (i = 0; i < numFeatures; i++) {
      Serial.print("(");
      Serial.print(featureList[i].e0.x);
      Serial.print(", ");
      Serial.print(featureList[i].e0.y);
      Serial.print(")");
      Serial.println();
      Serial.print("(");
      Serial.print(featureList[i].e1.x);
      Serial.print(", ");
      Serial.print(featureList[i].e1.y);
      Serial.print(")");
      Serial.println();
      Serial.println();
    }
    Serial.println();
}

void printFeatureArray(FeatureArray featureArray) {
  int i = 0;
  for (i = 0; i < featureArray.s; i++) {
    Serial.print("(");
    Serial.print(featureArray.featureArray[i].e0.x);
    Serial.print(", ");
    Serial.print(featureArray.featureArray[i].e0.y);
    Serial.print(")");
    Serial.println();
    Serial.print("(");
    Serial.print(featureArray.featureArray[i].e1.x);
    Serial.print(", ");
    Serial.print(featureArray.featureArray[i].e1.y);
    Serial.print(")");
    Serial.println();
    Serial.println();
  }
  Serial.println();
}

void printPoseList() {
  int i = 0;
  for (i = 0; i < poseCount; i++) {
    Serial.print("Pose ");
    Serial.println(i);
    Serial.print("x = ");
    Serial.println(poseList[i].x);
    Serial.print("y = ");
    Serial.println(poseList[i].y);
    Serial.print("heading = ");
    Serial.println(poseList[i].heading);
    Serial.print("Num Landmarks = ");
    Serial.println(poseList[i].numLandmarks);
    for (int j = 0; j < poseList[i].numLandmarks; j++) {
      Serial.print("landmark ");
      Serial.print(j);
      Serial.print(" = ");
      Serial.println(poseList[i].landmarkList[j]);
    }
    Serial.println();
    Serial.println();
  }
  Serial.println();
}

float rad_to_deg(float rad) {
  return (rad * (180/PI));
}

float deg_to_rad(int deg) {
  float rads = deg * (PI/180);
  return rads;
}

// Convert sensor reading to x and y coords
Point rangeToPoint(int range, int thetaOffset) {
  Point i = {0, 0};
  i.x = short(range * (cos(deg_to_rad(robotTheta - 90 + thetaOffset)))) + robotXPos;
  i.y = short(range * (sin(deg_to_rad(robotTheta - 90 + thetaOffset)))) + robotYPos;
  return i;
}

// Reverse angle for the steering servo
int reverseTurn(int deg) {
  return (180 - deg);
}

void turnLeft() {
  turnPin.write(40);
  delay(500);
}

void turnRight() {
  turnPin.write(120);
  delay(500);
}

void turnCenter() {
  turnPin.write(turnZero);
  delay(500);
}

// Find orthogonal projection of a line to the origin
Point FindProjection(float slope, Point endpoint) {
  float inverseSlope = -1/slope;
  float b = endpoint.y - (slope*endpoint.x);
  Point proj;
  proj.x = (short)(b / (slope - inverseSlope));
  proj.y = (short)(inverseSlope * proj.x);
  return proj;
}

// Nearest Neighbor feature association
short CompareLandmarks(Feature candidate) {
  // Detect landmarks that overlap by comparing their slopes within a threshold and then comparing them projected onto (0, 0). 
  float candidateSlope = slope(candidate.e0, candidate.e1);
  short minDist = 34000;
  short minIndex = -1;
  for (int i = 0; i < numLandmarks; i++) {
    // Includes slope threshold
    Point candidateProj = FindProjection(candidateSlope, candidate.e0);
    int dist = (int)sqrt((pow((candidateProj.x - landmarks[i].x), 2.0) + (pow((candidateProj.y - landmarks[i].y), 2.0))));
    //Landmark distance threshold
    if (dist < minDist && dist < 30) {
      minDist = dist;
      minIndex = i;
    }
  }
  return minIndex;
}

long microsecondsToCentimeters(long microseconds) {
  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the distance of the object we
  // take half of the distance travelled.
  return microseconds / 29 / 2;
}

// Using a differential model of motion determine how long to run rear motor to turn to a selected heading. 
int Determine_Run_Duration(short target, bool backing) {
  float theta1 = deg_to_rad((-1 * target) + 90);
  theta1 = tan(theta1);
  float radius = sqrt((pow(((-1 * ROBOT_BODY_LENGTH)/theta1) - 0.0, 2.0) + pow(-18.0, 2.0)));
  float circumference = 2 * PI * radius;
  float theta2 = asin(ROBOT_BODY_LENGTH / radius);
  float travel = (int)((circumference * rad_to_deg(theta2))/360);
  if (backing) {
    travel = circumference / 8;
    theta2 = 30.0;
  }
  int duration = (int)(travel / ROBOT_CM_PER_SEC) * 1000;
  // Move forward for a reasonable amount of time if we're pointing forward 
  if (duration < 2000) {
    duration = 4000;
  }

  // Going right
  if (target > 90) {
    robotXPos = (short)(robotXPos + (radius * cos(deg_to_rad(robotTheta - theta2))));
    robotYPos = (short)(robotYPos + (radius * sin(deg_to_rad(robotTheta - theta2))));
    motionJacobian[0][2] = (short)(radius * -sin(deg_to_rad(robotTheta - theta2)));
    motionJacobian[1][2] = (short)(radius * cos(deg_to_rad(robotTheta - theta2)));
  }
  else if(target <= 90) {
    robotXPos = (short)(robotXPos + (radius * cos(deg_to_rad(robotTheta + theta2))));
    robotYPos = (short)(robotYPos + (radius * sin(deg_to_rad(robotTheta + theta2))));
    motionJacobian[0][2] = radius * -sin(deg_to_rad(robotTheta + theta2));
    motionJacobian[1][2] = radius * cos(deg_to_rad(robotTheta + theta2));
  }
  
  return duration;
}

// Use range sensor to get distance
long getRange() {
  long duration, cm;

  pinMode(rangePin, OUTPUT);
  digitalWrite(rangePin, LOW);
  delayMicroseconds(5);
  digitalWrite(rangePin, HIGH);
  delayMicroseconds(8);
  digitalWrite(rangePin, LOW);

  pinMode(rangePin, INPUT);
  duration = pulseIn(rangePin, HIGH);

  cm = microsecondsToCentimeters(duration);
  return cm;
}

// Rotate sensor to collect range points surronding robot
int checkRange() {
  long duration, cm;

  // Fail if buffer is full
  if (currentSightRecord >= MAX_SIGHT_RECORD) {
    return -1;
  }

  // Collect 36 samples of depths
  for (int i = 0; i <= 180; i += 5) {
    sensorArray.write(i); 
    delay(80);
    int range = getRange();
    if (range <= 200) {
      sightRecord[currentSightRecord] = rangeToPoint(range, i);
    }
    else {
      Point negPoint;
      negPoint.x = 0;
      negPoint.y = 0;
      sightRecord[currentSightRecord] = negPoint;
    }
    currentSightRecord = currentSightRecord + 1;
  }
  sensorArray.write(sensorZero);
  delay(80);

  printSightRecord();
  
  return 1;
}

// Use incremental line extraction and then IEPF to extract feature endpoints
void findFeatures() {
   checkRange();
   
   detectedFeatures = Incremental_Line_Extraction();

   //printFeatureList();

  int index = 0;
   for (int i = 0; i < detectedFeatures; i++) {
    FeatureArray featureArray = IEPF_Line_Extraction(featureIndexList[i].x, featureIndexList[i].y, 0);

    for (int j = 0; j < featureArray.s; j++) {
      featureList[numFeatures] = featureArray.featureArray[j];
      numFeatures = numFeatures + 1;
    }
   }
   printFeatureList();
}

// Record the current status of the robot
void recordPose() {
  Serial.print("numFeatures ");
  Serial.print(numFeatures);
  Serial.print("\n");
  if (firstPose == true) {
    poseList[0].x = robotXPos;
    poseList[0].y = robotYPos;
    poseList[0].heading = robotTheta;
    for (int i = 0; i < numFeatures; i++) {
      float featureSlope = slope(featureList[i].e0, featureList[i].e1);
      Point proj = FindProjection(featureSlope, featureList[i].e0);
      landmarks[i].x = proj.x;
      landmarks[i].y = proj.y;
      poseList[0].landmarkList[i] = i;
    }
    numLandmarks = numFeatures;
    poseList[0].numLandmarks = numFeatures;
    poseCount = 1;
    firstPose = false;
  }
  else {
    poseList[poseCount].x = robotXPos;
    poseList[poseCount].y = robotYPos;
    poseList[poseCount].heading = robotTheta;
    for (int i = 0; i < numFeatures; i++) {
      short matchingLandmarkIndex = CompareLandmarks(featureList[i]);
      Serial.print("Feature ");
      Serial.print(i);
      Serial.print(" = Landmark ");
      Serial.print(matchingLandmarkIndex);
      Serial.print("\n");
      if (matchingLandmarkIndex > -1) {
        poseList[poseCount].landmarkList[poseList[poseCount].numLandmarks] = matchingLandmarkIndex;
        poseList[poseCount].numLandmarks = poseList[poseCount].numLandmarks + 1;
      }
      else {
        float featureSlope = slope(featureList[i].e0, featureList[i].e1);
        Point proj = FindProjection(featureSlope, featureList[i].e0);
        // Add new landmark to landmarks
        landmarks[numLandmarks].x = proj.x;
        landmarks[numLandmarks].y = proj.y;
        poseList[poseCount].landmarkList[poseList[poseCount].numLandmarks] = numLandmarks;
        Serial.print("Pose numLandmarks ");
        Serial.print(poseList[poseCount].numLandmarks);
        Serial.print(" = Global numLandmarks ");
        Serial.print(numLandmarks);
        Serial.print("\n");
        poseList[poseCount].numLandmarks = poseList[poseCount].numLandmarks + 1;
        numLandmarks = numLandmarks + 1;
      }
    }
  }

  printPoseList();
}

// Select heading and drive for a duration to reach that heading. Then predict new location
void executeMotion() {
  // Find largest open space between landmarks
  short selectedHeading = -1;
  int duration = 0;
  bool forward = true;
  if (detectedFeatures > 1) {
    for (int i = 1; i < detectedFeatures; i++) {
      // Look for an open space of 20deg within the range the wheels can turn
      if ((featureIndexList[i].x - featureIndexList[i-1].y > 6) && (featureIndexList[i-1].y > 5 || featureIndexList[i].x < 31)){
        selectedHeading = floor((featureIndexList[i].x - featureIndexList[i-1].y)/2) + featureIndexList[i-1].y;
      }
    }

    if (selectedHeading < 0) {
      selectedHeading = 180;
      forward = false;
    }
    else {
      selectedHeading = 180 - (5*selectedHeading);
    }
  }
  else {
    selectedHeading = 90;
  }
  
  // Set limits on heading and turn wheels
  if (selectedHeading < 40) {
    turnLeft();
    delay(500);
  }
  else if (selectedHeading > 120) {
    turnRight();
    delay(500);
  }
  else {
    turnPin.write(selectedHeading);
    delay(500);
  }
  

  // Set the wheels in motion and guesimate new robot position
  if (forward) {
    //Determine how long to continue in chosen direction
    duration = Determine_Run_Duration(selectedHeading, false);
    
    checkDriveStatus(1);
    robotTheta = ((180 - selectedHeading) - 90) + robotTheta;
    if (robotTheta >= 360) {
      robotTheta = robotTheta - 360;
    }
    else if(robotTheta < 0) {
      robotTheta = robotTheta + 360;
    }
  }
  else {
    //Determine how long to continue in chosen direction
    duration = Determine_Run_Duration(140, true);
    checkDriveStatus(-1);
    
    robotTheta = robotTheta + 30;
    if (robotTheta >= 360) {
      robotTheta = robotTheta - 360;
    }
  }

  Serial.print("Robot Theta: ");
  Serial.print(robotTheta);
  Serial.print("\n");
  Serial.print("Heading: ");
  Serial.print(180 - selectedHeading);
  Serial.print("\n");
  Serial.print("Duration: ");
  Serial.print(duration);
  Serial.print("\n");
  
  delay(duration);

  turnCenter();
  delay(200);
  checkDriveStatus(0);
  delay(1000);
  
}

// EKF Prediction step
void EKF_SLAM_Prediction() {
  short temp_matrix[3][3];
  short invJ[3][3];
  short tempTop[3][2*MAX_LANDMARK_COUNT];

  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < (2*MAX_LANDMARK_COUNT); j++) {
      tempTop[i][j] = covarTopRight[i][j];
    }
  }
  
  // Update covar matrix (Fx * coar * inv(Fx) + Error) Assume no error for now
  Matrix.Multiply((short*)motionJacobian, (short*)covarTopLeft, 3, 3, 3, (short*)temp_matrix);
  Matrix.Transpose((short*)motionJacobian, 3, 3, (short*)invJ);
  Matrix.Multiply((short*)temp_matrix, (short*)invJ, 3, 3, 3, (short*)covarTopLeft);

  Matrix.Multiply((short*)motionJacobian, (short*)tempTop, 3, 3, 2*MAX_LANDMARK_COUNT, (short*)covarTopRight);
  Matrix.Transpose((short*)covarTopRight, 3, 2*MAX_LANDMARK_COUNT, (short*)covarBotLeft);
}

// Update covariance matrix
void UpdateSigma() {
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      Sigma[i][j] = covarTopLeft[i][j];
    }
  }
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 2*MAX_LANDMARK_COUNT; j++) {
      Sigma[i][j+3] = covarTopRight[i][j];
    }
  }
  for (int i = 0; i < 2*MAX_LANDMARK_COUNT; i++) {
    for (int j = 0; j < 3; j++) {
      Sigma[i+3][j] = covarBotLeft[i][j];
    }
  }
  for (int i = 0; i < 2*MAX_LANDMARK_COUNT; i++) {
    for (int j = 0; j < 2*MAX_LANDMARK_COUNT; j++) {
      Sigma[i+3][j+3] = covarBotRight[i][j];
    }
  }
}

// EKF correction step (uses too much memory)
void EKF_SLAM_Correction() {
  short m = poseList[poseCount].numLandmarks;
  short Z[2*MAX_LANDMARK_COUNT + 3];
  short ExpectedZ[2*MAX_LANDMARK_COUNT + 3];
  short DeltaZ[2*MAX_LANDMARK_COUNT + 3];
  short DeltaZt[2*MAX_LANDMARK_COUNT + 3][1];
  short H[2*MAX_LANDMARK_COUNT + 3][2*MAX_LANDMARK_COUNT + 3];
  short Ht[2*MAX_LANDMARK_COUNT + 3][2*MAX_LANDMARK_COUNT + 3];
  short temp_matrix[2*MAX_LANDMARK_COUNT + 3][2*MAX_LANDMARK_COUNT + 3];
  short HQ[2*MAX_LANDMARK_COUNT + 3][2*MAX_LANDMARK_COUNT + 3];
  short Si[2*MAX_LANDMARK_COUNT + 3][2*MAX_LANDMARK_COUNT + 3];
  short K[2*MAX_LANDMARK_COUNT + 3][2*MAX_LANDMARK_COUNT + 3];

  H[0][0] = 1;
  H[1][1] = 1;
  H[2][2] = 1;

  for (int i = 0; i < m; i++) {
    int landmarkNum = poseList[poseCount].landmarkList[i];
    // Then computer where you can visually see the landmarks are. 
    float featureSlope = slope(featureList[i].e0, featureList[i].e1);
    Point proj = FindProjection(featureSlope, featureList[i].e0);
    short landmarkX = proj.x;
    short landmarkY = proj.y;
    short q = (short)sqrt((pow((landmarkX - robotXPos), 2.0) + (pow((landmarkY - robotYPos), 2.0))));
    float robotSlope = tan(atan(robotYPos/robotXPos) - robotTheta);
    float landmarkSlope = (robotYPos - landmarkY)/(robotXPos - landmarkX);
  
    Z[2*landmarkNum + 3] = q;
    Z[2*landmarkNum + 4] = (short)atan((landmarkSlope - robotSlope)/(1 + (robotSlope * landmarkSlope))); 
    
    // First compute where you think the landmarks should be based on where they were. 
    landmarkX = landmarks[poseList[poseCount].landmarkList[i]].x;
    landmarkY = landmarks[poseList[poseCount].landmarkList[i]].y;
    short deltax = landmarkX - robotXPos;
    short deltay = landmarkY - robotYPos;
    q = (short)sqrt((pow((deltax), 2.0) + (pow((deltay), 2.0))));
    robotSlope = tan(atan(robotYPos/robotXPos) - robotTheta);
    landmarkSlope = (robotYPos - landmarkY)/(robotXPos - landmarkX);
  
    ExpectedZ[2*landmarkNum + 3] = q;
    ExpectedZ[2*landmarkNum + 4] = (short)atan((landmarkSlope - robotSlope)/(1 + (robotSlope * landmarkSlope)));

    H[2*landmarkNum + 3][0] = -sqrt(q)*deltax/q;
    H[2*landmarkNum + 3][1] = -sqrt(q)*deltay/q;
    H[2*landmarkNum + 3][2] = 0;
    H[2*landmarkNum + 4][0] = deltay/q;
    H[2*landmarkNum + 4][1] = -deltax/q;
    H[2*landmarkNum + 4][2] = -1;

    H[2*landmarkNum + 3][2*landmarkNum + 3] = sqrt(q)*deltax/q;
    H[2*landmarkNum + 3][2*landmarkNum + 4] = sqrt(q)*deltay/q;
    H[2*landmarkNum + 4][2*landmarkNum + 3] = -deltay/q;
    H[2*landmarkNum + 4][2*landmarkNum + 4] = deltax/q;
  }

  Matrix.Transpose((short*)H, 2*MAX_LANDMARK_COUNT+3, 2*MAX_LANDMARK_COUNT+3, (short*)Ht);
  UpdateSigma();
  Matrix.Multiply((short*)H, (short*)Sigma, 2*MAX_LANDMARK_COUNT+3, 2*MAX_LANDMARK_COUNT+3, 2*MAX_LANDMARK_COUNT, (short*)temp_matrix);
  Matrix.Multiply((short*)temp_matrix, (short*)Ht, 2*MAX_LANDMARK_COUNT+3, 2*MAX_LANDMARK_COUNT+3, 2*MAX_LANDMARK_COUNT, (short*)HQ);
  Matrix.Transpose((short*)HQ, 2*MAX_LANDMARK_COUNT+3, 2*MAX_LANDMARK_COUNT+3, (short*)Si);
  Matrix.Multiply((short*)Sigma, (short*)Ht, 2*MAX_LANDMARK_COUNT+3, 2*MAX_LANDMARK_COUNT+3, 2*MAX_LANDMARK_COUNT, (short*)temp_matrix);
  Matrix.Multiply((short*)temp_matrix, (short*)Si, 2*MAX_LANDMARK_COUNT+3, 2*MAX_LANDMARK_COUNT+3, 2*MAX_LANDMARK_COUNT, (short*)K);
  
  Matrix.Multiply((short*)temp_matrix, (short*)Ht, 2*MAX_LANDMARK_COUNT+3, 2*MAX_LANDMARK_COUNT+3, 2*MAX_LANDMARK_COUNT, (short*)HQ);
  Matrix.Subtract((short*)Z, (short*)ExpectedZ, 1, 2*MAX_LANDMARK_COUNT+3, (short*)DeltaZ);
  Matrix.Transpose((short*)DeltaZ, 1, 2*MAX_LANDMARK_COUNT+3, (short*)DeltaZt);
  Matrix.Multiply((short*)K, (short*)DeltaZt, 2*MAX_LANDMARK_COUNT+3, 1, 2*MAX_LANDMARK_COUNT+3, (short*)temp_matrix);
  K[0][0] = K[0][0] + robotXPos;
  K[1][1] = K[1][1] + robotYPos;
  K[2][2] = K[2][2] + robotTheta;
  Matrix.Multiply((short*)temp_matrix, (short*)H, 2*MAX_LANDMARK_COUNT+3, 2*MAX_LANDMARK_COUNT+3, 2*MAX_LANDMARK_COUNT+3, (short*)K);
  Matrix.Multiply((short*)K, (short*)Sigma, 2*MAX_LANDMARK_COUNT+3, 2*MAX_LANDMARK_COUNT+3, 2*MAX_LANDMARK_COUNT+3, (short*)temp_matrix);
  Matrix.Subtract((short*)Sigma, (short*)temp_matrix, 2*MAX_LANDMARK_COUNT+3, 2*MAX_LANDMARK_COUNT+3, (short*)K);
  for (int i = 0; i < 2*MAX_LANDMARK_COUNT+3; i++) {
    for (int j = 0; j < 2*MAX_LANDMARK_COUNT+3; j++) {
      Sigma[i][j] = K[i][j];
    }
  }
  robotXPos = Sigma[0][0];
  robotYPos = Sigma[1][1];
  robotTheta = Sigma[2][2];
  Serial.print("RobotXPos = ");
  Serial.print(robotXPos);
  Serial.print("\n");
  Serial.print("RobotYPos = ");
  Serial.print(robotYPos);
  Serial.print("\n");
  Serial.print("RobotTheta = ");
  Serial.print(robotTheta);
  Serial.print("\n");
  Serial.println();
}

void setup() {
 turnPin.attach(2);
 driver.attach(4);
 driver.write(driveZero);
 sensorArray.attach(7);
 sensorArray.write(sensorZero);
 delay(20);

 for (int i = 0; i < MAX_LANDMARK_COUNT * 2; i++) {
  covarBotRight[i][i] = 32000;
 }

 for (int i = 0; i < 3; i++) {
  for (int j = 0; j < 3; j++) {
    motionJacobian[i][j] = 0;
  }
  motionJacobian[i][i] = 1;
 }

 for (int i = 0; i < 3; i++) {
    for (int j = 0; j < (2*MAX_LANDMARK_COUNT); j++) {
      covarTopRight[i][j] = 0;
    }
  }

 firstPose = true;
 
 Serial.begin(9600);
}

// Main program loop
void loop() {
 checkDriveStatus(0);

 findFeatures();

 recordPose();

 EKF_SLAM_Prediction();
 EKF_SLAM_Correction();

 executeMotion();

 numFeatures = 0;
 currentSightRecord = 0;
 poseCount = (poseCount + 1) % 2;
 for (int i = 0; i < 3; i++) {
  for (int j = 0; j < 3; j++) {
    motionJacobian[i][j] = 0;
  }
  motionJacobian[i][i] = 1;
 }
 
 
 delay(2000);

}

// Set the drive servo
void checkDriveStatus(int newStatus) {
 if (newStatus != driveStatus) {
   if(newStatus == 1) {
    driver.write(45);
    delay(500);
    isMoving = true;
   }
   else if (newStatus == 0) {
    if (isMoving == false) {
      return;
    }
    if (driveStatus == -1) {
      driver.write(100);
      delay(500);
      driver.write(60);
      delay(500);
      driver.write(80);
      delay(500);
    }
    else {
      driver.write(100);
      delay(1000);
      driver.write(80);
      delay(500);
    }
    isMoving = false;
   }
   else if (newStatus == -1) {
    driver.write(110);
    delay(500);
    isMoving = true;
   }

   driveStatus = newStatus;
 }
 return;
}


// =================================================================
// Incremental_Line_Extraction
//==================================================================
float slope(Point e0, Point e1) {
  float m = (e1.y - e0.y)/(e1.x - e0.x);
  if ((e1.x - e0.x) == 0) {
    m = 10000.0;
  }
  else if (m < 0.0001 && m > -0.0001) {
    m = 0.0001;
  }
  return m;
}

// Returns a list of the position of the endpoints in the sightRange array
short Incremental_Line_Extraction() {
  short detectedFeatures = 0;
  int epsilon = 15;
  int i = 0;
  Point newFeature;
  Point firstInFeature = sightRecord[0];
  Point candidatePoint = sightRecord[1];
  bool candidateAdded = false;
  
  //int prevDist = (int)sqrt((pow((sightRecord[1].x - sightRecord[0].x), 2.0) + (pow((sightRecord[1].y - sightRecord[0].y), 2.0))));
  //float aveSlope = slope(sightRecord[0], sightRecord[1]);
  for(i = 1; i < currentSightRecord; i++) {
    int dist = (int)sqrt((pow((sightRecord[i].x - sightRecord[i-1].x), 2.0) + (pow((sightRecord[i].y - sightRecord[i-1].y), 2.0))));
    //Serial.println(dist);
    if (abs(dist) < epsilon && dist > 0) {
      if (candidateAdded == false) {
        newFeature.x = i-1;
        candidateAdded = true;
      }
      else {
        // idk
      }
    }
    else {
      if (candidateAdded == true) {
        // If a candidate only has 2 points then forget it
        if (((i - 1) - newFeature.x) < 2) {
          candidateAdded = false;
        }
        else {
          newFeature.y = i - 1;
          candidateAdded = false;
          featureIndexList[detectedFeatures] = newFeature;
          detectedFeatures = detectedFeatures + 1;
        }
      }
    }
  }
  if (candidateAdded == true) {
    // If a candidate only has 2 points then forget it
    if (((i - 1) - newFeature.x) < 2) {
      candidateAdded = false;
    }
    else {
      newFeature.y = i - 1;
      candidateAdded = false;
      featureIndexList[detectedFeatures] = newFeature;
      detectedFeatures = detectedFeatures + 1;
    }
  }

  return detectedFeatures;
}


// =================================================================
// IEPF Algo
// =================================================================

float perpendicularDistance(Point p, Point e0, Point e1) {
  float m = (e1.y - e0.y)/(e1.x - e0.x);
  if ((e1.x - e0.x) == 0) {
    m = 10000.0;
  }
  else if (m < 0.0001 && m > -0.0001) {
    m = 0.0001;
  }
  float b = e0.y - (m * e0.x);
  float m1 = -1/m;
  float b1 = p.y - (m1 * p.x);
  float x = (b1 - b)/(m - m1);
  float y = (m1 * x) + b1;
  float dist = sqrt((pow((p.x - x), 2.0) + (pow((p.y - y), 2.0))));
  //Serial.println(dist);
  return dist;
}

FeatureArray IEPF_Line_Extraction(short start, short finish, short IEPF_Iteration) {
  float epsilon = 20.0;
  int delta = 25;
  float dmax = 0.0;
  short index = 0;
  short i = 0;
  for (i = start + 1; i < finish; i++) {
    float d = perpendicularDistance(sightRecord[i], sightRecord[start], sightRecord[finish]);
    //printSightRecord();
    if (d > dmax) {
      index = i;
      dmax = d;
    }
  }

//  Serial.println(dmax);
//  Serial.println();

  FeatureArray tempFeatureList;
  if (dmax > epsilon && IEPF_Iteration < MAX_FEATURE_ARRAY) {
    IEPF_Iteration = IEPF_Iteration + 1;
    FeatureArray results1 = IEPF_Line_Extraction(start, index, IEPF_Iteration);
    FeatureArray results2 = IEPF_Line_Extraction(index, finish, IEPF_Iteration);
    
//    Serial.print(start);
//    Serial.print(" ");
//    Serial.print(index);
//    Serial.print(" ");
//    Serial.print(finish);
//    Serial.println();
    for (i = 0; i < results1.s; i++) {
      tempFeatureList.featureArray[i] = results1.featureArray[i];
    }
    tempFeatureList.s = results1.s;
    for (i = 0; i < results2.s; i++) {
      if (i == MAX_FEATURE_ARRAY) {
        break;
      }
      tempFeatureList.featureArray[tempFeatureList.s] = results2.featureArray[i];
      tempFeatureList.s = tempFeatureList.s + 1;
    }
  }
  else {
    tempFeatureList.featureArray[0].e0 = sightRecord[start];
    tempFeatureList.featureArray[0].e1 = sightRecord[finish];
    tempFeatureList.s = 1;
  }

  return tempFeatureList;
  
}
