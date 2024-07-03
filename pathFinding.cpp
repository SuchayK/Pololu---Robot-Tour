#include <queue>
#include <vector>
#include <cmath>

const int INF = 1e9;
const int GRID_SIZE = 16;
const int DIRECTIONS = 4;

int functionCount = 1;
void (*functionArray[50])();  
int distance[50];         
int prevNode = -1;

void map() {  

  functionArray[0] = go_fwd;
  distance[0] = 11.5+25;
  for (int i = 1; i < 50; i++) {
    functionArray[i] = stop_Stop;
    distance[i] = 45;
  }

}

enum class Orientation { North, East, South, West };
enum class Direction { Left = -1, Right = 1, Up = -4, Down = 4 };

const int MAX_ROUTE = 100;
const int MAX_FUNCTIONS = 200;

std::array<int, MAX_ROUTE> route;
int routeCount = 0;

std::array<std::function<void()>, MAX_FUNCTIONS> movementFunctions;
std::array<int, MAX_FUNCTIONS> distanceArray;
int functionIndex = 0;

Orientation currentOrientation = Orientation::North;

void turn_left();
void turn_right();
void go_forward();
void go_backward();
void full_turn();

void logMovement(int fromNode, int toNode) {
    Serial.print(fromNode);
    Serial.print(" --> ");
    Serial.println(toNode);
    Serial.println(toNode - fromNode);
}

void updateOrientation(Direction dir) {
    int orientationChange = static_cast<int>(dir);
    currentOrientation = static_cast<Orientation>((static_cast<int>(currentOrientation) + orientationChange + 4) % 4);
}

void addMovement(std::function<void()> movement, int distanceAdd = 0) {
    if (distanceAdd > 0 && functionIndex > 0) {
        distanceArray[functionIndex - 1] += distanceAdd;
    } else {
        movementFunctions[functionIndex] = movement;
        distanceArray[functionIndex] = distanceAdd;
        functionIndex++;
    }
}

void handleMovement(Direction dir) {
    const int distanceIncrement = 45;
    
    std::array<std::array<std::function<void()>, 4>, 4> movementMatrix = {{
        {go_forward, turn_right, full_turn, turn_left},
        {turn_left, go_forward, turn_right, full_turn},
        {full_turn, turn_left, go_forward, turn_right},
        {turn_right, full_turn, turn_left, go_forward}
    }};

    int orientationIndex = static_cast<int>(currentOrientation);
    int directionIndex = (static_cast<int>(dir) + 5) % 4;  // Map -4, -1, 1, 4 to 0, 1, 2, 3

    if (movementMatrix[orientationIndex][directionIndex] == go_forward) {
        addMovement(nullptr, distanceIncrement);
    } else {
        addMovement(movementMatrix[orientationIndex][directionIndex]);
        addMovement(go_forward);
    }

    updateOrientation(dir);
}

void reconstructPath(const int parent[], int node, const std::array<int, MAX_ROUTE>& endPoints, int gateCount) {
    if (parent[node] == -1) {
        Serial.println(node);
        route[routeCount++] = node;
        return;
    }

    reconstructPath(parent, parent[node], endPoints, gateCount);
    logMovement(parent[node], node);

    Direction moveDirection = static_cast<Direction>(node - parent[node]);
    handleMovement(moveDirection);

    if (node == endPoints[gateCount + 1]) {
        addMovement(go_backward, 13);
    }
}


bool adjMatrix[16][16];

void createMatrix() {

  for (int a = 0; a < 16; a++) {

    for (int b = 0; b < 16; b++) {

      adjMatrix[a][b] = 1;

    }

  }

  for (int a = 0; a < 16; a++) {

    int i = a / 4;
    int j = a % 4;
    String str = maze[i][j];
    int w = 0;       

    if (str.indexOf('G') >= 0) {  

      w = -16;            

    }
    if (str.indexOf('T') < 0 && i > 0) {  //IF THERE IS NO TOP BARRIER && IS NOT IN THE TOP ROW

      adjMatrix[4 * (i - 1) + j][4 * i + j] = w;

      if (maze[i - 1][j].indexOf('G') >= 0) {  //CHECK IF IT LEADS TO A GATE ZONE

        adjMatrix[4 * (i - 1) + j][4 * i + j] = -16;

      }

    }
    if (str.indexOf('B') < 0 && i < 3) {  // IF THERE IS NO BOTTOM BARRIER && IS NOT IN THE BOTTOM ROW

      adjMatrix[4 * (i + 1) + j][4 * i + j] = w;

      if (maze[i + 1][j].indexOf('G') >= 0) {  //CHECK IF IT LEADS TO A GATE ZONE

        adjMatrix[4 * (i + 1) + j][4 * i + j] = -16;

      }

    }
    if (str.indexOf('L') < 0 && j > 0) {  //IF THERE IS NO LEFT BARRIER && IS NOT IN THE LEFT-MOST COLUMN

      adjMatrix[4 * i + j][4 * i + j - 1] = w;

      if (maze[i][j - 1].indexOf('G') >= 0) {  //CHECK IF IT LEADS TO A GATE ZONE

        adjMatrix[4 * i + j][4 * i + j - 1] = -16;

      }

    }

    if (str.indexOf('R') < 0 && j < 3) {  // IF THERE IS NO RIGHT BARRIER && IS NOT IN THE RIGHT MOST COLUMN

      adjMatrix[4 * i + j][4 * i + j + 1] = w;

      if (maze[i][j + 1].indexOf('G') >= 0) {  //CHECK IF IT LEADS TO A GATE ZONE

        adjMatrix[4 * i + j][4 * i + j + 1] = -16;

      }

    }

  }

    for (int a = 0; a < 16; a++) {

        for (int b = 0; b < 16; b++) {

            Serial.print(adjMatrix[a][b]);
            Serial.print("\t");

        }

    Serial.println("");//DEBUG CHECKING
    }

}


int hCost(int node) {

  return abs(node % 4 - endNode % 4) + abs(node / 4 - endNode / 4);

}

void Path() {

  for (int a = 1; a < gateCount + 2; a++) {

    endNode = endPoints[a];
    startNode = endPoints[a - 1];
    int parent[16];  

    for (int i = 0; i < 16; i++) {

      parent[i] = -1;

    }

    int currentNode = startNode;
    bool openSet[16] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
    bool closedSet[16] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
    int fCostList[16] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

    openSet[startNode] = 0;
    route[routeCount-2] = startNode;
    prevNode = startNode;

    while (currentNode != endNode) {

      int minCost = INF;

      for (int i = 0; i < 16; i++) { 

        if (openSet[i] == true) {

          if (fCostList[i] <= minCost) {

            minCost = fCostList[i];
            currentNode = i;
            Serial.println(minCost);

          }

        }

      }

      openSet[currentNode] = 0;  
      closedSet[currentNode] = 1; 

      if (currentNode == endNode) {

        break;

      }

      if (currentNode > 3) {      
                                                                                         
        if (!(closedSet[currentNode - 4]) && adjMatrix[currentNode][currentNode - 4] < 0) {       
 
          if (adjMatrix[currentNode][currentNode - 4] < fCostList[currentNode - 4] || !openSet[currentNode - 4]) { 

            fCostList[currentNode - 4] = adjMatrix[currentNode][currentNode - 4] + hCost(currentNode - 4);
            parent[currentNode - 4] = currentNode;

            if (!openSet[currentNode - 4]) {

              openSet[currentNode - 4] = true;

            }

          }

        }

      }
      
      if (currentNode < 10) { 
                                                                                        
        if (!(closedSet[currentNode + 4]) && adjMatrix[currentNode][currentNode + 4] < 0) {       

          if (adjMatrix[currentNode][currentNode + 4] < fCostList[currentNode + 4] || !openSet[currentNode + 4]) { 

            fCostList[currentNode + 4] = adjMatrix[currentNode][currentNode + 4] + hCost(currentNode + 4);
            parent[currentNode + 4] = currentNode;

            if (!openSet[currentNode + 4]) {

              openSet[currentNode + 4] = true;

            }

          }

        }

      }

      if (currentNode % 4 < 3) {     
                                                                                   
        if (!(closedSet[currentNode + 1]) && adjMatrix[currentNode][currentNode + 1] < 0) {                         

          if (adjMatrix[currentNode][currentNode + 1] < fCostList[currentNode + 1] || !openSet[currentNode + 1]) { 

            fCostList[currentNode + 1] = adjMatrix[currentNode][currentNode + 1] + hCost(currentNode + 1);
            parent[currentNode + 1] = currentNode;

            if (!openSet[currentNode + 1]) {

              openSet[currentNode + 1] = true;

            }

          }

        }

      }

      if (currentNode % 4 > 0) {                
                                                                    
        if (!(closedSet[currentNode - 1]) && adjMatrix[currentNode][currentNode - 1] < 0) {           
         
          if (adjMatrix[currentNode][currentNode - 1] < fCostList[currentNode - 1] || !openSet[currentNode - 1]) { 
            
            fCostList[currentNode - 1] = adjMatrix[currentNode][currentNode - 1] + hCost(currentNode - 1);
            parent[currentNode - 1] = currentNode;

            if (!openSet[currentNode - 1]) {

              openSet[currentNode - 1] = true;

            }

          }

        }

      }

    }

    if (currentNode == endNode) {

      Serial.print("Path: ");
      printPath(parent, endNode);

    }

  }

}

void setup() {

  init_GPIO();
  Serial.begin(115200);
  delay(500);
  attachInterrupt(digitalPinToInterrupt(encoder1), count, RISING);
  attachInterrupt(digitalPinToInterrupt(encoder2), count2, RISING);
  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
  //SETTING THE INITIAL ANGLE
  mpu6050.update();
  fAngle = mpu6050.getAngleZ();
  init_angle = fAngle;

  createMatrix();
  emptyFunction();
  Serial.println("got here");
  Path();

  for(int i = 0; i < 50; i++){

    currentDistance = distance[i];
    functionArray[i]();

    if (functionArray[i] == stop_Stop){

      break;

    }

    Serial.println("");

  }

  for (int i = 0 ; i < 50; i++){

    currentDistance = distance[i];

      if (functionArray[i] == go_fwd) {

      Serial.print("Going fwd ");

    } else if (functionArray[i] == turn_right) {

      Serial.print("Turning right ");

    } else if (functionArray[i] == turn_left) {

      Serial.print("Turning left ");

    } else if (functionArray[i] == fullTurn) {

      Serial.print("Full Turn");

    } else if (functionArray[i] == stop_Stop){

      Serial.print("Stop");

    }e lse{

      Serial.print("Else");

    }

    Serial.println(currentDistance);

  }

    Serial.println("finished");

  delay(5000);

}

void loop() {

  mpu6050.update();
  angle = mpu6050.getAngleZ();
  mpu6050.update();
  angle = mpu6050.getAngleZ();
  currentDistance = distance[step];
  functionArray[step]();

}