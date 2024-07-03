#include <queue>
#include <vector>
#include <cmath>
#include <algorithm>

const int INF = 1e9;
const int GRID_SIZE = 16;
const int DIRECTIONS = 4;
const int MAX_ROUTE = 100;
const int MAX_FUNCTIONS = 200;

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
    int directionIndex = (static_cast<int>(dir) + 5) % 4;

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
    if (str.indexOf('T') < 0 && i > 0) {
      adjMatrix[4 * (i - 1) + j][4 * i + j] = w;
      if (maze[i - 1][j].indexOf('G') >= 0) {
        adjMatrix[4 * (i - 1) + j][4 * i + j] = -16;
      }
    }
    if (str.indexOf('B') < 0 && i < 3) {
      adjMatrix[4 * (i + 1) + j][4 * i + j] = w;
      if (maze[i + 1][j].indexOf('G') >= 0) {
        adjMatrix[4 * (i + 1) + j][4 * i + j] = -16;
      }
    }
    if (str.indexOf('L') < 0 && j > 0) {
      adjMatrix[4 * i + j][4 * i + j - 1] = w;
      if (maze[i][j - 1].indexOf('G') >= 0) {
        adjMatrix[4 * i + j][4 * i + j - 1] = -16;
      }
    }
    if (str.indexOf('R') < 0 && j < 3) {
      adjMatrix[4 * i + j][4 * i + j + 1] = w;
      if (maze[i][j + 1].indexOf('G') >= 0) {
        adjMatrix[4 * i + j][4 * i + j + 1] = -16;
      }
    }
  }

  for (int a = 0; a < 16; a++) {
    for (int b = 0; b < 16; b++) {
      Serial.print(adjMatrix[a][b]);
      Serial.print("\t");
    }
    Serial.println("");
  }
}

int hCost(int node) {
  return abs(node % 4 - endNode % 4) + abs(node / 4 - endNode / 4);
}

int improvedHCost(int node, int endNode) {
    int dx = abs(node % 4 - endNode % 4);
    int dy = abs(node / 4 - endNode / 4);
    return (dx + dy) + (141 * std::min(dx, dy)) / 100;
}

std::vector<int> bidirectionalAStar(int start, int end) {
    std::priority_queue<std::pair<int, int>, std::vector<std::pair<int, int>>, std::greater<std::pair<int, int>>> openSetForward, openSetBackward;
    std::vector<int> gScoreForward(16, INF), gScoreBackward(16, INF);
    std::vector<int> parentForward(16, -1), parentBackward(16, -1);
    std::vector<bool> closedSetForward(16, false), closedSetBackward(16, false);

    openSetForward.push({0, start});
    openSetBackward.push({0, end});
    gScoreForward[start] = 0;
    gScoreBackward[end] = 0;

    while (!openSetForward.empty() && !openSetBackward.empty()) {
        int currentForward = openSetForward.top().second;
        int currentBackward = openSetBackward.top().second;

        if (closedSetForward[currentBackward] || closedSetBackward[currentForward]) {
            return reconstructBidirectionalPath(parentForward, parentBackward, currentForward, currentBackward);
        }

        openSetForward.pop();
        openSetBackward.pop();

        closedSetForward[currentForward] = true;
        closedSetBackward[currentBackward] = true;

        for (int dir = 0; dir < DIRECTIONS; ++dir) {
            int nextForward = getNeighbor(currentForward, dir);
            if (nextForward != -1 && !closedSetForward[nextForward] && adjMatrix[currentForward][nextForward] < 0) {
                int tentativeGScore = gScoreForward[currentForward] + (-adjMatrix[currentForward][nextForward]);
                if (tentativeGScore < gScoreForward[nextForward]) {
                    parentForward[nextForward] = currentForward;
                    gScoreForward[nextForward] = tentativeGScore;
                    int fScore = tentativeGScore + improvedHCost(nextForward, end);
                    openSetForward.push({fScore, nextForward});
                }
            }
        }

        for (int dir = 0; dir < DIRECTIONS; ++dir) {
            int nextBackward = getNeighbor(currentBackward, dir);
            if (nextBackward != -1 && !closedSetBackward[nextBackward] && adjMatrix[nextBackward][currentBackward] < 0) {
                int tentativeGScore = gScoreBackward[currentBackward] + (-adjMatrix[nextBackward][currentBackward]);
                if (tentativeGScore < gScoreBackward[nextBackward]) {
                    parentBackward[nextBackward] = currentBackward;
                    gScoreBackward[nextBackward] = tentativeGScore;
                    int fScore = tentativeGScore + improvedHCost(nextBackward, start);
                    openSetBackward.push({fScore, nextBackward});
                }
            }
        }
    }

    return {};
}

std::vector<int> reconstructBidirectionalPath(const std::vector<int>& parentForward, const std::vector<int>& parentBackward, int meetingPoint, int end) {
    std::vector<int> path;
    for (int node = meetingPoint; node != -1; node = parentForward[node]) {
        path.push_back(node);
    }
    std::reverse(path.begin(), path.end());
    for (int node = parentBackward[meetingPoint]; node != -1; node = parentBackward[node]) {
        path.push_back(node);
    }
    return path;
}

int getNeighbor(int node, int direction) {
    switch (direction) {
        case 0: return (node > 3) ? node - 4 : -1;
        case 1: return (node < 12) ? node + 4 : -1;
        case 2: return (node % 4 < 3) ? node + 1 : -1;
        case 3: return (node % 4 > 0) ? node - 1 : -1;
        default: return -1;
    }
}

void improvedPath() {
    for (int a = 1; a < gateCount + 2; a++) {
        endNode = endPoints[a];
        startNode = endPoints[a - 1];

        std::vector<int> path = bidirectionalAStar(startNode, endNode);

        if (!path.empty()) {
            Serial.print("Path: ");
            for (int i = 0; i < path.size() - 1; ++i) {
                logMovement(path[i], path[i + 1]);
                Direction moveDirection = static_cast<Direction>(path[i + 1] - path[i]);
                handleMovement(moveDirection);
            }
            if (endNode == endPoints[gateCount + 1]) {
                addMovement(go_backward, 13);
            }
        } else {
            Serial.println("No path found!");
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
  mpu6050.update();
  fAngle = mpu6050.getAngleZ();
  init_angle = fAngle;

  createMatrix();
  emptyFunction();
  Serial.println("Starting improved pathfinding");
  improvedPath();

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
    } else {
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