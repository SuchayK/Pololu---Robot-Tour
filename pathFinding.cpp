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

int orientation = 0;
void printPath(int parent[], int node) {

  if (parent[node] == -1) {
    Serial.println(node);
    route[routeCount] = node;
    return;
  }

  printPath(parent, parent[node]);

  Serial.print(" --> ");
  Serial.println(node);

  Serial.println(node - prevNode);

  int distanceAdditive = 45;

  switch (node - prevNode) { 

    case -1: //left

      switch(orientation){

        case 0:

          functionArray[functionCount] = turn_left;
          functionArray[functionCount+1] = go_fwd;
          functionCount++;
          break;

        case 1:

          functionArray[functionCount] = fullTurn;
          functionArray[functionCount+1] = go_fwd;
          functionCount++;
          break;

        case 2:

          functionArray[functionCount] = turn_right;
          functionArray[functionCount+1] = go_fwd; 
          functionCount++;  
          break;

        case 3:

          functionArray[functionCount] = ;     
          distance[functionCount-1] += distanceAdditive;      
          functionCount--;
          break;

        default:

          Serial.println(node-prevNode);
          break;

      }

      orientation = 3; //left
      break;

    case 1: //right

      switch(orientation){

        case 0:

          functionArray[functionCount] = turn_right;
          functionArray[functionCount+1] = go_fwd;
          functionCount++;
          break;

        case 1:

          functionArray[functionCount] = ;     
          distance[functionCount-1] += distanceAdditive;    
          functionCount--;
          break;

        case 2:

          functionArray[functionCount] = turn_left;
          functionArray[functionCount+1] = go_fwd;  
          functionCount++; 
          break;

        case 3:

          functionArray[functionCount] = fullTurn;
          functionArray[functionCount+1] = go_fwd;
          functionCount++;
          break;

        default:

          Serial.println(node-prevNode);
          break;
      }
      orientation = 1; //right
      break;

    case -4://up
      switch(orientation){
        case 0:
          distance[functionCount-1] += distanceAdditive;    
          functionCount--;
          break;
        case 1:
          functionArray[functionCount] = turn_left;
          functionArray[functionCount+1] = go_fwd;
          functionCount++;
          break;
        case 2:
          functionArray[functionCount] = fullTurn;
          functionArray[functionCount+1] = go_fwd;
          functionCount++;
          break;
        case 3:
          functionArray[functionCount] = turn_right;
          functionArray[functionCount+1] = go_fwd;
          functionCount++;
          break;
        default:
          Serial.println(node-prevNode);
          break;
      }
      orientation = 0;
      break;
    case 4:
      switch(orientation){
        case 0:
          functionArray[functionCount] = fullTurn;
          functionArray[functionCount+1] = go_fwd;
          functionCount++;
          break;
        case 1:
          functionArray[functionCount] = turn_right;
          functionArray[functionCount+1] = go_fwd;
          functionCount++;
          break;
        case 2:
          distance[functionCount-1] += distanceAdditive;  
          functionCount--;
          break;
        case 3:
          functionArray[functionCount] = turn_left;
          functionArray[functionCount+1] = go_fwd;
          functionCount++;
          break;
        default:
          Serial.println(node-prevNode);
          break;        
      }
      orientation = 2;
      break;
    default:
      Serial.println(node-prevNode);
      break;
  }
  if (node == endPoints[gateCount + 1]){ /
    functionArray[functionCount+1] = go_bck;
    functionCount++;
    distance[functionCount] = 13;
  }
  prevNode = node;
  functionCount++; 
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

struct Node {

  int id, fCost;
  Node(int i, int f) : id(i), fCost(f) {}
  bool operator>(const Node& other) const { return fCost > other.fCost; }

};

int dx[] = {-1, 1, 0, 0};
int dy[] = {0, 0, 1, -1};

int heuristic(int current, int end) {

  int cx = current / 4, cy = current % 4;
  int ex = end / 4, ey = end % 4;
  return std::abs(cx - ex) + std::abs(cy - ey);

}

void reconstructPath(const std::vector<int>& cameFrom, int current) {

  std::vector<int> path;

  while (current != -1) {

    path.push_back(current);
    current = cameFrom[current];

  }

  for (auto it = path.rbegin(); it != path.rend(); ++it) {

    std::cout << *it << " ";

  }

  std::cout << std::endl;

}

void findPath(const std::vector<std::vector<int>>& adjMatrix, const std::vector<int>& endPoints) {
  for (int i = 1; i < endPoints.size(); ++i) {

    auto [startNode, endNode] = std::make_pair(endPoints[i - 1], endPoints[i]);
        
    std::unordered_set<int> explored;
    std::vector<int> distances(GRID_SIZE, INF);
    std::vector<int> estimatedTotalCost(GRID_SIZE, INF);
    std::vector<int> predecessor(GRID_SIZE, -1);
    
    auto cmp = [](const Node& a, const Node& b) { return a > b; };
    std::priority_queue<Node, std::vector<Node>, decltype(cmp)> frontier(cmp);

    distances[startNode] = 0;
    estimatedTotalCost[startNode] = heuristic(startNode, endNode);
    frontier.emplace(startNode, estimatedTotalCost[startNode]);
    
    while (!frontier.empty()) {

      int currentNode = frontier.top().id;
      frontier.pop();
      
      if (currentNode == endNode) {

        reconstructPath(predecessor, endNode);
        break;

      }
      
      if (explored.count(currentNode)) continue;

      explored.insert(currentNode);
      
      for (int dir = 0; dir < DIRECTIONS; ++dir) {

        int newX = currentNode / 4 + dx[dir];
        int newY = currentNode % 4 + dy[dir];
        int neighborNode = newX * 4 + newY;
        
        if (newX < 0 || newX >= 4 || newY < 0 || newY >= 4 || explored.count(neighborNode) || adjMatrix[currentNode][neighborNode] >= 0) {

          continue;

        }
        
        int newDistance = distances[currentNode] + std::abs(adjMatrix[currentNode][neighborNode]);
        
        if (newDistance < distances[neighborNode]) {

          predecessor[neighborNode] = currentNode;
          distances[neighborNode] = newDistance;
          estimatedTotalCost[neighborNode] = distances[neighborNode] + heuristic(neighborNode, endNode);
          frontier.emplace(neighborNode, estimatedTotalCost[neighborNode]);

        }
      }

    }


    // int start = endPoints[i - 1];
    // int end = endPoints[i];
    
    // std::vector<bool> closedSet(GRID_SIZE, false);
    // std::vector<int> gScore(GRID_SIZE, INF);
    // std::vector<int> fScore(GRID_SIZE, INF);
    // std::vector<int> cameFrom(GRID_SIZE, -1);
    
    // std::priority_queue<Node, std::vector<Node>, std::greater<Node>> openSet;
    
    // gScore[start] = 0;
    // fScore[start] = heuristic(start, end);
    // openSet.push(Node(start, fScore[start]));
    
    // while (!openSet.empty()) {
    //   int current = openSet.top().id;
    //   openSet.pop();
      
    //   if (current == end) {

    //     reconstructPath(cameFrom, end);
    //     break;

    //   }

    // }

    // closedSet[current] = true;
            
    // for (int dir = 0; dir < DIRECTIONS; ++dir) {

    //   int nx = current / 4 + dx[dir];
    //   int ny = current % 4 + dy[dir];
    //   int neighbor = nx * 4 + ny;
      
    //   if (nx < 0 || nx >= 4 || ny < 0 || ny >= 4 || closedSet[neighbor] || adjMatrix[current][neighbor] >= 0) {

    //     continue;

    //   }
      
    //   int tentativeGScore = gScore[current] + std::abs(adjMatrix[current][neighbor]);
      
    //   if (tentativeGScore < gScore[neighbor]) {

    //     cameFrom[neighbor] = current;
    //     gScore[neighbor] = tentativeGScore;
    //     fScore[neighbor] = gScore[neighbor] + heuristic(neighbor, end);
    //     openSet.push(Node(neighbor, fScore[neighbor]));

    //   }

    // }

  }

}