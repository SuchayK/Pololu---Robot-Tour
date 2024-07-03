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
