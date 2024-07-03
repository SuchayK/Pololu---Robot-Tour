
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

  }
}