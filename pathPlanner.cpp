#include <iostream>
#include <vector>
#include <array>
#include <queue>
#include <algorithm>
#include <cmath>
#include <functional>

const int GRID_DIM = 4;
const int TOTAL_CELLS = GRID_DIM * GRID_DIM;
const int MOVE_DIRECTIONS = 4;
const int INFINITY = 1000000000;

enum class Move { L = -1, R = 1, U = -GRID_DIM, D = GRID_DIM };

struct GridCell {

    bool walls[MOVE_DIRECTIONS] = {false};
    bool isGate = false;

};

class PathPlanner {

private:

    std::array<GridCell, TOTAL_CELLS> maze;
    std::vector<int> checkpoints;

    int calculateHeuristic(int current, int goal) {

        int dx = std::abs((current % GRID_DIM) - (goal % GRID_DIM));
        int dy = std::abs((current / GRID_DIM) - (goal / GRID_DIM));
        return dx + dy + (std::min(dx, dy) * 41 / 100);

    }

    int getAdjacentCell(int cell, int direction) {

        switch (direction) {

            case 0: return (cell >= GRID_DIM) ? cell - GRID_DIM : -1;
            case 1: return (cell % GRID_DIM < GRID_DIM - 1) ? cell + 1 : -1;
            case 2: return (cell < TOTAL_CELLS - GRID_DIM) ? cell + GRID_DIM : -1;
            case 3: return (cell % GRID_DIM > 0) ? cell - 1 : -1;
            default: return -1;

        }

    }

    std::vector<int> reconstructPath(const std::vector<int>& cameFrom, int current) {

        std::vector<int> path;

        while (current != -1) {

            path.push_back(current);
            current = cameFrom[current];

        }

        std::reverse(path.begin(), path.end());
        return path;

    }

    std::vector<int> findOptimalPath(int start, int end) {

        std::priority_queue<std::pair<int, int>, std::vector<std::pair<int, int>>, std::greater<>> openSet;
        std::vector<int> cameFrom(TOTAL_CELLS, -1);
        std::vector<int> gScore(TOTAL_CELLS, INFINITY);
        std::vector<int> fScore(TOTAL_CELLS, INFINITY);

        openSet.push({0, start});
        gScore[start] = 0;
        fScore[start] = calculateHeuristic(start, end);

        while (!openSet.empty()) {

            int current = openSet.top().second;
            openSet.pop();

            if (current == end) {

                return reconstructPath(cameFrom, current);

            }

            for (int dir = 0; dir < MOVE_DIRECTIONS; ++dir) {

                int neighbor = getAdjacentCell(current, dir);

                if (neighbor == -1 || maze[current].walls[dir]) continue;

                int tentativeGScore = gScore[current] + 1;

                if (tentativeGScore < gScore[neighbor]) {

                    cameFrom[neighbor] = current;
                    gScore[neighbor] = tentativeGScore;
                    fScore[neighbor] = gScore[neighbor] + calculateHeuristic(neighbor, end);
                    openSet.push({fScore[neighbor], neighbor});

                }

            }

        }

        return {};

    }

public:

    void initializeMaze() {

        for (int i = 0; i < TOTAL_CELLS; ++i) {

            maze[i].walls[0] = (i < GRID_DIM);
            maze[i].walls[1] = (i % GRID_DIM == GRID_DIM - 1);
            maze[i].walls[2] = (i >= TOTAL_CELLS - GRID_DIM);
            maze[i].walls[3] = (i % GRID_DIM == 0);

        }

        maze[5].isGate = true;
        maze[10].isGate = true;

        checkpoints = {0, 5, 10, 15};

    }

    void planAndPrintPath() {

        std::cout << "Path planning started\n";

        for (size_t i = 1; i < checkpoints.size(); ++i) {

            int start = checkpoints[i - 1];
            int end = checkpoints[i];
            std::vector<int> path = findOptimalPath(start, end);
            
            if (!path.empty()) {

                std::cout << "Path from " << start << " to " << end << ":\n";

                for (size_t j = 1; j < path.size(); ++j) {
                    
                    int from = path[j - 1];
                    int to = path[j];
                    Move moveDirection = static_cast<Move>(to - from);
                    
                    std::cout << from << " -> " << to << " : ";
                    switch (moveDirection) {

                        case Move::L: std::cout << "Move Left\n"; break;

                        case Move::R: std::cout << "Move Right\n"; break;

                        case Move::U: std::cout << "Move Up\n"; break;

                        case Move::D: std::cout << "Move Down\n"; break;

                    }

                }

                if (end == checkpoints.back()) {

                    std::cout << "Reverse at the end point\n";

                }

                std::cout << "\n";

            } else {

                std::cout << "No path found from " << start << " to " << end << "\n";

            }
        }

    }

    void PathPlanner::executeSmoothTurn(int from, int to) {

        Point start = {static_cast<double>(from % GRID_DIM), static_cast<double>(from / GRID_DIM)};
        Point end = {static_cast<double>(to % GRID_DIM), static_cast<double>(to / GRID_DIM)};
        
        Point control1 = {(start.x + end.x) / 2, start.y};
        Point control2 = {(start.x + end.x) / 2, end.y};

        RobotController controller;
        controller.performSmoothTurn(start, end, control1, control2);

    }


}
