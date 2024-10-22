// Shahzeb Umer     21i-0893
// Mohsin Raamzan   20i-2354
// Muhammad Sharjeel Nadir  21i-2699


#include <iostream>
#include <ctime>
//#include <iostream>
//#include <bits/stdc++.h>
#include <unordered_map>
#include <unordered_set>
#include <queue>
#include <vector>
using namespace std;


class Node
{
public:
    int id;
    int reward_score;
    int count;
    int height;
    Node* left;
    Node* right;

    Node() : id(0), reward_score(0), count(0), height(0), left(nullptr), right(nullptr) {}
};

class AVLTree
{
private:
    Node* root;
public:
    AVLTree() : root(new Node()) {
        srand(time(0));
    }
    void updateNode(int newId, int newRewardScore) {
        if (root != nullptr) {
            root->id = newId;
            root->reward_score = newRewardScore;
        }
    }
    int getRewardScore(int id) {
        Node* current = root;
        while (current != nullptr) {
            if (id == current->id) {
                return current->reward_score;
            }
            else if (id < current->id) {
                current = current->left;
            }
            else {
                current = current->right;
            }
        }
        cout << " id " << id << " not found in the tree." << endl;
        return -1;
    }


    int height(Node* node) {
        return node ? node->height : 0;
    }

    int balanceFactor(Node* node) {
        return height(node->right) - height(node->left);
    }

    void updateHeight(Node* node) {
        node->height = 1 + std::max(height(node->left), height(node->right));
    }

    Node* rotateRight(Node* y) {
        Node* x = y->left;
        y->left = x->right;
        x->right = y;
        updateHeight(y);
        updateHeight(x);
        return x;
    }

    Node* rotateLeft(Node* x) {
        Node* y = x->right;
        x->right = y->left;
        y->left = x;
        updateHeight(x);
        updateHeight(y);
        return y;
    }

    Node* balance(Node* node) {
        updateHeight(node);
        if (balanceFactor(node) == 2) {
            if (balanceFactor(node->right) < 0) {
                node->right = rotateRight(node->right);
            }
            return rotateLeft(node);
        }
        if (balanceFactor(node) == -2) {
            if (balanceFactor(node->left) > 0) {
                node->left = rotateLeft(node->left);
            }
            return rotateRight(node);
        }
        return node;
    }

    Node* insert(Node* node, int id, int reward_score) {
        if (!node) {
            return new Node();
        }

        if (id == node->id) {
            node->count++;
            node->reward_score += reward_score;
        }
        else if (id < node->id) {
            node->left = insert(node->left, id, reward_score);
        }
        else {
            node->right = insert(node->right, id, reward_score);
        }

        return balance(node);
    }

    Node* findMin(Node* node) {
        return node->left ? findMin(node->left) : node;
    }

    Node* removeMin(Node* node) {
        if (!node->left) {
            return node->right;
        }
        node->left = removeMin(node->left);
        return balance(node);
    }

    Node* remove(Node* node, int id) {
        if (!node) {
            return nullptr;
        }

        if (id < node->id) {
            node->left = remove(node->left, id);
        }
        else if (id > node->id) {
            node->right = remove(node->right, id);
        }
        else {
            if (!node->right) {
                return node->left;
            }

            Node* min = findMin(node->right);
            min->right = removeMin(node->right);
            min->left = node->left;
            delete node;
            return balance(min);
        }

        return balance(node);
    }
};


enum NodeType {
    C, // Clear path
    J, // Jewel
    P, // Potion
    W, // Weapon
    D, // Death point
    O, // Obstacle
    DR, // Dragon
    G, // Goblin
    WE, // Werewolf
    X // Crystal (goal point)
};

class GameMap {
private:
    unordered_map<int, unordered_set<int>> adjacencyList;
    int currentRow, currentCol;
    vector<int> getNeighbors(int row, int col) {
        vector<int> neighbors;
        vector<pair<int, int>> offsets = { {-1, 0}, {1, 0}, {0, -1}, {0, 1} };

        for (const auto& offset : offsets) {
            int newRow = row + offset.first;
            int newCol = col + offset.second;

            if (newRow >= 0 && newRow < rows && newCol >= 0 && newCol < cols) {
                neighbors.push_back(newRow * cols + newCol);
            }
        }

        return neighbors;
    }
    string getNodeTypeName(NodeType nodeType) {
        for (auto it = nodeTypeMap.begin(); it != nodeTypeMap.end(); ++it) {
            if (it->second == nodeType) {
                return string(1, it->first);
            }
        }
        return "Unknown";
    }

public:
    unordered_map<char, NodeType> nodeTypeMap = {
        {'C', C}, {'J', J}, {'P', P}, {'W', W}, {'%', D},
        {'#', O}, {'&', DR}, {'$', G}, {'@', WE}, {'*', X}
    };

    unordered_map<int, NodeType> nodeTypes;

    int rows, cols;

    GameMap(int rows, int cols) : rows(rows), cols(cols) {}

    void addNode(int position, char nodeTypeChar) {
        NodeType nodeType = nodeTypeMap[nodeTypeChar];
        nodeTypes[position] = nodeType;
    }
    void connectAdjacentNodes() {
        for (int i = 0; i < rows; ++i) {
            for (int j = 0; j < cols; ++j) {
                int position = i * cols + j;
                vector<int> neighbors = getNeighbors(i, j);

                for (int neighbor : neighbors) {
                    adjacencyList[position].insert(neighbor);
                }
            }
        }
    }

    void displayAdjacencyList() {
        for (const auto& entry : adjacencyList) {
            cout << "Node " << entry.first << ": ";
            for (const int neighbor : entry.second) {
                cout << neighbor << " ";
            }
            cout << endl;
        }
    }
    void move() {
        char direction;
        cout << "Enter the direction to move (W: Up, A: Left, S: Down, D: Right): ";
        cin >> direction;

        int newRow = currentRow;
        int newCol = currentCol;

        switch (direction) {
        case 'W':
        case 'w':
            newRow -= 1;
            break;
        case 'A':
        case 'a':
            newCol -= 1;
            break;
        case 'S':
        case 's':
            newRow += 1;
            break;
        case 'D':
        case 'd':
            newCol += 1;
            break;
        default:
            cout << "Invalid direction!" << endl;
            return;
        }

        if (newRow >= 0 && newRow < rows && newCol >= 0 && newCol < cols) {
            int newPosition = newRow * cols + newCol;
            int currentPosition = currentRow * cols + currentCol;
            if (adjacencyList[currentPosition].count(newPosition) == 1) {
                currentRow = newRow;
                currentCol = newCol;
                cout << "Moved to position (" << currentRow << ", " << currentCol << ")." << endl;
            }
            else {
                cout << "Cannot move in the given direction!" << endl;
            }
        }
        else {
            cout << "Cannot move outside the game map!" << endl;
        }
    }
    void setCurrentPosition(int row, int col) {
        if (row >= 0 && row < rows && col >= 0 && col < cols) {
            currentRow = row;
            currentCol = col;
            cout << "Character position set to (" << currentRow << ", " << currentCol << ")." << endl;
        }
        else {
            cout << "Invalid position!" << endl;
        }
    }
    NodeType getCurrentNodeValue() {
        int currentPosition = currentRow * cols + currentCol;
        if (nodeTypes.find(currentPosition) != nodeTypes.end()) {
            return nodeTypes[currentPosition];
        }
        //return ; // If the position is not in nodeTypes, return the Clear path NodeType as default.
    }

    void floydWarshall(vector<vector<int>>& distanceMatrix) {
        int totalNodes = rows * cols;

        for (int k = 0; k < totalNodes; ++k) {
            for (int i = 0; i < totalNodes; ++i) {
                for (int j = 0; j < totalNodes; ++j) {
                    if (distanceMatrix[i][k] != INT_MAX && distanceMatrix[k][j] != INT_MAX) {
                        distanceMatrix[i][j] = min(distanceMatrix[i][j], distanceMatrix[i][k] + distanceMatrix[k][j]);
                    }
                }
            }
        }
    }
    vector<vector<int>> initializeDistanceMatrix() {
        int totalNodes = rows * cols;
        vector<vector<int>> distanceMatrix(totalNodes, vector<int>(totalNodes, INT_MAX));

        for (int i = 0; i < totalNodes; ++i) {
            distanceMatrix[i][i] = 0;
            for (int neighbor : adjacencyList[i]) {
                if (nodeTypes[neighbor] != O && nodeTypes[neighbor] != D && nodeTypes[neighbor] != WE &&
                    nodeTypes[neighbor] != DR && nodeTypes[neighbor] != G) {
                    distanceMatrix[i][neighbor] = 1;
                }
            }
        }

        return distanceMatrix;
    }

    void displayShortestPathsFloydWarshall() {
        int startPosition = currentRow * cols + currentCol;
        vector<vector<int>> distanceMatrix = initializeDistanceMatrix();
        floydWarshall(distanceMatrix);

        int targetPosition = findCrystalPosition();
        if (targetPosition == -1) {
            cout << "No crystal found on the map!" << endl;
            return;
        }

        cout << "Shortest path to the crystal from the current position (" << startPosition / cols << ", " << startPosition % cols << ") (Floyd-Warshall Algorithm):" << endl;

        if (distanceMatrix[startPosition][targetPosition] == INT_MAX) {
            cout << "No valid path found!" << endl;
        }
        else {
            cout << "Shortest path length: " << distanceMatrix[startPosition][targetPosition] << endl;
        }
    }

    void displayShortestPathsFromInitialPositionFloydWarshall() {
        int initialPosition = 0;
        vector<vector<int>> distanceMatrix = initializeDistanceMatrix();
        floydWarshall(distanceMatrix);

        cout << "Shortest paths from the initial position (0, 0) (Floyd-Warshall Algorithm):" << endl;
        for (int j = 0; j < rows * cols; ++j) {
            if (distanceMatrix[initialPosition][j] == INT_MAX) {
                cout << "INF";
            }
            else {
                cout << distanceMatrix[initialPosition][j];
            }
            cout << "\t";
        }
        cout << endl;
    }
    void displayShortestPathsFromUserDefinedPositionFloydWarshall() {
        int startPositionRow, startPositionCol;
        cout << "Enter the starting position row: ";
        cin >> startPositionRow;
        cout << "Enter the starting position column: ";
        cin >> startPositionCol;

        if (startPositionRow < 0 || startPositionRow >= rows || startPositionCol < 0 || startPositionCol >= cols) {
            cout << "Invalid starting position!" << endl;
            return;
        }

        int startPosition = startPositionRow * cols + startPositionCol;
        vector<vector<int>> distanceMatrix = initializeDistanceMatrix();
        floydWarshall(distanceMatrix);

        cout << "Shortest paths from the user-defined position (" << startPositionRow << ", " << startPositionCol << ") (Floyd-Warshall Algorithm):" << endl;
        for (int j = 0; j < rows * cols; ++j) {
            if (distanceMatrix[startPosition][j] == INT_MAX) {
                cout << "INF";
            }
            else {
                cout << distanceMatrix[startPosition][j];
            }
            cout << "\t";
        }
        cout << endl;
    }
    int edgeWeight(int position1, int position2) {
        NodeType nodeType1 = nodeTypes[position1];
        NodeType nodeType2 = nodeTypes[position2];

        if (nodeType1 == O || nodeType1 == D || nodeType1 == G || nodeType1 == WE || nodeType1 == DR ||
            nodeType2 == O || nodeType2 == D || nodeType2 == G || nodeType2 == WE || nodeType2 == DR) {
            return INT_MAX; // Maximum weight for edges connecting obstacles or dangerous points
        }

        return 1; // For other cases, the edge weight is simply 1
    }
    vector<pair<int, int>> prim() {
        vector<pair<int, int>> mstEdges; // Minimum Spanning Tree edges
        unordered_set<int> visited;
        priority_queue<pair<int, pair<int, int>>, vector<pair<int, pair<int, int>>>, greater<pair<int, pair<int, int>>>> pq;

        int startNode = currentRow * cols + currentCol;
        visited.insert(startNode);

        for (int neighbor : adjacencyList[startNode]) {
            pq.push({ edgeWeight(startNode, neighbor), {startNode, neighbor} });
        }

        while (!pq.empty()) {
            int weight = pq.top().first;
            int node1 = pq.top().second.first;
            int node2 = pq.top().second.second;
            pq.pop();

            if (visited.count(node2) == 0) {
                visited.insert(node2);
                mstEdges.push_back({ node1, node2 });

                for (int neighbor : adjacencyList[node2]) {
                    if (visited.count(neighbor) == 0) {
                        pq.push({ edgeWeight(node2, neighbor), {node2, neighbor} });
                    }
                }
            }
        }

        return mstEdges;
    }
    void displayMST() {
        vector<pair<int, int>> mstEdges = prim();

        cout << "Minimum Spanning Tree Edges:" << endl;
        for (const auto& edge : mstEdges) {
            cout << "(" << edge.first / cols << ", " << edge.first % cols << ") - ("
                << edge.second / cols << ", " << edge.second % cols << ")" << endl;
        }
    }
    vector<int> parent;

    int findSet(int x) {
        if (parent[x] == x) {
            return x;
        }
        return parent[x] = findSet(parent[x]);
    }

    void unionSets(int x, int y) {
        int rootX = findSet(x);
        int rootY = findSet(y);
        if (rootX != rootY) {
            parent[rootY] = rootX;
        }
    }
    vector<pair<int, int>> kruskal() {
        vector<pair<int, int>> mstEdges; // Minimum Spanning Tree edges
        parent.resize(rows * cols);

        for (int i = 0; i < rows * cols; ++i) {
            parent[i] = i;
        }

        vector<pair<int, pair<int, int>>> edges;
        for (const auto& entry : adjacencyList) {
            int node1 = entry.first;
            for (const int node2 : entry.second) {
                edges.push_back({ edgeWeight(node1, node2), {node1, node2} });
            }
        }

        sort(edges.begin(), edges.end());

        for (const auto& edge : edges) {
            int weight = edge.first;
            int node1 = edge.second.first;
            int node2 = edge.second.second;

            if (findSet(node1) != findSet(node2)) {
                unionSets(node1, node2);
                mstEdges.push_back({ node1, node2 });
            }
        }

        return mstEdges;
    }
    void displayMSTKruskal() {
        vector<pair<int, int>> mstEdges = kruskal();

        cout << "Minimum Spanning Tree Edges (Kruskal's Algorithm):" << endl;
        for (const auto& edge : mstEdges) {
            cout << "(" << edge.first / cols << ", " << edge.first % cols << ") - ("
                << edge.second / cols << ", " << edge.second % cols << ")" << endl;
        }
    }
    unordered_map<int, int> dijkstra(int targetPosition) {
        int startPosition = currentRow * cols + currentCol;

        unordered_map<int, int> prev; // To store previous node in the shortest path
        unordered_map<int, int> dist; // To store the shortest distance to each node
        unordered_set<int> unvisited;

        for (int i = 0; i < rows * cols; ++i) {
            dist[i] = INT_MAX;
            prev[i] = -1;
            unvisited.insert(i);
        }

        dist[startPosition] = 0;

        while (!unvisited.empty()) {
            int currentNode = -1;
            int minDist = INT_MAX;

            for (int node : unvisited) {
                if (dist[node] < minDist) {
                    currentNode = node;
                    minDist = dist[node];
                }
            }

            if (currentNode == -1) {
                break;
            }

            unvisited.erase(currentNode);

            if (currentNode == targetPosition) {
                break;
            }

            for (int neighbor : adjacencyList[currentNode]) {
                int weight = edgeWeight(currentNode, neighbor);

                if (dist[currentNode] + weight < dist[neighbor]) {
                    dist[neighbor] = dist[currentNode] + weight;
                    prev[neighbor] = currentNode;
                }
            }
        }

        return prev;
    }
    vector<int> reconstructPath(int targetPosition, unordered_map<int, int>& prev) {
        vector<int> path;
        int currentNode = targetPosition;

        while (currentNode != -1) {
            path.push_back(currentNode);
            currentNode = prev[currentNode];
        }

        reverse(path.begin(), path.end());

        return path;
    }

    int findCrystalPosition() {
        for (const auto& entry : nodeTypes) {
            if (entry.second == X) {
                return entry.first;
            }
        }
        return -1;
    }

    void displayShortestPathDijkstra() {
        int targetPosition = findCrystalPosition();
        if (targetPosition == -1) {
            cout << "No crystal found on the map!" << endl;
            return;
        }

        unordered_map<int, int> prev = dijkstra(targetPosition);
        vector<int> path = reconstructPath(targetPosition, prev);

        if (path.size() <= 1) {
            cout << "No valid path found!" << endl;
        }
        else {
            cout << "Shortest path to the crystal (Dijkstra's Algorithm):" << endl;
            for (int position : path) {
                cout << "(" << position / cols << ", " << position % cols << ")";
                if (position != targetPosition) {
                    cout << " -> ";
                }
            }
            cout << endl;
        }
    }

    void displayShortestPathFromUserDefinedPositionDijkstra() {
        int startPositionRow, startPositionCol;
        cout << "Enter the starting position row: ";
        cin >> startPositionRow;
        cout << "Enter the starting position column: ";
        cin >> startPositionCol;

        if (startPositionRow < 0 || startPositionRow >= rows || startPositionCol < 0 || startPositionCol >= cols) {
            cout << "Invalid starting position!" << endl;
            return;
        }

        int startPosition = startPositionRow * cols + startPositionCol;
        int targetPosition = findCrystalPosition();
        if (targetPosition == -1) {
            cout << "No crystal found on the map!" << endl;
            return;
        }

        setCurrentPosition(startPositionRow, startPositionCol);
        unordered_map<int, int> prev = dijkstra(targetPosition);
        vector<int> path = reconstructPath(targetPosition, prev);

        if (path.size() <= 1) {
            cout << "No valid path found!" << endl;
        }
        else {
            cout << "Shortest path to the crystal from user-defined position (" << startPositionRow << ", " << startPositionCol << ") (Dijkstra's Algorithm):" << endl;
            for (int position : path) {
                cout << "(" << position / cols << ", " << position % cols << ")";
                if (position != targetPosition) {
                    cout << " -> ";
                }
            }
            cout << endl;
        }
    }
    void displayGameMap() {
        for (int i = 0; i < rows; ++i) {
            for (int j = 0; j < cols; ++j) {
                int position = i * cols + j;
                if (position == currentRow * cols + currentCol) {
                    cout << "-";
                }
                else if (nodeTypes.find(position) != nodeTypes.end()) {
                    cout << getNodeTypeName(nodeTypes[position]);
                }
                else {
                    cout << " ";
                }
                cout << " ";
            }
            cout << endl;
        }
    }
};

int main() {
    GameMap gameMap(4, 4);
    AVLTree player;
    srand(time(0));

    gameMap.addNode(0, 'C');
    gameMap.addNode(1, 'C');
    gameMap.addNode(2, 'C');
    gameMap.addNode(3, '#');
    gameMap.addNode(4, '%');
    gameMap.addNode(5, '$');
    gameMap.addNode(6, 'C');
    gameMap.addNode(7, '&');
    gameMap.addNode(8, '@');
    gameMap.addNode(9, 'C');
    gameMap.addNode(10, 'P');
    gameMap.addNode(11, 'G');
    gameMap.addNode(12, 'J');
    gameMap.addNode(13, 'C');
    gameMap.addNode(14, '*');
    gameMap.addNode(15, 'W');

    cout << "Game map:" << endl;
    gameMap.displayGameMap();
    //cout << "----" << endl;
    gameMap.connectAdjacentNodes();
    /* cout << "Adjacency list of the game map:" << endl;
     gameMap.displayAdjacencyList();*/
    gameMap.setCurrentPosition(0, 0);
    int decide = 0;
    int score = 0;
    int id;
    int choice;
    do
    {
        cout << "Press 1 for existing player or 2 for new player or 0 to exit" << endl;
        cin >> decide;
        if (decide == 1)
        {
            id = 100;
            player.updateNode(id, score);
        }
        else if (decide == 2)
        {
            id = (rand() % 200) + 0;
            Node* play = new Node();
            player.insert(play, id, score);
        }
        else
        {
            cout << "Invalid Input" << endl;
        }
        do
        {
            player.updateNode(id, score);
            gameMap.move();
            gameMap.displayGameMap();
            if (gameMap.getCurrentNodeValue() == J)
            {
                score = score + 50;
            }
            if (gameMap.getCurrentNodeValue() == W)
            {
                score = score + 30;
            }
            if (gameMap.getCurrentNodeValue() == P)
            {
                score = score + 70;
            }
            if (gameMap.getCurrentNodeValue() == WE)
            {
                score = score - 30;
            }
            if (gameMap.getCurrentNodeValue() == G)
            {
                score = score - 70;
            }
            if (gameMap.getCurrentNodeValue() == DR)
            {
                score = score - 50;
            }
            if (gameMap.getCurrentNodeValue() == X)
            {
                cout << endl;
                cout << "       Congrats! You Win" << endl;
                cout << endl;

                cout << "Score : " << player.getRewardScore(id) << endl;
                break;
            }
            if (gameMap.getCurrentNodeValue() == D)
            {
                cout << endl;
                cout << "       You are dead    " << endl;
                cout << endl;

                break;
            }
            if (player.getRewardScore(id) < 0)
            {
                cout << endl;
                cout << "       You Lose" << endl;
                cout << endl;

                break;
            }
            cout << "Score : " << player.getRewardScore(id) << endl;
        } while (gameMap.getCurrentNodeValue() != X || player.getRewardScore(100) < 0);
        gameMap.displayMST();
        gameMap.displayMSTKruskal();
        gameMap.displayShortestPathDijkstra();
        gameMap.displayShortestPathsFloydWarshall();
        gameMap.displayShortestPathsFromInitialPositionFloydWarshall();
        cout << "For advnace search press 1 else press 0" << endl;
        cin >> choice;
        if (choice == 1)
        {
            gameMap.displayShortestPathFromUserDefinedPositionDijkstra();
            gameMap.displayShortestPathsFromUserDefinedPositionFloydWarshall();
        }
        cout << "press 0 to quite or 1 to conitnue" << endl;
        cin >> decide;
    } while (decide != 0);
    return 0;
};
