// ---------- Pin definitions ----------
#define TRIG_FRONT 9
#define ECHO_FRONT 10
#define TRIG_LEFT 7
#define ECHO_LEFT 8
#define TRIG_RIGHT 5
#define ECHO_RIGHT 6

// Motor driver pins (L298N)
#define IN1 2
#define IN2 3
#define IN3 4
#define IN4 12
#define ENA 11  // PWM speed control, left motor
#define ENB 13  // PWM speed control, right motor

// ---------- Maze data structures ----------
#define ROWS 5
#define COLS 5

int maze[ROWS][COLS];      // -1 = unexplored, 0 = open, 1 = wall
int visited[ROWS][COLS];   // 0 = not visited, 1 = visited

int currentRow = 0;
int currentCol = 0;
int facing = 0; // 0 = North, 1 = East, 2 = South, 3 = West

int goalRow = ROWS - 1;
int goalCol = COLS - 1;

// ---------- BFS data structures ----------
int parentRow[ROWS][COLS];
int parentCol[ROWS][COLS];
bool bfsVisited[ROWS][COLS];

int queueRow[ROWS * COLS];
int queueCol[ROWS * COLS];
int qHead, qTail;

// ---------- Function prototypes ----------
long readDistance(int trigPin, int echoPin);
bool wallDetected(long distanceCM);
void printMaze();
void moveForward();
void turnLeft();
void turnRight();
void stopMoving();
void getNextCell(int row, int col, int fac, int *nextRow, int *nextCol);
void updateMap(bool frontWall, bool leftWall, bool rightWall);
void markWall(int row, int col, int direction);
bool isValidCell(int row, int col);
void turnToFace(int targetFacing);
void explore(int row, int col);
bool runBFS();
void printPath();

// ---------- Setup ----------
void setup() {
  Serial.begin(9600);

  pinMode(TRIG_FRONT, OUTPUT);
  pinMode(ECHO_FRONT, INPUT);
  pinMode(TRIG_LEFT, OUTPUT);
  pinMode(ECHO_LEFT, INPUT);
  pinMode(TRIG_RIGHT, OUTPUT);
  pinMode(ECHO_RIGHT, INPUT);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  delay(1000);

  for (int r = 0; r < ROWS; r++) {
    for (int c = 0; c < COLS; c++) {
      maze[r][c] = -1;
      visited[r][c] = 0;
    }
  }

  Serial.println("Maze initialized:");
  printMaze();

  // ---- Step 7: Explore the maze ----
  Serial.println("Starting maze exploration...");
  explore(currentRow, currentCol);
  Serial.println("Exploration complete. Final map:");
  printMaze();

  // ---- Step 8: Compute shortest path with BFS ----
  Serial.println("Running BFS for shortest path...");
  bool found = runBFS();

  if (found) {
    Serial.println("Shortest path found:");
    printPath();
  } else {
    Serial.println("No path found to goal.");
  }

  Serial.println("=== Full run complete (Step 9 checkpoint) ===");
}

// ---------- Sensor functions ----------
long readDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 30000);
  if (duration == 0) return 999;
  return duration * 0.034 / 2;
}

bool wallDetected(long distanceCM) {
  return distanceCM < 10;
}

// ---------- Direction/position logic ----------
void getNextCell(int row, int col, int fac, int *nextRow, int *nextCol) {
  *nextRow = row;
  *nextCol = col;
  if (fac == 0) *nextRow = row - 1;
  if (fac == 1) *nextCol = col + 1;
  if (fac == 2) *nextRow = row + 1;
  if (fac == 3) *nextCol = col - 1;
}

// ---------- Sensor-to-map integration ----------
void updateMap(bool frontWall, bool leftWall, bool rightWall) {
  visited[currentRow][currentCol] = 1;
  if (maze[currentRow][currentCol] == -1) maze[currentRow][currentCol] = 0;

  int frontDir = facing;
  int leftDir = (facing + 3) % 4;
  int rightDir = (facing + 1) % 4;

  if (frontWall) markWall(currentRow, currentCol, frontDir);
  if (leftWall)  markWall(currentRow, currentCol, leftDir);
  if (rightWall) markWall(currentRow, currentCol, rightDir);
}

void markWall(int row, int col, int direction) {
  int wallRow = row, wallCol = col;
  if (direction == 0) wallRow = row - 1;
  if (direction == 1) wallCol = col + 1;
  if (direction == 2) wallRow = row + 1;
  if (direction == 3) wallCol = col - 1;

  if (wallRow >= 0 && wallRow < ROWS && wallCol >= 0 && wallCol < COLS) {
    maze[wallRow][wallCol] = 1;
  }
}

// ---------- Exploration logic (Step 7) ----------
bool isValidCell(int row, int col) {
  if (row < 0 || row >= ROWS || col < 0 || col >= COLS) return false;
  if (maze[row][col] == 1) return false;
  if (visited[row][col] == 1) return false;
  return true;
}

void turnToFace(int targetFacing) {
  int diff = (targetFacing - facing + 4) % 4;
  if (diff == 1) turnRight();
  else if (diff == 3) turnLeft();
  else if (diff == 2) { turnRight(); turnRight(); }
}

void explore(int row, int col) {
  long frontDist = readDistance(TRIG_FRONT, ECHO_FRONT);
  long leftDist  = readDistance(TRIG_LEFT, ECHO_LEFT);
  long rightDist = readDistance(TRIG_RIGHT, ECHO_RIGHT);

 Serial.print("Front: ");
 Serial.print(frontDist);
 Serial.print(" cm | Left: ");
 Serial.print(leftDist);
 Serial.print(" cm | Right: ");
 Serial.print(rightDist);
 Serial.println(" cm");

  bool frontWall = wallDetected(frontDist);
  bool leftWall  = wallDetected(leftDist);
  bool rightWall = wallDetected(rightDist);

  updateMap(frontWall, leftWall, rightWall);

  for (int dir = 0; dir < 4; dir++) {
    int nextRow, nextCol;
    getNextCell(row, col, dir, &nextRow, &nextCol);

    if (isValidCell(nextRow, nextCol)) {
      turnToFace(dir);
      moveForward();

      currentRow = nextRow;
      currentCol = nextCol;

      explore(nextRow, nextCol);

      turnToFace((dir + 2) % 4);
      moveForward();
      currentRow = row;
      currentCol = col;
      turnToFace(dir);
    }
  }
}

// ---------- Step 8: BFS shortest path ----------
bool runBFS() {
  for (int r = 0; r < ROWS; r++) {
    for (int c = 0; c < COLS; c++) {
      bfsVisited[r][c] = false;
      parentRow[r][c] = -1;
      parentCol[r][c] = -1;
    }
  }

  qHead = 0;
  qTail = 0;
  queueRow[qTail] = 0;
  queueCol[qTail] = 0;
  qTail++;
  bfsVisited[0][0] = true;

  while (qHead < qTail) {
    int r = queueRow[qHead];
    int c = queueCol[qHead];
    qHead++;

    if (r == goalRow && c == goalCol) {
      return true;
    }

    int deltaRow[4] = {-1, 0, 1, 0};
    int deltaCol[4] = {0, 1, 0, -1};

    for (int i = 0; i < 4; i++) {
      int nr = r + deltaRow[i];
      int nc = c + deltaCol[i];

      if (nr >= 0 && nr < ROWS && nc >= 0 && nc < COLS &&
          maze[nr][nc] != 1 && !bfsVisited[nr][nc]) {
        bfsVisited[nr][nc] = true;
        parentRow[nr][nc] = r;
        parentCol[nr][nc] = c;
        queueRow[qTail] = nr;
        queueCol[qTail] = nc;
        qTail++;
      }
    }
  }

  return false;
}

void printPath() {
  int pathRow[ROWS * COLS];
  int pathCol[ROWS * COLS];
  int pathLength = 0;

  int r = goalRow;
  int c = goalCol;

  while (!(r == 0 && c == 0)) {
    pathRow[pathLength] = r;
    pathCol[pathLength] = c;
    pathLength++;

    int pr = parentRow[r][c];
    int pc = parentCol[r][c];
    r = pr;
    c = pc;
  }
  pathRow[pathLength] = 0;
  pathCol[pathLength] = 0;
  pathLength++;

  for (int i = pathLength - 1; i >= 0; i--) {
    Serial.print("(");
    Serial.print(pathRow[i]);
    Serial.print(",");
    Serial.print(pathCol[i]);
    Serial.print(") ");
  }
  Serial.println();
}

// ---------- Maze helper functions ----------
void printMaze() {
  for (int r = 0; r < ROWS; r++) {
    for (int c = 0; c < COLS; c++) {
      Serial.print(maze[r][c]);
      Serial.print(" ");
    }
    Serial.println();
  }
}

// ---------- Motor functions (Step 10 - real driver code) ----------
void moveForward() {
  Serial.println("Motor: moving forward");
  Serial.println("ACTION: Move forward");
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 180); // speed 0-255, tune this
  analogWrite(ENB, 180);
  delay(500); // tune: how long = roughly one maze cell
  stopMoving();
}

void turnLeft() {
  Serial.println("ACTION: Turn left");
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 150);
  analogWrite(ENB, 150);
  delay(300); // tune: how long = roughly 90 degrees
  stopMoving();
  facing = (facing + 3) % 4;
}

void turnRight() {
  Serial.println("ACTION: Turn right");
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, 150);
  analogWrite(ENB, 150);
  delay(300); // tune: how long = roughly 90 degrees
  stopMoving();
  facing = (facing + 1) % 4;
}

void stopMoving() {
  Serial.println("ACTION: Stop");
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

// ---------- Main loop ----------
void loop() {
  // Everything already ran once in setup().
}