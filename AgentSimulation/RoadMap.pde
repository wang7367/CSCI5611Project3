import java.util.PriorityQueue;
import java.util.Queue;
import java.util.Comparator;

// probabilitic roadmap class
class RoadMap{
  int maxNumNodes = 1000, numNodes;
  float[][] edges = new float[maxNumNodes+2][maxNumNodes+2];  // adjency matrix
  Vec2[] nodePos = new Vec2[maxNumNodes+2]; // nodes positions
  
  RoadMap(int nodeNum, Vec2[] circlePos, float[] circleRad, int circleNum,
                       Vec2[] boxPos, float[] boxW, float[] boxH, int boxNum){
    numNodes = nodeNum;
    // build map
    generateRandomNodes(circlePos, circleRad, circleNum, boxPos, boxW, boxH, boxNum);
    connectNeighbors(circlePos, circleRad, circleNum, boxPos, boxW, boxH, boxNum);
  }
  
  // plan a path
  ArrayList<Integer> planPath(Vec2 startPos, Vec2 goalPos,
                              Vec2[] circlePos, float[] circleRad, int circleNum,
                              Vec2[] boxPos, float[] boxW, float[] boxH, int boxNum){
    ArrayList<Integer> path = new ArrayList();
    // add start & goal pos as new nodes
    int startID = numNodes;
    addNewNode(startPos, circlePos, circleRad, circleNum, boxPos, boxW, boxH, boxNum);
    int goalID = numNodes;
    addNewNode(goalPos, circlePos, circleRad, circleNum, boxPos, boxW, boxH, boxNum);

    // run AStar to find path
    path = runAStar(startID, goalID);
    
    // delete newly added nodes
    deleteTails(2);
    return path;
  }
  
  void displayMap(){
    // draw nodes
    fill(0);
    for (int i = 0; i < numNodes; i++)
      circle(nodePos[i].x, nodePos[i].y, 5);
    
    // draw lines
    stroke(100, 100, 100);
    strokeWeight(1);
    for (int i = 0; i < numNodes; i++)
      for (int j = i+1; j < numNodes; j++)
        if (edges[i][j] > 0)
          line(nodePos[i].x, nodePos[i].y, nodePos[j].x, nodePos[j].y);
  }
  
  void displayPath(Vec2 startPos, Vec2 goalPos, ArrayList<Integer> path){

    if (path.size() == 1 && path.get(0) == -1) return; // stop if no path exists
    // draw settings
    if (path.size() == 0)
      line(startPos.x, startPos.y, goalPos.x, goalPos.y);
    else{
      Vec2 node0, node1 = nodePos[path.get(0)];
      line(startPos.x, startPos.y, node1.x, node1.y);
      for (int i = 0; i < path.size()-1; i++){
        node0 = nodePos[path.get(i)];
        node1 = nodePos[path.get(i+1)];
        line(node0.x, node0.y, node1.x, node1.y);
      }
      line(node1.x, node1.y, goalPos.x, goalPos.y);
    }
  }
  
  // run AStar
  ArrayList<Integer> runAStar(int startID, int goalID){
    ArrayList<Integer> path = new ArrayList<Integer>();
  
    Vec2 goal = nodePos[goalID];
    
  
    
    // create pairs list
    Pair[] aStarPairs = new Pair[numNodes];
    for (int i = 0; i < numNodes; i++)
      aStarPairs[i] = new Pair(i);
    
    // compute h(node)
    float[] h_val = new float[numNodes];
    for (int i = 0; i < numNodes; i++)
      h_val[i] = nodePos[i].distanceTo(goal);
    
    // initialize visited array & parentID array
    Boolean[] polled = new Boolean[numNodes];
    Integer[] parentID = new Integer[numNodes];
    for (int i = 0; i < numNodes; i++) {
      polled[i] = false;
      parentID[i] = -1;
    } 
  
    // initialize priority queue
    Queue<Pair> pq = new PriorityQueue<Pair>(new PairComparator());
    // add start point
    Pair startPair = aStarPairs[startID];
    startPair.value = h_val[startID] + 0.0;
    pq.add(startPair);
  
    while (!pq.isEmpty()){
      // poll a pair with least f(n)
      Pair curPair = pq.poll();
      polled[curPair.ID] = true;
    
      // goal found
      if (curPair.ID == goalID) break;
    
      // check neighbors of curNode
      for (int neighborID = 0; neighborID < numNodes; neighborID++){
        if (edges[curPair.ID][neighborID] <= 0) continue;  // skip if not connected or connect to itself
        if (polled[neighborID]) continue; // skip if path to that node already polled
      
      Pair neighborPair = aStarPairs[neighborID];
      float curGCost = (curPair.value - h_val[curPair.ID]) + edges[curPair.ID][neighborID];
      // add neighbor not in the priority queue
      if (!pq.contains(neighborPair)){
        neighborPair.value = h_val[neighborID] + curGCost;
        parentID[neighborID] = curPair.ID;
        pq.add(neighborPair);
      }
      else { 
        float oldGCost = neighborPair.value - h_val[neighborID];
        // update if a better path found
        if (curGCost < oldGCost) {
          parentID[neighborID] = curPair.ID;
          neighborPair.value = h_val[neighborID] + curGCost;
        }
      }
    }
  }
  
  
  int iterID = goalID;
  if (parentID[iterID] == -1) {
    path.add(-1);
  }
  else {
    while (parentID[iterID] != startID){
      iterID = parentID[iterID];
      path.add(0, iterID);
    }
  }
  return path;
}
  
  // generate random nodes
  //(ref: HW3_Test.zip: CSCI 5611 (001) Animation & Planning in Games (Summer 2020). (n.d.). 
  // Retrieved July 12, 2020, from https://canvas.umn.edu/courses/201953/files/13812334?module_item_id=4134939)
  void generateRandomNodes(Vec2[] circlePos, float[] circleRad, int circleNum,
                           Vec2[] boxPos, float[] boxW, float[] boxH, int boxNum){
    for (int i = 0; i < numNodes; i++){
      Vec2 randPos = new Vec2(random(width),random(height));
      boolean insideAnyCircle = pointInCircleList(circlePos, circleRad, circleNum, randPos);
      boolean insideAnyBox = pointInBoxList(boxPos, boxW, boxH, boxNum, randPos);
      while (insideAnyCircle || insideAnyBox){
        randPos = new Vec2(random(width),random(height));
        insideAnyCircle = pointInCircleList(circlePos, circleRad, circleNum, randPos);
        insideAnyBox = pointInBoxList(boxPos, boxW, boxH, boxNum, randPos);
      }
      nodePos[i] = randPos;
    }
  }
  
  void updateCurrentNodes(Vec2[] circlePos, float[] circleRad, int circleNum,
                          Vec2[] boxPos, float[] boxW, float[] boxH, int boxNum){
    for (int i = 0; i < numNodes; i++){
      boolean insideAnyCircle = pointInCircleList(circlePos, circleRad, circleNum, nodePos[i]);
      boolean insideAnyBox = pointInBoxList(boxPos, boxW, boxH, boxNum, nodePos[i]);
      while (insideAnyCircle || insideAnyBox){
        nodePos[i] = new Vec2(random(width),random(height)); // generate new node
        insideAnyCircle = pointInCircleList(circlePos, circleRad, circleNum, nodePos[i]);
        insideAnyBox = pointInBoxList(boxPos, boxW, boxH, boxNum, nodePos[i]);
      }
    }
  }
  
  // fill adjency matrix
  void connectNeighbors(Vec2[] circlePos, float[] circleRad, int circleNum,
                        Vec2[] boxPos, float[] boxW, float[] boxH, int boxNum){
    for (int i = 0; i < numNodes; i++){
      edges[i][i] = 0;
      for (int j = i+1; j < numNodes; j++) {
        Vec2 dir = nodePos[i].minus(nodePos[j]).normalized();
        float dist = nodePos[i].distanceTo(nodePos[j]);
        hitInfo hitCircles = rayCircleListIntersect(circlePos, circleRad, circleNum, nodePos[j], dir, dist);
        hitInfo hitBoxes = rayBoxListIntersect(boxPos, boxW, boxH, boxNum, nodePos[j], dir, dist);
        if (!hitCircles.hit && !hitBoxes.hit) {
          edges[i][j] = dist;
          edges[j][i] = dist;
        }
        else {
          edges[i][j] = -1;
          edges[j][i] = -1;
        }
      }
    }
  }
  
  void addNewNode(Vec2 newPos,
                  Vec2[] circlePos, float[] circleRad, int circleNum,
                  Vec2[] boxPos, float[] boxW, float[] boxH, int boxNum){
    // add new edges with original nodes
    for (int i = 0; i < numNodes; i++){
      // detect if collision with new node
      Vec2 dir = nodePos[i].minus(newPos).normalized();
      float dist = nodePos[i].distanceTo(newPos);
      hitInfo hitCircles = rayCircleListIntersect(circlePos, circleRad, circleNum, newPos, dir, dist);
      hitInfo hitBoxes = rayBoxListIntersect(boxPos, boxW, boxH, boxNum, newPos, dir, dist);
      if (!hitCircles.hit && !hitBoxes.hit) {
        edges[i][numNodes] = dist;
        edges[numNodes][i] = dist;
      }
      else {
        edges[i][numNodes] = -1;
        edges[numNodes][i] = -1;
      }
    }
    edges[numNodes][numNodes] = 0;
    nodePos[numNodes] = new Vec2(newPos.x, newPos.y);
    numNodes++;
  }
  
  void deleteTails(int deleteLen){
    numNodes-= deleteLen;
  }
}

// my own pair class used for A-Star priority queue search
public class Pair {
  public Integer ID;
  public float value;
  
  public Pair(int id){
    ID = id;
    value = 999999;
  }
}

class PairComparator implements Comparator<Pair>{
  public int compare(Pair a, Pair b){
    if (a.value > b.value) return 1;
    if (a.value == b.value) return 0;
    return -1;
  }
}
