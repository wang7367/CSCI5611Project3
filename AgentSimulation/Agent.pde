// agent class
class Agent{
  float rad;  // size attribution
  // motion attributions
  Vec2 pos;
  Vec2 vel=new Vec2(0,1.0);
  float k_avoid=3; //k avoid force parameter
  // goal attributions
  int maxGoals = 20, numGoals = 0, firstGoal = 0;
  Vec2[] goals = new Vec2[maxGoals];
  
  Agent(Vec2 position, float radius){
    pos = new Vec2(position.x, position.y);
    rad = radius;
  }
  
  // add new goal to the current list
  boolean addGoal(Vec2 newGoal){
    if (numGoals == maxGoals) return false; // return false if list is full
    goals[(firstGoal+numGoals)%maxGoals] = new Vec2(newGoal.x, newGoal.y);
    numGoals++;
    return true;
  }
  
  // move toward the goal
  void step(float stepLen,
            Vec2[] circlePos, float[] circleRad, int circleNum,
            Vec2[] boxPos, float[] boxW, float[] boxH, int boxNum, Agent[] agent, int id, int numAgent){
    // test to see if shortcutting possible
    while (numGoals > 1){
      int secondGoal = (firstGoal+1) % maxGoals;
      Vec2 dir = goals[secondGoal].minus(pos).normalized();
      float dist = goals[secondGoal].distanceTo(pos);
      hitInfo circleHit = rayCircleListIntersect(circlePos, circleRad, circleNum, pos, dir, dist);
      hitInfo boxHit = rayBoxListIntersect(boxPos, boxW, boxH, boxNum, pos, dir, dist);
      if (!circleHit.hit && !boxHit.hit) {
        firstGoal = secondGoal;
        numGoals--;
      }
      else
        break;
    }
    if (numGoals == 0) return;
    
    //Adding avoid force (with other agents)
    Vec2 avoidForce = new Vec2(0,0);
    for(int i=0; i<numAgent; i++){
      if(i == id) continue;
      float ttc = computeTTC(agent[id].pos, agent[id].vel, agentRad, agent[i].pos, agent[i].vel, agentRad);
      if(ttc > -1){
        Vec2 agentPo = agent[id].pos.plus(agent[id].vel.times(ttc));
        Vec2 agentNeh = agent[i].pos.plus(agent[i].vel.times(ttc));
        Vec2 futRelPos = agentPo.minus(agentNeh).normalized();
        avoidForce.add(futRelPos.times((k_avoid * (1/ttc))));
      }
    }
  
    // compute avoid force (with other obstacles)
    Vec2 avoidObjForce = new Vec2(0, 0);
    // with circles
    for (int i = 0; i < circleNum; i++){
      if (pos.distanceTo(circlePos[i]) < circleRad[i] + rad){
        Vec2 normal = (pos.minus(circlePos[i])).normalized();
        Vec2 velNormal = normal.times(dot(vel,normal));
        avoidObjForce.add(velNormal.times(1.0/pos.distanceTo(circlePos[i])));
      }
    }
    // with boxes
    for (int i = 0; i < boxNum; i++){
      Vec2 boxCenter=new Vec2(boxPos[i].x+boxW[i]/2.0,boxPos[i].y+boxH[i]/2.0);
      if ((abs(pos.x-boxCenter.x)<boxW[i]/2.0)&&(abs(pos.y-boxCenter.y)<boxH[i]/2.0)){
        Vec2 normal = (pos.minus(boxCenter)).normalized();
        Vec2 velNormal = normal.times(dot(vel,normal));
        avoidObjForce.add(velNormal.times(1.0/pos.distanceTo(boxCenter)));
      }
    }
    
    // compute leading force (toward next goal)
    Vec2 goalPos = goals[firstGoal];
    Vec2 goalForce = goalPos.minus(pos).normalized();
    
    // move forward
    float dist = goalPos.distanceTo(pos);
    // if reach the goal, remove it from the list (avoid turbulence)
    if (dist < stepLen) {
      pos.x = goalPos.x;
      pos.y = goalPos.y;
      firstGoal = (firstGoal+1) % maxGoals;
      numGoals--;
    }
    else{
      vel.add(goalForce.times(1.3));
      vel.add(avoidForce.times(1.0));
      vel.add(avoidObjForce.times(0.5));
      vel.clampToLength(5.0);
      pos.add(vel.times(0.2));
    }
  }
  
  void display(){
    // compute velAngle to rotate the model according to velocity
    float velAngle = dot(vel, new Vec2(0,1))/vel.length();
    if (vel.x > 0) velAngle = -acos(velAngle);
    else velAngle = +acos(velAngle);
    pushMatrix();
    translate(pos.x, pos.y, 0);
    scale(rad/10.0);
    rotateX(PI/2.0);
    rotateY(velAngle);
    shape(agentShape, 0, 0);
    popMatrix();
  }
  
  // Clear out every previous goal
   void clearGoal(){
     numGoals = 0;
     firstGoal = 0;
  }
  
}
