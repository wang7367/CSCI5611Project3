class Agent{
  Vec2 pos;
  float rad;
  Vec2 vel=new Vec2(0,0);
  float k_avoid=3; //k avoid force
  int maxGoals = 20, numGoals = 0, firstGoal = 0;
  Vec2[] goals = new Vec2[maxGoals];
  
  Agent(Vec2 position, float radius){
    pos = new Vec2(position.x, position.y);
    rad = radius;
  }
  
  boolean addGoal(Vec2 newGoal){
    if (numGoals == maxGoals) return false;
    goals[(firstGoal+numGoals)%maxGoals] = new Vec2(newGoal.x, newGoal.y);
    numGoals++;
    return true;
  }
  
 
  
  
  void step(float stepLen,
            Vec2[] circlePos, float[] circleRad, int circleNum,
            Vec2[] boxPos, float[] boxW, float[] boxH, int boxNum, Agent[] agent, int id, int numAgent){
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
    
    //Adding avoid force
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
    
    
    
    Vec2 goalPos = goals[firstGoal];
    vel = goalPos.minus(pos).normalized();
    float dist = goalPos.distanceTo(pos);
    // if reach the goal, remove it from the list
    if (dist < stepLen) {
      pos.x = goalPos.x;
      pos.y = goalPos.y;
      firstGoal = (firstGoal+1) % maxGoals;
      numGoals--;
    }
    else{
      vel.add(avoidForce.times(stepLen));
      pos.add(vel.times(stepLen));
    }
    
  
    
  }
    
  void display(){
    pushMatrix();
    translate(pos.x, pos.y, 0);
    sphere(rad);
    popMatrix();
  }
  
  //Clear out every previous goal
   void clearGoal(){
     numGoals=0;
     goals = new Vec2[maxGoals];
  }
  
}
