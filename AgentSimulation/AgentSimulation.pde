// initialize obstacle settings
static int maxNumObstacles = 1000;
// initialize circle obstacles
int numCircle = 50;
Vec2 circlePos[] = new Vec2[maxNumObstacles/2];
float circleRad[] = new float[maxNumObstacles/2];
// initialize box obstacles
int numBox = 50;
Vec2 boxTopLeft[] = new Vec2[maxNumObstacles/2];
float[] boxW = new float[maxNumObstacles/2];
float[] boxH = new float[maxNumObstacles/2];

// initialize prm settings
int numNodes  = 500;
RoadMap prm;

int numAgent=5;

// initialize path settings
Vec2[] startPos = new Vec2[numAgent];
Vec2[] goalPos = new Vec2[numAgent];
Vec2[] initialStartPos =new Vec2[numAgent];
ArrayList<Integer>[] curPath=new ArrayList[numAgent];

// initialize agent settings
float agentRad = 10.0;


Agent[] myAgent=new Agent[numAgent];

// camera parameters
Vec3 cameraPos, cameraDir;
float theta, phi;

void setup(){
  size(1024,768, P3D);

  // set camera parameters
  cameraPos = new Vec3(width/2.0, height/2.0, 750);
  theta = -PI/2; phi = PI/2;
  cameraDir = new Vec3(cos(theta)*sin(phi),cos(phi),sin(theta)*sin(phi));
  cameraDir.mul(800);
  
  // place obstacles
  placeRandomObstacles(numCircle, numBox);
  
  // create start & goal
  
  for(int i=0; i<numAgent; i++){
    startPos[i] = sampleFreePos();
  initialStartPos[i]=startPos[i];
  goalPos[i] = sampleFreePos();
  myAgent[i] = new Agent(startPos[i], agentRad);
  }
   
  // create prm
  prm = new RoadMap(numNodes, circlePos, circleRad, numCircle, boxTopLeft, boxW, boxH, numBox);
  // path plan
   for(int i=0; i<numAgent; i++){
    curPath[i]=prm.planPath(startPos[i], goalPos[i], circlePos, circleRad, numCircle, boxTopLeft, boxW, boxH, numBox);
   
  // add path goals to agent
  if (curPath[i].size() == 1 && curPath[i].get(0) == -1)
    continue;
    for (int ind : curPath[i])
    myAgent[i].addGoal(prm.nodePos[ind]);
    
   myAgent[i].addGoal(goalPos[i]);
    
   }
   

  
  
}

void draw(){
  //if the user has placed a new obstacle, reset the road map and the start position becomes where the agent is currently at.
  if(changed==true){
    for(int i=0;i<numAgent;i++){
    startPos[i]=myAgent[i].pos;
     myAgent[i] = new Agent(startPos[i], agentRad);
   
   }
    reset(myAgent);
  }
  
  if (!paused){
    for(int i=0;i<numAgent;i++){
      myAgent[i].step(1.0, circlePos, circleRad, numCircle, boxTopLeft, boxW, boxH, numBox, myAgent, i, numAgent);
    }
  }
  cameraUpdate(0.05);
  
  //println("FrameRate:",frameRate);
  strokeWeight(1);

  // grey background
  background(200);
  // a cutting plane
  noStroke();
  fill(200);
  rect(0,0,5000,5000);
  // camera settings
  camera(cameraPos.x, cameraPos.y, cameraPos.z,
  cameraPos.x+cameraDir.x, cameraPos.y+cameraDir.y, cameraPos.z+cameraDir.z,
  0.0, 1.0, 0.0);

  // lights settings
  directionalLight(180, 180, 180, -1, 1, -1);
  ambientLight(150, 150, 150);
  specular(255);
  
  // obstacles settings
  fill(255);
  // draw the circle obstacles
  for (int i = 0; i < numCircle; i++){
    Vec2 c = circlePos[i];
    float r = circleRad[i]-agentRad;
    pushMatrix();
    translate(c.x, c.y, 0);
    sphere(r);
    popMatrix();
  }
  // draw the box obstacles
  for (int i = 0; i < numBox; i++){
    Vec2 c = boxTopLeft[i];
    float lenX = boxW[i]-agentRad*2, lenY = boxH[i]-agentRad*2, lenZ = agentRad*2;
    pushMatrix();
    translate(c.x+agentRad+lenX/2, c.y+agentRad+lenY/2, 0);
    box(boxW[i]-agentRad*2, boxH[i]-agentRad*2, agentRad*2);
    popMatrix();
  }
      
 
   for(int i=0; i<numAgent;i++){ 
   // draw start
  fill(20,60,250);
  circle(initialStartPos[i].x,initialStartPos[i].y,20);
  // draw goal
  fill(250,30,50);
  circle(goalPos[i].x,goalPos[i].y,20);
   }
  // draw agent
  fill(100, 255, 200);
 for(int i=0;i<numAgent;i++){
  myAgent[i].display();
 }
  // draw path
  stroke(20,255,40);
  strokeWeight(5);
  for(int i=0;i<numAgent;i++){
   prm.displayPath(startPos[i], goalPos[i], curPath[i]);
  }
  
}

void placeRandomObstacles(int numCircle, int numBox){
  // initial obstacle position
  for (int i = 0; i < numCircle; i++){
    circlePos[i] = new Vec2(random(50,950),random(50,700));
    circleRad[i] = agentRad+10+40*pow(random(1),2);
  }
  for (int i = 0; i < numBox; i++){
    boxTopLeft[i] = new Vec2(random(50,950), random(50,700));
    boxW[i] = agentRad*2+20+80*pow(random(1),2);
    boxH[i] = agentRad*2+20+80*pow(random(1),2);
  }
}

Vec2 sampleFreePos(){
  Vec2 randPos = new Vec2(random(width),random(height));
  boolean insideAnyCircle = pointInCircleList(circlePos,circleRad,numCircle,randPos);
  boolean insideAnyBox = pointInBoxList(boxTopLeft, boxW, boxH, numBox, randPos);
  while (insideAnyCircle || insideAnyBox){
    randPos = new Vec2(random(width),random(height));
    insideAnyCircle = pointInCircleList(circlePos,circleRad,numCircle,randPos);
    insideAnyBox = pointInBoxList(boxTopLeft, boxW, boxH, numBox, randPos);
  }
  return randPos;
}

// control camera according to keyboard and mouse inputs
void cameraUpdate(float step){
  if (ctrlPressed){
    Vec3 up = new Vec3(0.0,-1.0,0.0);
    up.subtract(cameraDir.times(cameraDir.y));
    up.normalize();
    if (upPressed) cameraPos.add(up.times(step*20));
    if (downPressed) cameraPos.subtract(up.times(step*20));
    Vec3 left = cross(cameraDir, up);
    if (leftPressed) cameraPos.add(left.times(step*20));
    if (rightPressed) cameraPos.subtract(left.times(step*20));
  }
  else{
    if (upPressed) phi += step;
    if (downPressed) phi -= step;
    if (leftPressed) theta -= step;
    if (rightPressed) theta += step;
    cameraDir.x = cos(theta)*sin(phi);
    cameraDir.y = cos(phi);
    cameraDir.z = sin(theta)*sin(phi);
  }
}
// control camera according to mouse input
void mouseWheel(MouseEvent event){
  cameraPos.add(cameraDir.times(-10*event.getCount()));
}

boolean leftPressed, rightPressed, upPressed, downPressed;
boolean ctrlPressed;
boolean wPressed, aPressed, sPressed, dPressed, qPressed, ePressed;
void keyPressed(){
  if (keyCode == LEFT) leftPressed = true;
  if (keyCode == RIGHT) rightPressed = true;
  if (keyCode == UP) upPressed = true;  
  if (keyCode == DOWN) downPressed = true;
  if (keyCode == CONTROL) ctrlPressed = true;
  if(hasStarted==false){
  if (key == 'w' || key == 'W') startPos[0].y-=5;   wPressed = true;
  if (key == 'a' || key == 'A') startPos[0].x-=5;   aPressed = true;
  if (key == 's' || key == 'S') startPos[0].y+=5;   sPressed = true;
  if (key == 'd' || key == 'D') startPos[0].x+=5;   dPressed = true;
      
     myAgent[0] = new Agent(startPos[0], agentRad); reset(myAgent);
  }
  if (key == 'q' || key == 'Q') qPressed = true;
  if (key == 'e' || key == 'E') ePressed = true;
  if (key == 'r' || key == 'R') {paused = true; setup();}
  
  
  if (key == ' ') { paused = !paused; hasStarted=true;}

  
  
 }
 
void keyReleased(){
  if (keyCode == LEFT) leftPressed = false;
  if (keyCode == RIGHT) rightPressed = false;
  if (keyCode == UP) upPressed = false;
  if (keyCode == DOWN) downPressed = false;
  if (keyCode == CONTROL) ctrlPressed = false;
  if (key == 'w' || key == 'W') wPressed = false;
  if (key == 'a' || key == 'A') aPressed = false;
  if (key == 's' || key == 'S') sPressed = false;
  if (key == 'd' || key == 'D') dPressed = false;
  if (key == 'q' || key == 'Q') qPressed = false;
  if (key == 'e' || key == 'E') ePressed = false;
}


boolean changed; //track if the mouse has been pressed.
boolean hasStarted;

//place obstacles by mouse
void mousePressed(){
 placeObstacles(mouseX, mouseY);
 changed=true;
}

void placeObstacles(float posX, float posY){
     
    circlePos[numCircle] = new Vec2(posX, posY);
    circleRad[numCircle] = agentRad+30;
    numCircle++;
    
    
}

boolean paused = true;

//reset the roadmap
void reset(Agent[] myAgent){
 
 // create prm
  prm = new RoadMap(numNodes, circlePos, circleRad, numCircle, boxTopLeft, boxW, boxH, numBox);
  // path plan
   
 for(int i=0; i<numAgent; i++){
  curPath[i] = prm.planPath(startPos[i], goalPos[i], circlePos, circleRad, numCircle, boxTopLeft, boxW, boxH, numBox);
   
  // add path goals to agent
  if (curPath[i].size() == 1 && curPath[i].get(0) == -1)
    continue;
   
      myAgent[i].clearGoal();
    for (int ind : curPath[i])
       myAgent[i].addGoal(prm.nodePos[ind]);
    myAgent[i].addGoal(goalPos[i]);
  }
  changed=false;
}
