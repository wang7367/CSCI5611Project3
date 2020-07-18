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
float agentRad = 20.0;
Agent[] myAgent=new Agent[numAgent];
PShape agentShape;

// camera parameters
Vec3 cameraPos, cameraDir;
float theta, phi;

void setup(){
  size(1024,768, P3D);

  // set camera parameters
  cameraPos = new Vec3(width/2.0, height/2.0, 665);
  theta = -PI/2; phi = PI/2;
  cameraDir = new Vec3(cos(theta)*sin(phi),cos(phi),sin(theta)*sin(phi));
  
  // load sail model
  agentShape = loadShape("boat_small.obj");
  
  // place obstacles
  placeRandomObstacles(numCircle, numBox);
  
  // create start & goal
  for (int i = 0; i < numAgent; i++){
    startPos[i] = sampleFreePos();
    initialStartPos[i] = startPos[i];
    goalPos[i] = sampleFreePos();
    myAgent[i] = new Agent(startPos[i], agentRad);
  }
  
  prm = new RoadMap(numNodes, circlePos, circleRad, numCircle, boxTopLeft, boxW, boxH, numBox);
  
  // CHANGE FUNCTION NAME
  // reset(): build PRM and find paths
  reset();
}

void draw(){
  surface.setTitle(String.format("fps: %f",frameRate));
  
  //if the user has placed a new obstacle, reset the road map and the start position becomes where the agent is currently at.
  if(changed==true){
    for(int i=0;i<numAgent;i++){
      // deep copy
      startPos[i].x = myAgent[i].pos.x;
      startPos[i].y = myAgent[i].pos.y;
   }
    reset();
    changed = false;
  }
  
  if (!paused){
    for(int i=0;i<numAgent;i++){
      myAgent[i].step(1.0, circlePos, circleRad, numCircle, boxTopLeft, boxW, boxH, numBox, myAgent, i, numAgent);
    }
  }
  cameraUpdate(0.05);
  
  // lights settings
  directionalLight(180, 180, 180, -1, 1, -1);
  ambientLight(200, 200, 200);
  specular(255);
  
  strokeWeight(1);

  // sky blue background
  background(#87CEEB);
  
  // ground
  stroke(#4B1E0B);
  fill(#8A360F);
  float widthBlank = 200.0, heightBlank = 150.0, depthBlank = 25.0;
  //pushMatrix();
  //translate(width/2.0, height/2.0, -2000.0);
  //box(width+widthBlank, height+heightBlank, 4000);
  //popMatrix();
    
  // lake
  fill(#375BC4);
  pushMatrix();
  translate(width/2.0, height/2.0, -2000.0);
  box(width+widthBlank, height+heightBlank, 4000.0);
  popMatrix();
  
  // camera settings
  camera(cameraPos.x, cameraPos.y, cameraPos.z,
  cameraPos.x+cameraDir.x, cameraPos.y+cameraDir.y, cameraPos.z+cameraDir.z,
  0.0, 1.0, 0.0);

  noStroke();
  // obstacles settings
  fill(#8A360F);
  // draw the circle obstacles
  for (int i = 0; i < numCircle; i++){
    Vec2 c = circlePos[i];
    float r = circleRad[i]-agentRad;
    pushMatrix();
    translate(c.x, c.y, 0);
    for (float rad = 0.0; rad < PI*2; rad+=PI/6){
      Vec2 c1 = new Vec2(r*cos(rad), r*sin(rad));
      Vec2 c2 = new Vec2(r*cos(rad+PI/6), r*sin(rad+PI/6));
      // draw bottom triangle
      beginShape();
      vertex(0, 0, 0);
      vertex(c1.x, c1.y, 0);
      vertex(c2.x, c2.y, 0);
      endShape(CLOSE);
      // draw top triangle
      beginShape();
      vertex(0, 0, depthBlank);
      vertex(c1.x, c1.y, depthBlank);
      vertex(c2.x, c2.y, depthBlank);
      endShape(CLOSE);
      // draw sides
      beginShape();
      vertex(0, 0, 0);
      vertex(0, 0, depthBlank);
      vertex(c1.x, c1.y, depthBlank);
      vertex(c1.x, c1.y, 0);
      endShape(CLOSE);
      beginShape();
      vertex(0, 0, 0);
      vertex(0, 0, depthBlank);
      vertex(c2.x, c2.y, depthBlank);
      vertex(c2.x, c2.y, 0);
      endShape(CLOSE);
      beginShape();
      vertex(c1.x, c1.y, 0);
      vertex(c1.x, c1.y, depthBlank);
      vertex(c2.x, c2.y, depthBlank);
      vertex(c2.x, c2.y, 0);
      endShape(CLOSE);
    }
    popMatrix();
  }
  // draw the box obstacles
  for (int i = 0; i < numBox; i++){
    Vec2 c = boxTopLeft[i];
    float lenX = boxW[i]-agentRad*2, lenY = boxH[i]-agentRad*2, lenZ = agentRad*2;
    pushMatrix();
    translate(c.x+agentRad+lenX/2, c.y+agentRad+lenY/2, depthBlank/2.0);
    box(boxW[i]-agentRad*2, boxH[i]-agentRad*2, depthBlank);
    popMatrix();
  }
      
   Vec3 color0 = new Vec3(20, 60, 250);
   Vec3 color1 = new Vec3(250, 30, 50);
   for(int i=0; i<numAgent;i++){
     Vec3 colorNow = interpolate(color0, color1, i/(float)numAgent);
   // TODO: LINEAR INTERPOLATION -> start & goal COLOR
   // draw start
   noStroke();
  fill(colorNow.x, colorNow.y, colorNow.z);
  circle(initialStartPos[i].x,initialStartPos[i].y,agentRad*2);
  // draw goal
  circle(goalPos[i].x,goalPos[i].y,agentRad*2);
  stroke(colorNow.x, colorNow.y, colorNow.z);
  strokeWeight(5);
  prm.displayPath(startPos[i], goalPos[i], curPath[i]);
   }
  // draw agent
  fill(100, 255, 200);
 for(int i=0;i<numAgent;i++){
  myAgent[i].display();
 }
}

void placeRandomObstacles(int numCircle, int numBox){
  // initial obstacle position
  for (int i = 0; i < numCircle; i++){
    circlePos[i] = new Vec2(random(50,950),random(50,700));
    circleRad[i] = agentRad+10+40*pow(random(1),2);
  }
  for (int i = 0; i < numBox; i++){
    boxTopLeft[i] = new Vec2(random(50,950), random(50,600));
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
      
     myAgent[0] = new Agent(startPos[0], agentRad); reset();
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

int whichOne;
boolean circleSelected;
boolean movingObstacles;
//place obstacles by mouse
void mousePressed(){
  if(ctrlPressed){
     circleSelected=false;
     movingObstacles=true;
    
    for (int i = 0; i < numCircle; i++){
    if(dist(circlePos[i].x,circlePos[i].y,mouseX,mouseY)<20){
        whichOne=i;
        circleSelected=true;
         changed=true;
        return;
    }
  }
  if(!circleSelected){
  for (int i = 0; i < numBox; i++){
    if(((mouseX-boxTopLeft[i].x)>boxW[i]/4)&&((mouseX-boxTopLeft[i].x)<boxW[i]*3/4)&&((mouseY-boxTopLeft[i].y)>boxH[i]/4)&&((mouseY-boxTopLeft[i].y)<boxH[i]*3/4)) {
      whichOne=i;
      changed=true;
    return;
    }
  }
  }
  }else{
 placeObstacles(mouseX, mouseY);
 changed=true;
 movingObstacles=false;
  }
}

void mouseDragged(){
  if(movingObstacles){
    
  if(circleSelected){
   circlePos[whichOne].x=mouseX;
        circlePos[whichOne].y=mouseY;
  }else{
      boxTopLeft[whichOne].x=mouseX;
    boxTopLeft[whichOne].y=mouseY;
  }
  }
  
}

void placeObstacles(float posX, float posY){
  circlePos[numCircle] = new Vec2(posX, posY);
  circleRad[numCircle] = agentRad+30;
  numCircle++; 
}

boolean paused = true;

// rebuild the roadmap
void reset(){
  // create prm
  prm.updateCurrentNodes(circlePos, circleRad, numCircle, boxTopLeft, boxW, boxH, numBox);
  prm.connectNeighbors(circlePos, circleRad, numCircle, boxTopLeft, boxW, boxH, numBox);
  
  // plan paths
  for(int i=0; i<numAgent; i++){
    // find path in PRM
    curPath[i] = prm.planPath(startPos[i], goalPos[i], circlePos, circleRad, numCircle, boxTopLeft, boxW, boxH, numBox);
   
    // add path goals to agents
    myAgent[i].clearGoal(); // clear goal buffer
    if (curPath[i].size() == 1 && curPath[i].get(0) == -1)
      continue;  // skip if no path
    for (int ind : curPath[i])
      myAgent[i].addGoal(prm.nodePos[ind]);
    myAgent[i].addGoal(goalPos[i]);
  }
}
