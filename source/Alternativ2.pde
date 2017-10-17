void alternativ2(){
  // fading background
   physics1.update();
 // fill(0,0,255);
  // draw all springs
  //stroke(255,0,255);
  for(VerletSpring2D s : physics1.springs) {
    line(s.a.x,s.a.y, s.b.x, s.b.y);
  }
  // show all particles
  //fill(0);
  noStroke();
  for(VerletParticle2D p : physics1.particles) {
    ellipse(p.x,p.y,5,5);
  }
  // highlight selected particle (if there is one currently)
  if (selectedParticle!=null) {
    //fill(255,0,255);
    ellipse(selectedParticle.x,selectedParticle.y,20,20);
  }
  //physics
  drawSkeleton1();
  // hand right
  spiel();
  Dragged();
  
  //hand left
   Spiel1();
   Dragged1();
   
  
  //fill(255,0,0);
    
  //// Display headline at x location
  //textFont(f, 30);
  //textAlign (LEFT);
  
  //// A specific String from the array is displayed according to the value of the "index" variable.
  //text(headlines[index], x, height-20); 

  //// Decrement x
  //x = x - 3;

  //// If x is less than the negative width, then it is off the screen
  //// textWidth() is used to calculate the width of the current String.
  //float w = textWidth(headlines[index]); 
  //if (x < -w) {
  //  x = width;
  //  // index is incremented when the current String has left the screen in order to display a new String.
  //  index = (index + 1) % headlines.length;
  //}
  
     
  //fill(0,0,255);
 
  //textSize(16);
  //textFont(f);
  //text("Spiel 1", 60, 50);
  //fill(0,100);
  //rect(10,20,140,60);
     
 
  // text("Spiel 2", width-60, 20);
  
  // put the image into a PImage

  cam = kinect.getBodyTrackImage();

  // copy the image into the smaller blob image

  blobs.copy(cam, 0, 0, cam.width, cam.height, 0, 0, blobs.width, blobs.height);

  // blur the blob image

  blobs.filter(BLUR);

  // detect the blobs

  theBlobDetection.computeBlobs(blobs.pixels);

  // clear the polygon (original functionality)

  poly2.reset();

  // create the polygon from the blobs (custom functionality, see class)

  poly2.createPolygon();
   // draw all springs
  

  drawFlowfield();
  //noLoop();
}
void setupFlowfield() {

  // set stroke weight (for particle display) to 2.5

  strokeWeight(2.5);

  // initialize all particles in the flow

  for(int i=0; i<flow.length; i++) {

    flow[i] = new Particle(i/10000.0);

  }

  // set all colors randomly now

  setRandomColors(1);
  

}
void drawFlowfield() {

  // center and reScale from Kinect to custom dimensions

  translate(0, (height-kinectHeight*reScale)/2);

  scale(reScale);

  // set global variables that influence the particle flow's movement

  globalX = noise(frameCount * 0.01) * width/2 + width/4;

  globalY = noise(frameCount * 0.005 + 5) * height;

  // update and display all particles in the flow

  for (Particle p : flow) {

    p.updateAndDisplay();

  }

  // set the colors randomly every 240th frame

  setRandomColors(240);

}
void setRandomColors(int nthFrame) {

  if (frameCount % nthFrame == 0) {

    // turn a palette into a series of strings

    String[] paletteStrings = split(palettes[int(random(palettes.length))], ",");

    // turn strings into colors

     colorPalette2 = new color[paletteStrings.length];

    for (int i=0; i<paletteStrings.length; i++) {

      colorPalette2[i] = int(paletteStrings[i]);

    }

    // set background color to first color from palette

    bgColor = colorPalette2[0];

    // set all particle colors randomly to color from palette (excluding first aka background color)

    for (int i=0; i<flow.length; i++) {

      flow[i].col = colorPalette2[int(random(1, colorPalette2.length))];

    }

  }

}



void spiel() {
  // find particle under mouse
  //Vec2D mousePos=new Vec2D(mouseX,mouseY);
  for(int i=1; i<physics1.particles.size()-1; i++) {
    VerletParticle2D p=physics1.particles.get(i);
    // using distanceToSquared() is faster than distanceTo()
    if (hand_right.distanceToSquared(p)<60*60) {
      // lock it and store for further reference
      selectedParticle=p.lock();
      // force quit the loop
      break;
    }
    else{
      if (selectedParticle!=null) {
    selectedParticle.unlock();
    selectedParticle=null;
  }
      
    }
    
}
    
}
void Spiel1(){
    for(int i=1; i<physics1.particles.size()-1; i++) {
    VerletParticle2D p=physics1.particles.get(i);
    // using distanceToSquared() is faster than distanceTo()
    if (hand_left.distanceToSquared(p)<SNAP_DIST) {
      // lock it and store for further reference
      selectedParticle=p.lock();
      // force quit the loop
      break;
    }
    else{
      if (selectedParticle!=null) {
    selectedParticle.unlock();
    selectedParticle=null;
  }
      
    }
    
}
  
}
  void Dragged1(){
  if (selectedParticle!=null) {
    // move selected particle to new mouse pos
    selectedParticle.set(hand_left.x,hand_left.y);
  }
  }
    
      
   


void Dragged() {
  if (selectedParticle!=null) {
    // move selected particle to new mouse pos
    selectedParticle.set(hand_right.x,hand_right.y);
  }
}

void Released() {
  // unlock the selected particle
  if (selectedParticle!=null) {
    selectedParticle.unlock();
    selectedParticle=null;
  }
}