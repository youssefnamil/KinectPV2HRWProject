void alternativ1(){
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
     //drawSkeleton();
   
   for (VerletParticle2D p : physics.particles) {
     for( Vec2D vec : mouseList){
   if(dist(vec.x, vec.y,p.x,p.y)<30){
   mousePos = new Vec2D(vec.x, vec.y);
  // create a new positive attraction force field around the mouse position (radius=250px)
  mouseAttractor = new AttractionBehavior2D(mousePos, 250, 0.9f);
  physics.addBehavior(mouseAttractor);
   mousePos.set(vec.x, vec.y);
   
   }
   }
   }
   
  
   
  //image(kinect.getDepthImage(), 0, 0);
  cam = kinect.getBodyTrackImage(); 
 
 // blobs = kinect.getBodyTrackImage(); 
 // copy the image into the smaller blob image
  blobs.copy(cam, 0, 0, cam.width, cam.height, 0, 0, blobs.width, blobs.height);
  // blur the blob image
  blobs.filter(BLUR, 1);
  // detect the blobs
  theBlobDetection.computeBlobs(blobs.pixels);
  // initialize a new polygon
  poly1 = new PolygonBlob1();
  // create the polygon from the blobs (custom functionality, see class)
  poly1.createPolygon();
  // create the box2d body from the polygon
  poly1.createBody();
 
  // update and draw everything (see method)
  updateAndDrawBox2D();
  // destroy the person's body (important!)
  poly1.destroyBody();
  //draw ths Skeleton
   
  // set the colors randomly every 240th frame
  setRandomColors1(240);
}
void updateAndDrawBox2D() {
  // if frameRate is sufficient, add a polygon and a circle with a random radius
  //noStroke();
  //fill(255,0,0);
  if (physics.particles.size() < NUM_PARTICLES) {
   //addParticle();
  }
  //physics.update();
  for (VerletParticle2D p : physics.particles) {
    //ellipse(p.x, p.y, 10, 5);
  }
   if(frameRate>30){
     //CustomShape shape2 = new CustomShape(kinectWidth/2, -50, random(2.5, 20),BodyType.DYNAMIC);
     //shape2.display();
   }
   if(millis() - time2 >= wait2){
    tick2 = !tick2;//if it is, do something
    time2 = millis();//also update the stored time
  }
  if(!tick2){
   if(random(1)<0.3){
    
     CustomShape shape1 = new CustomShape(kinectWidth/2, -50, -1,BodyType.DYNAMIC) ;
     polygons.add(shape1);
   }
   
  }
  else{
   
   if(random(1)<0.05){
      CustomShape shape2 = new CustomShape(random(40,width-40), -50, 20,BodyType.DYNAMIC);
     polygons.add(shape2);  
     
   }
   drawSkeleton();
   
  }
    //CustomShape shape1 = new CustomShape(kinectWidth/2, -50, -1,BodyType.DYNAMIC) ;
    //CustomShape shape2 = new CustomShape(kinectWidth/2, -50, random(2.5, 20),BodyType.DYNAMIC);
    //polygons.add(shape1);
    //polygons.add(shape2);
  
  // take one step in the box2d physics world
  box2d.step();
  
  // center and reScale from Kinect to custom dimensions
  translate(0, (height-kinectHeight*reScale)/2);
  scale(reScale);
 
  // display the person's polygon  
  noStroke();
  fill(0);
  gfx.polygon2D(poly1);
 
  // display all the shapes (circles, polygons)
  // go backwards to allow removal of shapes
  for (int i=polygons.size()-1; i>=0; i--) {
    CustomShape cs = polygons.get(i);
    // if the shape is off-screen remove it (see class for more info)
    
    
    if (cs.done()) {
      polygons.remove(i);
    // otherwise update (keep shape outside person) and display (circle or polygon)
    } else {
      cs.update();
      cs.display();
    }
  }
}
// sets the colors every nth frame
void setRandomColors1(int nthFrame) {
  if (frameCount % nthFrame == 0) {
    // turn a palette into a series of strings
    String[] paletteStrings = split(palettes1[int(random(palettes1.length))], ",");
    // turn strings into colors
    colorPalette1 = new color[paletteStrings.length];
    for (int i=0; i<paletteStrings.length; i++) {
      colorPalette1[i] = int(paletteStrings[i]);
    }
    // set background color to first color from palette
    bgColor = colorPalette1[0];
    // set blob color to second color from palette
    blobColor = colorPalette1[1];
    // set all shape colors randomly
    for (CustomShape cs: polygons) { cs.col = getRandomColor1(); }
  }
}
color getRandomColor1() {
  return colorPalette1[int(random(1, colorPalette1.length-1))];
}