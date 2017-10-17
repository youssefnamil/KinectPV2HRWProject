import processing.core.*; 
import processing.data.*; 
import processing.event.*; 
import processing.opengl.*; 

import blobDetection.*; 
import toxi.geom.*; 
import toxi.processing.*; 
import shiffman.box2d.*; 
import org.jbox2d.collision.shapes.*; 
import org.jbox2d.dynamics.joints.*; 
import org.jbox2d.common.*; 
import org.jbox2d.dynamics.*; 
import KinectPV2.KJoint; 
import KinectPV2.*; 
import toxi.geom.mesh2d.*; 
import toxi.physics2d.*; 
import toxi.physics2d.behaviors.*; 
import toxi.util.datatypes.*; 
import toxi.processing.*; 
import java.util.List; 
import java.util.Timer; 
import java.awt.Polygon; 
import java.util.List; 
import java.util.Arrays; 
import java.util.Collections; 

import java.util.HashMap; 
import java.util.ArrayList; 
import java.io.File; 
import java.io.BufferedReader; 
import java.io.PrintWriter; 
import java.io.InputStream; 
import java.io.OutputStream; 
import java.io.IOException; 

public class ESNA_v05 extends PApplet {


// import libraries


 // blobs
 // toxiclibs shapes and vectors
 // toxiclibs display
 // shiffman's jbox2d helper library
 // jbox2d

 // jbox2d
 // jbox2d












// this is a regular java import so we can use and extend the polygon class (see PolygonBlob)



//alternativ1
VerletPhysics2D physics1;
int NUM_PARTICLES = 150;


AttractionBehavior2D mouseAttractor;

Vec2D mousePos;
FloatRange radius;
Vec2D origin;
 ArrayList<Vec2D> mouseList;
 PolygonBlob1 poly1 = new PolygonBlob1();
 // ToxiclibsSupport for displaying polygons
ToxiclibsSupport gfx;
 PImage cam1 = createImage(640, 480, RGB);
 int blobColor;
 int[] colorPalette1;
  int[] colorPalette2;
 // the main PBox2D object in which all the physics-based stuff is happening
Box2DProcessing box2d;
// list to hold all the custom shapes (circles, polygons)
ArrayList<CustomShape> polygons = new ArrayList<CustomShape>();
public void addParticle() {
  VerletParticle2D p = new VerletParticle2D(Vec2D.randomVector().scale(5).addSelf(width / 2, 0));
  if(random(1)<0.05f){
   physics1.addParticle(p);
  }
  // add a negative attraction force field around the new particle
  //physics.addBehavior(new AttractionBehavior2D(p, 20, -1.2f, 0.01f));
}
 //
 // Chain und Physics
 



KinectPV2 kinect;


// declare BlobDetection object

BlobDetection theBlobDetection;

// declare custom PolygonBlob object (see class for more info)

PolygonBlob2 poly2 = new PolygonBlob2();



// PImage to hold incoming imagery and smaller one for blob detection

PImage cam, blobs;

// the kinect's dimensions to be used later on for calculations

int kinectWidth = 640;

int kinectHeight = 480;

// to center and rescale from 640x480 to higher custom resolutions

float reScale;
// An array of news headlines
String[] headlines = {
  "Warten Sie auf 8 Seconden f\u00fcr die zweite Alternativ", 
};
PFont f; // Global font variable
float x; // Horizontal location
int index = 0;


// background color

int bgColor;

// three color palettes (artifact from me storing many interesting color palettes as strings in an external data file ;-)

String[] palettes = {

  "-1117720,-13683658,-8410437,-9998215,-1849945,-5517090,-4250587,-14178341,-5804972,-3498634", 

  "-67879,-9633503,-8858441,-144382,-4996094,-16604779,-588031", 

  "-16711663,-13888933,-9029017,-5213092,-1787063,-11375744,-2167516,-15713402,-5389468,-2064585"

};
// three color palettes (artifact from me storingmany interesting color palettes as strings in an external data file ;-)
String[] palettes1 = {
  "-1117720,-13683658,-8410437,-9998215,-1849945,-5517090,-4250587,-14178341,-5804972,-3498634", 
  "-67879,-9633503,-8858441,-144382,-4996094,-16604779,-588031", 
  "-1978728,-724510,-15131349,-13932461,-4741770,-9232823,-3195858,-8989771,-2850983,-10314372"
};



// an array called flow of 2250 Particle objects (see Particle class)

Particle[] flow = new Particle[2250];

// global variables to influence the movement of all particles

float globalX, globalY;
//Chain 
Vec2D mouse;
Vec2D hand_left;
Vec2D hand_right;


// number of particles for string
int STRING_RES=100;
// number particles for ball
int BALL_RES=60;
// ball size
int BALL_RADIUS=80;

// squared snap distance for mouse selection
float SNAP_DIST = 20 * 20;

VerletPhysics2D physics;
VerletParticle2D selectedParticle;
long lStartTime;
int time;
int wait2=10000;
int wait = 20000;
PImage hmida; 
boolean tick;
boolean tick2;
long ltime ;
long time2;


public void setup() {

  // it's possible to customize this, for example 1920x1080

  
  // Alternativ1
  
  gfx = new ToxiclibsSupport(this);
  physics = new VerletPhysics2D();
  physics.setDrag(0.05f);
  physics.setWorldBounds(new Rect(0,0,width,height));
  
  // the NEW way to add gravity to the simulation, using behaviors
  physics.addBehavior(new GravityBehavior2D(new Vec2D(0, 0.15f)));
  radius = new BiasedFloatRange(30, 100, 30, 0.6f);
  origin = new Vec2D(width/2,height/2);
  initPhysics();
  // setup box2d, create world, set gravity
    box2d = new Box2DProcessing(this);
    box2d.createWorld();
    box2d.setGravity(0, -20);
    mouseList = new ArrayList<Vec2D>();
    //setRandomColors1(1);
    float gap = kinectWidth / 21;
    for (int i=0; i<20; i++)
    {
      drawString(gap * (i+1), 2, 10);
    }
  
  //
  
  
  f = createFont( "Arial", 16);
   
  // Initialize headline offscreen
  x = width;
  mouse= new Vec2D();
   hand_left = new Vec2D();
   hand_right= new Vec2D();
  // initialize Kinect

    kinect = new KinectPV2(this);
    kinect.enableDepthMaskImg(true);
    //kinect.setMirror(true);
    kinect.enableSkeleton3DMap(true);
    kinect.enableBodyTrackImg(true);
    kinect.enableSkeletonColorMap(true);
    kinect.enableColorImg(true);
    kinect.init();

    // calculate the reScale value

    // currently it's rescaled to fill the complete width (cuts of top-bottom)

    // it's also possible to fill the complete height (leaves empty sides)

    reScale = (float) width / kinectWidth;

    // create a smaller blob image for speed and efficiency

    blobs = createImage(kinectWidth/4, kinectHeight/4, RGB);

    // initialize blob detection object to the blob image dimensions

    theBlobDetection = new BlobDetection(blobs.width, blobs.height);
    theBlobDetection.setPosDiscrimination(true);

    theBlobDetection.setThreshold(0.5f);

    setupFlowfield();

  // time test
  //
       time = millis();
       time2 = millis();//store the current time
       
  smooth();
  strokeWeight(3);


}
// Alternativ1 function
public void drawString(float x, float size, int cards) {
  
  float gap = kinectHeight/cards;
  // anchor card
  CustomShape s1 = new CustomShape(x, -40, size, BodyType.DYNAMIC);
  polygons.add(s1);
  
  CustomShape last_shape = s1;
  CustomShape next_shape;
  for (int i=0; i<cards; i++)
  {
    float y = -20 + gap * (i+1);
    next_shape = new CustomShape(x, -20 + gap * (i+1), size, BodyType.DYNAMIC);
    DistanceJointDef jd = new DistanceJointDef();

    Vec2 c1 = last_shape.body.getWorldCenter();
    Vec2 c2 = next_shape.body.getWorldCenter();
  // offset the anchors so the cards hang vertically
    c1.y = c1.y + size / 5;
    c2.y = c2.y - size / 5;
    jd.initialize(last_shape.body, next_shape.body, c1, c2);
    jd.length = box2d.scalarPixelsToWorld(gap - 1);
    box2d.createJoint(jd);
    polygons.add(next_shape);
    last_shape = next_shape;
  }
}

 

public void draw() {
  // 1st update
  physics.update();
  
    background(bgColor);
    
    if(millis() - time >= wait){
    tick = !tick;//if it is, do something
    time = millis();//also update the stored time
  }
  if(!tick){  
  alternativ1();
   }
   else{
   
   alternativ2();
}
}



public void drawSkeleton(){
  
 // Get the 2D array data points from the Skeleton Color Map  
  ArrayList<KSkeleton> skeletonArray =  kinect.getSkeletonColorMap();

// Get the Skeleton data joints (X & Y only)

  for (int i = 0; i < skeletonArray.size(); i++) {
    KSkeleton skeleton = (KSkeleton) skeletonArray.get(i);
    if (skeleton.isTracked()) {
      KJoint[] joints = skeleton.getJoints();

      //printSkeleton(joints);

      int col  = skeleton.getIndexColor();
      fill(col);
      stroke(col);
      drawBody(joints);
     // points.add(new PVector(joints[HAND_LEFT].getX(),joints[HAND_LEFT].getY()));
     // points.add(new PVector(joints[HAND_RIGHT].getX(),joints[HAND_RIGHT].getY()));
      //draw different color for each hand state
     drawHandState(joints[HAND_LEFT]);
      drawHandState(joints[HAND_RIGHT]);
      mouseList.add(new Vec2D(joints[HAND_LEFT].getX(),joints[HAND_LEFT].getY()));
      mouseList.add(new Vec2D(joints[HAND_RIGHT].getX(),joints[HAND_RIGHT].getY()));
      //mouse = new Vec2D(joints[HAND_LEFT].getX(),joints[HAND_LEFT].getY()-60);
     mouse.x = joints[HAND_LEFT].getX();
     mouse.y=  joints[HAND_LEFT].getY();
      hand_left.x=joints[HAND_LEFT].getX();
      hand_left.y = joints[HAND_LEFT].getY();
      
      hand_right.x= joints[HAND_RIGHT].getX();
      hand_right.y= joints[HAND_RIGHT].getY();
    }
  }
}
public void drawSkeleton1(){
  
 // Get the 2D array data points from the Skeleton Color Map  
  ArrayList<KSkeleton> skeletonArray =  kinect.getSkeletonColorMap();

// Get the Skeleton data joints (X & Y only)

  for (int i = 0; i < skeletonArray.size(); i++) {
    KSkeleton skeleton = (KSkeleton) skeletonArray.get(i);
    if (skeleton.isTracked()) {
      KJoint[] joints = skeleton.getJoints();

      //printSkeleton(joints);

      int col  = skeleton.getIndexColor();
      fill(col);
      stroke(col);
      drawBody(joints);
      // points.add(new PVector(joints[HAND_LEFT].getX(),joints[HAND_LEFT].getY()));
      // points.add(new PVector(joints[HAND_RIGHT].getX(),joints[HAND_RIGHT].getY()));
      //draw different color for each hand state
      drawHandState(joints[HAND_LEFT]);
      drawHandState(joints[HAND_RIGHT]);
      mouseList.add(new Vec2D(joints[HAND_LEFT].getX(),joints[HAND_LEFT].getY()));
      mouseList.add(new Vec2D(joints[HAND_RIGHT].getX(),joints[HAND_RIGHT].getY()));
      //mouse = new Vec2D(joints[HAND_LEFT].getX(),joints[HAND_LEFT].getY()-60);
     mouse.x = joints[HAND_LEFT].getX();
     mouse.y=  joints[HAND_LEFT].getY();
      hand_left.x=joints[HAND_LEFT].getX()-235;
      hand_left.y = joints[HAND_LEFT].getY()-110;
      
      hand_right.x= joints[HAND_RIGHT].getX()-235;
      hand_right.y= joints[HAND_RIGHT].getY()-110;
    }
  }
}



// sets the colors every nth frame
public void alternativ1(){
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
public void updateAndDrawBox2D() {
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
   if(random(1)<0.3f){
    
     CustomShape shape1 = new CustomShape(kinectWidth/2, -50, -1,BodyType.DYNAMIC) ;
     polygons.add(shape1);
   }
   
  }
  else{
   
   if(random(1)<0.05f){
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
public void setRandomColors1(int nthFrame) {
  if (frameCount % nthFrame == 0) {
    // turn a palette into a series of strings
    String[] paletteStrings = split(palettes1[PApplet.parseInt(random(palettes1.length))], ",");
    // turn strings into colors
    colorPalette1 = new int[paletteStrings.length];
    for (int i=0; i<paletteStrings.length; i++) {
      colorPalette1[i] = PApplet.parseInt(paletteStrings[i]);
    }
    // set background color to first color from palette
    bgColor = colorPalette1[0];
    // set blob color to second color from palette
    blobColor = colorPalette1[1];
    // set all shape colors randomly
    for (CustomShape cs: polygons) { cs.col = getRandomColor1(); }
  }
}
public int getRandomColor1() {
  return colorPalette1[PApplet.parseInt(random(1, colorPalette1.length-1))];
}
public void alternativ2(){
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
public void setupFlowfield() {

  // set stroke weight (for particle display) to 2.5

  strokeWeight(2.5f);

  // initialize all particles in the flow

  for(int i=0; i<flow.length; i++) {

    flow[i] = new Particle(i/10000.0f);

  }

  // set all colors randomly now

  setRandomColors(1);
  

}
public void drawFlowfield() {

  // center and reScale from Kinect to custom dimensions

  translate(0, (height-kinectHeight*reScale)/2);

  scale(reScale);

  // set global variables that influence the particle flow's movement

  globalX = noise(frameCount * 0.01f) * width/2 + width/4;

  globalY = noise(frameCount * 0.005f + 5) * height;

  // update and display all particles in the flow

  for (Particle p : flow) {

    p.updateAndDisplay();

  }

  // set the colors randomly every 240th frame

  setRandomColors(240);

}
public void setRandomColors(int nthFrame) {

  if (frameCount % nthFrame == 0) {

    // turn a palette into a series of strings

    String[] paletteStrings = split(palettes[PApplet.parseInt(random(palettes.length))], ",");

    // turn strings into colors

     colorPalette2 = new int[paletteStrings.length];

    for (int i=0; i<paletteStrings.length; i++) {

      colorPalette2[i] = PApplet.parseInt(paletteStrings[i]);

    }

    // set background color to first color from palette

    bgColor = colorPalette2[0];

    // set all particle colors randomly to color from palette (excluding first aka background color)

    for (int i=0; i<flow.length; i++) {

      flow[i].col = colorPalette2[PApplet.parseInt(random(1, colorPalette2.length))];

    }

  }

}



public void spiel() {
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
public void Spiel1(){
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
  public void Dragged1(){
  if (selectedParticle!=null) {
    // move selected particle to new mouse pos
    selectedParticle.set(hand_left.x,hand_left.y);
  }
  }
    
      
   


public void Dragged() {
  if (selectedParticle!=null) {
    // move selected particle to new mouse pos
    selectedParticle.set(hand_right.x,hand_right.y);
  }
}

public void Released() {
  // unlock the selected particle
  if (selectedParticle!=null) {
    selectedParticle.unlock();
    selectedParticle=null;
  }
}
// The Nature of Code
// <http://www.shiffman.net/teaching/nature>
// Spring 2010
// Toxiclibs example

// A soft pendulum (series of connected springs)

class Chain {

  // Chain properties
  float totalLength;  // How long
  int numPoints;      // How many points
  float strength;     // Strength of springs
  float radius;       // Radius of ball at tail

  // Let's keep an extra reference to the tail particle
  // This is just the last particle in the ArrayList
  VerletParticle2D tail;

  // Some variables for mouse dragging
  PVector offset = new PVector();
  boolean dragged = false;

  // Chain constructor
  Chain(float l, int n, float r, float s) {

    totalLength = l;
    numPoints = n;
    radius = r;
    strength = s;

    float len = totalLength / numPoints;

    // Here is the real work, go through and add particles to the chain itself
    for(int i=0; i < numPoints; i++) {
      // Make a new particle with an initial starting location
      VerletParticle2D particle=new VerletParticle2D(width/2,i*len);

      // Redundancy, we put the particles both in physics and in our own ArrayList
      physics1.addParticle(particle);

      // Connect the particles with a Spring (except for the head)
      if (i>0) {
        VerletParticle2D previous = physics1.particles.get(i-1);
        VerletSpring2D spring=new VerletSpring2D(particle,previous,len,strength);
        // Add the spring to the physics world
        physics1.addSpring(spring);
      }
    }

    // Keep the top fixed
    VerletParticle2D head=physics1.particles.get(0);
    head.lock();

    // Store reference to the tail
    tail = physics1.particles.get(numPoints-1);
  }

  // Check if a point is within the ball at the end of the chain
  // If so, set dragged = true;
  public void contains(int x, int y) {
    float d = dist(x,y,tail.x,tail.y);
    if (d < radius) {
      offset.x = tail.x - x;
      offset.y = tail.y - y;
      tail.lock();
      dragged = true;
    }
  }

  // Release the ball
  public void release() {
    tail.unlock();
    dragged = false;
  }

  // Update tail location if being dragged
  public void updateTail(int x, int y) {
    if (dragged) {
      tail.set(x+offset.x,y+offset.y);
    }
  }

  // Draw the chain
  public void display() {
    // Draw line connecting all points
    for(int i=0; i < physics1.particles.size()-1; i++) {
      VerletParticle2D p1 = physics1.particles.get(i);
      VerletParticle2D p2 = physics1.particles.get(i+1);
      stroke(0);
      line(p1.x,p1.y,p2.x,p2.y);
    }

    // Draw a ball at the tail
    stroke(0);
    fill(175);
    ellipse(tail.x,tail.y,radius*2,radius*2);
  }
}




class CustomShape {
  // to hold the box2d body
  Body body;
  // to hold the Toxiclibs polygon shape
  Polygon2D toxiPoly;
  // custom color for each shape
  int col;
  // radius (also used to distinguish between circles and polygons in this combi-class
  float r;

  CustomShape(float x, float y, float r, BodyType type) {
    this.r = r;
    // create a body (polygon or circle based on the r)
    makeBody(x, y, type);
    // get a random color
    //col = getRandomColor();
    col = color(0);
  }

  public void makeBody(float x, float y, BodyType type) 
  {
    // define a dynamic body positioned at xy in box2d world coordinates,
    // create it and set the initial values for this box2d body's speed and angle
    BodyDef bd = new BodyDef();
    bd.type = type;
    bd.position.set(box2d.coordPixelsToWorld(new Vec2(x, y)));
    body = box2d.createBody(bd);
    body.setLinearVelocity(new Vec2(random(-2, 8), random(2, 8)));
    body.setAngularVelocity(random(-5, 5));

    // box2d polygon shape
    //PolygonShape sd = new PolygonShape();
    /* toxiPoly = new Polygon2D(Arrays.asList(new Vec2D(-r, r*1.5), 
     new Vec2D(r, r*1.5), 
     new Vec2D(r, -r*1.5), 
     new Vec2D(-r, -r*1.5)));*/

    if (r == -1) 
    {
      // box2d polygon shape
      PolygonShape sd = new PolygonShape();
      // toxiclibs polygon creator (triangle, square, etc)
      toxiPoly = new Circle(random(5, 20)).toPolygon2D(PApplet.parseInt(random(3, 6)));
      // place the toxiclibs polygon's vertices into a vec2d array
      Vec2[] vertices = new Vec2[toxiPoly.getNumPoints()];

      for (int i=0; i<vertices.length; i++) 
      {
        Vec2D v = toxiPoly.vertices.get(i);
        vertices[i] = box2d.vectorPixelsToWorld(new Vec2(v.x, v.y));
      }
      // put the vertices into the box2d shape
      sd.set(vertices, vertices.length);
      // create the fixture from the shape (deflect things based on the actual polygon shape)
      body.createFixture(sd, 1);
    }

    else 
    {
      // box2d circle shape of radius r
      CircleShape cs = new CircleShape();
      cs.m_radius = box2d.scalarPixelsToWorld(r);
      // tweak the circle's fixture def a little bit
      FixtureDef fd = new FixtureDef();
      fd.shape = cs;
      fd.density = 1;
      fd.friction = 0.001f;
      fd.restitution = 0.9f;
      // create the fixture from the shape's fixture def (deflect things based on the actual circle shape)
      body.createFixture(fd);
    }
  }




  // method to loosely move shapes outside a person's polygon
  // (alternatively you could allow or remove shapes inside a person's polygon)
  public void update() 
  {
    // get the screen position from this shape (circle of polygon)
    Vec2 posScreen = box2d.getBodyPixelCoord(body);
    // turn it into a toxiclibs Vec2D
    Vec2D toxiScreen = new Vec2D(posScreen.x, posScreen.y);
    // check if this shape's position is inside the person's polygon
    boolean inBody = poly1.containsPoint(toxiScreen);
    // if a shape is inside the person
    if (inBody) {
      // find the closest point on the polygon to the current position
      Vec2D closestPoint = toxiScreen;
      float closestDistance = 9999999;
      for (Vec2D v : poly1.vertices) 
      {
        float distance = v.distanceTo(toxiScreen);
        if (distance < closestDistance) 
        {
          closestDistance = distance;
          closestPoint = v;
        }
      }
      // create a box2d position from the closest point on the polygon
      Vec2 contourPos = new Vec2(closestPoint.x, closestPoint.y);
      Vec2 posWorld = box2d.coordPixelsToWorld(contourPos);
      float angle = body.getAngle();
      // set the box2d body's position of this CustomShape to the new position (use the current angle)
      body.setTransform(posWorld, angle);
    }
  }

  // display the customShape
  public void display() {
    // get the pixel coordinates of the body
    Vec2 pos = box2d.getBodyPixelCoord(body);
    pushMatrix();
    // translate to the position
    translate(pos.x, pos.y);
    noStroke();
    // use the shape's custom color
    fill(col);

    if (r == -1) {
      // rotate by the body's angle
      float a = body.getAngle();
      rotate(-a); // minus!
      gfx.polygon2D(toxiPoly);
    } 
    else {
      
      ellipse(0, 0, r*2, r*2);
    }

    popMatrix();
  }

  // if the shape moves off-screen, destroy the box2d body (important!)
  // and return true (which will lead to the removal of this CustomShape object)
  public boolean done() {
    Vec2 posScreen = box2d.getBodyPixelCoord(body);
    boolean offscreen = posScreen.y > height;
    if (offscreen) {
      box2d.destroyBody(body);
      return true;
    }
    return false;
  }
  
  //void edge(){
  //  Vec2 posScreen = box2d.getBodyPixelCoord(body);
  //  if(posScreen.y>height)
  //     circle =new CustomShape(kinectWidth/2, -50, random(2.5, 20),BodyType.DYNAMIC);
  //   polygons.add(circle);
  //}
}

// a basic noise-based moving particle
class Particle {
  // unique id, (previous) position, speed
  float id, x, y, xp, yp, s, d;
  int col; // color
  
  Particle(float id) {
    this.id = id;
    s = random(2, 6); // speed
  }
  
  public void updateAndDisplay() {
    // let it flow, end with a new x and y position
    id += 0.01f;
    d = (noise(id, x/globalY, y/globalY)-0.5f)*globalX;
    x += cos(radians(d))*s;
    y += sin(radians(d))*s;

    // constrain to boundaries
    if (x<-10) x=xp=kinectWidth+10;
    if (x>kinectWidth+10) x=xp=-10;
    if (y<-10) y=yp=kinectHeight+10;
    if (y>kinectHeight+10) y=yp=-10;

    // if there is a polygon (more than 0 points)
    if (poly2.npoints > 0) {
      // if this particle is outside the polygon
      if (!poly2.contains(x, y)) {
        // while it is outside the polygon
        while(!poly2.contains(x, y)) {
          // randomize x and y
          x = random(kinectWidth);
          y = random(kinectHeight);
        }
        // set previous x and y, to this x and y
        xp=x;
        yp=y;
      }
    }
    
    // individual particle color
    stroke(col);
    // line from previous to current position
    line(xp, yp, x, y);
    
    // set previous to current position
    xp=x;
    yp=y;
  }
}
public void initPhysics() {
  physics1=new VerletPhysics2D();
  // set screen bounds as bounds for physics sim
  physics1.setWorldBounds(new Rect(0,0,width,height));
  // add gravity along positive Y axis
  physics1.addBehavior(new GravityBehavior2D(new Vec2D(0,0.1f)));
  // compute spacing for string particles
  float delta=(float)width/(STRING_RES-1);
  for(int i=0; i<STRING_RES; i++) {
    // create particles along X axis
    VerletParticle2D p=new VerletParticle2D(i*delta,height/2);
    physics1.addParticle(p);
    // define a repulsion field around each particle
    // this is used to push the ball away
    physics1.addBehavior(new AttractionBehavior2D(p,delta*1.5f,-20));
    // connect each particle to its previous neighbour
    if (i>0) {
      VerletParticle2D q=physics1.particles.get(i-1);
      VerletSpring2D s=new VerletSpring2D(p,q,delta*0.5f,0.1f);
      physics1.addSpring(s);
    }
  }
  // lock 1st & last particles
  physics1.particles.get(0).lock();
  physics1.particles.get(physics1.particles.size()-1).lock();
  
  // create ball
  // first create a particle as the ball centre
  VerletParticle2D c=new VerletParticle2D(width/2,100);
  physics1.addParticle(c);
  // list to store all ball perimeter particles
  List<VerletParticle2D> cparts=new ArrayList<VerletParticle2D>();
  for(int i=0; i<BALL_RES; i++) {
    // create a rotation vector, scale it to the radius and move relative to ball center
    Vec2D pos=Vec2D.fromTheta(i*TWO_PI/BALL_RES).scaleSelf(BALL_RADIUS).addSelf(c);
    // create particle and add to lists
    VerletParticle2D p = new VerletParticle2D(pos);
    cparts.add(p);
    physics1.addParticle(p);
    // connect to ball center for extra stability
    physics1.addSpring(new VerletSpring2D(c,p,BALL_RADIUS,0.01f));
    // also connect all perimeter particles sequentially
    if (i>0) {
      VerletParticle2D q=cparts.get(i-1);
      physics1.addSpring(new VerletSpring2D(p,q,p.distanceTo(q),1));
   }
  }
  // finally close ball perimeter by connecting first & last particle
  VerletParticle2D p=cparts.get(0);
  VerletParticle2D q=cparts.get(BALL_RES-1);
  physics1.addSpring(new VerletSpring2D(p,q,p.distanceTo(q),1));
}


// an extended polygon class quite similar to the earlier PolygonBlob class (but extending Toxiclibs' Polygon2D class instead)
// The main difference is that this one is able to create (and destroy) a box2d body from it's own shape
class PolygonBlob1 extends Polygon2D {
  // to hold the box2d body
  Body body;
 
  // the createPolygon() method is nearly identical to the one presented earlier
  // see the Kinect Flow Example for a more detailed description of this method (again, feel free to improve it)
  public void createPolygon() {
    ArrayList<ArrayList<PVector>> contours = new ArrayList<ArrayList<PVector>>();
    int selectedContour = 0;
    int selectedPoint = 0;
 
    // create contours from blobs
    for (int n=0 ; n<theBlobDetection.getBlobNb(); n++) {
      Blob b = theBlobDetection.getBlob(n);
      if (b != null && b.getEdgeNb() > 100) {
        ArrayList<PVector> contour = new ArrayList<PVector>();
        for (int m=0; m<b.getEdgeNb(); m++) {
          EdgeVertex eA = b.getEdgeVertexA(m);
          EdgeVertex eB = b.getEdgeVertexB(m);
          if (eA != null && eB != null) {
            EdgeVertex fn = b.getEdgeVertexA((m+1) % b.getEdgeNb());
            EdgeVertex fp = b.getEdgeVertexA((max(0, m-1)));
            float dn = dist(eA.x*kinectWidth, eA.y*kinectHeight, fn.x*kinectWidth, fn.y*kinectHeight);
            float dp = dist(eA.x*kinectWidth, eA.y*kinectHeight, fp.x*kinectWidth, fp.y*kinectHeight);
            if (dn > 15 || dp > 15) {
              if (contour.size() > 0) {
                contour.add(new PVector(eB.x*kinectWidth, eB.y*kinectHeight));
                contours.add(contour);
                contour = new ArrayList<PVector>();
              } else {
                contour.add(new PVector(eA.x*kinectWidth, eA.y*kinectHeight));
              }
            } else {
              contour.add(new PVector(eA.x*kinectWidth, eA.y*kinectHeight));
            }
          }
        }
      }
    }
    
    while (contours.size() > 0) {
      
      // find next contour
      float distance = 999999999;
      if (getNumPoints() > 0) {
        Vec2D vecLastPoint = vertices.get(getNumPoints()-1);
        PVector lastPoint = new PVector(vecLastPoint.x, vecLastPoint.y);
        for (int i=0; i<contours.size(); i++) {
          ArrayList<PVector> c = contours.get(i);
          PVector fp = c.get(0);
          PVector lp = c.get(c.size()-1);
          if (fp.dist(lastPoint) < distance) { 
            distance = fp.dist(lastPoint); 
            selectedContour = i; 
            selectedPoint = 0;
          }
          if (lp.dist(lastPoint) < distance) { 
            distance = lp.dist(lastPoint); 
            selectedContour = i; 
            selectedPoint = 1;
          }
        }
      } else {
        PVector closestPoint = new PVector(width, height);
        for (int i=0; i<contours.size(); i++) {
          ArrayList<PVector> c = contours.get(i);
          PVector fp = c.get(0);
          PVector lp = c.get(c.size()-1);
          if (fp.y > kinectHeight-5 && fp.x < closestPoint.x) { 
            closestPoint = fp; 
            selectedContour = i; 
            selectedPoint = 0;
          }
          if (lp.y > kinectHeight-5 && lp.x < closestPoint.y) { 
            closestPoint = lp; 
            selectedContour = i; 
            selectedPoint = 1;
          }
        }
      }
 
      // add contour to polygon
      ArrayList<PVector> contour = contours.get(selectedContour);
      if (selectedPoint > 0) { Collections.reverse(contour); }
      for (PVector p : contour) {
        add(new Vec2D(p.x, p.y));
      }
      contours.remove(selectedContour);
    }
  }
 
  // creates a shape-deflecting physics chain in the box2d world from this polygon
  public void createBody() {
    // for stability the body is always created (and later destroyed)
    BodyDef bd = new BodyDef();
    // setting the position of the body in center
   // bd.position.set(CENTER);
    body = box2d.createBody(bd);
    // if there are more than 0 points (aka a person on screen)...
    if (getNumPoints() > 0) {
      // create a vec2d array of vertices in box2d world coordinates from this polygon
      Vec2[] verts = new Vec2[getNumPoints()/2];
      for (int i=0; i<getNumPoints()/2; i++) {
        Vec2D v = vertices.get(i*2);
        verts[i] = box2d.coordPixelsToWorld(v.x, v.y);
      }
      // create a chain from the array of vertices
      ChainShape chain = new ChainShape();
      chain.createChain(verts, verts.length);
      // create fixture in body from the chain (this makes it actually deflect other shapes)
      //body.createFixture(chain, 1);
       FixtureDef fd= new FixtureDef();
      //fd.shape= chain;
      //fd.density =1;
      //fd.friction =0.3;
      body.createFixture(chain,1);
      
    }
  }
  // destroy the box2d body (important!)
  public void destroyBody() {
    box2d.destroyBody(body);
  }
}

// an extended polygon class with my own customized createPolygon() method (feel free to improve!)
class PolygonBlob2 extends Polygon {

  // took me some time to make this method fully self-sufficient
  // now it works quite well in creating a correct polygon from a person's blob
  // of course many thanks to v3ga, because the library already does a lot of the work
  public void createPolygon() {
    // an arrayList... of arrayLists... of PVectors
    // the arrayLists of PVectors are basically the person's contours (almost but not completely in a polygon-correct order)
    ArrayList<ArrayList<PVector>> contours = new ArrayList<ArrayList<PVector>>();
    // helpful variables to keep track of the selected contour and point (start/end point)
    int selectedContour = 0;
    int selectedPoint = 0;

    // create contours from blobs
    // go over all the detected blobs
    for (int n=0 ; n<theBlobDetection.getBlobNb(); n++) {
      Blob b = theBlobDetection.getBlob(n);
      // for each substantial blob...
      if (b != null && b.getEdgeNb() > 100) {
        // create a new contour arrayList of PVectors
        ArrayList<PVector> contour = new ArrayList<PVector>();
        // go over all the edges in the blob
        for (int m=0; m<b.getEdgeNb(); m++) {
          // get the edgeVertices of the edge
          EdgeVertex eA = b.getEdgeVertexA(m);
          EdgeVertex eB = b.getEdgeVertexB(m);
          // if both ain't null...
          if (eA != null && eB != null) {
            // get next and previous edgeVertexA
            EdgeVertex fn = b.getEdgeVertexA((m+1) % b.getEdgeNb());
            EdgeVertex fp = b.getEdgeVertexA((max(0, m-1)));
            // calculate distance between vertexA and next and previous edgeVertexA respectively
            // positions are multiplied by kinect dimensions because the blob library returns normalized values
            float dn = dist(eA.x*kinectWidth, eA.y*kinectHeight, fn.x*kinectWidth, fn.y*kinectHeight);
            float dp = dist(eA.x*kinectWidth, eA.y*kinectHeight, fp.x*kinectWidth, fp.y*kinectHeight);
            // if either distance is bigger than 15
            if (dn > 15 || dp > 15) {
              // if the current contour size is bigger than zero
              if (contour.size() > 0) {
                // add final point
                contour.add(new PVector(eB.x*kinectWidth, eB.y*kinectHeight));
                // add current contour to the arrayList
                contours.add(contour);
                // start a new contour arrayList
                contour = new ArrayList<PVector>();
              // if the current contour size is 0 (aka it's a new list)
              } else {
                // add the point to the list
                contour.add(new PVector(eA.x*kinectWidth, eA.y*kinectHeight));
              }
            // if both distance are smaller than 15 (aka the points are close)  
            } else {
              // add the point to the list
              contour.add(new PVector(eA.x*kinectWidth, eA.y*kinectHeight));
            }
          }
        }
      }
    }
    
    // at this point in the code we have a list of contours (aka an arrayList of arrayLists of PVectors)
    // now we need to sort those contours into a correct polygon. To do this we need two things:
    // 1. The correct order of contours
    // 2. The correct direction of each contour

    // as long as there are contours left...    
    while (contours.size() > 0) {
      
      // find next contour
      float distance = 999999999;
      // if there are already points in the polygon
      if (npoints > 0) {
        // use the polygon's last point as a starting point
        PVector lastPoint = new PVector(xpoints[npoints-1], ypoints[npoints-1]);
        // go over all contours
        for (int i=0; i<contours.size(); i++) {
          ArrayList<PVector> c = contours.get(i);
          // get the contour's first point
          PVector fp = c.get(0);
          // get the contour's last point
          PVector lp = c.get(c.size()-1);
          // if the distance between the current contour's first point and the polygon's last point is smaller than distance
          if (fp.dist(lastPoint) < distance) {
            // set distance to this distance
            distance = fp.dist(lastPoint);
            // set this as the selected contour
            selectedContour = i;
            // set selectedPoint to 0 (which signals first point)
            selectedPoint = 0;
          }
          // if the distance between the current contour's last point and the polygon's last point is smaller than distance
          if (lp.dist(lastPoint) < distance) {
            // set distance to this distance
            distance = lp.dist(lastPoint);
            // set this as the selected contour
            selectedContour = i;
            // set selectedPoint to 1 (which signals last point)
            selectedPoint = 1;
          }
        }
      // if the polygon is still empty
      } else {
        // use a starting point in the lower-right
        PVector closestPoint = new PVector(width, height);
        // go over all contours
        for (int i=0; i<contours.size(); i++) {
          ArrayList<PVector> c = contours.get(i);
          // get the contour's first point
          PVector fp = c.get(0);
          // get the contour's last point
          PVector lp = c.get(c.size()-1);
          // if the first point is in the lowest 5 pixels of the (kinect) screen and more to the left than the current closestPoint
          if (fp.y > kinectHeight-5 && fp.x < closestPoint.x) {
            // set closestPoint to first point
            closestPoint = fp;
            // set this as the selected contour
            selectedContour = i;
            // set selectedPoint to 0 (which signals first point)
            selectedPoint = 0;
          }
          // if the last point is in the lowest 5 pixels of the (kinect) screen and more to the left than the current closestPoint
          if (lp.y > kinectHeight-5 && lp.x < closestPoint.y) {
            // set closestPoint to last point
            closestPoint = lp;
            // set this as the selected contour
            selectedContour = i;
            // set selectedPoint to 1 (which signals last point)
            selectedPoint = 1;
          }
        }
      }

      // add contour to polygon
      ArrayList<PVector> contour = contours.get(selectedContour);
      // if selectedPoint is bigger than zero (aka last point) then reverse the arrayList of points
      if (selectedPoint > 0) { java.util.Collections.reverse(contour); }
      // add all the points in the contour to the polygon
      for (PVector p : contour) {
        addPoint(PApplet.parseInt(p.x), PApplet.parseInt(p.y));
      }
      // remove this contour from the list of contours
      contours.remove(selectedContour);
      // the while loop above makes all of this code loop until the number of contours is zero
      // at that time all the points in all the contours have been added to the polygon... in the correct order (hopefully)
    }
  }
}
// Create constants variables for easier usage of joints

// * Skeleton joints

// Base
int SPINE_BASE = KinectPV2.JointType_SpineBase; // #0
int SPINE_MID = KinectPV2.JointType_SpineMid; // #1

// Head
int NECK = KinectPV2.JointType_Neck; // #2
int HEAD = KinectPV2.JointType_Head; // #3

// Left arm
int SHOULDER_LEFT = KinectPV2.JointType_ShoulderLeft; // #4
int ELBOW_LEFT = KinectPV2.JointType_ElbowLeft; // #5
int WRIST_LEFT = KinectPV2.JointType_WristLeft; // #6
int HAND_LEFT = KinectPV2.JointType_HandLeft; // #7

// Right arm
int SHOULDER_RIGHT = KinectPV2.JointType_ShoulderRight; // #8
int ELBOW_RIGHT = KinectPV2.JointType_ElbowRight; // #9
int WRIST_RIGHT = KinectPV2.JointType_WristRight; // #10
int HAND_RIGHT = KinectPV2.JointType_HandRight; // #11

// Left leg
int HIP_LEFT = KinectPV2.JointType_HipLeft; // #12
int KNEE_LEFT = KinectPV2.JointType_KneeLeft; // #13
int ANKLE_LEFT = KinectPV2.JointType_AnkleLeft; // #14
int FOOT_LEFT = KinectPV2.JointType_FootLeft; // #15

//// Right Leg
int HIP_RIGHT = KinectPV2.JointType_HipRight; // #16
int KNEE_RIGHT = KinectPV2.JointType_KneeRight; // #17
int ANKLE_RIGHT = KinectPV2.JointType_AnkleRight; // #18
int FOOT_RIGHT = KinectPV2.JointType_FootRight; // #19

// Bonus
int SPINE_SHOULDER = KinectPV2.JointType_SpineShoulder; // #20

// Left Hand details
int HAND_TIP_LEFT = KinectPV2.JointType_HandTipLeft; // #21
int THUMB_LEFT = KinectPV2.JointType_ThumbLeft; // #22

// Right Hand details
int HAND_TIP_RIGHT = KinectPV2.JointType_HandTipRight; // #23
int THUMB_RIGHT = KinectPV2.JointType_ThumbRight; // #24

// * Skeleton tracking state
int SKELETON_NOT_TRACKED = KinectPV2.TrackingState_NotTracked; // 0
int SKELETON_INFERRED = KinectPV2.TrackingState_Inferred; // 1
int SKELETON_TRACKED = KinectPV2.TrackingState_Tracked; // 2

// * Handstate
int HAND_UNKNOWN = KinectPV2.HandState_Unknown; // 0
int HAND_NOT_TRACKED = KinectPV2.HandState_NotTracked; // 1
int HAND_OPEN = KinectPV2.HandState_Open; // 2
int HAND_CLOSED = KinectPV2.HandState_Closed; // 3
int HAND_LASSO = KinectPV2.HandState_Lasso; // 4
//DRAW BODY
public void drawBody(KJoint[] joints) {
  drawBone(joints, HEAD, NECK);
  drawBone(joints, NECK, SPINE_SHOULDER);
  drawBone(joints, SPINE_SHOULDER, SPINE_MID);
  drawBone(joints, SPINE_MID, SPINE_BASE);
  drawBone(joints, SPINE_SHOULDER, SHOULDER_RIGHT);
  drawBone(joints, SPINE_SHOULDER, SHOULDER_LEFT);
  drawBone(joints, SPINE_BASE, HIP_RIGHT);
  drawBone(joints, SPINE_BASE, HIP_LEFT);

  // Right Arm
  drawBone(joints, SHOULDER_RIGHT, ELBOW_RIGHT);
  drawBone(joints, ELBOW_RIGHT, WRIST_RIGHT);
  drawBone(joints, WRIST_RIGHT, HAND_RIGHT);
  drawBone(joints, HAND_RIGHT, HAND_TIP_RIGHT);
  drawBone(joints, WRIST_RIGHT, THUMB_RIGHT);

  // Left Arm
  drawBone(joints, SHOULDER_LEFT, ELBOW_LEFT);
  drawBone(joints, ELBOW_LEFT, WRIST_LEFT);
  drawBone(joints, WRIST_LEFT, HAND_LEFT);
  drawBone(joints, HAND_LEFT, HAND_TIP_LEFT);
  drawBone(joints, WRIST_LEFT, THUMB_LEFT);

  // Right Leg
  drawBone(joints, HIP_RIGHT, KNEE_RIGHT);
  drawBone(joints, KNEE_RIGHT, ANKLE_RIGHT);
  drawBone(joints, ANKLE_RIGHT, FOOT_RIGHT);

  //// Left Leg
  drawBone(joints, HIP_LEFT, KNEE_LEFT);
  drawBone(joints, KNEE_LEFT, ANKLE_LEFT);
  drawBone(joints, ANKLE_LEFT, FOOT_LEFT);

  drawJoint(joints, HAND_TIP_LEFT);
  drawJoint(joints, HAND_TIP_RIGHT);
  drawJoint(joints, FOOT_LEFT);
  drawJoint(joints, FOOT_RIGHT);

  drawJoint(joints, THUMB_LEFT);
  drawJoint(joints, THUMB_RIGHT);

  drawJoint(joints, HEAD);
}

//draw joint
public void drawJoint(KJoint[] joints, int jointType) {
  pushMatrix();
  translate(joints[jointType].getX()-235, joints[jointType].getY()-110);
  ellipse(0, 0, 25, 25);
  popMatrix();
}

//draw bone
public void drawBone(KJoint[] joints, int jointType1, int jointType2) {
  pushMatrix();
  translate(joints[jointType1].getX(), joints[jointType1].getY(), joints[jointType1].getZ());
  //ellipse(0, 0, 25, 25);
  popMatrix();
  strokeWeight(10);
  line(joints[jointType1].getX()-235, joints[jointType1].getY()-110, joints[jointType1].getZ(), joints[jointType2].getX()-235, joints[jointType2].getY()-110, joints[jointType2].getZ());
}

//draw hand state
public void drawHandState(KJoint joint) {
  // mouseList.add(new Vec2D(joint.getX(),joint.getY()));
  noStroke();
  handState(joint.getState());
  pushMatrix();
  translate(joint.getX()-235, joint.getY()-110, joint.getZ());
 
      //mouseList.add(new Vec2D(joint.getX(),joint.getY()));
  //ellipse(0, 0, 70, 70);
  popMatrix();
}

/*
Different hand state
 KinectPV2.HandState_Open
 KinectPV2.HandState_Closed
 KinectPV2.HandState_Lasso
 KinectPV2.HandState_NotTracked
 */
public void handState(int handState) {
  switch(handState) {
  case KinectPV2.HandState_Open:
    fill(0, 255, 0);
    break;
  case KinectPV2.HandState_Closed:
    fill(255, 0, 0);
    break;
  case KinectPV2.HandState_Lasso:
    fill(0, 0, 255);
    break;
  case KinectPV2.HandState_NotTracked:
    fill(255, 255, 255);
    break;
  }
}

public float getZJoint(KJoint[] joints3D, int jointType) {
  //float zpos = joints3D[jointType].getZ();
/* Convert from 0.5 and 4.5 because 0.5 meter is the minimum distance 
for the Kinect to get data and 4.5 meters is the maximum distance */
  float convertZ = map(joints3D[jointType].getZ(), 0.57f, 4.5f, 0, 100);
  return convertZ;
}

/* 
  It also happens that the value of the Z value from
  the getSkeleton3d() function is between 0.5 and 4.5 (magic, no ?)
*/
  public void settings() {  size(1440, 900, P3D);  smooth(); }
  static public void main(String[] passedArgs) {
    String[] appletArgs = new String[] { "--present", "--window-color=#666666", "--stop-color=#cccccc", "ESNA_v05" };
    if (passedArgs != null) {
      PApplet.main(concat(appletArgs, passedArgs));
    } else {
      PApplet.main(appletArgs);
    }
  }
}
