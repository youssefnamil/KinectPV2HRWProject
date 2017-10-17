
// import libraries


import blobDetection.*; // blobs
import toxi.geom.*; // toxiclibs shapes and vectors
import toxi.processing.*; // toxiclibs display
import shiffman.box2d.*; // shiffman's jbox2d helper library
import org.jbox2d.collision.shapes.*; // jbox2d
import org.jbox2d.dynamics.joints.*;
import org.jbox2d.common.*; // jbox2d
import org.jbox2d.dynamics.*; // jbox2d
import KinectPV2.KJoint;
import KinectPV2.*;
import toxi.geom.mesh2d.*;
import toxi.physics2d.*;
import toxi.physics2d.behaviors.*;
import toxi.util.datatypes.*;
import toxi.processing.*;

import java.util.List;
import java.util.Timer;


// this is a regular java import so we can use and extend the polygon class (see PolygonBlob)

import java.awt.Polygon;

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
 color blobColor;
 color[] colorPalette1;
  color[] colorPalette2;
 // the main PBox2D object in which all the physics-based stuff is happening
Box2DProcessing box2d;
// list to hold all the custom shapes (circles, polygons)
ArrayList<CustomShape> polygons = new ArrayList<CustomShape>();
void addParticle() {
  VerletParticle2D p = new VerletParticle2D(Vec2D.randomVector().scale(5).addSelf(width / 2, 0));
  if(random(1)<0.05){
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
  "Warten Sie auf 8 Seconden für die zweite Alternativ", 
};
PFont f; // Global font variable
float x; // Horizontal location
int index = 0;


// background color

color bgColor;

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


void setup() {

  // it's possible to customize this, for example 1920x1080

  size(1440, 900, P3D);
  // Alternativ1
  smooth();
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
void drawString(float x, float size, int cards) {
  
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

 

void draw() {
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



void drawSkeleton(){
  
 // Get the 2D array data points from the Skeleton Color Map  
  ArrayList<KSkeleton> skeletonArray =  kinect.getSkeletonColorMap();

// Get the Skeleton data joints (X & Y only)

  for (int i = 0; i < skeletonArray.size(); i++) {
    KSkeleton skeleton = (KSkeleton) skeletonArray.get(i);
    if (skeleton.isTracked()) {
      KJoint[] joints = skeleton.getJoints();

      //printSkeleton(joints);

      color col  = skeleton.getIndexColor();
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
void drawSkeleton1(){
  
 // Get the 2D array data points from the Skeleton Color Map  
  ArrayList<KSkeleton> skeletonArray =  kinect.getSkeletonColorMap();

// Get the Skeleton data joints (X & Y only)

  for (int i = 0; i < skeletonArray.size(); i++) {
    KSkeleton skeleton = (KSkeleton) skeletonArray.get(i);
    if (skeleton.isTracked()) {
      KJoint[] joints = skeleton.getJoints();

      //printSkeleton(joints);

      color col  = skeleton.getIndexColor();
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