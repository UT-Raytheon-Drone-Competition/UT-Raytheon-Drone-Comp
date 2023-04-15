PVector coord = new PVector(0,0);
PVector vel = new PVector(0, 0);
color c = color(211, 151, 19);

//ArrayList<UGV> ugvs;
Flock flock;
ArrayList<UGV> actives;
Drone drone;
UGV finish;

void setup(){
  frameRate(4);
  size(800, 500);
  rectMode(CENTER);
  flock = new Flock(6);
  finish = new UGV(new PVector(width-100, height/2), new PVector(0,0), color(255, 255,255));
  actives = flock.flock;
  drone = new Drone(coord, vel, c);
}

void draw(){
  background(255);
  stroke(3);
  line(width-100, 0, width-100,height);
  stroke(1);
  flock.displayAll();
  flock.updateAll();
  
  drone.display();
  drone.update();
  
  droneAttract();
  finish.c = color(255);
}

void droneAttract(){
  UGV target = actives.get(actives.size()-1);
  
  float dx = abs(drone.coord.x - target.coord.x);
  float dy = abs(drone.coord.y = target.coord.y);
  
  if(drone.coord.x > target.coord.x){
    drone.coord.x -= dx;
  }else{drone.vel.x += dx;}
  
  if(drone.coord.y > target.coord.y){
    drone.coord.y -= dy;
  }else{drone.vel.y += dy;}
  
  if(drone.coord.dist(target.coord) < 2){
    target.tagged = true;
    target.c = color(255, 0, 0);
    actives.remove(target);
    if(actives.size() == 0){
      noStroke();
      flock.addUGV(finish);
    }
  }
}
