class Drone{
  PVector coord;
  PVector vel;
  color c;
  
  float rad = 10.0;
  
  Drone(PVector coord, PVector vel, color c){
    this.coord = coord;
    this.vel = vel;
    this.c = c;
  }
  
  void display(){
    fill(c);
    ellipse(coord.x, coord.y, rad, rad);
  }

  void update(){
    coord.x += vel.x;
    coord.y += vel.y;
  }

}
