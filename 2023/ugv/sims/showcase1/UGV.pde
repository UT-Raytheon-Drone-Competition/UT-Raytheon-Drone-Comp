class UGV{
  PVector coord;
  PVector vel;
  color c;
  PShape body;
  boolean tagged = false;
  
  float size = 15.0;
  
  UGV(PVector coord, PVector vel, color c){
    this.coord = coord;
    this.vel = vel;
    this.c = c;
  }
  
  void display(){
    fill(c);
    rect(coord.x, coord.y, size, size);
  }
  
  void update(){
    this.coord.x += this.vel.x;
    this.coord.y += this.vel.y;
  }

}
