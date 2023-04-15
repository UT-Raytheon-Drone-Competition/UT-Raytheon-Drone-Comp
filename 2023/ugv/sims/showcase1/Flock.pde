class Flock{
  ArrayList<UGV> flock;
  
  float minVel = 0;
  float maxVel = 0;
  int numUGV;
  
  Flock(int numUGV){
    this.numUGV = numUGV;
    this.flock = new ArrayList<UGV>();
    for(int i=0; i<numUGV; i++){
      flock.add(new UGV(new PVector(random(0,width-200),random(0,height)),
               new PVector(0,0), color(160, 160, 160)));
    }
  }
  
  void addUGV(UGV ugv){
    flock.add(ugv);
  }
  
  void displayAll(){
    for(UGV u: flock){
      u.display();
    }
  }
  
  void updateAll(){
    for(UGV u: flock){
      u.update();
    }
  }
  
}
