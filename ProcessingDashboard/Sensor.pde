class Sensor {
  float value; // all the values are doubles for ease of use
  double min,max; // limits
  int type;
  String name, id, units; // name of setting
  int x,y;
  
  float minv,maxv,avgv;
  
  
  Sensor(String id, int x, int y) { 
    this.id = id;
    this.x = x;
    this.y = y;
    println(id);
    
    this.name = getString(id + "-name");
    this.units = getString(id + "-units");
    this.min = getNumber(id + "-min");
    this.max = getNumber(id + "-max")+0.01;
    this.type = (int)getNumber(id + "-type");
    
    this.minv = 100000;
    this.maxv = 0;
    this.avgv = 0;
  }
  
  void disp() {
    double temp = getNumber(this.id + "-val");
    this.value = (float)((int)(temp * 100) / 100.0);
    
    if (this.value < this.minv) this.minv = this.value;
    if (this.value > this.maxv) this.maxv = this.value;
    this.avgv = (this.avgv + this.value) / 2; // i think this might work
    
    noStroke();
    
    switch (this.type) {
      case 1: // just display number
      case 4:
        // nothing here
      break;
      case 2: // one-way scrollbar
      case 5:
        fill(255,0,0);
        rect(this.x,this.y,map((float)this.value,(float)this.min,(float)this.max,1,400),23);
      break;
      case 3: // two-way scrollbar
        fill(255,0,0);
        rect(this.x + 200,this.y,map((float)this.value,(float)this.min,(float)this.max,-200,200),23);
      break;
    }
    
    fill(255);
    
    String stat = "";
    if (this.type == 4 || this.type == 5) {
      stat = " (" + this.minv + "~" + this.maxv + "  " + this.avgv + ")";
    }
    
    // draw the text afterwards:
    text(this.name + ": " + this.value +this.units + stat, this.x+10,this.y+15);
  }
  
  void reset() { // reset stats
    this.minv = 100000;
    this.maxv = 0;
    this.avgv = 0;
  }
}