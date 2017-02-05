class ScrollBar {
  float x,y,w,h;
  float min,max,val;
  String name;
  
  ScrollBar(float xp, float yp, float sw, float sh, float lo, float hi, float v, String n) {
    x=xp;
    y=yp;
    w=sw;
    h=sh;
    min=lo;
    max=hi;
    val=v;
    name=n;
  }
  
  void run() {
    if (mousePressed && mouseX > x && mouseY > y && mouseX < x+w && mouseY < y+h) {
      val = map(mouseX - x,0,w,min,max);
    }
    
    float computed = x + map(val,min,max+0.1,0,w);
    
    noStroke();
    fill(60);
    rect(x,y,w,h);
    strokeWeight(2);
    stroke(255,255,0);
    line(computed,y,computed,y+h);
    fill(255,0,0);
    text(min+"-"+max+" ~~ "+name+" ~ "+val,x+10,y+12);
  }
  
  float getVal() {
    return val;
  }
}