import javax.swing.JOptionPane;

class ScrollBar {
  float x,y,w,h;
  float min,max,val;
  String name;
  boolean clicked;
  
  ScrollBar(float xp, float yp, float sw, float sh, float lo, float hi, float v, String n) {
    x=xp;
    y=yp;
    w=sw;
    h=sh;
    min=lo;
    max=hi;
    val=v;
    name=n;
    clicked = false;
  }
  
  void run() {
    if (!clicked && mousePressed && mouseX > x && mouseY > y && mouseX < x+w && mouseY < y+h) {
      val = map(mouseX - x,0,w,min,max);
    }
    if (!clicked && mousePressed && mouseX > x+w && mouseY > y && mouseX < x+w+h && mouseY < y+h) {
      val -= (max-min)/100.0;
      clicked = true;
    }
    if (!clicked && mousePressed && mouseX > x+w+h && mouseY > y && mouseX < x+w+2*h && mouseY < y+h) {
      val += (max-min)/100.0;
      clicked = true;
    }
    if (!clicked && mousePressed && mouseX > x+w+2*h && mouseY > y && mouseX < x+w+3*h && mouseY < y+h) {
      String t = JOptionPane.showInputDialog(null,"New value for "+name+" (original was "+ val +")?", "" + val);
      try {
        val = Float.parseFloat(t);
      } catch (Exception e) {}
      clicked = true;
    }
    if (!mousePressed) {
      clicked = false;
    }
    
    val = constrain(val,min,max);
    
    float computed = x + map(val,min,max+0.00001,0,w);
    
    noStroke();
    fill(60);
    rect(x,y,w,h);
    fill(255,0,0);
    rect(x+w,y,h,h);
    fill(0,255,0);
    rect(x+w+h,y,h,h);
    fill(255,255,0);
    rect(x+w+2*h,y,h,h);
    strokeWeight(2);
    stroke(255,255,0);
    line(computed,y,computed,y+h);
    fill(255,0,0);
    text(min+"-"+max+" ~~ "+name+" ~ "+val,x+10,y+10);
    fill(0);
    text("-",w+x+5,y+12);
    text("+",w+h+x+5,y+12);
    text("?",w+h*2+x+5,y+12);
  }
  
  float getVal() {
    return val;
  }
}