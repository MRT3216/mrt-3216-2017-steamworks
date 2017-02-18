class Setting {
  double value; // all the values are doubles for ease of use
  double min,max; // limits
  String name; // name of setting
  ScrollBar bar;
  
  
  Setting(String name, int x, int y) { 
    this.name = name;
    println(name);
    this.value = getSetting(name + "_val");
    this.min = getSetting(name + "_min");
    this.max = getSetting(name + "_max");
    this.bar = new ScrollBar(x,y,400,15,(float)min,(float)max,(float)value,this.name);
  }
  
  void disp() {
    this.bar.run();
    fill(0,255,0);
    if (this.value != this.bar.getVal()) {
      this.value = this.bar.getVal();
      pref.putNumber(this.name + "_val",this.value);
  }
}
}