import edu.wpi.first.wpilibj.networktables.NetworkTable;

NetworkTable table;
NetworkTable pref;

ArrayList<Setting> settings;
ArrayList<Sensor> sensors;

ConnectionListener cl = new ConnectionListener();

boolean connected = false;

void setup() {
  size(1000,700);
  // Initialize NetworkTables
  NetworkTable.setClientMode();
  NetworkTable.setIPAddress("roboRIO-3216-FRC.local");
  table = NetworkTable.getTable("sensor_table");
  pref = NetworkTable.getTable("Preferences");
  pref.addConnectionListener(cl,false);
  
  frameRate(20); // decrease the speed to ease network stuff
  
  initset(); // would be nice to have this wait until server was connected.......
  initsens();
}


float amap = 16;
float pmap = 2;
int[] chans = {1,0,2,15,14,13};  // pdb channels to display


float ch_max[] = new float[16];
float minbat_v = 1000, maxbat_v = -1000;
//float bat_graph = new float[100];

void draw() {
  background(0);
  
  //// settings
  try {
    for (Setting sett: settings) {
      sett.disp();
    }
  } catch (Exception e) { }
  
  try {
    for (Sensor sen: sensors) {
      sen.disp();
    }
  } catch (Exception e) { }
  
  noStroke();
  if (connected) {
    fill(0,255,0);
  } else { 
    fill(255,0,0);
  }
  rect(450,4,100,30);
  fill(255);
  if (connected) {
    text("Connected",460,25);
  } else {
    text("Disonnected",460,25);
  }
  fill(255,255,0);
  rect(450,36,170,30);
  fill(2);
  text("reset - hover and press R",455,56);
}


float getNumber(String z) { // needed to simplify this
  try {
    return (float)table.getNumber(z,0);
  } catch (Exception e) { 
    return 0;
  }
}

String getString(String z) { // needed to simplify this
  try {
    return table.getString(z,"");
  } catch (Exception e) { 
    return "";
  }
}

float getSetting(String z) { // needed to simplify this
  try {
    return (float)pref.getNumber(z,0);
  } catch (Exception e) { 
    return 0;
  }
}

void keyPressed() { // space resets the max/mix stuff
  if (key == ' ') {
    initset(); // refresh from server
    for (Sensor sen: sensors) {
      sen.reset();
    }
  }
  if (key == 'r' && mouseX > 450 && mouseX < 450+170 && mouseY > 36 && mouseY < 36+30) {
    pref.putBoolean("reset",true);
  }
}

void initset() { // initiate settings
  settings = new ArrayList<Setting>();
  int yv = 10;
  for (Object s: pref.getKeys()) {
    String k = (String)s;
    if (k.endsWith("_val")) {
      String[] j = k.split("[_]");
      settings.add(new Setting(j[0],620,yv));
      yv += 25;
    }
  }
}

void initsens() { // initiate sensors
  sensors = new ArrayList<Sensor>();
  //int yv = 10;
  for (Object s: table.getKeys()) {
    try {
      String k = (String)s;
      if (k.endsWith("-val")) {
        String[] j = k.split("[-]");
        String[] l = j[0].split("[.]");
        int yv = Integer.parseInt(l[0]);
        sensors.add(new Sensor(j[0],5,yv*20));
        //yv += 20;
      }
    } catch (Exception e) { }
  }
}

class ConnectionListener implements IRemoteConnectionListener {
  void connected(IRemote i) {
    initset(); // hopefully this works?
    initsens();
    connected = true;
  }
  
  void disconnected(IRemote i) {
    connected = false;
  }
}