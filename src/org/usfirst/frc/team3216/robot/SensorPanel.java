package org.usfirst.frc.team3216.robot;

import java.util.HashMap;
import java.util.Map;
import edu.wpi.first.wpilibj.networktables.NetworkTable;

public class SensorPanel {
	static Map<String,SensorPanel> sensors = new HashMap<String,SensorPanel>(); // hashmap to hold the objects
	static NetworkTable datatable;
	static int nums = 0;
	
	static void add(String id, String name, Type type, double min, double max, String units) {
		if (datatable == null) {
			datatable = NetworkTable.getTable("sensor_table");
			for (String s: datatable.getKeys()) { datatable.delete(s); }
		}
		
		SensorPanel temp = new SensorPanel(nums + "." + id,name,type,min,max,units);
		nums++;
		sensors.put(id,temp);
	}
	
	static void report(String id, double value) {
		SensorPanel temp = sensors.get(id);
		if (temp != null) { // gotta make sure it's not null or we crash the whole thing
			temp._report(value);
		} else {
			System.out.println("Sensor not found: "+id); // hopefully this get to the DS so people can see what they messed up on
		}
	}
	
	enum Type {NUMBER, BAR, CENTER};
	
	///////////////////////// dynamic
	
	String id, name, units;
	Type type;
	double min, max;
	
	private SensorPanel(String id, String name, Type type, double min, double max, String units) {
		this.id = id;
		this.name = name;
		this.type = type;
		this.min = min;
		this.max = max;
		this.units = units;
		
		datatable.putString(id+"-name", name);
		datatable.putString(id+"-units", units);
		datatable.putNumber(id+"-min", min);
		datatable.putNumber(id+"-max", max);
		switch (type) {
		case NUMBER:
			datatable.putNumber(id+"-type", 1);
			break;
		case BAR:
			datatable.putNumber(id+"-type", 2);
			break;
		case CENTER:
			datatable.putNumber(id+"-type", 3);
			break;
		}
		
		datatable.putNumber(id+"-val", -1);
	}
	
	private void _report(double value) {
		datatable.putNumber(this.id+"-val", value);
	}
}
