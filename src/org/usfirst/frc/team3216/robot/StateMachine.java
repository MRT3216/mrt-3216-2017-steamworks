package org.usfirst.frc.team3216.robot;

import edu.wpi.first.wpilibj.Timer;

import java.util.HashMap;
import java.util.Map;


public class StateMachine {
	static Map<String,StateMachine> states = new HashMap<String,StateMachine>();
	
	static void add(String name, double time) {
		StateMachine temp = new StateMachine(name,time);
		states.put(name, temp);
	}
	
	static void start(String name) {
		StateMachine temp = states.get(name);
		if (temp != null) {
			temp._start();
		} else {
			System.out.println("StateMachine not found: "+name);
		}
	}
	
	static boolean check(String name) {
		StateMachine temp = states.get(name);
		if (temp != null) {
			return temp._check();
		} else {
			System.out.println("StateMachine not found: "+name);
		}
		return false;
	}
	
	static void reset() {
		for (StateMachine i: states.values()) {
			i._reset();
		}
	}
	
	//////////// dynamic
	
	String name;
	double time;
	Timer timer;
	boolean running;
	boolean triggered;
	
	private StateMachine(String name, double time) {
		this.name = name;
		this.time = time;
		this.timer = new Timer();
		this.running = false;
		this.triggered = false;
	}
	
	private void _start() {
		this.timer.stop();
		this.timer.reset();
		this.running = true;
		this.triggered = false;
		this.timer.start();
	}
	
	private boolean _check() {
		if (this.triggered) {
			return true;
		} else if (this.running && this.timer.get() > this.time) {
			this.triggered = true;
			this.timer.stop();
			return true;
		}
		return false;
	}
	
	private void _reset() {
		this.timer = new Timer();
		this.running = false;
		this.triggered = false;
	}
}
