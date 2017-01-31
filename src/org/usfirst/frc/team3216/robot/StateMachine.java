package org.usfirst.frc.team3216.robot;

import edu.wpi.first.wpilibj.Timer;

import java.util.HashMap;
import java.util.Map;


public class StateMachine {
	static Map<String,StateMachine> states = new HashMap<String,StateMachine>();
	
	static void add(String name) {
		StateMachine temp = new StateMachine(name);
		states.put(name, temp);
	}
	
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
	
	static void trigger(String name) {
		StateMachine temp = states.get(name);
		if (temp != null) {
			temp._trigger();
		} else {
			System.out.println("StateMachine not found: "+name);
		}
	}
	
	static void cancel(String name) {
		StateMachine temp = states.get(name);
		if (temp != null) {
			temp._cancel();
		} else {
			System.out.println("StateMachine not found: "+name);
		}
	}
	
	static boolean isRunning(String name) {
		StateMachine temp = states.get(name);
		if (temp != null) {
			return temp._running();
		} else {
			System.out.println("StateMachine not found: "+name);
		}
		return false;
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
	
	static void reset(String name) {
		StateMachine temp = states.get(name);
		if (temp != null) {
			temp._reset();
		} else {
			System.out.println("StateMachine not found: "+name);
		}
	}
	
	//////////// dynamic
	
	String name;
	double time;
	Timer timer;
	boolean running;
	boolean triggered;
	boolean manual;
	boolean cancelled;
	
	private StateMachine(String name) { // manually triggered by the trigger() function
		this.name = name;
		this.running = false;
		this.triggered = false;
		this.manual = true;
		this.cancelled = false;
	}
	
	private StateMachine(String name, double time) { // automaticaly goes off based on a timer
		this.name = name;
		this.time = time;
		this.timer = new Timer();
		this.running = false;
		this.triggered = false;
		this.manual = false;
		this.cancelled = false;
	}
	
	private void _start() {
		if (!this.manual) {
			this.timer.stop();
			this.timer.reset();
			this.running = true;
			this.triggered = false;
			this.timer.start();
		}
	}
	
	private boolean _check() {
		if (this.cancelled) {
			return false;
		}
		if (!this.manual) {
			if (this.triggered) {
				return true;
			} else if (this.running && this.timer.get() > this.time) {
				this.triggered = true;
				this.timer.stop();
				return true;
			}
			return false;
		} else {
			return this.triggered;
		}
	}
	
	private boolean _running() {
		if (!cancelled) {
			return this.running; 
		} else {
			return false;
		}
	}
	
	private void _trigger() {
		this.triggered = true;
	}
	
	private void _cancel() {
		this.cancelled = true;
	}
	
	private void _reset() {
		if (!manual)
			this.timer = new Timer();
		this.running = false;
		this.triggered = false;
		this.cancelled = false;
	}
}
