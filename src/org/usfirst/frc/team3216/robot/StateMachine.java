package org.usfirst.frc.team3216.robot;

import edu.wpi.first.wpilibj.Timer;

import java.util.HashMap;
import java.util.Map;


public class StateMachine {
	static Map<String,StateMachine> states = new HashMap<String,StateMachine>(); // hashmap to hold the objects
	
	static void add(String name) { // add a new statemachine
		StateMachine temp = new StateMachine(name);
		states.put(name, temp);
	}
	
	static void add(String name, double time) { // add one with an automatic time delay
		StateMachine temp = new StateMachine(name,time);
		states.put(name, temp);
	}
	
	static void start(String name) { // start the timer on a timed one, or just mark a normal one as running
		StateMachine temp = states.get(name);
		if (temp != null) { // gotta make sure it's not null or we crash the whole thing
			temp._start();
		} else {
			System.out.println("StateMachine not found: "+name); // hopefully this get to the DS so people can see what they messed up on
		}
	}
	
	static void trigger(String name) { // manually set the triggered flag
		StateMachine temp = states.get(name);
		if (temp != null) {
			temp._trigger();
		} else {
			System.out.println("StateMachine not found: "+name);
		}
	}
	
	static void cancel(String name) { // set the cancelled flag to make it false forever
		StateMachine temp = states.get(name);
		if (temp != null) {
			temp._cancel();
		} else {
			System.out.println("StateMachine not found: "+name);
		}
	}
	
	static boolean isRunning(String name) { // see if it's marked as running
		StateMachine temp = states.get(name);
		if (temp != null) {
			return temp._running();
		} else {
			System.out.println("StateMachine not found: "+name);
		}
		return false;
	}
	
	static boolean check(String name) { // see if it's been triggered but not yet cancelled
		StateMachine temp = states.get(name);
		if (temp != null) {
			return temp._check();
		} else {
			System.out.println("StateMachine not found: "+name);
		}
		return false;
	}
	
	static void reset() { // reset all of them to original state
		for (StateMachine i: states.values()) {
			i._reset();
		}
	}
	
	static void reset(String name) { // reset one by name
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
		this.name = name; // set up all vars
		this.running = false;
		this.triggered = false;
		this.manual = true;
		this.cancelled = false;
	}
	
	private StateMachine(String name, double time) { // automaticaly goes off based on a timer
		this.name = name; // more vars for this one
		this.time = time;
		this.timer = new Timer();
		this.running = false;
		this.triggered = false;
		this.manual = false;
		this.cancelled = false;
	}
	
	private void _start() {
		if (!this.manual) { // start timer if timed
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
			} else if (this.running && this.timer.get() > this.time) { // if timer has run out
				this.triggered = true;
				this.timer.stop();
				return true;
			}
			return false;
		} else {
			return this.triggered;
		}
	}
	
	private boolean _running() { // running and not cancelled
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
	
	private void _reset() { // reset all vars
		if (!manual)
			this.timer = new Timer();
		this.running = false;
		this.triggered = false;
		this.cancelled = false;
	}
}
