package org.usfirst.frc.team3216.robot;

import edu.wpi.first.wpilibj.Timer;

public class Toggle {
	boolean pressed, state;
	Timer debounce;
	
	Toggle() {
		pressed = false;
		state = false;
		debounce = new Timer();
	}
	
	void input(boolean button) {
		if (button && !this.pressed && this.debounce.get() > 0.07) { // perhaps make a Settings variable for this
			this.pressed = true;
			this.state = !this.state;
			this.debounce.reset();
			this.debounce.start();
		}
		if (!button) {
			this.pressed = false;
		}
	}
	
	boolean get() {
		return this.pressed;
	}
}
