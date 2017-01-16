package org.usfirst.frc.team3216.robot;
// all them imports:
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.networktables.NetworkTable;

// Juice was here
public class Robot extends IterativeRobot {
	// setting up all the objects
	// global objects:
	Joystick xBox, bpanel; // xbox controller, obvoiusly (well actually a logitech gamepad) [also button panel]
	
	PowerDistributionPanel pdp; // to get voltage/amperage stuff
	DriverStation ds; // getting DS state, other info
	NetworkTable table; // sending data back & forth for sensors
	
	VictorSP leftdrive,rightdrive; // y-cable these outputs to the two speed controllers (2 motors per side)
	VictorSP balllauncher;
	
	AnalogInput range;
	Encoder launcherencoder;
	ADIS16448_IMU imu;
	
	
	//REVDigitBoard disp; // digit board connected to MXP 
	MovingAverage rangefinder; // smooth spikes in the rangefinder input by averaging the last several samples

	/* Connections:
	 * 
	 * left motors: victor on pwm 0
	 * right motors: victor on pwm 1
	 
	 * rangefinder: analog input 0
	 
	 * 
	 */
	
	/* Controls:
	 * 
	 * vertical joystick axes: tank drive left and right
	 * right trigger: 
	 * left trigger: 
	 * left bumper: 
	 */
	
	/*
	 * Secondary button panel:
	 * sw
	 *     1   3   6
	 *     2   4   5
	 *     
	 * a.k.a.
	 * sw
	 *   
	 *   
	 */
	
	public void robotInit() {
		/// prefs: here we instantiate values and stuff
		Settings.add("deadzone", 0.07,0,1); // deadzone in joysticks
		Settings.add("motormap", 0.7, 0, 1); // motor slow down factor
		Settings.add("autonspeed", 0.5, 0, 1); // speed to drive in auton
		Settings.add("autondelay", 4, 0, 15); // delay before driving forward to allow compressor to power up and lift plate, so it won't get stuck
		Settings.add("launcherrpm", 3000, 0, 6000);
		Settings.add("launcherdeadzone", 5, 0, 30);
		
		/// now we set up the objects
		xBox = new Joystick(0); // joystick port 0
		bpanel = new Joystick(1); // secondary button panel
		
		table = NetworkTable.getTable("datatable"); // this table communicates back to the computer for diagnostic purposes
		pdp = new PowerDistributionPanel(); // pdp objecto to read amperages, etc.
		ds = DriverStation.getInstance(); // to get match info for LEDs
		
		leftdrive = new VictorSP(0); // left motors = pwm 0
		rightdrive = new VictorSP(1); // right motors = pwm 1
		balllauncher = new VictorSP(2);
		
		range = new AnalogInput(0); // analog rangefinder
		rangefinder = new MovingAverage(3,250); // moving average for rangefinder (samples, start value)
		launcherencoder = new Encoder(0, 1, false, Encoder.EncodingType.k1X);
		
		launcherencoder.setDistancePerPulse(1/20.0);
		
		/*
		disp = new REVDigitBoard(); // REV digit board object
		disp.clear(); // clear any prevoius data
		disp.display("-nc-"); // indicate that the robot is loading. this will be overwritten in the sendData periodic function
		*/
		imu = new ADIS16448_IMU();
	}
	
	public void autonomousInit() {
		
	}

	// This function is called periodically during autonomous
	public void autonomousPeriodic() {
		sendData(); // this also does the moving average stuff
		
	}
	
	public void teleopInit() {
		// initialize things
	}

	// variables used in teleop:
	double leftdrive_in, rightdrive_in;
	// This function is called periodically during operator control
	public void teleopPeriodic() {
		// this took a lot of guess-and-check, since several 
		// axes were messed up this year. don't trust the 
		// joystick explorer, use the DS for testing these
		leftdrive_in = xBox.getRawAxis(1); // these are supposed to be the vertical axes (for tank drive)
		rightdrive_in = xBox.getRawAxis(5); // checked
		
		drive(leftdrive_in, -rightdrive_in); // drive function
		
		sendData(); // periodic function
	}
	
	/* the following useful functions:
	 * 
	 * drive(double left_joystick, double right_joystick) : moves the motors, handles deadzone and ideally reversing stuff
	 */
	
	//////////////////////////////// various management functions
	
	void drive(double left, double right) {
		if (Math.abs(left) > Settings.get("deadzone")) { // deadzone the motors
			leftdrive.set(Math.pow(left,3)*Settings.get("motormap")); // cubic motor map
		} else {
			leftdrive.set(0); // else stop it
		}
		
		if (Math.abs(right) > Settings.get("deadzone")) { // deadzone
			rightdrive.set(Math.pow(right,3)*Settings.get("motormap"));
		} else {
			rightdrive.set(0); // else stop it
		}
	}
	
	double launcherspeed = 1; // this global value we set to the latest value written to the motors
	
	void runlauncher(boolean on) { // on is whether or not to run the shooter
		if (on) {
			double rate = launcherencoder.getRate() * 60; // convert encoder to RPM
			double idealrate = Settings.get("launcherrpm"); // ideal rpm specified in settings
			if (Math.abs(rate - idealrate) < Settings.get("launcherdeadzone")) { // handle deadzone mechanics for the speed
				balllauncher.set(launcherspeed); // just run the motor with the last value
			} else if (rate > idealrate) { // if it's too fast:
				launcherspeed -= map(Math.abs(rate - idealrate),0,5000,0,0.3); // slow it down based on how far the discrepency is
				balllauncher.set(launcherspeed); // set the new value
			} else {
				launcherspeed += map(Math.abs(rate - idealrate),0,5000,0,0.3); // speed it up
				balllauncher.set(launcherspeed); // set the new value
			}
		} else {
			balllauncher.set(0); // else, stop the motor
		}
	}
	
	////////////////////////// miscellaneous stuff
	
	public void testPeriodic() {
		sendData(); // send data in test
	}
	
	public void disabledPeriodic() {
		sendData(); // send data in disabled
	}
	
	double map(double value, double istart, double istop, double ostart, double ostop){ // to map stuff from one range to another
		return ostart + (ostop - ostart) * ((value - istart) / (istop - istart));
	}
	
	// this kinda became the all-encompassing function to handle periodic tasks.
	void sendData() {
		//moving average for rangefinder
		double a_range = range.getValue(); // centimeters hopefully
		rangefinder.newSample(a_range);
		
		Settings.sync(); // this syncs local settings with the NetworkTable and the DS config utility
		
		syncSensors(); // disable for competition
		
		//digit board
		//disp.display(ControllerPower.getInputVoltage()); //live voltage readout ideally, or whatever we need
	}
	
	void syncSensors() {
		try { // put data into table (probably disable this during comp)
			table.putNumber("pwr_v",pdp.getVoltage()); // PDP voltage (not the same as DS voltage)
			table.putNumber("pwr_t",pdp.getTemperature()); // useful to tell if there are things heating up
			table.putNumber("pwr_c",pdp.getTotalCurrent()); // total current draw
			for (int i = 0; i < 16; i++) table.putNumber("pwr_c_" + i,pdp.getCurrent(i)); // current draw for all 16 channels
			//table.putNumber("pcm_c",pcm.getCompressorCurrent()); // compressor current draw
			table.putNumber("range",rangefinder.getAverage()); // averaged rangefinder value
			table.putNumber("ctl_v",ControllerPower.getInputVoltage()); // roborio voltage
			table.putNumber("ctl_c",ControllerPower.getInputCurrent()); // roborio current draw
			table.putNumber("ctl_fault",ControllerPower.getFaultCount3V3()+ControllerPower.getFaultCount5V()+ControllerPower.getFaultCount6V()); // total voltage fault count
			//table.putNumber("inf_range",balllimit.getValue());
		} catch (RuntimeException a) { } // runtime exception could be caused by CAN timeout
	}
}
