package org.usfirst.frc.team3216.robot;
// all them imports:
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import org.usfirst.frc.team3216.robot.Utility.*;

public class Robot extends IterativeRobot {
	// setting up all the objects
	// global objects:
	Joystick xBox, bpanel; // xbox controller, obvoiusly (well actually a logitech gamepad) [also button panel]
	Toggle reverse;
	
	PowerDistributionPanel pdp; // to get voltage/amperage stuff
	DriverStation ds; // getting DS state, other info

	VictorSP leftdrive,rightdrive; // y-cable these outputs to the two speed controllers (2 motors per side)
	Spark balllauncher; // motor for launching the balls
	Spark balllift; // two BAG motors running the intake
	
	AnalogInput range_front, range_rear; // two different rangefinders
	Counter launcherencoder; // detects rate at which launcher spins
	ADIS16448_IMU imu; // fancy IMU on MXP port
	AnalogInput cradle_prox;
	
	MovingAverage front_avg, rear_avg; // smooth spikes in the rangefinders input by averaging the last several samples
	
	SendableChooser<Station> station; // chooser for where we're stationed
	
	NetworkTable lifttracker,boilertracker;
	
	
	public void robotInit() {
		initializeStuff(); // see the bottom of the file
		// input devices
		xBox = new Joystick(0); // joystick port 0
		bpanel = new Joystick(1); // secondary button panel
		reverse = new Toggle();
		// communication
		pdp = new PowerDistributionPanel(); // pdp object to read amperages, etc.
		ds = DriverStation.getInstance(); // to get match info for LEDs
		// speed controllers
		leftdrive = new VictorSP(0); // left motors = pwm 0
		rightdrive = new VictorSP(1); // right motors = pwm 1
		balllauncher = new Spark(2); // flywheel to launch fuel = pwm 2
		balllift = new Spark(3);
		// sensors
		range_front = new AnalogInput(0); // analog rangefinder on the front
		range_rear = new AnalogInput(1); // analog rangefinder on the front
		cradle_prox = new AnalogInput(2); // IR proximity sensor
		
		launcherencoder = new Counter(0); // encoder on the CIM that runs the fuel shooter
		imu = new ADIS16448_IMU(); // the fancy IMU that plugs into the MXP
		
		// sensor processing
		front_avg = new MovingAverage(7,250); // moving average for rangefinder (samples, start value)
		rear_avg = new MovingAverage(7,0); // moving average for rangefinder (samples, start value)
		
		// post-init
		launcherencoder.setDistancePerPulse(1/20.0); // the encoder has 20 pulses per revolution
		
		lifttracker = NetworkTable.getTable("LiftTracker");
		boilertracker = NetworkTable.getTable("TowerTracker");
	}
	
	Alliance auto_alliance = Alliance.RED; // defaults to red alliance here if the communication errors or something
	Station auto_station = Station.CENTER; // defaults to center (simplest) if chooser errors somehow
	
	public void autonomousInit() {
		StateMachine.reset(); // reset all the timers so we can run auto again without rebooting (issue last year!!)
		
		switch (ds.getAlliance()) { //  we don't need a SendableChooser for the alliance since we can get that info from the FMS
		case Blue:
			auto_alliance = Alliance.BLUE; // blue
			break;
		case Red:
			auto_alliance = Alliance.RED; // red 
			break;
		case Invalid: //  needed this to not throw an error
		}
		
		auto_station = station.getSelected(); // need to select where robot is before auto!
		
		StateMachine.start("initial_delay"); // start the timed delay while things calibrate
		
		// reset and recalibrate the IMU at the start of every match
		imu.calibrate(); // this has delays in it so i hope it doesn't take too long
		imu.reset(); // reset even though calibrate() probably already does that
	}

	// This function is called periodically during autonomous
	public void autonomousPeriodic() {
		// first, we check the different triggers to advance the auton routine
		if (StateMachine.check("initial_delay")) { // once finished sleeping
			StateMachine.cancel("initial_delay");
			if (auto_station != Station.CENTER) { // if not center station
				StateMachine.start("drive_back_1"); // left and right stations need to drive and turn
			} else { // else, if at center station
				StateMachine.start("drive_back_2"); // middle station just goes forward
			}
		}
		if (StateMachine.isRunning("drive_back_1") && // first drive stage
				front_avg.getLatest() > Settings.get("autondist1")) { // use the front rangefinder when driving backwards to see the back wall
			StateMachine.cancel("drive_back_1"); // StateMachine makes my life so much easier compared to last year
			if (auto_station != Station.HALT) { // if we select Halt, just stop here after crossing the baseline (though why we would makes no sense)
				StateMachine.start("turn"); // then turn 
			}
		}
		if (StateMachine.isRunning("turn") && (
				(auto_station == Station.LEFT && imu.getAngle() > Settings.get("autonangle")) || // if it turns the wrong way, flip LEFT and RIGHT
				(auto_station == Station.RIGHT && imu.getAngle() < -Settings.get("autonangle")))) { // so many booleans
			StateMachine.cancel("turn");
			StateMachine.start("drive_back_2");
		}
		if (StateMachine.isRunning("drive_back_2") &&
				rear_avg.getLatest() < Settings.get("liftdist")) { // use the rear rangefinder when driving backwards to see the lift
			StateMachine.cancel("drive_back_2");
			StateMachine.start("wait_gear"); // wait for the user to lift the gear 
		}
		if (StateMachine.isRunning("wait_gear") &&
				cradle_prox.getValue() > Settings.get("geardetect")) { // wait until the gear is lifted out of the cradle
			StateMachine.cancel("wait_gear");
			if ((auto_alliance == Alliance.BLUE && auto_station == Station.LEFT) ||  // these are the two positions from where we could shoot
					(auto_alliance == Alliance.RED && auto_station == Station.RIGHT)) { // i hope we can get these every time
				StateMachine.start("drive_fwd_3"); // drive back toward the high goal
			} else { // if we're not in those positions, then just get ready for teleop
				// ???
			}
		}
		if (StateMachine.isRunning("drive_fwd_3") && // drive
				rear_avg.getLatest() > Settings.get("autondist2")) { 
			StateMachine.cancel("drive_fwd_3");
			StateMachine.start("aim_high");  
		}
		if (StateMachine.isRunning("aim_high") && // rotate so we're aiming at the tower
				true /* TODO: fix sensor here*/) { 
			StateMachine.cancel("aim_high");
			StateMachine.start("shoot");  
		}
		
		// TODO: add the code in here to run motors, etc for auto stages
		if (StateMachine.isRunning("drive_back_1")) { // drive backward
			
		} else if (StateMachine.isRunning("turn")) { // turn toward the lift
			if (auto_station == Station.LEFT) { // turn to the right
				
			} else if (auto_station == Station.RIGHT) { // turn to the left
				
			}
		} else if (StateMachine.isRunning("drive_back_2")) { // drive back, targeting the lift with vision
			placeGear(true);
		} else if (StateMachine.isRunning("wait_gear")) { // wait for the gear to be lifted
			placeGear(false);
			// probably don't need to do anything here
		} else if (StateMachine.isRunning("drive_fwd_3")) { // drive forward
			
		} else if (StateMachine.isRunning("aim_high")) { // aim based on the vision
			aimLauncher();
		} else if (StateMachine.isRunning("shoot")) { // shoot balls into the high goal
			runLauncher(true);
		}
	}
	
	public void teleopInit() {
		// initialize things
		launcherspeed = 1; // set this high to make it speed up faster, might work
	}

	// variables used in teleop:
	// the variables that have _in are from the joystick
	double leftdrive_in, rightdrive_in;
	boolean runintake_in,  runshooter_in,  rungear_in, reverse_in,  slow_in, straight_in;
	//boolean runlaunch_btn, runshooter_btn, rungear_btn, reverse_btn, slow_btn; // don't need separate vars for button panel (yet)
	
	boolean buttons_connected = false; // not used but possibly will be
	// This function is called periodically during operator control
	public void teleopPeriodic() {
		leftdrive_in = xBox.getRawAxis(1); // these are supposed to be the vertical axes (for tank drive)
		rightdrive_in = xBox.getRawAxis(5); // checked
		
		runintake_in = xBox.getRawButton(3); // X / blue
		runshooter_in = xBox.getRawButton(2); // B / red
		rungear_in = xBox.getRawButton(4); // Y / yellow
		reverse_in = xBox.getRawButton(5); // left trigger (toggle)
		slow_in = xBox.getRawButton(9); // press left joystick
		straight_in = xBox.getRawButton(10); // press right joystick
		
		try {
			// button panel is in here so when we disconnect it, it doesn't throw errors
			runintake_in |= bpanel.getRawButton(1); // TODO: change these indexes
			runshooter_in |= bpanel.getRawButton(2);
			rungear_in |= bpanel.getRawButton(3);
			reverse_in |= bpanel.getRawButton(4);
			slow_in |= bpanel.getRawButton(5);
			straight_in |= bpanel.getRawButton(6);
			
			buttons_connected = true;
		} catch (Exception e) { // this will throw errors if the button panel is not connected
			buttons_connected = false;
		}
		
		reverse.input(reverse_in);
		
		if (straight_in) { // so we can drive perfectly straight at any speed
			leftdrive_in = rightdrive_in;
		}
		
		if (slow_in) { // change the motor map so we have more maneuverability (possibly reverse this??)
			rightdrive_in = rightdrive_in * Settings.get("slow");
			leftdrive_in = leftdrive_in * Settings.get("slow");
		}
		
		if (reverse.get()) { // we can drive backwards
			drive(-rightdrive_in, -leftdrive_in); // drive backward
		} else {
			drive(leftdrive_in, rightdrive_in); // drive function
		}
		
		runLauncher(runshooter_in); // run launcher if buttons are pressed
		
		placeGear(rungear_in); // do the whole gear placement routine of the button is pressed
		
		intake(runintake_in); // run the intake motor if button is pressed
	}
	
	/* the following useful functions:
	 * 
	 * drive(double left_joystick, double right_joystick) : moves the motors, handles deadzone and ideally reversing stuff
	 */
	
	//////////////////////////////// various management functions
	
	void drive(double left, double right) { // tank drive
		right = -right;
		
		if (Math.abs(left) > Settings.get("deadzone")) { // deadzone the motors
			leftdrive.set((Math.pow(left,3) * Settings.get("motormap") + (1 - Settings.get("motormap")) * left) * Settings.get("motorproportion")); // cubic motor map
		} else {
			leftdrive.set(0); // else stop it
		}
		
		if (Math.abs(right) > Settings.get("deadzone")) { // deadzone
			rightdrive.set((Math.pow(right,3) * Settings.get("motormap") + (1 - Settings.get("motormap")) * right) * Settings.get("motorproportion"));
		} else {
			rightdrive.set(0); // else stop it
		}
	}
	
	double launcherspeed = 1; // this global value we set to the latest value written to the motors
	
	void runLauncher(boolean on) { // on is whether or not to run the shooter
		if (on) {
			double rate = launcherencoder.getRate() * 60; // convert encoder to RPM
			double idealrate = Settings.get("launcherrpm"); // ideal rpm specified in settings
			if (Math.abs(rate - idealrate) < Settings.get("launcherdeadzone")) { // handle deadzone mechanics for the speed
			} else if (rate > idealrate) { // if it's too fast:
				launcherspeed -= Utility.map(Math.abs(rate - idealrate),0,5000,0,Settings.get("launcher-p")); // slow it down based on how far the discrepency is
			} else {
				launcherspeed += Utility.map(Math.abs(rate - idealrate),0,5000,0,Settings.get("launcher-p")); // speed it up
			}
			balllauncher.set(launcherspeed); // set the new value
		} else {
			balllauncher.set(0); // else, stop the motor
		}
	}
	
	void aimLauncher() { // turn the robot to aim at the high goal
		
	}
	
	void placeGear(boolean on) { // drive backward to place the gear while aiming
		// need a state machine to handle the aiming and then driving
		if (on) {
			if (!StateMachine.isGroupRunning("gear")) {
				StateMachine.start("center_gear"); // start the first stage if not already running
			}
		} else { // as soon as the button is lifted, reset
			StateMachine.resetGroup("gear");
		}
		
		if (StateMachine.isRunning("center_gear") && 
				(true /* check the center */)) {
			StateMachine.cancel("center_gear");
			StateMachine.start("aim_gear");
		} else if (StateMachine.isRunning("aim_gear") && 
				(true /* check the vision deadzone */)) {
			StateMachine.cancel("aim_gear");
			StateMachine.start("drive_gear");
		} else if (StateMachine.isRunning("drive_gear") && 
				rear_avg.getLatest() < Settings.get("liftdist")) {
			StateMachine.cancel("drive_gear");
			StateMachine.start("pause_gear");
		}
		
		if (StateMachine.isRunning("center_gear")) {
			centerGear();
		} else if (StateMachine.isRunning("aim_gear")) {
			aimGear();
		} else if (StateMachine.isRunning("drive_gear")) {
			
		} else if (StateMachine.isRunning("pause_gear")) {
			// probably do nothing
		}
		
	}
	
	void aimGear() { // move the gear cradle based on vision input
		
	}
	
	void centerGear() { // center the gear cradle so that we can use vision
		
	}
	
	void intake(boolean on) { // just run the motors to lift fuel off the ground
		if (on) {
			balllift.set(Settings.get("intakespeed"));
		} else {
			balllift.set(0);
		}
	}
	
	
	////////////////////////// miscellaneous stuff
	
	public void testPeriodic() {
		
	}
	
	public void disabledPeriodic() {
		
	}
	
	// all-encompassing function to handle periodic tasks.
	public void robotPeriodic() {
		//moving average for rangefinders
		front_avg.newSample(range_front.getValue()); //  i think these inputs are in centimeters
		rear_avg.newSample(range_rear.getValue()); // probably not though
		
		Settings.sync(); // this syncs local settings with the NetworkTable and the DS config utility
		
		syncSensors(); // TODO: disable this line for competition because it's mainly for testing
	}
	
	void syncSensors() {
		try { // put data into table (probably disable this during comp)
			SensorPanel.report("ctl_v",ControllerPower.getInputVoltage()); // roborio voltage
			SensorPanel.report("range_f",front_avg.getLatest()); // averaged rangefinder value
			SensorPanel.report("range_r",rear_avg.getLatest()); // averaged rangefinder value
			SensorPanel.report("gyro_x",imu.getAngleX()); // gyroscope on IMU
			SensorPanel.report("gyro_y",imu.getAngleY()); 
			SensorPanel.report("gyro_z",imu.getAngleZ()); 
			SensorPanel.report("accel_x",imu.getAccelX()); // accelerometer on IMU
			SensorPanel.report("accel_y",imu.getAccelY());
			SensorPanel.report("accel_z",imu.getAccelZ());
			SensorPanel.report("imu_p",imu.getPitch()); // calculated angles
			SensorPanel.report("imu_r",imu.getRoll());
			SensorPanel.report("imu_y",imu.getYaw());
			SensorPanel.report("cradle_p",cradle_prox.getValue());
			SensorPanel.report("enc_r",launcherencoder.getRate() * 60);
			// do these last so if they fail, the rest still runs
			SensorPanel.report("pwr_c",pdp.getTotalCurrent()); // total current draw
			SensorPanel.report("pwr_v",pdp.getVoltage()); // PDP voltage (not the same as DS voltage)
			SensorPanel.report("pwr_t",pdp.getTemperature()); // useful to tell if there are things heating up
			for (int i = 0; i < 16; i++) SensorPanel.report("pwr_c_"+i, pdp.getCurrent(i)); // current draw for all 16 channels
		} catch (Exception a) {
			System.out.println("error in syncSensors");
		} // runtime exception could be caused by CAN timeout
	}
	
	void initializeStuff() {
		// all of this used to be up in the init but it's nice to have it down here out of the way
		/// persistent settings are set up here
		Settings.add("deadzone", 0.07,0,1); // deadzone in joysticks
		Settings.add("motormap", 1, 0, 1); // motor cubic factor
		Settings.add("motorproportion", 0.7, 0, 1); // motor proprortional slow factor
		Settings.add("slow", 0.7,0,1); // multiplier for when we hit the slow down button
		Settings.add("marginoferror", 10, 0, 150); // margin of error for the MovingAverage class when we call getLatest()
		Settings.add("launcherrpm", 3000, 0, 6000); // rpm to keep the  launcher at while it is shooting
		Settings.add("launcherdeadzone", 5, 0, 30); // deadzone at which to stop atjusting the motor input (+- rpm)
		Settings.add("launcher-p", 0.3, 0, 1); // rate at which to adjust the launcher speed (maybe switch to PID if this doesn't work)
		Settings.add("visiondeadzone", 20, 1, 100); // deadzone in pixels in which to aim at vision targets
		Settings.add("geardetect", 1000, 0, 4096); // analog read form the IR sensor to tell when the gear is present
		Settings.add("intakespeed", 1, 0, 1); // speed to run the intake motors
		// auto settings
		Settings.add("autonspeed", 0.5, 0, 1); // speed to drive in auton
		Settings.add("autondelay", 4, 0, 15); // delays during auton
		Settings.add("autondist1", 60, 0, 250); // distance to drive in auto before turning (front rangefinder because the gear is on the back)
		Settings.add("autondist2", 100, 0, 250); // distance to drive from the lift before shooting
		Settings.add("autonangle", 60, 0, 100); // angle to turn in auto when targeting the lifts
		Settings.add("liftdist", 20, 0, 250); // distance to get from the lift when placing a gear (rear rangefinder) (also used when auto-targeting in teleop)
		Settings.add("autonturnspeed", 0.4, 0, 1); // rate to turn in auto (very slow is good, but not too slow)
		
		// set up the sensors!
		SensorPanel.add("pwr_v", "PDB Voltage", SensorPanel.Type.BAR_STAT, 0, 15, "V");
		SensorPanel.add("ctl_v", "Controller Voltage", SensorPanel.Type.BAR_STAT, 0, 15, "V");
		SensorPanel.add("pwr_c", "PDB Total Current", SensorPanel.Type.BAR_STAT, 0, 500, "A");
		SensorPanel.add("pwr_t", "PDB Temperature", SensorPanel.Type.NUMBER, 0, 1, "F");
		for (int i = 0; i < 16; i++) SensorPanel.add("pwr_c_"+i, "PDB Current CH"+i, SensorPanel.Type.BAR_STAT, 0, 15, "A");
		SensorPanel.add("range_f", "Front Rangefinder", SensorPanel.Type.BAR, 0, 260, "cm");
		SensorPanel.add("range_r", "Rear Rangefinder", SensorPanel.Type.BAR, 0, 260, "cm");
		SensorPanel.add("gyro_x", "Gyroscope X", SensorPanel.Type.NUMBER, 0, 1, "deg");
		SensorPanel.add("gyro_y", "Gyroscope Y", SensorPanel.Type.NUMBER, 0, 1, "deg");
		SensorPanel.add("gyro_z", "Gyroscope Z", SensorPanel.Type.NUMBER, 0, 1, "deg");
		SensorPanel.add("accel_x", "Accelerometer X", SensorPanel.Type.CENTER, -2, 2, "g");
		SensorPanel.add("accel_y", "Accelerometer Y", SensorPanel.Type.CENTER, -2, 2, "g");
		SensorPanel.add("accel_z", "Accelerometer Z", SensorPanel.Type.CENTER, -2, 2, "g");
		SensorPanel.add("imu_p", "Pitch", SensorPanel.Type.NUMBER, 0, 1, "deg");
		SensorPanel.add("imu_r", "Roll", SensorPanel.Type.NUMBER, 0, 1, "deg");
		SensorPanel.add("imu_y", "Yaw", SensorPanel.Type.NUMBER, 0, 1, "deg");
		SensorPanel.add("cradle_p", "Cradle Proximity", SensorPanel.Type.BAR, 0, 4096, "");
		SensorPanel.add("enc_r", "Launcher encoder", SensorPanel.Type.NUMBER, 0, 1, "rpm");
		//SensorPanel.add("", "", SensorPanel.Type.BAR, 0, 1, "");
		
		// lay out the auton state machines (stages)
		StateMachine.add("initial_delay", 0.5); // start out by pausing for a moment
		StateMachine.add("drive_back_1"); // back up (gear on back) specified distance
		StateMachine.add("turn"); // rotate to face the peg if needed
		StateMachine.add("drive_back_2"); // drive again up to the peg (with vision)
		StateMachine.add("wait_gear"); // wait until the gear is lifted
		StateMachine.add("drive_fwd_3"); // drive up to the point where we can shoot
		StateMachine.add("aim_high"); // aim for the high goal with vision
		StateMachine.add("shoot"); // shoot as many balls as possible
		
		// statemachines to handle the gear placing 
		StateMachine.add("center_gear","gear"); // center the gear cradle first
		StateMachine.add("aim_gear","gear"); // turn so the vision targets are in the middle of the FOV
		StateMachine.add("drive_gear","gear"); // drive until the robot gets to the lift
		StateMachine.add("pause_gear","gear"); // wait (don't start another aiming run)
		
		// set up auton chooser
		station = new SendableChooser<Station>(); // pretty simple to choose mode
		station.addDefault("Center Station", Station.CENTER); // default is center, where we just drive froward
		station.addObject("Left station", Station.LEFT); // we prefer to be on the side of the high goal so we can shoot
		station.addObject("Right station", Station.RIGHT); // however we want all options to be open
		station.addObject("Do Nothing", Station.HALT); // of we want, we can just cross the base line and stop
	}
}

