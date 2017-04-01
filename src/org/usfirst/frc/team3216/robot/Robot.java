package org.usfirst.frc.team3216.robot;
// all them imports:
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc.team3216.robot.Utility.*;

public class Robot extends IterativeRobot {
	// setting up all the objects
	// global objects:
	Joystick xBox, bpanel; // xbox controller, obvoiusly (well actually a logitech gamepad) [also button panel]
	Toggle reverse;
	
	PowerDistributionPanel pdp; // to get voltage/amperage stuff
	DriverStation ds; // getting DS state, other info

	VictorSP leftdrive,rightdrive; // y-cable these outputs to the two speed controllers (2 motors per side)
	VictorSP balllauncher; // motor for launching the balls
	Spark balllift; // two BAG motors running the intake
	Talon agitator, indexer;
	
	AnalogInput range_front, range_rear; // two different rangefinders
	Counter launcherencoder; // detects rate at which launcher spins
	ADIS16448_IMU imu; // fancy IMU on MXP port
	AnalogInput cradle_prox;
	
	MovingAverage front_avg, rear_avg; // smooth spikes in the rangefinders input by averaging the last several samples
	SerialPort arduino;
	Timer arduinoTimer;
	Timer matchTimer;
	Timer indexerTimer;
	
	SendableChooser<Station> station; // chooser for where we're stationed
	
	NetworkTable lifttracker,boilertracker;
	
	boolean arduino_att = false;
	
	
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
		balllauncher = new VictorSP(2); // flywheel to launch fuel = pwm 2
		balllift = new Spark(3);
		agitator = new Talon(4);
		indexer = new Talon(5);
		
		// sensors
		range_front = new AnalogInput(0); // analog rangefinder on the front
		range_rear = new AnalogInput(1); // analog rangefinder on the front
		cradle_prox = new AnalogInput(2); // IR proximity sensor
		
		launcherencoder = new Counter(0); // encoder on the CIM that runs the fuel shooter
		imu = new ADIS16448_IMU(); // the fancy IMU that plugs into the MXP
		
		// sensor processing
		front_avg = new MovingAverage(3,0); // moving average for rangefinder (samples, start value)
		rear_avg = new MovingAverage(3,0); // moving average for rangefinder (samples, start value)
		
		try {
			arduino = new SerialPort(115200,SerialPort.Port.kUSB);
			arduino_att = true;
		} catch (Exception e) {
			arduino_att = false;
		}
		arduinoTimer = new Timer();
		arduinoTimer.start();
		matchTimer = new Timer();
		indexerTimer = new Timer();
		matchTimer.start();
		
		// post-init
		launcherencoder.setDistancePerPulse(1/20.0); // the encoder has 20 pulses per revolution
		
		lifttracker = NetworkTable.getTable("LiftTracker");
		boilertracker = NetworkTable.getTable("BoilerTracker");
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
		
		matchTimer.reset();
		matchTimer.start();
		rear_vision = -100;
		front_vision = -100;
		
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
				front_avg.getAverage() > Settings.get("autondist1")) { // use the front rangefinder when driving backwards to see the back wall
			StateMachine.cancel("drive_back_1"); // StateMachine makes my life so much easier compared to last year
			if (auto_station != Station.HALT) { // if we select Halt, just stop here after crossing the baseline (though why we would makes no sense)
				StateMachine.start("turn"); // then turn 
			}
		}
		if (StateMachine.isRunning("turn") && (
				(auto_station == Station.LEFT && imu.getAngleZ() > Settings.get("autonangle")) || // if it turns the wrong way, flip LEFT and RIGHT
				(auto_station == Station.RIGHT && imu.getAngleZ() < -Settings.get("autonangle")))) { // so many booleans
			StateMachine.cancel("turn");
			StateMachine.start("drive_back_2");
		}
		if (StateMachine.isRunning("drive_back_2") &&
				
				rear_avg.getAverage() < Settings.get("liftdist")) { // use the rear rangefinder when driving backwards to see the lift
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
				rear_avg.getAverage() > Settings.get("autondist2")) { 
			StateMachine.cancel("drive_fwd_3");
			StateMachine.start("aim_high");  
		}
		
		if (StateMachine.isRunning("drive_back_1")) { // drive backward
			drive(-Settings.get("autonspeed"),-Settings.get("autonspeed")); // TODO: switch if backwards
			System.out.println("drive back 1");
		} else if (StateMachine.isRunning("turn")) { // turn toward the lift
			if (auto_station == Station.LEFT) { // turn to the right
				drive(-Settings.get("autonturnspeed"),Settings.get("autonturnspeed")); // TODO: switch if backwards
			} else if (auto_station == Station.RIGHT) { // turn to the left
				drive(Settings.get("autonturnspeed"),-Settings.get("autonturnspeed")); // TODO: switch if backwards
			}
			System.out.println("turning");
		} else if (StateMachine.isRunning("drive_back_2")) { // drive back, targeting the lift with vision
			placeGear(true);
			System.out.println("drive back 2");
		} else if (StateMachine.isRunning("wait_gear")) { // wait for the gear to be lifted
			placeGear(false);
			System.out.println("wait for gear");
			// probably don't need to do anything here
		} else if (StateMachine.isRunning("drive_fwd_3")) { // drive forward
			drive(-Settings.get("autonspeed"),-Settings.get("autonspeed")); // TODO: switch if backwards
			System.out.println("drive fwd 1");
		} else if (StateMachine.isRunning("aim_high")) { // aim based on the vision (and shoot)
			highGoal(true);
			System.out.println("shoot");
		} 
	}
	
	public void teleopInit() {
		// initialize things
		launcherspeed = 1; // set this high to make it speed up faster, might work
	}

	// variables used in teleop:
	// the variables that have _in are from the joystick
	double leftdrive_in, rightdrive_in;
	boolean runintake_in,  runshooter_in,  rungear_in, reverse_in,  slow_in, straight_in, boost_in;
	boolean autogear_in, autoshoot_in, climb_in, revclimb_in;
	//boolean runlaunch_btn, runshooter_btn, rungear_btn, reverse_btn, slow_btn; // don't need separate vars for button panel (yet)
	
	boolean buttons_connected = false; // not used but possibly will be
	// This function is called periodically during operator control
	public void teleopPeriodic() {
		/*
		leftdrive_in = xBox.getRawAxis(5); // these are supposed to be the vertical axes (for tank drive)
		rightdrive_in = xBox.getRawAxis(1); // checked
		*/
		
		
		leftdrive_in = -xBox.getRawAxis(5);//Temporarily reversing these for testing with SixChainz, uncomment above code for competition.
		rightdrive_in = -xBox.getRawAxis(1);
		
		runintake_in = xBox.getRawAxis(2) > 0.5; // LeftBumper / blue
		runshooter_in = xBox.getRawButton(5); // B / red
		rungear_in = xBox.getRawButton(4); // Y / yellow
		reverse_in = xBox.getRawButton(2); // left bumper (toggle)
		boost_in = xBox.getRawButton(6); // right bumper
		slow_in = xBox.getRawButton(9); // press left joystick
		straight_in = xBox.getRawButton(10); // press right joystick
		
		autogear_in = false; // these have no buttons on the joystick
		autoshoot_in = false;
		climb_in = false;
		
		try {// button panel is in here so when we disconnect it, it doesn't throw errors
			runintake_in |= bpanel.getRawButton(1); // TODO: change these indexes
			runshooter_in |= bpanel.getRawButton(2);
			rungear_in |= bpanel.getRawButton(3);
			reverse_in |= bpanel.getRawButton(4); // not on the button panel
			//slow_in |= bpanel.getRawButton(5);
			//straight_in |= bpanel.getRawButton(6);
			
			//autogear_in |= bpanel.getRawButton(4);
			//autoshoot_in |= bpanel.getRawButton(5);
			revclimb_in = bpanel.getRawButton(5);
			climb_in |= bpanel.getRawButton(6);
			
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
		
		double mult = Settings.get("motorproportion");
		
		if (boost_in) mult = 1; // full speed!
		
		if (reverse_in) { // we can drive backwards
			drive(-rightdrive_in, -leftdrive_in, mult); // drive backward
		} else {
			drive(leftdrive_in, rightdrive_in, mult); // drive function
		}
		
		runLauncher(runshooter_in); // run launcher if buttons are pressed
		highGoal(autoshoot_in); // auto aiming of the high goal shooter
		
		placeGear(rungear_in); // do the whole gear placement routine of the button is pressed
		
		intake(runintake_in); // run the intake motor if button is pressed
		
		if (climb_in) indexer.set(-1);
		if (revclimb_in) indexer.set(1);
	}
	
	/* the following useful functions:
	 * 
	 * drive(double left_joystick, double right_joystick) : moves the motors, handles deadzone and ideally reversing stuff
	 */
	
	//////////////////////////////// various management functions
	
	void drive(double left, double right) {
		drive(left,right,Settings.get("motorproportion"));
	}
	
	void drive(double left, double right, double multiplier) { // tank drive
		right = -right;
		
		if (Math.abs(left) > Settings.get("deadzone")) { // deadzone the motors
			leftdrive.set((Math.pow(left,3) * Settings.get("motormap") + (1 - Settings.get("motormap")) * left) * (multiplier + Settings.get("motordisparity"))); // cubic motor map
		} else {
			leftdrive.set(0); // else stop it
		}
		
		if (Math.abs(right) > Settings.get("deadzone")) { // deadzone
			rightdrive.set((Math.pow(right,3) * Settings.get("motormap") + (1 - Settings.get("motormap")) * right) * (multiplier - Settings.get("motordisparity")));
		} else {
			rightdrive.set(0); // else stop it
		}
	}
	
	void highGoal(boolean on) { // full 
		// need a state machine to handle the aiming and then driving
		if (on) {
			if (!StateMachine.isGroupRunning("boiler")) {
				StateMachine.start("aim_boiler"); // start the first stage if not already running
				vision_r();
			}
		} else { // as soon as the button is lifted, reset
			StateMachine.resetGroup("boiler");
		}
		
		if (StateMachine.isRunning("aim_boiler") && 
				(Math.abs(boiler_angle) < Settings.get("visiondeadzone"))) {
			StateMachine.cancel("aim_boiler");
			StateMachine.start("drive_boiler");
		} else if (StateMachine.isRunning("drive_boiler") && 
				(true /* check the distance */)) {
			StateMachine.cancel("drive_boiler");
			StateMachine.start("shoot_boiler");
		}
		
		if (StateMachine.isRunning("aim_boiler")) {
			aimLauncher();
		} else if (StateMachine.isRunning("drive_boiler")) {
			driveLauncher();
		} else if (StateMachine.isRunning("shoot_boiler")) {
			runLauncher(true);
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
			balllauncher.set(-launcherspeed); // set the new value
			
			agitator.set(Settings.get("agitatorspeed"));
		} else {
			balllauncher.set(0); // else, stop the motor
			indexer.set(0);
			agitator.set(0);
			StateMachine.resetGroup("indexer");
			indexerTimer.stop();
		}
	}
	
	double boiler_distance, boiler_angle;
	
	void aimLauncher() { // turn the robot to aim at the high goal
		vision_f();
		
		double turn_speed = 0;
		
		if (boiler_angle > Settings.get("visiondeadzone")) {
			turn_speed = Utility.map(Math.abs(boiler_angle),0,60,0.05,Settings.get("highaim-p"));
		} else if (boiler_angle < -Settings.get("visiondeadzone")) {
			turn_speed = -Utility.map(Math.abs(boiler_angle),0,60,0.05,Settings.get("highaim-p"));
		}
		
		drive(turn_speed, -turn_speed); // TODO: switch if backwards
	}
	
	void driveLauncher() {
		vision_f();
		
		double turn_speed = 0, drive_speed = 0;
		
		if (boiler_angle > Settings.get("visiondeadzone")) {
			turn_speed = Utility.map(Math.abs(boiler_angle),0,60,0.05,Settings.get("gearaim-p"));
		} else if (boiler_angle < -Settings.get("visiondeadzone")) {
			turn_speed = -Utility.map(Math.abs(boiler_angle),0,60,0.05,Settings.get("gearaim-p"));
		}
		
		drive_speed = Utility.map(Math.abs(Settings.get("idealboilerdist") - boiler_distance),0,Settings.get("maxboilerdist"),0.1,Settings.get("autodrivespd"));
		
		drive(drive_speed+turn_speed, drive_speed-turn_speed); // TODO: switch if backwards
	}
	
	void placeGear(boolean on) { // drive backward to place the gear while aiming
		// need a state machine to handle the aiming and then driving
		if (on) {
			vision_r();
			if (!StateMachine.isGroupRunning("gear")) {
				StateMachine.start("aim_gear"); // start the first stage if not already running
				vision_r();
			}
		} else { // as soon as the button is lifted, reset
			StateMachine.resetGroup("gear");
		}
		
		if (StateMachine.isRunning("aim_gear") && 
				(Math.abs(lift_angle) < Settings.get("visiondeadzone"))) {
			StateMachine.cancel("aim_gear");
			StateMachine.start("drive_gear");
		} else if (StateMachine.isRunning("drive_gear") && 
				rear_avg.getAverage() < Settings.get("liftdist")) {
			StateMachine.cancel("drive_gear");
			StateMachine.start("pause_gear");
		}
		
		if (StateMachine.isRunning("aim_gear")) {
			aimGear();
		} else if (StateMachine.isRunning("drive_gear")) {
			driveGear();
		} else if (StateMachine.isRunning("pause_gear")) {
			// probably do nothing
		}
		
	}
	
	double lift_distance, lift_angle;
	
	void aimGear() { // rotate the robot based on vision input
		vision_r();
		
		double turn_speed = 0;
		
		if (lift_angle > Settings.get("visiondeadzone")) {
			turn_speed = Utility.map(Math.abs(lift_angle),0,60,0.05,Settings.get("gearaim-p"));
		} else if (lift_angle < -Settings.get("visiondeadzone")) {
			turn_speed = -Utility.map(Math.abs(lift_angle),0,60,0.05,Settings.get("gearaim-p"));
		}
		
		drive(-turn_speed, turn_speed); // TODO: switch if backwards
	}
	
	void driveGear() { // drive forward slowly while aiming
		vision_r();
		
		double turn_speed = 0, drive_speed = 0;
		
		if (lift_angle > Settings.get("visiondeadzone")) {
			turn_speed = Utility.map(Math.abs(lift_angle),0,60,0.05,Settings.get("gearaim-p"));
		} else if (lift_angle < -Settings.get("visiondeadzone")) {
			turn_speed = -Utility.map(Math.abs(lift_angle),0,60,0.05,Settings.get("gearaim-p"));
		}
		
		drive_speed = Utility.map(Utility.constrain(Math.abs(lift_distance),0,Settings.get("maxgeardist")),0,Settings.get("maxgeardist"),0.1,Settings.get("autodrivespd"));
		
		drive(-drive_speed-turn_speed, -drive_speed+turn_speed); // TODO: switch if backwards
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
		vision_r();
		vision_f();
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
		
		updateArduino();
	}
	
	// this stuff for the arduino status messages
	double rear_vision = -100, front_vision = -100;
	
	void vision_r() { // set a timer to turn on the vision lights
		rear_vision = matchTimer.get();
	}
	void vision_f() {
		front_vision = matchTimer.get();
	}
	
	void updateArduino() {
		try {
			if (arduinoTimer.get() > Settings.get("arduinotimer")) { // send periodically to avoid buffer overflows
				byte mode1 = 0;  //////// structure: 0b<red><blue><fms><auton><teleop><disabled><enabled><attached>
				if (ds.getAlliance() == DriverStation.Alliance.Red)  mode1 |= 0b10000000; // on the red alliance
				if (ds.getAlliance() == DriverStation.Alliance.Blue) mode1 |= 0b01000000; // blue alliance 
				if (ds.isAutonomous() && ds.isEnabled())             mode1 |= 0b00100000; // auton mode
				if (ds.isOperatorControl() && ds.isEnabled())        mode1 |= 0b00010000; // teleop mode
				if (front_vision + 0.5 > matchTimer.get())           mode1 |= 0b00001000; // enable front vision leds
				if (rear_vision + 0.5 > matchTimer.get())            mode1 |= 0b00000100; // enable rear vision leds
				if (ds.isDisabled())                                 mode1 |= 0b00000010;

				byte[] mode2 = {mode1};
				arduino.write(mode2,1); // send the byte of status over
				
				arduinoTimer.reset(); // reset the timer so we can 
				arduinoTimer.start(); // probably don't need to do this
			}
		} catch (RuntimeException a) { }
	}
	
	void syncSensors() {
		lift_distance = lifttracker.getNumber("distanceFromTarget",0); // get the vision things
		lift_angle = lifttracker.getNumber("angleFromGoal",0);
		boiler_distance = boilertracker.getNumber("distanceFromTarget",0);
		boiler_angle = boilertracker.getNumber("angleFromGoal",0);
		
		try { // put data into table (probably disable this during comp)
			SensorPanel.report("ctl_v",ControllerPower.getInputVoltage()); // roborio voltage
			SensorPanel.report("range_f",front_avg.getAverage()); // averaged rangefinder value
			SensorPanel.report("range_r",rear_avg.getAverage()); // averaged rangefinder value
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
			SensorPanel.report("vis_bd",boiler_distance);
			SensorPanel.report("vis_ba",boiler_angle);
			SensorPanel.report("vis_ld",lift_distance);
			SensorPanel.report("vis_la",lift_angle);
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
		Settings.add("marginoferror", 10, 0, 150); // margin of error for the MovingAverage class when we call getAverage()
		Settings.add("launcherrpm", 3000, 3000, 4500); // rpm to keep the launcher at while it is shooting
		Settings.add("launcherdeadzone", 5, 0, 30); // deadzone at which to stop atjusting the motor input (+- rpm)
		Settings.add("launcher-p", 0.3, 0, 1); // rate at which to adjust the launcher speed (maybe switch to PID if this doesn't work)
		Settings.add("visiondeadzone", 4, 0, 90); // deadzone in degrees in which to aim at vision targets
		Settings.add("geardetect", 1000, 0, 4096); // analog read from the IR sensor to tell when the gear is present
		Settings.add("intakespeed", 1, 0, 1); // speed to run the intake motors
		Settings.add("arduinotimer",0.1,0.01,1); // arduino timer to send data in milliseconds
		Settings.add("gearaim-p", 0.2, 0, 1); // proportion for aiming the gear
		Settings.add("highaim-p", 0.2, 0, 1); // proportion for aiming the boiler goal
		Settings.add("autodrivespd", 0.4, 0, 1); // speed to drive at when automatically aiming
		Settings.add("idealboilerdist", 300, 0, 1000); // distance to boiler for auto aiming
		Settings.add("maxboilerdst", 300, 0, 1000);
		Settings.add("maxgeardist", 100, 0, 1000);
		Settings.add("boilerdst", 300, 0, 1000); // arbitrary units so drive from the boiler when shooting
		Settings.add("boilerdz", 3, 0, 40); // deadzone for the boiler distance
		Settings.add("indexerofftime", 0.4, 0.1, 3); // seconds to stop the indexer
		Settings.add("indexerontime", 0.04, 0, 0.5); // seconds to run the indexer
		Settings.add("indexerspeed", 0.7, 0, 1); // speed to run indexer motor
		Settings.add("agitatorspeed", 0.9, 0, 1); // speed to run the agitator motor
		Settings.add("motordisparity", 0.1, -0.5, 0.5); // number to use when correcting the disparity in traction on the tank drive
		// auto settings
		Settings.add("autonspeed", 0.5, 0, 1); // speed to drive in auton
		Settings.add("autondelay", 4, 0, 15); // delays during auton
		Settings.add("autondist1", 60, 0, 2500); // distance to drive in auto before turning (front rangefinder because the gear is on the back)
		Settings.add("autondist2", 100, 0, 2500); // distance to drive from the lift before shooting
		Settings.add("autonangle", 60, 0, 100); // angle to turn in auto when targeting the lifts
		Settings.add("liftdist", 20, 0, 250); // distance to get from the lift when placing a gear (rear rangefinder) (also used when auto-targeting in teleop)
		Settings.add("autonturnspeed", 0.4, 0, 1); // rate to turn in auto (very slow is good, but not too slow)
		
		// set up the sensors!
		SensorPanel.add("pwr_v", "PDB Voltage", SensorPanel.Type.BAR_STAT, 0, 15, "V");
		SensorPanel.add("ctl_v", "Controller Voltage", SensorPanel.Type.BAR_STAT, 0, 15, "V");
		SensorPanel.add("pwr_c", "PDB Total Current", SensorPanel.Type.BAR_STAT, 0, 500, "A");
		SensorPanel.add("pwr_t", "PDB Temperature", SensorPanel.Type.NUMBER, 0, 1, "F");
		for (int i = 0; i < 16; i++) SensorPanel.add("pwr_c_"+i, "PDB Current CH"+i, SensorPanel.Type.BAR_STAT, 0, 15, "A");
		SensorPanel.add("range_f", "Front Rangefinder", SensorPanel.Type.BAR, 0, 4000, "cm");
		SensorPanel.add("range_r", "Rear Rangefinder", SensorPanel.Type.BAR, 0, 4000, "cm");
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
		SensorPanel.add("vis_bd", "Boiler Distance", SensorPanel.Type.BAR, 0, 400, "in");
		SensorPanel.add("vis_ba", "Boiler Angle", SensorPanel.Type.CENTER, -90, 90, "deg");
		SensorPanel.add("vis_ld", "Lift Distance", SensorPanel.Type.BAR, 0, 40090, "in");
		SensorPanel.add("vis_la", "List Angle", SensorPanel.Type.CENTER, -90, 90, "deg");
		//SensorPanel.add("", "", SensorPanel.Type.BAR, 0, 1, "");
		
		// lay out the auton state machines (stages)
		StateMachine.add("initial_delay", 0.1); // start out by pausing for a moment
		StateMachine.add("drive_back_1"); // back up (gear on back) specified distance
		StateMachine.add("turn"); // rotate to face the peg if needed
		StateMachine.add("drive_back_2"); // drive again up to the peg (with vision)
		StateMachine.add("wait_gear"); // wait until the gear is lifted
		StateMachine.add("drive_fwd_3"); // drive up to the point where we can shoot
		StateMachine.add("aim_high"); // aim for the high goal with vision and then shoot
		//StateMachine.add("shoot"); // shoot as many balls as possible
		
		// statemachines to handle the gear placing 
		StateMachine.add("aim_gear","gear"); // turn so the vision targets are in the middle of the FOV
		StateMachine.add("drive_gear","gear"); // drive until the robot gets to the lift
		StateMachine.add("pause_gear","gear"); // wait (don't start another aiming run)
		
		//statemachines to control automatic shooting
		StateMachine.add("aim_boiler","boiler");
		StateMachine.add("drive_boiler","boiler");
		StateMachine.add("shoot_boiler","boiler");
		
		StateMachine.add("indexer_on","indexer");
		StateMachine.add("indexer_off","indexer");
		
		// set up auton chooser
		station = new SendableChooser<Station>(); // pretty simple to choose mode
		station.addDefault("Center Station", Station.CENTER); // default is center, where we just drive froward
		station.addObject("Left station", Station.LEFT); // we prefer to be on the side of the high goal so we can shoot
		station.addObject("Right station", Station.RIGHT); // however we want all options to be open
		station.addObject("Do Nothing", Station.HALT); // of we want, we can just cross the base line and stop
		SmartDashboard.putData("Choose Auton Starting Position",station);
	}
}