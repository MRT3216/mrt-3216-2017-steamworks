package org.usfirst.frc.team3216.robot;

public class MovingAverage {
	/// decided to move this into its own class
	int numreadings; // number of samples to smooth
	double[] readings; // stores the values from the rangefinder
	int index; // location within the array to store the value
	double r_average; // smoothed value
	double total; // running total
	double latest;
	
	MovingAverage(int num) {
		this.numreadings = num; // init everything
		this.readings = new double[numreadings];
		this.index = 0;
		this.r_average = 0;
		this.total = 0;
		this.latest = 0;
	}
	
	MovingAverage(int num, double init) { // starts the average at a specified value to avoid the slope up from zero in the first few seconds
		this.numreadings = num;
		this.readings = new double[numreadings];
		this.index = 0;
		this.r_average = init;
		for (int i = 0; i < this.numreadings; i++) this.readings[i] = init; // set to initial value 
		this.total = numreadings * init;
		this.latest = init;
	}
	
	double getAverage() {
		return this.r_average;
	}
	
	double getLatest() {
		// if the value is within a margin of error, return it. otherwise, return the average.
		// this always allows us to have the latest value without having big spikes
		double moe = Math.abs(this.latest - this.r_average) / this.r_average;
		if (moe < Settings.get("marginoferror")) {
			return this.latest;
		} else {
			return this.r_average;
		}
		
	}
	
	void newSample(double sample) {
		this.latest = sample;
		this.total -= this.readings[this.index];         //subtract the last reading
		this.readings[this.index] = sample;              //place value from sensor
		this.total += this.readings[this.index];         //add the reading to the total
		this.index++;                                    //advance to the next position in the array
		this.index %= this.numreadings;                  //if its at the end of the array, wrap around to the beginning
		this.r_average = this.total / (double)this.numreadings;    //calculate the average
	}
}
