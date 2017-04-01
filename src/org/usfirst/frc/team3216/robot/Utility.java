package org.usfirst.frc.team3216.robot;

public class Utility {
	enum Alliance { RED, BLUE } // assymetric field means we need different auto for the red and blue sides
	enum Station { LEFT, RIGHT, CENTER, HALT } // also different auto based on what station we're in front of
	
	public static double map(double value, double istart, double istop, double ostart, double ostop) { // to map stuff from one range to another
		return ostart + (ostop - ostart) * ((value - istart) / (istop - istart));
	}
	
	public static double constrain(double value, double imin, double imax) {
		return Math.min(Math.max(value,imin),imax);
	}
}
