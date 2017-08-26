package com.java.Camera;


/**
 * @author shyama
 * Constants Class
 * This class has camera calibration values obtained using MATLAB.
 * 
 */

public class CalibrationConstants {
	
	public static double[][] INTRINSICMATRIX__C= {{3.910822345298497e+03,0,0},
												  {0,3.819781106503626e+03,0},
												  {1.558715140129255e+03,1.449166214971588e+03,1}};
	
	public static double PIXELSIZE__C=1.2e-3;//in mm
	
	public static double[][] ROTATIONMATRIX__C= {{0.0998969612269511,0.768150688628856,0.106078176335808},
												{0.173718548302820,0.208933361601969,0.0144621077913455},
												{-0.0281211377120473,0.735573949503692,-0.0403294491274490}};
	
	
	public static double[] TRANSLATIONVECTOR__C={60.5741725144218,-53.0605936558261,820.285984863846};
	
	public static double[] FOCALLInPX__C={3.910822345298497e+03,3.819781106503626e+03}; //in pixels
	
	public static double[] PRINCIPALPOINT__C= {1558.71514012926, 1449.16621497159};
	
	
	public static double[] RADIALDISTCOEF__C={0.256023564557143,-0.413783597001224,0.00857507053713601,-0.0142977823450091};
	
	public static double[] TANGDISTCOEF__C={0.00857507053713601,-0.0142977823450091};
	
	
}
	
