package com.java.Image;


import com.java.Camera.CameraMatrix_W2C;
import com.java.MatrixCalculations.MatrixMultiplication;

/**
 * This class has methods which converts 3D world coordinates to image pixels using Camera Matrix
 * @author shyama
 *
 */
public class ConvertToImagePixel {

	public static double[] point_im;
	public static double[] cameraPoint;
	public static double[] worldPoint;
	
	/**
	 * Class Constructor
	 */
	public ConvertToImagePixel() {
		super();
		point_im=new double[3];
		cameraPoint=new double[4];
		worldPoint=new double[4];
		findPixelCoordinates();
	}
	
	/**
	 * This method multiplies world points with camera matrix to get corresponding pixel points
	 */
	public void findPixelCoordinates(){
		CameraMatrix_W2C cM=new CameraMatrix_W2C();
		/*for(int i=0;i<3;i++){
			worldPoint[i]=CalibrationConstants.SAMPLEWORLDPOINT__C[i];
		}*/
		worldPoint[3]=1.0;
		MatrixMultiplication mM=new MatrixMultiplication();
		cM.formCameraMatrix();
		cameraPoint=mM.MatrixVectorMultiplication(cM.getCameraMatrix(),worldPoint);
		point_im[0]=(int) (cameraPoint[0]/cameraPoint[2]);
		point_im[1]=(int) (cameraPoint[1]/cameraPoint[2]);
		point_im[2]=1;
	}

	public double[] getPoint_im() {
		return point_im;
	}
	
}
