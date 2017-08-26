package com.java.Camera;

import com.java.MatrixCalculations.InitializeMatrix;
import com.java.MatrixCalculations.MatrixMultiplication;

/**
 * @author shyama
 *This class has all the methods to convert World points to corresponding pixels.
 */
public class CameraMatrix_W2C {
	private double[][] extrinsicMatrix_w2c;
	private double[][] cameraMatrix_w2c;
	private double[][] intrinsicMatrix_w2c;
	
	public void formExtrinsicMatrix(){
		int i=0,j=0;
		
		for(i=0;i<3;i++){
			for(j=0;j<3;j++){
				if(j<3){
					this.extrinsicMatrix_w2c[i][j]=CalibrationConstants.ROTATIONMATRIX__C[i][j];
				}
			}
		}
		for(i=0;i<3;i++){
			this.extrinsicMatrix_w2c[i][3]=CalibrationConstants.TRANSLATIONVECTOR__C[i];
		}
			this.extrinsicMatrix_w2c[3][3]=1.0;
	}
	
	/**
	 * This method forms the intrinsic matrix.
	 */
	public void formIntrinsicMatrix(){
		
		this.intrinsicMatrix_w2c[0][0]=(-1)*CalibrationConstants.FOCALLInPX__C[0];
		this.intrinsicMatrix_w2c[1][1]=(-1)*CalibrationConstants.FOCALLInPX__C[1];
		this.intrinsicMatrix_w2c[0][2]=CalibrationConstants.PRINCIPALPOINT__C[0];
		this.intrinsicMatrix_w2c[1][2]=CalibrationConstants.PRINCIPALPOINT__C[1];
		this.intrinsicMatrix_w2c[2][2]=1.0;
		this.intrinsicMatrix_w2c[3][3]=1.0;
		
		
	}
	
	/**
	 * This method computes camera matrix
	 */
	public void formCameraMatrix(){
		
		MatrixMultiplication mM=new MatrixMultiplication();
		this.cameraMatrix_w2c=mM.MultiplyMatrices(this.intrinsicMatrix_w2c,this.extrinsicMatrix_w2c);
	}
	
	/**
	 * This method computes the Camera Matrix
	 */
	public CameraMatrix_W2C() {
		super();
		
		this.extrinsicMatrix_w2c=new double[4][4];
		this.cameraMatrix_w2c=new double[4][4];
		this.intrinsicMatrix_w2c=new double[4][4];
		
		InitializeMatrix iN=new InitializeMatrix();
		this.extrinsicMatrix_w2c = iN.initializeMatrix4by4(extrinsicMatrix_w2c);
		this.cameraMatrix_w2c = iN.initializeMatrix4by4(cameraMatrix_w2c);
		this.intrinsicMatrix_w2c = iN.initializeMatrix4by4(intrinsicMatrix_w2c);
		formExtrinsicMatrix();
		formIntrinsicMatrix();
	}

	public double[][] getExtrinsicMatrix() {
		return extrinsicMatrix_w2c;
	}

	public void setExtrinsicMatrix(double[][] extrinsicMatrix_w2c) {
		this.extrinsicMatrix_w2c = extrinsicMatrix_w2c;
	}
	
	public double[][] getIntrinsicMatrix_w2c() {
		return intrinsicMatrix_w2c;
	}

	public void setIntrinsicMatrix_w2c(double[][] intrinsicMatrix_w2c) {
		this.intrinsicMatrix_w2c = intrinsicMatrix_w2c;
	}

	public double[][] getCameraMatrix() {
		return cameraMatrix_w2c;
	}

	public void setCameraMatrix(double[][] cameraMatrix_w2c) {
		this.cameraMatrix_w2c = cameraMatrix_w2c;
	}

	public double[][] getIntrinsicMatrix() {
		return intrinsicMatrix_w2c;
	}

	public void setIntrinsicMatrix(double[][] intrinsicMatrix_w2c) {
		this.intrinsicMatrix_w2c = intrinsicMatrix_w2c;
	}
	
	
}
