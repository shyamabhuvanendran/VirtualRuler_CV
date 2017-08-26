package com.java.Camera;

import com.java.MatrixCalculations.InitializeMatrix;
import com.java.MatrixCalculations.MatrixMultiplication;
import com.java.MatrixCalculations.MatrixTranspose;

/**
 * This class calculates the camera matrix for Camera to World
 * @author shyama
 *
 */
public class CameraMatrix_C2W {
	private static double[][] extrinsicMatrix_c2w;
	private static double[] translation_c2w;
	private static double[][] rotation_c2w;
	private static double[][] cameraMatrix_c2w;
	private static double[][] intrinsicMatrix_c2w;
	
	//Calculates inverse of translation vector
	public void InverseTranslation(){
		MatrixMultiplication mM=new MatrixMultiplication();
		translation_c2w=mM.MatrixVectorMultiplication(rotation_c2w, CalibrationConstants.TRANSLATIONVECTOR__C);
		for(int i=0;i<3;i++){
			translation_c2w[i]=(-1)*translation_c2w[i];
		}
	}
	
	//Forms the extrinsic matrix using inverse of translation and transpose of rotation matrix
	public void formExtrinsicMatrix(){
		int i=0,j=0;
		
		for(i=0;i<3;i++){
			for(j=0;j<3;j++){
				if(j<3){
					extrinsicMatrix_c2w[i][j]=rotation_c2w[i][j];
				}
			}
		}
		for(i=0;i<3;i++){
			extrinsicMatrix_c2w[i][3]=translation_c2w[i];
		}
			extrinsicMatrix_c2w[3][3]=1.0;
	}
	
	//Forms the inverse intrinsic matrix
	public void formIntrinsicMatrix(){
		
		double PxByFX, PxByFY;
		PxByFX=(-1)/CalibrationConstants.FOCALLInPX__C[0];
		PxByFY=(-1)/CalibrationConstants.FOCALLInPX__C[0];
		intrinsicMatrix_c2w[0][0]=(-1)/(CalibrationConstants.FOCALLInPX__C[0]);
		intrinsicMatrix_c2w[1][1]=(-1)/(CalibrationConstants.FOCALLInPX__C[1]);
		intrinsicMatrix_c2w[0][2]=(CalibrationConstants.PRINCIPALPOINT__C[0])*PxByFX;
		intrinsicMatrix_c2w[1][2]=(CalibrationConstants.PRINCIPALPOINT__C[1])*PxByFY;
		intrinsicMatrix_c2w[2][2]=1.0;
		intrinsicMatrix_c2w[3][3]=1.0;
		
		
	}
	
	//forms the camera matrix
	public void formCameraMatrix(){
		MatrixMultiplication mM=new MatrixMultiplication();
		cameraMatrix_c2w=mM.MultiplyMatrices(extrinsicMatrix_c2w,intrinsicMatrix_c2w);
	}
	
	/**
	 * Class Constructor
	 */
	public CameraMatrix_C2W() {
		super();
		extrinsicMatrix_c2w = new double[4][4];
		translation_c2w = new double[3];
		rotation_c2w = new double[3][3];
		cameraMatrix_c2w = new double[4][4];
		intrinsicMatrix_c2w = new double[4][4];
		
		InitializeMatrix iN=new InitializeMatrix();
		extrinsicMatrix_c2w=iN.initializeMatrix4by4(extrinsicMatrix_c2w);
		translation_c2w=iN.initializeMatrix3by1(translation_c2w);
		rotation_c2w=iN.initializeMatrix4by4(rotation_c2w);
		cameraMatrix_c2w=iN.initializeMatrix4by4(cameraMatrix_c2w);
		intrinsicMatrix_c2w=iN.initializeMatrix4by4(intrinsicMatrix_c2w);
		
		MatrixTranspose mT=new MatrixTranspose();
		rotation_c2w=mT.matrixTranspose(CalibrationConstants.ROTATIONMATRIX__C);
		
		InverseTranslation();
		formExtrinsicMatrix();
		formIntrinsicMatrix();

	}
	
	/**
	 * Getters are Setters
	 * @return
	 */
	public double[][] getextrinsicMatrix_c2w() {
		return extrinsicMatrix_c2w;
	}

	public double[] getTranslation_c2w() {
		return translation_c2w;
	}

	public double[][] getRotation_c2w() {
		return rotation_c2w;
	}

	public double[][] getCameraMatrix_c2w() {
		return cameraMatrix_c2w;
	}

	public double[][] getIntrinsicMatrix_c2w() {
		return intrinsicMatrix_c2w;
	}

}
