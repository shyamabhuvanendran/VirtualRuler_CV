package com.java.World;

import com.java.Camera.CalibrationConstants;
import com.java.Camera.CameraMatrix_C2W;
import com.java.Image.ConvertToImagePixel;
import com.java.MatrixCalculations.MatrixMultiplication;

/**
 * This class has methods to convert image pixel points to world coordinates
 * @author shyama
 *
 */
public class ImageToWorld {
	private static double[] imagePoints;
	private static double[] imgPtsAftrScSh;
	private static double[] cameraPoints;
	private static double[] worldPoints;
	private static double[][] intrinsicMatrix;
	private static double[][] extrinsicMatrix;

	public void convertToWorld(){
		MatrixMultiplication mM=new MatrixMultiplication();
		for (int i = 0; i < 3; i++) { 
		    for (int j = 0; j < 3; j++) { 
		    	cameraPoints[i] += intrinsicMatrix[i][j]*imagePoints[j];
		    	//System.out.println("Intrinsic value "+intrinsicMatrix[i][j]+"   Image Value "+imagePoints[j]+"   Multiplied Value"+intrinsicMatrix[i][j]*imagePoints[j]);
		    }
		    System.out.println("Added Value "+cameraPoints[i]);
		}
		
	  //  cameraPoints=mM.MatrixVectorIntMultiplication(intrinsicMatrix, imagePoints);
	    //intrinsicMatrix[i][j]*mP.getPoint_im()[j];
		//this.cameraPoints=mM.MatrixVectorMultiplication3(intrinsicMatrix, this.imagePoints);
		//cameraPoints[3]=1;
		//worldPoints=mM.MatrixVectorMultiplication(extrinsicMatrix, cameraPoints);
		CameraMatrix_C2W cW=new CameraMatrix_C2W();
		worldPoints=mM.MatrixVectorMultiplication(cW.getRotation_c2w(), cameraPoints);
		for(int i=0;i<3;i++){
			worldPoints[i]=worldPoints[i]+cW.getTranslation_c2w()[i];
		}
		 
	}
	
	public void convertToWorldEquations(){
		MatrixMultiplication mM=new MatrixMultiplication();
		System.out.println("Phew! Finally");
		worldPoints= mM.MatrixVectorMultiplication(extrinsicMatrix, imgPtsAftrScSh);
	}

	
	public double[] getImgPtsAftrScSh() {
		return imgPtsAftrScSh;
	}

	public void setImgPtsAftrScSh(double[] imgPtsAftrScSh) {
		ImageToWorld.imgPtsAftrScSh = imgPtsAftrScSh;
	}

	public ImageToWorld() {
		super();
		imagePoints = new double[3];;
		cameraPoints = new double[3];
		worldPoints = new double[3];
		intrinsicMatrix=new double[3][3];
		extrinsicMatrix=new double[4][4];
		imgPtsAftrScSh=new double[3];
		
		imagePoints=ConvertToImagePixel.point_im;
		imgPtsAftrScSh[0]=(imagePoints[0]-CalibrationConstants.PRINCIPALPOINT__C[0])*((-1)*(CalibrationConstants.PIXELSIZE__C));
		imgPtsAftrScSh[1]=(imagePoints[1]-CalibrationConstants.PRINCIPALPOINT__C[1])*((-1)*(CalibrationConstants.PIXELSIZE__C));
		imgPtsAftrScSh[2]=CalibrationConstants.FOCALLInPX__C[0];
		
		CameraMatrix_C2W cM=new CameraMatrix_C2W();
		for(int i=0;i<3;i++){
			for(int j=0;j<3;j++){
				intrinsicMatrix[i][j]=cM.getIntrinsicMatrix_c2w()[i][j];
			}
		}
		
		for(int i=0;i<4;i++){
			for(int j=0;j<4;j++){
				extrinsicMatrix[i][j]=cM.getextrinsicMatrix_c2w()[i][j];
			}
		}
		
	}


	public double[] getImagePoints() {
		return imagePoints;
	}


	public double[] getCameraPoints() {
		return cameraPoints;
	}


	public double[] getWorldPoints() {
		return worldPoints;
	}


	public double[][] getIntrinsicMatrix() {
		return intrinsicMatrix;
	}


	public void setIntrinsicMatrix(double[][] intrinsicMatrix) {
		ImageToWorld.intrinsicMatrix = intrinsicMatrix;
	}


	public double[][] getExtrinsicMatrix() {
		return extrinsicMatrix;
	}


	public void setExtrinsicMatrix(double[][] extrinsicMatrix) {
		ImageToWorld.extrinsicMatrix = extrinsicMatrix;
	}
	
	
}
