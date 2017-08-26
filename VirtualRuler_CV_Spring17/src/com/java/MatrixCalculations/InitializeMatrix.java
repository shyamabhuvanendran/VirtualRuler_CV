package com.java.MatrixCalculations;
/**
 * This class has method to initialize matrices and vectors
 * @author shyam
 *
 */

public class InitializeMatrix {
	
	public double[][] initializeMatrix3by3(double[][] A){
		double[][] B=new double[3][3];
		int i=0,j=0;
		
		for(i=0;i<3;i++){
			for(j=0;j<3;j++){
				B[i][j]=0.0;
			}
		}
		
		return B;
	}
	
	public double[][] initializeMatrix4by4(double[][] A){
		double[][] B=new double[4][4];
		int i=0,j=0;
		
		for(i=0;i<4;i++){
			for(j=0;j<4;j++){
				B[i][j]=0.0;
			}
		}
		
		return B;
	}
	
	public double[] initializeMatrix3by1(double[] A){
		double[] B=new double[3];
		int i=0;
		
		for(i=0;i<3;i++){
			B[i]=0.0;
		}
		
		return B;
	}

	public double[] initializeMatrix4by1(double[] c) {
		double[] B=new double[4];
		int i=0;
		
		for(i=0;i<4;i++){
			B[i]=0.0;
		}
		
		return B;
	}
	
	public int[] initializeMatrix2by1(int[] c) {
		int[] B=new int[2];
		int i=0;
		
		for(i=0;i<2;i++){
			B[i]=0;
		}
		
		return B;
	}


}
