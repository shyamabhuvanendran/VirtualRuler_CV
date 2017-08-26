package com.java.MatrixCalculations;

/**
 * This class has methods to perform matrix addition on double arrays
 * @author shyama
 *
 */
public class MatrixAddition {
	
	public double[][] twoMatrixAddition(double[][] A, double[][] B){
		double[][] C=new double[3][3];
		int i=0,j=0;
		
		for(i=0;i<3;i++){
			for(j=0;j<3;j++){
				C[i][j]=0.0;
			}
		}
		
		for(i=0;i<3;i++){
	        for(j=0;j<3;j++)
	        	C[i][j]=A[i][j]+B[i][j];
	    }
		
		return C;
	}

}
