package com.java.MatrixCalculations;

/**
 * This class had method to transpose a double matrix
 * @author shyam
 *
 */
public class MatrixTranspose {
	public double[][] matrixTranspose(double[][] A){
		double[][] B=new double[3][3];
		int i=0,j=0;
		
		InitializeMatrix iM=new InitializeMatrix();
		iM.initializeMatrix3by3(B);
		
		for(i=0;i<3;i++){
	        for(j=0;j<3;j++){
	        	B[i][j]=A[j][i];
	        }
	    }
		
		return B;
	}
}
