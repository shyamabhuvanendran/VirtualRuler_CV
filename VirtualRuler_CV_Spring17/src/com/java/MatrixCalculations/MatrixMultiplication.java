package com.java.MatrixCalculations;

/**
 * This class has method to perform multiplication on vectors and matrices
 * @author shyam
 *
 */
public class MatrixMultiplication {
	
	public double[][] MultiplyMatrices(double[][] A, double[][] B){
		double[][] C=new double[4][4];
		int i=0,j=0,k=0;
		
		InitializeMatrix iN=new InitializeMatrix();
		C=iN.initializeMatrix4by4(C);
		
		for (i = 0; i < A.length; i++) { 
		    for (j = 0; j < B.length; j++) { 
		        for (k = 0; k < A[0].length; k++) { 
		            C[i][j] += A[i][k] * B[k][j];
		        }
		    }
		}
		return C;
	}
	
	public double[] MatrixVectorMultiplication(double[][] A, double[] B){
		int m=B.length;
		double[] C=new double[m];
		int i=0,j=0;
		
		InitializeMatrix iN=new InitializeMatrix();
		C=iN.initializeMatrix4by1(C);
		
		for (i = 0; i < A.length; i++) { 
		    for (j = 0; j < B.length; j++) { 
		            C[i] += A[i][j] * B[j];
		    }
		}
		return C;
	}
	
	public double[] MatrixVectorIntMultiplication(double[][] A, int[] B){
		int m=B.length;
		double[] C=new double[m];
		int i=0,j=0;
		
		InitializeMatrix iN=new InitializeMatrix();
		C=iN.initializeMatrix4by1(C);
		
		for (i = 0; i < A.length; i++) { 
		    for (j = 0; j < B.length; j++) { 
		            C[i] += A[i][j] * B[j];
		    }
		}
		return C;
	}

}
