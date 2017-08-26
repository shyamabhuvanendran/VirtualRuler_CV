package com.java.MatrixCalculations;

/**
 * This class has methods to display double matrices and vectors.
 * @author shyama
 *
 */
public class DisplayMatrix {

	public void displayMatrix(double[][] A){
		int i=0,j=0;
		System.out.println();
		for(i=0;i<A.length;i++){
			for(j=0;j<A.length;j++){
				System.out.print(A[i][j]);
				System.out.print("\t\t");
			}
			System.out.println();
		}
		
		System.out.println();
	}
	
	
	public void displayVector(double[] A){
		int i=0;
		System.out.println();
		for(i=0;i<A.length;i++){
				System.out.print(A[i]);
				System.out.print("\t\t");
		}
		
		System.out.println();
	}
	
	public void displayVectorInt(int[] A){
		int i=0;
		System.out.println();
		for(i=0;i<A.length;i++){
				System.out.print(A[i]);
				System.out.print("\t\t");
		}
		
		System.out.println();
	}
}
