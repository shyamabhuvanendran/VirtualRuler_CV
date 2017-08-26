package com.java.MatrixCalculations;

import org.opencv.core.Mat;

/**
 * This class has method that converts double arrays to OpenCV object Mat.
 * @author shyam
 *
 */
public class convertDoubleToMat {

	public static Mat convertToDbl(double[][] A){
		Mat B=new Mat();
		int size=A.length*A.length;
		double[] aDbl=new double[size];
		int k=0;
		for(int i=0;i<A.length;i++){
			for(int j=0;j<A.length;j++){
				aDbl[k]=(float)A[i][j];
				System.out.println(aDbl[k]);
				k++;
			}
		}
		B.put(4, 4, aDbl);
		//System.out.println("B: "+B.dump());
		return B;
	}
}
