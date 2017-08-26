package com.java.ImageProcess;


import java.text.DecimalFormat;
import java.util.ArrayList;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point;
import org.opencv.core.Point3;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

import com.java.Camera.CalibrationConstants;
import com.java.Camera.CameraMatrix_C2W;
import com.java.Camera.CameraMatrix_W2C;

/**
 * This class implements the Reference Object method to find object dimensions
 * @author shyama
 *
 */
public class ReferenceObject {
	private Mat source;
	private Mat edgeDetected;
	private Mat grayScaled;
	private Mat blurred;
	private Mat dialated;
	private Mat eroded;
	private Point vertices[];
	private ArrayList<Point> verticesList;
	private Point refVertices[];
	private ArrayList<Point> refVerticesList;
	private java.util.List<MatOfPoint> contours;
	private Mat extrinsic;
	private Mat intrinsicMat;
	private Mat intrinsicInv; 
	private MatOfPoint2f imagePoints;
	private MatOfPoint3f worldPoints;
	private MatOfDouble radDist;
	private Scalar colorRed;
	private Scalar colorBlue;
	private Scalar colorWhite;
	private Mat rvec;
	private Mat translation;
	private Mat Rotation;
	private Mat Z;
	private Mat cameraPoints1;
	private Mat cameraPoints2;
	private Mat cameraPoints3;
	private Mat cameraPoints4;
	private Mat refLength;
	private Mat refBreadth;
	
	/**
	 * This method starts the Reference Object Method.
	 */
	public void referenceObjectMethod(){
 	   	int refId=0;    
 	   	ArrayList<Point3> pointsList = new ArrayList<Point3>();
 	   	
 	   	//Initialize object points with the known dimensions of the reference object.
 	   	Point3[] objectPoints= {new Point3(0.0,0.0,0.0), new Point3(-50.0,0.0,0.0),new Point3(-50.0,88.0,0.0),new Point3(0.0,88.0,0.0)};       //8.8cm and 5cm - village magnet (Reference Object) dimensions
 		System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
 		
 		System.out.println();
 		System.out.println("Starting Reference Object Approach");
	   	System.out.println("***********************************");
 	   	
	   	//Preprocess the image
        imagePreProcess(source);
        
        //Detect the reference object in the image
        refId=findReferenceObject();
        //Get the vertices of minimum bounding rectangle for the reference object
        MatOfPoint2f refContourPoints = new MatOfPoint2f(contours.get(refId).toArray());
        getRefVertices(refContourPoints);
       
        imagePoints.fromList(refVerticesList);
 	   	
        for( int j=0;j<4;j++){
 		   pointsList.add(objectPoints[j]);
 	   	}
        
        
        worldPoints.fromList(pointsList);
        //Compute the extrinsic matrix for the image
        formExtrinsicMatrix(); 
        //Compute Z
    	computeZ();
    	//Find the reference object dimensions. This is just to verify if this method works fine.
    	findReferenceObjectDimensions();
    	//Find dimensions of other objects in the image
    	findOtherObjectDimensions(refId);
    	
    	System.out.println("Reference Method Completed! Check the Resources/Images/ReferenceResults.JPG for results.");
    	
	}
	

	/**
	 * This method computes the dimensions of all objects except the reference object in the image
	 * @param refId
	 */
	private void findOtherObjectDimensions(int refId) {
		
		Mat imagePoints1=new Mat(3,1,6);
 	   	Mat imagePoints2=new Mat(3,1,6);
 	   	Mat imagePoints3=new Mat(3,1,6);
 	   	Mat imagePoints4=new Mat(3,1,6);
 	   	Mat otherCameraPoints1=new Mat(4,0,6);
 	   	Mat otherCameraPoints2=new Mat(4,0,6);
 	   	Mat otherCameraPoints3=new Mat(4,0,6);
 	   	Mat otherCameraPoints4=new Mat(4,0,6);
 	   	Mat side1Difference=new Mat(3,3,6);
 	   	Mat side1DifferenceX=new Mat(1,1,6);
 	   	Mat side1DifferenceY=new Mat(1,1,6);
 	   	Mat side2Difference=new Mat(3,3,6);
 	   	Mat side2DifferenceX=new Mat(1,1,6);
 	   	Mat side2DifferenceY=new Mat(1,1,6);
 	   	int noOfOtherObjects=0;
 	   	
 	   	
		for(int i=0;i<contours.size();i++){
    		
    		ArrayList<Point> otherVerticesList=new ArrayList<Point>();
    		MatOfPoint2f otherImagePoints=new MatOfPoint2f();
    		MatOfPoint2f[] writePoints=new MatOfPoint2f[2];
    		Mat objLength=new Mat(1,1,6);
    		Mat objBreadth=new Mat(1,1,6);
    		Point p1=new Point();
		 	Point p2=new Point();
		 	double length=0.0;
		 	double breadth=0.0;
    		
		 	//Removing contours that have very less contour area
    		if(Imgproc.contourArea(contours.get(i))<10000){
    			continue;
    		}
    		
    		MatOfPoint2f contourPoints = new MatOfPoint2f(contours.get(i).toArray());
    		if(i!=refId){
    			noOfOtherObjects++;
    			//Get the vertices of minimum bounding rectangle
    			getVertices(contourPoints);
    			//draw the minimum bounding rectangles around the object
    			drawBoundingRectangles(vertices, "nonRef");
    			for(int j=0;j<4;j++){
    				otherVerticesList.add(vertices[j]);
    			}
    			
    			otherImagePoints.fromList(otherVerticesList);
    			
    			imagePoints1.put(0,0,otherImagePoints.get(0, 0));
    			imagePoints1.put(2, 0, 1);
    			imagePoints2.put(0,0,otherImagePoints.get(1, 0));
    			imagePoints2.put(2, 0, 1);
    			imagePoints3.put(0,0,otherImagePoints.get(2, 0));
    			imagePoints3.put(2, 0, 1);
    			imagePoints4.put(0,0,otherImagePoints.get(3, 0));
    			imagePoints4.put(2, 0, 1);	
    			
    			//Multiply the pixel points with inverse intrinsic matrix to get the camera points.
    			Core.gemm(intrinsicInv, imagePoints1, 1, new Mat(), 0, otherCameraPoints1);
    			Core.gemm(intrinsicInv, imagePoints2, 1, new Mat(), 0, otherCameraPoints2);
    			Core.gemm(intrinsicInv, imagePoints3, 1, new Mat(), 0, otherCameraPoints3);
    			Core.gemm(intrinsicInv, imagePoints4, 1, new Mat(), 0, otherCameraPoints4);
    			
    			//Multiply with the computed Z
    			Core.multiply(otherCameraPoints1, Z, otherCameraPoints1);
    		 	Core.multiply(otherCameraPoints2, Z, otherCameraPoints2);
    		 	Core.multiply(otherCameraPoints3, Z, otherCameraPoints3);
    		 	Core.multiply(otherCameraPoints4, Z, otherCameraPoints4);		
    			
    		 	//Use distance formula to calculate object dimensions
    			Core.subtract(otherCameraPoints1, otherCameraPoints2, side1Difference);
    		 	side1DifferenceX.put(0, 0, side1Difference.get(0, 0));
    		 	side1DifferenceY.put(0, 0, side1Difference.get(1, 0));
    		 	Core.magnitude(side1DifferenceX, side1DifferenceY, objLength);
    		 	  
    		 	Core.subtract(otherCameraPoints2, otherCameraPoints3, side2Difference);
    		 	side2DifferenceX.put(0, 0, side2Difference.get(0, 0));
    		 	side2DifferenceY.put(0, 0, side2Difference.get(1, 0));
    		 	Core.magnitude(side2DifferenceX, side2DifferenceY, objBreadth);	
    		 	
    		 	//Get write points to input the computed dimensions in the resulting image
    		 	writePoints=getLocationToPutDimensions(otherVerticesList);
    		 	
    		 	p1.set(writePoints[0].get(0, 0));
    		 	p2.set(writePoints[1].get(0, 0));
    		 	
    		 	length=roundTwoDecimals(objLength.get(0, 0)[0]);
    		 	breadth=roundTwoDecimals(objBreadth.get(0, 0)[0]);
    		
    		 	System.out.println("Length of Object "+noOfOtherObjects+" : "+length+"mm");
    		 	System.out.println("Breadth of Object "+noOfOtherObjects+" : "+breadth+"mm");
    		 	
    		 	Imgproc.putText(source, Double.toString(length), p1, 5, 1.8, colorWhite,2);
    		 	Imgproc.putText(source, Double.toString(breadth), p2, 5, 1.8, colorWhite,2);
    		 	
    		 	System.out.println();
    		 	
    		}
    	}
		
		Imgcodecs.imwrite("./Resources/Images/ReferenceResults.JPG",source);
	}
	

	/**
	 * This method rounds double values to two decimal places
	 * @param d
	 * @return
	 */
	public double roundTwoDecimals(double d) {
	    DecimalFormat twoDForm = new DecimalFormat("#.##");
	    return Double.valueOf(twoDForm.format(d));
	}
	
	
	/**
	 * This method gets the pixels points to input the computed dimensions near their respective objects
	 * @param otherVerticesList
	 * @return
	 */
	public MatOfPoint2f[] getLocationToPutDimensions(ArrayList<Point> otherVerticesList) {

		MatOfPoint2f writePoints1=new MatOfPoint2f();
		MatOfPoint2f writePoints2=new MatOfPoint2f();
		MatOfPoint2f writePoints3=new MatOfPoint2f();
		MatOfPoint2f writePoints4=new MatOfPoint2f();
		
		MatOfPoint2f[] writePoints=new MatOfPoint2f[2];
	
		writePoints1.fromArray(otherVerticesList.get(0));
		writePoints2.fromArray(otherVerticesList.get(1));
		writePoints3.fromArray(otherVerticesList.get(2));
		writePoints4.fromArray(otherVerticesList.get(3));
		
		Core.add(writePoints3, writePoints4, writePoints1);
		Core.divide(writePoints1, new Scalar(2,2), writePoints1);
		
		Core.add(writePoints2, writePoints3, writePoints3);
		Core.divide(writePoints3, new Scalar(2,2), writePoints3);
	
		writePoints[0]=writePoints1;
		writePoints[1]=writePoints3;
		
		return writePoints;
	}


	/**
	 * To find the reference object dimensions. This method is to test.
	 */
	public void findReferenceObjectDimensions() {
		// TODO Auto-generated method stub
		
	   Mat side1Difference=new Mat(3,3,6);
	   Mat side1DifferenceX=new Mat(1,1,6);
	   Mat side1DifferenceY=new Mat(1,1,6);
 	   Mat side2Difference=new Mat(3,3,6);
 	   Mat side2DifferenceX=new Mat(1,1,6);
	   Mat side2DifferenceY=new Mat(1,1,6);
	   double referenceLength=0.0;
	   double referenceBreadth=0.0;
	   
	   //Multiply camera points with the computed Z
	   Core.multiply(cameraPoints1, Z, cameraPoints1);
 	   Core.multiply(cameraPoints2, Z, cameraPoints2);
 	   Core.multiply(cameraPoints3, Z, cameraPoints3);
 	   Core.multiply(cameraPoints4, Z, cameraPoints4);
 	   
 	   //Compute the dimensions using distance formula
 	   Core.subtract(cameraPoints1, cameraPoints2, side1Difference);
 	   side1DifferenceX.put(0, 0, side1Difference.get(0, 0));
 	   side1DifferenceY.put(0, 0, side1Difference.get(1, 0));
 	   Core.magnitude(side1DifferenceX, side1DifferenceY, refLength);
 	   
 	   Core.subtract(cameraPoints2, cameraPoints3, side2Difference);
 	   side2DifferenceX.put(0, 0, side2Difference.get(0, 0));
 	   side2DifferenceY.put(0, 0, side2Difference.get(1, 0));
 	   Core.magnitude(side2DifferenceX, side2DifferenceY, refBreadth);
 	   
 	   referenceLength=roundTwoDecimals(refLength.get(0, 0)[0]);
 	   referenceBreadth=roundTwoDecimals(refBreadth.get(0, 0)[0]);
 	   
	  /* System.out.println("Length of Reference Object : "+referenceLength+"mm");
 	   System.out.println("Breadth of Reference Object : "+referenceBreadth+"mm");*/
 	   System.out.println();

	}


	/**
	 * This method computes Z using extrinsic matrix and the assumed world points
	 */
	public void computeZ() {

	   Mat imagePoints1=new Mat(3,1,6);
 	   Mat imagePoints2=new Mat(3,1,6);
 	   Mat imagePoints3=new Mat(3,1,6);
 	   Mat imagePoints4=new Mat(3,1,6);
 	   	
 	   Mat objPntsMat1=new Mat(3,1,6);
	   Mat objPntsMat2=new Mat(3,1,6);
	   Mat objPntsMat3=new Mat(3,1,6);
	   Mat objPntsMat4=new Mat(3,1,6);
	   
	   Mat cameraProjPoints1=new Mat(4,0,6);
	   Mat cameraProjPoints2=new Mat(4,0,6);
	   Mat cameraProjPoints3=new Mat(4,0,6);
	   Mat cameraProjPoints4=new Mat(4,0,6);
	   
	   objPntsMat1.put(0, 0, worldPoints.get(0, 0));
	   objPntsMat2.put(0, 0, worldPoints.get(1, 0));
	   objPntsMat3.put(0, 0, worldPoints.get(2, 0));
	   objPntsMat4.put(0, 0, worldPoints.get(3, 0));
	  
	   //Project the world points to the camera coordinate system using extrinsic matrix
	   Core.gemm(Rotation, objPntsMat1, 1, new Mat(), 0, cameraProjPoints1);
	   Core.add(cameraProjPoints1, translation, cameraProjPoints1);
	   
	   Core.gemm(Rotation, objPntsMat2, 1, new Mat(), 0, cameraProjPoints2);
	   Core.add(cameraProjPoints2, translation, cameraProjPoints2);
	   
	   Core.gemm(Rotation, objPntsMat3, 1, new Mat(), 0, cameraProjPoints3);
	   Core.add(cameraProjPoints3, translation, cameraProjPoints3);
	   
	   Core.gemm(Rotation, objPntsMat4, 1, new Mat(), 0, cameraProjPoints4);
	   Core.add(cameraProjPoints4, translation, cameraProjPoints4);	   
	   
	   imagePoints1.put(0,0,imagePoints.get(0, 0));
	   imagePoints1.put(2, 0, 1);
	   imagePoints2.put(0,0,imagePoints.get(1, 0));
	   imagePoints2.put(2, 0, 1);
	   imagePoints3.put(0,0,imagePoints.get(2, 0));
	   imagePoints3.put(2, 0, 1);
	   imagePoints4.put(0,0,imagePoints.get(3, 0));
	   imagePoints4.put(2, 0, 1);
	   
	   //Re-project the image points to the camera coordinate system using inverse intrinsic matrix 
	   Core.gemm(intrinsicInv, imagePoints1, 1, new Mat(), 0, cameraPoints1);
	   Core.gemm(intrinsicInv, imagePoints2, 1, new Mat(), 0, cameraPoints2);
	   Core.gemm(intrinsicInv, imagePoints3, 1, new Mat(), 0, cameraPoints3);
	   Core.gemm(intrinsicInv, imagePoints4, 1, new Mat(), 0, cameraPoints4);
	   
	   Z.put(0, 0, cameraProjPoints2.get(2,0)[0]);
		
	}


	
	/**
	 * This method computes the extrinsic matrix of the image using solvePnP method
	 */
	public void formExtrinsicMatrix() {
		
		if (!radDist.empty()&& !intrinsicMat.empty()){
 		   Calib3d.solvePnP(worldPoints, imagePoints, intrinsicMat, radDist, rvec, translation);
 	   }
 	   else{
 		   System.out.println("radical distortion or intrinsic matrix is empty!!");
 	   }
		
	   //To convert 3*1 rotation vector to 3*3 rotation matrix
 	   Calib3d.Rodrigues(rvec, Rotation);
 	   Rotation=Rotation.t();
 	   
 	  for(int j=0;j<3;j++){
		   for(int k=0;k<3;k++){
			   extrinsic.put(j, k, Rotation.get(j, k));
		   }
	   }
	   
	   extrinsic.put(0, 3, translation.get(0,0));
	   extrinsic.put(1, 3, translation.get(1,0));
	   extrinsic.put(2, 3, translation.get(2,0));
	   extrinsic.put(3, 0, 0);
	   extrinsic.put(3, 1, 0);
	   extrinsic.put(3, 2, 0);
	   extrinsic.put(3, 3, 1);
 	   
	}


	
	
	/**
	 * Detect the reference object using contours.
	 * @return
	 */
	public int findReferenceObject() {
		
		double maxContourArea=0.0;
		int refId=0;
		maxContourArea=Imgproc.contourArea(contours.get(0));
        for(int j=1;j<contours.size();j++){
        	if(Imgproc.contourArea(contours.get(j))>maxContourArea){
        		maxContourArea=Imgproc.contourArea(contours.get(j));
        		refId=j;
        	}
        	else{
        		continue;
        	}
        }
        return refId;
        
	}



	
	/**
	 * This method preprocesses the image
	 * @param source
	 */
	public void imagePreProcess(Mat source){  
	
        Mat element=new Mat();
        element = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new  Size(6,6));
   	   	
   	   	Imgproc.cvtColor(source, grayScaled, Imgproc.COLOR_BGR2GRAY);
		Imgproc.GaussianBlur(grayScaled, blurred, new Size(5,5), 0);
	    Imgproc.erode(blurred, eroded, element);
	    Imgproc.dilate(eroded, dialated, element);
	    Imgproc.Canny(dialated, edgeDetected, 50, 70);
	    
	    Imgproc.dilate(edgeDetected, dialated, element);
	    Imgproc.findContours(dialated, contours , new Mat(), Imgproc.RETR_EXTERNAL,Imgproc.CHAIN_APPROX_SIMPLE);  
	    
	}
	
	
	
	/**
	 * Converts intrinsic matrices from double array to Mat objects
	 */
	public void formintrinsicMatrices() {
		
		CameraMatrix_W2C wC=new CameraMatrix_W2C();
	    CameraMatrix_C2W cW=new CameraMatrix_C2W();
	    
	    for(int j=0;j<3;j++){
 		   for(int k=0;k<3;k++){
 			   intrinsicMat.row(j).col(k).setTo(Scalar.all(wC.getIntrinsicMatrix()[j][k]));
 		   }
 	   }
	    for(int j=0;j<3;j++){
 		   for(int k=0;k<3;k++){

 			   intrinsicInv.put(j, k, cW.getIntrinsicMatrix_c2w()[j][k]);
 		   }
 	   }
		
	}
	
	
	
	
	/**
	 * To get the vertices of minimum bounding rectangles using contours
	 * @param contourPoints
	 */
	public void getVertices(MatOfPoint2f contourPoints){
		RotatedRect rrect = Imgproc.minAreaRect(contourPoints);
		rrect.points(vertices);
		for( int j=0;j<4;j++){
	 	   		//System.out.println("Vertices1 : "+j+"="+vertices1[j]);
			verticesList.add(vertices[j]);
	 	}
	}
	
	
	
	/**
	 * Get the vertices of minimum bounding rectangle of the reference object
	 * @param contourPoints
	 */
	public void getRefVertices(MatOfPoint2f contourPoints){
		RotatedRect rrect = Imgproc.minAreaRect(contourPoints);
		String name="Ref";
		rrect.points(refVertices);
		for( int j=0;j<4;j++){
	 	   		//System.out.println("Vertices1 : "+j+"="+vertices1[j]);
			refVerticesList.add(refVertices[j]);
	 	}
	 	drawBoundingRectangles(refVertices, name);	
	}
	
	
	
	
	/**
	 * To draw minimum bounding rectangles around the objects
	 * @param vertices
	 * @param name
	 */
	public void drawBoundingRectangles(Point[] vertices, String name){
		Mat refBound=new Mat();
		if(name.equals("Ref")){
			refBound=source.clone();
			for (int j = 0; j < 4; j++){
				
		 	       Imgproc.line(source, vertices[j], vertices[(j+1)%4], colorBlue,3);
			}
		}
		
		else{
			for (int j = 0; j < 4; j++){
		 	       Imgproc.line(source, vertices[j], vertices[(j+1)%4], new Scalar(255,0,0),3);
			}
		}
	
	}
	
	
	
	/**
	 * To calculate the difference between sides of the minimum bounding rectangle using its vertices.
	 * @param vertices
	 * @return
	 */
	public double sideDifference(Point[] vertices){

		double[] sides=new double[4];
		double difference=0.0;
  	   	int k=0;
  		
  	   	for( int j=0;j<2;j++){
  		   
  		   sides[k]=(vertices[(j+1)%4].x-vertices[j].x)*(vertices[(j+1)%4].x-vertices[j].x)+(vertices[(j+1)%4].y-vertices[j].y)*(vertices[(j+1)%4].y-vertices[j].y);
  		   sides[k]=Math.sqrt(sides[k]);
  		   k++;
  		 }
  	       
  	   	difference=sides[1]-sides[0];
	  	if(difference<0){
				   difference=(-1)*difference;
	  	}
			
	  	return difference;
	}
	
	
	/**
	 * Class constructor
	 */

	public ReferenceObject() {
		super();
		this.source=new Mat();
		this.dialated=new Mat();
		this.eroded=new Mat();
		this.edgeDetected = new Mat();
		this.grayScaled = new Mat();
		this.blurred = new Mat();
		this.vertices=new Point[4];
		this.refVertices=new Point[4];
		this.verticesList=new ArrayList<Point>();
		this.refVerticesList=new ArrayList<Point>();
		this.contours=new ArrayList<MatOfPoint>();
		this.extrinsic=new Mat(4,4,6);
		this.intrinsicMat = new Mat(3,3,6);
		this.intrinsicInv = new Mat(3,3,6);
		this.colorRed=new Scalar(0,0,255);
		this.colorWhite=new Scalar(255,255,255);
		this.colorBlue=new Scalar(255,0,0);
		this.imagePoints=new MatOfPoint2f();
		this.worldPoints=new MatOfPoint3f();
		this.radDist=new MatOfDouble(CalibrationConstants.RADIALDISTCOEF__C);
		this.rvec=new Mat();
		this.translation=new Mat();
		this.Rotation=new Mat();
		this.Z=new Mat(1,1,6);
		this.cameraPoints1=new Mat(4,0,6);
		this.cameraPoints2=new Mat(4,0,6);
		this.cameraPoints3=new Mat(4,0,6);
		this.cameraPoints4=new Mat(4,0,6);
		this.refLength=new Mat(1,1,6);
		this.refBreadth=new Mat(1,1,6);
		
		formintrinsicMatrices();
		
		source=Imgcodecs.imread("./Resources/Images/Reference_1.jpg");
	}

	/**
	 * Getters and Setters
	 * @return
	 */

	public Mat getSource() {
		return source;
	}



	public void setSource(Mat source) {
		this.source = source;
	}



	public Mat getEdgeDetected() {
		return edgeDetected;
	}



	public void setEdgeDetected(Mat edgeDetected) {
		this.edgeDetected = edgeDetected;
	}



	public Mat getGrayScaled() {
		return grayScaled;
	}



	public void setGrayScaled(Mat grayScaled) {
		this.grayScaled = grayScaled;
	}



	public Mat getBlurred() {
		return blurred;
	}



	public void setBlurred(Mat blurred) {
		this.blurred = blurred;
	}



	public Mat getDialated() {
		return dialated;
	}



	public void setDialated(Mat dialated) {
		this.dialated = dialated;
	}



	public Mat getEroded() {
		return eroded;
	}



	public void setEroded(Mat eroded) {
		this.eroded = eroded;
	}



	public Point[] getVertices() {
		return vertices;
	}



	public void setVertices(Point[] vertices) {
		this.vertices = vertices;
	}



	public ArrayList<Point> getVerticesList() {
		return verticesList;
	}



	public void setVerticesList(ArrayList<Point> verticesList) {
		this.verticesList = verticesList;
	}



	public Point[] getRefVertices() {
		return refVertices;
	}



	public void setRefVertices(Point[] refVertices) {
		this.refVertices = refVertices;
	}



	public ArrayList<Point> getRefVerticesList() {
		return refVerticesList;
	}



	public void setRefVerticesList(ArrayList<Point> refVerticesList) {
		this.refVerticesList = refVerticesList;
	}



	public java.util.List<MatOfPoint> getContours() {
		return contours;
	}



	public void setContours(java.util.List<MatOfPoint> contours) {
		this.contours = contours;
	}



	public Mat getExtrinsic() {
		return extrinsic;
	}



	public void setExtrinsic(Mat extrinsic) {
		this.extrinsic = extrinsic;
	}



	public Mat getIntrinsicMat() {
		return intrinsicMat;
	}



	public void setIntrinsicMat(Mat intrinsicMat) {
		this.intrinsicMat = intrinsicMat;
	}



	public Mat getIntrinsicInv() {
		return intrinsicInv;
	}



	public void setIntrinsicInv(Mat intrinsicInv) {
		this.intrinsicInv = intrinsicInv;
	}



	public MatOfPoint2f getImagePoints() {
		return imagePoints;
	}



	public void setImagePoints(MatOfPoint2f imagePoints) {
		this.imagePoints = imagePoints;
	}



	public MatOfPoint3f getWorldPoints() {
		return worldPoints;
	}



	public void setWorldPoints(MatOfPoint3f worldPoints) {
		this.worldPoints = worldPoints;
	}



	public MatOfDouble getRadDist() {
		return radDist;
	}



	public void setRadDist(MatOfDouble radDist) {
		this.radDist = radDist;
	}



	public Scalar getColorRed() {
		return colorRed;
	}



	public void setColorRed(Scalar colorRed) {
		this.colorRed = colorRed;
	}



	public Scalar getColorBlue() {
		return colorBlue;
	}



	public void setColorBlue(Scalar colorBlue) {
		this.colorBlue = colorBlue;
	}



	public Scalar getColorWhite() {
		return colorWhite;
	}



	public void setColorWhite(Scalar colorWhite) {
		this.colorWhite = colorWhite;
	}



	public Mat getRvec() {
		return rvec;
	}



	public void setRvec(Mat rvec) {
		this.rvec = rvec;
	}



	public Mat getTranslation() {
		return translation;
	}



	public void setTranslation(Mat translation) {
		this.translation = translation;
	}



	public Mat getRotation() {
		return Rotation;
	}



	public void setRotation(Mat rotation) {
		Rotation = rotation;
	}



	public Mat getZ() {
		return Z;
	}



	public void setZ(Mat z) {
		Z = z;
	}



	public Mat getCameraPoints1() {
		return cameraPoints1;
	}



	public void setCameraPoints1(Mat cameraPoints1) {
		this.cameraPoints1 = cameraPoints1;
	}



	public Mat getCameraPoints2() {
		return cameraPoints2;
	}



	public void setCameraPoints2(Mat cameraPoints2) {
		this.cameraPoints2 = cameraPoints2;
	}



	public Mat getCameraPoints3() {
		return cameraPoints3;
	}



	public void setCameraPoints3(Mat cameraPoints3) {
		this.cameraPoints3 = cameraPoints3;
	}



	public Mat getCameraPoints4() {
		return cameraPoints4;
	}



	public void setCameraPoints4(Mat cameraPoints4) {
		this.cameraPoints4 = cameraPoints4;
	}



	public Mat getRefLength() {
		return refLength;
	}



	public void setRefLength(Mat refLength) {
		this.refLength = refLength;
	}



	public Mat getRefBreadth() {
		return refBreadth;
	}



	public void setRefBreadth(Mat refBreadth) {
		this.refBreadth = refBreadth;
	}
	
	
}
