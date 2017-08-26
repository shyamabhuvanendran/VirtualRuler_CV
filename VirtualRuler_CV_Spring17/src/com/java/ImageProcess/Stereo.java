package com.java.ImageProcess;

import java.util.ArrayList;
import java.util.LinkedList;
import org.opencv.calib3d.Calib3d;
import org.opencv.calib3d.StereoSGBM;
import org.opencv.core.Core;
import org.opencv.core.DMatch;
import org.opencv.core.KeyPoint;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDMatch;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfKeyPoint;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.features2d.DescriptorExtractor;
import org.opencv.features2d.DescriptorMatcher;
import org.opencv.features2d.FeatureDetector;
import org.opencv.features2d.Features2d;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import com.java.Camera.CalibrationConstants;
import com.java.Camera.CameraMatrix_C2W;
import com.java.Camera.CameraMatrix_W2C;

/**
 *This class implements the stereo method to find object dimensions 
 *@shyama
 *
 */


public class Stereo {
	
	private Mat source1;
	private Mat source2;
	private Mat edgeDetected1;
	private Mat grayScaled1;
	private Mat blurred1;
	private Mat dialated1;
	private Mat eroded1;
	private Mat grayScaled2;
	private Mat blurred2;
	private Point vertices1[];
	private java.util.List<MatOfPoint> contours1;
    private Mat Rotation;
    private Mat translation;
    private Mat fundMat;
    private  Mat disparity;
    private Mat intrinsicMat;
    private Mat intrinsicInv;
    private MatOfDouble radDist;
    private Mat disparityOutput;
    private double distance;
	private double length;
	private double breadth;
	private double baselineDistance;
	private Scalar colorWhite;
	private Mat P1;
	private Mat P2;
	private Point zPoint;
	
	
	/**
	 * This method starts the stereo method of the application. 
	 */
	
	public void useStereoMethod(){
		
		System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
		
		int noOfObjects=0;
		Mat imagePoints1=new Mat(3,1,6);
 	   	Mat imagePoints2=new Mat(3,1,6);
 	   	Mat imagePoints3=new Mat(3,1,6);
 	   	Mat imagePoints4=new Mat(3,1,6);
 	   	Mat cameraPoints1=new Mat(4,1,6);
	   	Mat cameraPoints2=new Mat(4,1,6);
	   	Mat cameraPoints3=new Mat(4,1,6);
	   	Mat cameraPoints4=new Mat(4,1,6);
	   	Mat side1Difference=new Mat(3,3,6);
 	   	Mat side1DifferenceX=new Mat(1,1,6);
 	   	Mat side1DifferenceY=new Mat(1,1,6);
 	   	Mat side2Difference=new Mat(3,3,6);
 	   	Mat side2DifferenceX=new Mat(1,1,6);
 	   	Mat side2DifferenceY=new Mat(1,1,6);
 	    ReferenceObject refObj=new ReferenceObject();
	   	double z=0.0;
	   	
	   	System.out.println();
	   	System.out.println("Starting Stereo Approach");
	   	System.out.println("*************************");
	   	
	   	//Preprocess the two images
		imagePreProcess(source1,1);
		imagePreProcess(source2,2);
		
		//Find the fundamental matrix using corresponding points from the stereo images
		findFundamentalMatrix();
		
		//this method computes the disparity map of the two images
	    formDisparity();
	    
	    for(int i=0;i<contours1.size();i++){
	    	if((Imgproc.contourArea(contours1.get(i))>25000)&&(Imgproc.contourArea(contours1.get(i))<250000)){
	    		Mat objLength=new Mat(1,1,6);
	    		Mat objBreadth=new Mat(1,1,6);
	    		MatOfPoint2f contourPoints = new MatOfPoint2f(contours1.get(i).toArray());
	    		MatOfPoint2f imagePoints=new MatOfPoint2f();
	    		MatOfPoint2f[] writePoints=new MatOfPoint2f[2];
	    		Point p1=new Point();
			 	Point p2=new Point();
			 	ArrayList<Point> verticesList1 = new ArrayList<Point>();
			 	
				noOfObjects++;
				
				//to get vertices of the bounding rectangle
				getVertices(contourPoints);
				//draw bounding rectangles around object
				drawBoundingRectangles(vertices1, "Stereo");
				
				for( int j=0;j<4;j++){
    				verticesList1.add(vertices1[j]);
    				//System.out.println("vertices1 :"+vertices1[j].x+": "+vertices1[j].y);
    		 	}
    			
				
				imagePoints.fromList(verticesList1);
				
				imagePoints1.put(0,0,imagePoints.get(0, 0));
    			imagePoints1.put(2, 0, 1);
    			imagePoints2.put(0,0,imagePoints.get(1, 0));
    			imagePoints2.put(2, 0, 1);
    			imagePoints3.put(0,0,imagePoints.get(2, 0));
    			imagePoints3.put(2, 0, 1);
    			imagePoints4.put(0,0,imagePoints.get(3, 0));
    			imagePoints4.put(2, 0, 1);
    			
    			//Multiply with inverse intrinsic matrix to re-project pixel points to camera coordinate system
    			Core.gemm(intrinsicInv, imagePoints1, 1, new Mat(), 0, cameraPoints1);
    			Core.gemm(intrinsicInv, imagePoints2, 1, new Mat(), 0, cameraPoints2);
    			Core.gemm(intrinsicInv, imagePoints3, 1, new Mat(), 0, cameraPoints3);
    			Core.gemm(intrinsicInv, imagePoints4, 1, new Mat(), 0, cameraPoints4);
    			
    			//Calculates Z using disparity map, focal length and baseline distance
    		 	z=calculateZ(vertices1);
    		 	
    		 	//Multiply with the determined Z to get the actual camera coordinates
    		 	Core.multiply(cameraPoints1, new Scalar(z), cameraPoints1);
    		 	Core.multiply(cameraPoints2, new Scalar(z), cameraPoints2);
    		 	Core.multiply(cameraPoints3, new Scalar(z), cameraPoints3);
    		 	Core.multiply(cameraPoints4, new Scalar(z), cameraPoints4);	
    			
    		 	//Use distance formula to calculate the dimensions
    		 	Core.subtract(cameraPoints1, cameraPoints2, side1Difference);
    		 	side1DifferenceX.put(0, 0, side1Difference.get(0, 0));
    		 	side1DifferenceY.put(0, 0, side1Difference.get(1, 0));
    		 	Core.magnitude(side1DifferenceX, side1DifferenceY, objLength);
    		 	  
    		 	Core.subtract(cameraPoints2, cameraPoints3, side2Difference);
    		 	side2DifferenceX.put(0, 0, side2Difference.get(0, 0));
    		 	side2DifferenceY.put(0, 0, side2Difference.get(1, 0));
    		 	Core.magnitude(side2DifferenceX, side2DifferenceY, objBreadth);
    		 	
    		 	length=refObj.roundTwoDecimals(objLength.get(0, 0)[0]);
    		 	breadth=refObj.roundTwoDecimals(objBreadth.get(0, 0)[0]);
    		 	
    		 	//To get the writepoints to write the dimensions on the image
    		 	writePoints=refObj.getLocationToPutDimensions(verticesList1);
    		 	p1.set(writePoints[0].get(0, 0));
    		 	p2.set(writePoints[1].get(0, 0));
    		 	
    		 	System.out.println("Length of Object "+noOfObjects+" : "+length+"mm");
    		 	System.out.println("Breadth of Object "+noOfObjects+" : "+breadth+"mm");
    		 	System.out.println();
    		 	
    		 	Imgproc.putText(source1, Double.toString(length), p1, 5, 1.8, colorWhite,3);
    		 	Imgproc.putText(source1, Double.toString(breadth), p2, 5, 1.8, colorWhite,3);
    		 
	    	}				
	    }
	    Imgcodecs.imwrite("./Resources/Images/StereoResults1.JPG",source1);
	    System.out.println("Stereo Method Completed! Check the Resources/Images/StereoResults.JPG for results.");
	}
	
	
	/**
	 * This method draws minimum bounding rectangles around the objects
	 * @param vertices12
	 * @param name
	 */
	public void drawBoundingRectangles(Point[] vertices12, String name) {
		for (int j = 0; j < 4; j++){
	 	       Imgproc.line(source1, vertices1[j], vertices1[(j+1)%4], new Scalar(255,0,0),3);
		}
		//Imgcodecs.imwrite("./Resources/Images/BoundingRects"+name+".JPG",source1);
		
	}

	
	/**
	 * This method gets the vertices of the minimum bounding rectangles of contour points
	 * @param contourPoints
	 */
	public void getVertices(MatOfPoint2f contourPoints){
		RotatedRect rrect = Imgproc.minAreaRect(contourPoints);
		rrect.points(vertices1);
	}
	
	
	/**
	 * This method calculates Z using disparity map, baseline distance and focal length.
	 * @param vertices
	 * @return
	 */
	private double calculateZ(Point[] vertices) {
		double disp=0.0, z=0.0;
		double[][] temp=new double[2][2];
		double d=vertices1[0].x+vertices1[1].x;
		zPoint.x=d/2;
		System.out.println("Vertices "+d/2);
		
		temp[0][0]=(vertices1[0].x+vertices1[1].x)/2;
		temp[0][1]=(vertices1[0].y+vertices1[1].y)/2;
	
		
		temp[1][0]=(vertices1[2].x+vertices1[3].x)/2;
		temp[1][1]=(vertices1[2].y+vertices1[3].y)/2;
		
		
		zPoint.x=(temp[0][0]+temp[1][0])/2;
		zPoint.y=(temp[0][1]+temp[1][1])/2;	
		
		
		System.out.println("zPoint: "+zPoint.x+", "+zPoint.y);
		
	    //disp=disparity.get((int) vertices[0].y,(int)vertices[0].x)[0];
		disp=disparity.get((int) zPoint.y,(int)zPoint.x)[0];
		
		System.out.println("disp: "+disp);
		
	    if(disp==0){
	    	System.out.println("Disparity is zero!! Setting Z to zero");
	    }
	    
	    else{
	    	z=(CalibrationConstants.FOCALLInPX__C[0]/disp)*baselineDistance;
	    }
	    
	    System.out.println("Z: "+z);
	    
	    return z;
	}
	

	/**
	 * This method computes the disparity map of two stereo images.
	 */
	private void formDisparity() {
		Mat R11 = new Mat();
	    Mat R12 = new Mat();
	    Mat Q = new Mat();
	    Size newImageSize = new Size();
		double balance = 0;
		double fov_scale=0;
																								
		Calib3d.stereoRectify(intrinsicMat, radDist, intrinsicMat, radDist, grayScaled1.size(), Rotation, translation, R11, R12, P1, P2, Q,Calib3d.CALIB_ZERO_DISPARITY, newImageSize, balance, fov_scale);
	    StereoSGBM stereo=StereoSGBM.create(15,16,5);
	    stereo.compute(grayScaled1, grayScaled2, disparity);
	    Calib3d.reprojectImageTo3D(disparity, disparityOutput, Q);
	   
	}
	

	/**
	 * This method converts the intrinsic matrices from double array to Mat objects.
	 */
	private void formintrinsicMatrices() {	
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
	 * This method calculates the fundamental matrix of two images using corresponding points.
	 */
	private void findFundamentalMatrix() {
		
		FeatureDetector detector = FeatureDetector.create(FeatureDetector.ORB);
		MatOfKeyPoint keys1 = new MatOfKeyPoint();
		MatOfKeyPoint keys2 = new MatOfKeyPoint();
		DescriptorExtractor extractor = DescriptorExtractor.create(DescriptorExtractor.ORB); 
		DescriptorMatcher matcher = DescriptorMatcher.create(DescriptorMatcher.BRUTEFORCE_HAMMING);
		MatOfDMatch matches = new MatOfDMatch();
		LinkedList<DMatch> matchesList = new LinkedList<DMatch>();
		LinkedList<DMatch> goodMatches = new LinkedList<DMatch>();
	    MatOfDMatch gm = new MatOfDMatch();	    
	    LinkedList<Point> im1List = new LinkedList<Point>();
	    LinkedList<Point> im2List = new LinkedList<Point>();
	    LinkedList<KeyPoint> keypointsIm1List = new LinkedList<KeyPoint>();
	    LinkedList<KeyPoint> keypointsIm2List =new LinkedList<KeyPoint>(); 
	    MatOfPoint2f im1Points = new MatOfPoint2f();
	    MatOfPoint2f im2Points = new MatOfPoint2f();
	    Mat im1 = new Mat();
	    Mat im2 = new Mat() ;
	    Mat out1=new Mat();
		Mat out2=new Mat();
		Double maxDist = 0.0;
	    Double minDist =24.0;
	    Double dist=0.0;
		
	    //detect key points in two images
		detector.detect(blurred1, keys1);
		detector.detect(blurred2, keys2);
		
		//draws key points
		Features2d.drawKeypoints(blurred1, keys1, out1);
		Features2d.drawKeypoints(blurred2, keys2, out2);
		
		//extract features of key points
	    extractor.compute(blurred1, keys1, im1);
	    extractor.compute(blurred2, keys2, im2);
	    
	    //Mat the key points of two images to find good matches
	    matcher.clear();
	    matcher.match(im1, im2, matches);
	    matchesList.addAll(matches.toList());
	    
	    for(int i = 0; i < im1.rows(); i++){
	        dist = (double) matchesList.get(i).distance;
	        if(dist < minDist){
	        	minDist = dist;
	        }
	        if(dist > maxDist){
	        	maxDist = dist;
	        }
	    }
	    
	    for(int i = 0; i < im1.rows(); i++){
	        if(matchesList.get(i).distance < 2*minDist){
	            goodMatches.addLast(matchesList.get(i));
	        }
	    }
	    
	    
		
	    gm.fromList(goodMatches);
	    keypointsIm1List.addAll(keys1.toList());
	    keypointsIm2List.addAll(keys2.toList());
	    
	    for(int i = 0; i<goodMatches.size(); i++){
	        im1List.addLast(keypointsIm1List.get(goodMatches.get(i).queryIdx).pt);
	        im2List.addLast(keypointsIm2List.get(goodMatches.get(i).trainIdx).pt);
	    }
	    
	    im1Points.fromList(im1List);
	    im2Points.fromList(im2List);
	    
	    //using corresponding key points calculates the fundamental matrix of the stereo system.
	    fundMat=Calib3d.findFundamentalMat(im1Points, im2Points, Calib3d.FM_RANSAC,3,0.99 );
	    //System.out.println("fundMat: \n"+fundMat.dump());
	}

	
	/**
	 * This method preprocesses the two stereo images
	 * @param source
	 * @param i
	 */
	public void imagePreProcess(Mat source, int i){
		
        Mat element=new Mat();
   	   	element = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new  Size(7,7));
 	   	
	 	if(i==1){
	 	   	Imgproc.cvtColor(source1, grayScaled1, Imgproc.COLOR_BGR2GRAY);
			Imgproc.GaussianBlur(grayScaled1, blurred1, new Size(7,7), 0);
	        Imgproc.erode(blurred1, eroded1, element);
	        
	        Imgproc.dilate(eroded1, dialated1, element);
	        
	        Imgproc.Canny(dialated1, edgeDetected1, 70, 200);
	        Imgproc.dilate(edgeDetected1, dialated1, element);
	        Imgproc.dilate(dialated1, dialated1, element);
	        Imgproc.findContours(dialated1, contours1 , new Mat(), Imgproc.RETR_EXTERNAL,Imgproc.CHAIN_APPROX_SIMPLE);
	 	}
	 	
	 	else if(i==2){
	 	   	Imgproc.cvtColor(source2, grayScaled2, Imgproc.COLOR_BGR2GRAY);
			Imgproc.GaussianBlur(grayScaled2, blurred2, new Size(7,7), 0); 
	 	}
	}
	
	/**
	 * Initializes the translation matrix
	 */
	private void formTranslationMatrix() {
		translation.put(0, 0, 50);
		translation.put(1, 0, 0);
		translation.put(2, 0, 0);
	}
	
	
	/**
	 * Initializes the rotation matrix
	 */
	private void formRotationMatrix() {
		Rotation.put(0, 0, 1);
		Rotation.put(0, 1, 0);
		Rotation.put(0, 2, 0);
		Rotation.put(1, 0, 0);
		Rotation.put(1, 1, 1);
		Rotation.put(1, 2, 0);
		Rotation.put(2, 0, 0);
		Rotation.put(2, 1, 0);
		Rotation.put(2, 2, 1);
		//System.out.println("Rotation:\n"+Rotation.dump());
		
	}

	/**
	 * Constructor of the class.
	 */
	public Stereo() {
		super();
		this.source1=new Mat();
		this.source2=new Mat();
		this.eroded1=new Mat();
		this.edgeDetected1 = new Mat();
		this.grayScaled1 = new Mat();
		this.blurred1 = new Mat();
		this.grayScaled2 = new Mat();
		this.blurred2 = new Mat();
		this.dialated1=new Mat();
		this.vertices1=new Point[4];
		this.contours1 = new ArrayList<MatOfPoint>();
		this.Rotation=new Mat(3,3,6);
		this.translation=new Mat(3,1,6);
		this.fundMat=new Mat(3,3,4);
		this.disparity=new Mat();
		this.intrinsicInv= new Mat(3,3,6);
		this.intrinsicMat= new Mat(3,3,6);
		this.radDist=new MatOfDouble();
		this.disparityOutput=new Mat();
		this.distance=0.0;
		this.baselineDistance=50.0;
		this.colorWhite=new Scalar(255,255,255);
		this.P1=new Mat();
		this.P2=new Mat();
		this.zPoint=new Point();
		
		formintrinsicMatrices();
		formRotationMatrix();
		formTranslationMatrix();
		radDist=new MatOfDouble(CalibrationConstants.RADIALDISTCOEF__C);
		
		this.source1=Imgcodecs.imread("./Resources/Images/Stereo_1a.jpg");
		this.source2=Imgcodecs.imread("./Resources/Images/Stereo_1b.jpg");
	}

	/**
	 * Getters and setters
	 * @return
	 */
	public Mat getSource1() {
		return source1;
	}


	public void setSource1(Mat source1) {
		this.source1 = source1;
	}


	public Mat getSource2() {
		return source2;
	}


	public void setSource2(Mat source2) {
		this.source2 = source2;
	}


	public Mat getEdgeDetected1() {
		return edgeDetected1;
	}


	public void setEdgeDetected1(Mat edgeDetected1) {
		this.edgeDetected1 = edgeDetected1;
	}


	public Mat getGrayScaled1() {
		return grayScaled1;
	}


	public void setGrayScaled1(Mat grayScaled1) {
		this.grayScaled1 = grayScaled1;
	}


	public Mat getBlurred1() {
		return blurred1;
	}


	public void setBlurred1(Mat blurred1) {
		this.blurred1 = blurred1;
	}


	public Mat getDialated1() {
		return dialated1;
	}


	public void setDialated1(Mat dialated1) {
		this.dialated1 = dialated1;
	}


	public Mat getEroded1() {
		return eroded1;
	}


	public void setEroded1(Mat eroded1) {
		this.eroded1 = eroded1;
	}


	public Mat getGrayScaled2() {
		return grayScaled2;
	}


	public void setGrayScaled2(Mat grayScaled2) {
		this.grayScaled2 = grayScaled2;
	}


	public Mat getBlurred2() {
		return blurred2;
	}


	public void setBlurred2(Mat blurred2) {
		this.blurred2 = blurred2;
	}


	public Point[] getVertices1() {
		return vertices1;
	}


	public void setVertices1(Point[] vertices1) {
		this.vertices1 = vertices1;
	}


	public java.util.List<MatOfPoint> getContours1() {
		return contours1;
	}


	public void setContours1(java.util.List<MatOfPoint> contours1) {
		this.contours1 = contours1;
	}


	public Mat getRotation() {
		return Rotation;
	}


	public void setRotation(Mat rotation) {
		Rotation = rotation;
	}


	public Mat getTranslation() {
		return translation;
	}


	public void setTranslation(Mat translation) {
		this.translation = translation;
	}


	public Mat getFundMat() {
		return fundMat;
	}


	public void setFundMat(Mat fundMat) {
		this.fundMat = fundMat;
	}


	public Mat getDisparity() {
		return disparity;
	}


	public void setDisparity(Mat disparity) {
		this.disparity = disparity;
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


	public MatOfDouble getRadDist() {
		return radDist;
	}


	public void setRadDist(MatOfDouble radDist) {
		this.radDist = radDist;
	}


	public Mat getDisparityOutput() {
		return disparityOutput;
	}


	public void setDisparityOutput(Mat disparityOutput) {
		this.disparityOutput = disparityOutput;
	}


	public double getDistance() {
		return distance;
	}


	public void setDistance(double distance) {
		this.distance = distance;
	}


	public double getLength() {
		return length;
	}


	public void setLength(double length) {
		this.length = length;
	}


	public double getBreadth() {
		return breadth;
	}


	public void setBreadth(double breadth) {
		this.breadth = breadth;
	}


	public double getBaselineDistance() {
		return baselineDistance;
	}


	public void setBaselineDistance(double baselineDistance) {
		this.baselineDistance = baselineDistance;
	}


	public Scalar getColorWhite() {
		return colorWhite;
	}


	public void setColorWhite(Scalar colorWhite) {
		this.colorWhite = colorWhite;
	}


	public Mat getP1() {
		return P1;
	}


	public void setP1(Mat p1) {
		P1 = p1;
	}


	public Mat getP2() {
		return P2;
	}


	public void setP2(Mat p2) {
		P2 = p2;
	}
	

}
