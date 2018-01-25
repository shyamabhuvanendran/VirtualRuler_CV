# Real object dimensions from Images using OpenCV Java (Computer Vision)

This project finds the real object dimesions from images. Two methods are implemented - reference object method and stereo method. Reference object method uses an object of known dimensions in the image to compute the real dimensions of other objects in the same image. Stereo method uses two images of the same scene with known camera orientations to determine the real dimensions of objects in the image.

Implemented two approaches – Reference Object Method and Stereo Vision method.
The Reference Object Method uses an object of known dimensions called reference object in the image.
Assume one of the corners of the reference object to be the world origin and represent the other corners
using the known dimensions of the object. Preprocess the image, detect the reference object in the image
and get the vertices of the minimum bounding rectangle. Using the pixel points of the vertices and the
corresponding world coordinates, calculate the Extrinsic Matrix of the image using solvePnP() method of
OpenCV Library. Project the world coordinates to the camera coordinate system using the determined
Extrinsic Matrix to get Z. Detect other objects in the image and re-project their pixel points to the camera
coordinate system. Multiply with the determined Z to get the actual camera coordinates. Use distance
formula to calculate the real object dimensions.

The Stereo Vision Method uses two images of the scene captured by the same camera. Take a picture of
the required scene and translate the camera to a known distance (baseline distance) horizontally to capture
the second picture of the same scene. Preprocess the images and find the disparity map of the images
using compute() method of StereoSGBM class in OpenCV library. Detect the objects in the left image and
get the vertices of the minimum bounding rectangles. Using the disparity map, find the disparity
corresponding to each of the vertices. Use the below formula to calculate Z:
Z = (focal length * base line distance) / disparity

Re-project these vertices to camera coordinate system using inverse intrinsic matrix and multiply with the
determined Z. Use distance formula to calculate the real object dimensions.
The Reference Object Method could determine the object dimensions with high accuracy (99% to 100%)
for objects lying on the same X-Y plane as the reference object. The accuracy drops as the object distance
from the reference object increases along the Z-axis. The Stereo Vision Method, on the other hand, could
determine the Z accurately but the accuracy of the determined dimensions dropped in the cases where the
object detection method used couldn’t get the exact vertices of the objects in the image.

****************
Execution Steps:

1. Download the project.

2. Download OpenCV 3.2.0 and link it to the project.

3. Compile and Run the ExecuteMe.java class present in the path VirtualRuler_CV_Spring17\src\com\java\Test\ExecuteMe.java

Results:

1. Reference Object Method result 'ReferenceResults.JPG' can be found in the path VirtualRuler_CV_Spring17\Resources\Images\ReferenceResults.JPG

2. Stereo Object Method result can be found in the path VirtualRuler_CV_Spring17\Resources\Images\StereoResults.JPG
