#include <opencv2/opencv.hpp>
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <fstream>
#include <iterator>
#include <iostream>
#include <vector>
#include <bits/stdc++.h> 
#include <math.h>

using namespace std;
using namespace cv;
using namespace cv::xfeatures2d;

//creating global variables for the the first and second pair of images 
Mat il,ir, il1, ir1;

// creating a vector to store the good matches after feature matching 
std::vector<DMatch> good_matches, good_matches2;
vector <cv::Point2f> leftCoordinatesVector, rightCoordinatesVector, leftCoordinatesVector2; 
vector <Point3f> position;

// initializing the left and right projection vectors from the calibrtion files in the Kitti dataset 
double projl_vec [12] =  {7.070912000000e+02, 0.000000000000e+00, 6.018873000000e+02, 4.688783000000e+01, 0.000000000000e+00, 7.070912000000e+02, 1.831104000000e+02, 1.178601000000e-01, 0.000000000000e+00, 0.000000000000e+00, 1.000000000000e+00, 6.203223000000e-03};						 
double projr_vec [12] =  {7.070912000000e+02, 0.000000000000e+00, 6.018873000000e+02, -3.334597000000e+02, 0.000000000000e+00, 7.070912000000e+02, 1.831104000000e+02, 1.930130000000e+00, 0.000000000000e+00, 0.000000000000e+00, 1.000000000000e+00, 3.318498000000e-03};

// creating matrices out of the vectors above
Mat projl_new = Mat(3,4, CV_64F, projl_vec);
Mat projr_new = Mat(3,4, CV_64F, projr_vec);

// initializing the left and right camera vectors from the calibrtion files in the Kitti dataset 
double cameraMatl [9]= {7.070912000000e+02, 0.000000000000e+00, 6.018873000000e+02, 0.000000000000e+00, 7.070912000000e+02, 1.831104000000e+02, 0.000000000000e+00, 0.000000000000e+00, 1.000000000000e+00};
double cameraMatr [9]= {7.070912000000e+02, 0.000000000000e+00, 6.018873000000e+02, 0.000000000000e+00, 7.070912000000e+02, 1.831104000000e+02, 0.000000000000e+00, 0.000000000000e+00, 1.000000000000e+00};

// creating matrices out of the vectors above
Mat cameraMatrixl = Mat(3,3, CV_64F, cameraMatl);
Mat cameraMatrixr = Mat(3,3, CV_64F, cameraMatr);

// initializing the distortion coefficients from the calibrtion files in the Kitti dataset 
double distortion[4] = {0, 0, 0, 0};
Mat distortioncoeff = Mat(1,4, CV_64F, distortion);

// the following function reads the left image from the directory if all images, converts it to grayscale and returns the image
Mat readLeftImage (int j)
{
	string filename;
	/*if(j<10)
      filename = "/home/akshay/Desktop/WPI/Code/VO/kittinew/2011_09_28/2011_09_28_drive_0001_sync/image_00/data/000000000" + to_string(j) + ".png";
    else if (j>=10 && j< 100)
      filename = "/home/akshay/Desktop/WPI/Code/VO/kittinew/2011_09_28/2011_09_28_drive_0001_sync/image_00/data/00000000" + to_string(j) + ".png";
    else
      filename = "/home/akshay/Desktop/WPI/Code/VO/kittinew/2011_09_28/2011_09_28_drive_0001_sync/image_00/data/0000000" + to_string(j) + ".png";*/
	
	// initialize filename as per its sequence 
	if(j<10)
      filename = "/home/akshay/Desktop/WPI/Code/VO/long/image_2/00000" + to_string(j) + ".png";
    else if (j>=10 && j< 100)
      filename = "/home/akshay/Desktop/WPI/Code/VO/long/image_2/0000"  + to_string(j) + ".png";
    else if(j>=100 && j<1000)
      filename = "/home/akshay/Desktop/WPI/Code/VO/long/image_2/000" + to_string(j) + ".png";
  	else
  	  filename = "/home/akshay/Desktop/WPI/Code/VO/long/image_2/00" + to_string(j) + ".png";
	
  	// read image and convert to grayscale
	Mat il = imread(filename);
	cvtColor(il, il, CV_BGR2GRAY);
	return il;	
}

// the following function reads the right image from the directory if all images, converts it to grayscale and returns the image
Mat readRightImage (int j)
{
	string filename;
	/*if(j<10)
      filename = "/home/akshay/Desktop/WPI/Code/VO/kittinew/2011_09_28/2011_09_28_drive_0001_sync/image_01/data/000000000" + to_string(j) + ".png";
    else if (j>=10 && j< 100)
      filename = "/home/akshay/Desktop/WPI/Code/VO/kittinew/2011_09_28/2011_09_28_drive_0001_sync/image_01/data/00000000" + to_string(j) + ".png";
    else
      filename = "/home/akshay/Desktop/WPI/Code/VO/kittinew/2011_09_28/2011_09_28_drive_0001_sync/image_01/data/0000000" + to_string(j) + ".png";*/
	
	// initialize filename as per its sequence 
	if(j<10)
      filename = "/home/akshay/Desktop/WPI/Code/VO/long/image_3/00000" + to_string(j) + ".png";
    else if (j>=10 && j< 100)
      filename = "/home/akshay/Desktop/WPI/Code/VO/long/image_3/0000"  + to_string(j) + ".png";
    else if(j>=100 && j<1000)
      filename = "/home/akshay/Desktop/WPI/Code/VO/long/image_3/000" + to_string(j) + ".png";
  	else
  	  filename = "/home/akshay/Desktop/WPI/Code/VO/long/image_3/00" + to_string(j) + ".png";
	
  	// read image and convert to grayscale
	Mat ir = imread(filename);
	cvtColor(ir, ir, CV_BGR2GRAY);
	return ir;	
}

// the following function takes 2 vectors containing 2D points from the left and the right images respectively and returns the Vector of Point3f consisting 
// of the corresponding 3D points formed by triangulation 
vector <Point3f> triangulate (const vector <Point2f> leftCoordinatesVector, const vector <Point2f> rightCoordinatesVector)
{
	
	Mat_<float> d4points;
	vector<Point3f> d3vec;
    Point3f d3points;

    // obtain the 3D points in homogenous coordinates 
	triangulatePoints(projl_new, projr_new, leftCoordinatesVector, rightCoordinatesVector, d4points);
    
    // convert from homogenous coordinates to Cartesian 3D coordinates  
    for (int i = 0; i< d4points.cols; i++)
    {
        d3points.x = d4points(0,i)/d4points(3,i);
        d3points.y = d4points(1,i)/d4points(3,i);
        d3points.z = d4points(2,i)/d4points(3,i);
    	if(d3points.x < 10000 && d3points.x < 10000 && d3points.x < 10000)
    		d3vec.push_back(d3points);
    }   

    return d3vec;
}

// the function does the follwing:
	// take 2 images 
	// finds features in them
	// match features between the 2 images
	// retain the best matches
	// triangulate the matched points 
void lrMatchN3D (const Mat& il, const Mat& ir, vector <Point2f>& left0, vector <Point3f>& d30)
{
	
	Mat descriptorsl, descriptorsr, descriptorsl2;
	std::vector<KeyPoint> keypointsl, keypointsr, keypointsl2;   
	vector< std::vector<DMatch> > matches;
	Ptr<Feature2D> orb = ORB::create(500);
	vector<Point2f> right0;
    
	// detect features in the left and right images
    orb->detectAndCompute(il, Mat(), keypointsl, descriptorsl);
    orb->detectAndCompute(ir, Mat(), keypointsr, descriptorsr);
    
    descriptorsl.convertTo(descriptorsl, CV_32F);
    descriptorsr.convertTo(descriptorsr, CV_32F);
    
    // create a L1 BruteForce matcher 
    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create(DescriptorMatcher::BRUTEFORCE_L1 );

    matcher->knnMatch(descriptorsl, descriptorsr, matches, 2);
    
    // retain only good matches
    const float ratio_thresh = 0.7f;
    good_matches.clear();
    for (size_t i = 0; i < matches.size(); i++)
    {
      if (matches[i][0].distance < ratio_thresh * matches[i][1].distance)
        good_matches.push_back(matches[i][0]);
    }
  
    for( size_t i = 0; i < good_matches.size(); i++ )
    {
      left0.push_back( keypointsl[ good_matches[i].queryIdx ].pt );
      right0.push_back( keypointsr[ good_matches[i].trainIdx ].pt );
    }
	
	// triangulate retained points 
    d30 = triangulate (left0, right0);
}


// the following function tracks the features of one image in the other image using Lucas Kanade Tracker 
void track(const Mat& refImg, const Mat& currImg, std::vector<Point2f>& left0, vector<Point3f>& d30, vector<Point2f>& leftFinal,  vector<Point3f>& d3pointsFinal)
{
	vector<Point2f> temp;
	vector<uchar> status;
	vector<float> err;
	
	if (left0.size()!=0)
		calcOpticalFlowPyrLK(refImg, currImg, left0, temp, status,  err);

	for (int i = 0; i < status.size(); i++)
	{
		if(status[i] == 1)
			leftFinal.push_back(temp[i]);
			d3pointsFinal.push_back(d30[i]);			
	}
}

// the following function 
void newFeatures (int i, const Mat& inv_transform, std::vector<Point2f>& left0, vector<Point3f>& d30)
{
	if (left0.size()!=0)
	{
		left0.clear();
		d30.clear();
	}

	Mat currL = readLeftImage(i);
	Mat currR = readRightImage(i);

	vector<Point2f> lTemp; vector<Point3f> d3Temp;

	lrMatchN3D(currL, currR, lTemp, d3Temp);
	

	for (int k = 0; k < d3Temp.size(); k++) {
 
	   	const Point3f& pt = d3Temp[k];

	   	Point3f p;

	   	p.x = inv_transform.at<double>(0, 0)*pt.x + inv_transform.at<double>(0, 1)*pt.y + inv_transform.at<double>(0, 2)*pt.z + inv_transform.at<double>(0, 3);
	   	p.y = inv_transform.at<double>(1, 0)*pt.x + inv_transform.at<double>(1, 1)*pt.y + inv_transform.at<double>(1, 2)*pt.z + inv_transform.at<double>(1, 3);
	   	p.z = inv_transform.at<double>(2, 0)*pt.x + inv_transform.at<double>(2, 1)*pt.y + inv_transform.at<double>(2, 2)*pt.z + inv_transform.at<double>(2, 3);

	   	if (p.z > 0) {
			d30.push_back(p);
			left0.push_back(lTemp[k]);
	   	}
	   	
   }

}

// the follwing function gets the groundtruth poses from the files provided in the KITTI dataset
vector<Point3f> getPose() {
  	  vector <Point3f> gt_poses;
  	  Point3f pose;    
      string line;
      int i = 0;
      ifstream myfile ("/home/akshay/Desktop/WPI/Code/VO/long/poses.txt");
      double x =0, y=0, z = 0, val = 0;
      double x_prev, y_prev, z_prev;
      if (myfile.is_open())
      {
        while (( getline (myfile,line) ))
        {
          z_prev = z;
          x_prev = x;
          y_prev = y;
          std::istringstream in(line);
          //cout << line << '\n';
          for (int j=0; j<12; j++)  {
            in >> val ;
            //cout << z << " \n";
            if (j==3) pose.x=val;
            if (j==7)  pose.y=val;
            if (j==11)  pose.z=val;
          }
         
          i++;
          gt_poses.push_back(pose);
        }
        myfile.close();
      }
      return gt_poses;
}


// the following function runs the main algorithm as described in the README
void VO (vector <Point3f> gt_poses)
{
	vector<Point2f> left0, right0;
	vector<Point3f> d30;
	vector <float> error, XError, ZError;
	
	double a,b,c;
	Mat il = readLeftImage(0);
	Mat ir = readRightImage(0);
	Mat& refImg = il;
	Mat currImg;
	Mat traj = Mat::zeros(1000, 1000, CV_8UC3);
	lrMatchN3D (il, ir, left0, d30);
	Mat inv_transform = Mat::zeros(3,4,CV_64F); 
	inv_transform.at<double>(0,0) = 1;
	inv_transform.at<double>(1,1) = 1;
	inv_transform.at<double>(2,2) = 1;
	Mat tvec, tvec1;
	for (int i = 1; i< 360; i++)
	{
		Mat rvec, rmat, rmat1;
		vector<Point2f> leftFinal;
		vector<Point3f> d3pointsFinal;
		currImg = readLeftImage(i);
		if (leftFinal.size()!=0)
			leftFinal.clear();
		if(d3pointsFinal.size()!=0)
			d3pointsFinal.clear();

		track(refImg, currImg, left0, d30, leftFinal, d3pointsFinal);

		if(d3pointsFinal.size() == 0)
			continue;

		
		vector<int> inliers;

		vector<Point2f> leftFinalTrunc(leftFinal.begin(), leftFinal.begin()+40);
		vector<Point3f> d3pointsFinalTrunc(d3pointsFinal.begin(), d3pointsFinal.begin()+40);
		Mat leftFinalTruncMat = Mat(leftFinalTrunc);
		Mat d3pointsFinalTruncMat = Mat(d3pointsFinalTrunc);

		if(i>=2 && tvec1.cols !=0 & tvec1.rows!=0)
		{a = tvec1.at<float>(0);
	     b = tvec1.at<float>(1);
	     c = tvec1.at<float>(2);}
	    else
	    	a=0;b=0;c=0;
	  
	    tvec.release();
	    tvec1.release();

		// to obtain the transformation matrix given a set of 3D and 2D points
		solvePnPRansac(d3pointsFinalTruncMat, leftFinalTruncMat, cameraMatrixl, distortioncoeff, rvec, tvec,false, 100, 8.0, 0.99, inliers);

		cout<<"inliers :"<<inliers.size()<<"for image number"<<i<<endl;
		if(inliers.size()<5)
			continue;

		Rodrigues(rvec, rmat);
	    
	    rmat1 = rmat.t();  // rotation of inverse
	    tvec1 = -rmat1 * tvec; // translation of inverse
	    
	    //cout<<"tvec1:\n"<<tvec1<<endl;
	    

	    double x = tvec1.at<double>(0);
	    double y = tvec1.at<double>(1);
	    double z = tvec1.at<double>(2);
    	
	
	    if(abs(x)<1000 && abs(y)<1000 && z<1000 && z>0 && abs(a-x)<40) //&& abs(b-y)<20 && abs(c-z)<20)
	    {
	        //cout<<x<<" "<<y<<" "<<z<<endl;
	        position.push_back(Point3f(x,y,z));
	        rmat1.col(0).copyTo(inv_transform.col(0));
		    rmat1.col(1).copyTo(inv_transform.col(1));
	    	rmat1.col(2).copyTo(inv_transform.col(2));
	    	tvec1.copyTo(inv_transform.col(3));	    
	    }
	    tvec1.convertTo(tvec1, CV_32F);
	   
		newFeatures(i, inv_transform, left0, d30);
	    
	    refImg = currImg;

	    // visualize the estimated and groundtruth trajectory
	    string text  = "Red color: estimated trajectory";
	    string textp = "Green color: ground truth";
	    string coord = "Coordinates: x= " + to_string(tvec1.at<float>(0)) + " y= " +  to_string(tvec1.at<float>(1)) + " z= " +  to_string(tvec1.at<float>(2)*-1);
	    Point2f center = Point2f(int(tvec1.at<float>(0))*1+ 400, int(tvec1.at<float>(2))*-1+ 300);
	    Point2f t_center = Point2f(int(gt_poses[i].x) + 400, int(gt_poses[i].z)*-1 + 300);
	    circle(traj, center ,1, CV_RGB(255,0,0), 2);
	    circle(traj, t_center,1, CV_RGB(0,255,0), 2);
	    
	   	// calculate the error in estimation
	    float curr_error = sqrt(pow(tvec1.at<float>(0) - gt_poses[i].x, 2) + pow(tvec1.at<float>(1) - gt_poses[i].y, 2) + pow(tvec1.at<float>(2) - gt_poses[i].z, 2));
	    float x_err = abs (tvec1.at<float>(0) - gt_poses[i].x);
	    float z_err = abs (tvec1.at<float>(2) - gt_poses[i].z);
	    cout<<"\n current error: "<<x_err<<" "<<z_err;
	    error.push_back(curr_error);
	    XError.push_back(x_err);
	    ZError.push_back(z_err);
	    float mean_x_error = 0;
	 	 for(auto& itr : XError)
	  		mean_x_error += itr;
	  	mean_x_error /= XError.size();
	  	float mean_z_error = 0;
	  	for(auto& itr : ZError)
	  		mean_z_error += itr;
	  	mean_z_error /= ZError.size();
	  	cout<<endl<<"mean x error: "<<mean_x_error<<"mean z error: "<<mean_z_error;
	  	rectangle(traj, Point2f(380, 10) , Point2f(1000, 200), CV_RGB(0,0,0), cv::FILLED);
	    putText(traj, text, Point2f(400,40), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.7, Scalar(0, 0,255), 1, 5);
	    putText(traj, textp, Point2f(400,60), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.7, Scalar(0,255,0), 1, 5);
	    putText(traj, coord, Point2f(400 ,90), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.7, Scalar(0,0,255), 1, 5);
	    //cvSetWindowProperty("Name", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);
	    cv::imshow("Camera view", currImg);
	    cv::imshow("Trajectory", traj);
	    waitKey(1);
	    
	  }
 	
	ofstream output_file ("positions.txt");
	ostream_iterator<Point3f> o_iter(output_file,"\n");
	copy(position.begin(),position.end(),o_iter);
  	imwrite("map2.png", traj);
  	float mean_error = 0;
  	for(auto& itr : error)
  		mean_error += itr;
  	mean_error /= error.size();
    waitKey(0);
}

int main()
{
	vector<Point3f> gt_poses = getPose();
	cout<<"ground truth:\n";
	
	// call the function to carry out the VO algorithm
	VO(gt_poses);

	return 0;
}