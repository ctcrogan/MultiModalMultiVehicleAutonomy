//
//  Quadcopter Project and Ground Robot Project
//
//  Authors:  Nathan Ledoux and Manaswi.
//  Copyright (c) 2014. All rights reserved.
//

//ARIA
#include "Aria.h"
#include "ArGPS.h"
#include "ArGPSConnector.h"

//General
#include "GPS_ECEF_LLA.h"		//GPS Conversions (included in project directory)
#include "PortChat.h"			//Supports port communications (included in project directory)
#include <Windows.h>
#include <fstream>
#include <assert.h>
#include <ctime>
#include <string> 

//OpenCV
#include <opencv/cv.h>						//We shouldn't need this, because we should
													//replace all of the C code with C++ code
#include <opencv2/core/core.hpp>        // Basic OpenCV structures (cv::Mat)
#include <opencv2/highgui/highgui.hpp>  // Video write
#include <opencv2/imgproc/imgproc.hpp>  // for image processing

using namespace cv;

// Ground Vehicle Code
double const PI = 3.14159265358979323846;
double earthRadiusKm = 6371.0;
double goalLat = 41.627261;		//41.378966;
double goalLong = -71.008224;	//-71.004269;
double yGoalKm, xGoalKm;
char chgDirection = 'X';
string GPSxyz;
ArGPS *GPS = NULL;
IplImage* frameGround = NULL;

// Aerial Vehicle Code
const int CONFIDENCE_THRESHOLD = 10;
const int AREA_THRESHOLD = 400;
//Raw Feed
IplImage* frameAerial = NULL;		//Image variable to store frame 
//Filtered Feed
IplImage* imgTracking = NULL;
//Holds frames we need to save
static IplImage* imgSaved = NULL;
//Weighted Average of the filtered image of the "target"
CvMoments* myMoments = NULL;
//Time stuff
struct tm* timeinfo = NULL;
time_t currentTime;
clock_t beginTime;
clock_t endTime;
bool found = false;
//Tracks the number of consecutive positive IDs
//Must overcome the confidenceThreshold to trigger
//target acquired
int numFrames = 0;

ofstream outfile;

//Last known (pixel) coordinates of the target's center
int lastX = -1;
int lastY = -1;

// Function Prototypes
//Main function of the aerial vision algorithm
int runAerialVision();
//Converts the GPS coordinates from ECEF to LLA format
//and outputs and logs the data
void parseTelemetryData();
//Main function for the ground vehicle
void runGroundVision();
//Uses the "hot-cold" algorithm to find the passed coordinates
int groundRobotFindGPSLocation(int *argc, const char * argv[]);
//Applies and HSV color filter to the image
Mat ApplyHSVFilter(Mat imgSourceFrame);
//Tracks the detected object
void detectTarget(Mat imgFilteredFrame);
//Writes output to the logfile
void write_X_and_y(std::ofstream& outfile, int x, int y, double time);


//Mathematical Support Functions
double deg2rad(double deg) { return (deg * PI / 180); }
double rad2deg(double rad) { return (rad * 180 / PI); }


/***************************************************
 * Applies an HSV color filter to the image.
 * Returns a binary image with all of the source
 * pixels within the filter bounds in white.
 ***************************************************/
Mat ApplyHSVFilter(Mat imgSourceFrame)
{
	//ToDo: Convert to HSV within this function so that
	// other types of images can be passed to the function
	
	Mat imgFilteredFrame;
	inRangeS(imgSource, Scalar(170,150,30), Scalar(180,255,255), imgFilteredFrame); // red
	
	
	return imgFilteredFrame;
}




/*************************************************************
 * Uses the area (M00) and centroid of the grey-scale
 * image to determine (a) if the object is present
 * and (b) the location of the object. If the object
 * is present, it checks to see if the object has
 * been present for consecutive frames, and when
 * the number of consecutive frames set in the
 * CONFIDENCE_THRESHOLD, it flags the object as found.
 *
 * ToDo: This algorithm is not very good. It has very
 * weak noise checks and filters, and it frequently
 * flags false positives. A simple fix would be to set
 * a threshold for the second moments to compute the
 * the variance of center of gravity of the image
 * and only accept if it is below a certain threshold
 * it is likely, however, that a more complicated 
 * algorithm would yield better results.
 *************************************************************/
void detectTarget(Mat imgFilteredFrame)
{
	//Calculate the moments of the filtered image
	//This should be done using the C++ moments() function
	moments(imgFilteredFrame, myMoments, 1);
	//This can be done using myMoments::m10 m01 and m00 using C++
	double M_10 = cvGetSpatialMoment(myMoments, 1, 0);
	double M_01 = cvGetSpatialMoment(myMoments, 0, 1);
	double M_00 = cvGetCentralMoment(myMoments, 0, 0);
	double M_20 = cvGetCentralMoment(myMoments, 2, 0);
	double M_02 = cvGetCentralMoment(myMoments, 0, 2);
	
	//Computes the centroid of the image. 
	double x_bar = moment10/area;
	double y_bar = moment01/area;
	
	//Computes the variance of the image
	double var_x = (M_20 - x_bar * M_10) / M_00;
	double var_y = (M_02 - y_bar * M_01) / M_00;

	
	
	if(area > AREA_THRESHOLD )
	{
		
		
		//This if is broken. It is supposed to handle the counting
		//of consecutive positive IDs, but it is not handled properly.
		//It performs adequately in tests, but it does not work in the 
		//manner it is supposed to.
		if(lastX>=0 && lastY>=0 && posX>=0 && posY>=0)
		{       
			//If numFrames (which needs to be renamed) does not
			//exceed the confidence threshold, increment it, and
			//write to the console.
			if(numFrames < CONFIDENCE_THRESHOLD)
			{
				cout<<"Target Spotted for "<<numFrames<< " consecutive frames."<<endl;
				numFrames++;
			}
			//If the count exceeds the threshold, set found to true,
			//get the time and GPS coordinates, and write them to the logfile
			//
			//ToDo: Improve the data collection/logging information to make
			//debugging and analysis easier. Everything should be time/date
			//stamped, and should be associated with an ID number unique to 
			//the specific test flight that generated it. The altitude of
			//the quadcopter at the moment of target acquisition should also 
			//be stored, for both testing purposes, and possibly, in the long
			//run, to compute a more accurate location of the target.
			else 	
			{	
				
				found = true;
				endTime = clock();
				
				// GET THE GPS COORDINATES FOR THE GROUND ROBOT
				GPSxyz = PortChat::getGPScoords();

				double elapsedTime = ((double)(endTime - beginTime))/CLOCKS_PER_SEC;
				write_X_and_y(outfile, posX, posY, elapsedTime);
				cvSaveImage("blackWhiteFrame.jpg" , imgThresh);
				cvSaveImage("colorFrame.jpg" , frameAerial);
			}
		}
		
		//The up-to-date location of the quadcopter
		//at the last suspected sighting of the target
		//These should be renamed
		lastX = posX;
		lastY = posY;  
	}
}




/***************************************************
 * Runs the aerial vision algorithm
 ***************************************************/
int runAerialVision()
{
	//Set up timestamps
	beginTime = clock();
	currentTime = time(NULL);
	timeinfo = localtime (&currentTime);
	
	//Open capture pointer to aerial cam
	CvCapture* capAerial = cvCreateCameraCapture(0);
	
	//UI Feed Windows
	cvNamedWindow("Live Aerial Feed");
	cvNamedWindow("Processed Aerial Feed");
	
	//Open logfile
	outfile.open("FlightData.txt");
	if(outfile.fail())			//ToDo: If file doesn't exist, create file
	{

		std::cerr<<"\n The file was not successfully opened"
			<<"\n Please check that the file currently exists"
			<<std::endl;
		exit(3);
	}
	
	//Wait 1.0 seconds
	cvWaitKey(1000);
	//Get next frame from aerial cam
	frameAerial = cvQueryFrame(capAerial);
	//Outputs the VideoCapture Property
	//ToDo: Convert from c to cpp format
	cout<< cvGetCaptureProperty(capAerial, CV_CAP_PROP_FORMAT)<<endl;

	//Creates a new image that is the same size as the captured frame
	//This should be an unnecessary constructor call, since
	//OpenCV will dynamically allocate memory for the images it creates
	//and will do so more efficiently than this constructor.
	
	//This allocates 1.5kb of RAM for no reason
	imgTracking = cvCreateImage(cvGetSize(frameAerial),IPL_DEPTH_8U, 3);
	//Performs 1.5 thousand writes to memory for no reason
	cvZero(imgTracking); //covert the image, 'imgTracking' to black

	IplImage* imgHSV = NULL;
	IplImage* imgThresh = NULL;

	//iterate through each frames of the video
	//This is the main search loop, and should be rewritten entirely.
	//The major focus of this project will likely be the improvement of this
	//search algorithm.
	while(true)
	{   
		//Get next frame
		frameAerial = cvQueryFrame(capAerial);

		//Copy the captured frame to a new mem location
		frameAerial = cvCloneImage(frameAerial);
		
		//Run a Gaussian kernel to smooth the image (removes noise)
		//We will want to recalibrate these arguments
		//We should also change it to GaussianBlur() which is
		//the C++ equivalent of cvSmooth (which is a C function)
		cvSmooth(frameAerial, frameAerial, CV_GAUSSIAN,3,3); 
		
		//Allocates more memory for another blank image
		//(again I don't think this is necessary OpenCV's
		//functions should do this dynamically
		imgHSV = cvCreateImage(cvGetSize(frameAerial), IPL_DEPTH_8U, 3);

		//Converts the image from BGR to HSV format
		//Again, this is C, not C++, which is at the
		//very least annoyingly inconsistent, and is possibly
		//leading to memory problems, since the C and C++
		//functions allocate and deallocate memory differently.
		cvCvtColor(frameAerial, imgHSV, CV_BGR2HSV); //Change the color format from BGR to HSV

		//Calls the very short function above, which allocates more memory for
		//the imgHSV (which has already had memory allocated for it in this function)
		//and then makes a single inRangeS call which applies our color filter
		imgThresh = ApplyHSVFilter(imgHSV);
		
		//Smooths the filtered image using the Gaussian to remove noise
		//This should be done using the C++ GuassianBlur rather than
		//cvSmooth
		cvSmooth(imgThresh, imgThresh, CV_GAUSSIAN,3,3); //smooth the binary image using Gaussian kernel

		//If the target has not yet been flagged, call track object
		//If found, break out
		//This should be handled differently
		if(!found)
			detectTarget(imgThresh);	//track the position of the object
		else
			break;

		
		//This literally performs about 1500 useless arithmetic operations
		//It performs an element-wise matrix addition 
		//between the live raw feed and the an otherwise unused
		//matrix that has been initialized to zeroes. 
		//This performs 3x480 identity computations and wastes
		//as many bytes of memory. There is no discernible purpose
		//for this computation.
		cvAdd(frameAerial, imgTracking, frameAerial);	// Add the tracking image and the frame
		
		//Outputs the feeds to the screen
		//Should use imshow (C++ function) rather than cvShowImage (C function)
		cvShowImage("Processed Aerial Feed", imgThresh);
		cvShowImage("Live Aerial Feed", frameAerial);

		//Wait 10mS
		int c = cvWaitKey(10);

		//If 'ESC' is pressed, break the loop
		if((char)c == 27) 
			break;
	}

	//This should be changed to capAerial::FrameCount
	int frame_count = cvGetCaptureProperty(capAerial, CV_CAP_PROP_FRAME_COUNT); // getting the frame count
	cout<<"Displaying the Frame Count: "<<frame_count<<endl;

	//capAereal::FPS
	double fps1 = cvGetCaptureProperty(capAerial, CV_CAP_PROP_FPS); // getting the frame per second
	cout<<"Displaying the Frame FPS: "<<fps1<<endl;

	//Clean up used images
	cvReleaseImage(&imgHSV);
	cvReleaseImage(&imgThresh);
	cvReleaseImage(&frameAerial);
	cvReleaseImage(&imgTracking);
	cvReleaseCapture(&capAerial);
	cvDestroyAllWindows();
	return 0;
}





/***************************************************
 * Runs the Ground Vision algorithm
 ***************************************************/
void runGroundVision()
{
	int number = 0;
	
	//Get ground camera
	//This should use the C++ VideoCapture not CvCapture*
	//Cam number should be assigned in a constant GROUND_CAM 
	//for convenience and clarity
	//VideoCapture capGround(GROUND_CAM);
	CvCapture* capGround = cvCreateCameraCapture(0);
	
	//use C++ namedWindow() 
	cvNamedWindow("Live Ground Feed", CV_WINDOW_AUTOSIZE);
	cvNamedWindow("Processed Ground Feed", CV_WINDOW_AUTOSIZE);

	//use Mat
	IplImage* imgHSV = NULL;
	IplImage* imgThresh = NULL;

	//wait 1.0 seconds
	//use C++ waitKey()
	cvWaitKey(1000);

	while(true)
	{
		//Gets the next frame
		//Should use C++
		//capGround >> frameGround;
		frameGround = cvQueryFrame(capGround);

		//Allocates memory for the matrix
		//This uses legacy code that will not be necessary once
		//we switch it over to C++
		imgHSV = cvCreateImage(cvGetSize(frameGround), IPL_DEPTH_8U, 3);
		
		//Converts from BGR to HSV
		//Should use cvtColor
		cvCvtColor(frameGround, imgHSV, CV_BGR2HSV);
		
		//Another unnecessary allocation
		imgThresh=cvCreateImage(cvGetSize(imgHSV),IPL_DEPTH_8U, 1);
		
		//This is the color filter
		//Needs to be transitioned to inRangeS() using Scalar()
		cvInRangeS(imgHSV, cvScalar(170,106,60), cvScalar(180,255,255), imgThresh);
		
		//Calculates the moments of the filtered image
		//This should use the C++ functions
		//It should also be improved, since this is a weak
		//detection algorithm
		cvMoments(imgThresh, myMoments, 1);
		double moment10 = cvGetSpatialMoment(myMoments, 1, 0);
		double moment01 = cvGetSpatialMoment(myMoments, 0, 1);
		double area = cvGetCentralMoment(myMoments, 0, 0);
		
		//Shows the live and processed feeds in highGUI windows
		//Should be transitioned to imshow()
		cvShowImage("Live Ground Feed",frameGround);
		cvShowImage("Processed Ground Feed",imgThresh);

		//If the algorithm detects 595 red pixels, it considers the 
		//target acquired at the centroid of the image
		if(area > 595)
		{
			//Compute the centroid
			double posX = moment10/area;
			double posY = moment01/area;
		
			if(lastX>=0 && lastY>=0 && posX>=0 && posY>=0)
			{       
				//Must aquire the target on ten frames
				if(number < 10)
				{
					number++;
				}
				//Target Acquired
				else
				{
					//Outputs the direction signal
					cout<<"Number: "<<number<<" Headed: "<<chgDirection<<endl;
					
					//This should be looked at
					if(posX < 100)
						chgDirection = 'L';
					else if(posX < 500)
						chgDirection = 'S';
					else
						chgDirection = 'R';
				}
			}
			
			//Last known position of the target
			lastX = posX;
			lastY = posY;
			
		}
		else	// Lost sight of the target, cuts down on noise as well
		{
			//Reset the count (which needs to be renamed)
			number = 0;
			chgDirection = 'X';
			cout<<"number: "<<number<<" Headed: "<<chgDirection<<endl;
		}

			//Wait 10ms
			//Change to C++
			char c=cvWaitKey(10);

			//Break if Esc is pressed
			if(c==27)
				break;
	}

	//Clean up used images
	cvReleaseImage(&imgHSV);
	cvReleaseImage(&imgThresh);
	cvReleaseImage(&frameGround);
	cvDestroyAllWindows();
}




/*******************************************************
 * Writes data to the logfile
 * ToDo: We should store more data in the logfile
 *******************************************************/
void write_X_and_y(std::ofstream& outfile, int x, int y, double time)
{
	outfile <<"X = "<<x<<"."<<std::endl;
	outfile <<"Y = "<<y<<"."<<std::endl;
	outfile <<"Mission started at "<<asctime(timeinfo)<<endl;
	outfile <<"Target Acquired in "<<time<<" seconds.\n";
}


/***********************************************
 * Processes the telemetry data transmitted
 * by the quadcopter. The quadcopter transmits
 * data in EFEC format which uses X, Y, Z 
 * coordinates relative to the center of the
 * Earth. This converts them to Lat, Lng, Alt
 ***********************************************/
void parseTelemetryData()
{
	//Gets data from the GPS
	int pos1 = GPSxyz.find(',',4);
	int pos2 = GPSxyz.find(',',pos1+1);
	int pos3 = GPSxyz.find(',',pos2+1);

	//Extracts the EFEC location data
	//Data is in meters from the center of the earth
	double meters_X = (double)atoi(GPSxyz.substr(4, pos1-4).c_str()) / 100;
	double meters_Y = (double)atoi(GPSxyz.substr(pos1+1, pos2-pos1-1).c_str()) / 100;
	double meters_Z = (double)atoi(GPSxyz.substr(pos2+1, pos3-pos2-1).c_str()) / 100;

	//Outputs the EFEC data to the console
	cout<<"\n(Meters)";
	cout<<"\nX: "<<meters_X;
	cout<<"\nY: "<<meters_Y;
	cout<<"\nZ: "<<meters_Z;
	cout<<"\nEFEC format GPS: "<<GPSxyz;
	outfile<<"\nEFEC format GPS: "<<GPSxyz;

	//Converts from EFEC to Lat, Lon, Alt
	//Stores the data in the logfile and outputs it to the console
	double* lla = convertGPS(meters_X, meters_Y, meters_Z);
	cout<<"\n\nLat = "<<lla[0]<<"\nLon = "<<lla[1]<<"\nAlt = "<<lla[2]<<endl;
	outfile<<"\n\nLat = "<<lla[0]<<"\nLon = "<<lla[1]<<"\nAlt = "<<lla[2]<<endl;
	outfile.close();

	//Sets the goal latitude and longitude
	goalLat = lla[0];
	goalLong = lla[1];
	delete lla;
	lla = NULL;
}




/***************************************************
 * Runs the hot-cold object location algorithm.
 * There are a number of ways to improve this.
 ***************************************************/
int groundRobotFindGPSLocation(int *argc, const char * argv[])
{
	//Open logfile
	ofstream fout;
	fout.open("distance.txt");
	
	//Initialize the aria library and instantiate the robot
	Aria::init();
	ArRobot robot;

	//Instantiate the argument parser and connectors
	ArArgumentParser parser(argc,(char**)argv);		//Instantiate argument parser 
	ArSimpleConnector connector(&parser);			//Instantiate connector
	ArGPSConnector gpsConnector(&parser);

	//Loads the default 
	parser.loadDefaultArguments();

	//If the arguments are not recognized, bail
	if(!connector.parseArgs())
	{ 
		cout<<"Unknown settings\n";
		char wait;		
		cin>>wait;
		Aria::exit(0);
		exit(1);	//Exit Main
	} 
	
	//If connection fails, bail
	if (!connector.connectRobot(&robot))
	{ 
		cout<<"Connection Failed.\n";
		char wait;		cin>>wait;
		Aria::exit(0);
		exit(1);	//Exit Main
	}

	//Assign the GPS connector
	GPS = gpsConnector.createGPS(&robot);
	
	//Throws an error if GPS is not working
	assert(GPS);
	
	//Sleep for 1.0 second
	_sleep(1000);

	//If GPS is not working correctly, bail
	if(!GPS || !GPS->connect())
	{
		cout<<"gpsRobotTaskExample: Error connecting to GPS device.  Try -gpsType, -gpsPort, and/or -gpsBaud command-line arguments. Use -help for help. Exiting.\n";
		char wait;		cin>>wait;
		Aria::exit(0);
		exit(1);	//Exit Main
	}

	
	ArKeyHandler keyHandler;
	Aria::setKeyHandler(&keyHandler);
	robot.attachKeyHandler(&keyHandler);
	robot.runAsync(true);					//Prevents robot from crashing if disconnected

	robot.lock();							//Lock robot during set up
	robot.comInt(ArCommands::ENABLE, 1);	//Turn on the motors	[robot.enableMotors();]
	robot.unlock();							//Unlock the robot

	//Converts from lat and long to km
	yGoalKm = goalLat*111.13;
	xGoalKm = goalLong*(111.320*cos(deg2rad(goalLat)));

	//
	robot.lock();
	int r = GPS->read();
	
	
   if(r & ArGPS::ReadError)
	{
		cout<<"\ngpsExample: Warning: error reading GPS data.";
		fout<<"\ngpsExample: Warning: error reading GPS data.";
		return -1;
	}
	robot.unlock();

	//Create a new thread for the ground vision algorithm
	Thread^ grndVisionThread = gcnew Thread(gcnew ThreadStart(runGroundVision));
	grndVisionThread->Start();

	double currLat = GPS->getLatitude();
	double currLong = GPS->getLongitude();
	double yCurrKm = currLat*111.13;
	double xCurrKm = currLong*111.320*cos(deg2rad(currLat));

	double currDist = (yCurrKm-yGoalKm) * (yCurrKm-yGoalKm) + (xCurrKm-xGoalKm) * (xCurrKm-xGoalKm);

	fout<<"\nGoal Latitude: "<<setprecision(9)<<goalLat;
	fout<<"\nGoal Longitude: "<<goalLong;
	fout<<"\nGoal Y Kilometers: "<<yGoalKm;
	fout<<"\nGoal X Kilometers: "<<xGoalKm;
	fout<<"\nOrignal Latitude: "<<currLat;
	fout<<"\nOriginal Longitude: "<<currLong;
	fout<<"\nOrignal Y Kilometers: "<<yCurrKm;
	fout<<"\nOriginal X Kilometers: "<<xCurrKm;
	fout<<"\nOriginal Distance: "<<currDist;
	
	while(true)
	{
		int r = GPS->read();
		if(r & ArGPS::ReadError)
		{
			cout<<"\ngpsExample: Warning: error reading GPS data.";
			fout<<"\ngpsExample: Warning: error reading GPS data.";
			continue;
		}

		double prevDist = currDist;
		currLat = GPS->getLatitude();
		currLong = GPS->getLongitude();
		yCurrKm = currLat * 111.13;
		xCurrKm = currLong * 111.320 * cos(deg2rad(currLat));

		//D = sqrt(X^2 + Y^2)
		currDist = (yCurrKm-yGoalKm) * (yCurrKm-yGoalKm) + (xCurrKm-xGoalKm) * (xCurrKm-xGoalKm); 

		fout<<"\n\nCurrent Latitude: "<<currLat;
		fout<<"\nCurrent Longitude: "<<currLong;
		fout<<"\ncurrent Y Kilometers: "<<yCurrKm;
		fout<<"\nCurrent X Kilometers: "<<xCurrKm;
		fout<<"\nCurrent Distance: "<<currDist;

		//This is the main "hot-cold" algorithm that governs the
		//robot's movement decisions. We are going to want to re-work,
		//or at least re-calibrate this algorithm
		
		//The algorithm checks its current distance to target
		//against a past distance, and changes course (with a sharp 
		//left turn) if it is getting further away from the target,
		//and maintains course if it is getting closer. Its default
		//course has a slight right bias (which is currently not
		//slight enough) to prevent it from getting stuck in a loop.
		//Using a probabilistic inference algorithm, we could drastically 
		//improve the robot's performance.
		if(currDist > prevDist)	// Turn Left Sharply
		{
			robot.setVel2(300,600);
			_sleep(700);
		}
		else
		{
			
		//We will want to have the ground vision work with the gps location algorithm,
		//instead of in place of it. 
		if(chgDirection == 'L')//	Pass control to Ground Vision since we are headed in the right direction
			robot.setVel2(500,540);			// 500, 540
		else if(chgDirection == 'S')
			robot.setVel2(540,540);
		else
			robot.setVel2(540,500);			// Right Hand BIAS
		}
	}

	fout.close();
	// wait for robot task loop to end before exiting the program
	robot.waitForRunExit();
	Aria::exit(0);

	grndVisionThread->Join();
	return 0;
}




/***************************************************
 * The main function initializes each of the 
 * main systems in their respective threads. 
 ***************************************************/
int main(int argc, const char * argv[])
{
	myMoments = new CvMoments();

	//***** Start Mission Planner and connect the quadcopter prior to running this line *****//
	Thread^ telemetryThread = gcnew Thread(gcnew ThreadStart(PortChat::startChat));
	telemetryThread->Start();

	//I don't know if we need this...
	// SETS A SHARED VARIABLE WITH TELEMETRY THREAD (need to disable this line of code:		GPSxyz = PortChat::getGPScoords();	   to test seperately)
	
	//Runs the areal vision algorithm
	runAerialVision();

	//Opens a thread to process the telemetry data
	telemetryThread->Join();
	parseTelemetryData();

	//Ask Nate about this ...
	//GROUND VISION THREAD START IS INSIDE THIS FUNCTION	(need to disable 4 lines if you need to test seperately)
	
	//Runs the ground vision algorithm
	groundRobotFindGPSLocation(&argc, argv);

	//Deletes myMoments ... don't know why this is here
	delete myMoments;
	myMoments = NULL;

	char wait;
	cin>>wait;
	return 0;
}
