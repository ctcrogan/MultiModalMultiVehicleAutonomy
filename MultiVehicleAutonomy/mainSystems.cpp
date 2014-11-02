//
//  Multi-Modal Multi-Vehicle Autonomy Project
//	Project Sponsored by Dr. Ram Bala
//
//	Authors: Christopher Crogan, Peter Carr, Thu Hoang, Justin Smith
//
//  Modified from Combined.cpp (enclosed) written by Nathan Ledoux and Manaswi.
//	Modified with permission from Nathan Ledoux.
//
//  Copyright (c) 2015. All rights reserved.
//

/*******************************************
 * Includes
 *******************************************/
//ARIA Mobile Robotics Interface Library
#include "Aria.h"
#include "ArGPS.h"
#include "ArGPSConnector.h"

//OpenCV Computer Vision Library
#include <opencv/cv.h>					//Core CV functions
#include <opencv2/core/core.hpp>        //Core computer vision structures
#include <opencv2/highgui/highgui.hpp>  //Provides GUI components used in operational display
#include <opencv2/imgproc/imgproc.hpp>  //Core image processing functions and matrix operations


//General
#include "GPS_ECEF_LLA.h"		//GPS Conversions (included in project directory)
#include "PortChat.h"			//Supports port communications (included in project directory)
#include <Windows.h>			//
#include <fstream>
#include <assert.h>
#include <ctime>
#include <string> 

using namespace cv;

/**************************
 * Ground Vehicle Code 
 **************************/
//Mathematical Constants
double const PI = 3.14159265358979323846;
double const EARTH_RADIUS_KM = 6371.0;

//Hard Coded Goal LatLong
double goalLat = 41.627261;		//41.378966;
double goalLong = -71.008224;	//-71.004269;


double yGoalKm, xGoalKm;    //Do these need to be global????

//Nav Command Constants
char const STOP = 'X';
char const TURN_RIGHT = 'R';
char const TURN_LEFT = 'L';
char const DRIVE_STRAIGHT = 'S';

//Nav command - Do we need this to be global???
char navCommand = STOP;

//GPS Variables (Why are these global?)
string GPSxyz;
ArGPS *GPS = NULL;

/***********************
 * Aerial Vehicle Code
 ***********************/
//Confidence filter constants
const int CONFIDENCE_THRESHOLD = 10;
const int AREA_THRESHOLD = 600;

//Camera IDs
int const AIR_CAM = 1;
int const GROUND_CAM = 1;

//Time stuff
struct tm* timeinfo = NULL;
time_t currentTime;
clock_t beginTime;
clock_t endTime;

//I don't know what this does or why it is global
bool found = false;

//Tracks the number of consecutive positive IDs
//Must overcome the confidenceThreshold to trigger
//target acquired
int consecFrameCount = 0;
int framesSinceLastLog = 0;
int imgNumber = 0;
const int FLIGHT_NUMBER = 2;
Moments mmt;

//Stores the current altitude and ground altitude
double alt;
double const GROUND_ALT = -9413;


const double TEST_LAT = 41.627843;
const double TEST_LNG = -71.00218;

ofstream outfile;

//Last known (pixel) coordinates of the target's center
//These values are shared between the air and ground vision algorithms
//which will eventually need to run concurrently. There is no reason for
//these values to be global anyways, since it is safer to pass them by ref
//between the functions
int lastX = -1;
int lastY = -1;

/********************************************************
 * Function Prototype Declarations
 ********************************************************/
	//Main function of the aerial vision algorithm
	int runAerialVision();

	//Converts the GPS coordinates from ECEF to LLA format
	//and outputs and logs the data
	void parseTelemetryData();

	//Main function for the ground vehicle
	void runGroundVision();

	//Uses the "hot-cold" algorithm to find the passed coordinates
	int groundRobotFindGPSLocation(int *argc, const char * argv[]);

	//Apply's vision filter to each frame 
	void applyHSVFilter(Mat imgHSV, Mat imgProcessed);

	//Tracks the detected object
	void trackTarget(Mat imgFrame, Mat imgRaw);

	//Writes output to the logfile
	void write_X_and_y(std::ofstream& outfile, int x, int y, double time);

	//Stores the aerial feeds in a folder
	void generateLog(Mat frame, int flightNumber, int imgNumber);


//Mathematical Support Functions
double deg2rad(double deg) { return (deg * PI / 180); }
double rad2deg(double rad) { return (rad * 180 / PI); }


/***************************************************
 * Converts the image to a binary image based on
 * the color filter.
 ***************************************************/
void applyHSVFilter(Mat imgHSV, Mat * imgProcessed)
{
	//This is unnecessary if we use the C++ functions instead of the C functions
	//We can declare: Mat imgThresh; (which we should rename regardless) which
	//will create a null matrix pointer, and pass it to inRangeS() which is the 
	//C++ equivalent to cvInRangeS(). inRangeS() will handle the initialization,
	//and this prevents memory leakage. 
	//Note: cv::IplImage is a legacy C struct, which has been replaced with cv::Mat
	//in versions 2.x
	inRange(imgHSV, Scalar(170,150,30), Scalar(180,255,255), *imgProcessed); // red
	//inRange(imgHSV, Scalar(104, 178, 70), Scalar(130, 240, 255), imgProcessed);// blue color
	//inRange(imgHSV, Scalar(25, 150, 50), Scalar(50, 255, 255), imgProcessed);// yellow color
	

}




/*************************************************************
 * Uses the zeroth order moment and centroid of the grey-scale
 * image to determine (a) if the object is present
 * and (b) the location of the object. If the object
 * is present, it checks to see if the object has
 * been present for consecutive frames, and when
 * the number of consecutive frames set in the
 * CONFIDENCE_THRESHOLD, it flags the object as found.
 *
 * 
 *************************************************************/
void trackTarget(Mat imgProcessedFrame, Mat imgRawFrame)
{
	//Calculate the moments of the filtered image
	//This should be done using the C++ moments() function
	

	mmt = moments(imgProcessedFrame, true);

	//Compute the centroid x, y
	double x_bar = mmt.m10 / mmt.m00;
	double y_bar = mmt.m01 / mmt.m00;

	//Compute the variance
	double var_x = (mmt.m20 - x_bar * mmt.m10) / mmt.m00;
	double var_y = (mmt.m02 - y_bar * mmt.m01) / mmt.m00;

	//Compute standard deviation
	double std_x = sqrt(var_x);
	double std_y = sqrt(var_y);

	//Compute confidence 
	//ToDo: This uses an arbitrary computation. It should be normalized
	//ToDo: Altitude is not currently considered and should be taken into account
	double confidence_x = mmt.m00 / std_x;
	double confidence_y = mmt.m00 / std_y;

	
	//We should experiment with different values for area
	//and confidence threshold
	if(mmt.m00 > AREA_THRESHOLD) 
	{
		//Stores the centroid of the image in different variables
		//I leave this in for now for convenience, but it should be
		//removed, and the tracking algorithm should be rewritten.
		double posX = x_bar;
		double posY = y_bar;
		
		//This is broken. It is supposed to handle the counting
		//of consecutive positive IDs, but it is not handled properly.
		//It performs adequately in tests, but it does not work in the 
		//manner it is supposed to.
		//There are better tracking algorithms we could use
		if(lastX>=0 && lastY>=0 && posX>=0 && posY>=0)
		{       
			//If numFrames (which needs to be renamed) does not
			//exceed the confidence threshold, increment it, and
			//write to the console.
			if(consecFrameCount < CONFIDENCE_THRESHOLD)
			{
				cout<<"Target Spotted for "<<consecFrameCount<< " consecutive frames."<<endl;
				consecFrameCount++;
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
				imwrite("blackWhiteFrame.jpg" , imgProcessedFrame);
				imwrite("colorFrame.jpg" , imgRawFrame);
			}
		}
		else 
			consecFrameCount = 0;
		
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
	
	//Declare our matrices
	Mat imgRawFeed,			//This is the raw data, and should not be modified
		imgPreProcessed,	//The pre-processed image stored int Hue/Sat/Val format
		imgProcessed;		//The greyscale processed image		


	//Open capture pointer to aerial cam
	VideoCapture capAerial(AIR_CAM);


	
	
	//UI Feed Windows
	namedWindow("Live Aerial Feed");
	namedWindow("Processed Aerial Feed");
	
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
	capAerial >> imgRawFeed;
	//Outputs the VideoCapture Property
	//ToDo: Convert from c to cpp format
	//cout<< cvGetCaptureProperty(capAerial, CV_CAP_PROP_FORMAT)<<endl;

	//Creates a new image that is the same size as the captured frame
	//This should be an unnecessary constructor call, since
	//OpenCV will dynamically allocate memory for the images it creates
	//and will do so more efficiently than this constructor.
	
	//This allocates 1.5kb of RAM for no reason
	//Performs 1.5 thousand writes to memory for no reason

	//iterate through each frames of the video
	//This is the main search loop, and should be rewritten entirely.
	//The major focus of this project will likely be the improvement of this
	//search algorithm.
	while(true)
	{   
		//Get next frame
		capAerial >> imgRawFeed;

			
		//Log Frames for Analysis
		framesSinceLastLog++;
		if (framesSinceLastLog >= 25)
		{
			imgNumber++;
			generateLog(imgRawFeed, FLIGHT_NUMBER, imgNumber);
			framesSinceLastLog = 0;
		}
		
		
		//Run a Gaussian kernel to smooth the image (removes noise)
		GaussianBlur(imgRawFeed, imgPreProcessed, Size(7,7), 0, 0);

		//Converts the image from BGR to HSV format
		cvtColor(imgPreProcessed, imgPreProcessed, CV_BGR2HSV);

		//Crop the outer 10 pixels on each border of 
		//the image to eliminate noise at the edges of the frame
		imgPreProcessed = imgPreProcessed(Rect(10, 10, imgPreProcessed.cols - 10, imgPreProcessed.rows - 10));
		

		//Calls the very short function above, which allocates more memory for
		//the imgHSV (which has already had memory allocated for it in this function)
		//and then makes a single inRangeS call which applies our color filter
		applyHSVFilter(imgPreProcessed, &imgProcessed);
		//inRange(imgPreProcessed, Scalar(170,150,30), Scalar(180,255,255), imgProcessed);
		
		//Smooths the filtered image using the Gaussian to remove noise
		//This should be done using the C++ GuassianBlur rather than
		//cvSmooth
		//cvSmooth(imgThresh, imgThresh, CV_GAUSSIAN,3,3); //smooth the binary image using Gaussian kernel

		//If the target has not yet been flagged, call track object
		//If found, break out
		//This should be handled differently
		if(!found)
			trackTarget(imgProcessed, imgRawFeed);	//track the position of the object
		else
			break;
		
		//Outputs the feeds to the screen
		//Should use imshow (C++ function) rather than cvShowImage (C function)
		imshow("Processed Aerial Feed", imgProcessed);
		imshow("Live Aerial Feed", imgRawFeed);

		

		//Wait 10mS
		int c = cvWaitKey(33);

		//If 'ESC' is pressed, break the loop
		if((char)c == 27) 
			break;
	}

	/*** I don't think we need any of this...
	//This should be changed to capAerial::FrameCount
	int frame_count = cvGetCaptureProperty(capAerial, CV_CAP_PROP_FRAME_COUNT); // getting the frame count
	cout<<"Displaying the Frame Count: "<<frame_count<<endl;

	//capAereal::FPS
	double fps1 = cvGetCaptureProperty(capAerial, CV_CAP_PROP_FPS); // getting the frame per second
	cout<<"Displaying the Frame FPS: "<<fps1<<endl;
	****/

	//Clean up used images
	imgProcessed.release();
	imgPreProcessed.release();
	imgRawFeed.release();
	destroyAllWindows();
	return 0;
}





/***************************************************
 * Runs the Ground Vision algorithm
 ***************************************************/
void runGroundVision()
{
	//Counts consecutive frames on which the target was detected
	int consecFrameCount = 0;
	
	//use C++ namedWindow() 
	namedWindow("Live Ground Feed");
	namedWindow("Processed Ground Feed");

	//Declare our matrices
	Mat imgRawFeed,			//This is the raw data, and should not be modified
		imgPreProcessed,	//The pre-processed image stored int Hue/Sat/Val format
		imgProcessed;		//The greyscale processed image		

	//Get ground camera
	VideoCapture capGround(GROUND_CAM);

	//wait 1.0 seconds
	waitKey(1000);

	while(true)
	{
		//Gets the next frame
		//Should use C++
		capGround >> imgRawFeed;
		
		//Run a Gaussian kernel to smooth the image (removes noise)
		GaussianBlur(imgRawFeed, imgPreProcessed, Size(7,7), 0, 0);
		
		//Converts from BGR to HSV
		cvtColor(imgPreProcessed, imgPreProcessed, CV_BGR2HSV);

		//Crop noise (Don't know if we'll need this, but we should experiment with it)
		imgPreProcessed = imgPreProcessed(Rect(10, 10, imgPreProcessed.cols - 10, imgPreProcessed.rows - 10));
		
		//Apply color filter
		inRange(imgPreProcessed, Scalar(170,106,60), Scalar(180,255,255), imgProcessed);
		
		//Calculates the moments of the filtered image
		//Eventually, all of this code should be moved to a processGroundImage() function
		Moments frameMoments = moments(imgProcessed, true);
		
		//Shows the live and processed feeds in highGUI windows
		//Should be transitioned to imshow()
		imshow("Live Ground Feed", imgRawFeed);
		imshow("Processed Ground Feed", imgProcessed);

		//If the algorithm detects 595 red pixels, it considers the 
		//target acquired at the centroid of the image
		if(frameMoments.m00 > 595)
		{
			//Compute the centroid
			double x_bar = frameMoments.m10 / frameMoments.m00;
			double y_bar = frameMoments.m01 / frameMoments.m00;
		
			if(lastX >= 0 && lastY >= 0 && x_bar >= 0 && x_bar >= 0)
			{       
				//Must aquire the target on ten frames
				if(consecFrameCount < 10)
				{
					consecFrameCount++;
				}
				//Target Acquired
				else
				{
					//Outputs the direction signal
					cout<<"Visual contact for " << consecFrameCount << " frames\nCurrent nav command: " << navCommand << endl;
					
					//This should be looked at.
					//OpenCV has built in image-geometry functions that could improve this
					if(x_bar < 100)
						navCommand = TURN_LEFT;
					else if(x_bar < 500)
						navCommand = DRIVE_STRAIGHT;
					else
						navCommand = TURN_RIGHT;
				}
			}
			
			//Last known position of the target
			lastX = x_bar;
			lastY = x_bar;
			
		}
		else	// Lost sight of the target, cuts down on noise as well
		{
			//Reset the count (which needs to be renamed)
			consecFrameCount = 0;
			navCommand = STOP;
			cout << "Visual Contact Lost!\nCurrent nav command:" << navCommand <<endl;
		}

			//Wait 10ms
			//Change to C++
			char key = waitKey(10);

			//Break if Esc is pressed
			if(key == 'ESC')
				break;
	}

	//Clean up used images
	imgProcessed.release();
	imgPreProcessed.release();
	imgRawFeed.release();
	destroyAllWindows();
}




/*******************************************************
 * Writes data to the logfile
 * ToDo: We should store more data in the logfile
 *******************************************************/
void write_X_and_y(std::ofstream& outfile, int x, int y, double time)
{
	outfile << "X = " << x << "." << endl;
	outfile << "Y = " << y << "." << endl;
	outfile << "Mission started at " << asctime(timeinfo) << endl;
	outfile << "Target Acquired in " << time << " seconds.\n";
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
	int pos1 = GPSxyz.find(',', 4);
	int pos2 = GPSxyz.find(',', pos1 + 1);
	int pos3 = GPSxyz.find(',', pos2 + 1);

	//Extracts the EFEC location data
	//Data is in meters from the center of the earth
	double meters_X = (double) atoi(GPSxyz.substr(4, pos1 - 4).c_str()) / 100;
	double meters_Y = (double) atoi(GPSxyz.substr(pos1+1, pos2-pos1-1).c_str()) / 100;
	double meters_Z = (double) atoi(GPSxyz.substr(pos2+1, pos3-pos2-1).c_str()) / 100;

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
	alt = lla[2];
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
	//NOTE: If default args are used the robot MUST be connected to COM1
	parser.loadDefaultArguments();

	//If the arguments are not recognized, bail 
	//This should not happen when using default args
	if(!connector.parseArgs())
	{ 
		cout<<"Unknown settings\n";
		char wait;		
		cin>>wait;
		Aria::exit(0);
		exit(1);	//Exit Main
	} 
	
	//If connection fails, bail
	//IF default args are used, and the robot is not on COM1, this will fail
	if (!connector.connectRobot(&robot))
	{ 
		cout<<"Connection Failed.\n";
		char wait;		
		cin>>wait;
		Aria::exit(0);
		exit(1);	//Exit Main
	}

	//Assign the GPS connector
	GPS = gpsConnector.createGPS(&robot);
	
	//Mounts the GPS to the robot
	//Throws an error if it fails (this should be handled by our code)
	assert(GPS);
	
	//Sleep for 1.0 second to give the GPS time to connect
	_sleep(1000);

	//If GPS is not working correctly, bail
	if(!GPS || !GPS->connect())
	{
		cout << "Error connecting to GPS device.\n";
		char wait;		
		cin>>wait;
		Aria::exit(0);
		exit(1);	//Exit Main
	}

	
	//This key handler is not currently used. 
	//We should establish a failsafe key
	ArKeyHandler keyHandler;
	Aria::setKeyHandler(&keyHandler);
	robot.attachKeyHandler(&keyHandler);
	
	//Prevents robot from crashing if disconnected
	robot.runAsync(true);					

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

	//Output to console
	fout<<"\nGoal Latitude: "<< setprecision(9) <<goalLat;		//setprecision(int) should be replaced
	fout<<"\nGoal Longitude: "<< goalLong;
	fout<<"\nGoal Y Kilometers: "<< yGoalKm;
	fout<<"\nGoal X Kilometers: "<< xGoalKm;
	fout<<"\nOrignal Latitude: "<< currLat;
	fout<<"\nOriginal Longitude: "<< currLong;
	fout<<"\nOrignal Y Kilometers: "<< yCurrKm;
	fout<<"\nOriginal X Kilometers: "<< xCurrKm;
	fout<<"\nOriginal Distance: "<< currDist;
	
	//Main search routine - runs until the target is found or
	while(true)
	{
		int r = GPS->read();
		
		//GPS connection fault tolarance - continues on a bad GPS read
		if(r & ArGPS::ReadError)
		{
			cout<<"\ngpsExample: Warning: error reading GPS data.";
			fout<<"\ngpsExample: Warning: error reading GPS data.";
			continue;
		}

		//Stores previous distance, then computes the current distance
		double prevDist = currDist;
		currLat = GPS->getLatitude();
		currLong = GPS->getLongitude();
		yCurrKm = currLat * 111.13;
		xCurrKm = currLong * 111.320 * cos(deg2rad(currLat));

		//Computes the Euclidean Distacne to the target: D = sqrt(delta_X ^2 + delta_Y ^2)
		currDist = (yCurrKm-yGoalKm) * (yCurrKm-yGoalKm) + (xCurrKm-xGoalKm) * (xCurrKm-xGoalKm); 

		//Logs lat, long, and distances.
		//ToDo: Use comma-separated-valeus (csv) so we can analyze this with excell
		fout<<"\n\nCurrent Latitude: "<<currLat;
		fout<<"\nCurrent Longitude: "<<currLong;
		fout<<"\ncurrent Y Distance: "<<yCurrKm;
		fout<<"\nCurrent X Distance: "<<xCurrKm;
		fout<<"\nCurrent Euclidean Distance: "<<currDist;
		
		//If the robot is getting further away from the target,
		//Turn left. else continue on course.
		//ToDo: Recalibrate these numbers, since they seem to not work
		//currently. The current algorithm uses a slight-right bias to prevent
		//the robot from getting stuck in a loop.
		if(currDist > prevDist)	// Turn Left Sharply
		{
			robot.setVel2(300,600);
			_sleep(700);				//Algorithm sleeps and the robot turns for 700ms
		}

		else
		{	
			//Parse GVision nav commands
			if(navCommand == TURN_LEFT)//	Pass control to Ground Vision since we are headed in the right direction
				robot.setVel2(500,540);			// 500, 540
		
			else if(navCommand == DRIVE_STRAIGHT)
				robot.setVel2(540,540);

			else //Should there be something here?
				robot.setVel2(540,500);			// Right Hand BIAS
		}
	}

	fout.close();
	
	// wait for robot task loop to end before exiting the program
	robot.waitForRunExit();
	Aria::exit(0);

	//Kill the ground vision thread
	grndVisionThread->Join();
	return 0;
}


/***************************************************
 * Generates a data file containing the flight
 * and image data from the aerial vision algorithm
 * Used for data collection and analysis,
 * this will not be part of the final build.
 ***************************************************/
void generateLog(Mat frame, int flightNumber, int imgNumber)
{
	//Generate the file name
	ostringstream filename;
	filename << "C:\\SeniorDesignData\\aerialCam\\aerialFrame" << flightNumber << "_" << imgNumber << ".jpg";

	cout << (imwrite(filename.str().c_str(), frame) ? "Frame Saved" : "Save Failed!") << endl;
	/*
	//Gets data from the GPS
	int pos1 = GPSxyz.find(',', 4);
	int pos2 = GPSxyz.find(',', pos1 + 1);
	int pos3 = GPSxyz.find(',', pos2 + 1);

	//Extracts the EFEC location data
	//Data is in meters from the center of the earth
	double meters_X = (double) atoi(GPSxyz.substr(4, pos1 - 4).c_str()) / 100;
	double meters_Y = (double) atoi(GPSxyz.substr(pos1+1, pos2-pos1-1).c_str()) / 100;
	double meters_Z = (double) atoi(GPSxyz.substr(pos2+1, pos3-pos2-1).c_str()) / 100;
	*/


}

/***************************************************
 * The main function initializes each of the 
 * main systems in their respective threads. 
 ***************************************************/
int main(int argc, const char * argv[])
{

	//***** Start Mission Planner and connect the quadcopter prior to running this line *****//
	Thread^ telemetryThread = gcnew Thread(gcnew ThreadStart(PortChat::startChat));
	telemetryThread->Start();


	
	// SETS A SHARED VARIABLE WITH TELEMETRY THREAD (need to disable this line of code:		GPSxyz = PortChat::getGPScoords();	   to test seperately)
	
	//Runs the aereal vision algorithm
	runAerialVision();

	//Opens a thread to process the telemetry data
	telemetryThread->Join();
	parseTelemetryData();

	//Ask Nate about this ...
	//GROUND VISION THREAD START IS INSIDE THIS FUNCTION	(need to disable 4 lines if you need to test seperately)
	
	//Runs the ground vision algorithm
	groundRobotFindGPSLocation(&argc, argv);

	char wait;
	cin>>wait;
	return 0;
}
