#include "ardrone/ardrone.h"
#include <opencv2/aruco.hpp>
#include <opencv2/objdetect.hpp>
#include <math.h>

#include "pid.hpp"
#include <cstring>
#define PI 3.14159265

// Parameter for calibration pattern
#define PAT_ROWS (6)	  // Rows of pattern
#define PAT_COLS (9)	  // Columns of pattern
#define CHESS_SIZE (22.0) // Size of a pattern [mm]

// Checks if a matrix is a valid rotation matrix.
bool isRotationMatrix(Mat &R)
{
	Mat Rt;
	transpose(R, Rt);
	Mat shouldBeIdentity = Rt * R;
	Mat I = Mat::eye(3, 3, shouldBeIdentity.type());

	return norm(I, shouldBeIdentity) < 1e-6;
}

// Calculates rotation matrix to euler angles
// The result is the same as MATLAB except the order
// of the euler angles ( x and z are swapped ).
Vec3f rotationMatrixToEulerAngles(Mat &R)
{

	assert(isRotationMatrix(R));

	float sy = sqrt(R.at<double>(0, 0) * R.at<double>(0, 0) + R.at<double>(1, 0) * R.at<double>(1, 0));

	bool singular = sy < 1e-6; // If

	float x, y, z;
	if (!singular)
	{
		x = atan2(R.at<double>(2, 1), R.at<double>(2, 2));
		y = atan2(-R.at<double>(2, 0), sy);
		z = atan2(R.at<double>(1, 0), R.at<double>(0, 0));
	}
	else
	{
		x = atan2(-R.at<double>(1, 2), R.at<double>(1, 1));
		y = atan2(-R.at<double>(2, 0), sy);
		z = 0;
	}
	return Vec3f(x, y, z);
}

// --------------------------------------------------------------------------
// main(Number of arguments, Argument values)
// Description  : This is the entry point of the program.
// Return value : SUCCESS:0  ERROR:-1
// --------------------------------------------------------------------------
int main(int argc, char *argv[])
{
	// AR.Drone class
	PIDManager PID("pid.yaml");

	ARDrone ardrone;

	// Initialize
	if (!ardrone.open())
	{
		std::cout << "Failed to initialize." << std::endl;
		return -1;
	}

	// Battery
	std::cout << "Battery = " << ardrone.getBatteryPercentage() << "[%]" << std::endl;

	// Instructions
	std::cout << "***************************************" << std::endl;
	std::cout << "*       CV Drone sample program       *" << std::endl;
	std::cout << "*           - How to play -           *" << std::endl;
	std::cout << "***************************************" << std::endl;
	std::cout << "*                                     *" << std::endl;
	std::cout << "* - Controls -                        *" << std::endl;
	std::cout << "*    'Space' -- Takeoff/Landing       *" << std::endl;
	std::cout << "*    'Up'    -- Move forward          *" << std::endl;
	std::cout << "*    'Down'  -- Move backward         *" << std::endl;
	std::cout << "*    'Left'  -- Turn left             *" << std::endl;
	std::cout << "*    'Right' -- Turn right            *" << std::endl;
	std::cout << "*    'Q'     -- Move upward           *" << std::endl;
	std::cout << "*    'A'     -- Move downward         *" << std::endl;
	std::cout << "*                                     *" << std::endl;
	std::cout << "* - Others -                          *" << std::endl;
	std::cout << "*    'C'     -- Change camera         *" << std::endl;
	std::cout << "*    'Esc'   -- Exit                  *" << std::endl;
	std::cout << "*                                     *" << std::endl;
	std::cout << "***************************************" << std::endl;

	// Calibration loading
	// Open XML file
	std::string filename("camera.xml");
	std::string filename_2("camera_down.xml");
	cv::FileStorage fs(filename, cv::FileStorage::READ);
	cv::FileStorage fs_2(filename_2, cv::FileStorage::READ);

	// Load camera parameters
	cv::Mat cameraMatrix, distCoeffs;
	fs["intrinsic"] >> cameraMatrix;
	fs["distortion"] >> distCoeffs;

	// Create undistort map
	cv::Mat mapx, mapy;
	cv::Mat frame = ardrone.getImage();
	cv::initUndistortRectifyMap(cameraMatrix, distCoeffs, cv::Mat(), cameraMatrix, frame.size(), CV_32FC1, mapx, mapy);

	// Marker detection variable
	cv::Ptr<cv::aruco::Dictionary> dictionary;
	dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
	std::vector<int> ids;
	std::vector<std::vector<cv::Point2f> > corners;
	std::vector<cv::Vec3d> rvecs, tvecs;
	float markerLength = 9.4;

	//autopolit
	Mat error = Mat(4, 1, CV_64F);
	Mat output = Mat(4, 1, CV_64F);
	//chack if steady enough to go to next id
	const int stady_const[5] = {250, 25, 20, 25, 50};
	int stady_count[5] = {0};
	bool setp_finding = true;
	bool step_upfront = false;
	bool step_id_1 = false;
	bool step_id_2 = false;
	bool step_id_3 = false;
	bool step_id_4 = false;
	bool step_id_5 = false;
	bool step_id_6 = false;
	bool face_detection = false;
	bool landing = false;
	int ltcantfi = 85;
	int counter = 0;
	bool go_to_next_1 = false;
	bool go_to_next_2 = false;
	bool priority_face = false;

	int face_scan_rate = 0;

	//face detection
	CascadeClassifier CC;
	CC.load("haarcascade_frontalface_alt2.xml");

	int last_vx = 0;
	int last_vy = 0;
	int last_vr = 0;

	while (1)
	{

		// Key input
		int key = cv::waitKey(33);
		if (key == 0x1b)
			break;

		// Get an image
		cv::Mat image = ardrone.getImage();

		// Take off / Landing
		if (key == ' ')
		{
			if (ardrone.onGround())
				ardrone.takeoff();
			else
				ardrone.landing();
		}

		// Move
		double vx = 0.0, vy = 0.0, vz = 0.0, vr = 0.0;
		if (key == 'i' || key == CV_VK_UP || key == 'e')
			vx = 1.0;
		if (key == 'k' || key == CV_VK_DOWN || key == 'd')
			vx = -1.0;
		if (key == 'u' || key == CV_VK_LEFT|| key == 'w')
			vr = 1.0;
		if (key == 'o' || key == CV_VK_RIGHT|| key == 'r')
			vr = -1.0;
		if (key == 'j'|| key == 's')
			vy = 1.0;
		if (key == 'l'|| key == 'f')
			vy = -1.0;
		if (key == 'q')
			vz = 1.0;
		if (key == 'a')
			vz = -1.0;

		// Change camera
		static int mode = 0;
		if (key == 'c')
		{
			ardrone.setCamera(++mode % 4);
			// //down_camera_calibration
			// fs_2["intrinsic"] >> cameraMatrix;
			// fs_2["distortion"] >> distCoeffs;
			// // calculate undistort map
			// cv::Mat frame = ardrone.getImage();
			// cv::initUndistortRectifyMap(cameraMatrix, distCoeffs, cv::Mat(), cameraMatrix, frame.size(), CV_32FC1, mapx, mapy);
		}

		// Undistort
		cv::remap(image, image, mapx, mapy, cv::INTER_LINEAR);
		// Detect marker
		cv::aruco::detectMarkers(image, dictionary, corners, ids);

		if (ids.size() > 0)
		{
			// Draw out the boarder of marker
			cv::aruco::drawDetectedMarkers(image, corners, ids);
			// Pose estimation
			cv::aruco::estimatePoseSingleMarkers(corners, markerLength, cameraMatrix, distCoeffs, rvecs, tvecs);
			// Draw
			// for (int i = 0; i < ids.size(); i++)
			// {
			// 	cv::aruco::drawAxis(image, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 12.3);
			// 	// char str[100];
			// 	// sprintf(str, "%g %g %g", tvecs[i][0], tvecs[i][1], tvecs[i][2]);
			// 	// cv::putText(image, str, cv::Point(20, 40), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0, 255, 0));
			// }
		}

		// tvecs[i][2] -> deep
		// tvecs[i][0] -> left hand +
		// tvecs[i][1] -> height +

		// rvecs[i][2] -> rotate
		// rvecs[i][0] -> up and down
		// rvecs[i][1] -> slide

		//vz -> up and down
		//vy -> left and right slide
		//vx -> proceed and backward
		//vr -> rotate left + right -

		if (key == -1)
		{
			vx = 0;
			vy = 0;
			vr = 0;
			//face detection
			face_scan_rate++;
			if (face_scan_rate >= 1 && face_detection == true)
			{
				vector<Rect> faces;
				CC.detectMultiScale(image, faces, 1.05, 25, 0 | CASCADE_SCALE_IMAGE, Size(35, 35));
				for (size_t i = 0; i < faces.size(); i++)
				{
					int scale = 1;
					Rect r = faces[i];
					Scalar color = Scalar(0, 255, 255);
					double aspect_ratio = (double)r.width / r.height;
					rectangle(image, cvPoint(cvRound(r.x * scale), cvRound(r.y * scale)),
							  cvPoint(cvRound((r.x + r.width - 1) * scale), cvRound((r.y + r.height - 1) * scale)),
							  color, 3, 8, 0);

					std::cout << r.width << " " << r.x << " " << r.y << endl;
					if(r.width > 68 && r.x < 550 &&  r.x > 200)
					{
						vx = 0.5;
						vy = -0.8;
						vr = 0;
						vz = 0;
						ardrone.move3D(vx, vy, vz, vr);
						ardrone.move3D(vx, vy, vz, vr);
						ardrone.move3D(vx, vy, vz, vr);
						priority_face = true;
					}
					if (r.width > 68 && r.x <= 200 && go_to_next_1 == false)
					{

						if (step_id_1 == true)
						{
							step_id_2 = true;
							step_id_1 = false;
							go_to_next_1 = true;
							std::cout << "Go to next step_id_2" << endl;
							vx = 2;
							vy = 0;
							vr = 0;
							vz = 0;
							ardrone.move3D(vx, vy, vz, vr);
							ardrone.move3D(vx, vy, vz, vr);
							ardrone.move3D(vx, vy, vz, vr);
							priority_face = true;
						}
						else if (step_id_2 == true)
						{
							step_id_3 = true;
							step_id_2 = false;
							go_to_next_1 = true;
							std::cout << "Go to next step_id_3" << endl;
							vx = 4;
							vy = 0;
							vr = 0;
							vz = 0;
							ardrone.move3D(vx, vy, vz, vr);
							ardrone.move3D(vx, vy, vz, vr);
							ardrone.move3D(vx, vy, vz, vr);
							priority_face = true;
						}
					}
					if (r.width > 68 && r.x <= 200 && go_to_next_2 == false)
					{

						if (step_id_5 == true)
						{
							landing = true;
							step_id_5 = false;
							go_to_next_2 = true;
							std::cout << "Go to next step_id_6(landing)" << endl;
							vx = 4;
							vy = 0;
							vr = 0;
							vz = 0;
							ardrone.move3D(vx, vy, vz, vr);
							ardrone.move3D(vx, vy, vz, vr);
							ardrone.move3D(vx, vy, vz, vr);
							priority_face = true;
						}
						else if (step_id_6 == true)
						{
							landing = true;
							step_id_6 = false;
							go_to_next_2 = true;
							std::cout << "Go to landing" << endl;
							vx = 3;
							vy = 0;
							vr = 0;
							vz = 0;
							ardrone.move3D(vx, vy, vz, vr);
							ardrone.move3D(vx, vy, vz, vr);
							ardrone.move3D(vx, vy, vz, vr);
							priority_face = true;
						}
					}

					// approximate : 90
				}
				face_scan_rate = 0;
			}

			if(priority_face == true) {
				priority_face = false;
				continue;
			}

			//flying and finding -> 11
			if (setp_finding == true && ids.size() == 0)
			{
				vr = -0.1;
			}
			else if (setp_finding == true)
			{
				if (rvecs[0][0] < 0.0)
				{
					rvecs[0][2] = -rvecs[0][2];
				}

				if (ids[0] == 11) // not only see id1, but also id1 is near image center
				{
					setp_finding = false;
					step_upfront = true;
					std::cout << "first time find marker11" << std::endl;
					vr = -2;
					counter = 0;
				}
				else
				{
					vr = -0.1; // not id1, or id1 is not near image center, keep turning
				}
			}

			counter++;
			if (!setp_finding && counter >= ltcantfi)
			{
				vx = 0;
				vy = 0;
				vz = 0;
				vr = 0;
				if (!landing)
					vr = -0.1;
				else
					vz = 0.05;
				std::cout << "I am finding next marker now\n";
			}

			//find ids
			for (int i = 0; i < ids.size(); i++)
			{
				// calculate the angle of rotation
				Mat a = Mat(3, 1, CV_64F);
				a.at<double>(0, 0) = rvecs[i][0];
				a.at<double>(1, 0) = rvecs[i][1];
				a.at<double>(2, 0) = rvecs[i][2];
				Mat rotation3x3;
				cv::Rodrigues(a, rotation3x3);
				Vec3f eurla_rotate = rotationMatrixToEulerAngles(rotation3x3);
				if (eurla_rotate[0] < 0)
				{
					eurla_rotate[1] = -eurla_rotate[1];
				}

				// fylying to id_11
				if (step_upfront == true && ids[i] == 11)
				{
					counter = 0;

					std::cout << "in 11" << std::endl;
					//aimming to marker 11's right 20cm front 50cm
					error.at<double>(0, 0) = tvecs[i][2] - 60 * sin(eurla_rotate[1] / 3.1415926 * 180);
					error.at<double>(1, 0) = tvecs[i][0] + 60 * cos(eurla_rotate[1] / 3.1415926 * 180);
					error.at<double>(2, 0) = tvecs[i][1];
					// error.at<double>(3, 0) = abs(eurla_rotate[1] / 3.1415926 * 180) - 45;
					if (rvecs[i][0] < 0.0)
					{
						error.at<double>(3, 0) = -rvecs[i][2];
					} else {
						error.at<double>(3, 0) = rvecs[i][2];
					}

					PID.getCommand(error, output);

					if (abs(error.at<double>(0, 0)) > 7)
					{
						vx = 0.01 * output.at<double>(0, 0);
					}

					// if (tvecs[i][2] > 100 && abs(error.at<double>(3, 0)) > 0.1)
					// {
					// 	vr = 1.0 * output.at<double>(3, 0);
					// }

					if (tvecs[i][2] < 100 && abs(error.at<double>(1, 0)) > 2)
					{
						vy = -0.01 * output.at<double>(1, 0);
					}

					if (sqrt(pow(tvecs[i][2], 2) + pow(tvecs[i][2], 2)) < 110) //(abs(error.at<double>(0, 0)) < 25 && abs(error.at<double>(1, 0)) < 25)
					{
						step_upfront = false;
						step_id_1 = true;
						face_detection = true;
						vx = 0;
						vy = 0;
						vz = 0;
						vr = -2;
						std::cout << "ok fuck bye 11 (dis)\n";
					}

					// if (stady_count[0] == stady_const[0])
					// {
					// 	step_upfront = false;
					// 	step_id_1 = true;
					// 	vx = 0;
					// 	vy = 0;
					// 	vz = 0;
					// 	vr = 0;
					// 	std::cout << "ok fuck bye 1(const)\n";
					// }

					// stady_count[0]++;
					//vr = 0;
					//vr = -0.02 * output.at<double>(3, 0);
					//char str[100];
					//eurla_rotate[0], eurla_rotate[1] / 3.1415926 * 180
					//sprintf(str, "%5g %5g %5g", vx, vy, error.at<double>(1, 0)); //,// vz);
					//cv::putText(image, str, cv::Point(20, 40), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0, 255, 0));
					std::cout << ids[i] << " " << sqrt(pow(tvecs[i][2], 2) + pow(tvecs[i][2], 2)) << endl;
				}

				// flying to id_12
				else if (step_id_1 == true && ids[i] == 12)
				{
					counter = 0;

					std::cout << "in 12" << std::endl;
					//aimming to marker 12's right 20cm front 50cm
					error.at<double>(0, 0) = tvecs[i][2] - 100 * sin(eurla_rotate[1] / 3.1415926 * 180);
					error.at<double>(1, 0) = tvecs[i][0] + 100 * cos(eurla_rotate[1] / 3.1415926 * 180);
					error.at<double>(2, 0) = tvecs[i][1];
					if (rvecs[i][0] < 0.0)
					{
						error.at<double>(3, 0) = -rvecs[i][2];
					}
					else
					{
						error.at<double>(3, 0) = rvecs[i][2];
					}

					PID.getCommand(error, output);


					if (abs(error.at<double>(0, 0)) > 7)
					{
						vx = 0.01 * output.at<double>(0, 0);
					}
					

					// if (tvecs[i][2] > 130 && abs(error.at<double>(3, 0)) > 0.1)
					// {
					// 	vr = 1.2 * output.at<double>(3, 0);
					// }

					if (abs(error.at<double>(1, 0)) > 2)
					{
						vy = -0.01 * output.at<double>(1, 0);
					}

					if (sqrt(pow(tvecs[i][2], 2) + pow(tvecs[i][2], 2)) < 130) //(abs(error.at<double>(0, 0)) < 20 && abs(error.at<double>(1, 0)) < 20)
					{
						//stady_count[1]++;
						step_id_1 = false;
						step_id_2 = true;
						vx = 0;
						vy = 0;
						vz = 0;
						vr = -2;
						std::cout << "ok fuck bye 12 (dis)\n";
					}
					// if (stady_count[1] == stady_const[1])
					// {
					// 	step_id_1 = false;
					// 	step_id_3 = true;
					// 	vx = 0;
					// 	vy = 0;
					// 	vz = 0;
					// 	vr = -1;
					// 	std::cout << "ok fuck bye 2 (dis)\n";
					// }
					//vr = 0;
					//vr = -0.02 * output.at<double>(3, 0);
					//char str[100];
					//eurla_rotate[0], eurla_rotate[1] / 3.1415926 * 180
					//sprintf(str, "%5g %5g %5g", vx, vy, error.at<double>(1, 0)); //,// vz);
					//cv::putText(image, str, cv::Point(20, 40), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0, 255, 0));
					std::cout << ids[i] << " " << sqrt(pow(tvecs[i][2], 2) + pow(tvecs[i][2], 2)) << endl;
				}
				// if seeing id_13, go to it
				// else if (step_id_1 == true && ids[i] == 13)
				// {
				// 	counter = 0;

				// 	std::cout << "in 13" << std::endl;
				// 	//aimming to marker 13's right 30cm
				// 	error.at<double>(0, 0) = tvecs[i][2] - 30 * sin(eurla_rotate[1] / 3.1415926 * 180) - 40;
				// 	error.at<double>(1, 0) = tvecs[i][0] + 30 * cos(eurla_rotate[1] / 3.1415926 * 180);
				// 	error.at<double>(2, 0) = tvecs[i][1];
				// 	if (rvecs[i][0] < 0.0)
				// 	{
				// 		error.at<double>(3, 0) = -rvecs[i][2];
				// 	}
				// 	else
				// 	{
				// 		error.at<double>(3, 0) = rvecs[i][2];
				// 	}

				// 	PID.getCommand(error, output);

				// 	if (abs(error.at<double>(0, 0)) > 7)
				// 	{
				// 		vx = 0.0005 * output.at<double>(0, 0);
				// 	}
				// 	else
				// 	{
				// 		//stady_count[2]++;
				// 		step_id_1 = false;
				// 		step_id_3 = true;
				// 		vx = 0;
				// 		vy = 0;
				// 		vz = 0;
				// 		vr = 0;
				// 		std::cout << "ok fuck bye 13 (dis)\n";
				// 	}

				// 	// if (tvecs[i][2] > 130 && abs(error.at<double>(3, 0)) > 0.1)
				// 	// {
				// 	// 	vr = 1.2 * output.at<double>(3, 0);
				// 	// }

				// 	if (abs(error.at<double>(1, 0)) > 2)
				// 	{
				// 		vy = -0.01 * output.at<double>(1, 0);
				// 	}

				// 	// if (stady_count[2] == stady_const[2])
				// 	// {
				// 	// 	step_id_1 = false;
				// 	// 	step_id_3 = true;
				// 	// 	vx = 0;
				// 	// 	vy = 0;
				// 	// 	vz = 0;
				// 	// 	vr = -1;
				// 	// 	std::cout << "ok fuck bye 2 (dis)\n";
				// 	// }
				// 	//vr = 0;
				// 	//vr = -0.02 * output.at<double>(3, 0);
				// 	char str[100];
				// 	//eurla_rotate[0], eurla_rotate[1] / 3.1415926 * 180
				// 	sprintf(str, "%5g %5g %5g", vx, vy, error.at<double>(0, 0)); //,// vz);
				// 	cv::putText(image, str, cv::Point(20, 40), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0, 255, 0));
				// }
				// //flying to id_2
				// else if (step_id_1 == true && ids[i] == 2)
				// {

				// 	counter = 0;
				// 	std::cout << "in 2\n";
				// 	error.at<double>(0, 0) = tvecs[i][2] - 110;
				// 	error.at<double>(1, 0) = tvecs[i][0];
				// 	error.at<double>(2, 0) = tvecs[i][1];
				// 	if (rvecs[i][0] < 0.0)
				// 	{
				// 		error.at<double>(3, 0) = -rvecs[i][2];
				// 	}
				// 	else
				// 	{
				// 		error.at<double>(3, 0) = rvecs[i][2];
				// 	}

				// 	PID.getCommand(error, output);

				// 	if (abs(error.at<double>(3, 0)) > 0.1 && tvecs[i][2] < 150)
				// 	{
				// 		vr = 1.2 * output.at<double>(3, 0);
				// 	}
				// 	else
				// 	{
				// 		vr = 0;
				// 	}

				// 	if (abs(error.at<double>(0, 0)) > 7)
				// 	{
				// 		vx = 0.005 * output.at<double>(0, 0);
				// 	}
				// 	else
				// 	{
				// 		stady_count[2]++;
				// 		vx = 0;
				// 	}

				// 	//if stady enought, then go to next step
				// 	if (stady_count[2] == stady_const[2])
				// 	{
				// 		step_id_1 = false;
				// 		step_id_4 = true;
				// 		face_detection = false;
				// 		vx = 0;
				// 		vy = 0;
				// 		vz = 0;
				// 		vr = -1;
				// 		std::cout << "ok fuck bye 2 (dis)\n";
				// 	}
				// 	//vr = 0;
				// 	//vr = -0.02 * output.at<double>(3, 0);
				// 	char str[100];
				// 	//eurla_rotate[0], eurla_rotate[1] / 3.1415926 * 180
				// 	sprintf(str, "%5g %5g %5g", vx, vy, error.at<double>(0, 0)); //,// vz);
				// 	cv::putText(image, str, cv::Point(20, 40), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0, 255, 0));
				// }

				// flying to id_13, might encounter some face
				else if (step_id_2 == true && ids[i] == 13)
				{
					counter = 0;

					std::cout << "in 13" << std::endl;
					//aimming to marker 13's right 30cm
					error.at<double>(0, 0) = tvecs[i][2] - 100 * sin(eurla_rotate[1] / 3.1415926 * 180);
					error.at<double>(1, 0) = tvecs[i][0] + 100 * cos(eurla_rotate[1] / 3.1415926 * 180);
					error.at<double>(2, 0) = tvecs[i][1];
					if (rvecs[i][0] < 0.0)
					{
						error.at<double>(3, 0) = -rvecs[i][2];
					}
					else
					{
						error.at<double>(3, 0) = rvecs[i][2];
					}

					PID.getCommand(error, output);

					if (abs(error.at<double>(0, 0)) > 7)
					{
						vx = 0.01 * output.at<double>(0, 0);
					}
					

					// if (tvecs[i][2] > 130 && abs(error.at<double>(3, 0)) > 0.1)
					// {
					// 	vr = 1.2 * output.at<double>(3, 0);
					// }

					if (abs(error.at<double>(1, 0)) > 2) //tvecs[i][2] < 100 &&
					{
						vy = -0.01 * output.at<double>(1, 0);
					}

					if (sqrt(pow(tvecs[i][2], 2) + pow(tvecs[i][2], 2)) < 130) //(abs(error.at<double>(0, 0)) < 20 && abs(error.at<double>(1, 0)) < 20)
					{
						//stady_count[2]++;
						step_id_2 = false;
						step_id_3 = true;
						vx = 0;
						vy = 0;
						vz = 0;
						vr = -1;
						std::cout << "ok fuck bye 13 (dis)\n";
					}
					// if (stady_count[2] == stady_const[2])
					// {
					// 	step_id_1 = false;
					// 	step_id_3 = true;
					// 	vx = 0;
					// 	vy = 0;
					// 	vz = 0;
					// 	vr = -1;
					// 	std::cout << "ok fuck bye 2 (dis)\n";
					// }
					//vr = 0;
					//vr = -0.02 * output.at<double>(3, 0);
					//char str[100];
					//eurla_rotate[0], eurla_rotate[1] / 3.1415926 * 180
					//sprintf(str, "%5g %5g %5g", vx, vy, error.at<double>(1, 0)); //,// vz);
					//cv::putText(image, str, cv::Point(20, 40), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0, 255, 0));
					std::cout << ids[i] << " " << sqrt(pow(tvecs[i][2], 2) + pow(tvecs[i][2], 2)) << endl;
				}
				// //flying to id_2
				// else if (step_id_2 == true && ids[i] == 2)
				// {
				// 	counter = 0;
				// 	std::cout << "in 2\n";
				// 	error.at<double>(0, 0) = tvecs[i][2] - 110;
				// 	error.at<double>(1, 0) = tvecs[i][0];
				// 	error.at<double>(2, 0) = tvecs[i][1];
				// 	if (rvecs[i][0] < 0.0)
				// 	{
				// 		error.at<double>(3, 0) = -rvecs[i][2];
				// 	}
				// 	else
				// 	{
				// 		error.at<double>(3, 0) = rvecs[i][2];
				// 	}

				// 	PID.getCommand(error, output);

				// 	if (abs(error.at<double>(3, 0)) > 0.1 && tvecs[i][2] < 150)
				// 	{
				// 		vr = 1.2 * output.at<double>(3, 0);
				// 	}
				// 	else
				// 	{
				// 		vr = 0;
				// 	}

				// 	if (abs(error.at<double>(0, 0)) > 7)
				// 	{
				// 		vx = 0.01 * output.at<double>(0, 0);
				// 	}
				// 	else
				// 	{
				// 		stady_count[2]++;
				// 		vx = 0;
				// 	}

				// 	//if stady enought, then go to next step
				// 	if (stady_count[2] == stady_const[2])
				// 	{
				// 		step_id_2 = false;
				// 		step_id_4 = true;
				// 		face_detection = false;
				// 		vx = 0;
				// 		vy = 0;
				// 		vz = 0;
				// 		vr = -1;
				// 		std::cout << "ok fuck bye 2 (dis)\n";
				// 	}
				// 	//vr = 0;
				// 	//vr = -0.02 * output.at<double>(3, 0);
				// 	char str[100];
				// 	//eurla_rotate[0], eurla_rotate[1] / 3.1415926 * 180
				// 	sprintf(str, "%5g %5g %5g", vx, vy, error.at<double>(0, 0)); //,// vz);
				// 	cv::putText(image, str, cv::Point(20, 40), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0, 255, 0));
				// }

				//flying to id_2
				else if (step_id_3 == true && ids[i] == 2)
				{

					counter = 0;
					std::cout << "in 2\n";
					error.at<double>(0, 0) = tvecs[i][2] - 80 * cos(eurla_rotate[1] / 3.1415926 * 180);
					error.at<double>(1, 0) = tvecs[i][0] - 80 * sin(eurla_rotate[1] / 3.1415926 * 180);
					error.at<double>(2, 0) = tvecs[i][1];
					if (rvecs[i][0] < 0.0)
					{
						error.at<double>(3, 0) = -rvecs[i][2];
					}
					else
					{
						error.at<double>(3, 0) = rvecs[i][2];
					}

					PID.getCommand(error, output);

					if (abs(error.at<double>(3, 0)) > 0.08 && sqrt(pow(tvecs[i][2], 2) + pow(tvecs[i][2], 2)) < 130)
					{
						vr = 1.1 * output.at<double>(3, 0);
					}
					else
					{
						vr = 0;
					}

					// if (abs(error.at<double>(1, 0)) > 2 && sqrt(pow(abs(error.at<double>(0, 0)), 2) + pow(abs(error.at<double>(1, 0)), 2)) < 200) //tvecs[i][2] < 100 &&
					// {
					// 	vy = -0.01 * output.at<double>(1, 0);
					// }

					if (abs(error.at<double>(0, 0)) > 7 )
					{
						vx = 0.01 * output.at<double>(0, 0);
					}
					else
					{
						stady_count[2]++;
						vx = 0;
					}

					

					//if stady enought, then go to next step
					if (sqrt(pow(tvecs[i][2], 2) + pow(tvecs[i][2], 2)) < 115) //(stady_count[2] == stady_const[2])
					{
						step_id_3 = false;
						step_id_4 = true;
						vx = 0;
						vy = 0;
						vz = 0;
						vr = -1;
						std::cout << "ok fuck bye 2 (dis)\n";
					}
					//vr = 0;
					//vr = -0.02 * output.at<double>(3, 0);
					//char str[100];
					//eurla_rotate[0], eurla_rotate[1] / 3.1415926 * 180
					// sprintf(str, "%5g %5g %5g", vx, vy, error.at<double>(1, 0)); //,// vz);
					// cv::putText(image, str, cv::Point(20, 40), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0, 255, 0));
					std::cout << ids[i]  <<  " " << sqrt(pow(tvecs[i][2], 2) + pow(tvecs[i][2], 2)) << endl;
				}

				//flying to id_3
				else if (step_id_4 == true && ids[i] == 3)
				{

					counter = 0;
					std::cout << "in 3\n";
					error.at<double>(0, 0) = tvecs[i][2] - 60 * cos(eurla_rotate[1] / 3.1415926 * 180);
					error.at<double>(1, 0) = tvecs[i][0] - 60 * sin(eurla_rotate[1] / 3.1415926 * 180);
					error.at<double>(2, 0) = tvecs[i][1];
					if (rvecs[i][0] < 0.0)
					{
						error.at<double>(3, 0) = -rvecs[i][2];
					}
					else
					{
						error.at<double>(3, 0) = rvecs[i][2];
					}

					PID.getCommand(error, output);

					if (abs(error.at<double>(3, 0)) > 0.08 && sqrt(pow(tvecs[i][2], 2) + pow(tvecs[i][2], 2)) < 125)
					{
						vr = 1.1 * output.at<double>(3, 0);
					}
					else
					{
						vr = 0;
					}

				    // if (abs(error.at<double>(1, 0)) > 2 && sqrt(pow(abs(error.at<double>(0, 0)), 2) + pow(abs(error.at<double>(1, 0)), 2)) < 200) //tvecs[i][2] < 100 &&
					// {
					// 	vy = -0.01 * output.at<double>(1, 0);
					// }
					if (abs(error.at<double>(0, 0)) > 7)
					{
						vx = 0.01 * output.at<double>(0, 0);
					}
					else
					{
						vx = 0;
					}

					if (sqrt(pow(tvecs[i][2], 2) + pow(tvecs[i][2], 2)) < 110)
					{
						step_id_4 = false;
						step_id_5 = true;
						face_detection = true;
						vx = 0;
						vy = 0;
						vz = 0;
						vr = -1;
						std::cout << "ok fuck bye 3 (dis)\n";
					}

					//vr = 0;
					//vr = -0.02 * output.at<double>(3, 0);
					char str[100];
					//eurla_rotate[0], eurla_rotate[1] / 3.1415926 * 180
					sprintf(str, "%5g %5g %5g", vy, error.at<double>(0, 0), error.at<double>(1, 0)); //,// vz);
					cv::putText(image, str, cv::Point(20, 40), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0, 255, 0));
					std::cout << ids[i] << " " << sqrt(pow(tvecs[i][2], 2) + pow(tvecs[i][2], 2)) << endl;
				}

				//flying to id_21
				else if (step_id_5 == true && ids[i] == 21)
				{
					counter = 0;
					std::cout << "in 21\n";
					//aimming to marker 21's right 30cm
					error.at<double>(0, 0) = tvecs[i][2] - 60 * sin(eurla_rotate[1] / 3.1415926 * 180);
					error.at<double>(1, 0) = tvecs[i][0] + 60 * cos(eurla_rotate[1] / 3.1415926 * 180);
					error.at<double>(2, 0) = tvecs[i][1];
					if (rvecs[i][0] < 0.0)
					{
						error.at<double>(3, 0) = -rvecs[i][2];
					}
					else
					{
						error.at<double>(3, 0) = rvecs[i][2];
					}

					PID.getCommand(error, output);

					if (abs(error.at<double>(0, 0)) > 7)
					{
						vx = 0.01 * output.at<double>(0, 0);
					}

					if (sqrt(pow(tvecs[i][2], 2) + pow(tvecs[i][2], 2)) < 110) //(abs(error.at<double>(0, 0)) < 20 && abs(error.at<double>(1, 0)) < 20)
					{
						//stady_count[1]++;
						step_id_5 = false;
						step_id_6 = true;
						vx = 0;
						vy = 0;
						vz = 0;
						vr = -2;
						std::cout << "ok fuck bye 21 (dis)\n";
					}

					// if (tvecs[i][2] > 130 && abs(error.at<double>(3, 0)) > 0.1)
					// {
					// 	vr = 1.2 * output.at<double>(3, 0);
					// }

					if (abs(error.at<double>(1, 0)) > 2)
					{
						vy = -0.01 * output.at<double>(1, 0);
					}

					// //if stady enought, then go to next step
					// if (stady_count[3] == stady_const[3])
					// {
					// 	step_id_5 = false;
					// 	landing = true;
					// 	vx = 0;
					// 	vy = 0;
					// 	vz = 0;
					// 	vr = 0;
					// 	ardrone.setCamera(++mode % 4);
					// 	// //down_camera_calibration
					// 	// fs_2["intrinsic"] >> cameraMatrix;
					// 	// fs_2["distortion"] >> distCoeffs;
					// 	// // calculate undistort map
					// 	// cv::Mat frame = ardrone.getImage();
					// 	// cv::initUndistortRectifyMap(cameraMatrix, distCoeffs, cv::Mat(), cameraMatrix, frame.size(), CV_32FC1, mapx, mapy);
					// 	// std::cout << "ok fuck bye 4 (dis)\n";
					// }

					//vr = 0;
					//vr = -0.02 * output.at<double>(3, 0);
					//char str[100];
					//eurla_rotate[0], eurla_rotate[1] / 3.1415926 * 180
					//sprintf(str, "%5g %5g %5g", vy, error.at<double>(0, 0), error.at<double>(1, 0)); //,// vz);
					//cv::putText(image, str, cv::Point(20, 40), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0, 255, 0));
					std::cout << ids[i] << " " << sqrt(pow(tvecs[i][2], 2) + pow(tvecs[i][2], 2)) << endl;
				}

				//flying to id_22
				else if (step_id_6 == true && ids[i] == 22)
				{
					counter = 0;
					std::cout << "in 22\n";
					//aimming to marker 21's right 30cm
					error.at<double>(0, 0) = tvecs[i][2] - 100 * sin(eurla_rotate[1] / 3.1415926 * 180);
					error.at<double>(1, 0) = tvecs[i][0] + 100 * cos(eurla_rotate[1] / 3.1415926 * 180);
					error.at<double>(2, 0) = tvecs[i][1];
					if (rvecs[i][0] < 0.0)
					{
						error.at<double>(3, 0) = -rvecs[i][2];
					}
					else
					{
						error.at<double>(3, 0) = rvecs[i][2];
					}

					PID.getCommand(error, output);

					if (abs(error.at<double>(0, 0)) > 7)
					{
						vx = 0.01 * output.at<double>(0, 0);
					}

					if (sqrt(pow(tvecs[i][2], 2) + pow(tvecs[i][2], 2)) < 130) //(abs(error.at<double>(0, 0)) < 20 && abs(error.at<double>(1, 0)) < 20)
					{
						//stady_count[1]++;
						step_id_6 = false;
						landing = true;
						face_detection = false;
						vx = 0;
						vy = 0;
						vz = 0;
						vr = 0;
						std::cout << "ok fuck bye 22 (dis)\n";
					}

					// if (tvecs[i][2] > 130 && abs(error.at<double>(3, 0)) > 0.1)
					// {
					// 	vr = 1.2 * output.at<double>(3, 0);
					// }

					if (abs(error.at<double>(1, 0)) > 2)
					{
						vy = -0.01 * output.at<double>(1, 0);
					}

					// //if stady enought, then go to next step
					// if (stady_count[3] == stady_const[3])
					// {
					// 	step_id_5 = false;
					// 	landing = true;
					// 	vx = 0;
					// 	vy = 0;
					// 	vz = 0;
					// 	vr = 0;
					// 	ardrone.setCamera(++mode % 4);
					// 	// //down_camera_calibration
					// 	// fs_2["intrinsic"] >> cameraMatrix;
					// 	// fs_2["distortion"] >> distCoeffs;
					// 	// // calculate undistort map
					// 	// cv::Mat frame = ardrone.getImage();
					// 	// cv::initUndistortRectifyMap(cameraMatrix, distCoeffs, cv::Mat(), cameraMatrix, frame.size(), CV_32FC1, mapx, mapy);
					// 	// std::cout << "ok fuck bye 4 (dis)\n";
					// }
					//vr = 0;
					//vr = -0.02 * output.at<double>(3, 0);
					//char str[100];
					//eurla_rotate[0], eurla_rotate[1] / 3.1415926 * 180
					//sprintf(str, "%5g %5g %5g", vy, error.at<double>(0, 0), error.at<double>(1, 0)); //,// vz);
					//cv::putText(image, str, cv::Point(20, 40), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0, 255, 0));
					std::cout << ids[i] << " " << sqrt(pow(tvecs[i][2], 2) + pow(tvecs[i][2], 2)) << endl;
				}

				// landing
				else if (landing == true && ids[i] == 4)
				{
					counter = 0;
					std::cout << "in 4\n";
					error.at<double>(0, 0) = tvecs[i][2] - 90 * cos(eurla_rotate[1] / 3.1415926 * 180);
					error.at<double>(1, 0) = tvecs[i][0] - 90 * sin(eurla_rotate[1] / 3.1415926 * 180);
					error.at<double>(2, 0) = tvecs[i][1];
					if (rvecs[i][0] < 0.0)
					{
						error.at<double>(3, 0) = -rvecs[i][2];
					}
					else
					{
						error.at<double>(3, 0) = rvecs[i][2];
					}

					PID.getCommand(error, output);

					if (abs(error.at<double>(3, 0)) > 0.08 && sqrt(pow(tvecs[i][2], 2) + pow(tvecs[i][2], 2)) < 105)
					{
						vr = 1 * output.at<double>(3, 0);
					}
					else
					{
						vr = 0;
					}
					// if (abs(error.at<double>(1, 0)) > 2 && sqrt(pow(abs(error.at<double>(0, 0)), 2) + pow(abs(error.at<double>(1, 0)), 2)) < 200) //tvecs[i][2] < 100 &&
					// {
					// 	vy = -0.01 * output.at<double>(1, 0);
					// }
					if (abs(error.at<double>(0, 0)) > 7)
					{
						vx = 0.01 * output.at<double>(0, 0);
					}
					else
					{
						stady_count[4]++;
						vx = 0;
					}

					if(sqrt(pow(tvecs[i][2], 2) + pow(tvecs[i][2], 2)) < 95){
						landing = false;
						ardrone.landing();
						std::cout << "landing\n";
					}

					// //if stady enought, then go to next step
					// if (stady_count[4] == stady_const[4])
					// {
					// 	landing = false;
					// 	ardrone.landing();
					// 	std::cout << "landing\n";
					// }
					// error.at<double>(0, 0) = tvecs[i][1] + 4; // 2
					// error.at<double>(1, 0) = tvecs[i][0];
					// error.at<double>(2, 0) = tvecs[i][2]; // 1
					// // if (rvecs[0][0] < 0.0)
					// // {
					// // 	error.at<double>(3, 0) = -rvecs[i][2];
					// // }

					// PID.getCommand(error, output);

					// //if drone on the right position, then doing landing
					// if (abs(tvecs[i][2] - 73) < 2)
					// {
					// 	ardrone.landing();
					// 	landing = false;
					// }

					// if (abs(error.at<double>(0, 0)) > 2)
					// {



						
					// 	vx = -0.01 * output.at<double>(0, 0);
					// 	vz = -0.008; // -0.07
					// }
					// else
					// {
					// 	vx = 0;
					// 	vz = -0.1; // -0.08
					// }

					// vy = 0;
					// vr = 0;
					//char str[100];
					//eurla_rotate[0], eurla_rotate[1] / 3.1415926 * 180
					//sprintf(str, "%5g %5g %5g", error.at<double>(3, 0), stady_count[4], error.at<double>(0, 0)); //,// vz);
					//cv::putText(image, str, cv::Point(20, 40), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0, 255, 0));
					std::cout << ids[i] << " " << sqrt(pow(tvecs[i][2], 2) + pow(tvecs[i][2], 2)) << endl;
				}

				
				//vr = -0.01 * output.at<double>(3, 0);
			}
		}
		// if(vx == 0 && vy == 0 && vr == 0)  {
		// 	vx = last_vx/2;
		// 	vy = last_vy/2;
		// 	vr = last_vr/2;
		// }
		// if(abs(vx - last_vx) > 1.5) {
		// 	vx = last_vx;
		// }
		// if (abs(vy - last_vy) > 1.5)
		// {
		// 	vy = last_vx;
		// }
		// if (abs(vr - last_vr) > 1.5)
		// {
		// 	vr = last_vr;
		// }
		// last_vx = vx * 0.5 + last_vx * 0.5;
		// last_vy = vy * 0.4 + last_vy * 0.6;
		// last_vr = vr * 0.4 + last_vr * 0.6;
		ardrone.move3D(vx, vy, vz, vr);
		ardrone.move3D(vx, vy, vz, vr);
		ardrone.move3D(vx, vy, vz, vr);

		// Display the image
		cv::imshow("camera", image);
	}

	// See you
	ardrone.close();

	return 0;
}
