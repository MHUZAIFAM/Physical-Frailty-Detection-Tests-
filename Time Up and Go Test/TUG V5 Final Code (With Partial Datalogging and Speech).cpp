//set the data logging function and break/continue

#include <Kinect.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <chrono>
#include <deque>
#include <numeric>
#include <iomanip>
#include <cmath>
#include<deque>
#include <fstream>
#include <string>
#include <sstream>
#include <sapi.h> // Speech API
#include <iostream>
#include <string>
#include<algorithm>
using namespace std;

// Constants
const float CHAIR_DEPTH = 4.0f;        // Depth when sitting on the chair
const float TARGET_DEPTH = 1.0f;      // Target depth during walking
const float Y_CHANGE_THRESHOLD = 000.5f; // Threshold for Y-coordinate change
const float DEPTH_TOLERANCE = 0.5f;   // Allowable error in depth comparison
const float Y_COORD_TOLERANCE = 0.05f; // Allowable error in Y-coordinate comparison

// Timer variables
bool isTiming = false;
bool reachedTargetDepth = false;
std::chrono::steady_clock::time_point startTime;
std::chrono::steady_clock::time_point endTime;

// Initial Y-coordinate for validation
float initialYCoordinate = -1.0f;

//coordinate for stability detection
const int stabilityFramesThreshold = 20; // Number of frames to check for stability
const float stabilityYThreshold = 0.02f; // Y-coordinate fluctuation threshold for stability

void speak(const std::string& text) {
    ISpVoice* pVoice = NULL;

    if (FAILED(::CoInitialize(NULL))) {
        std::cerr << "Failed to initialize COM library." << std::endl;
        return;
    }

    HRESULT hr = CoCreateInstance(CLSID_SpVoice, NULL, CLSCTX_ALL, IID_ISpVoice, (void**)&pVoice);
    if (SUCCEEDED(hr)) {
        wchar_t wtext[1024];
        size_t convertedChars = 0;
        mbstowcs_s(&convertedChars, wtext, sizeof(wtext) / sizeof(wchar_t), text.c_str(), _TRUNCATE); // Corrected version
        pVoice->Speak(wtext, 0, NULL);
        pVoice->Release();
        pVoice = NULL;
    }
    else {
        std::cerr << "Failed to create voice instance." << std::endl;
    }
    CoUninitialize();
}

//stable function
bool isStable(const std::deque<float>& history, float threshold) {
    if (history.size() < stabilityFramesThreshold) return false;
    float minVal = *std::min_element(history.begin(), history.end());
    float maxVal = *std::max_element(history.begin(), history.end());
    return (maxVal - minVal) <= threshold;
}

//data logging function
// Data logging function
void logTUGTestTime(const std::vector<double>& testTimes) {
    std::string filename = "TUG_Test_Results.csv";
    std::ifstream infile(filename);
    std::ofstream outfile;

    // Check if the file exists
    bool fileExists = infile.good();
    infile.close();

    // Open the file in append mode
    outfile.open(filename, std::ios::app);

    // If the file is new, write the header once
    if (!fileExists) {
        outfile << "Timed Up and Go Test (s) 1";
        for (size_t i = 1; i < testTimes.size(); ++i) {
            outfile << ",Timed Up and Go Test (s) " << (i + 1);
        }
        outfile << "\n";
    }

    // Append test times as a new row
    for (size_t i = 0; i < testTimes.size(); ++i) {
        if (i > 0) outfile << ",";
        outfile << testTimes[i];
    }
    outfile << "\n";

    outfile.close();
}



//flags for person detected, person stable, test started, timer started, target depth reached, test completed, timer stopped
bool isPersonDetected = false;
bool isPersonStable = false;
bool isTestStarted = false;
bool isTimerStarted = false;
bool isTargetDepthReached = false;
bool isTestCompleted = false;
bool isTimerStopped = false;

//initial mid spine x,y,z coordinates
float initialMidSpineX = 0.0f;
float initialMidSpineY = 0.0f;
float initialMidSpineZ = 0.0f;

//right leg, left leg threshold 
const float rightLegThresholdY = 0.1f;
const float leftLegThresholdY = 0.1f;
const float rightLegThresholdX = 0.1f;
const float leftLegThresholdX = 0.1f;
float rightLegThreshold = 0.2f;
float leftLegThreshold = 0.2f;
//standing hips in line with knee threshold
float standingHipsThreshold = 0.1f;




// Timer functions
void startTimer(float depth, float yCoordinate) {
    isTiming = true;
    reachedTargetDepth = false; // Reset target depth tracking
    startTime = std::chrono::steady_clock::now();
    cout << "Timer started! Depth: " << depth << "m, Y-coordinate: " << yCoordinate << endl;
    //display live timer on the screen using put text
    
	
}

void stopTimer(float depth, float yCoordinate) {
    endTime = std::chrono::steady_clock::now();
    isTiming = false;

    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime).count();
    float elapsedSeconds = elapsedTime / 1000.0f;

    cout << "Timer stopped! Depth: " << depth << "m, Y-coordinate: " << yCoordinate << endl;
    cout << "Total time taken: " << fixed << setprecision(2) << elapsedSeconds << " seconds" << endl;

    // Reset for the next test
    initialYCoordinate = -1.0f;
}


// Main program
int main() {
    IKinectSensor* sensor = nullptr;
    IColorFrameReader* colorFrameReader = nullptr;
    IBodyFrameReader* bodyFrameReader = nullptr;
    ICoordinateMapper* coordinateMapper = nullptr;

    if (FAILED(GetDefaultKinectSensor(&sensor)) || !sensor) {
        cerr << "Kinect sensor not found!" << endl;
        return -1;
    }

    sensor->Open();
    sensor->get_CoordinateMapper(&coordinateMapper);

    IColorFrameSource* colorSource = nullptr;
    sensor->get_ColorFrameSource(&colorSource);
    colorSource->OpenReader(&colorFrameReader);

    IBodyFrameSource* bodySource = nullptr;
    sensor->get_BodyFrameSource(&bodySource);
    bodySource->OpenReader(&bodyFrameReader);

    cv::namedWindow("Kinect Walking Test", cv::WINDOW_AUTOSIZE);
	double elapsedSeconds = 0.0;

    while (true) {
        IColorFrame* colorFrame = nullptr;
        HRESULT hrColor = colorFrameReader->AcquireLatestFrame(&colorFrame);

        if (SUCCEEDED(hrColor)) {
            IFrameDescription* frameDescription = nullptr;
            colorFrame->get_FrameDescription(&frameDescription);

            int width, height;
            frameDescription->get_Width(&width);
            frameDescription->get_Height(&height);

            UINT bufferSize = width * height * 4;
            BYTE* colorBuffer = new BYTE[bufferSize];
            hrColor = colorFrame->CopyConvertedFrameDataToArray(bufferSize, colorBuffer, ColorImageFormat_Bgra);

            if (SUCCEEDED(hrColor)) {
                cv::Mat colorMat(height, width, CV_8UC4, colorBuffer);
                cv::Mat bgrMat;
                cv::cvtColor(colorMat, bgrMat, cv::COLOR_BGRA2BGR);

                IBodyFrame* bodyFrame = nullptr;
                HRESULT hrBody = bodyFrameReader->AcquireLatestFrame(&bodyFrame);

                if (SUCCEEDED(hrBody)) {
                    IBody* bodies[BODY_COUNT] = { 0 };
                    hrBody = bodyFrame->GetAndRefreshBodyData(_countof(bodies), bodies);

                    for (int i = 0; i < BODY_COUNT; ++i) {
                        IBody* body = bodies[i];
                        if (body) {
                            BOOLEAN isTracked = false;
                            body->get_IsTracked(&isTracked);
                             if (isTracked) {
                                 Joint joints[JointType_Count];
                                 body->GetJoints(_countof(joints), joints);
                                 //display the depth of mid spine on live feed
								 cv::putText(bgrMat, "Depth: " + to_string(joints[JointType_SpineMid].Position.Z), cv::Point(50, 50), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 255), 2);
                                 
                                //print person detected sitting on the chair
                                 //if mid spine Z is approximately at 4m depth, and hip and knee joint roughly align within threshold defined
								 if (joints[JointType_SpineMid].Position.Z <= 4.4f && joints[JointType_SpineMid].Position.Z >= 3.9f) {

									 if (joints[JointType_HipRight].Position.Y - joints[JointType_KneeRight].Position.Y < rightLegThresholdY &&
										 joints[JointType_HipLeft].Position.Y - joints[JointType_KneeLeft].Position.Y < leftLegThresholdY && 
                                         joints[JointType_HipRight].Position.X - joints[JointType_KneeRight].Position.X < rightLegThresholdX &&
                                         joints[JointType_HipLeft].Position.X - joints[JointType_KneeLeft].Position.X < leftLegThresholdX
                                         && !isPersonDetected) {
										 isPersonDetected = true;
										 cout << "Person detected sitting on the chair at depth of 4 meters" << endl;
                                         speak("Test ready please stand up");
										 cout << "Please Be Ready to Stand Up" << endl;

										 //speech API to speak the message
										 //speak("Person detected sitting on the chair. Test Ready");
                                         //speak("Please Stand up");
										 
										 //note initial mid spine x,y,z coordinates
										 initialMidSpineX = joints[JointType_SpineMid].Position.X;
										 initialMidSpineY = joints[JointType_SpineMid].Position.Y;
										 initialMidSpineZ = joints[JointType_SpineMid].Position.Z;
                                         // print y coordinates of mid spine
										 cout << "Initial Mid Spine Y Coordinate: " << initialMidSpineY << endl;
                                         
									 }
                                     //put text person deteccted sitting on chair Test Ready
                                     cv::putText(bgrMat, "Person detected sitting on chair. Test Ready", cv::Point(50, 100), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 255), 2);
                                     
								 }
                                 

                                 //if person stands up, then start the timer
                                 if (isPersonDetected && joints[JointType_SpineMid].Position.Y - initialMidSpineY > 0.05f && 
                                     joints[JointType_KneeLeft].Position.X - joints[JointType_HipLeft].Position.X < standingHipsThreshold &&
                                     joints[JointType_KneeRight].Position.X - joints[JointType_HipRight].Position.X < standingHipsThreshold
                                     && !isTestStarted) {
									 isTestStarted = true;
                                     //start timer by calling the function
									 startTimer(joints[JointType_SpineMid].Position.Z, joints[JointType_SpineMid].Position.Y);
                                     //start dynamic timer on screen
									 cout << "Timer Started" << endl;
                                     //speak("Timer Started");
                                     
                                 }
                                 //display dynamic timer on live feed

                                 if (isTestStarted)
                                 {
                                     cv::putText(bgrMat, "Timer Started", cv::Point(50, 150), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 255), 2);
                                     
                                 }

								 //if person reaches target depth of 1m, then print the message
                                 if (isTestStarted && joints[JointType_SpineMid].Position.Z < 1.1f && joints[JointType_SpineMid].Position.Z > 0.9f && !isTargetDepthReached) {
                                     isTargetDepthReached = true;
                                     cout << "Target depth reached: " << joints[JointType_SpineMid].Position.Z << "m" << endl;
                                    // speak("Target depth reached, Please Turn around");
                                     }
                                 if (isTargetDepthReached)
                                 {
                                     //display message on live feed
                                     cv::putText(bgrMat, "Target depth reached, Please Turn around", cv::Point(50, 200), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 255), 2);

                                 }
                                 // stop timer if hips align with knee again, and mid spine depth is 4m
                                 if (isTestStarted && joints[JointType_HipRight].Position.Y - joints[JointType_KneeRight].Position.Y < rightLegThreshold && 
                                     joints[JointType_HipLeft].Position.Y - joints[JointType_KneeLeft].Position.Y < leftLegThreshold && 
                                     joints[JointType_SpineMid].Position.Z < 4.2f && joints[JointType_SpineMid].Position.Z > 3.8f && 
                                     !isTestCompleted && isTargetDepthReached) 
                                 {
									 isTestCompleted = true;
									 stopTimer(joints[JointType_SpineMid].Position.Z, joints[JointType_SpineMid].Position.Y);
                                     //store the timer value in a variable
									 elapsedSeconds = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime).count() / 1000.0f;
									 //call the function to log the time
                                     logTUGTestTime(std::vector<double>{elapsedSeconds});
									 //log the time in a file
									 //display test complete on live feed   
                                    
                                 }
								 // now to reset the test, all the flags
                                 if (isTestCompleted) {
                                     // Display test completion messages
                                     cv::putText(bgrMat, "Timer Stopped", cv::Point(50, 250), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 255), 2);
                                     cv::putText(bgrMat, "Total time taken: " + to_string(elapsedSeconds) + " seconds", cv::Point(50, 350), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 255), 2);
                                     cv::putText(bgrMat, "Test Completed", cv::Point(50, 300), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 255), 2);
                                     speak("Test Completed");
                                     // Ask user if they want to repeat the test
                                     std::cout << "Do you want to take the test again? (y/n): ";
                                     char userInput;
                                     std::cin >> userInput;

                                     if (userInput == 'y' || userInput == 'Y') {
                                         // Reset the flags if the user wants to take the test again
                                         isPersonDetected = false;
                                         isTestStarted = false;
                                         isTimerStarted = false;
                                         isTargetDepthReached = false;
                                         isTestCompleted = false;
                                         isTimerStopped = false;
                                     }
									 else if (userInput == 'n' || userInput == 'N') {
										 // Exit the program if the user does not want to take the test again
										 continue;
									 }
                                 }

                                                             
                                 break;
                                
                             }
                        }
                    }

                    bodyFrame->Release();
                }

                cv::imshow("Kinect Walking Test", bgrMat);
            }

            delete[] colorBuffer;
        }

        if (colorFrame) {
            colorFrame->Release();

        }

        if (cv::waitKey(30) == 27) {
            break;
        }
    }

    sensor->Close();
    cv::destroyAllWindows();
    return 0;
}
