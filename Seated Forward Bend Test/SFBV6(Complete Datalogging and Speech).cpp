#include <Kinect.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <deque> // For stability history tracking
#include <fstream>
#include <vector>
#include <sstream>
#include <string>
#include <math.h>
#include<sapi.h>
#include <algorithm>
#include <iomanip>  // For setprecision

void logSeatedForwardBendTest(const std::vector<float>& rightHandDistances, const std::vector<float>& leftHandDistances) {
    std::string filename = "Seated_Forward_Bend_Test_Results.csv";
    std::ifstream infile(filename);
    std::ofstream outfile;

    // Check if the file exists
    bool fileExists = infile.good();
    infile.close();

    // Open file in append mode
    outfile.open(filename, std::ios::app);

    // Write header if the file is new
    if (!fileExists) {
        outfile << "Seated Forward Bend Test (cm) Right Hand,Seated Forward Bend Test (cm) Left Hand,Seated Forward Bend Test (cm)\n";
    }

    // Compute the maximum distances
    float maxRightHand = *std::max_element(rightHandDistances.begin(), rightHandDistances.end());
    float maxLeftHand = *std::max_element(leftHandDistances.begin(), leftHandDistances.end()) ;
    float maxOverall = std::max(maxRightHand, maxLeftHand);

    // Write data to file with 2 decimal precision
    outfile << std::fixed << std::setprecision(2)
        << maxRightHand << ","
        << maxLeftHand << ","
        << maxOverall << "\n";

    outfile.close();
}


void speak(const std::string& text) {
    std::thread([text]() {
        ISpVoice* pVoice = NULL;

        if (FAILED(::CoInitialize(NULL))) {
            std::cerr << "Failed to initialize COM library." << std::endl;
            return;
        }

        HRESULT hr = CoCreateInstance(CLSID_SpVoice, NULL, CLSCTX_ALL, IID_ISpVoice, (void**)&pVoice);
        if (SUCCEEDED(hr)) {
            wchar_t wtext[1024];
            size_t convertedChars = 0;
            mbstowcs_s(&convertedChars, wtext, sizeof(wtext) / sizeof(wchar_t), text.c_str(), _TRUNCATE);

            // Use SPF_ASYNC for non-blocking speech
            pVoice->Speak(wtext, SPF_ASYNC, NULL);

            // Wait for speech to complete (optional, remove if unnecessary)
            pVoice->WaitUntilDone(INFINITE);

            pVoice->Release();
            pVoice = NULL;
        }
        else {
            std::cerr << "Failed to create voice instance." << std::endl;
        }
        CoUninitialize();
        }).detach(); // Detach the thread so it runs independently
}

using namespace std;

//variables for Test Ready(non raised joints coordinates)
float nonRaisedLeftHandX = 0.0f, nonRaisedLeftHandY = 0.0f, nonRaisedLeftHandZ = 0.0f;
float nonRaisedRightHandX = 0.0f, nonRaisedRightHandY = 0.0f, nonRaisedRightHandZ = 0.0f;
float nonRaisedElbowLeftX = 0.0f, nonRaisedElbowLeftY = 0.0f, nonRaisedElbowLeftZ = 0.0f;
float nonRaisedElbowRightX = 0.0f, nonRaisedElbowRightY = 0.0f, nonRaisedElbowRightZ = 0.0f;
float nonRaisedMidSpineX = 0.0f, nonRaisedMidSpineY = 0.0f, nonRaisedMidSpineZ = 0.0f;
float nonRaisedShoulderSpineX = 0.0f, nonRaisedShoulderSpineY = 0.0f, nonRaisedShoulderSpineZ = 0.0f;

//variables for Raised Arms Joint Coordinates
float raisedLeftHandX = 0.0f, raisedLeftHandY = 0.0f, raisedLeftHandZ = 0.0f;
float raisedRightHandX = 0.0f, raisedRightHandY = 0.0f, raisedRightHandZ = 0.0f;
float raisedElbowLeftX = 0.0f, raisedElbowLeftY = 0.0f, raisedElbowLeftZ = 0.0f;
float raisedElbowRightX = 0.0f, raisedElbowRightY = 0.0f, raisedElbowRightZ = 0.0f;
float raisedMidSpineX = 0.0f, raisedMidSpineY = 0.0f, raisedMidSpineZ = 0.0f;
float raisedShoulderSpineX = 0.0f, raisedShoulderSpineY = 0.0f, raisedShoulderSpineZ = 0.0f;

//arms rasied threshold
float armsRaisedThresholdY = 0.1f;
float armsRaisedThresholdX = 0.2f;
float armmovedthresholdX = 0.2f;
float armmovedthresholdY = 0.2f;
float armmovedthresholdZ = 0.05f;
float initialPostureThreshold = 0.3f;
float armsinlinewithelbowthreshold = 0.1f;

//initialpositionreatin threshold
float initialPositionHandsRetainedZ = 0.05f;
float initialPositionHandsRetainedY = 0.1f;
float initialPositionHandsRetainedX = 0.1f;



//flags for the test
bool testStarted = false;
bool testReady = false;
bool armsRaised = false;
bool messagePrinted = false;
bool isPersonStable = false;
bool isPersonStraight = false;
bool FinalMaximumDistance = false;
bool testComplete = false;
bool onetimereading = false;
bool initialPostureretain = false;

//variable distance
float RightHandDistance = 0.0f;
float LeftHandDistance = 0.0f;
float MaximumRightHandDistance = 0.0f;
float MaximumLeftHandDistance = 0.0f;
float Distance = 0.0f;


// Constants for stability detection
const int stabilityFramesThreshold = 20; // Number of frames to check for stability
const float stabilityYThreshold = 0.02f; // Y-coordinate fluctuation threshold for stability

// Initial Z-coordinate values for left hand, mid spine, and shoulder spine (assuming -1 is invalid/uninitialized)
float initialLeftHandZ = -1.0f,initialRightHandZ = -1.0f,
      initialLeftElbowZ = -1.0f , initialRightElbowZ = -1.0f ,
      initialMidSpineZ = -1.0f, initialShoulderSpineZ = -1.0f;

// Deques to store Y-coordinate history for stability detection
std::deque<float> leftHandYHistory,rightHandYHistory, 
                  leftElbowYHistory, rightElbowYHistory,
                  midSpineYHistory, shoulderSpineYHistory;

// Variables to track Y-coordinates of joints in previous frames
float lastLeftHandY = -1.0f, lastRightHandY = -1.0f,
      lastLeftElbowY = -1.0f, lastRightElbowY = -1.0f,
      lastMidSpineY = -1.0f, lastShoulderSpineY = -1.0f;

int stabilityFrames = 0; // To track how many frames the joints are stable

// Function to check stability
bool isStable(const std::deque<float>& history, float threshold) {
    if (history.size() < stabilityFramesThreshold) return false;
    float minVal = *std::min_element(history.begin(), history.end());
    float maxVal = *std::max_element(history.begin(), history.end());
    return (maxVal - minVal) <= threshold;
}


int main() {
    // Initialize Kinect Sensor, readers, and coordinate mapper
    IKinectSensor* sensor = nullptr;
    IColorFrameReader* colorFrameReader = nullptr;
    IBodyFrameReader* bodyFrameReader = nullptr;
    ICoordinateMapper* coordinateMapper = nullptr;

    if (FAILED(GetDefaultKinectSensor(&sensor)) || !sensor) {
        std::cerr << "Kinect sensor not found!" << std::endl;
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

    cv::namedWindow("Seated Forward Bent Test", cv::WINDOW_AUTOSIZE);

    // Frame loop
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
                                

                                float leftHandY = 0, rightHandY = 0, 
                                      leftElbowY = 0, rightElbowY = 0,
                                      shoulderSpineY = 0, midSpineY = 0;

                                std::vector<cv::Point> jointPoints;  // Store valid joint positions

                                for (int j = 0; j < JointType_Count; ++j) {
                                    if (joints[j].TrackingState == TrackingState_Tracked) {
                                        ColorSpacePoint colorPoint;
                                        coordinateMapper->MapCameraPointToColorSpace(joints[j].Position, &colorPoint);

                                        int x = static_cast<int>(colorPoint.X);
                                        int y = static_cast<int>(colorPoint.Y);

                                        // Debugging output
                                        //std::cout << "Joint " << j << " -> X: " << x << ", Y: " << y << std::endl;

                                        if (x > 0 && x < width && y > 0 && y < height) {
                                            jointPoints.push_back(cv::Point(x, y));
                                        }
                                    }
                                }

                                // If we have valid joint points, draw bounding box
                                if (!jointPoints.empty()) {
                                    cv::Rect boundingRect = cv::boundingRect(jointPoints);
                                    cv::rectangle(bgrMat, boundingRect, cv::Scalar(0, 255, 0), 2);

                                }

                                //// Only draw circles for selected joints
                                //for (int j = 0; j < JointType_Count; j++) {
                                //    if (joints[j].TrackingState == TrackingState_Tracked &&
                                //        (j == JointType_HandLeft || j == JointType_HandRight ||
                                //            j == JointType_ElbowLeft || j == JointType_ElbowRight ||
                                //            j == JointType_SpineMid || j == JointType_SpineShoulder)) {

                                //        // Use the raw camera space coordinates (meters)
                                //        float x = joints[j].Position.X;
                                //        float y = joints[j].Position.Y;
                                //        float z = joints[j].Position.Z;

                                //        // Save specific Y values for the joints
                                //        if (j == JointType_HandLeft) {
                                //            leftHandY = y;
                                //            if (initialLeftHandZ == -1.0f) {
                                //                initialLeftHandZ = z;
                                //            }
                                //        }
                                //        else if (j == JointType_HandRight) {
                                //            rightHandY = y;
                                //            if (initialRightHandZ == -1.0f) {
                                //                initialRightHandZ = z;
                                //            }
                                //        }
                                //        else if (j == JointType_ElbowLeft) {
                                //            leftElbowY = y;
                                //            if (initialLeftElbowZ == -1.0f) {
                                //                initialLeftElbowZ = z;
                                //            }
                                //        }
                                //        else if (j == JointType_ElbowRight) {
                                //            rightElbowY = y;
                                //            if (initialRightElbowZ == -1.0f) {
                                //                initialRightElbowZ = z;
                                //            }
                                //        }
                                //        else if (j == JointType_SpineMid) {
                                //            midSpineY = y;
                                //            if (initialMidSpineZ == -1.0f) {
                                //                initialMidSpineZ = z;
                                //            }
                                //        }
                                //        else if (j == JointType_SpineShoulder) {
                                //            shoulderSpineY = y;
                                //            if (initialShoulderSpineZ == -1.0f) {
                                //                initialShoulderSpineZ = z;
                                //            }
                                //        }

                                //        // Convert camera space to color space for visualization
                                //        ColorSpacePoint colorPoint;
                                //        coordinateMapper->MapCameraPointToColorSpace(joints[j].Position, &colorPoint);
                                //        int cx = static_cast<int>(colorPoint.X); // Pixel coordinates for the live feed
                                //        int cy = static_cast<int>(colorPoint.Y);

                                //        // Ensure the pixel coordinates are within bounds before drawing
                                //        if (cx >= 0 && cx < width && cy >= 0 && cy < height) {
                                //            cv::circle(bgrMat, cv::Point(cx, cy), 10, cv::Scalar(0, 0, 255), -1); // Draw a circle

                                //           }
                                //    }
                                //}


                                //Update history
                                if (leftHandYHistory.size() >= stabilityFramesThreshold) leftHandYHistory.pop_front();
                                if (rightHandYHistory.size() >= stabilityFramesThreshold) rightHandYHistory.pop_front();
                                if (leftElbowYHistory.size() >= stabilityFramesThreshold) leftElbowYHistory.pop_front();
                                if (rightElbowYHistory.size() >= stabilityFramesThreshold) rightElbowYHistory.pop_front();
                                if (midSpineYHistory.size() >= stabilityFramesThreshold) midSpineYHistory.pop_front();
                                if (shoulderSpineYHistory.size() >= stabilityFramesThreshold) shoulderSpineYHistory.pop_front();

                                leftHandYHistory.push_back(leftHandY);
                                rightHandYHistory.push_back(rightHandY);
                                leftElbowYHistory.push_back(leftElbowY);
                                rightElbowYHistory.push_back(rightElbowY);
								midSpineYHistory.push_back(midSpineY);
								shoulderSpineYHistory.push_back(shoulderSpineY);

								// Check stability
								bool leftHandStable = isStable(leftHandYHistory, stabilityYThreshold);
								bool rightHandStable = isStable(rightHandYHistory, stabilityYThreshold);
								bool leftElbowStable = isStable(leftElbowYHistory, stabilityYThreshold);
								bool rightElbowStable = isStable(rightElbowYHistory, stabilityYThreshold);
								bool midSpineStable = isStable(midSpineYHistory, stabilityYThreshold);
								bool shoulderSpineStable = isStable(shoulderSpineYHistory, stabilityYThreshold);
								
                                
                                // Conditional statement: When arms are stable, print message
								if (leftElbowStable && rightElbowStable && midSpineStable && shoulderSpineStable &&
									joints[JointType_HandRight].Position.Y - joints[JointType_ElbowRight].Position.Y <= armsinlinewithelbowthreshold &&
									joints[JointType_HandLeft].Position.Y - joints[JointType_ElbowLeft].Position.Y <= armsinlinewithelbowthreshold &&
                                    !messagePrinted && !testReady && !testStarted && !isPersonStable) {

									messagePrinted = true; // Set the flag to true to prevent repeated printing
									testReady = true;
									isPersonStable = true;

									nonRaisedElbowLeftX = joints[JointType_ElbowLeft].Position.X;
									nonRaisedElbowLeftY = joints[JointType_ElbowLeft].Position.Y;
									nonRaisedElbowLeftZ = joints[JointType_ElbowLeft].Position.Z;

									nonRaisedElbowRightX = joints[JointType_ElbowRight].Position.X;
									nonRaisedElbowRightY = joints[JointType_ElbowRight].Position.Y;
									nonRaisedElbowRightZ = joints[JointType_ElbowRight].Position.Z;

									nonRaisedLeftHandX = joints[JointType_HandLeft].Position.X;
									nonRaisedLeftHandY = joints[JointType_HandLeft].Position.Y;
									nonRaisedLeftHandZ = joints[JointType_HandLeft].Position.Z;

									nonRaisedRightHandX = joints[JointType_HandRight].Position.X;
									nonRaisedRightHandY = joints[JointType_HandRight].Position.Y;
									nonRaisedRightHandZ = joints[JointType_HandRight].Position.Z;

									nonRaisedMidSpineX = joints[JointType_SpineMid].Position.X;
									nonRaisedMidSpineY = joints[JointType_SpineMid].Position.Y;
									nonRaisedMidSpineZ = joints[JointType_SpineMid].Position.Z;

									nonRaisedShoulderSpineX = joints[JointType_SpineShoulder].Position.X;
									nonRaisedShoulderSpineY = joints[JointType_SpineShoulder].Position.Y;
									nonRaisedShoulderSpineZ = joints[JointType_SpineShoulder].Position.Z;


									//std::cout << "Test Ready." << std::endl;
									//std::cout << "Please Raise your Arms." << std::endl;
									// Display initial Y coordinates for arms
									
                                    /*std::cout << "Initial Left Hand Y: " << leftHandY << std::endl;
									std::cout << "Initial Right Hand Y: " << rightHandY << std::endl;
									std::cout << "Initial Left Elbow Y: " << leftElbowY << std::endl;
									std::cout << "Initial Right Elbow Y: " << rightElbowY << std::endl;
									std::cout << "Initial Mid Spine Y: " << midSpineY << std::endl;
									std::cout << "Initial Shoulder Spine Y: " << shoulderSpineY << std::endl;*/

									speak("Please raise your arms");
								}

								if (messagePrinted && isPersonStable && testReady && !testStarted)
								{
									cv::putText(bgrMat, "Test Ready", cv::Point(50, 50), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0, 0, 0), 2);
								}

                                //now to raise the arms
                                if (joints[JointType_HandLeft].Position.Y - nonRaisedLeftHandY >= armsRaisedThresholdY && 
                                    joints[JointType_HandRight].Position.Y - nonRaisedRightHandY >= armsRaisedThresholdY &&
                                    joints[JointType_HandLeft].Position.X - nonRaisedLeftHandX <= armsRaisedThresholdX &&
                                    joints[JointType_HandRight].Position.X - nonRaisedRightHandX <= armsRaisedThresholdX &&
                                    joints[JointType_HandLeft].Position.Y - joints[JointType_ElbowLeft].Position.Y <= armsinlinewithelbowthreshold &&
                                    joints[JointType_HandRight].Position.Y - joints[JointType_ElbowRight].Position.Y <= armsinlinewithelbowthreshold &&
                                    shoulderSpineStable &&  midSpineStable && testReady && !testStarted && messagePrinted && !armsRaised)
                                {
                                    armsRaised = true;

                                    //storing raised hands and elbow coordinates
                                    raisedRightHandX = joints[JointType_HandRight].Position.X;
									raisedRightHandY = joints[JointType_HandRight].Position.Y;
									raisedRightHandZ = joints[JointType_HandRight].Position.Z;

									raisedLeftHandX = joints[JointType_HandLeft].Position.X;
									raisedLeftHandY = joints[JointType_HandLeft].Position.Y;
									raisedLeftHandZ = joints[JointType_HandLeft].Position.Z;

									raisedElbowRightX = joints[JointType_ElbowRight].Position.X;
									raisedElbowRightY = joints[JointType_ElbowRight].Position.Y;
									raisedElbowRightZ = joints[JointType_ElbowRight].Position.Z;

									raisedElbowLeftX = joints[JointType_ElbowLeft].Position.X;
									raisedElbowLeftY = joints[JointType_ElbowLeft].Position.Y;
									raisedElbowLeftZ = joints[JointType_ElbowLeft].Position.Z;

									raisedMidSpineX = joints[JointType_SpineMid].Position.X;
									raisedMidSpineY = joints[JointType_SpineMid].Position.Y;
									raisedMidSpineZ = joints[JointType_SpineMid].Position.Z;

									raisedShoulderSpineX = joints[JointType_SpineShoulder].Position.X;
									raisedShoulderSpineY = joints[JointType_SpineShoulder].Position.Y;
									raisedShoulderSpineZ = joints[JointType_SpineShoulder].Position.Z;
									
         //                           std::cout << "Right Hand Raised" << std::endl;
									//std::cout << "Left Hand Raised" << std::endl;

         //                           //print the initial coordinates for right and left hand, right and left elbow
         //                           std::cout << "Right Hand Coordinates | x: " << joints[JointType_HandRight].Position.X << "  | y: " << rightHandY << " | z: " << joints[JointType_HandRight].Position.Z << " |" << std::endl;
         //                           std::cout << "Right Elbow Coordinates| x: " << joints[JointType_ElbowRight].Position.X << " | y: " << rightElbowY << " | z: " << joints[JointType_ElbowRight].Position.Z << " |" << std::endl;
         //                           std::cout << "Left Hand Coordinates  | x: " << joints[JointType_HandLeft].Position.X << " | y: " << leftHandY << " | z: " << joints[JointType_HandLeft].Position.Z << " |" << std::endl;
         //                           std::cout << "Left Elbow Coordinates | x: " << joints[JointType_ElbowLeft].Position.X << "| y: " << leftElbowY << " | z: " << joints[JointType_ElbowLeft].Position.Z << " |" << std::endl;
         //                           

									//std::cout << "Arms Raised" << std::endl;
									//std::cout << "Please Bend Forward" << std::endl;
									speak("Test Ready, Please Bend Forward");
                                }

								if (armsRaised && testReady && !testStarted)
								{
									cv::putText(bgrMat, "Please Bend Forward", cv::Point(50, 100), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0, 0, 0), 2);
								}

                                //now the person bends forward coverind the distance in Z direction
								if (raisedLeftHandZ - joints[JointType_HandLeft].Position.Z >= armmovedthresholdZ &&
									raisedRightHandZ - joints[JointType_HandRight].Position.Z >= armmovedthresholdZ &&
									armsRaised  && !FinalMaximumDistance)
								{
									testStarted = true;
									//std::cout << "Test Started" << std::endl;
									//speak("Test Started");
                                
                                    // Calculate the distance of the hands from their initial positions
									RightHandDistance = fabs((raisedRightHandZ - joints[JointType_HandRight].Position.Z))*100.0f;    //current distance
									LeftHandDistance = fabs((raisedLeftHandZ - joints[JointType_HandLeft].Position.Z))*100.0f;       //current distance


                                    //cout << "Right Hand Distance: " << fabs((raisedRightHandZ - joints[JointType_HandRight].Position.Z)) * 100.0f << "cm" << endl;
									//cout << "Left Hand Distance: " << fabs((raisedLeftHandZ - joints[JointType_HandLeft].Position.Z)) * 100.0f << "cm" << endl;

									if ((MaximumRightHandDistance < RightHandDistance)) //checking if ccurrent distance is greater than maximum distance
									{
										MaximumRightHandDistance = RightHandDistance;
										}
                                    if (MaximumLeftHandDistance < LeftHandDistance)
                                    {
										MaximumLeftHandDistance = LeftHandDistance;
                                    }
									if ((RightHandDistance - MaximumRightHandDistance < 0.0f) && (LeftHandDistance - MaximumLeftHandDistance < 0.0f))
									{
										FinalMaximumDistance = true;
										//cout << "You Have Reached your limit." << endl;
									}
                                    
                                    if (armsRaised && testStarted && !initialPostureretain && !testComplete)
                                    {
										cv::putText(bgrMat, "Test Started", cv::Point(50, 50), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0, 0, 0), 2);
										cv::putText(bgrMat, "Right Hand Distance: " + std::to_string(RightHandDistance) + " cm",
											cv::Point(50, 500), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0, 0, 0), 2);
										cv::putText(bgrMat, "Left Hand Distance: " + std::to_string(LeftHandDistance) + " cm",
											cv::Point(50, 550), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0, 0, 0), 2);
										cv::putText(bgrMat, "Distance Covered: " + std::to_string(Distance) + " cm",
                                            cv::Point(50, 600), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0, 0, 0), 2);
                                    }

								}
                                if (testStarted && !testComplete && !initialPostureretain)
                                {
								//	cv::putText(bgrMat, "Test Started", cv::Point(50, 50), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0, 0, 0), 2);
                                
                                }
                                if (FinalMaximumDistance && testStarted && !testComplete)
                                {
                                    cv::putText(bgrMat, "Test Started", cv::Point(50, 50), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0, 0, 0), 2);
									cv::putText(bgrMat, "You Have Reached your limit.", cv::Point(50, 100), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0, 0, 0), 2);
                                    cv::putText(bgrMat, "Right Hand Distance: " + std::to_string(RightHandDistance) + " cm",
                                        cv::Point(50, 500), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0, 0, 0), 2);
                                    cv::putText(bgrMat, "Left Hand Distance: " + std::to_string(LeftHandDistance) + " cm",
                                        cv::Point(50, 550), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0, 0, 0), 2);
                                    cv::putText(bgrMat, "Distance Covered: " + std::to_string(Distance) + " cm",
                                        cv::Point(50, 600), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0, 0, 0), 2);

                                }

                                //now person moves back to initial position
                                if (raisedLeftHandZ - joints[JointType_HandLeft].Position.Z <= initialPositionHandsRetainedZ &&
                                    raisedRightHandZ - joints[JointType_HandRight].Position.Z <= initialPositionHandsRetainedZ &&
                                    fabs(raisedRightHandY - joints[JointType_HandRight].Position.Y) <= initialPositionHandsRetainedY &&
                                    fabs(raisedLeftHandY - joints[JointType_HandLeft].Position.Y) <= initialPositionHandsRetainedY &&
                                    fabs(raisedRightHandX - joints[JointType_HandRight].Position.X) <= initialPositionHandsRetainedX &&
                                    fabs(raisedLeftHandX - joints[JointType_HandLeft].Position.X) <= initialPositionHandsRetainedX &&
                                    testStarted && FinalMaximumDistance && !initialPostureretain && !testComplete)
                                {
                                    /*cout << "You Have Reached your initialPosition" << endl;
                                    std::cout << "Right Hand Coordinates | x: " << joints[JointType_HandRight].Position.X << "  | y: " << rightHandY << " | z: " << joints[JointType_HandRight].Position.Z << " |" << std::endl;
                                    std::cout << "Right Elbow Coordinates| x: " << joints[JointType_ElbowRight].Position.X << "  | y: " << rightElbowY << " | z: " << joints[JointType_ElbowRight].Position.Z << " |" << std::endl;
                                    std::cout << "Left Hand Coordinates  | x: " << joints[JointType_HandLeft].Position.X << " | y: " << leftHandY << " | z: " << joints[JointType_HandLeft].Position.Z << " |" << std::endl;
                                    std::cout << "Left Elbow Coordinates | x: " << joints[JointType_ElbowLeft].Position.X << " | y: " << leftElbowY << " | z: " << joints[JointType_ElbowLeft].Position.Z << " |" << std::endl;
                                    */
                                    initialPostureretain = true;
                                    testComplete = true;
                                    speak("Test Complete");
                                    if (MaximumRightHandDistance > MaximumLeftHandDistance)
                                    {
                                        Distance = MaximumRightHandDistance;
                                    }
                                    else if (MaximumLeftHandDistance > MaximumRightHandDistance)
                                    {
                                        Distance = MaximumLeftHandDistance;
                                    }

									cout << "Distance Covered: " << Distance << "cm" << endl;
									logSeatedForwardBendTest({ MaximumRightHandDistance }, { MaximumLeftHandDistance });
                                }
                                if (initialPostureretain && testComplete)
                                {
									cv::putText(bgrMat, "Test Complete", cv::Point(50, 50), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0, 0, 0), 2);
                                    cv::putText(bgrMat, "Right Hand Distance: " + std::to_string(RightHandDistance) + " cm",
                                        cv::Point(50, 500), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0, 0, 0), 2);
                                    cv::putText(bgrMat, "Left Hand Distance: " + std::to_string(LeftHandDistance) + " cm",
                                        cv::Point(50, 550), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0, 0, 0), 2);
                                    cv::putText(bgrMat, "Distance Covered: " + std::to_string(Distance) + " cm",
                                        cv::Point(50, 600), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0, 0, 0), 2);
                                }
                                
                                break;
                            }

    
                        }
                    }

                    bodyFrame->Release();
                }
                cv::imshow("Seated Forward Bent Test", bgrMat);
            }

            delete[] colorBuffer;
            colorFrame->Release();
            frameDescription->Release();
        }

        if (cv::waitKey(30) == 13) break;
    }

    colorFrameReader->Release();
    bodyFrameReader->Release();
    coordinateMapper->Release();
    colorSource->Release();
    sensor->Close();
    sensor->Release();

    return 0;
}
