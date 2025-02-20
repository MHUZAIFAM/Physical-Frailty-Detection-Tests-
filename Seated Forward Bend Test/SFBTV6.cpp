//without Datalogging but including voice features
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
float initialPositionHandsRetainedY = 0.3f;
float initialPositionHandsRetainedX = 0.3f;



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
const int stabilityFramesThreshold = 15; // Number of frames to check for stability
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
                                
                                // Get relevant joint positions
                                float minX = std::min({ joints[JointType_HandLeft].Position.X, joints[JointType_ElbowLeft].Position.X, joints[JointType_SpineShoulder].Position.X, joints[JointType_SpineMid].Position.X });
                                float maxX = std::max({ joints[JointType_HandRight].Position.X, joints[JointType_ElbowRight].Position.X, joints[JointType_SpineShoulder].Position.X, joints[JointType_SpineMid].Position.X });
                                float minY = std::min({ joints[JointType_HandLeft].Position.Y, joints[JointType_HandRight].Position.Y, joints[JointType_ElbowLeft].Position.Y, joints[JointType_ElbowRight].Position.Y, joints[JointType_SpineShoulder].Position.Y, joints[JointType_SpineMid].Position.Y });
                                float maxY = std::max({ joints[JointType_HandLeft].Position.Y, joints[JointType_HandRight].Position.Y, joints[JointType_ElbowLeft].Position.Y, joints[JointType_ElbowRight].Position.Y, joints[JointType_SpineShoulder].Position.Y, joints[JointType_SpineMid].Position.Y });

                                // Convert Kinect coordinates to image pixels
                                int rectX = static_cast<int>(minX * 500 + 320);
                                int rectY = static_cast<int>(minY * -500 + 240);
                                int rectWidth = static_cast<int>((maxX - minX) * 500);
                                int rectHeight = static_cast<int>((maxY - minY) * -500);

                                // Draw bounding rectangle
                                cv::rectangle(bgrMat, cv::Rect(rectX, rectY, rectWidth, rectHeight), cv::Scalar(0, 255, 0), 2);
                                
                                float leftHandY = 0, rightHandY = 0, 
                                      leftElbowY = 0, rightElbowY = 0,
                                      shoulderSpineY = 0, midSpineY = 0;

                                // Only draw circles for selected joints
                                for (int j = 0; j < JointType_Count; j++) {
                                    if (joints[j].TrackingState == TrackingState_Tracked &&
                                        (j == JointType_HandLeft || j == JointType_HandRight ||
                                            j == JointType_ElbowLeft || j == JointType_ElbowRight ||
                                            j == JointType_SpineMid || j == JointType_SpineShoulder)) {

                                        // Use the raw camera space coordinates (meters)
                                        float x = joints[j].Position.X;
                                        float y = joints[j].Position.Y;
                                        float z = joints[j].Position.Z;

                                        // Save specific Y values for the joints
                                        if (j == JointType_HandLeft) {
                                            leftHandY = y;
                                            if (initialLeftHandZ == -1.0f) {
                                                initialLeftHandZ = z;
                                            }
                                        }
                                        else if (j == JointType_HandRight) {
                                            rightHandY = y;
                                            if (initialRightHandZ == -1.0f) {
                                                initialRightHandZ = z;
                                            }
                                        }
                                        else if (j == JointType_ElbowLeft) {
                                            leftElbowY = y;
                                            if (initialLeftElbowZ == -1.0f) {
                                                initialLeftElbowZ = z;
                                            }
                                        }
                                        else if (j == JointType_ElbowRight) {
                                            rightElbowY = y;
                                            if (initialRightElbowZ == -1.0f) {
                                                initialRightElbowZ = z;
                                            }
                                        }
                                        else if (j == JointType_SpineMid) {
                                            midSpineY = y;
                                            if (initialMidSpineZ == -1.0f) {
                                                initialMidSpineZ = z;
                                            }
                                        }
                                        else if (j == JointType_SpineShoulder) {
                                            shoulderSpineY = y;
                                            if (initialShoulderSpineZ == -1.0f) {
                                                initialShoulderSpineZ = z;
                                            }
                                        }

                                        // Convert camera space to color space for visualization
                                        ColorSpacePoint colorPoint;
                                        coordinateMapper->MapCameraPointToColorSpace(joints[j].Position, &colorPoint);
                                        int cx = static_cast<int>(colorPoint.X); // Pixel coordinates for the live feed
                                        int cy = static_cast<int>(colorPoint.Y);

                                        // Ensure the pixel coordinates are within bounds before drawing
                                        if (cx >= 0 && cx < width && cy >= 0 && cy < height) {
                                            cv::circle(bgrMat, cv::Point(cx, cy), 10, cv::Scalar(0, 0, 255), -1); // Draw a circle

                                            // Add text labels for the joints
                                            std::string label;
                                            if (j == JointType_HandLeft) label = "Left Hand";
                                            else if (j == JointType_HandRight) label = "Right Hand";
                                            else if (j == JointType_ElbowLeft) label = "Left Elbow";
                                            else if (j == JointType_ElbowRight) label = "Right Elbow";
                                            else if (j == JointType_SpineMid) label = "Mid Spine";
                                            else if (j == JointType_SpineShoulder) label = "Shoulder Spine";

                                            cv::putText(bgrMat, label, cv::Point(cx + 10, cy), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2);

                                            // Display the decimal camera space coordinates
                                            cv::putText(bgrMat, "X: " + std::to_string(x) + " Y: " + std::to_string(y) + " Z: " + std::to_string(z),
                                                cv::Point(cx + 10, cy + 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
                                        }
                                    }
                                }


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
								if (leftHandStable && rightHandStable && leftElbowStable && rightElbowStable && midSpineStable && shoulderSpineStable &&
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


									std::cout << "Test Ready." << std::endl;
									std::cout << "Please Raise your Arms." << std::endl;
									// Display initial Y coordinates for arms
									
                                    std::cout << "Initial Left Hand Y: " << leftHandY << std::endl;
									std::cout << "Initial Right Hand Y: " << rightHandY << std::endl;
									std::cout << "Initial Left Elbow Y: " << leftElbowY << std::endl;
									std::cout << "Initial Right Elbow Y: " << rightElbowY << std::endl;
									std::cout << "Initial Mid Spine Y: " << midSpineY << std::endl;
									std::cout << "Initial Shoulder Spine Y: " << shoulderSpineY << std::endl;

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
                                    leftHandStable && rightHandStable && midSpineStable && testReady && !testStarted && messagePrinted && !armsRaised)
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
									
                                    std::cout << "Right Hand Raised" << std::endl;
									std::cout << "Left Hand Raised" << std::endl;

                                    //print the initial coordinates for right and left hand, right and left elbow
                                    std::cout << "Right Hand Coordinates | x: " << joints[JointType_HandRight].Position.X << "  | y: " << rightHandY << " | z: " << joints[JointType_HandRight].Position.Z << " |" << std::endl;
                                    std::cout << "Right Elbow Coordinates| x: " << joints[JointType_ElbowRight].Position.X << " | y: " << rightElbowY << " | z: " << joints[JointType_ElbowRight].Position.Z << " |" << std::endl;
                                    std::cout << "Left Hand Coordinates  | x: " << joints[JointType_HandLeft].Position.X << " | y: " << leftHandY << " | z: " << joints[JointType_HandLeft].Position.Z << " |" << std::endl;
                                    std::cout << "Left Elbow Coordinates | x: " << joints[JointType_ElbowLeft].Position.X << "| y: " << leftElbowY << " | z: " << joints[JointType_ElbowLeft].Position.Z << " |" << std::endl;
                                    

									std::cout << "Arms Raised" << std::endl;
									std::cout << "Please Bend Forward" << std::endl;
									speak("Please bend forward");
                                }

								if (armsRaised && testReady && !testStarted)
								{
									cv::putText(bgrMat, "Please Bend Forward", cv::Point(50, 100), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0, 0, 0), 2);
								}

                                //now the person bends forward coverind the distance in Z direction
								if (raisedLeftHandZ - joints[JointType_HandLeft].Position.Z >= armmovedthresholdZ &&
									raisedRightHandZ - joints[JointType_HandRight].Position.Z >= armmovedthresholdZ &&
									joints[JointType_HandRight].Position.X - raisedRightHandX <= armmovedthresholdX &&
									joints[JointType_HandLeft].Position.X - raisedLeftHandX <= armmovedthresholdX &&
									armsRaised  && !FinalMaximumDistance)
								{
									testStarted = true;
									std::cout << "Test Started" << std::endl;
									speak("Test Started");
                                
                                    // Calculate the distance of the hands from their initial positions
									RightHandDistance = ((raisedRightHandZ - joints[JointType_HandRight].Position.Z))*100.0f;
									LeftHandDistance = ((raisedLeftHandZ - joints[JointType_HandLeft].Position.Z))*100.0f;
                                    cout << "Right Hand Distance: " << RightHandDistance << "cm" << endl;
									cout << "Left Hand Distance: " << LeftHandDistance << "cm" << endl;

									if ((MaximumRightHandDistance < RightHandDistance) || (MaximumLeftHandDistance<LeftHandDistance))
									{
										MaximumRightHandDistance = RightHandDistance;
										MaximumLeftHandDistance = LeftHandDistance;
									}
                                    
									if (RightHandDistance - MaximumRightHandDistance <= 0.0f && LeftHandDistance - MaximumLeftHandDistance <= 0.0f)
									{
										FinalMaximumDistance = true;
										cout << "You Have Reached your limit." << endl;
									}
                                    
                                    if (MaximumRightHandDistance > MaximumLeftHandDistance)
                                    {
                                        Distance = MaximumRightHandDistance;
                                    }
                                    else if (MaximumLeftHandDistance > MaximumRightHandDistance)
                                    {
                                        Distance = MaximumLeftHandDistance;
                                    }

                                    if (armsRaised && testStarted && !initialPostureretain && !testComplete)
                                    {
										cv::putText(bgrMat, "Test Started", cv::Point(50, 50), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0, 0, 0), 2);
										cv::putText(bgrMat, "Right Hand Distance: " + std::to_string(RightHandDistance) + " cm",
											cv::Point(50, 400), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0, 0, 0), 2);
										cv::putText(bgrMat, "Left Hand Distance: " + std::to_string(LeftHandDistance) + " cm",
											cv::Point(50, 450), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0, 0, 0), 2);
										cv::putText(bgrMat, "Distance Covered: " + std::to_string(Distance) + " cm",
                                            cv::Point(50, 500), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0, 0, 0), 2);
                                    }

								}
                                if (testStarted && !testComplete && !initialPostureretain)
                                {
									cv::putText(bgrMat, "Test Started", cv::Point(50, 50), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0, 0, 0), 2);
                                }
                                if (FinalMaximumDistance && testStarted && !testComplete)
                                {
									cv::putText(bgrMat, "You Have Reached your limit.", cv::Point(50, 100), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0, 0, 0), 2);
                                }

                                //now person moves back to initial position
                                if (raisedLeftHandZ - joints[JointType_HandLeft].Position.Z <= initialPositionHandsRetainedZ &&
                                    raisedRightHandZ - joints[JointType_HandRight].Position.Z <= initialPositionHandsRetainedZ &&
                                    raisedRightHandY - joints[JointType_HandRight].Position.Y <= initialPositionHandsRetainedY &&
                                    raisedLeftHandY - joints[JointType_HandLeft].Position.Y <= initialPositionHandsRetainedY &&
                                    raisedRightHandX - joints[JointType_HandRight].Position.X <= initialPositionHandsRetainedX &&
                                    raisedLeftHandX - joints[JointType_HandLeft].Position.X <= initialPositionHandsRetainedX &&
                                    testStarted && FinalMaximumDistance && !initialPostureretain && !testComplete)
                                {
                                    cout << "You Have Reached your initialPosition" << endl;
                                    std::cout << "Right Hand Coordinates | x: " << joints[JointType_HandRight].Position.X << "  | y: " << rightHandY << " | z: " << joints[JointType_HandRight].Position.Z << " |" << std::endl;
                                    std::cout << "Right Elbow Coordinates| x: " << joints[JointType_ElbowRight].Position.X << "  | y: " << rightElbowY << " | z: " << joints[JointType_ElbowRight].Position.Z << " |" << std::endl;
                                    std::cout << "Left Hand Coordinates  | x: " << joints[JointType_HandLeft].Position.X << " | y: " << leftHandY << " | z: " << joints[JointType_HandLeft].Position.Z << " |" << std::endl;
                                    std::cout << "Left Elbow Coordinates | x: " << joints[JointType_ElbowLeft].Position.X << " | y: " << leftElbowY << " | z: " << joints[JointType_ElbowLeft].Position.Z << " |" << std::endl;
                                    initialPostureretain = true;
                                    testComplete = true;
                                    speak("Test Complete");
                                }
                                if (initialPostureretain && testComplete)
                                {
									cv::putText(bgrMat, "Test Complete", cv::Point(50, 50), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0, 0, 0), 2);
                                    cv::putText(bgrMat, "Right Hand Distance: " + std::to_string(RightHandDistance) + " cm",
                                        cv::Point(50, 400), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0, 0, 0), 2);
                                    cv::putText(bgrMat, "Left Hand Distance: " + std::to_string(LeftHandDistance) + " cm",
                                        cv::Point(50, 450), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0, 0, 0), 2);
                                    cv::putText(bgrMat, "Distance Covered: " + std::to_string(Distance) + " cm",
                                        cv::Point(50, 500), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0, 0, 0), 2);
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
