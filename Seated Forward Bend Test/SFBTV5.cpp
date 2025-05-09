//place camera on the stand and bend it forward
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
float nonRaisedElbowLeftX = 0.0f, nonRaisedElbowLeftY = 0.0f, nonRaisedElbowLeftZ = 0.0f;
float nonRaisedMidSpineX = 0.0f, nonRaisedMidSpineY = 0.0f, nonRaisedMidSpineZ = 0.0f;
float nonRaisedBaseSpineX = 0.0f, nonRaisedBaseSpineY = 0.0f, nonRaisedBaseSpineZ = 0.0f;

//variables for Raised Arms Joint Coordinates
float raisedLeftHandX = 0.0f, raisedLeftHandY = 0.0f, raisedLeftHandZ = 0.0f;
float raisedElbowLeftX = 0.0f, raisedElbowLeftY = 0.0f, raisedElbowLeftZ = 0.0f;
float raisedMidSpineX = 0.0f, raisedMidSpineY = 0.0f, raisedMidSpineZ = 0.0f;
float raisedBaseSpineX = 0.0f, raisedBaseSpineY = 0.0f, raisedBaseSpineZ = 0.0f;

//arms rasied threshold
float armsRaisedThresholdY = 0.1f;
float armmovedthresholdX = 0.2f;
float armmovedthresholdY = 0.2f;
float armmovedthresholdZ = 0.05f;
float initialPostureThreshold = 0.3f;


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
float Distance = 0.0f;
float MaximumDistance = 0.0f;
float ElbowDistance = 0.0f;
float ElbowMaximumDistance = 0.0f;

// Constants for stability detection
const int stabilityFramesThreshold = 15; // Number of frames to check for stability
const float stabilityYThreshold = 0.02f; // Y-coordinate fluctuation threshold for stability

// Initial Z-coordinate values for left hand, mid spine, and base spine (assuming -1 is invalid/uninitialized)
float initialLeftHandZ = -1.0f, initialMidSpineZ = -1.0f, initialBaseSpineZ = -1.0f;

// Deques to store Y-coordinate history for stability detection
std::deque<float> leftHandYHistory, midSpineYHistory, baseSpineYHistory;

// Variables to track Y-coordinates of joints in previous frames
float lastLeftHandY = -1.0f, lastMidSpineY = -1.0f, lastBaseSpineY = -1.0f;

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

                                // List of joints to label
                                const int jointsToLabel[] = {
                                    JointType_HandLeft,
                                    JointType_ElbowLeft,
                                    JointType_SpineMid,
                                    JointType_SpineBase
                                };

                                const char* jointLabels[] = {
                                    "Left Hand",
                                    "Left Elbow",
                                    "Mid Spine",
                                    "Base Spine"
                                };
                                // Draw and label the joints
                                // Draw and label the joints
                                for (size_t j = 0; j < sizeof(jointsToLabel) / sizeof(jointsToLabel[0]); j++) {
                                    int jointType = jointsToLabel[j];

                                    if (joints[jointType].TrackingState == TrackingState_Tracked) {
                                        // Use the raw camera space coordinates (meters)
                                        float x = joints[jointType].Position.X;
                                        float y = joints[jointType].Position.Y;
                                        float z = joints[jointType].Position.Z;

                                        // Convert camera space to color space for visualization
                                        ColorSpacePoint colorPoint;
                                        coordinateMapper->MapCameraPointToColorSpace(joints[jointType].Position, &colorPoint);
                                        int cx = static_cast<int>(colorPoint.X);
                                        int cy = static_cast<int>(colorPoint.Y);

                                        // Ensure pixel coordinates are within bounds
                                        if (cx >= 0 && cx < width && cy >= 0 && cy < height) {
                                            cv::circle(bgrMat, cv::Point(cx, cy), 10, cv::Scalar(255, 0, 0), -1); // Draw a circle
                                            cv::putText(bgrMat, jointLabels[j], cv::Point(cx + 10, cy), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2);

                                            // Display the decimal camera space coordinates
                                            cv::putText(bgrMat, "X: " + std::to_string(x) + " Y: " + std::to_string(y) + " Z: " + std::to_string(z),
                                                cv::Point(cx + 10, cy + 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
                                        }
                                    }
                                }

                                // Deques for stability tracking
                                if (leftHandYHistory.size() >= stabilityFramesThreshold) leftHandYHistory.pop_front();
                                if (midSpineYHistory.size() >= stabilityFramesThreshold) midSpineYHistory.pop_front();
                                if (baseSpineYHistory.size() >= stabilityFramesThreshold) baseSpineYHistory.pop_front();

                                // Adding current Y-values to the history for stability check
                                leftHandYHistory.push_back(joints[JointType_HandLeft].Position.Y);
                                midSpineYHistory.push_back(joints[JointType_SpineMid].Position.Y);
                                baseSpineYHistory.push_back(joints[JointType_SpineBase].Position.Y);

                                // Checking stability
                                bool leftHandStable = isStable(leftHandYHistory, stabilityYThreshold);
                                bool midSpineStable = isStable(midSpineYHistory, stabilityYThreshold);
                                bool baseSpineStable = isStable(baseSpineYHistory, stabilityYThreshold);

                                // Condition to check when joints are stable
                                if (leftHandStable && midSpineStable && baseSpineStable && !messagePrinted && !testReady && !testStarted && !armsRaised) {
                                    messagePrinted = true; // Set the flag to prevent repeated printing
                                    std::cout << "Test Ready" << std::endl;
                                    std::cout << "Person Stable" << std::endl;
                                    speak("Test Ready");
                                    speak("Please raise your arms");
                                    std::cout << "Please Raise your arms and sit straight" << std::endl;
                                    //print non raised arms coordinates on CLI for arm elbow and 
                                    nonRaisedLeftHandX = joints[JointType_HandLeft].Position.X;
                                    nonRaisedLeftHandY = joints[JointType_HandLeft].Position.Y;
                                    nonRaisedLeftHandZ = joints[JointType_HandLeft].Position.Z;
                                    nonRaisedElbowLeftX = joints[JointType_ElbowLeft].Position.X;
                                    nonRaisedElbowLeftY = joints[JointType_ElbowLeft].Position.Y;
                                    nonRaisedElbowLeftZ = joints[JointType_ElbowLeft].Position.Z;
                                    nonRaisedMidSpineX = joints[JointType_SpineMid].Position.X;
                                    nonRaisedMidSpineY = joints[JointType_SpineMid].Position.Y;
                                    nonRaisedMidSpineZ = joints[JointType_SpineMid].Position.Z;
                                    nonRaisedBaseSpineX = joints[JointType_SpineBase].Position.X;
                                    nonRaisedBaseSpineY = joints[JointType_SpineBase].Position.Y;
                                    nonRaisedBaseSpineZ = joints[JointType_SpineBase].Position.Z;

                                    //print non raised arms coordinates on CLI for arm elbow and spine
                                    std::cout << "Left Hand: " << nonRaisedLeftHandX << ", " << nonRaisedLeftHandY << ", " << nonRaisedLeftHandZ << std::endl;
                                    std::cout << "Left Elbow: " << nonRaisedElbowLeftX << ", " << nonRaisedElbowLeftY << ", " << nonRaisedElbowLeftZ << std::endl;
                                    std::cout << "Mid Spine: " << nonRaisedMidSpineX << ", " << nonRaisedMidSpineY << ", " << nonRaisedMidSpineZ << std::endl;
                                    std::cout << "Base Spine: " << nonRaisedBaseSpineX << ", " << nonRaisedBaseSpineY << ", " << nonRaisedBaseSpineZ << std::endl;
                                    testReady = true;
                                }



                                // Put text on screen when the test is ready
                                if (messagePrinted && !armsRaised && !testStarted && !armsRaised) {
                                    cv::putText(bgrMat, "Test Ready", cv::Point(50, 50), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0,0,0), 2);
                                    cv::putText(bgrMat, "Please Raise your arms and sit straight", cv::Point(50, 100), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0,0,0), 2);
                                }

                                if (fabs(joints[JointType_ElbowLeft].Position.Y - joints[JointType_HandLeft].Position.Y) <= armsRaisedThresholdY &&
                                    messagePrinted && testReady && !testStarted &&
                                    leftHandStable && midSpineStable && baseSpineStable && !armsRaised)
                                {
                                    armsRaised = true;
                                    std::cout << "Arms Raised" << std::endl;
                                    speak("Please bend forward");
                                    //print raised arms coordinates on CLI for arm elbow and spine
                                    raisedLeftHandX = joints[JointType_HandLeft].Position.X;
                                    raisedLeftHandY = joints[JointType_HandLeft].Position.Y;
                                    raisedLeftHandZ = joints[JointType_HandLeft].Position.Z;
                                    raisedElbowLeftX = joints[JointType_ElbowLeft].Position.X;
                                    raisedElbowLeftY = joints[JointType_ElbowLeft].Position.Y;
                                    raisedElbowLeftZ = joints[JointType_ElbowLeft].Position.Z;
                                    raisedMidSpineX = joints[JointType_SpineMid].Position.X;
                                    raisedMidSpineY = joints[JointType_SpineMid].Position.Y;
                                    raisedMidSpineZ = joints[JointType_SpineMid].Position.Z;
                                    raisedBaseSpineX = joints[JointType_SpineBase].Position.X;
                                    raisedBaseSpineY = joints[JointType_SpineBase].Position.Y;
                                    raisedBaseSpineZ = joints[JointType_SpineBase].Position.Z;

                                    //print the raised arms coordinates on CLI for arm elbow and spine
                                    std::cout << "Left Hand: " << raisedLeftHandX << ", " << raisedLeftHandY << ", " << raisedLeftHandZ << std::endl;
                                    std::cout << "Left Elbow: " << raisedElbowLeftX << ", " << raisedElbowLeftY << ", " << raisedElbowLeftZ << std::endl;
                                    std::cout << "Mid Spine: " << raisedMidSpineX << ", " << raisedMidSpineY << ", " << raisedMidSpineZ << std::endl;
                                    std::cout << "Base Spine: " << raisedBaseSpineX << ", " << raisedBaseSpineY << ", " << raisedBaseSpineZ << std::endl;
                                }


                                //put text test ready and arms raised on screen
                                if (armsRaised && testReady && !testStarted)
                                {
                                    // cv::putText(bgrMat, "Arms Raised", cv::Point(50, 100), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 255), 2);
                                    cv::putText(bgrMat, "Test Ready", cv::Point(50, 50), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0,0,0), 2);
                                    //cv::putText(bgrMat, "Please sit straight", cv::Point(50, 150), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 255), 2);
                                    //after a while please bend forward
                                    cv::putText(bgrMat, "Please bend forward", cv::Point(50, 100), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0, 0,0), 2);
                                }

                                //now person has bend forward, now to calculate distance
                                if (fabs(joints[JointType_HandLeft].Position.Z - raisedLeftHandZ) > armmovedthresholdZ &&
                                    fabs(joints[JointType_ElbowLeft].Position.X - raisedElbowLeftX) <= armmovedthresholdX &&
                                    testReady && messagePrinted && !FinalMaximumDistance)
                                {
                                    testStarted = true;
                                    //now to calculate distance
                                    /*Distance = sqrt(pow(raisedLeftHandX - joints[JointType_HandLeft].Position.X, 2) +
                                        pow(raisedLeftHandY - joints[JointType_HandLeft].Position.Y, 2) +
                                        pow(raisedLeftHandZ - joints[JointType_HandLeft].Position.Z, 2)) * 100.0f;*/

                                    Distance = (raisedLeftHandZ - joints[JointType_HandLeft].Position.Z) * 100.0f;
                                    //display Distance on live feed
                                    cv::putText(bgrMat, "Distance: " + std::to_string(Distance) + " cm", cv::Point(50, 100), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 0), 2);

                                    if (Distance > MaximumDistance)
                                    {
                                        MaximumDistance = Distance;
                                        std::cout << "Distance: " << Distance << std::endl;
                                    }
                                    else if (Distance < MaximumDistance)
                                    {
                                        FinalMaximumDistance = true;
                                        //initialPostureretain= true;
                                    }
                                   
                                }

                                if (FinalMaximumDistance && !initialPostureretain && !onetimereading && !testComplete)
                                {
                                    if (MaximumDistance > 0.0f)
                                    {
                                        //print the final Maximum Distance on CLI
                                        std::cout << "Distance Covered by Hand: " << MaximumDistance << std::endl;
                                        onetimereading = true;
                                    }
                                    else if (MaximumDistance == 0.0f)
                                    {
                                        //print invalid test, test restarted and reset all the flags
                                        cout << "Invalid Test" << endl;
                                        //reset all the flags
                                        testStarted = false;
                                        testReady = false;
                                        armsRaised = false;
                                        messagePrinted = false;
                                        isPersonStable = false;
                                        isPersonStraight = false;
                                        FinalMaximumDistance = false;
                                        testComplete = false;
                                        onetimereading = false;
                                        initialPostureretain = false;
                                        speak("Test Invalid");
										speak("Test Restarted");
                                    }

                                }


                                if (raisedLeftHandZ - joints[JointType_HandLeft].Position.Z <= initialPostureThreshold && FinalMaximumDistance && !initialPostureretain && !testComplete)
                                {
                                    std::cout << "Test completed Please retain your initial posture" << std::endl;
                                    initialPostureretain = true;
                                    testComplete = true;
                                    speak("Test Completed");


                                }
                                if (testComplete)
                                {
									//print the final Maximum Distance on live feed
									cv::putText(bgrMat, "Test Completed", cv::Point(50, 50), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0, 0, 0), 2);
									cv::putText(bgrMat, "Distance: " + std::to_string(MaximumDistance) + " cm", cv::Point(50, 100), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0, 0, 0), 2);
									
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
