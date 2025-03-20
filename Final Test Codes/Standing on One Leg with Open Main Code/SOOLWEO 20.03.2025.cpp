//Standing on One Leg With Open
#include <Kinect.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <deque> // For stability history tracking
#include <fstream>
#include <vector>
#include <sstream>
#include <string>
#include<sapi.h>
using namespace std;


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

//flags for the test and person stability
bool isPersonStable = false;
bool isTestReady = false;
bool isTestStarted = false;
bool isTestCompleted = false;
bool isRightFootRaised = false;
bool isLeftFootRaised = false;
bool rightFootRaised = false;
bool leftFootRaised = false;


std::chrono::duration<float> rightFootTimeElapsed;
std::chrono::duration<float> leftFootTimeElapsed;
std::chrono::steady_clock::time_point rightFootStartTime;
std::chrono::steady_clock::time_point rightFootEndTime;
std::chrono::steady_clock::time_point leftFootStartTime;
std::chrono::steady_clock::time_point leftFootEndTime;


bool isRightFootInAir = false;  // Flag to check if the right foot is in the air
bool isLeftFootInAir = false;   // Flag to check if the left foot is in the air
bool TestReadySpoken = false;

//variables to store coordinates of both foot
float initialRightFootX = 0.0f;
float initialRightFootY = 0.0f;
float initialRightFootZ = 0.0f;

float initialLeftFootX = 0.0f;
float initialLeftFootY = 0.0f;
float initialLeftFootZ = 0.0f;


//Thresholds for footraised
float rightFootRaisedThresholdZ = 0.1f;
float leftFootRaisedThresholdZ = 0.1f;
float rightFootRaisedThresholdY = 0.1f;
float leftFootRaisedThresholdY = 0.1f;

float rightFootElapsedTime = 0.0f;
float leftFootElapsedTime = 0.0f;


void logStandingOnOneLegTest(const std::vector<double>& rightFootElapsedTime, const std::vector<double>& leftFootElapsedTime) {
    std::string filename = "Standing_on_One_Leg_with_Eye_Open_Test_Results_2.csv";

    std::ofstream outfile(filename, std::ios::trunc);  // Open file in truncate mode (overwrite)
    if (!outfile) {
        std::cerr << "Error: Could not open file for writing.\n";
        return;
    }

    // Write the header
    outfile << "Standing on One Leg with Eye Open (s) 2\n";

    // Ensure vectors are not empty
    double maxRightFoot = rightFootElapsedTime.empty() ? 0.0 : *std::max_element(rightFootElapsedTime.begin(), rightFootElapsedTime.end());
    double maxLeftFoot = leftFootElapsedTime.empty() ? 0.0 : *std::max_element(leftFootElapsedTime.begin(), leftFootElapsedTime.end());
    double maxOverall = std::max(maxRightFoot, maxLeftFoot);

    // Write only the max overall standing time
    outfile << std::fixed << std::setprecision(2) << maxOverall << "\n";

    outfile.close();
}
// Constants for stability detection
const int stabilityFramesThreshold = 17; // Number of frames to check for stability
const float stabilityYThreshold = 0.1f; // Y-coordinate fluctuation threshold for stability
//const float stabilityXThreshold = 0.05f; // X-coordinate fluctuation threshold for stability 0.05f; // X-coordinate fluctuation threshold for stability

// Deques to store Y-coordinate history for stability detection
std::deque<float> leftFootYHistory, rightFootYHistory;

//timer library variables
std::chrono::steady_clock::time_point startTime;  // Store the start time when the foot is raised
std::chrono::steady_clock::time_point endTime;    // Store the end time when the foot comes back down


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

    cv::namedWindow("Standing on One Leg with Eye Open", cv::WINDOW_AUTOSIZE);

    bool messagePrinted = false; // Flag to track if message has been printed
    UINT64 trackedID = 0;  // Track the first participant
    bool isTrackingLocked = false;  // Ensure tracking remains locked

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

                    bool foundTrackedBody = false;

                    for (int i = 0; i < BODY_COUNT; ++i) {
                        IBody* body = bodies[i];
                        if (body) {
                            BOOLEAN isTracked = false;
                            body->get_IsTracked(&isTracked);

                            if (isTracked) {
                                // Print messages only once when a person is detected for the first time
                                UINT64 currentID;
                                body->get_TrackingId(&currentID);

                                if (!isTrackingLocked) {
                                    trackedID = currentID;
                                    isTrackingLocked = true;
                                    std::cout << "Participant Locked: " << trackedID << std::endl;
                                }
                                else if (trackedID != currentID) {
                                    // If a new person is detected, terminate the program
                                    std::cout << "Test Invalidated! New person detected." << std::endl;
                                    exit(0);
                                }

                                foundTrackedBody = true;

                                Joint joints[JointType_Count];
                                body->GetJoints(_countof(joints), joints);
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
                                float leftFootY = 0, rightFootY = 0;

                                // Only draw circles for the left and right foot joints
                                for (int j = 0; j < JointType_Count; j++) {
                                    if (joints[j].TrackingState == TrackingState_Tracked &&
                                        (j == JointType_FootLeft || j == JointType_FootRight)) {

                                        // Use the raw camera space coordinates (meters)
                                        float x = joints[j].Position.X;
                                        float y = joints[j].Position.Y;
                                        float z = joints[j].Position.Z;

                                        // Save specific Y values for the joints
                                        if (j == JointType_FootLeft) {
                                            leftFootY = y;
                                        }
                                        else if (j == JointType_FootRight) {
                                            rightFootY = y;
                                        }

                                        // Convert camera space to color space only for visualization
                                        ColorSpacePoint colorPoint;
                                        coordinateMapper->MapCameraPointToColorSpace(joints[j].Position, &colorPoint);
                                        int cx = static_cast<int>(colorPoint.X); // Pixel coordinates for the live feed
                                        int cy = static_cast<int>(colorPoint.Y);

                                        // Ensure the pixel coordinates are within bounds before drawing
                                        if (cx >= 0 && cx < width && cy >= 0 && cy < height) {
                                            cv::circle(bgrMat, cv::Point(cx, cy), 10, cv::Scalar(255, 0, 0), -1); // Draw a circle with radius 10

                                            // Add text label next to the joints
                                            if (j == JointType_FootLeft) {
                                                cv::putText(bgrMat, "Left Foot", cv::Point(cx + 10, cy), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2);
                                            }
                                            else if (j == JointType_FootRight) {
                                                cv::putText(bgrMat, "Right Foot", cv::Point(cx + 10, cy), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2);
                                            }

                                            // Display the decimal camera space coordinates
                                            cv::putText(bgrMat, "X: " + std::to_string(x) + " Y: " + std::to_string(y) + " Z: " + std::to_string(z),
                                                cv::Point(cx + 10, cy + 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
                                        }
                                    }
                                }

                                // Update history
                                if (leftFootYHistory.size() >= stabilityFramesThreshold) leftFootYHistory.pop_front();
                                if (rightFootYHistory.size() >= stabilityFramesThreshold) rightFootYHistory.pop_front();

                                leftFootYHistory.push_back(leftFootY);
                                rightFootYHistory.push_back(rightFootY);

                                // Check stability
                                bool leftFootStable = isStable(leftFootYHistory, stabilityYThreshold);
                                bool rightFootStable = isStable(rightFootYHistory, stabilityYThreshold);

                                // Conditional statement: When feet are stable, print message
                                if (leftFootStable && rightFootStable && !messagePrinted && !isTestReady && !isTestStarted && !isPersonStable) {
                                    messagePrinted = true; // Set the flag to true to prevent repeated printing
                                    isTestReady = true;
                                    isPersonStable = true;
                                    speak("Test Ready");
                                    if (!TestReadySpoken)
                                        TestReadySpoken = true;
                                    speak("Please raise your right foot");

                                    //std::cout << "Test Ready" << std::endl;
                                    //std::cout << "Feet are stable." << std::endl;

                                    // Display initial Y coordinates for feet
                                   // std::cout << "Initial Left Foot Y: " << leftFootY << std::endl;
                                    //std::cout << "Initial Right Foot Y: " << rightFootY << std::endl;

                                    initialRightFootX = joints[JointType_FootRight].Position.X;
                                    initialRightFootY = joints[JointType_FootRight].Position.Y;
                                    initialRightFootZ = joints[JointType_FootRight].Position.Z;
                                    initialLeftFootX = joints[JointType_FootLeft].Position.X;
                                    initialLeftFootY = joints[JointType_FootLeft].Position.Y;
                                    initialLeftFootZ = joints[JointType_FootLeft].Position.Z;
                                    //std::cout << "Right Foot Coordinates | x: " << joints[JointType_FootRight].Position.X << "  | y: " << rightFootY << " | z: " << joints[JointType_FootRight].Position.Z << " |" << std::endl;
                                    //std::cout << "Left Foot Coordinates  | x: " << joints[JointType_FootLeft].Position.X << "  | y: " << leftFootY << " | z: " << joints[JointType_FootLeft].Position.Z << " |" << std::endl;
                                    //std::cout << "Please Raise your Dominant Foot " << std::endl;
                                    //speak("Test Ready");
                                }
                                if (messagePrinted && isPersonStable && isTestReady && !isTestStarted)
                                {
                                    cv::putText(bgrMat, "Test Ready", cv::Point(50, 50), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0, 0, 0), 2);
                                    cv::putText(bgrMat, "Please Raise your Right Foot", cv::Point(50, 100), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0, 0, 0), 2);
                                    //speak("Test Ready");
                                    //speak("Please Raise your Right Foot");
                                }
                                //if person has a dominant foot of right foot, then raises right foot first if timer > 70, test complete if timer < 70 raisess left foot first and then test is complete
                                // IN CASE IF THE DOMINANT FOOT WAS RIGHT FOOT
                                //first Right Foot is raised
                                if ((fabs(initialRightFootY - joints[JointType_FootRight].Position.Y) > rightFootRaisedThresholdY)&& !rightFootRaised && !leftFootRaised && !isTestStarted && isPersonStable && messagePrinted) {
                                    rightFootRaised = true;
                                    isTestStarted = true;
                                    //std::cout << "Test Started" << std::endl;
                                    //std::cout << "Right Foot Raised" << std::endl;
                                    //std::cout << "Timer Started for Right Foot" << std::endl;

                                    // Start the timer when the right foot is raised
                                    rightFootStartTime = std::chrono::steady_clock::now();
                                    isRightFootInAir = true;
                                }

                                else if ((fabs(initialRightFootY - joints[JointType_FootRight].Position.Y) <= rightFootRaisedThresholdY) && rightFootRaised && isRightFootInAir && isTestStarted && !isTestCompleted && isPersonStable && messagePrinted) {
                                    // Stop the timer when the right foot touches the ground

                                    rightFootEndTime = std::chrono::steady_clock::now();
                                    rightFootTimeElapsed = rightFootEndTime - rightFootStartTime;
                                    /*std::cout << "Right Foot Returned to Ground." << std::endl;
                                    std::cout << "Right Foot in the air for: " << rightFootTimeElapsed.count() << " seconds." << std::endl;
                                    */
                                    rightFootElapsedTime = rightFootTimeElapsed.count();
                                    if (rightFootTimeElapsed.count() > 60.0f)
                                    {
                                        isTestCompleted = true;
                                        /*std::cout << "Dominant Foot Time Greater Than 60 seconds, no need to do test for left Foot" << std::endl;
                                        std::cout << "Test Complete" << std::endl;
                                        */
                                        speak("Test complete");
                                    }
                                    else if (rightFootTimeElapsed.count() <= 60.0f)
                                    {
                                        //std::cout << "Right Foot Time Less Than 60 seconds, Please Raise Your Left Foot." << std::endl;
                                        speak("Please Raise Your Left Foot");
                                    }
                                    isRightFootInAir = false;
                                }
                                //then left foot is raised
                                if ((fabs(initialLeftFootY - joints[JointType_FootLeft].Position.Y) >leftFootRaisedThresholdY) && !leftFootRaised && rightFootRaised && isTestStarted && !isTestCompleted && isPersonStable && messagePrinted) {
                                    leftFootRaised = true;
                                    /*std::cout << "Left Foot Raised" << std::endl;
                                    std::cout << "Timer Started for Left Foot" << std::endl;*/

                                    // Start the timer when the left foot is raised
                                    leftFootStartTime = std::chrono::steady_clock::now();
                                    isLeftFootInAir = true;
                                }
                                // Condition when the left foot touches the ground (timer stops)
                                else if ((fabs(initialLeftFootY - joints[JointType_FootLeft].Position.Y) <= leftFootRaisedThresholdY) && leftFootRaised && isLeftFootInAir && isTestStarted && !isTestCompleted && isPersonStable && messagePrinted) {
                                    // Stop the timer when the left foot touches the ground
                                    leftFootEndTime = std::chrono::steady_clock::now();
                                    leftFootTimeElapsed = leftFootEndTime - leftFootStartTime;
                                    /*std::cout << "Left Foot Returned to Ground." << std::endl;
                                    std::cout << "Left Foot in the air for: " << leftFootTimeElapsed.count() << " seconds." << std::endl;*/

                                    // Mark test as completed
                                    isTestCompleted = true;
                                    //std::cout << "Test Complete" << std::endl;
                                    speak("Test Complete");
                                    // Print time taken by left foot
                                    //std::cout << "Left Foot Time: " << leftFootTimeElapsed.count() << " seconds." << std::endl;
                                    // Print time taken by right foot
                                    //std::cout << "Right Foot Time: " << rightFootTimeElapsed.count() << " seconds." << std::endl;
                                    rightFootElapsedTime = rightFootTimeElapsed.count();
                                    leftFootElapsedTime = leftFootTimeElapsed.count();
                                    logStandingOnOneLegTest({ static_cast<double>(rightFootElapsedTime) },
                                        { static_cast<double>(leftFootElapsedTime) });
                                }
                                if (isTestCompleted)
                                {
                                    //put text to display test completed
                                    cv::putText(bgrMat, "Test Completed", cv::Point(50, 50), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0, 0, 0), 2);
                                    cv::putText(bgrMat, "Right Foot Time: " + std::to_string(rightFootElapsedTime), cv::Point(50, 550), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0, 0, 0), 2);
                                    cv::putText(bgrMat, "Left Foot Time: " + std::to_string(leftFootElapsedTime), cv::Point(50, 600), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0, 0, 0), 2);
                                    //speak("Test Completed");
                                }

                                break;
                            }

                        }
                    }

                    for (int i = 0; i < BODY_COUNT; ++i) {
                        if (bodies[i]) {
                            bodies[i]->Release();
                        }
                    }


                    bodyFrame->Release();
                }
                cv::imshow("Standing on One Leg with Eye Open", bgrMat);
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
