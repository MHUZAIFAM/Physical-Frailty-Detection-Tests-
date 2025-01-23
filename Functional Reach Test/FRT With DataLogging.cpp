//final perfect code
#include <Kinect.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <deque> // For stability history tracking
#include <fstream>
#include <vector>
#include <sstream>
#include <string>
using namespace std;

//DataLogging Function
void logFunctionalReachTest(const std::vector<double>& rightHandDistances, const std::vector<double>& leftHandDistances) {
    std::string filename = "Functional_Reach_Test_Results.csv";
    std::ifstream infile(filename);
    std::ofstream outfile;

    // Check if the file exists
    bool fileExists = infile.good();
    infile.close();

    // Read the current header to determine the number of columns
    std::vector<std::string> existingHeader;
    if (fileExists) {
        std::ifstream infile(filename);
        std::string line;
        std::getline(infile, line);
        std::istringstream ss(line);
        std::string col;
        while (std::getline(ss, col, ',')) {
            existingHeader.push_back(col);
        }
        infile.close();
    }

    // Determine the maximum number of columns needed
    size_t numColumnsNeeded = std::max(rightHandDistances.size(), leftHandDistances.size());
    if (!existingHeader.empty()) {
        numColumnsNeeded = std::max(numColumnsNeeded, existingHeader.size() / 2); // Each column pair is for right and left hands
    }

    // Open the file in append mode
    outfile.open(filename, std::ios::app);

    // Write the header if the file is new
    if (!fileExists) {
        for (size_t i = 0; i < numColumnsNeeded; ++i) {
            if (i > 0) outfile << ",";
            outfile << "Functional Reach Test (m) Right Hand " << (i + 1) << ",";
            outfile << "Functional Reach Test (m) Left Hand " << (i + 1);
        }
        outfile << "\n";
    }

    // Write the new test distances in a new row
    for (size_t i = 0; i < numColumnsNeeded; ++i) {
        if (i > 0) outfile << ",";
        if (i < rightHandDistances.size()) {
            outfile << rightHandDistances[i];
        }
        outfile << ",";
        if (i < leftHandDistances.size()) {
            outfile << leftHandDistances[i];
        }
    }
    outfile << "\n";
    outfile.close();
}


// Constants for stability detection
const int stabilityFramesThreshold = 20; // Number of frames to check for stability
const float stabilityYThreshold = 0.02f; // Y-coordinate fluctuation threshold for stability

float initialLeftHandZ = -1.0f, initialRightHandZ = -1.0f;
float initialLeftElbowZ = -1.0f, initialRightElbowZ = -1.0f;

// Deques to store Y-coordinate history for stability detection
std::deque<float> leftHandYHistory, rightHandYHistory, leftElbowYHistory, rightElbowYHistory;

float lastLeftHandY = -1.0f, lastRightHandY = -1.0f;
float lastLeftElbowY = -1.0f, lastRightElbowY = -1.0f;

int stabilityFrames = 0; // To track how many frames hands are stable
float currentRightHandZ = 0.0f, currentLeftHandZ = 0.0f; // To store the current Z coordinates of the hands
float DistanceRightHand = 0.0f;
float DistanceLeftHand = 0.0f;
float MaximumRightHandDistance = 0.0f;
float MaximumLeftHandDistance = 0.0f;

//Non raised elbow Y axis coordinates
float nonRaisedElbowRightY = 0.0f;
float nonRaisedElbowLeftY = 0.0f;
//arms raised threshold for both hands
float armsRaisedThresholdRight = 0.05f;
float armsRaisedThresholdLeft = 0.05f;
//arms in line with elbow threshold
float armsinlinewithelbow = 0.15f;

// Z axis right and left hand threshold
float ThresholdZ = 0.05f;

//Right hand threshold
float RightHandThreshold = 0.1f;

//left Hand Threshold
float LeftHandThreshold = 0.1f;

//initial x and y coordinate variable for right hand
float initialRightHandX = 0.0f;
float initialRightHandY = 0.0f;

//initial x and y coordinate variable for left hand
float initialLeftHandX = 0.0f;
float initialLeftHandY = 0.0f;

//initial Position Retained Threshold, while retaining initial position, the arms should be within 15cm in range of initial coordinates
float initialPositionHandsRetained = 0.15f;
//standing still with nonraised arms threshold for both hands
float nonRaisedArmsStandingStillFinalThreshold = 0.2f;

//Flags for the Functional Reach Test
bool armsStable = false;               //Arms are Stable in any position
bool armsRaised = false;               //Arms are Raised
bool FinalMaximumDistance = false;     //Final Maximum Distance Achieved
bool armsStablePrinted = false;        // Flag to print once when arms are stable
bool testCompleted = false;            //Test Completed
bool testStarted = false;              //Test Started
bool initialPositionRetained = false;

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

    cv::namedWindow("Kinect Skeleton", cv::WINDOW_AUTOSIZE);

    bool messagePrinted = false; // Flag to track if message has been printed

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
                                // Print messages only once when a person is detected for the first time

                                Joint joints[JointType_Count];
                                body->GetJoints(_countof(joints), joints);

                                float leftHandY = 0, rightHandY = 0, leftElbowY = 0, rightElbowY = 0;

                                // Only draw circles for the left and right hand joints
                                for (int j = 0; j < JointType_Count; j++) {
                                    if (joints[j].TrackingState == TrackingState_Tracked &&
                                        (j == JointType_HandLeft || j == JointType_HandRight || j == JointType_ElbowLeft || j == JointType_ElbowRight)) {

                                        ColorSpacePoint colorPoint;
                                        coordinateMapper->MapCameraPointToColorSpace(joints[j].Position, &colorPoint);

                                        int x = static_cast<int>(colorPoint.X);
                                        int y = static_cast<int>(colorPoint.Y);
                                        float z = joints[j].Position.Z; // This gives the Z coordinate of the hand or elbow

                                        if (j == JointType_HandLeft) {
                                            leftHandY = joints[j].Position.Y;
                                            // Save the initial Z coordinate for the left hand
                                            if (initialLeftHandZ == -1.0f) {
                                                initialLeftHandZ = z;
                                            }
                                        }
                                        else if (j == JointType_HandRight) {
                                            rightHandY = joints[j].Position.Y;
                                            // Save the initial Z coordinate for the right hand
                                            if (initialRightHandZ == -1.0f) {
                                                initialRightHandZ = z;
                                            }
                                        }
                                        else if (j == JointType_ElbowLeft) {
                                            leftElbowY = joints[j].Position.Y;
                                            // Save the initial Z coordinate for the left elbow
                                            if (initialLeftElbowZ == -1.0f) {
                                                initialLeftElbowZ = z;
                                            }
                                        }
                                        else if (j == JointType_ElbowRight) {
                                            rightElbowY = joints[j].Position.Y;
                                            // Save the initial Z coordinate for the right elbow
                                            if (initialRightElbowZ == -1.0f) {
                                                initialRightElbowZ = z;
                                            }
                                        }

                                        if (x >= 0 && x < width && y >= 0 && y < height) {
                                            cv::circle(bgrMat, cv::Point(x, y), 10, cv::Scalar(0, 0, 255), -1); // Draw a circle with radius 10

                                            // Add text label next to the hand joints
                                            if (j == JointType_HandLeft) {
                                                cv::putText(bgrMat, "Left Hand", cv::Point(x + 10, y), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2);
                                            }
                                            else if (j == JointType_HandRight) {
                                                cv::putText(bgrMat, "Right Hand", cv::Point(x + 10, y), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2);
                                            }

                                            // Display the coordinates next to the joint (X, Y, Z values)
                                            cv::putText(bgrMat, "X: " + std::to_string(x) + " Y: " + std::to_string(y) + " Z: " + std::to_string(z),
                                                cv::Point(x + 10, y + 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
                                        }
                                    }
                                }


                                // Update history
                                if (leftHandYHistory.size() >= stabilityFramesThreshold) leftHandYHistory.pop_front();
                                if (rightHandYHistory.size() >= stabilityFramesThreshold) rightHandYHistory.pop_front();
                                if (leftElbowYHistory.size() >= stabilityFramesThreshold) leftElbowYHistory.pop_front();
                                if (rightElbowYHistory.size() >= stabilityFramesThreshold) rightElbowYHistory.pop_front();

                                leftHandYHistory.push_back(leftHandY);
                                rightHandYHistory.push_back(rightHandY);
                                leftElbowYHistory.push_back(leftElbowY);
                                rightElbowYHistory.push_back(rightElbowY);

                                // Check stability
                                bool leftHandStable = isStable(leftHandYHistory, stabilityYThreshold);
                                bool rightHandStable = isStable(rightHandYHistory, stabilityYThreshold);
                                bool leftElbowStable = isStable(leftElbowYHistory, stabilityYThreshold);
                                bool rightElbowStable = isStable(rightElbowYHistory, stabilityYThreshold);

                                //Conditionl Statement When Arms are straight and stable, Message Printed flag is set to indicate that Test ready
                                if (leftHandStable && rightHandStable && leftElbowStable && rightElbowStable && !messagePrinted &&
                                    joints[JointType_HandRight].Position.X - joints[JointType_ElbowRight].Position.X < armsinlinewithelbow &&
                                    joints[JointType_HandLeft].Position.X - joints[JointType_ElbowLeft].Position.X < armsinlinewithelbow)

                                    //print test ready message when arms are stable and initial Y coordinates for elbows are printed
                                {
                                    messagePrinted = true; // Set the flag to true to prevent repeated printing

                                    nonRaisedElbowRightY = joints[JointType_ElbowRight].Position.Y;
                                    nonRaisedElbowLeftY = joints[JointType_ElbowLeft].Position.Y;
                                    
                                    std::cout << "Test Ready" << std::endl;
                                    std::cout << "Please Raise both of your arms" << std::endl;
                                    
                                    std::cout << "Initial Right Elbow Y Coordinates are: " << nonRaisedElbowRightY << std::endl;
                                    std::cout << "Initial Left Elbow Y Coordinates are: " << nonRaisedElbowLeftY << std::endl;
                                }
                              
                                //Put Text on the Screen, when arms are stable, message is printed that Test is Ready
                                if (messagePrinted && !testStarted)
                                {
                                    cv::putText(bgrMat, "Test Ready", cv::Point(50,50), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 255), 2);

                                }

                                //Conditional Statement, To Print the coordinates of arms when raised. Coordinates are printed in tabular format
                                //Arms Raised Flag is set however test is not yet started
                                if (joints[JointType_ElbowRight].Position.Y - joints[JointType_HandRight].Position.Y < armsRaisedThresholdRight &&
                                    joints[JointType_ElbowLeft].Position.Y - joints[JointType_HandLeft].Position.Y < armsRaisedThresholdLeft &&
                                    leftHandStable && rightHandStable && leftElbowStable && rightElbowStable && messagePrinted && !armsRaised && !testStarted)
                                {
                                    //setting the flag to true that the arms were raised
                                    armsRaised = true;
                                    armsStablePrinted = true; // Prevent printing again

                                    std::cout << "Arms Raised" << std::endl;
                                    std::cout << "Please keep your arms raised" << std::endl;
                                    
                                    //storing the Right Hand X,Y,Z coordinates
                                    initialRightHandZ = joints[JointType_HandRight].Position.Z;
                                    initialRightHandY = joints[JointType_HandRight].Position.Y;
                                    initialRightHandX = joints[JointType_HandRight].Position.X;
                                    //Storing the Left Hand X,Y,Z Coordinates
                                    initialLeftHandZ = joints[JointType_HandLeft].Position.Z;
                                    initialLeftHandY = joints[JointType_HandLeft].Position.Y;
                                    initialLeftHandX = joints[JointType_HandLeft].Position.X;

                                    std::cout << "Right Hand Coordinates | x: " << joints[JointType_HandRight].Position.X << " | y: " << rightHandY << " | z: " << joints[JointType_HandRight].Position.Z << " |" << std::endl;
                                    std::cout << "Right Elbow Coordinates| x: " << joints[JointType_ElbowRight].Position.X << " | y: " << rightElbowY << " | z: " << joints[JointType_ElbowRight].Position.Z << " |" << std::endl;
                                    std::cout << "Left Hand Coordinates  | x: " << joints[JointType_HandLeft].Position.X << "  | y: " << leftHandY << " | z: " << joints[JointType_HandLeft].Position.Z << " |" << std::endl;
                                    std::cout << "Left Elbow Coordinates | x: " << joints[JointType_ElbowLeft].Position.X << "  | y: " << leftElbowY << " | z: " << joints[JointType_ElbowLeft].Position.Z << " |" << std::endl;
                                    
                                    std::cout << "Initial Right Hand Z Coordinates are: " << initialRightHandZ << std::endl;
                                    std::cout << "Initial Left Hand Z Coordinates are: " << initialLeftHandZ << std::endl;

                                    //print message to bend forawrd
                                    std::cout << "Please bend forward" << std::endl;
                                }
                                
                                //Put text to display that the arms were raised and display message to bend forward
                                if (armsRaised && armsStablePrinted && !testStarted)
                                {
                                    cv::putText(bgrMat, "Arms Raised", cv::Point(50, 100), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 255), 2);
                                    //display message on screen in yellow to Bend Forward
                                    cv::putText(bgrMat, "Please stay still for a while, then Bend Forward as much as you can", cv::Point(50, 150), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 255), 2);
                                }


                                //Now to calcuate distance covered by hands when bend forward
                                if ((initialRightHandZ - joints[JointType_HandRight].Position.Z > ThresholdZ) &&
                                    (initialLeftHandZ - joints[JointType_HandLeft].Position.Z > ThresholdZ) &&
                                    (std::abs(joints[JointType_HandRight].Position.X - initialRightHandX) < RightHandThreshold) &&
                                    (std::abs(joints[JointType_HandRight].Position.Y - initialRightHandY) < RightHandThreshold) &&
                                    (std::abs(joints[JointType_HandLeft].Position.X - initialLeftHandX) < LeftHandThreshold) &&
                                    (std::abs(joints[JointType_HandLeft].Position.Y - initialLeftHandY) < LeftHandThreshold) && armsRaised && !testStarted)

                                {
                                    //set the flag of test Started to true
                                    testStarted = true;
                                    
                                    // Calculate the distance of the hands from their initial positions
                                    currentRightHandZ = joints[JointType_HandRight].Position.Z;
                                    currentLeftHandZ = joints[JointType_HandLeft].Position.Z;
                                    DistanceRightHand = std::abs(initialRightHandZ - currentRightHandZ);
                                    DistanceLeftHand = std::abs(initialLeftHandZ - currentLeftHandZ);

                                    std::cout << "Right Hand Distance: " << (DistanceRightHand) * 100.0f << " centimeters" << std::endl;
                                    std::cout << "Left Hand Distance: " << DistanceLeftHand * 100.0f << " centimeters" << std::endl;
                                    
                                    if (MaximumRightHandDistance < DistanceRightHand)
                                    {
                                        MaximumRightHandDistance = DistanceRightHand;
                                    }
                                    if (MaximumLeftHandDistance < DistanceLeftHand)
                                    {
                                        MaximumLeftHandDistance = DistanceLeftHand;
                                    }

                                    if ((DistanceRightHand - MaximumRightHandDistance < 0.0f) && (DistanceLeftHand-  MaximumLeftHandDistance <   0.0f))
                                    {
                                        FinalMaximumDistance = true;
                                        
                                    }
                                    
                                }

                                // Conditional to print test started and to display maximum distance arms can travel
                                if (armsRaised && testStarted && !initialPositionRetained && !testCompleted)
                                {
                                    //display test Started
                                    cv::putText(bgrMat, "Test Started", cv::Point(50, 50), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 255), 2);
                                    //display Right Hand Distance on Live Feed
                                    cv::putText(bgrMat, "Right Hand Distance: " + std::to_string(DistanceRightHand * 100.0f) + " cm",
                                        cv::Point(50, 100), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 255), 2);
                                    //display left Hand Distance on Live Feed
                                    cv::putText(bgrMat, "Left Hand Distance: " + std::to_string(DistanceLeftHand * 100.0f) + " cm",
                                        cv::Point(50, 150), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 255), 2);
                                }

                                //display text to order to go back to initial position once final maximum distance is achieved
                                if (testStarted && FinalMaximumDistance && !initialPositionRetained && !testCompleted)
                                {
                                    cv::putText(bgrMat, "You Have Reached your limit, Please go back to your initial Position now", cv::Point(50, 50), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 255), 2);

                                }

                                //Conditional to check if person has achieved the initial Position again or not
                                if (initialRightHandZ - joints[JointType_HandRight].Position.Z <= initialPositionHandsRetained  &&
                                    initialLeftHandZ - joints[JointType_HandLeft].Position.Z <= initialPositionHandsRetained &&
                                    FinalMaximumDistance && testStarted && !initialPositionRetained && !testCompleted)
                                {
                                    initialPositionRetained = true;
                                    cout << "Please Put your hands down now" << endl;

                                }
                                //conditional to print this instruction on the screen too
                                if (testStarted && initialPositionRetained && !testCompleted)
                                {
                                    cv::putText(bgrMat, "Now Please Put your Hands Down, to complete the test", cv::Point(50, 50), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 255), 2);
                                }
                                //conditional to conclude the test finally, if hands are in position near standing still position
                                if (joints[JointType_HandRight].Position.Y - nonRaisedElbowRightY <= nonRaisedArmsStandingStillFinalThreshold &&
                                    joints[JointType_HandLeft].Position.Y - nonRaisedElbowRightY <= nonRaisedArmsStandingStillFinalThreshold  &&
                                    testStarted && initialPositionRetained && !testCompleted)
                                {
                                    testCompleted = true;
                                    //display the final readings for both hands
                                    cout << "Distance Reached by Right Hand: " << MaximumRightHandDistance << endl;
                                    cout << "Distance Reached by Left Hand: " << MaximumLeftHandDistance << endl;
                                    cout << "Test Completed!" << endl;

                                    //data log these readings in csv file
                                    logFunctionalReachTest(std::vector<double>{MaximumRightHandDistance}, std::vector<double>{MaximumLeftHandDistance});

                                }

                                //display Test Completed on Live Feed
                                if (testCompleted)
                                {
                                    cv::putText(bgrMat, "Test Completed!", cv::Point(50, 50), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 255), 2);

                                }


                            }
                        }
                    }

                    bodyFrame->Release();
                }
                cv::imshow("Kinect Skeleton", bgrMat);
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
