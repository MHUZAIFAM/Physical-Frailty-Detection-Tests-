#include <iostream>
#include <fstream>
#include <vector>
#include <sstream>
#include <string>

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

int main() {
    // Example: Simulate test distances for different sessions
    std::vector<double> rightHandDistances1 = { 0.85, 0.92, 0.88 }; // First session, right hand
    std::vector<double> leftHandDistances1 = { 0.82, 0.90, 0.87 };  // First session, left hand
    logFunctionalReachTest(rightHandDistances1, leftHandDistances1);

    std::vector<double> rightHandDistances2 = { 0.91, 0.93 }; // Second session, right hand
    std::vector<double> leftHandDistances2 = { 0.89, 0.88 };  // Second session, left hand
    logFunctionalReachTest(rightHandDistances2, leftHandDistances2);

    std::vector<double> rightHandDistances3 = { 0.94, 0.95, 0.96, 0.97 }; // Third session, right hand
    std::vector<double> leftHandDistances3 = { 0.92, 0.93, 0.94, 0.95 };  // Third session, left hand
    logFunctionalReachTest(rightHandDistances3, leftHandDistances3);

    return 0;
}

