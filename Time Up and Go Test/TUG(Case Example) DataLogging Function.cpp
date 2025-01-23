#include <iostream>
#include <fstream>
#include <vector>
#include <sstream>
#include <string>

void logTUGTestTime(const std::vector<double>& testTimes) {
    std::string filename = "TUG_Test_Results.csv";
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
    size_t numColumnsNeeded = testTimes.size();
    if (!existingHeader.empty()) {
        numColumnsNeeded = std::max(numColumnsNeeded, existingHeader.size());
    }

    // Open the file in append mode
    outfile.open(filename, std::ios::app);

    // Write the header if the file is new
    if (!fileExists) {
        for (size_t i = 0; i < numColumnsNeeded; ++i) {
            if (i > 0) outfile << ",";
            outfile << "Timed Up and Go Test (s) " << (i + 1);
        }
        outfile << "\n";
    }

    // Write the new test times in a new row
    for (size_t i = 0; i < numColumnsNeeded; ++i) {
        if (i > 0) outfile << ",";
        if (i < testTimes.size()) {
            outfile << testTimes[i];
        }
    }
    outfile << "\n";
    outfile.close();
}

int main() {
    // Example: Simulate test times for different sessions
    std::vector<double> testTimes1 = { 12.34, 10.56, 15.42 }; // First session
    logTUGTestTime(testTimes1);

    std::vector<double> testTimes2 = { 9.78, 11.22 }; // Second session
    logTUGTestTime(testTimes2);

    std::vector<double> testTimes3 = { 13.45, 12.67, 14.22, 15.50 }; // Third session with more tests
    logTUGTestTime(testTimes3);

    return 0;
}
