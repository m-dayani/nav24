//
// Created by masoud on 2/8/24.
//


#include "TabularTextDS.hpp"


using namespace std;
using namespace NAV24;


int main([[maybe_unused]] int argc, char** argv) {

    google::InitGoogleLogging(argv[0]);
    google::InstallFailureSignalHandler();

    if (argc < 2) {
        cout << "Usage: tabular_text_ds base_dir\n";
        return 1;
    }

    // Image files

    // Image dir
    shared_ptr<TabularTextDS> pImageDs = make_shared<TabularTextDS>(argv[1], "", ".png");
    string nextFile = pImageDs->getNextFile();
    while(!nextFile.empty()) {
        cout << nextFile << "\n";
        nextFile = pImageDs->getNextFile();
    }

    // Image file .txt (TUM-RGBD Dataset)
    pImageDs = make_shared<TabularTextDS>(argv[1], "rgb.txt");
    pImageDs->open();
    nextFile = pImageDs->getNextLine();
    while (!nextFile.empty()) {
        cout << nextFile << "\n";
        nextFile = pImageDs->getNextLine();
    }
    pImageDs->close();

    // Or you can get the data
    pImageDs->reset();
    vector<string> vData;
    ulong n_data = pImageDs->getNextData(vData);
    while (n_data > 0) {
        for (const auto& d : vData) {
            cout << d << " - ";
        }
        cout << "\n";
        vData.clear();
        n_data = pImageDs->getNextData(vData);
    }
    pImageDs->close();

    return 0;
}