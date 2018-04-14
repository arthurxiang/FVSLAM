/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<iomanip>

#include<opencv2/opencv.hpp>

#include"System.h"
#include "SLAM_BIT.h"

using namespace std;
using namespace cv;

void LoadImages(const string &strPathToSequence, vector<imgdata> &imglist);

int main(int argc, char **argv) {
    if (argc != 4) {
        cerr << endl << "Usage: ./mono_bit path_to_vocabulary path_to_settings path_to_sequence" << endl;
        return 1;
    }
    // Retrieve paths to images
    vector<imgdata> vImgList;
    Mat K;
    LoadImages(string(argv[3]), vImgList);

    int nImages = vImgList.size();

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1], argv[2], ORB_SLAM2::System::MONOCULAR, true);

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;

    // Main loop
    cv::Mat im;
    double tframe = 0;
    for (int ni = 0; ni < nImages; ni++) {
        // Read image from file
        im = cv::imread(vImgList[ni].file, CV_LOAD_IMAGE_UNCHANGED);
        tframe += vImgList[ni].tspan;

        if (im.empty()) {
            cerr << endl << "Failed to load image at: " << vImgList[ni].file << endl;
            return 1;
        }

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        // Pass the image to the SLAM system
        SLAM.TrackMonocularWithFD(im, tframe, vImgList[ni].Data);

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

        double ttrack = std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        vTimesTrack[ni] = ttrack;

        // Wait to load the next frame
        double T = vImgList[ni].tspan;

        if (ttrack < T)
            usleep((T - ttrack) * 1e3);
    }

    // Stop all threads
    SLAM.Shutdown();

    // Tracking time statistics
    sort(vTimesTrack.begin(), vTimesTrack.end());
    float totaltime = 0;
    for (int ni = 0; ni < nImages; ni++) {
        totaltime += vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages / 2] << endl;
    cout << "mean tracking time: " << totaltime / nImages << endl;

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    return 0;
}

void LoadImages(const string &strPathToSequence, vector<imgdata> &imglist) {

    FileStorage fs;
    string strPathTimeFile = strPathToSequence + "/data.xml";
    fs.open(strPathTimeFile, FileStorage::READ);
    int firstno, lastno;
    fs["FirstNo"] >> firstno;
    fs["LastNo"] >> lastno;
    //firstno=8000;
    //lastno=10700;
    stringstream ss;
    for (int j = firstno; j < lastno; ++j) {
        Mat m;
        ss.str("");
        ss << "img" << j;
        fs[ss.str()] >> m;
        ss.str("");
        ss << strPathToSequence << "/img" << m.at<double>(0) << ".jpg";
        imgdata d;
        d.file = ss.str();
        d.tspan = m.at<double>(1);
        d.Data.pitch = m.at<double>(2);
        d.Data.roll = m.at<double>(3);
        d.Data.yaw = m.at<double>(4);
        d.Data.lng = m.at<double>(5);
        d.Data.lat = m.at<double>(6);
        d.Data.alt = m.at<double>(7);
        imglist.push_back(d);
    }
}
