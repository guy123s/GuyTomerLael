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


/*
tello arguments:
 /home/tgl/ORB_SLAM2/Vocabulary/ORBvoc.txt /home/tgl/Desktop/tello_videos/cheetu2.yaml /home/tgl/Desktop/tello_videos/vid3
custom video arguments:
 /home/tgl/ORB_SLAM2/Vocabulary/ORBvoc.txt /home/tgl/Desktop/tello_videos/cheetu2.yaml /home/tgl/Desktop/tello_videos/vid1
*/

#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<opencv2/core/core.hpp>

//#include "~/ORB_SLAM2/include/System.h"
#include "System.h"
#include <ctello.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>

const int FRAME_NUMBER{30};
const char* const TELLO_STREAM_URL{"udp://0.0.0.0:11111?overrun_nofatal=1&fifo_size=500000"};

using ctello::Tello;
using cv::CAP_FFMPEG;
using cv::imshow;
using cv::VideoCapture;
using cv::waitKey;

using namespace std;
using namespace ORB_SLAM2;
void LoadImages(const string &strFile, vector<string> &vstrImageFilenames,
                vector<double> &vTimestamps);

inline void TELLO(Tello &t, const string &s) {
    t.SendCommand(s);
    while(!(t.ReceiveResponse())) ;
}

void saveMap(ORB_SLAM2::System &SLAM){
    std::vector<ORB_SLAM2::MapPoint*> mapPoints = SLAM.GetMap()->GetAllMapPoints();
    std::ofstream pointData;
    pointData.open("/home/tgl/Desktop/points/data1.csv");
    for(auto p : mapPoints) {
        if (p != nullptr) {
            auto point = p->GetWorldPos();
            Eigen::Matrix<double, 3, 1> v = ORB_SLAM2::Converter::toVector3d(point);
            pointData << v.x() << "," << v.y() << "," << v.z()<<  std::endl;
            for (int j=0; j<3; j++) {
                pointData << (point.at<double>(0, j)) << "\t";   // maybe should be (j,0)? was (0,0)
                pointData << (point.at<double>(j, 0)) << "\t";   // maybe should be (j,0)? was (0,0)
                pointData << endl;
            }
            pointData << std::endl;
        }
    }
    pointData.close();
}

int main(int argc, char **argv) {

    if(argc != 4) {
        cerr << endl << "Usage: ./mono_tum path_to_vocabulary path_to_settings path_to_sequence" << endl;
        return 1;
    }

//    Tello tello{};
//    if (!tello.Bind()) return 0;

    // Retrieve paths to images
    vector<string> vstrImageFilenames;
    vector<double> vTimestamps;
    string strFile = string(argv[3])+"/rgb.txt";
    LoadImages(strFile, vstrImageFilenames, vTimestamps);

    int nImages = vstrImageFilenames.size();

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

//    TELLO(tello, "streamon");
//    VideoCapture capture{TELLO_STREAM_URL, cv::CAP_ANY};
//    capture.set(cv::CAP_PROP_BUFFERSIZE, 30);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;

    // Main loop
    cv::Mat im;
//    TELLO(tello, "takeoff");
    for(int ni=0; ni<nImages; ni++) {;
//        TELLO(tello, "forward 10");
        // Read image from file
        im = cv::imread(string(argv[3])+"/"+vstrImageFilenames[ni],CV_LOAD_IMAGE_UNCHANGED);

//        for (int i=0; i<FRAME_NUMBER; i++)
//            capture.grab();
//        capture >> im;
//        cv::resize(im, im, cv::Size(640, 480));
        // /for drone/
        double tframe = vTimestamps[ni];

        #ifdef COMPILEDWITHC11
            std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
        #else
            std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
        #endif

            // Pass the image to the SLAM system
        SLAM.TrackMonocular(im,ni);

        #ifdef COMPILEDWITHC11
            std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
        #else
            std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();//we made changes
        #endif
        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
        vTimesTrack[ni]=ttrack;

        // Wait to load the next frame
        double T=0;
        if(ni<nImages-1)      T = vTimestamps[ni+1]-tframe;
        else if(ni>0)      T = tframe-vTimestamps[ni-1];
        if(ttrack<T)     usleep((T-ttrack)*1e6);
    }

//    TELLO(tello, "streamoff");

    // Stop all threads
    SLAM.Shutdown();

    // Tracking time statistics
    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    for(int ni=0; ni<nImages; ni++) {
        totaltime+=vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
    cout << "mean tracking time: " << totaltime/float(nImages) << endl;

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
    saveMap(SLAM);
    return 0;
}

void LoadImages(const string &strFile, vector<string> &vstrImageFilenames, vector<double> &vTimestamps) {
    ifstream f;
    f.open(strFile.c_str());

    // skip first three lines
    string s0;
    getline(f,s0);
    getline(f,s0);
    getline(f,s0);

    while(!f.eof()) {
        string s;
        getline(f,s);
        if(!s.empty()) {
            stringstream ss;
            ss << s;
            double t;
            string sRGB;
            ss >> t;
            vTimestamps.push_back(t);
            ss >> sRGB;
            vstrImageFilenames.push_back(sRGB);
        }
    }
}

