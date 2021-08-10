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
#include "ctello.h"
#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include <unistd.h>	
#include<opencv2/core/core.hpp>
#include <thread>
#include<System.h>
#include <Converter.h>

using namespace std;
cv::Mat frame;
std::string const TELLO_STREAM_URL{"udp://0.0.0.0:11111"};
cv::VideoCapture capture{TELLO_STREAM_URL, cv::CAP_FFMPEG};
ctello::Tello tello{};
volatile int task2_finished=0;
void task1()
{
    while (true){
        capture >> frame;
        sleep(0.1);
    }
}
void task2()
{
    int i=0;
        for(i=0;i<3;i++)
        {
            tello.SendCommand("cw 20");
            sleep(3);
            tello.SendCommand("up 20");
            sleep(3);
            tello.SendCommand("down 20");
            sleep(3);
        }
        tello.SendCommand("land");
        task2_finished=100;
}
void LoadImages(const string &strFile, vector<string> &vstrImageFilenames,
                vector<double> &vTimestamps);
void saveMap(ORB_SLAM2::System& SLAM);


int main(int argc, char **argv)
{
int count=0;

    if (!tello.Bind())
    {
        return 0;
    }

    tello.SendCommand("streamon");
    while (!(tello.ReceiveResponse()));
    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);
    tello.SendCommand("takeoff");
    capture >> frame;
    thread t2(task2);
    thread t1(task1);

    while (task2_finished==0)
    {

        if (!frame.empty())
        {
            cv::resize(frame, frame, cv::Size(960, 720));
            SLAM.TrackMonocular(frame,count++);
        }
    }
    cout<<"finished"<<endl;
    // Stop all threads
    SLAM.Shutdown();
    saveMap(SLAM);
    t1.join();



    if(argc != 4)
    {
        cerr << endl << "Usage: ./mono_tum path_to_vocabulary path_to_settings path_to_sequence" << endl;
        return 1;
    }

    // Retrieve paths to images
    vector<string> vstrImageFilenames;
    vector<double> vTimestamps;
    string strFile = string(argv[3])+"/rgb.txt";
    LoadImages(strFile, vstrImageFilenames, vTimestamps);

    int nImages = vstrImageFilenames.size();


    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;

    // Main loop
    cv::Mat im;
    for(int ni=0; ni<nImages; ni++)
    {
        // Read image from file
        im = cv::imread(string(argv[3])+"/"+vstrImageFilenames[ni],CV_LOAD_IMAGE_UNCHANGED);
        double tframe = vTimestamps[ni];

        if(im.empty())
        {
            cerr << endl << "Failed to load image at: "
                 << string(argv[3]) << "/" << vstrImageFilenames[ni] << endl;
            return 1;
        }
//
//#ifdef COMPILEDWITHC11
//        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
//#else
//        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
//#endif

        // Pass the image to the SLAM system
        SLAM.TrackMonocular(im,tframe);
//
//#ifdef COMPILEDWITHC11
//        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
//#else
//        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
////#endif
//
//        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
//
//        vTimesTrack[ni]=ttrack;
//
//        // Wait to load the next frame
//        double T=0;
//        if(ni<nImages-1)
//            T = vTimestamps[ni+1]-tframe;
//        else if(ni>0)
//            T = tframe-vTimestamps[ni-1];
//
//        if(ttrack<T)
//            usleep((T-ttrack)*1e6);
  }




    // Tracking time statistics
    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    for(int ni=0; ni<nImages; ni++)
    {
        totaltime+=vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
    cout << "mean tracking time: " << totaltime/nImages << endl;

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    return 0;
}

void saveMap(ORB_SLAM2::System& SLAM){
    std::vector<ORB_SLAM2::MapPoint*> mapPoints = SLAM.GetMap()->GetAllMapPoints();
    std::ofstream pointData;
    pointData.open("/home/gal/Downloads/pointData.csv");
    for(auto p : mapPoints) {
        if (p != NULL)
        {
            auto point = p->GetWorldPos();
            Eigen::Matrix<double, 3, 1> v = ORB_SLAM2::Converter::toVector3d(point);
            pointData << v.x() << "," << v.y() << "," << v.z()<<  std::endl;
        }
    }
    cout<<"map saved to Downloads/pointData"<<endl;
    pointData.close();
}
void LoadImages(const string &strFile, vector<string> &vstrImageFilenames, vector<double> &vTimestamps)
{
    ifstream f;
    f.open(strFile.c_str());

    // skip first three lines
    string s0;
    getline(f,s0);
    getline(f,s0);
    getline(f,s0);

    while(!f.eof())
    {
        string s;
        getline(f,s);
        if(!s.empty())
        {
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
