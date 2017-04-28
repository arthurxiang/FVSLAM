//
// Created by arthurxiang on 17-4-21.
//
#include <string>
#ifndef ORB_SLAM2_SLAM_BIT_H_H
#define ORB_SLAM2_SLAM_BIT_H_H

    typedef struct _flightdata
    {
        /* imgid
         * tspan
         * pitch
         * roll
         * yaw
         * lat
         * lng
         * alt
         * */
        float pitch;
        float roll;
        float yaw;
        float lat;
        float lng;
        float alt;
    }flightdata;
    typedef struct _imgdata
    {
        std::string file;
        int tspan;
        flightdata Data;
    }imgdata;

#endif //ORB_SLAM2_SLAM_BIT_H_H
