//
// Created by arthurxiang on 17-8-5.
//

#ifndef XLIBSHOTER_XLIBSHOTER_H
#define XLIBSHOTER_XLIBSHOTER_H

#include <opencv2/opencv.hpp>  // This includes most headers!

#include <X11/Xlib.h>
#include <X11/Xutil.h>

#include <X11/extensions/XShm.h>
#include <sys/ipc.h>
#include <sys/shm.h>



class ScreenShot{
public:
    ScreenShot(uint x, uint y, uint width, uint height):
            x(x), y(y), width(width), height(height){

        display = XOpenDisplay(nullptr);
        root = DefaultRootWindow(display);

        XGetWindowAttributes(display, root, &window_attributes);
        screen = window_attributes.screen;
        ximg = XShmCreateImage(display, DefaultVisualOfScreen(screen), (uint)DefaultDepthOfScreen(screen), ZPixmap, NULL, &shminfo, width, height);

        shminfo.shmid = shmget(IPC_PRIVATE, (size_t)ximg->bytes_per_line * ximg->height, IPC_CREAT|0777);
        shminfo.shmaddr = ximg->data = (char*)shmat(shminfo.shmid, 0, 0);
        shminfo.readOnly = False;
        if(shminfo.shmid < 0)
            puts("Fatal shminfo error!");;
        Status s1 = XShmAttach(display, &shminfo);
        printf("XShmAttach() %s\n", s1 ? "success!" : "failure!");

        init = true;
    }

    void operator() (cv::Mat& cv_img){
        if(init)
            init = false;

        XShmGetImage(display, root, ximg, x, y, 0x00ffffff);
        cv_img = cv::Mat(height, width, CV_8UC4, ximg->data);
    }

    ~ScreenShot(){
        if(!init)
            XDestroyImage(ximg);

        XShmDetach(display, &shminfo);
        shmdt(shminfo.shmaddr);
        XCloseDisplay(display);
    }

private:
    Display* display;
    Window root;
    XWindowAttributes window_attributes;
    Screen* screen;
    XImage* ximg;
    XShmSegmentInfo shminfo;

    int x, y, width, height;

    bool init;
};

#endif //XLIBSHOTER_XLIBSHOTER_H
