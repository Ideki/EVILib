// evilib.c++
//
// Simple test program for the EVILib lib
//
// Author: Pic Mickael, AIST Japan, 2003.
// email: mickael.pic@aist.go.jp

#include "evilib.h"

int main(int argc, char *argv[])
{
    int cam_status = 0; // Camera status
    EVILib cam1;       // Allow access to the camera
    int n = 0;          // Return value of the functions
    float x, y, z;      // Axis value
    int val;            // Return value of the program

#if 1
    int i, d;
    double m_x, m_y;
#endif

#if 0
    timeval *alp = new timeval;
    timeval *bet = new timeval;
#endif

    int ExpEnd = 100;

//---------------------------

    cam_status = 0;
// Open serial port
    if(cam1.Init() != 1)
        goto bail;
    if(cam1.Open(1, "/dev/ttyS0") != 1)
        goto bail;

    cam_status = 1;

// We do a power ON only if the camera is not already ON.
    n = cam1.PowerInq();
    if(n == -1)
        goto bail;
    else
        if(n == EVILIB_OFF)
        {
            cout<<"Turning ON power\n";
            if(cam1.Power(EVILIB_ON) != 1)
                goto bail;
        }
    cout<<"Camera ready\n";

// Put the camera to the 'home' position. (usually 0,0,0)
    if(cam1.Home() != 1)
        goto bail;

// Get the position of the camera
// Get the x,y axis position
    if(cam1.Pan_TiltPosInq(x, y) != 1)
        goto bail;
// Get the z axis position
    if(cam1.ZoomPosInq(z) != 1)
        goto bail;

//-------------------------

// Set the autofocus to AUTO
    if(cam1.Focus(EVILIB_AUTO, 0, EVILIB_WAIT_COMP) != 1)
        goto bail;

// Set exposure mode to BRIGHT
    if(cam1.AE(EVILIB_BRIGHT, EVILIB_WAIT_COMP) != 1)
        goto bail;

//--------------------------

#if 0
//---------------
// The camera start to go to the first position. But as soon as the second command
//  is sent, the camera go to the new position...

    if(cam1.Pan_TiltDrive(EVILIB_ABSOLUTE, EVILIB_max_pspeed, EVILIB_max_tspeed, -100, -100, EVILIB_NO_WAIT_COMP) != 1)
        goto bail;
    if(cam1.Pan_TiltDrive(EVILIB_ABSOLUTE, EVILIB_max_pspeed, EVILIB_max_tspeed, 100, 100, EVILIB_WAIT_COMP) != 1)
        goto bail
    while(ExpEnd != 1);
#endif



#if 0
//--------------------
// Test for speed of command

        if(cam1.Pan_TiltDrive(EVILIB_ABSOLUTE, EVILIB_max_pspeed, EVILIB_max_tspeed, -100, -30, EVILIB_WAIT_COMP) != 1)
            goto bail;
        if(cam1.Pan_TiltPosInq(x, y) == 0)
            goto bail;
        cout<<x<<" "<<y<<endl;
        gettimeofday(alp, NULL);
        if(cam1.Pan_TiltDrive(EVILIB_ABSOLUTE, EVILIB_max_pspeed, EVILIB_max_tspeed, 100, 30, EVILIB_WAIT_COMP) != 1)
            goto bail;
        gettimeofday(bet, NULL);
        cout<<(bet->tv_sec * 1000000 + bet->tv_usec) - (alp->tv_sec * 1000000 + alp->tv_usec)<<endl;
        if(cam1.Pan_TiltPosInq(x, y) == 0)
            goto bail;
        cout<<x<<" "<<y<<endl;

        while(ExpEnd != 1);
#endif

#if 1
//-----------------------
// Move the camera using a sin() path

    i = 30;
    m_x = 90;
    m_y = 30;

    d = 0;
    x = - m_x;

    if(cam1.Zoom(EVILIB_DIRECT, EVILIB_max_ZoomSpeed, 0, EVILIB_WAIT_COMP) != 1)
        goto bail;

    while(ExpEnd)
    {
        y = sin(x  * (M_PI / 180.0)) * m_y;
        if(d == 0)
        {
            if(cam1.Pan_TiltDrive(EVILIB_ABSOLUTE, EVILIB_max_pspeed, EVILIB_max_tspeed, x, y, EVILIB_NO_WAIT_COMP) != 1)
                goto bail;
            x += i;
            if(x >= m_x)
            {
                d = 1;
                x = m_x;
            }
        }
        else
        {
            if(cam1.Pan_TiltDrive(EVILIB_ABSOLUTE, EVILIB_max_pspeed, EVILIB_max_tspeed, x, -y, EVILIB_NO_WAIT_COMP) != 1)
                goto bail;
            x -= i;
            if(x <= - m_x)
            {
                d = 0;
                x = - m_x;
            }
        }
	ExpEnd --;
    };
#endif

#if 0
//---------------
// Set the zoom,

    z = 5.0;
    if(cam1.Zoom(EVILIB_DIRECT, EVILIB_max_ZoomSpeed, z, EVILIB_WAIT_COMP) != 1)
        goto bail;
    while(ExpEnd != 1);

#endif

//------------

    val = 1;
    goto bail2;
 bail:
    cerr<<"Error Main Program\n";
    val = 0;
 bail2:
    if(cam_status == 1)
    {
        if(cam1.Power(EVILIB_OFF) != 1)
            cerr<<"Error Power Off\n";

        cam1.Close();
    }

//-------------

    return val;
}
