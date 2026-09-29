// evilib.c++
//
// Simple test program for the EVILib lib
//
// Author: Pic Mickael, AIST Japan, 2003.
// email: mickael.pic@aist.go.jp

#include "evilib.h"

int main(int argc, char *argv[])
{
  int cam_status = 0;   // Camera status
  EVI_D70 *cam1 = NULL; // Allow access to the camera
  int n = 0;            // Return value of the functions
  float x, y, z;        // Axis value
  int val;              // Return value of the program

  int data[10]; // Data for Title Set

#if 0
  int i, d;
  double m_x, m_y;
#endif

#if 0
  timeval *alp = new timeval;
  timeval *bet = new timeval;
#endif

  int ExpEnd = 100000;

//---------------------------

  cam1 = new EVI_D70();
  if(cam1 == NULL)
    {
      cerr<<"Error new cam1\n";
      goto bail;
    }

//---------------------------

  cam_status = 0;
// Open serial port
  if(cam1->Init() != 1)
    {
      cerr<<"Error cam1 Init\n";
      goto bail;
    }
  if(cam1->Open(1, "/dev/ttyS0") != 1)
    {
      cerr<<"Error cam1 Open\n";
      goto bail;
    }

  cam_status = 1;

// We do a power ON only if the camera is not already ON.
  cout<<"** PowerInq()\n";
  n = cam1->PowerInq();
  if(n == -1)
    goto bail;
  else
    if(n == EVILIB_OFF)
      {
	cout<<"Turning ON power\n";
	if(cam1->Power(EVILIB_ON) != 1)
	  goto bail;
      }
#if 1
//------------------------------------------
// This MUST be done for the EVI-D70 after power(ON) (only for D70(P))
  cout<<"** DZoomCSModeInq()\n";
  cam1->DZoomCSModeInq();
//------------------------------------------
#endif

  cout<<"Camera ready\n";

// Put the camera to the 'home' position. (usually 0,0,0)
  cout<<"** Home()\n";
  if(cam1->Home() != 1)
    goto bail;

// Get the position of the camera
// Get the x,y axis position
  cout<<"** Pan_TiltPosInq()\n";
  if(cam1->Pan_TiltPosInq(x, y) != 1)
    goto bail;

// Get the z axis position
  cout<<"** ZoomPosInq()\n";
  if(cam1->ZoomPosInq(z) != 1)
    goto bail;

//-------------------------

// Set the autofocus to AUTO
  cout<<"** Focus()\n";
  if(cam1->Focus(EVILIB_AUTO, 0, EVILIB_WAIT_COMP) != 1)
    goto bail;

// Set exposure mode to BRIGHT
  cout<<"** AE()\n";
  if(cam1->AE(EVILIB_BRIGHT, EVILIB_WAIT_COMP) != 1)
    goto bail;

//--------------------------
// The camera start to go to the first position. But as soon as the second command
//  is sent, the camera go to the new position...
#if 0
  if(cam1->Pan_TiltDrive(EVILIB_ABSOLUTE, EVILIB_max_pspeed, EVILIB_max_tspeed, EVILIB_minpan, EVILIB_mintilt, EVILIB_NO_WAIT_COMP) != 1)
    goto bail;
  if(cam1->Pan_TiltDrive(EVILIB_ABSOLUTE, EVILIB_max_pspeed, EVILIB_max_tspeed, EVILIB_maxpan, EVILIB_maxtilt, EVILIB_WAIT_COMP) != 1)
    goto bail;
  while(ExpEnd != 1) ExpEnd --;
#endif

//--------------------
// Test for speed of command
#if 0
  if(cam1->Pan_TiltDrive(EVILIB_ABSOLUTE, EVILIB_max_pspeed, EVILIB_max_tspeed, EVILIB_minpan, EVILIB_mintilt, EVILIB_NO_WAIT_COMP) != 1)
    goto bail;
  if(cam1->Pan_TiltPosInq(x, y) == 0)
    goto bail;
  cout<<x<<" "<<y<<endl;
  gettimeofday(alp, NULL);
  if(cam1->Pan_TiltDrive(EVILIB_ABSOLUTE, EVILIB_max_pspeed, EVILIB_max_tspeed, EVILIB_maxpan, EVILIB_maxtilt, EVILIB_NO_WAIT_COMP) != 1)
    goto bail;
  gettimeofday(bet, NULL);
  cout<<"time: "<<(bet->tv_sec * 1000000 + bet->tv_usec) - (alp->tv_sec * 1000000 + alp->tv_usec)<<endl;
  if(cam1->Pan_TiltPosInq(x, y) == 0)
    goto bail;
  cout<<x<<" "<<y<<endl;

  while(ExpEnd != 1) ExpEnd --;
#endif

//-----------------------
// Move the camera using a sin() path
#if 0
  i = 30;
  m_x = 90;
  m_y = 30;

  d = 0;
  x = - m_x;

  cout<<"** Zoom()\n";
  if(cam1->Zoom(EVILIB_DIRECT, EVILIB_max_zspeed, 0, EVILIB_WAIT_COMP) != 1)
    goto bail;

  while(ExpEnd)
    {
      y = sin(x  * (M_PI / 180.0)) * m_y;
      if(d == 0)
        {
	  if(cam1->Pan_TiltDrive(EVILIB_ABSOLUTE, EVILIB_max_pspeed, EVILIB_max_tspeed, x, y, EVILIB_NO_WAIT_COMP) != 1)
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
	  if(cam1->Pan_TiltDrive(EVILIB_ABSOLUTE, EVILIB_max_pspeed, EVILIB_max_tspeed, x, -y, EVILIB_NO_WAIT_COMP) != 1)
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

//---------------
// EVI-D70 only
#if 1
  z = 5.0;
  if(cam1->Zoom(EVILIB_DIRECT, EVILIB_max_zspeed, z, EVILIB_NO_WAIT_COMP) != 1)
    goto bail;

  if(cam1->LR_Reverse(EVILIB_ON, EVILIB_NO_WAIT_COMP) != 1)
    goto bail;

  if(cam1->PictureEffect(EVILIB_NEGART, EVILIB_NO_WAIT_COMP) != 1)
    goto bail;

  if(cam1->Display(EVILIB_ON, EVILIB_NO_WAIT_COMP) != 1)
    goto bail;

  data[0] = 1; // VPosition 1
  data[1] = 1; // HPosition 1
  data[2] = 3; // Color 3 - red
  data[3] = 0; // Does not blink
  if(cam1->Title(EVILIB_TITLE_SET1, data , EVILIB_NO_WAIT_COMP) != 1)
    goto bail;
  data[0] = 19;
  data[1] = 7;
  data[2] = 8;
  data[3] = 18;
  data[4] = 27;
  data[5] = 8;
  data[6] = 18;
  data[7] = 27;
  data[8] = 00;
  data[9] = 27;
  if(cam1->Title(EVILIB_TITLE_SET2, data , EVILIB_NO_WAIT_COMP) != 1)
    goto bail;
  data[0] = 19;
  data[1] = 4;
  data[2] = 18;
  data[3] = 19;
  data[4] = 27;
  data[5] = 27;
  data[6] = 27;
  data[7] = 27;
  data[8] = 27;
  data[9] = 27;
  if(cam1->Title(EVILIB_TITLE_SET3, data , EVILIB_NO_WAIT_COMP) != 1)
    goto bail;
  if(cam1->Title(EVILIB_ON, data , EVILIB_NO_WAIT_COMP) != 1)
    goto bail;
  while(ExpEnd != 1);
#endif

#if 0
  if(cam1->LR_Reverse(EVILIB_OFF, EVILIB_NO_WAIT_COMP) != 1)
    goto bail;

  if(cam1->PictureEffect(EVILIB_OFF, EVILIB_NO_WAIT_COMP) != 1)
    goto bail;

  if(cam1->Display(EVILIB_OFF, EVILIB_NO_WAIT_COMP) != 1)
    goto bail;

  if(cam1->Title(EVILIB_OFF, data , EVILIB_NO_WAIT_COMP) != 1)
    goto bail;

#endif

//------------
// EVI-D70 only
#if 0
//    if(cam1->DZoom(EVILIB_SEPARATE_MODE, 0, 0, EVILIB_NO_WAIT_COMP) != 1)
  if(cam1->DZoom(EVILIB_COMBINE_MODE, 0, 0, EVILIB_NO_WAIT_COMP) != 1)
    goto bail;
  if(cam1->DZoomCSModeInq() == EVILIB_SEPARATE_MODE)
    {
#if 1
      if(cam1->Zoom(EVILIB_DIRECT, EVILIB_max_zspeed, EVILIB_maxzoomS, EVILIB_WAIT_COMP) != 1)
	goto bail;
#else
      if(cam1->Zoom(EVILIB_DIRECT, EVILIB_max_zspeed, 0, EVILIB_NO_WAIT_COMP) != 1)
	goto bail;
#endif
      if(cam1->DZoom(EVILIB_DIRECT, EVILIB_max_zspeed, EVILIB_maxdzoom, EVILIB_NO_WAIT_COMP) != 1)
	goto bail;
    }
  else
    {
      if(cam1->Zoom(EVILIB_DIRECT, EVILIB_max_zspeed, EVILIB_maxzoomC, EVILIB_WAIT_COMP) != 1)
	goto bail;
    }

  while(ExpEnd != 1);
#endif

//------------
// EVI-D70 only

#if 0
  for(n = 0; n < 10; n ++)
    data[n] = 0;
  if(cam1->Alarm(EVILIB_ON, data, EVILIB_NO_WAIT_COMP) != 1)
    goto bail;
  data[0] = 0; // mode 00
  if(cam1->Alarm(EVILIB_ON, data, EVILIB_NO_WAIT_COMP) != 1)
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
      if(cam1->Power(EVILIB_OFF) != 1)
	cerr<<"Error Power Off\n";

      cam1->Close();
    }

  if(cam1 != NULL)
    {
      delete cam1;
      cam1 = NULL;
    }

//-------------

  return val;
}
