// EVILib.h
//
// Access functions to a SONY EVI-D30 / EVI-D100
//
// RS-232C Serial Port Communication by Max Lungarella.
// Email: max.lungarella@aist.go.jp
//
// Copyright (c) 2003 Pic Mickael
//
//----------------------------------------------------------------------------
//
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.
//
// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
// Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public
// License along with this library; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
//
//----------------------------------------------------------------------------
//
// Author: Pic Mickael, AIST Japan, 2002.
// Email: mickael.pic@aist.go.jp

#ifndef __EVILIB__
#define __EVILIB__

#include <fcntl.h>
#include <stdio.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <termios.h>
#include <unistd.h>
#include <string.h>
#include <math.h>
#include <iostream>
#include <pthread.h>
#include <errno.h>
#include <pthread.h>

//----------------------------

using namespace std;

//--------------------------------------------------------
// Here you can choose the model of the Sony camera that
//  you are using.
// Be carefull, the parameters between the two camera are
//  differents.
// Basically, you can use the same functions for the two
//  camera. But, some command of the D30 do not work with
//  the D100 (an error code will be return by the function).
//  And, the D100 have some function that are not in the D30.
//  (Theses functions are not implemented yet)

//#define __EVI_D30__
#define __EVI_D100__

//--------------------------------------------------------

#ifdef __EVI_D30__
#define EVILIB_minpan      -100
#define EVILIB_maxpan       100
#define EVILIB_mintilt      -25
#define EVILIB_maxtilt       25
#define EVILIB_maxPan       880
#define EVILIB_maxTilt      300
#define EVILIB_maxZoom     1023
#define EVILIB_min_ZoomSpeed  2
#define EVILIB_max_ZoomSpeed  7
#define EVILIB_minzoom        0
#define EVILIB_maxzoom       12
#define EVILIB_maxFocus   40959
#endif // __EVI_D30__

//--------------------------------------------------------

#ifdef __EVI_D100__
#define EVILIB_minpan      -164
#define EVILIB_maxpan       164
#define EVILIB_mintilt      -30
#define EVILIB_maxtilt       30
#define EVILIB_maxPan      1440
#define EVILIB_maxTilt      360
#define EVILIB_maxZoom    28672 // 0 -> 16384 = optical zoom | 16385 -> 28672 = digital zoom
#define EVILIB_min_ZoomSpeed  0
#define EVILIB_max_ZoomSpeed  7
#define EVILIB_minzoom        0
#define EVILIB_maxzoom       40
#define EVILIB_maxFocus   33792
#define EVILIB_min_FocusSpeed 0
#define EVILIB_max_FocusSpeed 7
#endif // __EVI_D100__

//--------------------------------------------------------

#define EVILIB_min_pspeed     1
#define EVILIB_max_pspeed    24
#define EVILIB_min_tspeed     1
#define EVILIB_max_tspeed    20
#define EVILIB_min_zspeed    34
#define EVILIB_max_zspeed    55

// Because the camera can only store two commands at one time,
//  we do not need a waiting time.
// The commands are regulated by the time needed by the camera to complete them.
#define __EVILIB_Waiting_Time__ 50

// Home position of the camera
#define EVILIB_HomeX 0
#define EVILIB_HomeY 0
#define EVILIB_HomeZ 5.0

#define EVILIB_OFF              1
#define EVILIB_ON               2
#define EVILIB_PAN              3
#define EVILIB_TILT             4
#define EVILIB_ZOOM             5
#define EVILIB_RESET            6
#define EVILIB_UP               7
#define EVILIB_DOWN             8
#define EVILIB_DIRECT           9
#define EVILIB_STOP            10
#define EVILIB_TELE_S          11
#define EVILIB_WIDE_S          12
#define EVILIB_FAR             13
#define EVILIB_NEAR            14
#define EVILIB_AUTO            15
#define EVILIB_MANUAL          16
#define EVILIB_AUTO_MANUAL     17
#define EVILIB_INDOOR          18
#define EVILIB_OUTDOOR         19
#define EVILIB_ONEPUSH_MODE    20
#define EVILIB_ONEPUSH_TRIGGER 21
#define EVILIB_SHUTTER_PRIO    22
#define EVILIB_IRIS_PRIO       23
#define EVILIB_BRIGHT          24
#define EVILIB_SET             25
#define EVILIB_RECALL          26
#define EVILIB_ON_OFF          27
#define EVILIB_LEFT            28
#define EVILIB_RIGHT           29
#define EVILIB_UPLEFT          30
#define EVILIB_UPRIGHT         31
#define EVILIB_DOWNLEFT        32
#define EVILIB_DOWNRIGHT       33
#define EVILIB_ABSOLUTE        34
#define EVILIB_RELATIVE        35
#define EVILIB_HOME            36
#define EVILIB_CLEAR           37
#define EVILIB_CHASE1          39
#define EVILIB_CHASE2          40
#define EVILIB_CHASE3          41
#define EVILIB_CHASE123        42
#define EVILIB_ENTRY1          43
#define EVILIB_ENTRY2          44
#define EVILIB_ENTRY3          45
#define EVILIB_ENTRY4          46
#define EVILIB_Y_LEVEL         47
#define EVILIB_HUE_LEVEL       48
#define EVILIB_SIZE            49
#define EVILIB_DISPLAYTIME     50
#define EVILIB_REFRESH_MODE1   51
#define EVILIB_REFRESH_MODE2   52
#define EVILIB_REFRESH_MODE3   53
#define EVILIB_REFRESH_TIME    54
#define EVILIB_NTSC            55
#define EVILIB_PAL             56
#define EVILIB_NORMAL          57
#define EVILIB_ATMODE          58
#define EVILIB_MDMODE          59
#define EVILIB_SETTING         60
#define EVILIB_TRACKING        61
#define EVILIB_LOST            62
#define EVILIB_UNDETECT        63
#define EVILIB_DETECTED        64
#define EVILIB_POWER_ON_OFF    65
#define EVILIB_ZOOM_TELE_WIDE  66
#define EVILIB_AF_ON_OFF       67
#define EVILIB_CAM_BACKLIGHT   68
#define EVILIB_CAM_MEMORY      69
#define EVILIB_PAN_TILT_DRIVE  70
#define EVILIB_AT_MODE_ON_OFF  71
#define EVILIB_MD_MODE_ON_OFF  72
#define EVILIB_TELE_V          73
#define EVILIB_WIDE_V          74
#define EVILIB_ATW             75
#define EVILIB_FAR_V           76
#define EVILIB_NEAR_V          77
#define EVILIB_INFINITY        78
#define EVILIB_AF_SENS_HIGH    79
#define EVILIB_AF_SENS_LOW     80
#define EVILIB_NEAR_LIMIT      81
#define EVILIB_DZOOM_ON        82
#define EVILIB_DZOOM_OFF       83
#define EVILIB_GAIN_PRIO       84
#define EVILIB_SHUTTER_AUTO    85
#define EVILIB_IRIS_AUTO       86
#define EVILIB_GAIN_AUTO       87
#define EVILIB_CINEMA          88
#define EVILIB_16_9_FULL       89
#define EVILIB_PASTEL          90
#define EVILIB_NEGART          91
#define EVILIB_SEPIA           92
#define EVILIB_BW              93
#define EVILIB_SOLARIZE        94
#define EVILIB_MOSAIC          95
#define EVILIB_SLIM            96
#define EVILIB_STRETCH         97
#define EVILIB_STILL           98
#define EVILIB_FLASH           99
#define EVILIB_LUMI            100
#define EVILIB_TRAIL           101
#define EVILIB_EFFECTLEVEL     102

// Value for the Iris
#define EVILIB_IRIS_CLOSE 0
#define EVILIB_IRIS_F28   1
#define EVILIB_IRIS_F22   2
#define EVILIB_IRIS_F19   3
#define EVILIB_IRIS_F16   4
#define EVILIB_IRIS_F14   5
#define EVILIB_IRIS_F11   6
#define EVILIB_IRIS_F9_6  7
#define EVILIB_IRIS_F8    8
#define EVILIB_IRIS_F6_8  9
#define EVILIB_IRIS_F5_6 10
#define EVILIB_IRIS_F4_8 11
#define EVILIB_IRIS_F4   12
#define EVILIB_IRIS_F3_4 13
#define EVILIB_IRIS_F2_8 14
#define EVILIB_IRIS_F2_4 15
#define EVILIB_IRIS_F2   16
#define EVILIB_IRIS_F1_8 17

// Value for the Bright
#define EVILIB_BRIGHT_CLOSE    0
#define EVILIB_BRIGHT_F28      1
#define EVILIB_BRIGHT_F22      2
#define EVILIB_BRIGHT_F19      3
#define EVILIB_BRIGHT_F16      4
#define EVILIB_BRIGHT_F14      5
#define EVILIB_BRIGHT_F11      6
#define EVILIB_BRIGHT_F9_6     7
#define EVILIB_BRIGHT_F8       8
#define EVILIB_BRIGHT_F6_8     9
#define EVILIB_BRIGHT_F5_6    10
#define EVILIB_BRIGHT_F4_8    11
#define EVILIB_BRIGHT_F4      12
#define EVILIB_BRIGHT_F3_4    13
#define EVILIB_BRIGHT_F2_8    14
#define EVILIB_BRIGHT_F2_4    15
#define EVILIB_BRIGHT_F2      16
#define EVILIB_BRIGHT_F1_8_0  17
#define EVILIB_BRIGHT_F1_8_3  18
#define EVILIB_BRIGHT_F1_8_6  19
#define EVILIB_BRIGHT_F1_8_9  20
#define EVILIB_BRIGHT_F1_8_12 21
#define EVILIB_BRIGHT_F1_8_15 22
#define EVILIB_BRIGHT_F1_8_18 23

#define EVILIB_NO_WAIT_COMP 0
#define EVILIB_WAIT_COMP    1

// Allowed range for the difference between the real position of the camera and the one asked.
#define EVILIB_RANGE_VALUE 0.1

//--------------------------------------------------------

//  * Fonction handled by a thread.
// Receive one answer from the camera.
// Return the number of bytes recevied on success, 0 on error
void *Receiver(void *arg);

//--------------------------------------------------------

class EVILib
{
// Id of the camera
    int Id_cam;

// Power state of the camera.
// If the camera is OFF, then no Command or Inq can be send to the camera.
// A power ON must be done before
    int power;

// Store the pan speed of the camera
    int panspeed;

// Store the tilt speed of the camera
    int tiltspeed;

// Store the zoom speed of the camera
    int zoomspeed;

    struct termios _oldtio;
    int _port;

// Store the command
    char *buffer;
// Store the messages from the camera
    char *buffer2;
// Store the messages from the camera.
    char *buffer3;

// Store the status codes of the Pan Tilt
    int *PanTiltStatus;
// Store the status codes of the AT
    int *ATStatus;
// Store the status code of the MD
    int *MDStatus;

// Store the signal of End for the thread
    int End;
// Store the size of the messagea from the camera
    int answer;
// Store if a command is waiting for completion
    int waitComp;
// Thread used for receiving the messages from the camera
    pthread_t threadReceiver;
// Synchronize the acces to 'buffer3'
    pthread_mutex_t AccessBuffer3;
// Synchronize the access to the 'ACK' messages from the camera
    pthread_mutex_t TakeAck;
// Synchronize the access to the 'information return' message from the camera
    pthread_mutex_t TakeInfo;
// Synchronize the access to the 'command completion' message from the camera
    pthread_mutex_t TakeComp;
// Synchronize the access to the 'information return' message from the camera when we want to catch
// the return from the IR commander
    pthread_mutex_t TakeReturn;
// Allow only to command at the same time in the buffer.
// This is because the camera have a buffer of size 2. If more than 2 commands are send to
//  the camera, a 'command buffer full' message is send by the camera
    pthread_mutex_t BufferDispo;
// Allow to synchronize the thread that handle the return from the camera device
    pthread_mutex_t LaunchThread;
// The camera can store up to two command at the same time.
// If more than 2 commands are send, the camera send back a "Command Buffer Full" error.
    pthread_mutex_t CommandBuffer;
    pthread_cond_t CommandBufferCond;

// Store the return value of pthread_mutex_trylock
    int mutex_return;

// The camera have buffer that can contain up to  command. So we need to check if the buffer is not full
    int Nb_Command_Buffer;

// Transform the data of Hex to the signal send to the camera
    int asciiToPackedHex(unsigned char *buf, int len);

// This function creates a string in the form: "0Y0Y0Y0Y"
//  where YYYY is the hex representation of "pos".
// This is needed for the pan and tilt fields of message to the Sony EVI-D30.
    void make0XString(int pos, char *buf, int len);

// Generate a table a bits from 'x'
    void bitGenerator(unsigned char x, int *table);

// Return the value set in the range of the type
// type is PAN, TILT, ZOOM
// limit is the maximum degree of the type
// maxi is the maximum range of the type 
    int convert(int type, float value, float limit, float maxi);

// Return the value contain in 'buffer3' from 'start' for 'length' bits
// limit and maxi are used to convert the value from Hexa to float
    float deconvert(float limit, float maxi, int start, int length);

// Return the value contain in 'buffer3' from 'start' for 'length' bits
    float deconvert(int start, int length);

// The thread must have direct access to the data.
    friend void *Receiver(void *arg);

// Display the status of the mutex
    void Mutex();

public:
    EVILib();
    ~EVILib();

// Set, Return Id_cam
    inline void SetId_Cam(int id){Id_cam = id;};
    inline int GetId_Cam(){return Id_cam;};

// Set, Return the pan speed
    inline void SetPanSpeed(int s){if(s >= EVILIB_min_pspeed && s <= EVILIB_max_pspeed) panspeed = s;};
    inline int GetPanSpeed(){return panspeed;};

// Set, Return the tilt speed
    inline void SetTiltSpeed(int s){if(s >= EVILIB_min_tspeed && s <= EVILIB_max_tspeed) tiltspeed = s;};
    inline int GetTiltSpeed(){return tiltspeed;};

// Set, Return the zoom speed
    inline void SetZoomSpeed(int s){if(s >= EVILIB_min_zspeed && s <= EVILIB_max_zspeed) zoomspeed = s;};
    inline int GetZoomSpeed(){return zoomspeed;};

// Because the camera is not Highly precise on position, small difference can appear.
// This test allow to check if the value received is near the one asked
// Return 1 on success, 0 otherwise
    inline int TestPanRange(float ask, float rec){if(ask - EVILIB_RANGE_VALUE < rec && rec < ask + EVILIB_RANGE_VALUE) return 1; else return 0;};

// Put the Camera to the Home position
// Return 1 on success, 0 on error
    inline int Home(){return Pan_TiltDrive(EVILIB_ABSOLUTE, EVILIB_max_pspeed, EVILIB_max_tspeed, EVILIB_HomeX, EVILIB_HomeY, EVILIB_WAIT_COMP);};

// Initialize the data
// Return 1 on success
// Return 0 on error
    int Init();

// Open Serial Port - terminal settings
// Return 1 on success, 0 on error
    int Open(int id, char *);

// Restore old port settings and close Serial Port
// Return 1 on success, 0 on error
    int Close();

// Send a command to the camera
// Return the number of bytes sended
    int SendCommand(unsigned char *buf, int len);

// Send AddressSet command and IF_Clear command before starting communication.
// Return 1 on success, 0 on error
    int AddressSet();

// Send AddressSet command and IF_Clear command before starting communication.
// Return 1 on success, 0 on error
    int IF_Clear();

// socket: socket number, 0 or 1
// Return 1 on success, 0 on error
    int CommandCancel(int socket);

// When camera main power is on, camera can be changed to Power Save Mode
// type: EVILIB_ON  : set power on
//       EVILIB_Off : set power off
// Return 1 on success, 0 on error
    int Power(int type);

#ifdef __EVI_D100__
// Auto Power Off
// timer = power off timer parameter 0000 (timer off) to 65535 minutes
// Initial value: 0000
// The power automatically turns off if the camera does not receive any VISCA commands
//  or signals from the Remote Commander for the duration you set in the timer
// waitC: wait for completion of command: EVILIB_NO_WAIT_COMP
//                                        EVILIB_WAIT_COMP
// Return 1 on success, 0 on error
    int AutoPowerOff(int timer, int waitC);
#endif

// Zoom control.
// type: EVILIB_STOP
//       EVILIB_TELE (Standard)
//       EVILIB_WIDE (Standard)
//       EVILIB_TELE (Variable) --> need 'speed'
//       EVILIB_WIDE (Variable) --> need 'speed'
//       EVILIB_DIRECT --> need 'zoom'
// speed: speed parameter EVILIB_min_ZoomSpeed (Low) to EVILIB_max_ZoomSpeed (High)
// zoom: EVILIB_minzoom (Wide) to EVILIB_maxzoom (Tele)
// waitC: wait for completion of command: EVILIB_NO_WAIT_COMP
//                                        EVILIB_WAIT_COMP
// Return 1 on success, 0 on error
    int Zoom(int type, int speed, float zoom, int waitC);

// Focus control.
// When adjust the focus, change the mode to Manual the send Far/Near or Direct command.
// type: EVILIB_STOP
//       EVILIB_FAR
//       EVILIB_NEAR
//       EVILIB_AUTO
//       EVILIB_MANUAL
//       EVILIB_AUTO_MANUAL
//       EVILIB_DIRECT --> need 'focus'
// focus: infinity = 4096, close = 40959
// waitC: wait for completion of command: EVILIB_NO_WAIT_COMP
//                                        EVILIB_WAIT_COMP
// Return 1 on success, 0 on error
    int Focus(int type, int focus, int waitC);

// White Balance Setting.
// type: EVILIB_AUTO         : Trace the light source automatically
//       EVILIB_INDOOR       : fixed at factory
//       EVILIB_OUTDOOR      : fixed at factory
//       EVILIB_ONEPUSH_MODE : Pull-in to White with a Trigger then hold the data until next Trigger comming
//       EVILIB_ONEPUSH_TRIGGER
// waitC: wait for completion of command: EVILIB_NO_WAIT_COMP
//                                        EVILIB_WAIT_COMP
// Return 1 on success, 0 on error
    int WB(int type, int waitC);

#ifdef __EVI_D100__
// type: EVILIB_RESET
//       EVILIB_UP
//       EVILIB_DOWN
//       EVILIB_DIRECT
// gain: R Gain 0000 to 255
// waitC: wait for completion of command: EVILIB_NO_WAIT_COMP
//                                        EVILIB_WAIT_COMP
// Return 1 on success, 0 on error
    int RGain(int type, int gain, int waitC);

// type: EVILIB_RESET
//       EVILIB_UP
//       EVILIB_DOWN
//       EVILIB_DIRECT
// gain: B Gain 0000 to 255
// waitC: wait for completion of command: EVILIB_NO_WAIT_COMP
//                                        EVILIB_WAIT_COMP
// Return 1 on success, 0 on error
    int BGain(int type, int gain, int waitC);
#endif // __EVI_D100__

// type: EVILIB_AUTO         : Auto Exposure Mode
//       EVILIB_MANUAL       : Manual control mode
//       EVILIB_SHUTTER_PRIO : Shutter priority automatic exposure mode
//       EVILIB_IRIS_PRIO    : Iris priority automatic exposure mode
//       EVILIB_GAIN_PRIO    : Gain priority automatic Exposure Mode
//       EVILIB_BRIGHT       : Bright mode (Manual control)
//       EVILIB_SHUTTER_AUTO : Automatic shutter mode
//       EVILIB_IRIS_AUTO    : Automatic iris mode
//       EVILIB_GAIN_AUTO    : Automatic gain mode
// waitC: wait for completion of command: EVILIB_NO_WAIT_COMP
//                                        EVILIB_WAIT_COMP
// Return 1 on success, 0 on error
    int AE(int type, int waitC);

#ifdef __EVI_D100__
// type: EVILIB_AUTO
//       DERVERV_MANUAL
// waitC: wait for completion of command: EVILIB_NO_WAIT_COMP
//                                        EVILIB_WAIT_COMP
// Return 1 on success, 0 on error
    int SlowShutter(int type, int waitC);
#endif // __EVI_D100__

// When turning on to Bright Mode, Iris, Gain and Shutter at the time then increase or decrease
// 3 dB/step using UP/DOWN command
// type: EVILIB_RESET
//       EVILIB_UP
//       EVILIB_DOWN
// cmd: CLOSE to F1.8
// waitC: wait for completion of command: EVILIB_NO_WAIT_COMP
//                                        EVILIB_WAIT_COMP
// Return 1 on success, 0 on error
    int Bright(int type, int cmd, int waitC);

#ifdef __EVI_D100__
// type: EVILIB_ON
//       EVILIB_OFF
//       EVILIB_RESET
//       EVILIB_UP
//       EVILIB_DOWN
//       EVILIB_DIRECT
// cmd: ExpComp Position -7 (-10.5 dB) to 7 (10.5dB), 15 steps at 1.5dB
// waitC: wait for completion of command: EVILIB_NO_WAIT_COMP
//                                        EVILIB_WAIT_COMP
// Return 1 on success, 0 on error
    int ExpComp(int type, int cmd, int waitC);
#endif // __EVI_D100__

// Electronic Shutter Setting
// Enable on AE_Manual, Shutter_Priority
// type: EVILIB_RESET
//       EVILIB_UP
//       EVILIB_DOWN
//       EVILIB_DIRECT --> need 'speed'
// speed: 1/60 to 1/10000 second
// Authorized values: 60 - 75 - 90 - 100 - 125 - 150 - 180 - 215 - 250 - 300 - 350 -
//   425 - 500 - 600 - 725 - 850 - 1000 - 1250 - 1500 - 1750 - 2000 - 2500 - 3000 -
//   3500 - 4000 - 6000 - 10000
// waitC: wait for completion of command: EVILIB_NO_WAIT_COMP
//                                        EVILIB_WAIT_COMP
// Return 1 on success, 0 on error
    int Shutter(int type, int speed, int waitC);

// Iris Setting. Enable on AE_Manual or Iris_Priority
// type: EVILIB_RESET
//       EVILIB_UP
//       EVILIB_DOWN
//       EVILIB_DIRECT --> need 'cmd'
// cmd: CLOSE to F1.8 (See the header of the file for the authorized values)
// waitC: wait for completion of command: EVILIB_NO_WAIT_COMP
//                                        EVILIB_WAIT_COMP
// Return 1 on success, 0 on error
    int Iris(int type, int cmd, int waitC);

// Gain Setting. Enable on AE_Manual only
// type: EVILIB_RESET
//       EVILIB_UP
//       EVILIB_DOWN
//       EVILIB_DIRECT --> need 'setting'
// setting: 0 dB to +18dB by step of 3
// waitC: wait for completion of command: EVILIB_NO_WAIT_COMP
//                                        EVILIB_WAIT_COMP
// Return 1 on success, 0 on error
    int Gain(int type, int setting, int waitC);

// Backlight compensation.
// Gain-up to 6 dB max.
// type: EVILIB_ON
//       EVILIB_OFF
// waitC: wait for completion of command: EVILIB_NO_WAIT_COMP
//                                        EVILIB_WAIT_COMP
// Return 1 on success, 0 on error
    int Backlight(int type, int waitC);

#ifdef __EVI_D100__
// type: EVILIB_RESET
//       EVILIB_UP
//       EVILIB_DOWN
//       EVILIB_DIRECT
// cmd: Aperture Gain 0 to 15, 16 steps, Initial value: 5
// waitC: wait for completion of command: EVILIB_NO_WAIT_COMP
//                                        EVILIB_WAIT_COMP
// Return 1 on success, 0 on error
    int Aperture(int type, int cmd, int waitC);

// Wide mode setting
// type: EVILIB_OFF
//       EVILIB_CINEMA
//       EVILIB_16_9_FULL
// waitC: wait for completion of command: EVILIB_NO_WAIT_COMP
//                                        EVILIB_WAIT_COMP
// Return 1 on success, 0 on error
    int Wide(int type, int waitC);

// Mirror image ON/OFF
// type: EVILIB_ON
//       EVILIB_OFF
// waitC: wait for completion of command: EVILIB_NO_WAIT_COMP
//                                        EVILIB_WAIT_COMP
// Return 1 on success, 0 on error
    int LR_Reverse(int type, int waitC);

// Still image ON/OFF
// type: EVILIB_ON
//       EVILIB_OFF
// waitC: wait for completion of command: EVILIB_NO_WAIT_COMP
//                                        EVILIB_WAIT_COMP
// Return 1 on success, 0 on error
    int Freeze(int type, int waitC);

// Picture effect setting
// type: EVILIB_OFF
//       EVILIB_PASTEL
//       EVILIB_NEGART
//       EVILIB_SEPIA
//       EVILIB_BW
//       EVILIB_SOLARIZE
//       EVILIB_MOSAIC
//       EVILIB_SLIM
//       EVILIB_STRETCH
// waitC: wait for completion of command: EVILIB_NO_WAIT_COMP
//                                        EVILIB_WAIT_COMP
// Return 1 on success, 0 on error
    int PictureEffect(int type, int waitC);

// Digital effect setting
// type: EVILIB_OFF
//       EVILIB_STILL
//       EVILIB_FLASH
//       EVILIB_LUMI
//       EVILIB_TRAIL
//       EVILIB_EFFECTLEVEL -> need 'effect'
// effect: effect level 0 to 24 (flash, trail), 0 to 32 (still, lumi)
// waitC: wait for completion of command: EVILIB_NO_WAIT_COMP
//                                        EVILIB_WAIT_COMP
// Return 1 on success, 0 on error
    int DigitalEffect(int type, int level, int waitC);
#endif // __EVI_D100__

// Preset memory for memorize camera condition
// type: EVILIB_RESET --> need 'position'
//       EVILIB_SET --> need 'position'
//       EVILIB_RECALL -- need 'position'
// position: 0 to 5
// waitC: wait for completion of command: EVILIB_NO_WAIT_COMP
//                                        EVILIB_WAIT_COMP
// Return 1 on success, 0 on error
    int Memory(int type, int position, int waitC);

#ifdef __EVI_D30__
// Enable/Disable for RS-232C and key control
// type: EVILIB_ON
//       EVILIB_OFF
// waitC: wait for completion of command: EVILIB_NO_WAIT_COMP
//                                        EVILIB_WAIT_COMP
// Return 1 on success, 0 on error
    int KeyLock(int type, int waitC);
#endif // __EVI_D30__

// Enable/Disable for IR remote commander
// type: EVILIB_ON
//       EVILIB_OFF
//       EVILIB_ON_OFF
// waitC: wait for completion of command: EVILIB_NO_WAIT_COMP
//                                        EVILIB_WAIT_COMP
// Return 1 on success, 0 on error
    int IR_Receive(int type, int waitC);

// Send replies what command received from IR Commander
// type: EVILIB_ON
//       EVILIB_OFF
// waitC: wait for completion of command: EVILIB_NO_WAIT_COMP
//                                        EVILIB_WAIT_COMP
// Return 1 on success, 0 on error
    int IR_ReceiveReturn(int type, int waitC);

#ifdef __EVI_D30__
// Automatic Target Trace ability Compensation when a wide conversion lens is installed
// setting: 0 (no conversion) to 7 (0.6 conversion)
// Return 1 on success, 0 on error
// waitC: wait for completion of command: EVILIB_NO_WAIT_COMP
//                                        EVILIB_WAIT_COMP
    int Wide_conLensSet(int setting, int waitC);
#endif // __EVI_D30__

// type: EVILIB_UP --> need 'pan_speed' & 'tilt_speed'
//       EVILIB_DOWN --> need 'pan_speed' & 'tilt_speed'
//       EVILIB_LEFT --> need 'pan_speed' & 'tilt_speed'
//       EVILIB_RIGHT --> need 'pan_speed' & 'tilt_speed'
//       EVILIB_UPLEFT --> need 'pan_speed' & 'tilt_speed'
//       EVILIB_UPRIGHT --> need 'pan_speed' & 'tilt_speed'
//       EVILIB_DOWNLEFT --> need 'pan_speed' & 'tilt_speed'
//       EVILIB_DOWNRIGHT --> need 'pan_speed' & 'tilt_speed'
//       EVILIB_STOP --> need 'pan_speed' & 'tilt_speed'
//       EVILIB_ABSOLUTE --> need 'pan_speed' & 'tilt_speed' & 'pan_pos' & 'tilt_pos'
//       EVILIB_RELATIVE --> need 'pan_speed' & 'tilt_speed' & 'pan_pos' & 'tilt_pos'
//       EVILIB_HOME
//       EVILIB_RESET
// pan_speed: pan speed EVILIB_min_pspeed to EVILIB_max_pspeed
// tilt_speed: tilt speed EVILIB_min_tspeed to EVILIB_max_tspeed
// pan_pos: pan position: approx. -EVILIB_maxpan to +EVILIB_maxpan
// tilt_pos: tilt position: approx. -EVILIB_mintilt to +EVILIB_maxtilt
// waitC: wait for completion of command: EVILIB_NO_WAIT_COMP
//                                        EVILIB_WAIT_COMP
// Return 1 on success, 0 on error
    int Pan_TiltDrive(int type, int pan_speed, int tilt_speed, float pan_pos, float tilt_pos, int waitC);

// Pan/Tilt limit set
// type: EVILIB_SET --> need 'mode' & 'pan_pos' & 'tilt_pos'
//       EVILIB_CLEAR --> need 'mode' & 'pan_pos' & 'tilt_pos'
// mode: EVILIB_UPRIGHT
//       EVILIB_DOWNLEFT
// pan_pos: pan position: approx. -EVILIB_maxpan to +EVILIB_maxpan
// tilt_pos: tilt position: approx. -EVILIB_mintilt to +EVILIB_maxtilt
// waitC: wait for completion of command: EVILIB_NO_WAIT_COMP
//                                        EVILIB_WAIT_COMP
// Return 1 on success, 0 on error
    int Pan_TiltLimitSet(int type, int mode, float pan_pos, float tilt_pos, int waitC);

// On screen Data Display ON/OFF
// type: EVILIB_ON
//       EVILIB_OFF
//       EVILIB_ON_OFF
// waitC: wait for completion of command: EVILIB_NO_WAIT_COMP
//                                        EVILIB_WAIT_COMP
// Return 1 on success, 0 on error
    int Datascreen(int type, int waitC);

#ifdef __EVI_D30__
// Target Tracking Mode ON/OFF
// type: EVILIB_ON
//       EVILIB_OFF
//       EVILIB_ON_OFF
// waitC: wait for completion of command: EVILIB_NO_WAIT_COMP
//                                        EVILIB_WAIT_COMP
// Return 1 on success, 0 on error
    int AT_Mode(int type, int waitC);

// Auto Exposure for the target
// type: EVILIB_ON
//       EVILIB_OFF
//       EVILIB_ON_OFF
// waitC: wait for completion of command: EVILIB_NO_WAIT_COMP
//                                        EVILIB_WAIT_COMP
// Return 1 on success, 0 on error
    int AT_AE(int type, int waitC);

// Automatic Zooming for the target
// type: EVILIB_ON
//       EVILIB_OFF
//       EVILIB_ON_OFF
// waitC: wait for completion of command: EVILIB_NO_WAIT_COMP
//                                        EVILIB_WAIT_COMP
// Return 1 on success, 0 on error
    int AT_AutoZoom(int type, int waitC);

// Sensing Frame Display ON/OFF
// type: EVILIB_ON
//       EVILIB_OFF
//       EVILIB_ON_OFF
// waitC: wait for completion of command: EVILIB_NO_WAIT_COMP
//                                        EVILIB_WAIT_COMP
// Return 1 on success, 0 on error
    int AT_MD_Frame_Display(int type, int waitC);

// Shifting the Sensing Frame for AR
// For Shifting use Pan/Tilt Drive Command
// type: EVILIB_ON
//       EVILIB_OFF
//       EVILIB_ON_OFF
// waitC: wait for completion of command: EVILIB_NO_WAIT_COMP
//                                        EVILIB_WAIT_COMP
// Return 1 on success, 0 on error
    int AT_Offset(int type, int waitC);

// Tracking or Detecting Start/Stop
// waitC: wait for completion of command: EVILIB_NO_WAIT_COMP
//                                        EVILIB_WAIT_COMP
// Return 1 on success, 0 on error
    int AT_MD_StartStop(int waitC);

// Select a Tracking Mode
// type: EVILIB_CHASE1
//       EVILIB_CHASE2
//       EVILIB_CHASE3
//       EVILIB_CHASE123
// waitC: wait for completion of command: EVILIB_NO_WAIT_COMP
//                                        EVILIB_WAIT_COMP
// Return 1 on success, 0 on error
    int AT_Chase(int type, int waitC);

// Select target study mode for AT
// type: EVILIB_ENTRY1
//       EVILIB_ENTRY2
//       EVILIB_ENTRY3
//       EVILIB_ENTRY4
// waitC: wait for completion of command: EVILIB_NO_WAIT_COMP
//                                        EVILIB_WAIT_COMP
// Return 1 on success, 0 on error
    int AT_Entry(int type, int waitC);

// Motion Detector Mode ON/OFF
// type: EVILIB_ON
//       EVILIB_OFF
//       EVILIB_ON_OFF
// waitC: wait for completion of command: EVILIB_NO_WAIT_COMP
//                                        EVILIB_WAIT_COMP
// Return 1 on success, 0 on error
    int MD_Mode(int type, int waitC);

// Detecting Area Set (Size or Position)
// waitC: wait for completion of command: EVILIB_NO_WAIT_COMP
//                                        EVILIB_WAIT_COMP
// Return 1 on success, 0 on error
    int MD_Frame(int waitC);

// Select Detecting Frame (1 or 2 or 1 + 2)
// waitC: wait for completion of command: EVILIB_NO_WAIT_COMP
//                                        EVILIB_WAIT_COMP
// Return 1 on success, 0 on error
    int MD_Detect(int waitC);

// Reply a completion when the camera lost the target in AT mode.
// waitC: wait for completion of command: EVILIB_NO_WAIT_COMP
//                                        EVILIB_WAIT_COMP
// Return 1 on success, 0 on error
    int AT_LostInfo(int waitC);

// Reply a completion when the camera detected a motion of image in MD mode.
// waitC: wait for completion of command: EVILIB_NO_WAIT_COMP
//                                        EVILIB_WAIT_COMP
// Return 1 on success, 0 on error
    int MD_LostInfo(int waitC);

// Set Detecting Condition
// type: EVILIB_Y_LEVEL --> need 'setting'
//       EVILIB_HUE_LEVEL --> need 'setting'
//       EVILIB_SIZE --> need 'setting'
//       EVILIB_DISPLAYTIME --> need 'setting'
//       EVILIB_REFRESH_MODE1
//       EVILIB_REFRESH_MODE2
//       EVILIB_REFRESH_MODE3
//       EVILIB_REFRESH_TIME --> need 'setting'
// setting: 0 to 15
// waitC: wait for completion of command: EVILIB_NO_WAIT_COMP
//                                        EVILIB_WAIT_COMP
// Return 1 on success, 0 on error
    int MD_Adjust(int type, int setting, int waitC);

// Target Condition Measure Mode for More Accurate Setting for Motion Detector
// type: EVILIB_ON
//       EVILIB_OFF
//       EVILIB_ON_OFF
// waitC: wait for completion of command: EVILIB_NO_WAIT_COMP
//                                        EVILIB_WAIT_COMP
// Return 1 on success, 0 on error
    int Measure_Mode1(int type, int waitC);

// Target Condition Measure Mode for More Accurate Setting for Motion Detector
// type: EVILIB_ON
//       EVILIB_OFF
//       EVILIB_ON_OFF
// waitC: wait for completion of command: EVILIB_NO_WAIT_COMP
//                                        EVILIB_WAIT_COMP
// Return 1 on success, 0 on error
    int Measure_Mode2(int type, int waitC);
#endif // __EVI_D30__

// Return on success: EVILIB_ON
//                    EVILIB_OFF
// Return on error: 0
    int PowerInq();

#ifdef __EVI_D100__
// timer: contain the power off timer
// Return 1 on success, 0 on error
    int AutoPowerOffInq(int &timer);

// Return on success: EVILIB_ON
//                    EVILIB_OFF
// Return on error: 0
    int DZoomModeInq();
#endif // __EVI_D100__

// zoom: contain the zoom position of the camera
// Return 1 on success, 0 on error
    int ZoomPosInq(float &zoom);

// Return on success: EVILIB_AUTO
//                    EVILIB_MANUAL
// Return on error: 0
    int FocusModeInq();

// focus: contain the focus position of the camera
// Return 1 on success, 0 on error
    int FocusPosInq(int &focus);

#ifdef __EVI_D100__
// Return on success: EVILIB_AF_SENS_HIGH
//                    EVILIB_AF_SENS_LOW
// Return 0 on error
    int AFModeInq();

// focus contain the Focus limit position
// Return 1 on success, 0 on error
    int FocusNearLimitInq(int &focus);
#endif // __EVI_D100__

// Return on success: EVILIB_AUTO
//                    EVILIB_INDOOR
//                    EVILIB_OUTDOOR
//                    EVILIB_ONEPUSH_MODE
//                    EVILIB_ATW
//                    EVILIB_MANUAL
// Return on error: 0
    int WBModeInq();

#ifdef __EVI_D100__
// gain contain the R Gain
// Return 1 on success, 0 on error
    int RGainInq(int &gain);

// gain contain the B Gain
// Return 1 on success, 0 on error
    int BGainInq(int &gain);
#endif // __EVI_D100__

// Return on success: EVILIB_AUTO
//                    EVILIB_MANUAL
//                    EVILIB_SHUTTER_PRIO
//                    EVILIB_IRIS_PRIO
//                    EVILIB_GAIN_PRIO
//                    EVILIB_BRIGHT
//                    EVILIB_SHUTTER_AUTO
//                    EVILIB_IRIS_AUTO
//                    EVILIB_GAIN_AUTO
// Return on error: 0
    int AEModeInq();

#ifdef __EVI_D100__
// Return on success: EVILIB_AUTO
//                    EVILIB_MANUAL
// Return 0 on error
    int SlowShutterModeInq();
#endif // __EVI_D100__

// shutter: contain the shutter position of the camera
// Return 1 on success, 0 on error
    int ShutterPosInq(int &shutter);

// iris: contain the iris position of the camera
// Return 1 on success, 0 on error
    int IrisPosInq(int &iris);

// gain: contain the iris position of the camera
// Return 1 on success, 0 on error
    int GainPosInq(int &gain);

#ifdef __EVI_D100__
// bright contain the bright value of the camera
// Return 1 on success, 0 on error
    int BrightPosInq(int &bright);

// Return on success: EVILIB_ON
//                   EVILIB_OFF
// Return on error: 0
    int ExpCompModeInq();

// ExpComp containt the ExpComp position
// Return 1 on success, 0 on error
    int ExpCompPosInq(int &ExpComp);
#endif // __EVI_D100__

// Return on success: EVILIB_ON
//                    EVILIB_OFF
// Return on error: 0
    int BacklightModeInq();

#ifdef __EVI_D100__
// aperture contain the Aperture Gain
// Return 1 on success, 0 on error
    int ApertureInq(int &aperture);

// Return on success: EVILIB_OFF
//                    EVILIB_CINEMA
//                    EVILIB_16_9_FULL
// Return on error: 0
    int WideModeInq();

// Return on success: EVILIB_ON
//                    EVILIB_OFF
// Return on error: 0
    int LR_ReverseModeInq();

// Return on success: EVILIB_ON
//                    EVILIB_OFF
// Return on error: 0
    int FreezeModeInq();

// Return on success: EVILIB_OFF
//                    EVILIB_PASTEL
//                    EVILIB_NEGART
//                    EVILIB_SEPIA
//                    EVILIB_BW
//                    EVILIB_SOLARIZE
//                    EVILIB_MOSAIC
//                    EVILIB_SLIM
//                    EVILIB_STRETCH
// Return on error: 0
    int PictureEffectModeInq();

// Return on success: EVILIB_OFF
//                    EVILIB_STILL
//                    EVILIB_FLASH
//                    EVILIB_LUMI
//                    EVILIB_TRAIL
// Return on error: 0
    int DigitalEffectModeInq();

// level contain the Effect level
// Return 1 on success, 0 on error
    int DigitalEffectLevelInq(int &level);
#endif // __EVI_D100__

// memory: contain the preset memory for memorize camera condition
// Return 1 on success, 0 on error
    int MemoryInq(int &memory);

#ifdef __EVI_D30__
// Return on success: EVILIB_ON
//                    EVILIB_OFF
// Return on error: 0
    int KeyLockInq();

// id: contain the ID of the camera
// Return 1 on success, 0 on error
    int IDInq(int &id);
#endif // __EVI_D30__

// Return on success: EVILIB_NTSC
//                    EVILIB_PAL
// Return on error: 0
    int VideoSystemInq();

#ifdef __EVI_D100__
// vender contain the Vender ID (1: Sony)
// model contain the Model ID
// rom contain the ROM version
// socket contain the Socket numer (=2)
// Return 1 on success, 0 on error
    int DeviceTypeInq(int &vender, int &model, int &rom, int &socket);
#endif // __EVI_D100__

#ifdef __EVI_D30__
// lens: contain the lens No.
// Return 1 on success, 0 on error
    int Wide_ConLensInq(int &lens);
#endif // __EVI_D30__

// After the completion of Pan_TiltModeInq, you can check the status with the functions
//  provided
// Return 1 on success, 0 on error
    int Pan_TiltModeInq();

// Return 1 if the 'Pan Left End' Pan/Tilter Status is true
// Return 0 otherwise
    inline int PanLeftEnd(){return PanTiltStatus[0];};

// Return 1 if the 'Pan Right End' Pan/Tilter Status is true
// Return 0 otherwise
    inline int PanRightEnd(){return PanTiltStatus[1];};

// Return 1 if the 'Tilt Upper End' Pan/Tilter Status is true
// Return 0 otherwise
    inline int TiltUpperEnd(){return PanTiltStatus[2];};

// Return 1 if the 'Tilt Down End' Pan/Tilter Status is true
// Return 0 otherwise
    inline int TiltDownEnd(){return PanTiltStatus[3];};

// Return 1 if the 'Pan Normal' Pan/Tilter Status is true
// Return 0 otherwise
    inline int PanNormal(){return PanTiltStatus[4];};

// Return 1 if the 'Pan Miss Position' Pan/Tilter Status is true
// Return 0 otherwise
    inline int PanMissPosition(){return PanTiltStatus[5];};

// Return 1 if the 'Pan Mechanical Disorder' Pan/Tilter Status is true
// Return 0 otherwise
    inline int PanMechanicalDisorder(){return PanTiltStatus[6];};

// Return 1 if the 'Tilt Normal' Pan/Tilter Status is true
// Return 0 otherwise
    inline int TiltNormal(){return PanTiltStatus[7];};

// Return 1 if the 'Tilt Miss Position' Pan/Tilter Status is true
// Return 0 otherwise
    inline int TiltMissPosition(){return PanTiltStatus[8];};

// Return 1 if the 'Tilt Mechanical Disorder' Pan/Tilter Status is true
// Return 0 otherwise
    inline int TiltMechanicalDisorder(){return PanTiltStatus[9];};

// Return 1 if the 'No Drive Command' Pan/Tilter Status is true
// Return 0 otherwise
    inline int NoDriveCommand(){return PanTiltStatus[10];};

// Return 1 if the 'Pan, Tilt In-Move' Pan/Tilter Status is true
// Return 0 otherwise
    inline int PanTiltInMove(){return PanTiltStatus[11];};

// Return 1 if the 'Pan, Tilt Drive Completion' Pan/Tilter Status is true
// Return 0 otherwise
    inline int PanTiltDriveCompletion(){return PanTiltStatus[12];};

// Return 1 if the 'Pan, Tilt Drive failure' Pan/Tilter Status is true
// Return 0 otherwise
    inline int PanTiltDriveFailure(){return PanTiltStatus[13];};

// Return 1 if the 'Before Initialize' Pan/Tilter Status is true
// Return 0 otherwise
    inline int BeforeInitialize(){return PanTiltStatus[14];};

// Return 1 if the 'In-Initialize' Pan/Tilter Status is true
// Return 0 otherwise
    inline int InInitialize(){return PanTiltStatus[15];};

// Return 1 if the 'Initialize complete' Pan/Tilter Status is true
// Return 0 otherwise
    inline int InitializeComplete(){return PanTiltStatus[16];};

// Return 1 if the 'Initialize failure' Pan/Tilter Status is true
// Return 0 otherwise
    inline int InitializeFailure(){return PanTiltStatus[17];};

// pan: contain the pan max speed of the camera
// tilt: containt the tilt max speed of the camera
// Return 1 on success, 0 on error
    int Pan_TiltMaxSpeedInq(int &pan, int &tilt);

// pan: contain the pan position of the camera
// tilt: containt the tilt position of the camera
// Return 1 on success, 0 on error
    int Pan_TiltPosInq(float &pan, float &tilt);

// Return on success: EVILIB_ON
//                    EVILIB_OFF
// Return on error: 0
    int DatascreenInq();

#ifdef __EVI_D30__
// Return on success: EVILIB_NORMAL
//                    EVILIB_ATMODE
//                    EVILIB_MDMODE
// Return on error: 0
    int ATMD_ModeInq();

// After the completion of ATModeInq, you can check the status with the functions
//  provided
// Return 1 on success, 0 on error
    int ATModeInq();

// Return 1 if the 'AT frame chase' AT Status is true
// Return 0 otherwise
    inline int ATFrameChase(){return ATStatus[0];};

// Return 1 if the 'AT pan chase' AT Status is true
// Return 0 otherwise
    inline int ATPanChase(){return ATStatus[1];};

// Return 1 if the 'AT frame/pan chase' AT Status is true
// Return 0 otherwise
    inline int ATFramePanChase(){return ATStatus[2];};

// Return 1 if the 'AT offset' AT Status is true
// Return 0 otherwise
    inline int ATOffset(){return ATStatus[3];};

// Return 1 if the 'AT AE on/off' AT Status is true
// Return 0 otherwise
    inline int ATAEOnOff(){return ATStatus[4];};

// Return 1 if the 'AT zoom on/off' AT Status is true
// Return 0 otherwise
    inline int ATZoomOnOff(){return ATStatus[5];};

// Return 1 if the 'AT frame display on/off' AT Status is true
// Return 0 otherwise
    inline int ATFrameDisplayOnOff(){return ATStatus[6];};

// Return 1 if the 'AT setting' AT Status is true
// Return 0 otherwise
    inline int ATSetting(){return ATStatus[7];};

// Return 1 if the 'AT working' AT Status is true
// Return 0 otherwise
    inline int ATWorking(){return ATStatus[8];};

// Return 1 if the 'AT lost' AT Status is true
// Return 0 otherwise
    inline int ATLost(){return ATStatus[9];};

// Return 1 if the 'AT memorizing' AT Status is true
// Return 0 otherwise
    inline int ATMemorizing(){return ATStatus[10];};

// Return on success: EVILIB_ENTRY1
//                    EVILIB_ENTRY2
//                    EVILIB_ENTRY3
//                    EVILIB_ENTRY4
// Return on error: 0
    int AT_EntryInq();

// After the completion of MDModeInq, you can check the status with the functions
//  provided
// Return 1 on success, 0 on error
    int MDModeInq();

// Return 1 if the 'MD detection method' MD Status is true
// Return 0 otherwise
    inline int MDDetectionMethod(){return MDStatus[0];};

// Return 1 if the 'MD settind' MD Status is true
// Return 0 otherwise
    inline int MDSetting(){return MDStatus[1];};

// Return 1 if the 'MD undetect' MD Status is true
// Return 0 otherwise
    inline int MDUndetect(){return MDStatus[2];};

// Return 1 if the 'MD detecting' MD Status is true
// Return 0 otherwise
    inline int MDDetecting(){return MDStatus[3];};

// Return 1 if the 'MD memorizing' MD Status is true
// Return 0 otherwise
    inline int MDMemorizing(){return MDStatus[4];};

// Return 1 if the 'MD frame 1' MD Status is true
// Return 0 otherwise
    inline int MDFrame1(){return MDStatus[5];};

// Return 1 if the 'MDFrame2' MD Status is true
// Return 0 otherwise
    inline int MDFrame2(){return MDStatus[6];};

// Return 1 if the 'MD frame 1 or 2' MD Status is true
// Return 0 otherwise
    inline int MDFrame1or2(){return MDStatus[7];};

// Return 1 if the 'MD frame display' MD Status is true
// Return 0 otherwise
    inline int MDFrameDisplay(){return MDStatus[8];};

// Dividing a sceen by 48*30 pixles, Return the center position of the detecting Frame.
// x: 4 to 42
// y: 3 to 27
// status: EVILIB_SETTING
//         EVILIB_TRACKING
//         EVILIB_LOST
// Return 1 on success, 0 on error
    int AT_ObjetPosInq(int &x, int &y, int &status);

// Dividing a scene by 48*30 pixels, Return the center position of the detecting Frame.
// x: 4 to 42
// y: 3 to 27
// status: EVILIB_SETTING
//         EVILIB_TRACKING
//         EVILIB_LOST
// Return 1 on success, 0 on error
    int MD_ObjetPosInq(int &x, int &y, int &status);

// level: 0 to 15
// Return 1 on success, 0 on error
    int MD_YLevelInq(int &level);

// level: 0 to 15
// Return 1 on success, 0 on error
    int MD_HueLevelInq(int &level);

// size: 0 to 15
// Return 1 on success, 0 on error
    int MD_SizeInq(int &size);

// dt: 0 to 15
// Return 1 on success, 0 on error
    int MD_DispTimeInq(int &dt);

// Return on success: EVILIB_REFRESH_MODE1
//                    EVILIB_REFRESH_MODE2
//                    EVILIB_REFRESH_MODE3
// Return on error: 0
    int MD_RefModeInq();

// rt: 0 to 15
// Return 1 on success, 0 on error
    int MD_RefTimeInq(int &rt);
#endif // __EVI_D30__

// Return on success: EVILIB_POWER_ON_OFF
//                    EVILIB_ZOOM_TELE_WIDE
//                    EVILIB_AF_ON_OFF
//                    EVILIB_CAM_BACKLIGHT
//                    EVILIB_CAM_MEMORY
//                    EVILIB_PAN_TILT_DRIVE
//                    EVILIB_AT_MODE_ON_OFF
//                    EVILIB_MD_MODE_ON_OFF
// Return on error: 0
    int IR_ReceiveReturn();
};

#endif // __EVILIB__


