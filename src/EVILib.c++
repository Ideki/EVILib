// EVILib.c++
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

#include "EVILib.h"

/*
 * Constructor
 */
EVILib::EVILib()
{
    _port = 1;

    buffer = NULL;
    buffer2 = NULL;
    buffer3 = NULL;
    PanTiltStatus = NULL;
    ATStatus = NULL;
    MDStatus = NULL;

    power = EVILIB_OFF;
    End = EVILIB_ON;
}

/*
 *
 */
EVILib::~EVILib()
{
    End = EVILIB_OFF;

    pthread_mutex_destroy(&AccessBuffer3);
    pthread_mutex_destroy(&TakeAck);
    pthread_mutex_destroy(&TakeInfo);
    pthread_mutex_destroy(&TakeComp);
    pthread_mutex_destroy(&TakeReturn);
    pthread_mutex_destroy(&LaunchThread);
    pthread_mutex_destroy(&BufferDispo);
    pthread_mutex_destroy(&CommandBuffer);
    pthread_cond_destroy(&CommandBufferCond);

    if(buffer != NULL)
    {
        delete[] buffer;
        buffer = NULL;
    }
    if(buffer2 != NULL)
    {
        delete[] buffer2;
        buffer2 = NULL;
    }
    if(buffer3 != NULL)
    {
        delete[] buffer3;
        buffer3 = NULL;
    }
    if(PanTiltStatus != NULL)
    {
        delete[] PanTiltStatus;
        PanTiltStatus = NULL;
    }
    if(ATStatus != NULL)
    {
        delete[] ATStatus;
        ATStatus = NULL;
    }
    if(MDStatus != NULL)
    {
        delete[] MDStatus;
        MDStatus = NULL;
    }
}

/*
 * Initialize the data
 * Return 1 on success
 * Return 0 on error
 */
int EVILib::Init()
{
    buffer = new char[50];
    if(buffer == NULL)
    {
        cerr<<"Error EVILib Init(): buffer\n";
        return 0;
    }
    buffer2 = new char[50];
    if(buffer2 == NULL)
    {
        cerr<<"Error EVILib Init(): buffer2\n";
        return 0;
    }
    buffer3 = new char[50];
    if(buffer3 == NULL)
    {
        cerr<<"Error EVILib Init(): buffer3\n";
        return 0;
    }
    PanTiltStatus = new int[18];
    if(PanTiltStatus == NULL)
    {
        cerr<<"Error EVILib Init(): PanTiltStatus\n";
        return 0;
    }
    ATStatus = new int[11];
    if(ATStatus == NULL)
    {
        cerr<<"Error EVILib Init(): ATStatus\n";
        return 0;
    }
    MDStatus = new int[9];
    if(MDStatus == NULL)
    {
        cerr<<"Error EVILib Init(): MDStatus\n";
        return 0;
    }

    _port = -1;

    Nb_Command_Buffer = 0;

    for(int c = 0; c < 18; c ++)
        PanTiltStatus[c] = 0;
    for(int c = 0; c < 11; c ++)
        ATStatus[c] = 0;
    for(int c = 0; c < 9; c ++)
        MDStatus[c] = 0;

    panspeed = EVILIB_max_pspeed;
    tiltspeed = EVILIB_max_tspeed;
    zoomspeed = EVILIB_max_zspeed;

    pthread_mutex_init(&AccessBuffer3, NULL);
    pthread_mutex_init(&TakeAck, NULL);
    pthread_mutex_init(&TakeInfo, NULL);
    pthread_mutex_init(&TakeComp, NULL);
    pthread_mutex_init(&TakeReturn, NULL);
    pthread_mutex_init(&LaunchThread, NULL);
    pthread_mutex_init(&BufferDispo, NULL);
    pthread_mutex_init(&CommandBuffer, NULL);

    pthread_cond_init(&CommandBufferCond, NULL);

    pthread_mutex_lock(&TakeAck);
    pthread_mutex_lock(&TakeInfo);
    pthread_mutex_lock(&TakeComp);
    pthread_mutex_lock(&TakeReturn);
    pthread_mutex_lock(&LaunchThread);

    power = EVILIB_OFF;

    waitComp = EVILIB_NO_WAIT_COMP;

    End = EVILIB_ON;

    return 1;
}

/*
 * Fonction handled by a thread.
 * Receive one answer from the camera.
 * Return the number of bytes recevied on success, 0 on error
 */
void *Receiver(void *arg)
{
    EVILib *commonData;
    commonData = (EVILib*)arg;

    int cam_number = 0;
    int sock_number = 0;
    int sub_mes = 0;
    int i;
    unsigned char c;
    int no_signal_reicv = 0;
    int mutex_return = 0;

#if 0
    int nn = 0;
#endif

    pthread_mutex_unlock(&commonData->LaunchThread);
    do
    {

        c = '\0';
        no_signal_reicv = 0;
        i = 0;

        if (commonData->_port < 1)
        {
            cerr<<"Error EVILib Receiver(void *): There were problems while receiving bytes from the serial port!\n";
            commonData->answer = 0;
        }

// Read until we get the terminating byte, \xff.
        while ((char)c != '\xff' && commonData->End != EVILIB_OFF)
        {
            switch(read(commonData->_port, &c, 1))
            {
            case -1:
                commonData->answer = 0;
                break;
            case 0:
                no_signal_reicv ++;
                if(no_signal_reicv >= 50)
                {
                    mutex_return = pthread_mutex_trylock(&commonData->TakeComp);
                    if(mutex_return == 0)
                        pthread_mutex_unlock(&commonData->TakeComp);
                    else
                    if((mutex_return == EBUSY && commonData->waitComp == EVILIB_WAIT_COMP))
                    {
                        commonData->answer = 0;
                        commonData->waitComp = EVILIB_NO_WAIT_COMP;
                        pthread_mutex_unlock(&commonData->TakeComp);
                        goto skip;
                    }
                    no_signal_reicv = 0;
                }
                break;
            case 1:
                no_signal_reicv = 0;
                commonData->buffer2[i++] = c;
#if 0
                printf("%X",c);
#endif
                break;
            default: // Either we have an error, read nothing, or read 1 byte...
                break;
            }
	    usleep(100);
        }
#if 0
        cout<<"\n";
#endif
    skip:


// *** Empties in/output buffer: Warning this may lead to a loss of information!!! ***
#if 0
        if(nn > 16384)
        {
            tcflush(commonData->_port, TCIFLUSH);
            nn=0;
        }
        nn += i;
#endif

        cam_number = int(((unsigned char) commonData->buffer2[0] & (unsigned char)'\xF0') >> 4) - 8;
        sock_number = int(((unsigned char) commonData->buffer2[1] & (unsigned char)'\x0F'));

// Check for error message
        switch(int(((unsigned char) commonData->buffer2[1] & (unsigned char)'\xF0') >> 4))
        {
        case 6:  // Error message
            sub_mes = int(((unsigned char) commonData->buffer2[2] & (unsigned char)'\xF0') >> 4) * 10 + int(((unsigned char) commonData->buffer2[2] & (unsigned char)'\x0F'));
            switch(sub_mes)
            {
            case 1:
                cerr<<"Error  EVILib Receiver(void *): Message length (>14 bytes) ";
                break;
            case 2:
                cerr<<"Error  EVILib Receiver(void *): syntax error ";
                break;
            case 3:
                commonData->Mutex();
                cerr<<"Error  EVILib Receiver(void *): command buffer full ";
                break;
            case 4:
                cerr<<"Error  EVILib Receiver(void *): command cancel ";
                break;
            case 5:
                cerr<<"Error  EVILib Receiver(void *): no sockets (to be cancelled) ";
                break;
            case 41:
                cerr<<"Error  EVILib Receiver(void *): command not executable ";
                break;
            default:
                cerr<<"Error  EVILib Receiver(void *): unknown error type\n";
                break;
            }
            commonData->answer = 0;
            cerr<<" from camera "<<cam_number<<" socket "<<sock_number<<endl;

// We have to free the mutex.
// Otherwise the server is waiting for an answer that will never come...
            pthread_mutex_unlock(&commonData->TakeAck);
            pthread_mutex_unlock(&commonData->TakeInfo);
            pthread_mutex_unlock(&commonData->TakeReturn);
            pthread_mutex_unlock(&commonData->TakeComp);
            commonData->waitComp = EVILIB_NO_WAIT_COMP;
	    pthread_mutex_lock(&commonData->CommandBuffer);
	    if(commonData->Nb_Command_Buffer > 0)
	      {
		commonData->Nb_Command_Buffer --;
		pthread_cond_broadcast(&commonData->CommandBufferCond);
	      }
            pthread_mutex_unlock(&commonData->CommandBuffer);
            break;

        case 3: // Sent from the peripheral device
#if 0
            cout<<"Broadcast\n";
#endif
            switch(sock_number) // In this case sock_number do not contain the socket number...
            {
            case 0: // Address
	        commonData->answer = i;
                mutex_return = pthread_mutex_trylock(&commonData->TakeComp);
                if(mutex_return == 0)
                    pthread_mutex_unlock(&commonData->TakeComp);
                else
                if(mutex_return == EBUSY && commonData->waitComp == EVILIB_WAIT_COMP)
                {
                    commonData->waitComp = EVILIB_NO_WAIT_COMP;
                    pthread_mutex_unlock(&commonData->TakeComp);                   
                }
		pthread_mutex_lock(&commonData->CommandBuffer);
		if(commonData->Nb_Command_Buffer > 0)
		  {
		    commonData->Nb_Command_Buffer --;
		    pthread_cond_broadcast(&commonData->CommandBufferCond);
		  }
                pthread_mutex_unlock(&commonData->CommandBuffer);
                break;
            case 8: // Network Change
                cerr<<"EVILib Receiver(void *): Network Change for camera "<<cam_number<<endl;
                break;
            default:
                cerr<<"Error  EVILib Receiver(void *): unknown Broadcast type\n";
            }
            break;
        case 4: // Acknowledgement
#if 0
            cout<<"Ack\n";
#endif
            commonData->answer = i;
            mutex_return = pthread_mutex_trylock(&commonData->TakeAck);
            if(mutex_return == 0 || mutex_return == EBUSY)
                pthread_mutex_unlock(&commonData->TakeAck);
// There is no 'semaphore_up(&commandData->CommandBuffer)' here, because the Ack do
//  not empty one of the command buffer.
            break;

        case 5: // Completion (commands or inquiries)
            commonData->answer = i;
            if(i == 3) // Command completion is size 3 only
            {
#if 0
                cout<<"Command Completion\n";
#endif
                mutex_return = pthread_mutex_trylock(&commonData->TakeComp);
                if(mutex_return == 0)
                    pthread_mutex_unlock(&commonData->TakeComp);
                else
                if(mutex_return == EBUSY && commonData->waitComp == EVILIB_WAIT_COMP)
                {
                    commonData->waitComp = EVILIB_NO_WAIT_COMP;
                    pthread_mutex_unlock(&commonData->TakeComp);
                }
            }
            else
            {
#if 0
                cout<<"Information return\n";
#endif
                pthread_mutex_lock(&commonData->AccessBuffer3);
                memcpy(commonData->buffer3, commonData->buffer2, i);
                pthread_mutex_unlock(&commonData->AccessBuffer3);
                mutex_return = pthread_mutex_trylock(&commonData->TakeInfo);
                if(mutex_return == 0 || mutex_return == EBUSY)
                    pthread_mutex_unlock(&commonData->TakeInfo);
            }
	    pthread_mutex_lock(&commonData->CommandBuffer);
	    if(commonData->Nb_Command_Buffer > 0)
	      {
		commonData->Nb_Command_Buffer --;
		pthread_cond_broadcast(&commonData->CommandBufferCond);
	      }
            pthread_mutex_unlock(&commonData->CommandBuffer);
            break;

        case 0:
            switch(sock_number) // In this case sock_number do not contain the socket number...
            {
            case 1: // Return from IF_Clear (broadcast)
                mutex_return = pthread_mutex_trylock(&commonData->TakeComp);
                if(mutex_return == 0)
                    pthread_mutex_unlock(&commonData->TakeComp);
                else
                if(mutex_return == EBUSY && commonData->waitComp == EVILIB_WAIT_COMP)
                {
                    commonData->waitComp = EVILIB_NO_WAIT_COMP;
                    pthread_mutex_unlock(&commonData->TakeComp);
                }
		pthread_mutex_lock(&commonData->CommandBuffer);
		if(commonData->Nb_Command_Buffer > 0)
		  {
		    commonData->Nb_Command_Buffer --;
		    pthread_cond_broadcast(&commonData->CommandBufferCond);
		  }
                pthread_mutex_unlock(&commonData->CommandBuffer);
                break;
            case 7: // IR Receiver Return
#if 0
                cout<<"Return for IR Commander\n";
#endif
                commonData->answer = i;
                pthread_mutex_lock(&commonData->AccessBuffer3);
                memcpy(commonData->buffer3, commonData->buffer2, i);
                pthread_mutex_unlock(&commonData->AccessBuffer3);
                mutex_return = pthread_mutex_trylock(&commonData->TakeReturn);
                if(mutex_return == 0 || mutex_return == EBUSY)
		  pthread_mutex_unlock(&commonData->TakeReturn);
		pthread_mutex_lock(&commonData->CommandBuffer);
		if(commonData->Nb_Command_Buffer > 0)
		  {
		    commonData->Nb_Command_Buffer --;
		    pthread_cond_broadcast(&commonData->CommandBufferCond);
		  }
                pthread_mutex_unlock(&commonData->CommandBuffer);
                break;
            default:
                cerr<<"Error  EVILib Receiver(void *): unknown return type\n";
                break;
            }
            break;
        default:
            cerr<<"Error  EVILib Receiver(void *): Unknown message received from camera\n";
            break;
        }
    }
    while(commonData->End != EVILIB_OFF);

    return NULL;
}

/*
 * Send a command to the camera
 * Return the number of bytes sended
 */
int EVILib::SendCommand(unsigned char *data, int len)
{
  int num;
// We do not allow more than two commands at the same time in the camera
  pthread_mutex_lock(&CommandBuffer); 
  do
    {
       if(Nb_Command_Buffer > 1)
	 pthread_cond_wait(&CommandBufferCond, &CommandBuffer);
    }
  while(Nb_Command_Buffer > 1);
  num = asciiToPackedHex((unsigned char *)data ,len);
  if(write(_port, data, num) != num)
    {
      cerr<<"Error EVILib SendCommand(unsigned char *, int)\n";
      pthread_mutex_unlock(&CommandBuffer);
      return 0;
    }
  Nb_Command_Buffer ++;
  pthread_mutex_unlock(&CommandBuffer);
// *** Empties in/output buffer: Warning this may lead to a loss of information!!! ***
#if 0
  tcflush(_port, TCOFLUSH);
#endif
  return num;
}

/*
 * Open Serial Port - terminal settings
 * Return 1 on success, 0 on error
 */
int EVILib::Open(int id, char *portname)
{
    struct termios newtio;

    if (_port >= 1) // Port already open
    {
        cerr<<"Error EVILib Open(int, char *) Port already open\n";
        return 0;
    }

//    _port = open(portname, O_RDWR | O_NOCTTY | O_NONBLOCK | O_NDELAY);
    _port = open(portname, O_CREAT | O_RDWR| O_NOCTTY );
    fcntl(_port, F_SETFL, 0);

    if (_port == -1)
    {
        cerr<<"Errro EVILib Open(int, char *) ";
        cerr<<errno<<" ";
        switch(errno)
        {
        case EEXIST:
            cerr<<"EEXIST\n";
            break;
        case EISDIR:
            cerr<<"EISDIR\n";
            break;
        case EACCES:
            cerr<<"EACCES\n";
            break;
        case ENAMETOOLONG:
            cerr<<"ENAMETOOLONG\n";
            break;
        case ENOENT:
            cerr<<"ENOENT\n";
            break;
        case ENOTDIR:
            cerr<<"ENOTDIR\n";
            break;
        case ENXIO:
            cerr<<"ENXIO\n";
            break;
        case ENODEV:
            cerr<<"ENODEV\n";
            break;
        case EROFS:
            cerr<<"EROFS\n";
            break;
        case ETXTBSY:
            cerr<<"ETXTBSY\n";
            break;
        case EFAULT:
            cerr<<"EFAULT\n";
            break;
        case ELOOP:
            cerr<<"ELOOP\n";
            break;
        case ENOSPC:
            cerr<<"ENOSPC\n";
            break;
        case ENOMEM:
            cerr<<"ENOMEM\n";
            break;
        case EMFILE:
            cerr<<"EMFILE\n";
            break;
        case ENFILE:
            cerr<<"ENFILE\n";
            break;
        case EBADF:
// This can happen if you set a wrong port name
// Usually this happen is you don't have the rights on the device...
            cerr<<"EBADF\n";
            break;
        default:
            cerr<<"UNKNOW\n";
            break;
        };
        return 0;
    }
    else
    {
        tcgetattr(_port, &_oldtio); 	// Save current port settings
        bzero(&newtio, sizeof(newtio));	// Clear struct for new port settings

        newtio.c_iflag = IGNBRK;
        newtio.c_oflag = 0;
        newtio.c_cflag = B9600 | CS8 | CSIZE | CLOCAL | CREAD;
	newtio.c_lflag = 0;
        newtio.c_cc[VMIN] = newtio.c_cc[VTIME] = 0;

        tcflush(_port, TCIFLUSH);
        if (tcsetattr(_port, TCSANOW, &newtio) < 0)
        {
            cerr<<"Error EVILib Open(int, char *) tcsetattr for new port settings\n";
            return 0;
        }

        if (fcntl(_port, F_SETFL, FNDELAY) < 0)
        {
            cerr<<"Error EVILib Open(int, char *) fcntl\n";
            return 0;
        }

        pthread_create(&threadReceiver, NULL, Receiver, this);

// We are waiting for the thread to be launch (synchronization...)
        pthread_mutex_lock(&LaunchThread);

// This must be done before starting communication
        if(AddressSet() == 0)
        {
            cerr<<"Error EVILib Open(int, char *) AddressSet\n";
            return 0;
        }
        if(IF_Clear() == 0)
        {
            cerr<<"Error EVILib Open(int, char*) IF_Clear\n";
            return 0;
        }

// Set the Id of the camera
        Id_cam = id;

// Set the power state of the camera
        power = PowerInq();

        cout<<"Serial setup OK ... \n";
        return 1;
    }
}

/*
 * Restore old port settings and close Serial Port
 * Return 1 on success, 0 on error
 */
int EVILib::Close()
{
    if(_port < 1) // How can I close an already closed serial port?
    {
        cerr<<"Error EVILib Open(int, char *) serial port already closed\n";
        return 0;
    }
    if(close(_port)!=0)
    {
        cerr<<"Error EVILib Open(int, char *) during closing serial port\n";
        return 0;
    }	
    tcsetattr(_port, TCSANOW, &_oldtio);
    _port = -1;

    End = EVILIB_OFF;

    pthread_join(threadReceiver, NULL);

    return 1;
}

/*
 * Display the value of the semaphore
 */
void EVILib::Mutex()
{
    cout<<"AccessBuffer3: ";
    mutex_return = pthread_mutex_trylock(&AccessBuffer3);
    if(mutex_return == EBUSY)
        cout<<"locked\n";
    else
        cout<<"unlocked\n";
    if(mutex_return == 0)
        pthread_mutex_unlock(&AccessBuffer3);
    cout<<"TakeAck: ";
    mutex_return = pthread_mutex_trylock(&TakeAck);
    if(mutex_return == EBUSY)
        cout<<"locked\n";
    else
        cout<<"unlocked\n";
    if(mutex_return == 0)
        pthread_mutex_unlock(&TakeAck);
    cout<<"TakeInfo: ";
    mutex_return = pthread_mutex_trylock(&TakeInfo);
    if(mutex_return == EBUSY)
        cout<<"locked\n";
    else
        cout<<"unlocked\n";
    if(mutex_return == 0)
        pthread_mutex_unlock(&TakeInfo);
    cout<<"TakeComp: ";
    mutex_return = pthread_mutex_trylock(&TakeComp);
    if(mutex_return == EBUSY)
        cout<<"locked\n";
    else
        cout<<"unlocked\n";
    if(mutex_return == 0)
        pthread_mutex_unlock(&TakeComp);
    cout<<"TakeReturn: ";
    mutex_return = pthread_mutex_trylock(&TakeReturn);
    if(mutex_return == EBUSY)
        cout<<"locked\n";
    else
        cout<<"unlocked\n";
    if(mutex_return == 0)
        pthread_mutex_unlock(&TakeReturn);
    cout<<"LaunchThread: ";
    mutex_return = pthread_mutex_trylock(&LaunchThread);
    if(mutex_return == EBUSY)
        cout<<"locked\n";
    else
        cout<<"unlocked\n";
    if(mutex_return == 0)
        pthread_mutex_unlock(&LaunchThread);
    cout<<"BufferDispo: ";
    mutex_return = pthread_mutex_trylock(&BufferDispo);
    if(mutex_return == EBUSY)
        cout<<"locked\n";
    else
        cout<<"unlocked\n";
    if(mutex_return == 0)
        pthread_mutex_unlock(&BufferDispo);
    cout<<"CommandBuffer: ";
    mutex_return = pthread_mutex_trylock(&CommandBuffer);
    if(mutex_return == EBUSY)
        cout<<"locked\n";
    else
        cout<<"unlocked\n";
    if(mutex_return == 0)
        pthread_mutex_unlock(&CommandBuffer);
}

/*
 * Transform the data of Hex to the signal send to the camera
 */
int EVILib::asciiToPackedHex(unsigned char *data, int len)
{
    int i,j;

    for (i=0,j=0; i<(len); i++,j++)
    {
        if (data[i] >= (unsigned char)'0' && data[i] <= (unsigned char)'9')
            data[j] = data[i] - 48;
        else
            data[j] = data[i] - 55;
        data[j] <<= 4;
        i++;
        if (data[i] >= (unsigned char)'0' && data[i] <= (unsigned char)'9')
            data[j] |= data[i] - 48;
        else
            data[j] |= data[i] - 55;
    }
    return j;
}

/*
 * This function creates a string in the form: "0Y0Y0Y0Y"
 *  where YYYY is the hex representation of "pos".
 * This is needed for the pan and tilt fields of message to the Sony EVI-D30/D100.
 */
void EVILib::make0XString(int pos, char *data, int len)
{
    char temp[80];
    int i, j;

    sprintf(data, "%X", pos);
    if((int)strlen(data) < len)
    {
        j = len - strlen(data);
        for(i = 0; i < j; i ++)
            temp[i] = '0';
        strcpy(&temp[i], data);
    }
    else
    {
        j = strlen(data) - len;
        strcpy(temp, &data[j]);
    }
    for (i=0,j=0; temp[i]!='\0';i++,j++)
    {
        data[j] = '0';
        j++;
        data[j] = temp[i];
    }
    data[j] = '\0';
}

/*
 * Generate a table a bits from 'x'
 */
void EVILib::bitGenerator(unsigned char x, int *table)
{
    int ct;

    for(ct = 0; ct < 8; ct ++)
        table[ct] = 0;

    for (ct = 7; x!=0; x>>=1)
    {
        if ( x & 01 )
            table[ct] = 1;
        else
            table[ct] = 0;
        ct --;
    }
}

/*
 * Return the value set in the range of the type
 * type is PAN, TILT, ZOOM
 * limit is the maximum degree of the type
 * maxi is the maximum range of the type
 */
int EVILib::convert(int type, float value, float limit, float maxi)
{
    double x;

    switch(type)
    {
    case EVILIB_PAN:
        x = value / limit * maxi;
        if(x > maxi)
	    x = maxi;
 	else
        if(x < -maxi)
	    x = -maxi;
        break;
    case EVILIB_TILT:
        x = value / limit * maxi;
	if(x > maxi)
	   x = maxi;
	else
        if(x < -maxi)
	   x = -maxi;
        break;
    case EVILIB_ZOOM:
        x = value / limit * maxi; 
	if(x < 0.0)
	    x = 0.0;
	else
        if(x > maxi)
	    x = maxi;
        break;
    default:
        return 0;
    }
    return int(rint(x));
}

/*
 * Return the value contain in 'buffer3' from 'start' for 'length' bits
 *  limit and maxi are used to convert the value from Hexa to float
 */
float EVILib::deconvert(float limit, float maxi, int start, int length)
{
    int c, i, y;
    y = 0;
    c = 1;

    for(i = length - 1; i >= 0; i --)
    {
        if((unsigned char) buffer3[start + i] == (unsigned char)'\x0'){y += 0 * c; goto suite;}
        if((unsigned char) buffer3[start + i] == (unsigned char)'\x1'){y += 1 * c; goto suite;}
        if((unsigned char) buffer3[start + i] == (unsigned char)'\x2'){y += 2 * c; goto suite;}
        if((unsigned char) buffer3[start + i] == (unsigned char)'\x3'){y += 3 * c; goto suite;}
        if((unsigned char) buffer3[start + i] == (unsigned char)'\x4'){y += 4 * c; goto suite;}
        if((unsigned char) buffer3[start + i] == (unsigned char)'\x5'){y += 5 * c; goto suite;}
        if((unsigned char) buffer3[start + i] == (unsigned char)'\x6'){y += 6 * c; goto suite;}
        if((unsigned char) buffer3[start + i] == (unsigned char)'\x7'){y += 7 * c; goto suite;}
        if((unsigned char) buffer3[start + i] == (unsigned char)'\x8'){y += 8 * c; goto suite;}
        if((unsigned char) buffer3[start + i] == (unsigned char)'\x9'){y += 9 * c; goto suite;}
        if((unsigned char) buffer3[start + i] == (unsigned char)'\xA'){y += 10 * c; goto suite;}
        if((unsigned char) buffer3[start + i] == (unsigned char)'\xB'){y += 11 * c; goto suite;}
        if((unsigned char) buffer3[start + i] == (unsigned char)'\xC'){y += 12 * c; goto suite;}
        if((unsigned char) buffer3[start + i] == (unsigned char)'\xD'){y += 13 * c; goto suite;}
        if((unsigned char) buffer3[start + i] == (unsigned char)'\xE'){y += 14 * c; goto suite;}
        if((unsigned char) buffer3[start + i] == (unsigned char)'\xF'){y += 15 * c; goto suite;}
        suite:
        c = c * 16;
    }

// if the first digit is not a '0', then we have a negative number
    if(buffer3[start] != '\x0')
        y -= c;

    return (y / maxi * limit);
}

/*
 * Return the value contain in 'buffer3' from 'start' for 'length' bits
 */
float EVILib::deconvert(int start, int length)
{
    int c, i, y;
    y = 0;
    c = 1;

    for(i = length - 1; i >= 0; i --)
    {
        if((unsigned char) buffer3[start + i] == (unsigned char)'\x0'){y += 0 * c; goto suite;}
        if((unsigned char) buffer3[start + i] == (unsigned char)'\x1'){y += 1 * c; goto suite;}
        if((unsigned char) buffer3[start + i] == (unsigned char)'\x2'){y += 2 * c; goto suite;}
        if((unsigned char) buffer3[start + i] == (unsigned char)'\x3'){y += 3 * c; goto suite;}
        if((unsigned char) buffer3[start + i] == (unsigned char)'\x4'){y += 4 * c; goto suite;}
        if((unsigned char) buffer3[start + i] == (unsigned char)'\x5'){y += 5 * c; goto suite;}
        if((unsigned char) buffer3[start + i] == (unsigned char)'\x6'){y += 6 * c; goto suite;}
        if((unsigned char) buffer3[start + i] == (unsigned char)'\x7'){y += 7 * c; goto suite;}
        if((unsigned char) buffer3[start + i] == (unsigned char)'\x8'){y += 8 * c; goto suite;}
        if((unsigned char) buffer3[start + i] == (unsigned char)'\x9'){y += 9 * c; goto suite;}
        if((unsigned char) buffer3[start + i] == (unsigned char)'\xA'){y += 10 * c; goto suite;}
        if((unsigned char) buffer3[start + i] == (unsigned char)'\xB'){y += 11 * c; goto suite;}
        if((unsigned char) buffer3[start + i] == (unsigned char)'\xC'){y += 12 * c; goto suite;}
        if((unsigned char) buffer3[start + i] == (unsigned char)'\xD'){y += 13 * c; goto suite;}
        if((unsigned char) buffer3[start + i] == (unsigned char)'\xE'){y += 14 * c; goto suite;}
        if((unsigned char) buffer3[start + i] == (unsigned char)'\xF'){y += 15 * c; goto suite;}
        suite:
        c = c * 16;
    }
    return y;
}

//--------------------------------------------------------------------------------------

/*
 * Send AddressSet command and IF_Clear command before starting communication.
 * Return 1 on success, 0 on error
 */
int EVILib::AddressSet()
{
    pthread_mutex_lock(&BufferDispo);
    sprintf(buffer, "883001FF");
    if(SendCommand((unsigned char *)buffer, strlen(buffer)) == 0)
    {
        pthread_mutex_unlock(&BufferDispo);
        goto bail;
    }
    pthread_mutex_unlock(&BufferDispo);
    waitComp = EVILIB_WAIT_COMP;
    pthread_mutex_lock(&TakeComp);
// receiv 'ACK'
    return answer;
 bail:
    return 0;
}

/*
 * Send AddressSet command and IF_Clear command before starting communication.
 * Return 1 on success, 0 on error
 */
int EVILib::IF_Clear()
{
    pthread_mutex_lock(&BufferDispo);
    sprintf(buffer, "88010001FF");
    if(SendCommand((unsigned char *)buffer, strlen(buffer)) == 0)
    {
        pthread_mutex_unlock(&BufferDispo);
        goto bail;
    }
    pthread_mutex_unlock(&BufferDispo);
    waitComp = EVILIB_WAIT_COMP;
    pthread_mutex_lock(&TakeComp);
// receiv 'ACK'
    return answer;
 bail:
    return 0;
}

/*
 * socket: socket number, 0 or 1
 * Return 1 on success, 0 on error
 */
int EVILib::CommandCancel(int socket)
{
    if(power == EVILIB_ON)
    {
        pthread_mutex_lock(&BufferDispo);
        sprintf(buffer, "8%d2%dFF", Id_cam, socket);
        if(SendCommand((unsigned char *)buffer, strlen(buffer)) == 0)
        {
            pthread_mutex_unlock(&BufferDispo);
            return 0;
        }
        pthread_mutex_unlock(&BufferDispo);
        pthread_mutex_lock(&TakeAck);
// receiv 'ACK'
        if(answer < 1)
            return 0;

// We do not take the completion for this command, because there is probably another command waiting
//  for the completion of the command we cancel. When the completion of cancel will arrive, the previous
//  command as to take it to be set free.
        return 1;
    }
    return 0;
}

/*
 * When camera main power is on, camera can be changed to Power Save Mode
 * type: EVILIB_ON  : set power on
 *       EVILIB_Off : set power off
 * Return 1 on success, 0 on error
 */
int EVILib::Power(int type)
{
    float x1, x2, y1, y2, z1, z2;

    switch(type)
    {
    case EVILIB_ON:
        pthread_mutex_lock(&BufferDispo);
        sprintf(buffer, "8%d01040002FF", Id_cam);
        if(SendCommand((unsigned char *)buffer, strlen(buffer)) == 0)
        {
            pthread_mutex_unlock(&BufferDispo);
            return 0;
        }
        pthread_mutex_unlock(&BufferDispo);
        pthread_mutex_lock(&TakeAck);
// receiv 'ACK'
        if(answer < 1)
            return 0;

        power = EVILIB_ON;

// We put a small sleep because we need time to send to POWER command to the camera
// And then, we wait until the camera have finish the power ON procedure.
// This can be check by the values of the position of the camera that are out of the predefined range
//        reliable_usleep(100000);
        do
        {

            do
            {
                if(Pan_TiltPosInq(x1, y1) == 0)
                    goto bail;
                if(ZoomPosInq(z1) == 0)
                    goto bail;
//                reliable_usleep(__EVILIB_Waiting_Time__);
                if(Pan_TiltPosInq(x2, y2) == 0)
                    goto bail;
                if(ZoomPosInq(z2) == 0)
                    goto bail;
            }
            while(x1 != x2 || y1 != y2 || z1 != z2);
        }
        while(x1 < EVILIB_minpan || x2 > EVILIB_maxpan ||
              y1 < EVILIB_mintilt || y2 > EVILIB_maxtilt ||
              z1 < EVILIB_minzoom || z2 > EVILIB_maxzoom);

        break;
    case EVILIB_OFF:
        pthread_mutex_lock(&BufferDispo);
        sprintf(buffer, "8%d01040003FF", Id_cam);
        if(SendCommand((unsigned char *)buffer, strlen(buffer)) == 0)
        {
            pthread_mutex_unlock(&BufferDispo);
            goto bail;
        }
        pthread_mutex_unlock(&BufferDispo);
        pthread_mutex_lock(&TakeAck);
// receiv 'ACK'
        if(answer < 1)
            goto bail;

        power = EVILIB_OFF;
        break;
    default:
        goto bail;
    }
    return 1;
 bail:
    return 0;
}

#ifdef __EVI_D100__
/*
 * Auto Power Off
 * timer = power off timer parameter 0000 (timer off) to 65535 minutes
 * Initial value: 0000
 * The power automatically turns off if the camera does not receive any VISCA commands
 *  or signals from the Remote Commander for the duration you set in the timer
 * waitC: wait for completion of command: EVILIB_NO_WAIT_COMP
 *                                        EVILIB_WAIT_COMP
 * Return 1 on success, 0 on error
 */
int EVILib::AutoPowerOff(int timer, int waitC)
{
    if(power == EVILIB_ON)
    {
        pthread_mutex_lock(&BufferDispo);
        if(timer >= 0 && timer <= 65535)
        {
            sprintf(buffer, "8%d010440", Id_cam);
            make0XString(timer, &buffer[strlen(buffer)], 4);
            strcat(buffer, "FF");
        }
        else
        {
            pthread_mutex_unlock(&BufferDispo);
            return 0;
        }
        if(SendCommand((unsigned char *)buffer, strlen(buffer)) == 0)
        {
            pthread_mutex_unlock(&BufferDispo);
            return 0;
        }
        pthread_mutex_unlock(&BufferDispo);
// receiv 'ACK'
        pthread_mutex_lock(&TakeAck);
// Wait for Completion
        if(waitC == EVILIB_WAIT_COMP)
        {
            waitComp = EVILIB_WAIT_COMP;
            pthread_mutex_lock(&TakeComp);
        }
        if(answer < 1)
            return 0;
        return 1;
    }
    return 0;
}
#endif // __EVI_D100__

/*
 * Zoom control.
 * type: EVILIB_STOP
 *       EVILIB_TELE_S (Standard)
 *       EVILIB_WIDE_S (Standard)
 *       EVILIB_TELE_V (Variable) --> need 'speed'
 *       EVILIB_WIDE_V (Variable) --> need 'speed'
 *       EVILIB_DIRECT --> need 'zoom'
 *       EVILIB_DZOOM_ON
 *       EVILIB_DZOOM_OFF
 * speed: speed parameter EVILIB_min_ZoomSpeed (Low) to EVILIB_max_ZoomSpeed (High)
 * zoom: EVILIB_minzoom (Wide) to EVILIB_maxzoom(Tele)
 * waitC: wait for completion of command: EVILIB_NO_WAIT_COMP
 *                                        EVILIB_WAIT_COMP
 * Return 1 on success, 0 on error
 */
int EVILib::Zoom(int type, int speed, float zoom, int waitC)
{
    if(power == EVILIB_ON)
    {
        pthread_mutex_lock(&BufferDispo);
        switch(type)
        {
        case EVILIB_STOP:
            sprintf(buffer, "8%d01040700FF", Id_cam);
            break;
        case EVILIB_TELE_S:
            sprintf(buffer, "8%d01040702FF", Id_cam);
            break;
        case EVILIB_WIDE_S:
            sprintf(buffer, "8%d01040703FF", Id_cam);
            break;
        case EVILIB_TELE_V:
            if(speed >= EVILIB_min_ZoomSpeed && speed <= EVILIB_max_ZoomSpeed)
                sprintf(buffer, "8%d01040720%XFF", Id_cam, speed);
            else
            {
                pthread_mutex_unlock(&BufferDispo);
                return 0;
            }
            break;
        case EVILIB_WIDE_V:
            if(speed >= EVILIB_min_ZoomSpeed && speed <= EVILIB_max_ZoomSpeed)
                sprintf(buffer, "8%d01040730%XFF", Id_cam, speed);
            else
            {
                pthread_mutex_unlock(&BufferDispo);
                return 0;
            }
            break;
        case EVILIB_DIRECT:
            if(zoom >= EVILIB_minzoom && zoom <= EVILIB_maxzoom)
            {
                sprintf(buffer, "8%d010447", Id_cam);
                make0XString(convert(EVILIB_ZOOM, zoom, EVILIB_maxzoom, EVILIB_maxZoom), &buffer[strlen(buffer)], 4);
                strcat(buffer, "FF");
            }
            else
            {
                pthread_mutex_unlock(&BufferDispo);
                return 0;
            }
            break;
#ifdef __EVI_D100__
        case EVILIB_DZOOM_ON:
            sprintf(buffer, "8%d01040602FF", Id_cam);
            break;
        case EVILIB_DZOOM_OFF:
            sprintf(buffer, "8%d01040603FF", Id_cam);
            break;
#endif // __EVI_D100__
        default:
            pthread_mutex_unlock(&BufferDispo);
            return 0;
        }
        if(SendCommand((unsigned char *)buffer, strlen(buffer)) == 0)
        {
            pthread_mutex_unlock(&BufferDispo);
            return 0;
        }
        pthread_mutex_unlock(&BufferDispo);
// receiv 'ACK'
        pthread_mutex_lock(&TakeAck);
// Wait for Completion
        if(waitC == EVILIB_WAIT_COMP)
        {
            waitComp = EVILIB_WAIT_COMP;
            pthread_mutex_lock(&TakeComp);
        }
        if(answer < 1)
            return 0;
        return 1;
    }
    return 0;
}

/*
 * Focus control.
 * When adjust the focus, change the mode to Manual the send Far/Near or Direct command.
 * type: EVILIB_STOP
 *       EVILIB_FAR
 *       EVILIB_NEAR
 *       EVILIB_FAR_V  -> focus: speed parameter, 0 (low) to 7 (high), 8 steps
 *       EVILIB_NEAR_V -> focus: speed parameter, 0 (low) to 7 (high), 8 steps
 *       EVILIB_DIRECT -> focus: infinity = 4096, close = EVILIB_maxFocus
 *       EVILIB_AUTO
 *       EVILIB_MANUAL
 *       EVILIB_AUTO_MANUAL
 *       EVILIB_ONEPUSH_TRIGGER
 *       EVILIB_INFINITY
 *       EVILIB_AF_SENS_HIGH
 *       EVILIB_AF_SENS_LOW
 *       EVILIB_NEAR_LIMIT -> focus: focus near limit position 1000 (far) to 8400 (near)
 * waitC: wait for completion of command: EVILIB_NO_WAIT_COMP
 *                                        EVILIB_WAIT_COMP
 * Return 1 on success, 0 on error
 */
int EVILib::Focus(int type, int focus, int waitC)
{
    if(power == EVILIB_ON)
    {
        pthread_mutex_lock(&BufferDispo);
        switch(type)
        {
        case EVILIB_STOP:
            sprintf(buffer, "8%d01040800FF", Id_cam);
            break;
        case EVILIB_FAR:
            sprintf(buffer, "8%d01040802FF", Id_cam);
            break;
        case EVILIB_NEAR:
            sprintf(buffer, "8%d01040803FF", Id_cam);
            break;
#ifdef __EVI_D100__
        case EVILIB_FAR_V:
            if(focus >= EVILIB_min_FocusSpeed && focus <= EVILIB_max_FocusSpeed)
                sprintf(buffer, "8%d0104082%XFF", Id_cam, focus);
            else
            {
                pthread_mutex_unlock(&BufferDispo);
                return 0;
            }
            break;
        case EVILIB_NEAR_V:
            if(focus >= EVILIB_min_FocusSpeed && focus <= EVILIB_max_FocusSpeed)
                sprintf(buffer, "8%d0104083%XFF", Id_cam, focus);
            else
            {
                pthread_mutex_unlock(&BufferDispo);
                return 0;
            }
            break;
#endif // __EVI_D100__
        case EVILIB_DIRECT:
            if(focus >= 4096 && focus <= EVILIB_maxFocus)
            {
                sprintf(buffer, "8%d010448", Id_cam);
                make0XString(focus, &buffer[strlen(buffer)], 4);
                strcat(buffer, "FF");
            }
            else
            {
                pthread_mutex_unlock(&BufferDispo);
                return 0;
            }
            break;
        case EVILIB_AUTO:
            sprintf(buffer, "8%d01043802FF", Id_cam);
            break;
        case EVILIB_MANUAL:
            sprintf(buffer, "8%d01043803FF", Id_cam);
            break;
        case EVILIB_AUTO_MANUAL:
            sprintf(buffer, "8%d01043810FF", Id_cam);
            break;
#ifdef __EVI_D100__
        case EVILIB_ONEPUSH_TRIGGER:
            sprintf(buffer, "8%d01041801FF", Id_cam);
            break;
        case EVILIB_INFINITY:
            sprintf(buffer, "8%d01041802FF", Id_cam);
            break;
        case EVILIB_AF_SENS_HIGH:
            sprintf(buffer, "8%d01045802FF", Id_cam);
            break;
        case EVILIB_AF_SENS_LOW:
            sprintf(buffer, "8%d01045803FF", Id_cam);
            break;
        case EVILIB_NEAR_LIMIT:
            if(focus >= 1000 && focus <= 8400)
            {
                sprintf(buffer, "8%d010428", Id_cam);
                make0XString(focus, &buffer[strlen(buffer)], 4);
                strcat(buffer, "FF");
            }
            else
            {
                pthread_mutex_unlock(&BufferDispo);
                return 0;
            }
            break;
#endif // __EVI_D100__
        default:
            pthread_mutex_unlock(&BufferDispo);
            return 0;
        }
        if(SendCommand((unsigned char *)buffer, strlen(buffer)) == 0)
        {
            pthread_mutex_unlock(&BufferDispo);
            return 0;
        }
        pthread_mutex_unlock(&BufferDispo);
// receiv 'ACK'
        pthread_mutex_lock(&TakeAck);
// Wait for Completion
        if(waitC == EVILIB_WAIT_COMP)
        {
            waitComp = EVILIB_WAIT_COMP;
            pthread_mutex_lock(&TakeComp);
        }
        if(answer < 1)
            return 0;
        return 1;
    }
    return 0;
}

/*
 * White Balance Setting.
 * type: EVILIB_AUTO         : Trace the light source automatically
 *       EVILIB_INDOOR       : fixed at factory
 *       EVILIB_OUTDOOR      : fixed at factory
 *       EVILIB_ONEPUSH_MODE : Pull-in to White with a Trigger then hold the data until next Trigger comming
 *       EVILIB_ONEPUSH_TRIGGER
 * waitC: wait for completion of command: EVILIB_NO_WAIT_COMP
 *                                        EVILIB_WAIT_COMP
 * Return 1 on success, 0 on error
 */
int EVILib::WB(int type, int waitC)
{
    if(power == EVILIB_ON)
    {
        pthread_mutex_lock(&BufferDispo);
        switch(type)
        {
        case EVILIB_AUTO:
            sprintf(buffer, "8%d01043500FF", Id_cam);
            break;
        case EVILIB_INDOOR:
            sprintf(buffer, "8%d01043501FF", Id_cam);
            break;
        case EVILIB_OUTDOOR:
            sprintf(buffer, "8%d01043502FF", Id_cam);
            break;
        case EVILIB_ONEPUSH_MODE:
            sprintf(buffer, "8%d01043503FF", Id_cam);
            break;
#ifdef __EVI_D100__
        case EVILIB_ATW:
            sprintf(buffer, "8%d01043504FF", Id_cam);
            break;
        case EVILIB_MANUAL:
            sprintf(buffer, "8%d01043505FF", Id_cam);
            break;
#endif // __EVI_D100__
        case EVILIB_ONEPUSH_TRIGGER:
            sprintf(buffer, "8%d01041005FF", Id_cam);
            break;
        default:
            pthread_mutex_unlock(&BufferDispo); 
            return 0;
        }
        if(SendCommand((unsigned char *)buffer, strlen(buffer)) == 0)
        {
            pthread_mutex_unlock(&BufferDispo);
            return 0;
        }
        pthread_mutex_unlock(&BufferDispo);
// receiv 'ACK'
        pthread_mutex_lock(&TakeAck);
// Wait for Completion
        if(waitC == EVILIB_WAIT_COMP)
        {
            waitComp = EVILIB_WAIT_COMP;
            pthread_mutex_lock(&TakeComp);
        }
        if(answer < 1)
            return 0;
        return 1;
    }
    return 0;
}

#ifdef __EVI_D100__
/*
 * type: EVILIB_RESET
 *       EVILIB_UP
 *       EVILIB_DOWN
 *       EVILIB_DIRECT
 * gain: R Gain 0 to 255, 256 steps
 * waitC: wait for completion of command: EVILIB_NO_WAIT_COMP
 *                                        EVILIB_WAIT_COMP
 * Return 1 on success, 0 on error
 */
int EVILib::RGain(int type, int gain, int waitC)
{
    if(power == EVILIB_ON)
    {
        pthread_mutex_lock(&BufferDispo);
        switch(type)
        {
        case EVILIB_RESET:
            sprintf(buffer, "8%d01040300FF", Id_cam);
            break;
        case EVILIB_UP:
            sprintf(buffer, "8%d01040302FF", Id_cam);
            break;
        case EVILIB_DOWN:
            sprintf(buffer, "8%d01040303FF", Id_cam);
            break;
        case EVILIB_DIRECT:
           if(gain >= 0 && gain <= 255)
            {
                sprintf(buffer, "8%d010443", Id_cam);
                make0XString(gain, &buffer[strlen(buffer)], 4);
                strcat(buffer, "FF");
            }
            else
            {
                pthread_mutex_unlock(&BufferDispo);
                return 0;
            }
           break;
        default:
            pthread_mutex_unlock(&BufferDispo);
            return 0;
        }
        if(SendCommand((unsigned char *)buffer, strlen(buffer)) == 0)
        {
            pthread_mutex_unlock(&BufferDispo);
            return 0;
        }
        pthread_mutex_unlock(&BufferDispo);
// receiv 'ACK'
        pthread_mutex_lock(&TakeAck);
// Wait for Completion
        if(waitC == EVILIB_WAIT_COMP)
        {
            waitComp = EVILIB_WAIT_COMP;
            pthread_mutex_lock(&TakeComp);
        }
        if(answer < 1)
            return 0;
        return 1;
    }
    return 0;
}

/*
 * type: EVILIB_RESET
 *       EVILIB_UP
 *       EVILIB_DOWN
 *       EVILIB_DIRECT
 * gain: B Gain 0 to 255, 256 steps
 * waitC: wait for completion of command: EVILIB_NO_WAIT_COMP
 *                                        EVILIB_WAIT_COMP
 * Return 1 on success, 0 on error
 */
int EVILib::BGain(int type, int gain, int waitC)
{
    if(power == EVILIB_ON)
    {
        pthread_mutex_lock(&BufferDispo);
        switch(type)
        {
        case EVILIB_RESET:
            sprintf(buffer, "8%d01040400FF", Id_cam);
            break;
        case EVILIB_UP:
            sprintf(buffer, "8%d01040402FF", Id_cam);
            break;
        case EVILIB_DOWN:
            sprintf(buffer, "8%d01040403FF", Id_cam);
            break;
        case EVILIB_DIRECT:
           if(gain >= 0 && gain <= 255)
            {
                sprintf(buffer, "8%d010444", Id_cam);
                make0XString(gain, &buffer[strlen(buffer)], 4);
                strcat(buffer, "FF");
            }
            else
            {
                pthread_mutex_unlock(&BufferDispo);
                return 0;
            }
           break;
        default:
            pthread_mutex_unlock(&BufferDispo);
            return 0;
        }
        if(SendCommand((unsigned char *)buffer, strlen(buffer)) == 0)
        {
            pthread_mutex_unlock(&BufferDispo);
            return 0;
        }
        pthread_mutex_unlock(&BufferDispo);
// receiv 'ACK'
        pthread_mutex_lock(&TakeAck);
// Wait for Completion
        if(waitC == EVILIB_WAIT_COMP)
        {
            waitComp = EVILIB_WAIT_COMP;
            pthread_mutex_lock(&TakeComp);
        }
        if(answer < 1)
            return 0;
        return 1;
    }
    return 0;
}
#endif // __EVI_D100__

/*
 * type: EVILIB_AUTO         : Auto exposure mode
 *       EVILIB_MANUAL       : Manual control mode
 *       EVILIB_SHUTTER_PRIO : Shutter priority automatic exposure mode
 *       EVILIB_IRIS_PRIO    : Iris priority automatic exposure mode
 *       EVILIB_GAIN_PRIO    : Gain priority automatic Exposure Mode
 *       EVILIB_BRIGHT       : Bright mode (Manual control)
 *       EVILIB_SHUTTER_AUTO : Automatic shutter mode
 *       EVILIB_IRIS_AUTO    : Automatic iris mode
 *       EVILIB_GAIN_AUTO    : Automatic gain mode
 * waitC: wait for completion of command: EVILIB_NO_WAIT_COMP
 *                                        EVILIB_WAIT_COMP
 * Return 1 on success, 0 on error
 */
int EVILib::AE(int type, int waitC)
{
    if(power == EVILIB_ON)
    {
        pthread_mutex_lock(&BufferDispo);
        switch(type)
        {
        case EVILIB_AUTO:
            sprintf(buffer, "8%d01043900FF", Id_cam);
            break;
        case EVILIB_MANUAL:
            sprintf(buffer, "8%d01043903FF", Id_cam);
            break;
        case EVILIB_SHUTTER_PRIO:
            sprintf(buffer, "8%d0104390AFF", Id_cam);
            break;
        case EVILIB_IRIS_PRIO:
            sprintf(buffer, "8%d0104390BFF", Id_cam);
#ifdef __EVI_D100__
        case EVILIB_GAIN_PRIO:
            sprintf(buffer, "8%d0104390CFF", Id_cam);
            break;
#endif // __EVI_D100__
        case EVILIB_BRIGHT:
            sprintf(buffer, "8%d0104390DFF", Id_cam);
            break;
#ifdef __EVI_D100__
        case EVILIB_SHUTTER_AUTO:
            sprintf(buffer, "8%d0104391AFF", Id_cam);
        case EVILIB_IRIS_AUTO:
            sprintf(buffer, "8%d0104391BFF", Id_cam);
        case EVILIB_GAIN_AUTO:
            sprintf(buffer, "8%d01043901CFF", Id_cam);
#endif // __EVI_D100__
        default:
            pthread_mutex_unlock(&BufferDispo);
            return 0;
        }
        if(SendCommand((unsigned char *)buffer, strlen(buffer)) == 0)
        {
            pthread_mutex_unlock(&BufferDispo);
            return 0;
        }
        pthread_mutex_unlock(&BufferDispo);
// receiv 'ACK'
        pthread_mutex_lock(&TakeAck);
// Wait for Completion
        if(waitC == EVILIB_WAIT_COMP)
        {
            waitComp = EVILIB_WAIT_COMP;
            pthread_mutex_lock(&TakeComp);
        }
        if(answer < 1)
            return 0;
        return 1;
    }
    return 0;
}

#ifdef __EVI_D100__
/*
 * type: EVILIB_AUTO
 *       DERVERV_MANUAL
 * waitC: wait for completion of command: EVILIB_NO_WAIT_COMP
 *                                        EVILIB_WAIT_COMP
 * Return 1 on success, 0 on error
 */
int EVILib::SlowShutter(int type, int waitC)
{
    if(power == EVILIB_ON)
    {
        pthread_mutex_lock(&BufferDispo);
        switch(type)
        {
        case EVILIB_AUTO:
            sprintf(buffer, "8%d01045A02FF", Id_cam);
            break;
        case EVILIB_MANUAL:
            sprintf(buffer, "8%d01045A03FF", Id_cam);
            break;
        default:
            pthread_mutex_unlock(&BufferDispo);
            return 0;
        }
        if(SendCommand((unsigned char *)buffer, strlen(buffer)) == 0)
        {
            pthread_mutex_unlock(&BufferDispo);
            return 0;
        }
        pthread_mutex_unlock(&BufferDispo);
// receiv 'ACK'
        pthread_mutex_lock(&TakeAck);
// Wait for Completion
        if(waitC == EVILIB_WAIT_COMP)
        {
            waitComp = EVILIB_WAIT_COMP;
            pthread_mutex_lock(&TakeComp);
        }
        if(answer < 1)
            return 0;
        return 1;
    }
    return 0;
}
#endif // __EVI_D100__

/*
 * When turning on to Bright Mode, Iris, Gain and Shutter at the time then increase or decrease
 * 3 dB/step using UP/DOWN command
 * type: EVILIB_RESET
 *       EVILIB_UP
 *       EVILIB_DOWN
 *       EVILIB_DIRECT
 * cmd: EVILIB_BRIGHT_CLOSE to EVILIB_BRIGHT_F1_8 (See the header of 'EVILib.h' for the authorized values)
 * waitC: wait for completion of command: EVILIB_NO_WAIT_COMP
 *                                        EVILIB_WAIT_COMP
 * Return 1 on success, 0 on error
 */
int EVILib::Bright(int type, int cmd, int waitC)
{
    if(power == EVILIB_ON)
    {
        pthread_mutex_lock(&BufferDispo);
        switch(type)
        {
        case EVILIB_RESET:
            sprintf(buffer, "8%d01040D00FF", Id_cam);
            break;
        case EVILIB_UP:
            sprintf(buffer, "8%d01040D02FF", Id_cam);
            break;
        case EVILIB_DOWN:
            sprintf(buffer, "8%d01040D03FF", Id_cam);
            break;
#ifdef __EVI_D100__
        case EVILIB_DIRECT:
            sprintf(buffer, "8%d01044D0000", Id_cam);
            switch(cmd)
            {
            case EVILIB_BRIGHT_CLOSE:
                strcat(buffer, "0000FF");
                break;
            case EVILIB_BRIGHT_F28:
                strcat(buffer, "0001FF");
                break;
            case EVILIB_BRIGHT_F22:
                strcat(buffer, "0002FF");
                break;
            case EVILIB_BRIGHT_F19:
                strcat(buffer, "0003FF");
                break;
            case EVILIB_BRIGHT_F16:
                strcat(buffer, "0004FF");
                break;
            case EVILIB_BRIGHT_F14:
                strcat(buffer, "0005FF");
                break;
            case EVILIB_BRIGHT_F11:
                strcat(buffer, "0006FF");
                break;
            case EVILIB_BRIGHT_F9_6:
                strcat(buffer, "0007FF");
                break;
            case EVILIB_BRIGHT_F8:
                strcat(buffer, "0008FF");
                break;
            case EVILIB_BRIGHT_F6_8:
                strcat(buffer, "0009FF");
                break;
            case EVILIB_BRIGHT_F5_6:
                strcat(buffer, "000AFF");
                break;
            case EVILIB_BRIGHT_F4_8:
                strcat(buffer, "000BFF");
                break;
            case EVILIB_BRIGHT_F4:
                strcat(buffer, "000CFF");
                break;
            case EVILIB_BRIGHT_F3_4:
                strcat(buffer, "000DFF");
                break;
            case EVILIB_BRIGHT_F2_8:
                strcat(buffer, "000EFF");
                break;
            case EVILIB_BRIGHT_F2_4:
                strcat(buffer, "000FFF");
                break;
            case EVILIB_BRIGHT_F2:
                strcat(buffer, "0100FF");
                break;
            case EVILIB_BRIGHT_F1_8_0:
                strcat(buffer, "0101FF");
                break;
            case EVILIB_BRIGHT_F1_8_3:
                strcat(buffer, "0102FF");
                break;
            case EVILIB_BRIGHT_F1_8_6:
                strcat(buffer, "0103FF");
                break;
            case EVILIB_BRIGHT_F1_8_9:
                strcat(buffer, "0104FF");
                break;
            case EVILIB_BRIGHT_F1_8_12:
                strcat(buffer, "0105FF");
                break;
            case EVILIB_BRIGHT_F1_8_15:
                strcat(buffer, "0106FF");
                break;
            case EVILIB_BRIGHT_F1_8_18:
                strcat(buffer, "0107FF");
                break;
            default:
                pthread_mutex_unlock(&BufferDispo);
                return 0;
            }
            break;
#endif // __EVI_D100__
        default:
            pthread_mutex_unlock(&BufferDispo);
            return 0;
        }
        if(SendCommand((unsigned char *)buffer, strlen(buffer)) == 0)
        {
            pthread_mutex_unlock(&BufferDispo);
            return 0;
        }
        pthread_mutex_unlock(&BufferDispo);
// receiv 'ACK'
        pthread_mutex_lock(&TakeAck);
// Wait for Completion
        if(waitC == EVILIB_WAIT_COMP)
        {
            waitComp = EVILIB_WAIT_COMP;
            pthread_mutex_lock(&TakeComp);
        }
        if(answer < 1)
            return 0;
        return 1;
    }
    return 0;
}

#ifdef __EVI_D100__
/*
 * type: EVILIB_ON
 *       EVILIB_OFF
 *       EVILIB_RESET
 *       EVILIB_UP
 *       EVILIB_DOWN
 *       EVILIB_DIRECT
 * cmd: ExpComp Position -7 (-10.5 dB) to 7 (10.5dB), 15 steps at 1.5dB
 * waitC: wait for completion of command: EVILIB_NO_WAIT_COMP
 *                                        EVILIB_WAIT_COMP
 * Return 1 on success, 0 on error
 */
int EVILib::ExpComp(int type, int cmd, int waitC)
{
    if(power == EVILIB_ON)
    {
        pthread_mutex_lock(&BufferDispo);
        switch(type)
        {
        case EVILIB_ON:
            sprintf(buffer, "8%d01043E02FF", Id_cam);
            break;
        case EVILIB_OFF:
            sprintf(buffer, "8%d01043E03FF", Id_cam);
            break;
        case EVILIB_RESET:
            sprintf(buffer, "8%d01040E00FF", Id_cam);
            break;
        case EVILIB_UP:
            sprintf(buffer, "8%d01040E02FF", Id_cam);
            break;
        case EVILIB_DOWN:
            sprintf(buffer, "8%d01040E03FF", Id_cam);
            break;
        case EVILIB_DIRECT:
           if(cmd >= -7 && cmd <= 7)
            {
                sprintf(buffer, "8%d01044E", Id_cam);
                make0XString(cmd + 7, &buffer[strlen(buffer)], 4);
                strcat(buffer, "FF");
            }
            else
            {
                pthread_mutex_unlock(&BufferDispo);
                return 0;
            }
            break;
        default:
            pthread_mutex_unlock(&BufferDispo);
            return 0;
        }
        if(SendCommand((unsigned char *)buffer, strlen(buffer)) == 0)
        {
            pthread_mutex_unlock(&BufferDispo);
            return 0;
        }
        pthread_mutex_unlock(&BufferDispo);
// receiv 'ACK'
        pthread_mutex_lock(&TakeAck);
// Wait for Completion
        if(waitC == EVILIB_WAIT_COMP)
        {
            waitComp = EVILIB_WAIT_COMP;
            pthread_mutex_lock(&TakeComp);
        }
        if(answer < 1)
            return 0;
        return 1;
    }
    return 0;
}
#endif // __EVI_D100__

/*
 * Electronic Shutter Setting
 * Enable on AE_Manual, Shutter_Priority
 * type: EVILIB_RESET
 *       EVILIB_UP
 *       EVILIB_DOWN
 *       EVILIB_DIRECT --> need 'speed'
 * speed: 1/60 to 1/10000 second
 * Authorized values: 60 - 75 - 90 - 100 - 125 - 150 - 180 - 215 - 250 - 300 - 350 -
 *   425 - 500 - 600 - 725 - 850 - 1000 - 1250 - 1500 - 1750 - 2000 - 2500 - 3000 -
 *   3500 - 4000 - 6000 - 10000
 * waitC: wait for completion of command: EVILIB_NO_WAIT_COMP
 *                                        EVILIB_WAIT_COMP
 * Return 1 on success, 0 on error
 */
int EVILib::Shutter(int type, int speed, int waitC)
{
    if(power == EVILIB_ON)
    {
        pthread_mutex_lock(&BufferDispo);
        switch(type)
        {
        case EVILIB_RESET:
            sprintf(buffer, "8%d01040A00FF", Id_cam);
            break;
        case EVILIB_UP:
            sprintf(buffer, "8%d01040A02FF", Id_cam);
            break;
        case EVILIB_DOWN:
            sprintf(buffer, "8%d01040A03FF", Id_cam);
            break;
// How to compute the speed automatically ?
        case EVILIB_DIRECT:
            sprintf(buffer, "8%d01044A0000", Id_cam);
            switch(speed)
            {
            case 60:
                strcat(buffer, "0000FF");
                break;
            case 75:
                strcat(buffer, "0002FF");
                break;
            case 90:
                strcat(buffer, "0003FF");
                break;
            case 100:
                strcat(buffer, "0004FF");
                break;
            case 125:
                strcat(buffer, "0005FF");
                break;
            case 150:
                strcat(buffer, "0006FF");
                break;
            case 180:
                strcat(buffer, "0007FF");
                break;
            case 215:
                strcat(buffer, "0008FF");
                break;
            case 250:
                strcat(buffer, "0009FF");
                break;
            case 300:
                strcat(buffer, "000AFF");
                break;
            case 350:
                strcat(buffer, "000BFF");
                break;
            case 425:
                strcat(buffer, "000CFF");
                break;
            case 500:
                strcat(buffer, "000DFF");
                break;
            case 600:
                strcat(buffer, "000EFF");
                break;
            case 725:
                strcat(buffer, "000FFF");
                break;
            case 850:
                strcat(buffer, "0100FF");
                break;
            case 1000:
                strcat(buffer, "0101FF");
                break;
            case 1250:
                strcat(buffer, "0102FF");
                break;
            case 1500:
                strcat(buffer, "0103FF");
                break;
            case 1750:
                strcat(buffer, "0104FF");
                break;
            case 2000:
                strcat(buffer, "0105FF");
                break;
            case 2500:
                strcat(buffer, "0106FF");
                break;
            case 3000:
                strcat(buffer, "0107FF");
                break;
            case 3500:
                strcat(buffer, "0108FF");
                break;
            case 4000:
                strcat(buffer, "0109FF");
                break;
            case 6000:
                strcat(buffer, "010AFF");
                break;
            case 10000:
                strcat(buffer, "010BFF");
                break;
            default:
                pthread_mutex_unlock(&BufferDispo);
                return 0;
            }
            break;
        default:
            pthread_mutex_unlock(&BufferDispo);
            return 0;
        }
        if(SendCommand((unsigned char *)buffer, strlen(buffer)) == 0)
        {
            pthread_mutex_unlock(&BufferDispo);
            return 0;
        }
        pthread_mutex_unlock(&BufferDispo);
// receiv 'ACK'
        pthread_mutex_lock(&TakeAck);
// Wait for Completion
        if(waitC == EVILIB_WAIT_COMP)
        {
            waitComp = EVILIB_WAIT_COMP;
            pthread_mutex_lock(&TakeComp);
        }
        if(answer < 1)
            return 0;
        return 1;
    }
    return 0;
}

/*
 * Iris Setting. Enable on AE_Manual or Iris_Priority
 * type: EVILIB_RESET
 *       EVILIB_UP
 *       EVILIB_DOWN
 *       EVILIB_DIRECT --> need 'cmd'
 * cmd: EVILIB_IRIS_CLOSE to EVILIB_IRIS_F1_8 (See the header of 'EVILib.h' for the authorized values)
 * waitC: wait for completion of command: EVILIB_NO_WAIT_COMP
 *                                        EVILIB_WAIT_COMP
 * Return 1 on success, 0 on error
 */
int EVILib::Iris(int type, int cmd, int waitC)
{
    if(power == EVILIB_ON)
    {
        pthread_mutex_lock(&BufferDispo);
        switch(type)
        {
        case EVILIB_RESET:
            sprintf(buffer, "8%d01040B00FF", Id_cam);
            break;
        case EVILIB_UP:
            sprintf(buffer, "8%d01040B02FF", Id_cam);
            break;
        case EVILIB_DOWN:
            sprintf(buffer, "8%d01040B03FF", Id_cam);
            break;
        case EVILIB_DIRECT:
            sprintf(buffer, "8%d01044B0000", Id_cam);
            switch(cmd)
            {
            case EVILIB_IRIS_CLOSE:
                strcat(buffer, "0000FF");
                break;
            case EVILIB_IRIS_F28:
                strcat(buffer, "0001FF");
                break;
            case EVILIB_IRIS_F22:
                strcat(buffer, "0002FF");
                break;
            case EVILIB_IRIS_F19:
                strcat(buffer, "0003FF");
                break;
            case EVILIB_IRIS_F16:
                strcat(buffer, "0004FF");
                break;
            case EVILIB_IRIS_F14:
                strcat(buffer, "0005FF");
                break;
            case EVILIB_IRIS_F11:
                strcat(buffer, "0006FF");
                break;
            case EVILIB_IRIS_F9_6:
                strcat(buffer, "0007FF");
                break;
            case EVILIB_IRIS_F8:
                strcat(buffer, "0008FF");
                break;
            case EVILIB_IRIS_F6_8:
                strcat(buffer, "0009FF");
                break;
            case EVILIB_IRIS_F5_6:
                strcat(buffer, "000AFF");
                break;
            case EVILIB_IRIS_F4_8:
                strcat(buffer, "000BFF");
                break;
            case EVILIB_IRIS_F4:
                strcat(buffer, "000CFF");
                break;
            case EVILIB_IRIS_F3_4:
                strcat(buffer, "000DFF");
                break;
            case EVILIB_IRIS_F2_8:
                strcat(buffer, "000EFF");
                break;
            case EVILIB_IRIS_F2_4:
                strcat(buffer, "000FFF");
                break;
            case EVILIB_IRIS_F2:
                strcat(buffer, "0100FF");
                break;
            case EVILIB_IRIS_F1_8:
                strcat(buffer, "0101FF");
                break;
            default:
                pthread_mutex_unlock(&BufferDispo);
                return 0;
            }
            break;
        default:
            pthread_mutex_unlock(&BufferDispo);
            return 0;
        }
        if(SendCommand((unsigned char *)buffer, strlen(buffer)) == 0)
        {
            pthread_mutex_unlock(&BufferDispo);
            return 0;
        }
        pthread_mutex_unlock(&BufferDispo);
// receiv 'ACK'
        pthread_mutex_lock(&TakeAck);
// Wait for Completion
        if(waitC == EVILIB_WAIT_COMP)
        {
            waitComp = EVILIB_WAIT_COMP;
            pthread_mutex_lock(&TakeComp);
        }
        if(answer < 1)
            return 0;
        return 1;
    }
    return 0;
}

/*
 * Gain Setting. Enable on AE_Manual only
 * type: EVILIB_RESET
 *       EVILIB_UP
 *       EVILIB_DOWN
 *       EVILIB_DIRECT --> need 'setting'
 * setting: 0 (-3 dB) to 7 (18 dB), 8 steps
 * waitC: wait for completion of command: EVILIB_NO_WAIT_COMP
 *                                        EVILIB_WAIT_COMP
 * Return 1 on success, 0 on error
 */
int EVILib::Gain(int type, int setting, int waitC)
{
    if(power == EVILIB_ON)
    {
        pthread_mutex_lock(&BufferDispo);
        switch(type)
        {
        case EVILIB_RESET:
            sprintf(buffer, "8%d01040C00FF", Id_cam);
            break;
        case EVILIB_UP:
            sprintf(buffer, "8%d01040C02FF", Id_cam);
            break;
        case EVILIB_DOWN:
            sprintf(buffer, "8%d01040C03FF", Id_cam);
            break;
        case EVILIB_DIRECT:
            if(setting >= 0 && setting <= 7)
            {
                sprintf(buffer, "8%d01044C", Id_cam);
                make0XString(setting, &buffer[strlen(buffer)], 4);
                strcat(buffer, "FF");
            }
            else
            {
                pthread_mutex_unlock(&BufferDispo);
                return 0;
            }
            break;
        default:
            pthread_mutex_unlock(&BufferDispo);
            return 0;
        }
        if(SendCommand((unsigned char *)buffer, strlen(buffer)) == 0)
        {
            pthread_mutex_unlock(&BufferDispo);
            return 0;
        }
        pthread_mutex_unlock(&BufferDispo);
// receiv 'ACK'
        pthread_mutex_lock(&TakeAck);
// Wait for Completion
        if(waitC == EVILIB_WAIT_COMP)
        {
            waitComp = EVILIB_WAIT_COMP;
            pthread_mutex_lock(&TakeComp);
        }
        if(answer < 1)
            return 0;
        return 1;
    }
    return 0;
}

/*
 * Backlight compensation.
 * Gain-up to 6 dB max.
 * type: EVILIB_ON
 *       EVILIB_OFF
 * waitC: wait for completion of command: EVILIB_NO_WAIT_COMP
 *                                        EVILIB_WAIT_COMP
 * Return 1 on success, 0 on error
 */
int EVILib::Backlight(int type, int waitC)
{
    if(power == EVILIB_ON)
    {
        pthread_mutex_lock(&BufferDispo);
        switch(type)
        {
        case EVILIB_ON:
            sprintf(buffer, "8%d01043302FF", Id_cam);
            break;
        case EVILIB_OFF:
            sprintf(buffer, "8%d01043303FF", Id_cam);
            break;
        default:
            pthread_mutex_unlock(&BufferDispo);
            return 0;
        }
        if(SendCommand((unsigned char *)buffer, strlen(buffer)) == 0)
        {
            pthread_mutex_unlock(&BufferDispo);
            return 0;
        }
        pthread_mutex_unlock(&BufferDispo);
// receiv 'ACK'
        pthread_mutex_lock(&TakeAck);
// Wait for Completion
        if(waitC == EVILIB_WAIT_COMP)
        {
            waitComp = EVILIB_WAIT_COMP;
            pthread_mutex_lock(&TakeComp);
        }
        if(answer < 1)
            return 0;
        return 1;
    }
    return 0;
}

#ifdef __EVI_D100__
/*
 * type: EVILIB_RESET
 *       EVILIB_UP
 *       EVILIB_DOWN
 *       EVILIB_DIRECT
 * cmd: Aperture Gain 0 to 15, 16 steps, Initial value: 5
 * waitC: wait for completion of command: EVILIB_NO_WAIT_COMP
 *                                        EVILIB_WAIT_COMP
 * Return 1 on success, 0 on error
 */
int EVILib::Aperture(int type, int cmd, int waitC)
{
    if(power == EVILIB_ON)
    {
        pthread_mutex_lock(&BufferDispo);
        switch(type)
        {
        case EVILIB_RESET:
            sprintf(buffer, "8%d01040200FF", Id_cam);
            break;
        case EVILIB_UP:
            sprintf(buffer, "8%d01040202FF", Id_cam);
            break;
        case EVILIB_DOWN:
            sprintf(buffer, "8%d01040203FF", Id_cam);
            break;
        case EVILIB_DIRECT:
            if(cmd >= 0 && cmd <= 15)
            {
                sprintf(buffer, "8%d010442", Id_cam);
                make0XString(cmd, &buffer[strlen(buffer)], 4);
                strcat(buffer, "FF");
            }
            else
            {
                pthread_mutex_unlock(&BufferDispo);
                return 0;
            }
            break;
        default:
            pthread_mutex_unlock(&BufferDispo);
            return 0;
        }
        if(SendCommand((unsigned char *)buffer, strlen(buffer)) == 0)
        {
            pthread_mutex_unlock(&BufferDispo);
            return 0;
        }
        pthread_mutex_unlock(&BufferDispo);
// receiv 'ACK'
        pthread_mutex_lock(&TakeAck);
// Wait for Completion
        if(waitC == EVILIB_WAIT_COMP)
        {
            waitComp = EVILIB_WAIT_COMP;
            pthread_mutex_lock(&TakeComp);
        }
        if(answer < 1)
            return 0;
        return 1;
    }
    return 0;
}

/*
 * Wide mode setting
 * type: EVILIB_OFF
 *       EVILIB_CINEMA
 *       EVILIB_16_9_FULL
 * waitC: wait for completion of command: EVILIB_NO_WAIT_COMP
 *                                        EVILIB_WAIT_COMP
 * Return 1 on success, 0 on error
 */
int EVILib::Wide(int type, int waitC)
{
    if(power == EVILIB_ON)
    {
        pthread_mutex_lock(&BufferDispo);
        switch(type)
        {
        case EVILIB_OFF:
            sprintf(buffer, "8%d01046000FF", Id_cam);
            break;
        case EVILIB_CINEMA:
            sprintf(buffer, "8%d01046001FF", Id_cam);
            break;
        case EVILIB_16_9_FULL:
            sprintf(buffer, "8%d01046002FF", Id_cam);
            break;
        default:
            pthread_mutex_unlock(&BufferDispo);
            return 0;
        }
        if(SendCommand((unsigned char *)buffer, strlen(buffer)) == 0)
        {
            pthread_mutex_unlock(&BufferDispo);
            return 0;
        }
        pthread_mutex_unlock(&BufferDispo);
// receiv 'ACK'
        pthread_mutex_lock(&TakeAck);
// Wait for Completion
        if(waitC == EVILIB_WAIT_COMP)
        {
            waitComp = EVILIB_WAIT_COMP;
            pthread_mutex_lock(&TakeComp);
        }
        if(answer < 1)
            return 0;
        return 1;
    }
    return 0;
}

/*
 * Mirror image ON/OFF
 * type: EVILIB_ON
 *       EVILIB_OFF
 * waitC: wait for completion of command: EVILIB_NO_WAIT_COMP
 *                                        EVILIB_WAIT_COMP
 * Return 1 on success, 0 on error
 */
int EVILib::LR_Reverse(int type, int waitC)
{
    if(power == EVILIB_ON)
    {
        pthread_mutex_lock(&BufferDispo);
        switch(type)
        {
        case EVILIB_ON:
            sprintf(buffer, "8%d01046102FF", Id_cam);
            break;
        case EVILIB_OFF:
            sprintf(buffer, "8%d01046103FF", Id_cam);
            break;
        default:
            pthread_mutex_unlock(&BufferDispo);
            return 0;
        }
        if(SendCommand((unsigned char *)buffer, strlen(buffer)) == 0)
        {
            pthread_mutex_unlock(&BufferDispo);
            return 0;
        }
        pthread_mutex_unlock(&BufferDispo);
// receiv 'ACK'
        pthread_mutex_lock(&TakeAck);
// Wait for Completion
        if(waitC == EVILIB_WAIT_COMP)
        {
            waitComp = EVILIB_WAIT_COMP;
            pthread_mutex_lock(&TakeComp);
        }
        if(answer < 1)
            return 0;
        return 1;
    }
    return 0;
}

/*
 * Still image ON/OFF
 * type: EVILIB_ON
 *       EVILIB_OFF
 * waitC: wait for completion of command: EVILIB_NO_WAIT_COMP
 *                                        EVILIB_WAIT_COMP
 * Return 1 on success, 0 on error
 */
int EVILib::Freeze(int type, int waitC)
{
    if(power == EVILIB_ON)
    {
        pthread_mutex_lock(&BufferDispo);
        switch(type)
        {
        case EVILIB_ON:
            sprintf(buffer, "8%d01046202FF", Id_cam);
            break;
        case EVILIB_OFF:
            sprintf(buffer, "8%d01046203FF", Id_cam);
            break;
        default:
            pthread_mutex_unlock(&BufferDispo);
            return 0;
        }
        if(SendCommand((unsigned char *)buffer, strlen(buffer)) == 0)
        {
            pthread_mutex_unlock(&BufferDispo);
            return 0;
        }
        pthread_mutex_unlock(&BufferDispo);
// receiv 'ACK'
        pthread_mutex_lock(&TakeAck);
// Wait for Completion
        if(waitC == EVILIB_WAIT_COMP)
        {
            waitComp = EVILIB_WAIT_COMP;
            pthread_mutex_lock(&TakeComp);
        }
        if(answer < 1)
            return 0;
        return 1;
    }
    return 0;
}

/*
 * Picture effect setting
 * type: EVILIB_OFF
 *       EVILIB_PASTEL
 *       EVILIB_NEGART
 *       EVILIB_SEPIA
 *       EVILIB_BW
 *       EVILIB_SOLARIZE
 *       EVILIB_MOSAIC
 *       EVILIB_SLIM
 *       EVILIB_STRETCH
 * waitC: wait for completion of command: EVILIB_NO_WAIT_COMP
 *                                        EVILIB_WAIT_COMP
 * Return 1 on success, 0 on error
 */
int EVILib::PictureEffect(int type, int waitC)
{
    if(power == EVILIB_ON)
    {
        pthread_mutex_lock(&BufferDispo);
        switch(type)
        {
        case EVILIB_OFF:
            sprintf(buffer, "8%d01046300FF", Id_cam);
            break;
        case EVILIB_PASTEL:
            sprintf(buffer, "8%d01046301FF", Id_cam);
            break;
        case EVILIB_NEGART:
            sprintf(buffer, "8%d01046302FF", Id_cam);
            break;
        case EVILIB_SEPIA:
            sprintf(buffer, "8%d01046303FF", Id_cam);
            break;
        case EVILIB_BW:
            sprintf(buffer, "8%d01046304FF", Id_cam);
            break;
        case EVILIB_SOLARIZE:
            sprintf(buffer, "8%d01046305FF", Id_cam);
            break;
        case EVILIB_MOSAIC:
            sprintf(buffer, "8%d01046306FF", Id_cam);
            break;
        case EVILIB_SLIM:
            sprintf(buffer, "8%d01046307FF", Id_cam);
            break;
        case EVILIB_STRETCH:
            sprintf(buffer, "8%d01046308FF", Id_cam);
            break;
        default:
            pthread_mutex_unlock(&BufferDispo);
            return 0;
        }
        if(SendCommand((unsigned char *)buffer, strlen(buffer)) == 0)
        {
            pthread_mutex_unlock(&BufferDispo);
            return 0;
        }
        pthread_mutex_unlock(&BufferDispo);
// receiv 'ACK'
        pthread_mutex_lock(&TakeAck);
// Wait for Completion
        if(waitC == EVILIB_WAIT_COMP)
        {
            waitComp = EVILIB_WAIT_COMP;
            pthread_mutex_lock(&TakeComp);
        }
        if(answer < 1)
            return 0;
        return 1;
    }
    return 0;
}

/*
 * Digital effect setting
 * type: EVILIB_OFF
 *       EVILIB_STILL
 *       EVILIB_FLASH
 *       EVILIB_LUMI
 *       EVILIB_TRAIL
 *       EVILIB_EFFECTLEVEL -> need 'effect'
 * waitC: wait for completion of command: EVILIB_NO_WAIT_COMP
 *                                        EVILIB_WAIT_COMP
 * effect: effect level 0 to 24 (flash, trail), 0 to 32 (still, lumi)
 */
int EVILib::DigitalEffect(int type, int level, int waitC)
{
    if(power == EVILIB_ON)
    {
        pthread_mutex_lock(&BufferDispo);
        switch(type)
        {
        case EVILIB_OFF:
            sprintf(buffer, "8%d01046400FF", Id_cam);
            break;
        case EVILIB_STILL:
            sprintf(buffer, "8%d01046401FF", Id_cam);
            break;
        case EVILIB_FLASH:
            sprintf(buffer, "8%d01046402FF", Id_cam);
            break;
        case EVILIB_LUMI:
            sprintf(buffer, "8%d01046403FF", Id_cam);
            break;
        case EVILIB_TRAIL:
            sprintf(buffer, "8%d01046404FF", Id_cam);
            break;
        case EVILIB_EFFECTLEVEL:
            if(level >= 0 && level <= 32)
            {
                if(level <= 15)
                    sprintf(buffer, "8%d0104650%XFF", Id_cam, level);
                else
                    sprintf(buffer, "8%d010465%XFF", Id_cam, level);
            }
            else
            {
                pthread_mutex_unlock(&BufferDispo);
                return 0;
            }
            break;
        default:
            pthread_mutex_unlock(&BufferDispo);
            return 0;
        }
        if(SendCommand((unsigned char *)buffer, strlen(buffer)) == 0)
        {
            pthread_mutex_unlock(&BufferDispo);
            return 0;
        }
        pthread_mutex_unlock(&BufferDispo);
// receiv 'ACK'
        pthread_mutex_lock(&TakeAck);
// Wait for Completion
        if(waitC == EVILIB_WAIT_COMP)
        {
            waitComp = EVILIB_WAIT_COMP;
            pthread_mutex_lock(&TakeComp);
        }
        if(answer < 1)
            return 0;
        return 1;
    }
    return 0;
}
#endif // __EVI_D100__

/*
 * Preset memory for memorize camera condition
 * type: EVILIB_RESET --> need 'position'
 *       EVILIB_SET --> need 'position'
 *       EVILIB_RECALL -- need 'position'
 * position: 0 to 5
 * waitC: wait for completion of command: EVILIB_NO_WAIT_COMP
 *                                        EVILIB_WAIT_COMP
 * Return 1 on success, 0 on error
 */
int EVILib::Memory(int type, int position, int waitC)
{
    if(power == EVILIB_ON)
    {
        if(position >= 0 && position <= 5)
        {
            pthread_mutex_lock(&BufferDispo);
            switch(type)
            {
            case EVILIB_RESET:
                sprintf(buffer, "8%d01043F000%XFF", Id_cam, position);
                break;
            case EVILIB_SET:
                sprintf(buffer, "8%d01043F010%XFF", Id_cam, position);
                break;
            case EVILIB_RECALL:
                sprintf(buffer, "8%d01043F020%XFF", Id_cam, position);
                break;
            default:
                pthread_mutex_unlock(&BufferDispo);
                return 0;
            }
            if(SendCommand((unsigned char *)buffer, strlen(buffer)) == 0)
            {
                pthread_mutex_unlock(&BufferDispo);
                return 0;
            }
            pthread_mutex_unlock(&BufferDispo);
// receiv 'ACK'
            pthread_mutex_lock(&TakeAck);
// Wait for Completion
            if(waitC == EVILIB_WAIT_COMP)
            {
                waitComp = EVILIB_WAIT_COMP;
                pthread_mutex_lock(&TakeComp);
            }
            if(answer < 1)
                return 0;
            return 1;
        }
    }
    return 0;
}

#ifdef __EVI_D30__
/*
 * Enable/Disable for RS-232C and key control
 * type: EVILIB_ON
 *       EVILIB_OFF
 * waitC: wait for completion of command: EVILIB_NO_WAIT_COMP
 *                                        EVILIB_WAIT_COMP
 * Return 1 on success, 0 on error
 */
int EVILib::KeyLock(int type, int waitC)
{
    if(power == EVILIB_ON)
    {
        pthread_mutex_lock(&BufferDispo);
        switch(type)
        {
        case EVILIB_OFF:
            sprintf(buffer, "8%d01041700FF", Id_cam);
            break;
        case EVILIB_ON:
            sprintf(buffer, "8%d01041702FF", Id_cam);
            break;
        default:
            pthread_mutex_unlock(&BufferDispo);
            return 0;
        }
        if(SendCommand((unsigned char *)buffer, strlen(buffer)) == 0)
        {
            pthread_mutex_unlock(&BufferDispo);
            return 0;
        }
        pthread_mutex_unlock(&BufferDispo);
// receiv 'ACK'
        pthread_mutex_lock(&TakeAck);
// Wait for Completion
        if(waitC == EVILIB_WAIT_COMP)
        {
            waitComp = EVILIB_WAIT_COMP;
            pthread_mutex_lock(&TakeComp);
        }
        if(answer < 1)
            return 0;
        return 1;
    }
    return 0;
}
#endif // __EVI_D30__

/*
 * Enable/Disable for IR remote commander
 * type: EVILIB_ON
 *       EVILIB_OFF
 *       EVILIB_ON_OFF
 * waitC: wait for completion of command: EVILIB_NO_WAIT_COMP
 *                                        EVILIB_WAIT_COMP
 * Return 1 on success, 0 on error
 */
int EVILib::IR_Receive(int type, int waitC)
{
    if(power == EVILIB_ON)
    {
        pthread_mutex_lock(&BufferDispo);
        switch(type)
        {
        case EVILIB_ON:
            sprintf(buffer, "8%d01060802FF", Id_cam);
            break;
        case EVILIB_OFF:
            sprintf(buffer, "8%d01060803FF", Id_cam);
            break;
        case EVILIB_ON_OFF:
            sprintf(buffer, "8%d01060810FF", Id_cam);
            break;
        default:
            pthread_mutex_unlock(&BufferDispo);
            return 0;
        }
        if(SendCommand((unsigned char *)buffer, strlen(buffer)) == 0)
        {
            pthread_mutex_lock(&BufferDispo);
            return 0;
        }
        pthread_mutex_unlock(&BufferDispo);
// receiv 'ACK'
        pthread_mutex_lock(&TakeAck);
// Wait for Completion
        if(waitC == EVILIB_WAIT_COMP)
        {
            waitComp = EVILIB_WAIT_COMP;
            pthread_mutex_lock(&TakeComp);
        }
        if(answer < 1)
            return 0;
        return 1;
    }
    return 0;
}

/*
 * Send replies what command received from IR Commander
 * type: EVILIB_ON
 *       EVILIB_OFF
 * waitC: wait for completion of command: EVILIB_NO_WAIT_COMP
 *                                        EVILIB_WAIT_COMP
 * Return 1 on success, 0 on error
 */
int EVILib::IR_ReceiveReturn(int type, int waitC)
{
    if(power == EVILIB_ON)
    {
        pthread_mutex_lock(&BufferDispo);
        switch(type)
        {
        case EVILIB_ON:
            sprintf(buffer, "8%d017D01030000FF", Id_cam);
            break;
        case EVILIB_OFF:
            sprintf(buffer, "8%d017D01130000FF", Id_cam);
            break;
        default:
            pthread_mutex_unlock(&BufferDispo);
            return 0;
        }
        if(SendCommand((unsigned char *)buffer, strlen(buffer)) == 0)
        {
            pthread_mutex_unlock(&BufferDispo);
            return 0;
        }
        pthread_mutex_unlock(&BufferDispo);
// receiv 'ACK'
        pthread_mutex_lock(&TakeAck);
// Wait for Completion
        if(waitC == EVILIB_WAIT_COMP)
        {
            waitComp = EVILIB_WAIT_COMP;
            pthread_mutex_lock(&TakeComp);
        }
        if(answer < 1)
            return 0;
        return 1;
    }
    return 0;
}

#ifdef __EVI_D30__
/*
 * Automatic Target Trace ability Compensation when a wide conversion lens is installed
 * setting: 0 (no conversion) to 7 (0.6 conversion)
 * waitC: wait for completion of command: EVILIB_NO_WAIT_COMP
 *                                        EVILIB_WAIT_COMP
 * Return 1 on success, 0 on error
 */
int EVILib::Wide_conLensSet(int setting, int waitC)
{
    if(power == EVILIB_ON)
    {
        pthread_mutex_lock(&BufferDispo);
        sprintf(buffer, "8%d01072600", Id_cam);
        switch(setting)
        {
        case 0: // Wide converstion lens ratio = 1.0
            strcat(buffer, "00FF");
            break;
        case 1: // Wide converstion lens ratio = 0.9
            strcat(buffer, "01FF");
            break;
        case 2: // Wide converstion lens ratio = 0.85
            strcat(buffer, "02FF");
            break;
        case 3: // Wide converstion lens ratio = 0.8
            strcat(buffer, "03FF");
            break;
        case 4: // Wide converstion lens ratio = 0.75
            strcat(buffer, "04FF");
            break;
        case 5: // Wide converstion lens ratio = 0.7
            strcat(buffer, "05FF");
            break;
        case 6: // Wide converstion lens ratio = 0.65
            strcat(buffer, "06FF");
            break;
        case 7: // Wide converstion lens ratio = 0.6
            strcat(buffer, "07FF");
            break;
        default:
            pthread_mutex_unlock(&BufferDispo);
            return 0;
        }
        if(SendCommand((unsigned char *)buffer, strlen(buffer)) == 0)
        {
            pthread_mutex_unlock(&BufferDispo);
            return 0;
        }
        pthread_mutex_unlock(&BufferDispo);
// receiv 'ACK'
        pthread_mutex_lock(&TakeAck);
// Wait for Completion
        if(waitC == EVILIB_WAIT_COMP)
        {
            waitComp = EVILIB_WAIT_COMP;
            pthread_mutex_lock(&TakeComp);
        }
        if(answer < 1)
            return 0;
        return 1;
    }
    return 0;
}
#endif // __EVI_D30__

/*
 * type: EVILIB_UP --> need 'pan_speed' & 'tilt_speed'
 *       EVILIB_DOWN --> need 'pan_speed' & 'tilt_speed'
 *       EVILIB_LEFT --> need 'pan_speed' & 'tilt_speed'
 *       EVILIB_RIGHT --> need 'pan_speed' & 'tilt_speed'
 *       EVILIB_UPLEFT --> need 'pan_speed' & 'tilt_speed'
 *       EVILIB_UPRIGHT --> need 'pan_speed' & 'tilt_speed'
 *       EVILIB_DOWNLEFT --> need 'pan_speed' & 'tilt_speed'
 *       EVILIB_DOWNRIGHT --> need 'pan_speed' & 'tilt_speed'
 *       EVILIB_STOP --> need 'pan_speed' & 'tilt_speed'
 *       EVILIB_ABSOLUTE --> need 'pan_speed' & 'tilt_speed' & 'pan_pos' & 'tilt_pos'
 *       EVILIB_RELATIVE --> need 'pan_speed' & 'tilt_speed' & 'pan_pos' & 'tilt_pos'
 *       EVILIB_HOME
 *       EVILIB_RESET
 * pan_speed: pan speed EVILIB_min_pspeed to EVILIB_max_pspeed
 * tilt_speed: tilt speed EVILIB_min_tspeed to EVILIB_max_tspeed
 * pan_pos: pan position: approx. -EVILIB_maxpan to +EVILIB_maxpan
 * tilt_pos: tilt position: approx. -EVILIB_mintilt to +EVILIB_maxtilt
 * waitC: wait for completion of command: EVILIB_NO_WAIT_COMP
 *                                        EVILIB_WAIT_COMP
 * Return 1 on success, 0 on error
 */
int EVILib::Pan_TiltDrive(int type, int pan_speed, int tilt_speed, float pan_pos, float tilt_pos, int waitC)
{
    if(power == EVILIB_ON)
    {
        pthread_mutex_lock(&BufferDispo);
        switch(type)
        {
        case EVILIB_HOME:
            sprintf(buffer, "8%d010604FF", Id_cam);
            break;
        case EVILIB_RESET:
            sprintf(buffer, "8%d010605FF", Id_cam);
            break;
        default:
            if(pan_speed >= EVILIB_min_pspeed && pan_speed <= EVILIB_max_pspeed &&
               tilt_speed >= EVILIB_min_tspeed && tilt_speed <= EVILIB_max_tspeed)
            {
                switch(type)
                {
                case EVILIB_ABSOLUTE:
                    sprintf(buffer, "8%d010602%X%X", Id_cam, pan_speed, tilt_speed);
                    make0XString(convert(EVILIB_PAN, pan_pos, EVILIB_maxpan, EVILIB_maxPan), &buffer[strlen(buffer)], 4);
                    make0XString(convert(EVILIB_TILT, tilt_pos, EVILIB_maxtilt, EVILIB_maxTilt), &buffer[strlen(buffer)], 4);
                    strcat(buffer, "FF");
                    break;
                case EVILIB_RELATIVE:
                    sprintf(buffer, "8%d010603%X%X", Id_cam, pan_speed, tilt_speed);
                    make0XString(convert(EVILIB_PAN, pan_pos, EVILIB_maxpan, EVILIB_maxPan), &buffer[strlen(buffer)], 4);
                    make0XString(convert(EVILIB_TILT, tilt_pos, EVILIB_maxtilt, EVILIB_maxTilt), &buffer[strlen(buffer)], 4);
                    strcat(buffer, "FF");
                    break;
                default:
                    sprintf(buffer, "8%d010601%X%X", Id_cam, pan_speed, tilt_speed);
                    switch(type)
                    {
                    case EVILIB_UP:
                        strcat(buffer, "0301FF");
                        break;
                    case EVILIB_DOWN:
                        strcat(buffer, "0302FF");
                        break;
                    case EVILIB_LEFT:
                        strcat(buffer, "0103FF");
                        break;
                    case EVILIB_RIGHT:
                        strcat(buffer, "0203FF");
                        break;
                    case EVILIB_UPLEFT:
                        strcat(buffer, "0101FF");
                        break;
                    case EVILIB_UPRIGHT:
                        strcat(buffer, "0201FF");
                        break;
                    case EVILIB_DOWNLEFT:
                        strcat(buffer, "0102FF");
                        break;
                    case EVILIB_DOWNRIGHT:
                        strcat(buffer, "0202FF");
                        break;
                    case EVILIB_STOP:
                        strcat(buffer, "0303FF");
                        break;
                    default:
                        pthread_mutex_unlock(&BufferDispo);
                        return 0;
                    }
                }
                break;
            }
            pthread_mutex_unlock(&BufferDispo);
            return 0;
        }
        if(SendCommand((unsigned char *)buffer, strlen(buffer)) == 0)
        {
            pthread_mutex_unlock(&BufferDispo);
            return 0;
        }
        pthread_mutex_unlock(&BufferDispo);
// receiv 'ACK'
        pthread_mutex_lock(&TakeAck);
// Wait for Completion
        if(waitC == EVILIB_WAIT_COMP)
        {
            waitComp = EVILIB_WAIT_COMP;
            pthread_mutex_lock(&TakeComp);
        }
        if(answer < 1)
            return 0;
        return 1;
    }
    return 0;
}

/*
 * Pan/Tilt limit set
 * type: EVILIB_SET --> need 'mode' & 'pan_pos' & 'tilt_pos'
 *       EVILIB_CLEAR --> need 'mode' & 'pan_pos' & 'tilt_pos'
 * mode: EVILIB_UPRIGHT
 *       EVILIB_DOWNLEFT
 * pan_pos: pan position: approx. -EVILIB_maxpan to +EVILIB_maxpan
 * tilt_pos: tilt position: approx. -EVILIB_mintilt to +EVILIB_maxtilt
 * waitC: wait for completion of command: EVILIB_NO_WAIT_COMP
 *                                        EVILIB_WAIT_COMP
 * Return 1 on success, 0 on error
 */
int EVILib::Pan_TiltLimitSet(int type, int mode, float pan_pos, float tilt_pos, int waitC)
{
    if(power == EVILIB_ON)
    {
        pthread_mutex_lock(&BufferDispo);
        switch(type)
        {
        case EVILIB_SET:
            switch(mode)
            {
            case EVILIB_UPRIGHT:
                sprintf(buffer, "8%d010607000%d", Id_cam, 1);
                break;
            case EVILIB_DOWNLEFT:
                sprintf(buffer, "8%d010607000%d", Id_cam, 0);
                break;
            default:
                pthread_mutex_unlock(&BufferDispo);
                return 0;
            }
            make0XString(convert(EVILIB_PAN, pan_pos, EVILIB_maxpan, EVILIB_maxPan), &buffer[strlen(buffer)], 4);
            make0XString(convert(EVILIB_TILT, tilt_pos, EVILIB_maxtilt, EVILIB_maxTilt), &buffer[strlen(buffer)], 4);
            strcat(buffer, "FF");
            break;
        case EVILIB_CLEAR:
            switch(mode)
            {
            case EVILIB_UPRIGHT:
                sprintf(buffer, "8%d010607010%d", Id_cam, 1);
                break;
            case EVILIB_DOWNLEFT:
                sprintf(buffer, "8%d010607010%d", Id_cam, 0);
                break;
            default:
                pthread_mutex_unlock(&BufferDispo);
                return 0;
            }
            make0XString(convert(EVILIB_PAN, pan_pos, EVILIB_maxpan, EVILIB_maxPan), &buffer[strlen(buffer)], 4);
            make0XString(convert(EVILIB_TILT, tilt_pos, EVILIB_maxtilt, EVILIB_maxTilt), &buffer[strlen(buffer)], 4);
            strcat(buffer, "FF");
            break;
        default:
            pthread_mutex_unlock(&BufferDispo);
            return 0;
        }
        if(SendCommand((unsigned char *)buffer, strlen(buffer)) == 0)
        {
            pthread_mutex_unlock(&BufferDispo);
            return 0;
        }
        pthread_mutex_unlock(&BufferDispo);
// receiv 'ACK'
        pthread_mutex_lock(&TakeAck);
// Wait for Completion
        if(waitC == EVILIB_WAIT_COMP)
        {
            waitComp = EVILIB_WAIT_COMP;
            pthread_mutex_lock(&TakeComp);
        }
        if(answer < 1)
            return 0;
        return 1;
    }
    return 0;
}

/*
 * On screen Data Display ON/OFF
 * type: EVILIB_ON
 *       EVILIB_OFF
 *       EVILIB_ON_OFF
 * waitC: wait for completion of command: EVILIB_NO_WAIT_COMP
 *                                        EVILIB_WAIT_COMP
 * Return 1 on success, 0 on error
 */
int EVILib::Datascreen(int type, int waitC)
{
    if(power == EVILIB_ON)
    {
        pthread_mutex_lock(&BufferDispo);
        switch(type)
        {
        case EVILIB_ON:
            sprintf(buffer, "8%d01060602FF", Id_cam);
            break;
        case EVILIB_OFF:
            sprintf(buffer, "8%d01060603FF", Id_cam);
            break;
        case EVILIB_ON_OFF:
            sprintf(buffer, "8%d01060610FF", Id_cam);
            break;
        default:
            pthread_mutex_unlock(&BufferDispo);
            return 0;
        }
        if(SendCommand((unsigned char *)buffer, strlen(buffer)) == 0)
        {
            pthread_mutex_unlock(&BufferDispo);
            return 0;
        }
        pthread_mutex_unlock(&BufferDispo);
// receiv 'ACK'
        pthread_mutex_lock(&TakeAck);
// Wait for Completion
        if(waitC == EVILIB_WAIT_COMP)
        {
            waitComp = EVILIB_WAIT_COMP;
            pthread_mutex_unlock(&TakeComp);
        }
        if(answer < 1)
            return 0;
        return 1;
    }
    return 0;
}

#ifdef __EVI_D30__
/*
 * Target Tracking Mode ON/OFF
 * type: EVILIB_ON
 *       EVILIB_OFF
 *       EVILIB_ON_OFF
 * waitC: wait for completion of command: EVILIB_NO_WAIT_COMP
 *                                        EVILIB_WAIT_COMP
 * Return 1 on success, 0 on error
 */
int EVILib::AT_Mode(int type, int waitC)
{
    if(power == EVILIB_ON)
    {
        pthread_mutex_lock(&BufferDispo);
        switch(type)
        {
        case EVILIB_ON:
            sprintf(buffer, "8%d01070102FF", Id_cam);
            break;
        case EVILIB_OFF:
            sprintf(buffer, "8%d01070103FF", Id_cam);
            break;
        case EVILIB_ON_OFF:
            sprintf(buffer, "8%d01070110FF", Id_cam);
            break;
        default:
            pthread_mutex_unlock(&BufferDispo);
            return 0;
        }
        if(SendCommand((unsigned char *)buffer, strlen(buffer)) == 0)
        {
            pthread_mutex_unlock(&BufferDispo);
            return 0;
        }
        pthread_mutex_unlock(&BufferDispo);
// receiv 'ACK'
        pthread_mutex_lock(&TakeAck);
// Wait for Completion
        if(waitC == EVILIB_WAIT_COMP)
        {
            waitComp = EVILIB_WAIT_COMP;
            pthread_mutex_lock(&TakeComp);
        }
        if(answer < 1)
            return 0;
        return 1;
    }
    return 0;
}

/*
 * Auto Exposure for the target
 * type: EVILIB_ON
 *       EVILIB_OFF
 *       EVILIB_ON_OFF
 * waitC: wait for completion of command: EVILIB_NO_WAIT_COMP
 *                                        EVILIB_WAIT_COMP
 * Return 1 on success, 0 on error
 */
int EVILib::AT_AE(int type, int waitC)
{
    if(power == EVILIB_ON)
    {
        pthread_mutex_lock(&BufferDispo);
        switch(type)
        {
        case EVILIB_ON:
            sprintf(buffer, "8%d01070202FF", Id_cam);
            break;
        case EVILIB_OFF:
            sprintf(buffer, "8%d01070203FF", Id_cam);
            break;
        case EVILIB_ON_OFF:
            sprintf(buffer, "8%d01070210FF", Id_cam);
            break;
        default:
            pthread_mutex_unlock(&BufferDispo);
            return 0;
        }
        if(SendCommand((unsigned char *)buffer, strlen(buffer)) == 0)
        {
            pthread_mutex_unlock(&BufferDispo);
            return 0;
        }
        pthread_mutex_unlock(&BufferDispo);
// receiv 'ACK'
        pthread_mutex_lock(&TakeAck);
// Wait for Completion
        if(waitC == EVILIB_WAIT_COMP)
        {
            waitComp = EVILIB_WAIT_COMP;
            pthread_mutex_lock(&TakeComp);
        }
        if(answer < 1)
            return 0;
        return 1;
    }
    return 0;
}

/*
 * Automatic Zooming for the target
 * type: EVILIB_ON
 *       EVILIB_OFF
 *       EVILIB_ON_OFF
 * waitC: wait for completion of command: EVILIB_NO_WAIT_COMP
 *                                        EVILIB_WAIT_COMP
 * Return 1 on success, 0 on error
 */
int EVILib::AT_AutoZoom(int type, int waitC)
{
    if(power == EVILIB_ON)
    {
        pthread_mutex_lock(&BufferDispo);
        switch(type)
        {
        case EVILIB_ON:
            sprintf(buffer, "8%d01070302FF", Id_cam);
            break;
        case EVILIB_OFF:
            sprintf(buffer, "8%d01070303FF", Id_cam);
            break;
        case EVILIB_ON_OFF:
            sprintf(buffer, "8%d01070310FF", Id_cam);
            break;
        default:
            pthread_mutex_unlock(&BufferDispo);
            return 0;
        }
        if(SendCommand((unsigned char *)buffer, strlen(buffer)) == 0)
        {
            pthread_mutex_unlock(&BufferDispo);
            return 0;
        }
        pthread_mutex_unlock(&BufferDispo);
// receiv 'ACK'
        pthread_mutex_lock(&TakeAck);
// Wait for Completion
        if(waitC == EVILIB_WAIT_COMP)
        {
            waitComp = EVILIB_WAIT_COMP;
            pthread_mutex_lock(&TakeComp);
        }
        if(answer < 1)
            return 0;
        return 1;
    }
    return 0;
}

/*
 * Sensing Frame Display ON/OFF
 * type: EVILIB_ON
 *       EVILIB_OFF
 *       EVILIB_ON_OFF
 * waitC: wait for completion of command: EVILIB_NO_WAIT_COMP
 *                                        EVILIB_WAIT_COMP
 * Return 1 on success, 0 on error
 */
int EVILib::AT_MD_Frame_Display(int type, int waitC)
{
    if(power == EVILIB_ON)
    {
        pthread_mutex_lock(&BufferDispo);
        switch(type)
        {
        case EVILIB_ON:
            sprintf(buffer, "8%d01070402FF", Id_cam);
            break;
        case EVILIB_OFF:
            sprintf(buffer, "8%d01070403FF", Id_cam);
            break;
        case EVILIB_ON_OFF:
            sprintf(buffer, "8%d01070410FF", Id_cam);
            break;
        default:
            pthread_mutex_unlock(&BufferDispo);
            return 0;
        }
        if(SendCommand((unsigned char *)buffer, strlen(buffer)) == 0)
        {
            pthread_mutex_unlock(&BufferDispo);
            return 0;
        }
        pthread_mutex_unlock(&BufferDispo);
// receiv 'ACK'
        pthread_mutex_lock(&TakeAck);
// Wait for Completion
        if(waitC == EVILIB_WAIT_COMP)
        {
            waitComp = EVILIB_WAIT_COMP;
            pthread_mutex_lock(&TakeComp);
        }
        if(answer < 1)
            return 0;
        return 1;
    }
    return 0;
}

/*
 * Shifting the Sensing Frame for AR
 * For Shifting use Pan/Tilt Drive Command
 * type: EVILIB_ON
 *       EVILIB_OFF
 *       EVILIB_ON_OFF
 * waitC: wait for completion of command: EVILIB_NO_WAIT_COMP
 *                                        EVILIB_WAIT_COMP
 * Return 1 on success, 0 on error
 */
int EVILib::AT_Offset(int type, int waitC)
{
    if(power == EVILIB_ON)
    {
        pthread_mutex_lock(&BufferDispo);
        switch(type)
        {
        case EVILIB_ON:
            sprintf(buffer, "8%d01070502FF", Id_cam);
            break;
        case EVILIB_OFF:
            sprintf(buffer, "8%d01070503FF", Id_cam);
            break;
        case EVILIB_ON_OFF:
            sprintf(buffer, "8%d01070510FF", Id_cam);
            break;
        default:
            pthread_mutex_unlock(&BufferDispo);
            return 0;
        }
        if(SendCommand((unsigned char *)buffer, strlen(buffer)) == 0)
        {
            pthread_mutex_unlock(&BufferDispo);
            return 0;
        }
        pthread_mutex_unlock(&BufferDispo);
// receiv 'ACK'
        pthread_mutex_lock(&TakeAck);
// Wait for Completion
        if(waitC == EVILIB_WAIT_COMP)
        {
            waitComp = EVILIB_WAIT_COMP;
            pthread_mutex_lock(&TakeComp);
        }
        if(answer < 1)
            return 0;
        return 1;
    }
    return 0;
}

/*
 * Tracking or Detecting Start/Stop
 * waitC: wait for completion of command: EVILIB_NO_WAIT_COMP
 *                                        EVILIB_WAIT_COMP
 * Return 1 on success, 0 on error
 */
int EVILib::AT_MD_StartStop(int waitC)
{
    if(power == EVILIB_ON)
    {
        pthread_mutex_lock(&BufferDispo);
        sprintf(buffer, "8%d01070610FF", Id_cam);
        if(SendCommand((unsigned char *)buffer, strlen(buffer)) == 0)
        {
            pthread_mutex_unlock(&BufferDispo);
            return 0;
        }
        pthread_mutex_unlock(&BufferDispo);
// receiv 'ACK'
        pthread_mutex_lock(&TakeAck);
// Wait for Completion
        if(waitC == EVILIB_WAIT_COMP)
        {
            waitComp = EVILIB_WAIT_COMP;
            pthread_mutex_lock(&TakeComp);
        }
        if(answer < 1)
            return 0;
        return 1;
    }
    return 0;
}

/*
 * Select a Tracking Mode
 * type: EVILIB_CHASE1
 *       EVILIB_CHASE2
 *       EVILIB_CHASE3
 *       EVILIB_CHASE123
 * waitC: wait for completion of command: EVILIB_NO_WAIT_COMP
 *                                        EVILIB_WAIT_COMP
 * Return 1 on success, 0 on error
 */
int EVILib::AT_Chase(int type, int waitC)
{
    if(power == EVILIB_ON)
    {
        pthread_mutex_lock(&BufferDispo);
        switch(type)
        {
        case EVILIB_CHASE1:
            sprintf(buffer, "8%d01070700FF", Id_cam);
            break;
        case EVILIB_CHASE2:
            sprintf(buffer, "8%d01070701FF", Id_cam);
            break;
        case EVILIB_CHASE3:
            sprintf(buffer, "8%d01070702FF", Id_cam);
            break;
        case EVILIB_CHASE123:
            sprintf(buffer, "8%d01070710FF", Id_cam);
            break;
        default:
            pthread_mutex_unlock(&BufferDispo);
            return 0;
        }
        if(SendCommand((unsigned char *)buffer, strlen(buffer)) == 0)
        {
            pthread_mutex_unlock(&BufferDispo);
            return 0;
        }
        pthread_mutex_unlock(&BufferDispo);
// receiv 'ACK'
        pthread_mutex_lock(&TakeAck);
// Wait for Completion
        if(waitC == EVILIB_WAIT_COMP)
        {
            waitComp = EVILIB_WAIT_COMP;
            pthread_mutex_lock(&TakeComp);
        }
        if(answer < 1)
            return 0;
        return 1;
    }
    return 0;
}

/*
 * Select target study mode for AT
 * type: EVILIB_ENTRY1
 *       EVILIB_ENTRY2
 *       EVILIB_ENTRY3
 *       EVILIB_ENTRY4
 * waitC: wait for completion of command: EVILIB_NO_WAIT_COMP
 *                                        EVILIB_WAIT_COMP
 * Return 1 on success, 0 on error
 */
int EVILib::AT_Entry(int type, int waitC)
{
    if(power == EVILIB_ON)
    {
        pthread_mutex_lock(&BufferDispo);
        switch(type)
        {
        case EVILIB_ENTRY1:
            sprintf(buffer, "8%d01071500FF", Id_cam);
            break;
        case EVILIB_ENTRY2:
            sprintf(buffer, "8%d01071501FF", Id_cam);
            break;
        case EVILIB_ENTRY3:
            sprintf(buffer, "8%d01071502FF", Id_cam);
            break;
        case EVILIB_ENTRY4:
            sprintf(buffer, "8%d01071503FF", Id_cam);
            break;
        default:
            pthread_mutex_unlock(&BufferDispo);
            return 0;
        }
        if(SendCommand((unsigned char *)buffer, strlen(buffer)) == 0)
        {
            pthread_mutex_unlock(&BufferDispo);
            return 0;
        }
        pthread_mutex_unlock(&BufferDispo);
// receiv 'ACK'
        pthread_mutex_lock(&TakeAck);
// Wait for Completion
        if(waitC == EVILIB_WAIT_COMP)
        {
            waitComp = EVILIB_WAIT_COMP;
            pthread_mutex_lock(&TakeComp);
        }
        if(answer < 1)
            return 0;
        return 1;
    }
    return 0;
}

/*
 * Motion Detector Mode ON/OFF
 * type: EVILIB_ON
 *       EVILIB_OFF
 *       EVILIB_ON_OFF
 * waitC: wait for completion of command: EVILIB_NO_WAIT_COMP
 *                                        EVILIB_WAIT_COMP
 * Return 1 on success, 0 on error
 */
int EVILib::MD_Mode(int type, int waitC)
{
    if(power == EVILIB_ON)
    {
        pthread_mutex_lock(&BufferDispo);
        switch(type)
        {
        case EVILIB_ON:
            sprintf(buffer, "8%d01070802FF", Id_cam);
            break;
        case EVILIB_OFF:
            sprintf(buffer, "8%d01070803FF", Id_cam);
            break;
        case EVILIB_ON_OFF:
            sprintf(buffer, "8%d01070810FF", Id_cam);
            break;
        default:
            pthread_mutex_unlock(&BufferDispo);
            return 0;
        }
        if(SendCommand((unsigned char *)buffer, strlen(buffer)) == 0)
        {
            pthread_mutex_unlock(&BufferDispo);
            return 0;
        }
        pthread_mutex_unlock(&BufferDispo);
// receiv 'ACK'
        pthread_mutex_lock(&TakeAck);
// Wait for Completion
        if(waitC == EVILIB_WAIT_COMP)
        {
            waitComp = EVILIB_WAIT_COMP;
            pthread_mutex_lock(&TakeComp);
        }
        if(answer < 1)
            return 0;
        return 1;
    }
    return 0;
}

/*
 * Detecting Area Set (Size or Position)
 * waitC: wait for completion of command: EVILIB_NO_WAIT_COMP
 *                                        EVILIB_WAIT_COMP
 * Return 1 on success, 0 on error
 */
int EVILib::MD_Frame(int waitC)
{
    if(power == EVILIB_ON)
    {
        pthread_mutex_lock(&BufferDispo);
        sprintf(buffer, "8%d010709FF", Id_cam);
        if(SendCommand((unsigned char *)buffer, strlen(buffer)) == 0)
        {
            pthread_mutex_unlock(&BufferDispo);
            return 0;
        }
        pthread_mutex_unlock(&BufferDispo);
// receiv 'ACK'
        pthread_mutex_lock(&TakeAck);
// Wait for Completion
        if(waitC == EVILIB_WAIT_COMP)
        {
            waitComp = EVILIB_WAIT_COMP;
            pthread_mutex_lock(&TakeComp);
        }
        if(answer < 1)
            return 0;
        return 1;
    }
    return 0;
}

/*
 * Select Detecting Frame (1 or 2 or 1 + 2)
 * waitC: wait for completion of command: EVILIB_NO_WAIT_COMP
 *                                        EVILIB_WAIT_COMP
 * Return 1 on success, 0 on error
 */
int EVILib::MD_Detect(int waitC)
{
    if(power == EVILIB_ON)
    {
        pthread_mutex_lock(&BufferDispo);
        sprintf(buffer, "8%d01070A10FF", Id_cam);
        if(SendCommand((unsigned char *)buffer, strlen(buffer)) == 0)
        {
            pthread_mutex_unlock(&BufferDispo);
            return 0;
        }
        pthread_mutex_unlock(&BufferDispo);
// receiv 'ACK'
        pthread_mutex_lock(&TakeAck);
// Wait for Completion
        if(waitC == EVILIB_WAIT_COMP)
        {
            waitComp = EVILIB_WAIT_COMP;
            pthread_mutex_lock(&TakeComp);
        }
        if(answer < 1)
            return 0;
        return 1;
    }
    return 0;
}

/*
 * Reply a completion when the camera lost the target in AT mode.
 * waitC: wait for completion of command: EVILIB_NO_WAIT_COMP
 *                                        EVILIB_WAIT_COMP
 * Return 1 on success, 0 on error
 */
int EVILib::AT_LostInfo(int waitC)
{
    if(power == EVILIB_ON)
    {
        pthread_mutex_lock(&BufferDispo);
        sprintf(buffer, "8%d0106200720FF", Id_cam);
        if(SendCommand((unsigned char *)buffer, strlen(buffer)) == 0)
        {
            pthread_mutex_unlock(&BufferDispo);
            return 0;
        }
        pthread_mutex_unlock(&BufferDispo);
// receiv 'ACK'
        pthread_mutex_lock(&TakeAck);
// Wait for Completion
        if(waitC == EVILIB_WAIT_COMP)
        {
            waitComp = EVILIB_WAIT_COMP;
            pthread_mutex_lock(&TakeComp);
        }
        if(answer < 1)
            return 0;
        return 1;
    }
    return 0;
}

/*
 * Reply a completion when the camera detected a motion of image in MD mode.
 * waitC: wait for completion of command: EVILIB_NO_WAIT_COMP
 *                                        EVILIB_WAIT_COMP
 * Return 1 on success, 0 on error
 */
int EVILib::MD_LostInfo(int waitC)
{
    if(power == EVILIB_ON)
    {
        pthread_mutex_lock(&BufferDispo);
        sprintf(buffer, "8%d0106200721FF", Id_cam);
        if(SendCommand((unsigned char *)buffer, strlen(buffer)) == 0)
        {
            pthread_mutex_unlock(&BufferDispo);
            return 0;
        }
        pthread_mutex_unlock(&BufferDispo);
// receiv 'ACK'
        pthread_mutex_lock(&TakeAck);
// Wait for Completion
        if(waitC == EVILIB_WAIT_COMP)
        {
            waitComp = EVILIB_WAIT_COMP;
            pthread_mutex_lock(&TakeComp);
        }
        if(answer < 1)
            return 0;
        return 1;
    }
    return 0;
}

/*
 * Set Detecting Condition
 * type: EVILIB_Y_LEVEL --> need 'setting'
 *       EVILIB_HUE_LEVEL --> need 'setting'
 *       EVILIB_SIZE --> need 'setting'
 *       EVILIB_DISPLAYTIME --> need 'setting'
 *       EVILIB_REFRESH_MODE1
 *       EVILIB_REFRESH_MODE2
 *       EVILIB_REFRESH_MODE3
 *       EVILIB_REFRESH_TIME --> need 'setting'
 * setting: 0 to 15
 * waitC: wait for completion of command: EVILIB_NO_WAIT_COMP
 *                                        EVILIB_WAIT_COMP
 * Return 1 on success, 0 on error
 */
int EVILib::MD_Adjust(int type, int setting, int waitC)
{
    if(power == EVILIB_ON)
    {
        if(setting >=0 && setting <= 15)
        {
            pthread_mutex_lock(&BufferDispo);
            switch(type)
            {
            case EVILIB_Y_LEVEL:
                sprintf(buffer, "8%d01070B000%XFF", Id_cam, setting);
                break;
            case EVILIB_HUE_LEVEL:
                sprintf(buffer, "8%d01070C000%XFF", Id_cam, setting);
                break;
            case EVILIB_SIZE:
                sprintf(buffer, "8%d01070D000%XFF", Id_cam, setting);
                break;
            case EVILIB_DISPLAYTIME:
                sprintf(buffer, "8%d01070F000%XFF", Id_cam, setting);
                break;
            case EVILIB_REFRESH_MODE1:
                sprintf(buffer, "8%d01071000FF", Id_cam);
                break;
            case EVILIB_REFRESH_MODE2:
                sprintf(buffer, "8%d01071001FF", Id_cam);
                break;
            case EVILIB_REFRESH_MODE3:
                sprintf(buffer, "8%d01071002FF", Id_cam);
                break;
// Seems to be a problem with the command list of the camera
//  the 'refresh time' command has the same packet as the 'Y level' command ...
// As it seems that the command and inquiry have some part of the packets in common,
//  it's seems logical to use '8x010711000ZFF' as command of the 'refresh time'
// (we have replace the '..0B..' by '..11..'
            case EVILIB_REFRESH_TIME:
                sprintf(buffer, "8%d010711000%XFF", Id_cam, setting);
                break;
            default:
                pthread_mutex_unlock(&BufferDispo);
                return 0;
            }
            if(SendCommand((unsigned char *)buffer, strlen(buffer)) == 0)
            {
                pthread_mutex_unlock(&BufferDispo);
                return 0;
            }
            pthread_mutex_unlock(&BufferDispo);
// receiv 'ACK'
            pthread_mutex_lock(&TakeAck);
// Wait for Completion
            if(waitC == EVILIB_WAIT_COMP)
            {
                waitComp = EVILIB_WAIT_COMP;
                pthread_mutex_lock(&TakeComp);
            }
            if(answer < 1)
                return 0;
            return 1;
        }
    }

    return 0;
}

/*
 * Target Condition Measure Mode for More Accurate Setting for Motion Detector
 * type: EVILIB_ON
 *       EVILIB_OFF
 *       EVILIB_ON_OFF
 * waitC: wait for completion of command: EVILIB_NO_WAIT_COMP
 *                                        EVILIB_WAIT_COMP
 * Return 1 on success, 0 on error
 */
int EVILib::Measure_Mode1(int type, int waitC)
{
    if(power == EVILIB_ON)
    {
        pthread_mutex_lock(&BufferDispo);
        switch(type)
        {
        case EVILIB_ON:
            sprintf(buffer, "8%d01072702FF", Id_cam);
            break;
        case EVILIB_OFF:
            sprintf(buffer, "8%d01072703FF", Id_cam);
            break;
        case EVILIB_ON_OFF:
            sprintf(buffer, "8%d01072710FF", Id_cam);
            break;
        default:
            pthread_mutex_unlock(&BufferDispo);
            return 0;
        }
        if(SendCommand((unsigned char *)buffer, strlen(buffer)) == 0)
        {
            pthread_mutex_unlock(&BufferDispo);
            return 0;
        }
        pthread_mutex_unlock(&BufferDispo);
// receiv 'ACK'
        pthread_mutex_lock(&TakeAck);
// Wait for Completion
        if(waitC == EVILIB_WAIT_COMP)
        {
            waitComp = EVILIB_WAIT_COMP;
            pthread_mutex_lock(&TakeComp);
        }
        if(answer < 1)
            return 0;
        return 1;
    }
    return 0;
}

/*
 * Target Condition Measure Mode for More Accurate Setting for Motion Detector
 * type: EVILIB_ON
 *       EVILIB_OFF
 *       EVILIB_ON_OFF
 * waitC: wait for completion of command: EVILIB_NO_WAIT_COMP
 *                                        EVILIB_WAIT_COMP
 * Return 1 on success, 0 on error
 */
int EVILib::Measure_Mode2(int type, int waitC)
{
    if(power == EVILIB_ON)
    {
        pthread_mutex_lock(&BufferDispo);
        switch(type)
        {
        case EVILIB_ON:
            sprintf(buffer, "8%d01072802FF", Id_cam);
            break;
        case EVILIB_OFF:
            sprintf(buffer, "8%d01072803FF", Id_cam);
            break;
        case EVILIB_ON_OFF:
            sprintf(buffer, "8%d01072810FF", Id_cam);
            break;
        default:
            pthread_mutex_unlock(&BufferDispo);
            return 0;
        }
        if(SendCommand((unsigned char *)buffer, strlen(buffer)) == 0)
        {
            pthread_mutex_unlock(&BufferDispo);
            return 0;
        }
        pthread_mutex_unlock(&BufferDispo);
// receiv 'ACK'
        pthread_mutex_lock(&TakeAck);
// Wait for Completion
        if(waitC == EVILIB_WAIT_COMP)
        {
            waitComp = EVILIB_WAIT_COMP;
            pthread_mutex_lock(&TakeComp);
        }
        if(answer < 1)
            return 0;
        return 1;
    }
    return 0;
}
#endif // __EVI_D30__

/*
 * Return on success: EVILIB_ON
 *                    EVILIB_OFF
 * Return on error: 0
 */
int EVILib::PowerInq()
{
    char c;
    pthread_mutex_lock(&BufferDispo);
    sprintf(buffer, "8%d090400FF", Id_cam);
    if(SendCommand((unsigned char *)buffer, strlen(buffer)) == 0)
    {
        pthread_mutex_unlock(&BufferDispo);
        return 0;
    }
    pthread_mutex_unlock(&BufferDispo);

// receiv 'information return'
    pthread_mutex_lock(&TakeInfo);
    if(answer < 1)
        return 0;
    pthread_mutex_lock(&AccessBuffer3);
    c = buffer3[2];
    pthread_mutex_unlock(&AccessBuffer3);
    if(c == '\x02')
    {
        power = EVILIB_ON;
        return EVILIB_ON;
    }
    if(c == '\x03')
    {
        power = EVILIB_OFF;
        return EVILIB_OFF;
    }
    return 0;
}

#ifdef __EVI_D100__
/*
 * timer: contain the power off timer
 * Return 1 on success, 0 on error
 */
int EVILib::AutoPowerOffInq(int &timer)
{
   timer = -1;
    if(power == EVILIB_ON)
    {
        pthread_mutex_lock(&BufferDispo);
        sprintf(buffer, "8%d090440FF", Id_cam);
        if(SendCommand((unsigned char *)buffer, strlen(buffer)) == 0)
        {
            pthread_mutex_unlock(&BufferDispo);
            return 0;
        }
        pthread_mutex_unlock(&BufferDispo);

// receiv 'information return'
        pthread_mutex_lock(&TakeInfo);
        if(answer < 1)
            return 0;
        pthread_mutex_lock(&AccessBuffer3);
        timer = int(deconvert(2, 4));
        pthread_mutex_unlock(&AccessBuffer3);
        return 1;
    }
    return 0;
}

/*
 * Return on success: EVILIB_ON
 *                    EVILIB_OFF
 * Return on error: 0
 */
int EVILib::DZoomModeInq()
{
    char c;
    pthread_mutex_lock(&BufferDispo);
    sprintf(buffer, "8%d090406FF", Id_cam);
    if(SendCommand((unsigned char *)buffer, strlen(buffer)) == 0)
    {
        pthread_mutex_unlock(&BufferDispo);
        return 0;
    }
    pthread_mutex_unlock(&BufferDispo);

// receiv 'information return'
    pthread_mutex_lock(&TakeInfo);
    if(answer < 1)
        return 0;
    pthread_mutex_lock(&AccessBuffer3);
    c = buffer3[2];
    pthread_mutex_unlock(&AccessBuffer3);
    if(c == '\x02')
        return EVILIB_ON;
    if(c == '\x03')
        return EVILIB_OFF;
    return 0;
}
#endif // __EVI_D100_

/*
 * 
 */

/*
 * zoom: contain the zoom position of the camera
 * Return 1 on success, 0 on error
 */
int EVILib::ZoomPosInq(float &zoom)
{
    zoom = -1.0;
    if(power == EVILIB_ON)
    {
        pthread_mutex_lock(&BufferDispo);
        sprintf(buffer, "8%d090447FF", Id_cam);
        if(SendCommand((unsigned char *)buffer, strlen(buffer)) == 0)
        {
            pthread_mutex_unlock(&BufferDispo);
            return 0;
        }
        pthread_mutex_unlock(&BufferDispo);

// receiv 'information return'
        pthread_mutex_lock(&TakeInfo);
        if(answer < 1)
            return 0;
        pthread_mutex_lock(&AccessBuffer3);
        zoom = (deconvert(2, 4) / EVILIB_maxZoom) * EVILIB_maxzoom;
        pthread_mutex_unlock(&AccessBuffer3);
        return 1;
    }
    return 0;
}

/*
 * Return on success: EVILIB_AUTO
 *                    EVILIB_MANUAL
 * Return on error: 0
 */
int EVILib::FocusModeInq()
{
    char c;
    pthread_mutex_lock(&BufferDispo);
    sprintf(buffer, "8%d090438FF", Id_cam);
    if(SendCommand((unsigned char *)buffer, strlen(buffer)) == 0)
    {
        pthread_mutex_unlock(&BufferDispo);
        return 0;
    }
    pthread_mutex_unlock(&BufferDispo);

// receiv 'information return'
    pthread_mutex_lock(&TakeInfo);
    if(answer < 1)
        return 0;
    pthread_mutex_lock(&AccessBuffer3);
    c = buffer3[2];
    pthread_mutex_unlock(&AccessBuffer3);
    if(c == '\x02')
        return EVILIB_AUTO;
    if(c == '\x03')
        return EVILIB_MANUAL;
    return 0;
}

/*
 * focus: contain the focus position of the camera
 * Return 1 on success, 0 on error
 */
int EVILib::FocusPosInq(int &focus)
{
    focus = -1;
    if(power == EVILIB_ON)
    {
        pthread_mutex_lock(&BufferDispo);
        sprintf(buffer, "8%d090448FF", Id_cam);
        if(SendCommand((unsigned char *)buffer, strlen(buffer)) == 0)
        {
            pthread_mutex_unlock(&BufferDispo);
            return 0;
        }
        pthread_mutex_unlock(&BufferDispo);

// receiv 'information return'
        pthread_mutex_lock(&TakeInfo);
        if(answer < 1)
            return 0;
        pthread_mutex_lock(&AccessBuffer3);
        focus = int(deconvert(2, 4));
        pthread_mutex_unlock(&AccessBuffer3);
        return 1;
    }
    return 0;
}

#ifdef __EVI_D100__
/*
 * Return on success: EVILIB_AF_SENS_HIGH
 *                    EVILIB_AF_SENS_LOW
 * Return 0 on error
 */
int EVILib::AFModeInq()
{
    char c;
    pthread_mutex_lock(&BufferDispo);
    sprintf(buffer, "8%d090458FF", Id_cam);
    if(SendCommand((unsigned char *)buffer, strlen(buffer)) == 0)
    {
        pthread_mutex_unlock(&BufferDispo);
        return 0;
    }
    pthread_mutex_unlock(&BufferDispo);

// receiv 'information return'
    pthread_mutex_lock(&TakeInfo);
    if(answer < 1)
        return 0;
    pthread_mutex_lock(&AccessBuffer3);
    c = buffer3[2];
    pthread_mutex_unlock(&AccessBuffer3);
    if(c == '\x02')
        return EVILIB_AF_SENS_HIGH;
    if(c == '\x03')
        return EVILIB_AF_SENS_LOW;
    return 0;
}

/*
 * focus contain the Focus limit position
 * Return 1 on success, 0 on error
 */
int EVILib::FocusNearLimitInq(int &focus)
{
    focus = -1;
    if(power == EVILIB_ON)
    {
        pthread_mutex_lock(&BufferDispo);
        sprintf(buffer, "8%d090428FF", Id_cam);
        if(SendCommand((unsigned char *)buffer, strlen(buffer)) == 0)
        {
            pthread_mutex_unlock(&BufferDispo);
            return 0;
        }
        pthread_mutex_unlock(&BufferDispo);

// receiv 'information return'
        pthread_mutex_lock(&TakeInfo);
        if(answer < 1)
            return 0;
        pthread_mutex_lock(&AccessBuffer3);
        focus = int(deconvert(2, 4));
        pthread_mutex_unlock(&AccessBuffer3);
        return 1;
    }
    return 0;
}
#endif // __EVI_D100__

/*
 * Return on success: EVILIB_AUTO
 *                    EVILIB_INDOOR
 *                    EVILIB_OUTDOOR
 *                    EVILIB_ONEPUSH_MODE
 *                    EVILIB_ATW
 *                    EVILIB_MANUAL
 * Return on error: 0
 */
int EVILib::WBModeInq()
{
    char c;
    pthread_mutex_lock(&BufferDispo);
    sprintf(buffer, "8%d090435FF", Id_cam);
    if(SendCommand((unsigned char *)buffer, strlen(buffer)) == 0)
    {
        pthread_mutex_unlock(&BufferDispo);
        return 0;
    }
    pthread_mutex_unlock(&BufferDispo);

// receiv 'information return'
    pthread_mutex_lock(&TakeInfo);
    if(answer < 1)
        return 0;
    pthread_mutex_lock(&AccessBuffer3);
    c = buffer3[2];
    pthread_mutex_unlock(&AccessBuffer3);
    if(c == '\x00')
        return EVILIB_AUTO;
    if(c == '\x01')
        return EVILIB_INDOOR;
    if(c == '\x02')
        return EVILIB_OUTDOOR;
    if(c == '\x03')
        return EVILIB_ONEPUSH_MODE;
    if(c == '\x04')
        return EVILIB_ATW;
    if(c == '\x05')
        return EVILIB_MANUAL;
    return 0;
}

#ifdef __EVI_D100__
/*
 * gain contain the R Gain
 * Return 1 on success, 0 on error
 */
int EVILib::RGainInq(int &gain)
{
    gain = -1;
    if(power == EVILIB_ON)
    {
        pthread_mutex_lock(&BufferDispo);
        sprintf(buffer, "8%d090443FF", Id_cam);
        if(SendCommand((unsigned char *)buffer, strlen(buffer)) == 0)
        {
            pthread_mutex_unlock(&BufferDispo);
            return 0;
        }
        pthread_mutex_unlock(&BufferDispo);

// receiv 'information return'
        pthread_mutex_lock(&TakeInfo);
        if(answer < 1)
            return 0;
        pthread_mutex_lock(&AccessBuffer3);
        gain = int(deconvert(2, 4));
        pthread_mutex_unlock(&AccessBuffer3);
        return 1;
    }
    return 0;
}

/*
 * gain contain the B Gain
 * Return 1 on success, 0 on error
 */
int EVILib::BGainInq(int &gain)
{
    gain = -1;
    if(power == EVILIB_ON)
    {
        pthread_mutex_lock(&BufferDispo);
        sprintf(buffer, "8%d090444FF", Id_cam);
        if(SendCommand((unsigned char *)buffer, strlen(buffer)) == 0)
        {
            pthread_mutex_unlock(&BufferDispo);
            return 0;
        }
        pthread_mutex_unlock(&BufferDispo);

// receiv 'information return'
        pthread_mutex_lock(&TakeInfo);
        if(answer < 1)
            return 0;
        pthread_mutex_lock(&AccessBuffer3);
        gain = int(deconvert(2, 4));
        pthread_mutex_unlock(&AccessBuffer3);
        return 1;
    }
    return 0;
}
#endif // __EVI_D100__

/*
 * Return on success: EVILIB_AUTO
 *                    EVILIB_MANUAL
 *                    EVILIB_SHUTTER_PRIO
 *                    EVILIB_IRIS_PRIO
 *                    EVILIB_GAIN_PRIO
 *                    EVILIB_BRIGHT
 *                    EVILIB_SHUTTER_AUTO
 *                    EVILIB_IRIS_AUTO
 *                    EVILIB_GAIN_AUTO
 * Return on error: 0
 */
int EVILib::AEModeInq()
{
    char c;
    pthread_mutex_lock(&BufferDispo);
    sprintf(buffer, "8%d090439FF", Id_cam);
    if(SendCommand((unsigned char *)buffer, strlen(buffer)) == 0)
    {
        pthread_mutex_unlock(&BufferDispo);
        return 0;
    }
    pthread_mutex_unlock(&BufferDispo);

// receiv 'information return'
    pthread_mutex_lock(&TakeInfo);
    if(answer < 1)
        return 0;
    pthread_mutex_lock(&AccessBuffer3);
    c = buffer3[2];
    pthread_mutex_unlock(&AccessBuffer3);
    if(c == '\x00')
        return EVILIB_AUTO;
    if(c == '\x03')
        return EVILIB_MANUAL;
    if(c == '\x0A')
        return EVILIB_SHUTTER_PRIO;
    if(c == '\x0B')
        return EVILIB_IRIS_PRIO;
    if(c == '\x0C')
        return EVILIB_GAIN_PRIO;
    if(c == '\x0D')
        return EVILIB_BRIGHT;
    if(c == '\x1A')
        return EVILIB_SHUTTER_AUTO;
    if(c == '\x1B')
        return EVILIB_IRIS_AUTO;
    if(c == '\x1C')
        return EVILIB_GAIN_AUTO;
    return 0;
}

#ifdef __EVI_D100__
/*
 * Return on success: EVILIB_AUTO
 *                    EVILIB_MANUAL
 * Return 0 on error
 */
int EVILib::SlowShutterModeInq()
{
    char c;
    pthread_mutex_lock(&BufferDispo);
    sprintf(buffer, "8%d09045AFF", Id_cam);
    if(SendCommand((unsigned char *)buffer, strlen(buffer)) == 0)
    {
        pthread_mutex_unlock(&BufferDispo);
        return 0;
    }
    pthread_mutex_unlock(&BufferDispo);

// receiv 'information return'
    pthread_mutex_lock(&TakeInfo);
    if(answer < 1)
        return 0;
    pthread_mutex_lock(&AccessBuffer3);
    c = buffer3[2];
    pthread_mutex_unlock(&AccessBuffer3);
    if(c == '\x02')
        return EVILIB_AUTO;
    if(c == '\x03')
        return EVILIB_MANUAL;
    return 0;
}
#endif // __EVI_D100__

/*
 * shutter: contain the shutter position of the camera
 * Return 1 on success, 0 on error
 */
int EVILib::ShutterPosInq(int &shutter)
{
    shutter = -1;
    if(power == EVILIB_ON)
    {
        pthread_mutex_lock(&BufferDispo);
        sprintf(buffer, "8%d09044AFF", Id_cam);
        if(SendCommand((unsigned char *)buffer, strlen(buffer)) == 0)
        {
            pthread_mutex_unlock(&BufferDispo);
            return 0;
        }
        pthread_mutex_unlock(&BufferDispo);

// receiv 'information return'
        pthread_mutex_lock(&TakeInfo);
        if(answer < 1)
            return 0;
        pthread_mutex_lock(&AccessBuffer3);
        int s = int(deconvert(2, 4));
        pthread_mutex_unlock(&AccessBuffer3);
        switch(s)
        {
        case 0:
            shutter = 60;
            break;
        case 1:
            shutter = 60;
            break;
        case 2:
            shutter = 75;
            break;
        case 3:
            shutter = 90;
            break;
        case 4:
            shutter = 100;
            break;
        case 5:
            shutter = 125;
            break;
        case 6:
            shutter = 150;
            break;
        case 7:
            shutter = 180;
            break;
        case 8:
            shutter = 215;
            break;
        case 9:
            shutter = 250;
            break;
        case 10:
            shutter = 300;
            break;
        case 11:
            shutter = 350;
            break;
        case 12:
            shutter = 425;
            break;
        case 13:
            shutter = 500;
            break;
        case 14:
            shutter = 600;
            break;
        case 15:
            shutter = 725;
            break;
        case 16:
            shutter = 850;
            break;
        case 17:
            shutter = 1000;
            break;
        case 18:
            shutter = 1250;
            break;
        case 19:
            shutter = 1500;
            break;
        case 20:
            shutter = 1750;
            break;
        case 21:
            shutter = 2000;
            break;
        case 22:
            shutter = 2500;
            break;
        case 23:
            shutter = 3000;
            break;
        case 24:
            shutter = 3500;
            break;
        case 25:
            shutter = 4000;
            break;
        case 26:
            shutter = 6000;
            break;
        case 27:
            shutter = 10000;
            break;
        default:
            return 0;
        }
        return 1;
    }
    return 0;
}

/*
 * iris: contain the iris position of the camera
 * Return 1 on success, 0 on error
 */
int EVILib::IrisPosInq(int &iris)
{
    iris = -1;
    if(power == EVILIB_ON)
    {
        pthread_mutex_lock(&BufferDispo);
        sprintf(buffer, "8%d09044BFF", Id_cam);
        if(SendCommand((unsigned char *)buffer, strlen(buffer)) == 0)
        {
            pthread_mutex_unlock(&BufferDispo);
            return 0;
        }
        pthread_mutex_unlock(&BufferDispo);

// receiv 'information return'
        pthread_mutex_lock(&TakeInfo);
        if(answer < 1)
            return 0;
        pthread_mutex_lock(&AccessBuffer3);
        int s = int(deconvert(2, 4));
        pthread_mutex_unlock(&AccessBuffer3);
        switch(s)
        {
        case 0:
            iris = EVILIB_IRIS_CLOSE;
            break;
        case 1:
            iris = EVILIB_IRIS_F28;
            break;
        case 2:
            iris = EVILIB_IRIS_F22;
            break;
        case 3:
            iris = EVILIB_IRIS_F19;
            break;
        case 4:
            iris = EVILIB_IRIS_F16;
            break;
        case 5:
            iris = EVILIB_IRIS_F14;
            break;
        case 6:
            iris = EVILIB_IRIS_F11;
            break;
        case 7:
            iris = EVILIB_IRIS_F9_6;
            break;
        case 8:
            iris = EVILIB_IRIS_F8;
            break;
        case 9:
            iris = EVILIB_IRIS_F6_8;
            break;
        case 10:
            iris = EVILIB_IRIS_F5_6;
            break;
        case 11:
            iris = EVILIB_IRIS_F4_8;
            break;
        case 12:
            iris = EVILIB_IRIS_F4;
            break;
        case 13:
            iris = EVILIB_IRIS_F3_4;
            break;
        case 14:
            iris = EVILIB_IRIS_F2_8;
            break;
        case 15:
            iris = EVILIB_IRIS_F2_4;
            break;
        case 16:
            iris = EVILIB_IRIS_F2;
            break;
        case 17:
            iris = EVILIB_IRIS_F1_8;
            break;
        default:
            return 0;
        }
        return 1;
    }
    return 0;
}

/*
 * gain: contain the gain of the camera (check to doc for the values)
 * Return 1 on success, 0 on error
 */
int EVILib::GainPosInq(int &gain)
{
    gain = -1;
    if(power == EVILIB_ON)
    {
        pthread_mutex_lock(&BufferDispo);
        sprintf(buffer, "8%d09044CFF", Id_cam);
        if(SendCommand((unsigned char *)buffer, strlen(buffer)) == 0)
        {
            pthread_mutex_unlock(&BufferDispo);
            return 0;
        }
        pthread_mutex_unlock(&BufferDispo);

// receiv 'information return'
        pthread_mutex_lock(&TakeInfo);
        if(answer < 1)
            return 0;
        pthread_mutex_lock(&AccessBuffer3);
        gain = int(deconvert(2, 4));
        pthread_mutex_unlock(&AccessBuffer3);
        return 1;
    }
    return 0;
}

#ifdef __EVI_D100__
/*
 * bright contain the bright value of the camera
 * Return 1 on success, 0 on error
 */
int EVILib::BrightPosInq(int &bright)
{
    bright = -1;
    if(power == EVILIB_ON)
    {
        pthread_mutex_lock(&BufferDispo);
        sprintf(buffer, "8%d09044DFF", Id_cam);
        if(SendCommand((unsigned char *)buffer, strlen(buffer)) == 0)
        {
            pthread_mutex_unlock(&BufferDispo);
            return 0;
        }
        pthread_mutex_unlock(&BufferDispo);

// receiv 'information return'
        pthread_mutex_lock(&TakeInfo);
        if(answer < 1)
            return 0;
        pthread_mutex_lock(&AccessBuffer3);
        int s = int(deconvert(2, 4));
        pthread_mutex_unlock(&AccessBuffer3);
        switch(s)
        {
        case 0:
            bright = EVILIB_BRIGHT_CLOSE;
            break;
        case 1:
            bright = EVILIB_BRIGHT_F28;
            break;
        case 2:
            bright = EVILIB_BRIGHT_F22;
            break;
        case 3:
            bright = EVILIB_BRIGHT_F19;
            break;
        case 4:
            bright = EVILIB_BRIGHT_F16;
            break;
        case 5:
            bright = EVILIB_BRIGHT_F14;
            break;
        case 6:
            bright = EVILIB_BRIGHT_F11;
            break;
        case 7:
            bright = EVILIB_BRIGHT_F9_6;
            break;
        case 8:
            bright = EVILIB_BRIGHT_F8;
            break;
        case 9:
            bright = EVILIB_BRIGHT_F6_8;
            break;
        case 10:
            bright = EVILIB_BRIGHT_F5_6;
            break;
        case 11:
            bright = EVILIB_BRIGHT_F4_8;
            break;
        case 12:
            bright = EVILIB_BRIGHT_F4;
            break;
        case 13:
            bright = EVILIB_BRIGHT_F3_4;
            break;
        case 14:
            bright = EVILIB_BRIGHT_F2_8;
            break;
        case 15:
            bright = EVILIB_BRIGHT_F2_4;
            break;
        case 16:
            bright = EVILIB_BRIGHT_F2;
            break;
        case 17:
            bright = EVILIB_BRIGHT_F1_8_0;
            break;
        case 18:
            bright = EVILIB_BRIGHT_F1_8_3;
            break;
        case 19:
            bright = EVILIB_BRIGHT_F1_8_6;
            break;
        case 20:
            bright = EVILIB_BRIGHT_F1_8_9;
            break;
        case 21:
            bright = EVILIB_BRIGHT_F1_8_12;
            break;
        case 22:
            bright = EVILIB_BRIGHT_F1_8_15;
            break;
        case 23:
            bright = EVILIB_BRIGHT_F1_8_18;
            break;
        default:
            return 0;
        }
        return 1;
    }
    return 0;
}

/*
 * Return on success: EVILIB_ON
 *                    EVILIB_OFF
 * Return on error: 0
 */
int EVILib::ExpCompModeInq()
{
    char c;
    pthread_mutex_lock(&BufferDispo);
    sprintf(buffer, "8%d09043EFF", Id_cam);
    if(SendCommand((unsigned char *)buffer, strlen(buffer)) == 0)
    {
        pthread_mutex_unlock(&BufferDispo);
        return 0;
    }
    pthread_mutex_unlock(&BufferDispo);

// receiv 'information return'
    pthread_mutex_lock(&TakeInfo);
    if(answer < 1)
        return 0;
    pthread_mutex_lock(&AccessBuffer3);
    c = buffer3[2];
    pthread_mutex_unlock(&AccessBuffer3);
    if(c == '\x02')
        return EVILIB_ON;
    if(c == '\x03')
        return EVILIB_OFF;
    return 0;
}

/*
 * ExpComp containt the ExpComp position
 * Return 1 on success, 0 on error
 */
int EVILib::ExpCompPosInq(int &ExpComp)
{
    ExpComp = -1;
    if(power == EVILIB_ON)
    {
        pthread_mutex_lock(&BufferDispo);
        sprintf(buffer, "8%d09044EFF", Id_cam);
        if(SendCommand((unsigned char *)buffer, strlen(buffer)) == 0)
        {
            pthread_mutex_unlock(&BufferDispo);
            return 0;
        }
        pthread_mutex_unlock(&BufferDispo);

// receiv 'information return'
        pthread_mutex_lock(&TakeInfo);
        if(answer < 1)
            return 0;
        pthread_mutex_lock(&AccessBuffer3);
        ExpComp = int(deconvert(2, 4));
        pthread_mutex_unlock(&AccessBuffer3);
        return 1;
    }
    return 0;
}

#endif // __EVI_D100__

/*
 * Return on success: EVILIB_ON
 *                    EVILIB_OFF
 * Return on error: 0
 */
int EVILib::BacklightModeInq()
{
    char c;
    pthread_mutex_lock(&BufferDispo);
    sprintf(buffer, "8%d090433FF", Id_cam);
    if(SendCommand((unsigned char *)buffer, strlen(buffer)) == 0)
    {
        pthread_mutex_unlock(&BufferDispo);
        return 0;
    }
    pthread_mutex_unlock(&BufferDispo);

// receiv 'information return'
    pthread_mutex_lock(&TakeInfo);
    if(answer < 1)
        return 0;
    pthread_mutex_lock(&AccessBuffer3);
    c = buffer3[2];
    pthread_mutex_unlock(&AccessBuffer3);
    if(c == '\x02')
        return EVILIB_ON;
    if(c == '\x03')
        return EVILIB_OFF;
    return 0;
}

#ifdef __EVI_D100__
/*
 * aperture contain the Aperture Gain
 * Return 1 on success, 0 on error
 */
int EVILib::ApertureInq(int &aperture)
{
    aperture = -1;
    if(power == EVILIB_ON)
    {
        pthread_mutex_lock(&BufferDispo);
        sprintf(buffer, "8%d090442FF", Id_cam);
        if(SendCommand((unsigned char *)buffer, strlen(buffer)) == 0)
        {
            pthread_mutex_unlock(&BufferDispo);
            return 0;
        }
        pthread_mutex_unlock(&BufferDispo);

// receiv 'information return'
        pthread_mutex_lock(&TakeInfo);
        if(answer < 1)
            return 0;
        pthread_mutex_lock(&AccessBuffer3);
        aperture = int(deconvert(2, 4));
        pthread_mutex_unlock(&AccessBuffer3);
        return 1;
    }
    return 0;
}

/*
 * Return on success: EVILIB_OFF
 *                    EVILIB_CINEMA
 *                    EVILIB_16_9_FULL
 * Return on error: 0
 */
int EVILib::WideModeInq()
{
    char c;
    pthread_mutex_lock(&BufferDispo);
    sprintf(buffer, "8%d090460FF", Id_cam);
    if(SendCommand((unsigned char *)buffer, strlen(buffer)) == 0)
    {
        pthread_mutex_unlock(&BufferDispo);
        return 0;
    }
    pthread_mutex_unlock(&BufferDispo);

// receiv 'information return'
    pthread_mutex_lock(&TakeInfo);
    if(answer < 1)
        return 0;
    pthread_mutex_lock(&AccessBuffer3);
    c = buffer3[2];
    pthread_mutex_unlock(&AccessBuffer3);
    if(c == '\x00')
        return EVILIB_OFF;
    if(c == '\x01')
        return EVILIB_CINEMA;
    if(c == '\x02')
        return EVILIB_16_9_FULL;
    return 0;
}

/*
 * Return on success: EVILIB_ON
 *                    EVILIB_OFF
 * Return on error: 0
 */
int EVILib::LR_ReverseModeInq()
{
    char c;
    pthread_mutex_lock(&BufferDispo);
    sprintf(buffer, "8%d090461FF", Id_cam);
    if(SendCommand((unsigned char *)buffer, strlen(buffer)) == 0)
    {
        pthread_mutex_unlock(&BufferDispo);
        return 0;
    }
    pthread_mutex_unlock(&BufferDispo);

// receiv 'information return'
    pthread_mutex_lock(&TakeInfo);
    if(answer < 1)
        return 0;
    pthread_mutex_lock(&AccessBuffer3);
    c = buffer3[2];
    pthread_mutex_unlock(&AccessBuffer3);
    if(c == '\x02')
        return EVILIB_ON;
    if(c == '\x03')
        return EVILIB_OFF;
    return 0;
}

/*
 * Return on success: EVILIB_ON
 *                    EVILIB_OFF
 * Return on error: 0
 */
int EVILib::FreezeModeInq()
{
    char c;
    pthread_mutex_lock(&BufferDispo);
    sprintf(buffer, "8%d090462FF", Id_cam);
    if(SendCommand((unsigned char *)buffer, strlen(buffer)) == 0)
    {
        pthread_mutex_unlock(&BufferDispo);
        return 0;
    }
    pthread_mutex_unlock(&BufferDispo);

// receiv 'information return'
    pthread_mutex_lock(&TakeInfo);
    if(answer < 1)
        return 0;
    pthread_mutex_lock(&AccessBuffer3);
    c = buffer3[2];
    pthread_mutex_unlock(&AccessBuffer3);
    if(c == '\x02')
        return EVILIB_ON;
    if(c == '\x03')
        return EVILIB_OFF;
    return 0;
}

/*
 * Return on success: EVILIB_OFF
 *                    EVILIB_PASTEL
 *                    EVILIB_NEGART
 *                    EVILIB_SEPIA
 *                    EVILIB_BW
 *                    EVILIB_SOLARIZE
 *                    EVILIB_MOSAIC
 *                    EVILIB_SLIM
 *                    EVILIB_STRETCH
 * Return on error: 0
 */
int EVILib::PictureEffectModeInq()
{
    char c;
    pthread_mutex_lock(&BufferDispo);
    sprintf(buffer, "8%d090463FF", Id_cam);
    if(SendCommand((unsigned char *)buffer, strlen(buffer)) == 0)
    {
        pthread_mutex_unlock(&BufferDispo);
        return 0;
    }
    pthread_mutex_unlock(&BufferDispo);

// receiv 'information return'
    pthread_mutex_lock(&TakeInfo);
    if(answer < 1)
        return 0;
    pthread_mutex_lock(&AccessBuffer3);
    c = buffer3[2];
    pthread_mutex_unlock(&AccessBuffer3);
    if(c == '\x00')
        return EVILIB_OFF;
    if(c == '\x01')
        return EVILIB_PASTEL;
    if(c == '\x02')
        return EVILIB_NEGART;
    if(c == '\x03')
        return EVILIB_SEPIA;
    if(c == '\x04')
        return EVILIB_BW;
    if(c == '\x05')
        return EVILIB_SOLARIZE;
    if(c == '\x06')
        return EVILIB_MOSAIC;
    if(c == '\x07')
        return EVILIB_SLIM;
    if(c == '\x08')
        return EVILIB_STRETCH;
    return 0;
}

/*
 * Return on success: EVILIB_OFF
 *                    EVILIB_STILL
 *                    EVILIB_FLASH
 *                    EVILIB_LUMI
 *                    EVILIB_TRAIL
 * Return on error: 0
 */
int EVILib::DigitalEffectModeInq()
{
    char c;
    pthread_mutex_lock(&BufferDispo);
    sprintf(buffer, "8%d090464FF", Id_cam);
    if(SendCommand((unsigned char *)buffer, strlen(buffer)) == 0)
    {
        pthread_mutex_unlock(&BufferDispo);
        return 0;
    }
    pthread_mutex_unlock(&BufferDispo);

// receiv 'information return'
    pthread_mutex_lock(&TakeInfo);
    if(answer < 1)
        return 0;
    pthread_mutex_lock(&AccessBuffer3);
    c = buffer3[2];
    pthread_mutex_unlock(&AccessBuffer3);
    if(c == '\x00')
        return EVILIB_OFF;
    if(c == '\x01')
        return EVILIB_STILL;
    if(c == '\x02')
        return EVILIB_FLASH;
    if(c == '\x03')
        return EVILIB_LUMI;
    if(c == '\x04')
        return EVILIB_TRAIL;
    return 0;
}

/*
 * level contain the Effect level
 * Return 1 on success, 0 on error
 */
int EVILib::DigitalEffectLevelInq(int &level)
{
    level = -1;
    if(power == EVILIB_ON)
    {
        pthread_mutex_lock(&BufferDispo);
        sprintf(buffer, "8%d090465FF", Id_cam);
        if(SendCommand((unsigned char *)buffer, strlen(buffer)) == 0)
        {
            pthread_mutex_unlock(&BufferDispo);
            return 0;
        }
        pthread_mutex_unlock(&BufferDispo);

// receiv 'information return'
        pthread_mutex_lock(&TakeInfo);
        if(answer < 1)
            return 0;
        pthread_mutex_lock(&AccessBuffer3);
        level = int(deconvert(2, 1));
        pthread_mutex_unlock(&AccessBuffer3);
        return 1;
    }
    return 0;
}
#endif // __EVI_D100__

/*
 * memory: contain the preset memory for memorize camera condition
 * Return 1 on success, 0 on error
 */
int EVILib::MemoryInq(int &memory)
{
    memory = -1;
    if(power == EVILIB_ON)
    {
        pthread_mutex_lock(&BufferDispo);
        sprintf(buffer, "8%d09043FFF", Id_cam);
        if(SendCommand((unsigned char *)buffer, strlen(buffer)) == 0)
        {
            pthread_mutex_unlock(&BufferDispo);
            return 0;
        }
        pthread_mutex_unlock(&BufferDispo);

// receiv 'information return'
        pthread_mutex_lock(&TakeInfo);
        if(answer < 1)
            return 0;
        pthread_mutex_lock(&AccessBuffer3);
        memory = int(deconvert(2, 1));
        pthread_mutex_unlock(&AccessBuffer3);
        return 1;
    }
    return 0;
}

#ifdef __EVI_D30__
/*
 * Return on success: EVILIB_ON
 *                    EVILIB_OFF
 * Return on error: 0
 */
int EVILib::KeyLockInq()
{
    char c;
    pthread_mutex_lock(&BufferDispo);
    sprintf(buffer, "8%d090417FF", Id_cam);
    if(SendCommand((unsigned char *)buffer, strlen(buffer)) == 0)
    {
        pthread_mutex_unlock(&BufferDispo);
        return 0;
    }
    pthread_mutex_unlock(&BufferDispo);

// receiv 'information return'
    pthread_mutex_lock(&TakeInfo);
    if(answer < 1)
        return 0;
    pthread_mutex_lock(&AccessBuffer3);
    c = buffer3[2];
    pthread_mutex_unlock(&AccessBuffer3);
    if(c == '\x00')
        return EVILIB_OFF;
    if(c == '\x02')
        return EVILIB_ON;
    return 0;
}

/*
 * id: contain the ID of the camera
 * Return 1 on success, 0 on error
 */
int EVILib::IDInq(int &id)
{
    id = -1;
    if(power == EVILIB_ON)
    {
        pthread_mutex_lock(&BufferDispo);
        sprintf(buffer, "8%d090422FF", Id_cam);
        if(SendCommand((unsigned char *)buffer, strlen(buffer)) == 0)
        {
            pthread_mutex_unlock(&BufferDispo);
            return 0;
        }
        pthread_mutex_unlock(&BufferDispo);

// receiv 'information return'
        pthread_mutex_lock(&TakeInfo);
        if(answer < 1)
            return 0;
        pthread_mutex_lock(&AccessBuffer3);
        id = int(deconvert(2, 2));
        pthread_mutex_unlock(&AccessBuffer3);
        return 1;
    }
    return 0;
}
#endif // __EVI_D30__

/*
 * Return on success: EVILIB_NTSC
 *                    EVILIB_PAL
 * Return on error: 0
 */
int EVILib::VideoSystemInq()
{
    char c;
    pthread_mutex_lock(&BufferDispo);
    sprintf(buffer, "8%d090623FF", Id_cam);
    if(SendCommand((unsigned char *)buffer, strlen(buffer)) == 0)
    {
        pthread_mutex_unlock(&BufferDispo);
        return 0;
    }
    pthread_mutex_unlock(&BufferDispo);

// receiv 'information return'
    pthread_mutex_lock(&TakeInfo);
    if(answer < 1)
        return 0;
    pthread_mutex_lock(&AccessBuffer3);
    c = buffer3[2];
    pthread_mutex_unlock(&AccessBuffer3);
    if(c == '\x00')
        return EVILIB_NTSC;
    if(c == '\x01')
        return EVILIB_PAL;
    return 0;
}

#ifdef __EVI_D100__
/*
 * vender contain the Vender ID (1: Sony)
 * model contain the Model ID
 * rom contain the ROM version
 * socket contain the Socket numer (=2)
 * Return 1 on success, 0 on error
 */
int EVILib::DeviceTypeInq(int &vender, int &model, int &rom, int &socket)
{
    vender = -1;
    model = -1;
    rom = -1;
    socket = -1;
    pthread_mutex_lock(&BufferDispo);
    sprintf(buffer, "8%d090002FF", Id_cam);
    if(SendCommand((unsigned char *)buffer, strlen(buffer)) == 0)
    {
        pthread_mutex_unlock(&BufferDispo);
        return 0;
    }
    pthread_mutex_unlock(&BufferDispo);

// receiv 'information return'
    pthread_mutex_lock(&TakeInfo);
    if(answer < 1)
        return 0;
    pthread_mutex_lock(&AccessBuffer3);
    vender = int(deconvert(2, 2));
    model = int(deconvert(4, 2));
    rom = int(deconvert(6, 2));
    socket = int(deconvert(8, 1));
    pthread_mutex_unlock(&AccessBuffer3);
    return 1;
}
#endif // __EVI_D100__

#ifdef __EVI_D30__
/*
 * lens: contain the lens No.
 * Return 1 on success, 0 on error
 */
int EVILib::Wide_ConLensInq(int &lens)
{
    lens = -1;
    if(power == EVILIB_ON)
    {
        pthread_mutex_lock(&BufferDispo);
        sprintf(buffer, "8%d090726FF", Id_cam);
        if(SendCommand((unsigned char *)buffer, strlen(buffer)) == 0)
        {
            pthread_mutex_unlock(&BufferDispo);
            return 0;
        }
        pthread_mutex_unlock(&BufferDispo);

// receiv 'information return'
        pthread_mutex_lock(&TakeInfo);
        if(answer < 1)
            return 0;
        pthread_mutex_lock(&AccessBuffer3);
        lens = int(deconvert(3, 1));
        pthread_mutex_unlock(&AccessBuffer3);
        return 1;
    }
    return 0;
}
#endif // __EVI_D30__

/*
 * After the completion of Pan_TiltModeInq, you can check the status with the functions
 *  provided
 * Return 1 on success, 0 on error
 */
int EVILib::Pan_TiltModeInq()
{
    unsigned char alpha;
    unsigned char beta;
    int low[8];
    int high[8];
    int ct;

    pthread_mutex_lock(&BufferDispo);
    sprintf(buffer, "8%d090610FF", Id_cam);
    if(SendCommand((unsigned char *)buffer, strlen(buffer)) == 0)
    {
        pthread_mutex_unlock(&BufferDispo);
        return 0;
    }
    pthread_mutex_unlock(&BufferDispo);

// receiv 'information return'
    pthread_mutex_lock(&TakeInfo);
    if(answer < 1)
        return 0;
    pthread_mutex_lock(&AccessBuffer3);
    alpha = buffer3[2];
    beta = buffer3[3];
    pthread_mutex_unlock(&AccessBuffer3);

    bitGenerator(alpha, low);
    bitGenerator(beta, high);

    for(ct = 0; ct < 18; ct ++)
        PanTiltStatus[ct] = 0;

    if(low[0] == 0 && high[0] == 0 && high[7] == 1)
        PanTiltStatus[0] = 1;
    if(low[0] == 0 && high[0] == 0 && high[6] == 1)
        PanTiltStatus[1] = 1;
    if(low[0] == 0 && high[0] == 0 && high[5] == 1)
        PanTiltStatus[2] = 1;
    if(low[0] == 0 && high[0] == 0 && high[4] == 1)
        PanTiltStatus[3] = 1;
    if(low[0] == 0 && high[2] == 0 && high[3] == 0)
        PanTiltStatus[4] = 1;
    if(low[0] == 0 && high[2] == 0 && high[3] == 1)
        PanTiltStatus[5] = 1;
    if(low[0] == 0 && high[2] == 1 && high[3] == 0)
        PanTiltStatus[6] = 1;
    if(low[0] == 0 && low[6] == 0 && low[7] == 0 && high[0] == 0)
        PanTiltStatus[7] = 1;
    if(low[0] == 0 && low[6] == 0 && low[7] == 1 && high[0] == 0)
        PanTiltStatus[8] = 1;
    if(low[0] == 0 && low[6] == 1 && low[7] == 0 && high[0] == 0)
        PanTiltStatus[9] = 1;
    if(low[0] == 0 && low[4] == 0 && low[5] == 0 && high[0] == 0)
        PanTiltStatus[10] = 1;
    if(low[0] == 0 && low[4] == 0 && low[5] == 1 && high[0] == 0)
        PanTiltStatus[11] = 1;
    if(low[0] == 0 && low[4] == 1 && low[5] == 0 && high[0] == 0)
        PanTiltStatus[12] = 1;
    if(low[0] == 0 && low[4] == 1 && low[5] == 1 && high[0] == 0)
        PanTiltStatus[13] = 1;
    if(low[0] == 0 && low[2] == 0 && low[3] == 0 && high[0] == 0)
        PanTiltStatus[14] = 1;
    if(low[0] == 0 && low[2] == 0 && low[3] == 1 && high[0] == 0)
        PanTiltStatus[15] = 1;
    if(low[0] == 0 && low[2] == 1 && low[3] == 0 && high[0] == 0)
        PanTiltStatus[16] = 1;
    if(low[0] == 0 && low[2] == 1 && low[3] == 1 && high[0] == 0)
        PanTiltStatus[17] = 1;
    return 1;
}

/*
 * pan: contain the pan max speed of the camera
 * tilt: containt the tilt max speed of the camera
 * Return 1 on success, 0 on error
 */
int EVILib::Pan_TiltMaxSpeedInq(int &pan, int &tilt)
{
    pan = -1;
    tilt = -1;
    if(power == EVILIB_ON)
    {
        pthread_mutex_lock(&BufferDispo);
        sprintf(buffer, "8%d090610FF", Id_cam);
        if(SendCommand((unsigned char *)buffer, strlen(buffer)) == 0)
        {
            pthread_mutex_unlock(&BufferDispo);
            return 0;
        }
        pthread_mutex_unlock(&BufferDispo);

// receiv 'information return'
        pthread_mutex_lock(&TakeInfo);
        if(answer < 1)
            return 0;
        pthread_mutex_lock(&AccessBuffer3);
        pan = int(deconvert(2, 1));
        tilt = int(deconvert(3, 1));
        pthread_mutex_unlock(&AccessBuffer3);
        return 1;
    }
    return 0;
}

/*
 * pan: contain the pan position of the camera
 * tilt: containt the tilt position of the camera
 * Return 1 on success, 0 on error
 */
int EVILib::Pan_TiltPosInq(float &pan, float &tilt)
{
    pan = -1.0;
    tilt = -1.0;
    if(power == EVILIB_ON)
    {
        pthread_mutex_lock(&BufferDispo);
        sprintf(buffer, "8%d090612FF", Id_cam);
        if(SendCommand((unsigned char *)buffer, strlen(buffer)) == 0)
        {
            pthread_mutex_unlock(&BufferDispo);
            return 0;
        }
        pthread_mutex_unlock(&BufferDispo);

// receiv 'information return'
        pthread_mutex_lock(&TakeInfo);
        if(answer < 1)
            return 0;
        pthread_mutex_lock(&AccessBuffer3);
        pan = deconvert(EVILIB_maxpan, EVILIB_maxPan, 2, 4);
        tilt = deconvert(EVILIB_maxtilt, EVILIB_maxTilt, 6, 4);
        pthread_mutex_unlock(&AccessBuffer3);
        return 1;
    }
    return 0;
}

/*
 * Return on success: EVILIB_ON
 *                    EVILIB_OFF
 * Return on error: 0
 */
int EVILib::DatascreenInq()
{
    char c;
    pthread_mutex_lock(&BufferDispo);
    sprintf(buffer, "8%d090606FF", Id_cam);
    if(SendCommand((unsigned char *)buffer, strlen(buffer)) == 0)
    {
        pthread_mutex_unlock(&BufferDispo);
        return 0;
    }
    pthread_mutex_unlock(&BufferDispo);

// receiv 'information return'
    pthread_mutex_lock(&TakeInfo);
    if(answer < 1)
        return 0;
    pthread_mutex_lock(&AccessBuffer3);
    c = buffer3[2];
    pthread_mutex_unlock(&AccessBuffer3);
    if(c == '\x02')
        return EVILIB_ON;
    if(c == '\x03')
        return EVILIB_OFF;
    return 0;
}

#ifdef __EVI_D30__
/*
 * Return on success: EVILIB_NORMAL
 *                    EVILIB_ATMODE
 *                    EVILIB_MDMODE
 * Return on error: 0
 */
int EVILib::ATMD_ModeInq()
{
    char c;
    pthread_mutex_lock(&BufferDispo);
    sprintf(buffer, "8%d090722FF", Id_cam);
    if(SendCommand((unsigned char *)buffer, strlen(buffer)) == 0)
    {
        pthread_mutex_unlock(&BufferDispo);
        return 0;
    }
    pthread_mutex_unlock(&BufferDispo);

// receiv 'information return'
    pthread_mutex_lock(&TakeInfo);
    if(answer < 1)
        return 0;
    pthread_mutex_lock(&AccessBuffer3);
    c = buffer3[2];
    pthread_mutex_unlock(&AccessBuffer3);
    if(c == '\x00')
        return EVILIB_NORMAL;
    if(c == '\x01')
        return EVILIB_ATMODE;
    if(c == '\x02')
        return EVILIB_MDMODE;
    return 0;
}

/*
 * After the completion of ATModeInq, you can check the status with the functions
 *  provided
 * Return 1 on success, 0 on error
 */
int EVILib::ATModeInq()
{
    unsigned char alpha;
    unsigned char beta;
    int low[8];
    int high[8];
    int ct;

    pthread_mutex_lock(&BufferDispo);
    sprintf(buffer, "8%d090723FF", Id_cam);
    if(SendCommand((unsigned char *)buffer, strlen(buffer)) == 0)
    {
        pthread_mutex_unlock(&BufferDispo);
        return 0;
    }
    pthread_mutex_unlock(&BufferDispo);

// receiv 'information return'
    pthread_mutex_lock(&TakeInfo);
    if(answer < 1)
        return 0;
    pthread_mutex_lock(&AccessBuffer3);
    alpha = buffer3[2];
    beta = buffer3[3];
    pthread_mutex_unlock(&AccessBuffer3);

    bitGenerator(alpha, low);
    bitGenerator(beta, high);

    for(ct = 0; ct < 11; ct ++)
        ATStatus[ct] = 0;

    if(low[0] == 0 && high[0] == 0 && high[6] == 0 && high[7] == 0)
        ATStatus[0] = 1;
    if(low[0] == 0 && high[0] == 0 && high[6] == 0 && high[7] == 1)
        ATStatus[1] = 1;
    if(low[0] == 0 && high[0] == 0 && high[6] == 1 && high[7] == 0)
        ATStatus[2] = 1;
    if(low[0] == 0 && high[0] == 0 && high[5] == 1)
        ATStatus[3] = 1;
    if(low[0] == 0 && high[0] == 0 && high[4] == 1)
        ATStatus[4] = 1;
    if(low[0] == 0 && high[0] == 0 && high[3] == 1)
        ATStatus[5] = 1;
    if(low[0] == 0 && high[0] == 1 && high[2] == 1)
        ATStatus[6] = 1;
    if(low[0] == 0 && low[6] == 0 && low[7] == 0 && high[0] == 0)
        ATStatus[7] = 1;
    if(low[0] == 0 && low[6] == 0 && low[7] == 1 && high[0] == 0)
        ATStatus[8] = 1;
    if(low[0] == 0 && low[6] == 1 && low[7] == 0 && high[0] == 0)
        ATStatus[9] = 1;
    if(low[0] == 0 && low[6] == 1 && low[7] == 1 && high[0] == 0)
        ATStatus[10] = 1;

    return 1;
}

/*
 * Return on success: EVILIB_ENTRY1
 *                    EVILIB_ENTRY2
 *                    EVILIB_ENTRY3
 *                    EVILIB_ENTRY4
 * Return on error: 0
 */
int EVILib::AT_EntryInq()
{
    char c;
    pthread_mutex_lock(&BufferDispo);
    sprintf(buffer, "8%d090715FF", Id_cam);
    if(SendCommand((unsigned char *)buffer, strlen(buffer)) == 0)
    {
        pthread_mutex_unlock(&BufferDispo);
        return 0;
    }
    pthread_mutex_unlock(&BufferDispo);

// receiv 'information return'
    pthread_mutex_lock(&TakeInfo);
    if(answer < 1)
        return 0;
    pthread_mutex_lock(&AccessBuffer3);
    c = buffer3[2];
    pthread_mutex_unlock(&AccessBuffer3);
    if(c == '\x00')
        return EVILIB_ENTRY1;
    if(c == '\x01')
        return EVILIB_ENTRY2;
    if(c == '\x02')
        return EVILIB_ENTRY3;
    if(c == '\x02')
        return EVILIB_ENTRY4;
    return 0;
}

/*
 * After the completion of MDModeInq, you can check the status with the functions
 *  provided
 * Return 1 on success, 0 on error
 */
int EVILib::MDModeInq()
{
    unsigned char alpha;
    unsigned char beta;
    int low[8];
    int high[8];
    int ct;

    pthread_mutex_lock(&BufferDispo);
    sprintf(buffer, "8%d090724FF", Id_cam);
    if(SendCommand((unsigned char *)buffer, strlen(buffer)) == 0)
    {
        pthread_mutex_unlock(&BufferDispo);
        return 0;
    }
    pthread_mutex_unlock(&BufferDispo);

// receiv 'information return'
    pthread_mutex_lock(&TakeInfo);
    if(answer < 1)
        return 0;
    pthread_mutex_lock(&AccessBuffer3);
    alpha = buffer3[2];
    beta = buffer3[3];
    pthread_mutex_unlock(&AccessBuffer3);

    bitGenerator(alpha, low);
    bitGenerator(beta, high);

    for(ct = 0; ct < 9; ct ++)
        MDStatus[ct] = 0;

    if(low[0] == 0 && high[0] == 0 && high[5] == 0 && high[6] == 0 && high[7] == 0)
        MDStatus[0] = 1;
        PanTiltStatus[0] = 1;
    if(low[0] == 0 && high[0] == 0 && high[5] == 0 && high[6] == 0 && high[7] == 1)
        MDStatus[1] = 1;
        PanTiltStatus[1] = 1;
    if(low[0] == 0 && high[0] == 0 && high[5] == 0 && high[6] == 1)
        MDStatus[2] = 1;
        PanTiltStatus[2] = 1;
    if(low[0] == 0 && high[0] == 0 && high[5] == 1 && high[6] == 0)
        MDStatus[3] = 1;
        PanTiltStatus[3] = 1;
    if(low[0] == 0 && high[0] == 0 && high[5] == 1 && high[6] == 1)
        MDStatus[4] = 1;
        PanTiltStatus[4] = 1;
    if(low[0] == 0 && high[0] == 0 && high[3] == 0 && high[4] == 1)
        MDStatus[5] = 1;
        PanTiltStatus[5] = 1;
    if(low[0] == 0 && high[0] == 0 && high[3] == 1 && high[4] == 0)
        MDStatus[6] = 1;
        PanTiltStatus[6] = 1;
    if(low[0] == 0 && high[0] == 0 && high[3] == 1 && high[4] == 1)
        MDStatus[7] = 1;
        PanTiltStatus[7] = 1;
    if(low[0] == 0 && high[0] == 0 && high[2] == 1 && high[4] == 0 && high[6] == 1)
        MDStatus[8] = 1;

    return 1;
}

/*
 * Dividing a scene by 48*30 pixels, Return the center position of the detecting Frame.
 * x: 4 to 42
 * y: 3 to 27
 * status: EVILIB_SETTING
 *         EVILIB_TRACKING
 *         EVILIB_LOST
 * Return 1 on success, 0 on error
 */
int EVILib::AT_ObjetPosInq(int &x, int &y, int &status)
{
    int n;
    x = 0;
    y = 0;
    status = -1;
    if(power == EVILIB_ON)
    {
        pthread_mutex_lock(&BufferDispo);
        sprintf(buffer, "8%d090720FF", Id_cam);
        if(SendCommand((unsigned char *)buffer, strlen(buffer)) == 0)
        {
            pthread_mutex_unlock(&BufferDispo);
            return 0;
        }
        pthread_mutex_unlock(&BufferDispo);

// receiv 'information return'
        pthread_mutex_lock(&TakeInfo);
        if(answer < 1)
            return 0;
        pthread_mutex_lock(&AccessBuffer3);
        x = int(deconvert(2, 1));
        y = int(deconvert(3, 1));
        n = int(deconvert(4, 1));
        pthread_mutex_unlock(&AccessBuffer3);
        switch(n)
        {
        case 0:
            status = EVILIB_SETTING;
            break;
        case 1:
            status = EVILIB_TRACKING;
            break;
        case 2:
            status = EVILIB_LOST;
            break;
        default:
            return 0;
        }
        return 1;
    }
    return 0;
}

/*
 * Dividing a sceen by 48*30 pixles, Return the center position of the detecting Frame.
 * x: 4 to 42
 * y: 3 to 27
 * status: EVILIB_SETTING
 *         EVILIB_TRACKING
 *         EVILIB_LOST
 * Return 1 on success, 0 on error
 */
int EVILib::MD_ObjetPosInq(int &x, int &y, int &status)
{
    int n;
    x = -1;
    y = -1;
    status = -1;
    if(power == EVILIB_ON)
    {
        pthread_mutex_lock(&BufferDispo);
        sprintf(buffer, "8%d090721FF", Id_cam);
        if(SendCommand((unsigned char *)buffer, strlen(buffer)) == 0)
        {
            pthread_mutex_unlock(&BufferDispo);
            return 0;
        }
        pthread_mutex_unlock(&BufferDispo);

// receiv 'information return'
        pthread_mutex_lock(&TakeInfo);
        if(answer < 1)
            return 0;
        pthread_mutex_lock(&AccessBuffer3);
        x = int(deconvert(2, 1));
        y = int(deconvert(3, 1));
        n = int(deconvert(4, 1));
        pthread_mutex_unlock(&AccessBuffer3);
        switch(n)
        {
        case 1:
            status = EVILIB_UNDETECT;
            break;
        case 2:
            status = EVILIB_DETECTED;
            break;
        default:
            return 0;
        }
        return 1;
    }
    return 0;
}

/*
 * level: 0 to 15
 * Return 1 on success, 0 on error
 */
int EVILib::MD_YLevelInq(int &level)
{
    level = -1;
    if(power == EVILIB_ON)
    {
        pthread_mutex_lock(&BufferDispo);
        sprintf(buffer, "8%d09070BFF", Id_cam);
        if(SendCommand((unsigned char *)buffer, strlen(buffer)) == 0)
        {
            pthread_mutex_unlock(&BufferDispo);
            return 0;
        }
        pthread_mutex_unlock(&BufferDispo);

// receiv 'information return'
        pthread_mutex_lock(&TakeInfo);
        if(answer < 1)
            return 0;
        pthread_mutex_lock(&AccessBuffer3);
        level = int(deconvert(3, 1));
        pthread_mutex_unlock(&AccessBuffer3);
        return 1;
    }
    return 0;
}

/*
 * level: 0 to 15
 * Return 1 on success, 0 on error
 */
int EVILib::MD_HueLevelInq(int &level)
{
    level = -1;
    if(power == EVILIB_ON)
    {
        pthread_mutex_lock(&BufferDispo);
        sprintf(buffer, "8%d09070CFF", Id_cam);
        if(SendCommand((unsigned char *)buffer, strlen(buffer)) == 0)
        {
            pthread_mutex_unlock(&BufferDispo);
            return 0;
        }
        pthread_mutex_unlock(&BufferDispo);

// receiv 'information return'
        pthread_mutex_lock(&TakeInfo);
        if(answer < 1)
            return 0;
        pthread_mutex_lock(&AccessBuffer3);
        level = int(deconvert(3, 1));
        pthread_mutex_unlock(&AccessBuffer3);
        return 1;
    }
    return 0;
}

/*
 * size: 0 to 15
 * Return 1 on success, 0 on error
 */
int EVILib::MD_SizeInq(int &size)
{
    size = -1;
    if(power == EVILIB_ON)
    {
        pthread_mutex_lock(&BufferDispo);
        sprintf(buffer, "8%d09070DFF", Id_cam);
        if(SendCommand((unsigned char *)buffer, strlen(buffer)) == 0)
        {
            pthread_mutex_unlock(&BufferDispo);
            return 0;
        }
        pthread_mutex_unlock(&BufferDispo);

// receiv 'information return'
        pthread_mutex_lock(&TakeInfo);
        if(answer < 1)
            return 0;
        pthread_mutex_lock(&AccessBuffer3);
        size = int(deconvert(3, 1));
        pthread_mutex_unlock(&AccessBuffer3);
        return 1;
    }
    return 0;
}

/*
 * dt: 0 to 15
 * Return 1 on success, 0 on error
 */
int EVILib::MD_DispTimeInq(int &dt)
{
    dt = -1;
    if(power == EVILIB_ON)
    {
        pthread_mutex_lock(&BufferDispo);
        sprintf(buffer, "8%d09070FFF", Id_cam);
        if(SendCommand((unsigned char *)buffer, strlen(buffer)) == 0)
        {
            pthread_mutex_unlock(&BufferDispo);
            return 0;
        }
        pthread_mutex_unlock(&BufferDispo);

// receiv 'information return'
        pthread_mutex_lock(&TakeInfo);
        if(answer < 1)
            return 0;
        pthread_mutex_lock(&AccessBuffer3);
        dt = int(deconvert(3, 1));
        pthread_mutex_unlock(&AccessBuffer3);
        return 1;
    }
    return 0;
}

/*
 * Return on success: EVILIB_REFRESH_MODE1
 *                    EVILIB_REFRESH_MODE2
 *                    EVILIB_REFRESH_MODE3
 * Return on error: 0
 */
int EVILib::MD_RefModeInq()
{
    char c;
    pthread_mutex_lock(&BufferDispo);
    sprintf(buffer, "8%d090710FF", Id_cam);
    if(SendCommand((unsigned char *)buffer, strlen(buffer)) == 0)
    {
        pthread_mutex_unlock(&BufferDispo);
        return 0;
    }
    pthread_mutex_unlock(&BufferDispo);

// receiv 'information return'
    pthread_mutex_lock(&TakeInfo);
    if(answer < 1)
        return 0;
    pthread_mutex_lock(&AccessBuffer3);
    c = buffer3[2];
    pthread_mutex_unlock(&AccessBuffer3);
    if(c == '\x00')
        return EVILIB_REFRESH_MODE1;
    if(c == '\x01')
        return EVILIB_REFRESH_MODE2;
    if(c == '\x02')
        return EVILIB_REFRESH_MODE3;
    return 0;
}

/*
 * rt: 0 to 15
 * Return 1 on success, 0 on error
 */
int EVILib::MD_RefTimeInq(int &rt)
{
    rt = -1;
    if(power == EVILIB_ON)
    {
        pthread_mutex_lock(&BufferDispo);
        sprintf(buffer, "8%d090711FF", Id_cam);
        if(SendCommand((unsigned char *)buffer, strlen(buffer)) == 0)
        {
            pthread_mutex_unlock(&BufferDispo);
            return 0;
        }
        pthread_mutex_unlock(&BufferDispo);

// receiv 'information return'
        pthread_mutex_lock(&TakeInfo);
        if(answer < 1)
            return 0;
        pthread_mutex_lock(&AccessBuffer3);
        rt = int(deconvert(3, 1));
        pthread_mutex_unlock(&AccessBuffer3);
        return 1;
    }
    return 0;
}
#endif // __EVI_D30__

/*
 * Return on success: EVILIB_POWER_ON_OFF
 *                    EVILIB_ZOOM_TELE_WIDE
 *                    EVILIB_AF_ON_OFF
 *                    EVILIB_CAM_BACKLIGHT
 *                    EVILIB_CAM_MEMORY
 *                    EVILIB_PAN_TILT_DRIVE
 *                    EVILIB_AT_MODE_ON_OFF
 *                    EVILIB_MD_MODE_ON_OFF
 * Return on error: 0
 */
int EVILib::IR_ReceiveReturn()
{
    char c, d;

    pthread_mutex_lock(&TakeReturn);
    pthread_mutex_lock(&AccessBuffer3);
    c = buffer3[4];
    d = buffer3[5];
    pthread_mutex_unlock(&AccessBuffer3);
// receiv 'information return'
    if(answer < 1)
        return 0;
    if(c == '\x04' && d == '\x00')
        return EVILIB_POWER_ON_OFF;
    if(c == '\x04' && d == '\x07')
        return EVILIB_ZOOM_TELE_WIDE;
    if(c == '\x04' && d == '\x38')
        return EVILIB_AF_ON_OFF;
    if(c == '\x04' && d == '\x33')
        return EVILIB_CAM_BACKLIGHT;
    if(c == '\x04' && d == '\x3F')
        return EVILIB_CAM_MEMORY;
    if(c == '\x06' && d == '\x01')
        return EVILIB_PAN_TILT_DRIVE;
#ifdef __EVI_D30__
    if(c == '\x07' && d == '\x23')
        return EVILIB_AT_MODE_ON_OFF;
    if(c == '\x07' && d == '\x24')
        return EVILIB_MD_MODE_ON_OFF;
#endif
    return 0;
}
