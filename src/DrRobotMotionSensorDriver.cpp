/*
 * DrRobotMotionSensorDriver.cpp
 *
 *  Created on: Mar 14, 2011
 *      Author: dri
 */

#include "DrRobotMotionSensorDriver.hpp"
#include "DrRobotCommConst.hpp"


//#define DEBUG_ERROR           //set printf out error message
#undef DEBUG_ERROR


using namespace std;
using namespace DrRobot_MotionSensorDriver;

/*! this function is construct function for DrRobotMotionSensorDriver Class
   It will initialize all the internal variables
*/

DrRobot_MotionSensorDriver::DrRobotMotionSensorDriver::DrRobotMotionSensorDriver()
{

  _robotConfig = new DrRobotMotionConfig();

  _robotConfig->portNum = DEFAULT_PORT;
  sprintf(_robotConfig->robotID, "DrRobot");
  sprintf(_robotConfig->robotIP, "192.168.0.201");

  _nMsgLen = 0;

  // Innitializes ip address, port number, etc. to defaults.
  bzero(&_addr, sizeof(_addr));
  inet_pton(_addr.sin_family,_robotConfig->robotIP, &_addr.sin_addr);
  _addr.sin_family = AF_INET;
  _addr.sin_port = htons(_robotConfig->portNum);
  _addr_len = sizeof _addr;

  _numbytes = 0;
  
  // TODO: Identify these
  _tv.tv_sec = 0;
  _tv.tv_usec = 200;             //200us ?
  // Flag to stop communications callback loop
  _stopComm = false;
  // Count of communication attempts since last received message
  _comCnt = 0;
  // Threading stuff
  _mutex_Data_Buf = PTHREAD_MUTEX_INITIALIZER;
  // Flag for whether successful transmission has occurred at least once
  _eCommState = Disconnected;

}

DrRobot_MotionSensorDriver::DrRobotMotionSensorDriver::~DrRobotMotionSensorDriver()
{
  if (portOpen())
    close();
}

bool DrRobot_MotionSensorDriver::DrRobotMotionSensorDriver::portOpen()
{
  if ( (_eCommState == Connected) && (!_stopComm))
    return true;
  else
    return false;
}

void DrRobot_MotionSensorDriver::DrRobotMotionSensorDriver::close()
{
    
  _stopComm = true;
  _pCommThread->join();
  _eCommState = Disconnected;
  if (_sockfd > 0)
  {
    ::close(_sockfd);
    _sockfd = -1;
  }
  
}



void DrRobot_MotionSensorDriver::DrRobotMotionSensorDriver::debug_ouput(const char* errorstr)
{
#ifdef DEBUG_ERROR
    printf("DrRobot Motion Sensor: %s", errorstr);
#endif
}

int DrRobot_MotionSensorDriver::DrRobotMotionSensorDriver::openNetwork()
{
  // Allow communications
  _stopComm = false;

  // Tries to open the socket
  if ((_sockfd = socket(AF_INET, SOCK_DGRAM, 0)) == -1)
  {
    debug_ouput("Couldn't open socket.");
    return -1;
    }
  // Tries to send an acknowledgment message
  _numbytes = sendAck();
  if (_numbytes < 0)
  {
    _stopComm = true;
    perror("Following error occured on first message attempt");
    return -2;
    }

  _eCommState = Connected;
  // Starts thread for callback loop
  _pCommThread = boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(&DrRobotMotionSensorDriver::commWorkingThread, this)));
  return 0;
  }

//communication callback thread here
void DrRobot_MotionSensorDriver::DrRobotMotionSensorDriver::commWorkingThread(){
  while(!_stopComm)
  {
  FD_ZERO(&_readfds);
  FD_SET(_sockfd, &_readfds);
  select(_sockfd + 1, &_readfds, NULL, NULL, &_tv);
  if (FD_ISSET(_sockfd,&_readfds))
  {
    if ((_numbytes = recvfrom(_sockfd, _recBuf, MAXBUFLEN-1 , 0,(struct sockaddr *)&_addr, &_addr_len)) == -1)
    {
      perror("recvfrom");
      return;
      }
  // Debug stream message
  // TODO: Integrate better with debug_output method
  #ifdef DEBUG_ERROR
    printf("listener: packet is %d bytes long\n", _numbytes);
  #endif

  _comCnt = 0;
  // Processes the com data
  handleComData(_recBuf,_numbytes);
    }
  else
  {
  	// Count of consecutive failed comm attempts
    _comCnt++;
    // Wait for 10 ms
    usleep(COMM_SLEEP_TM);
    // After two seconds, announce comm loss and keep trying
    // Updates on status every two seconds.
    if (_comCnt > COMM_LOST_TH)
    {
      printf("Communication lost, trying to re-establish. IP address: %s, Port: %d \n", _robotConfig->robotIP, _robotConfig->portNum);
      _comCnt = 0;
      }
    }
  }
  return;
}

// Generates and returns a CRC value
unsigned char DrRobot_MotionSensorDriver::DrRobotMotionSensorDriver::CalculateCRC(const unsigned char *lpBuffer, const int nSize)
{
  unsigned char shift_reg, sr_lsb, data_bit, v;
  int i,j;
  unsigned char fb_bit;

  shift_reg = 0; // initialize the shift register

  for(i=0;i<nSize;i++)
  {
    v = (unsigned char)(lpBuffer[i]&0x0000FFFF);
    for(j=0;j<8;j++) // for each bit
    {
            data_bit = v & 0x01; // isolate least sign bit
            sr_lsb = shift_reg & 0x01;
            fb_bit = (data_bit^sr_lsb) & 0x01; // calculate the feed back bit
            shift_reg = shift_reg >> 1;
            if (fb_bit == 1)
                    shift_reg = shift_reg ^ 0x8C;
            v = v >> 1;
    }
  }
  return shift_reg;

}
// Sends an acknowledgment message
int DrRobot_MotionSensorDriver::DrRobotMotionSensorDriver::sendAck()
{
  // See Sendcommand for msg documentation
  unsigned char msg[] = {COM_STX0,COM_STX1,COM_TYPE_MOT,0,COMTYPE_SYSTEM,1,DATA_ACK,0,COM_ETX0,COM_ETX1};
  msg[7] = CalculateCRC(&msg[2],msg[5] + 4);
  return sendCommand(msg,10);
}


void DrRobot_MotionSensorDriver::DrRobotMotionSensorDriver::DealWithPacket(const unsigned char *lpComData, const int nLen)
{
  char debugtemp[512] ;
  int temp;
  // Double check valid headers
  if ( (lpComData[INDEX_STX0] != COM_STX0) || (lpComData[INDEX_STX1] != COM_STX1))
  {

    debugCommMessage("invalid packet header ID, discard it!\n");

    return;
    }
  else if ( lpComData[INDEX_DES] != COM_TYPE_PC )
  {
    debugCommMessage("invalid packet destination id PC, discard it!\n");
    return;
    }

  BYTE ucLength = (BYTE)lpComData[INDEX_LENGTH];

  if ( (ucLength + INDEX_DATA + 3) != nLen )
  {
    sprintf(debugtemp, "invalid packet size, discarding! Whole length: %d, Data length: %d\n", nLen, ucLength);
    std::string temp(debugtemp);
    debugCommMessage(temp);

    return;
    }

  // check ETX indicator
  if ( (lpComData[nLen-1]!=COM_ETX1) || (lpComData[nLen-2]!=COM_ETX0))
  {
    debugCommMessage("invalid packet ETX indicator, discarding!\n");
    return;
    }

  // verify CRC correction
  if ( CalculateCRC(lpComData+INDEX_DES, ucLength+4) != (BYTE)lpComData[ucLength+INDEX_DATA] )
  {
    debugCommMessage("invalid CRC, discarding!\n");
    return;
    }

  // Data looks good so far... Begin processing based on msg type
  switch ( (unsigned char)lpComData[INDEX_TYPE] )
  {
    case COMTYPE_SYSTEM:
    
      switch ( lpComData[INDEX_DATA] )
  		{
        case DATA_ACK:

          debugCommMessage("Received acknowledgment packet!\n");
          sendAck();
          debugCommMessage("Send acknowledgement!\n");
          break;

        case DATA_PING:

          debugCommMessage("Received ping packet!\n");
          break;

        case DATA_URGENT_DATA:

          debugCommMessage("Received urgent packet!\n");
          sendAck();
          break;

        case DATA_SKIPPACKET:

          debugCommMessage("Received skip packet!\n");
          sendAck();
          break;

        default:

          debugCommMessage("Invalid packet data in system packet, discarding!\n");
          return;
      }
      break;

    case COMTYPE_SENSOR:

      debugCommMessage("Received Sensor data packet!\n");
      break;

    case COMTYPE_MOTOR_SENSOR:

      debugCommMessage("Received motor sensor data packet!\n");
      pthread_mutex_lock(&_mutex_Data_Buf);

      for (int i = 0; i < MOTORSENSOR_NUM; i ++)
      {
        _motorSensorData.motorSensorPot[i] = lpComData[INDEX_DATA + 2 * i] + lpComData[INDEX_DATA + 2 * i + 1] * 256;
        _motorSensorData.motorSensorPWM[i] = lpComData[INDEX_DATA + 2 * i] + lpComData[INDEX_DATA + 2 * i + 1] * 256;
        _motorSensorData.motorSensorCurrent[i] = lpComData[INDEX_DATA + 12 + 2 * i] + lpComData[INDEX_DATA + 12 + 2 * i + 1] * 256;
        }
      /* Redundant BS!!!
      for (int i = 0; i < MOTORSENSOR_NUM; i++)
      {
        _motorSensorData.motorSensorPWM[i] = lpComData[INDEX_DATA + 2 * i] + lpComData[INDEX_DATA + 2 * i + 1] * 256;
        }
      for (int i = 0; i < MOTORSENSOR_NUM; i ++)
      {
        _motorSensorData.motorSensorCurrent[i] = lpComData[INDEX_DATA + 12 + 2 * i] + lpComData[INDEX_DATA + 12 + 2 * i + 1] * 256;
        }
      */
      // Pulls encoder data from packet
      _motorSensorData.motorSensorEncoderPos[0] = lpComData[INDEX_DATA + 24] + lpComData[INDEX_DATA + 25] * 256;
      _motorSensorData.motorSensorEncoderVel[0] = lpComData[INDEX_DATA + 26] + lpComData[INDEX_DATA + 27] * 256;
      _motorSensorData.motorSensorEncoderPos[1] = lpComData[INDEX_DATA + 28] + lpComData[INDEX_DATA + 29] * 256;
      _motorSensorData.motorSensorEncoderVel[1] = lpComData[INDEX_DATA + 30] + lpComData[INDEX_DATA + 31] * 256;

      if (lpComData[INDEX_DATA + 32] & 0x01)
      {
        // Decodes flag for moter dir of first motor sensor
        _motorSensorData.motorSensorEncoderDir[0] = 1;
        }
      else
      {
        _motorSensorData.motorSensorEncoderDir[0] = -1;
        }

      if (lpComData[INDEX_DATA + 32] & 0x02)
      {
        // Decodes flag for moter dir of first motor sensor
        _motorSensorData.motorSensorEncoderDir[1] = 1;
        }
      else
      {
        _motorSensorData.motorSensorEncoderDir[1] = -1;
        }

      pthread_mutex_unlock(&_mutex_Data_Buf);
      break;

    case COMTYPE_CUSTOM_SENSOR:

      pthread_mutex_lock(&_mutex_Data_Buf);

      for (int i = 0; i < CUSTOMSENSOR_NUM ; i ++)
      {
        _customSensorData.customADData [i]  = lpComData[INDEX_DATA + 2 * i] + lpComData[INDEX_DATA + 2 * i + 1] * 256;
        }

      _customSensorData.customIO = lpComData[INDEX_DATA + 16];
      // Loads encoder 3 data
      _motorSensorData.motorSensorEncoderPos[3] = lpComData[INDEX_DATA + 17] + lpComData[INDEX_DATA + 18] * 256;
      _motorSensorData.motorSensorEncoderVel[3] = lpComData[INDEX_DATA + 19] + lpComData[INDEX_DATA + 20] * 256;

      temp = lpComData[INDEX_DATA + 21] + lpComData[INDEX_DATA + 22] * 256;
      if (temp & 0x01)
      {
        _motorSensorData.motorSensorEncoderDir[3] = 1;
        }
      else
      {
        _motorSensorData.motorSensorEncoderDir[3] = 0;
        }
      // Loads encoder 4 data
      _motorSensorData.motorSensorEncoderPos[4] = lpComData[INDEX_DATA + 23] + lpComData[INDEX_DATA + 24] * 256;
      _motorSensorData.motorSensorEncoderVel[4] = lpComData[INDEX_DATA + 25] + lpComData[INDEX_DATA + 26] * 256;

      temp = lpComData[INDEX_DATA + 27] + lpComData[INDEX_DATA + 28] * 256;
      if (temp & 0x01)
      {
        _motorSensorData.motorSensorEncoderDir[4] = 1;
        }
      else
      {
        _motorSensorData.motorSensorEncoderDir[4] = 0;
        }
      

      pthread_mutex_unlock(&_mutex_Data_Buf);
      debugCommMessage("Received custom sensor data packet!\n");
      break;

    case COMTYPE_STANDARD_SENSOR:

      pthread_mutex_lock(&_mutex_Data_Buf);
      // Loads human sensor data
      // TODO: Does the Jaguar even have these!? If not, remove.
      for (int i = 0; i < 4 ; i ++)
      {
        _standardSensorData.humanSensorData[i] = lpComData[INDEX_DATA + 6 + 2*i] + lpComData[INDEX_DATA + 6 + 2 * i + 1] * 256;
      }
      
      _standardSensorData.overHeatSensorData[0] = lpComData[INDEX_DATA + 18] + lpComData[INDEX_DATA + 19] * 256;
      _standardSensorData.overHeatSensorData[1] = lpComData[INDEX_DATA + 20] + lpComData[INDEX_DATA + 21] * 256;
      _standardSensorData.boardPowerVol = lpComData[INDEX_DATA + 30] + lpComData[INDEX_DATA + 31] * 256;
      _standardSensorData.motorPowerVol = lpComData[INDEX_DATA + 32] + lpComData[INDEX_DATA + 33] * 256;
      _standardSensorData.servoPowerVol = lpComData[INDEX_DATA + 34] + lpComData[INDEX_DATA + 35] * 256;
      _standardSensorData.refVol = lpComData[INDEX_DATA + 36] + lpComData[INDEX_DATA + 37] * 256;
      _standardSensorData.potVol = lpComData[INDEX_DATA + 38] + lpComData[INDEX_DATA + 39] * 256;

      pthread_mutex_unlock(&_mutex_Data_Buf);
      debugCommMessage("Received standard sensor data packet!\n");
      break;

    default:

       sprintf(debugtemp, "Invalid packet data type(%#2X), discarding!\n", (unsigned char)lpComData[INDEX_TYPE] );
      debugCommMessage(debugtemp);
      return;
  }
  return;
}

void DrRobot_MotionSensorDriver::DrRobotMotionSensorDriver::handleComData(const unsigned char *data, const int nLen)
{
    unsigned char msgHeader[] = {COM_STX0,COM_STX1};
    unsigned char msgTail[] = {COM_ETX0,COM_ETX1};
    int nStartIndex, nUnProcessedPacketLen, nPacketIndex, i;
    unsigned char* unProcessedPacket = NULL;


    nStartIndex = 0;
    nPacketIndex = 2;

    // If the buffer has room
    if (_nMsgLen + nLen < MAXBUFLEN){
      // Copies the new data onto the end of the dataBuf
      memcpy(_dataBuf + _nMsgLen, data, nLen);
      // New length of valid data
      _nMsgLen += nLen;
    }
    else{
      //clear the whole internal buffer, just keep the latest packet
      memcpy(_dataBuf,data,nLen);
      _nMsgLen = nLen;
    }


    // Checks if the first 2 bytes are MG_HEAD1,2
    // keep incrementing bytes until MSG_HEAD occurs
    for(i = 0; i < _nMsgLen -1; i++){
      if ( (_dataBuf[i] != msgHeader[0]) || ( (_dataBuf[i + 1] != msgHeader[1]) && (_nMsgLen - i >= 2 )) ){
        //do nothing
      }
      else if( (_nMsgLen - i == 1) || (_nMsgLen - i == 2)){
        break;  // All junk/corrupted headers!  Continue on without a complete packet...
        //return;  // Seems like it should be a break: the next if tree handles this case...
      }
      else{
        // Found uncorrupted header
        nPacketIndex = i + 2;
        nStartIndex = i;
        break;
      }
   }

    if ( i == _nMsgLen - 1){
      //no MSG_HEAD find so far, then check if the last byte is HEAD1, if yes, just keep this byte, otherwise clear the whole received buffer
      if (_dataBuf[_nMsgLen - 1] == msgHeader[0]){
        _dataBuf[0] = msgHeader[0];
        _nMsgLen = 1;
        return;
      }
      else{
        _nMsgLen = 0;
        return;
      }

    }
    while(nPacketIndex + 1 < _nMsgLen){
      // look for the MSG_TAIL
      if ( (_dataBuf[nPacketIndex] == msgTail[0]) && (nPacketIndex + 1 < _nMsgLen) && (_dataBuf[nPacketIndex + 1] == msgTail[1])){
        // find a data package
        // Memory address of start of packet data
        unProcessedPacket = _dataBuf + nStartIndex;
        // Byte lenght of packet data
        nUnProcessedPacketLen = nPacketIndex + 2 - nStartIndex;
        // Increment past the tail
        nPacketIndex += 2;
        // Send packet off for processing
        DealWithPacket(unProcessedPacket,nUnProcessedPacketLen);
        if (nPacketIndex == _nMsgLen){
          // Hit the end of valid data; everything left is junk; erase it
          _nMsgLen = 0;
          return;
        }
        else{
          // Still unchecked data in buffer; Find the next valid head.
          while(nPacketIndex < _nMsgLen){
            if ( (_dataBuf[nPacketIndex] == msgHeader[0]) && (_dataBuf[nPacketIndex + 1] == msgHeader[1]) && (nPacketIndex + 1 < _nMsgLen)){
              // Found the next head
              nStartIndex = nPacketIndex;
              break;
            }
            else{
              nPacketIndex += 2;
            }
          }
          if (nPacketIndex < _nMsgLen){
            // Found another head in the while loop; pass.
          }
          else if(_dataBuf[_nMsgLen - 1] == msgHeader[0]){
          	// Found start of next header; move to front, reset data length
            _dataBuf[0] = msgHeader[0];
            _nMsgLen = 1;
            return;
          }
          else{
          	// No header found; rest of data is garbage to erase
            _nMsgLen = 0;
            return;
          }
        }
      }
      // No tail, continue searching.
      else{
        nPacketIndex++;
      }

    }
    // Exceeded valid data bounds with no tail found
    if (nPacketIndex >= _nMsgLen -1){
      // Still has valid data with no end in buffer
      // So move to front and reset data length, removing junk before header
      if (nStartIndex != 0){
        memcpy(_dataBuf,_dataBuf + nStartIndex,_nMsgLen - nStartIndex);
      }
      _nMsgLen = _nMsgLen - nStartIndex;
    }

    return;
}

void DrRobot_MotionSensorDriver::DrRobotMotionSensorDriver::debugCommMessage(std::string msg)
{
#ifdef DEBUG_ERROR
  printf("DEBUG DrRobot Motion/Sensor Driver: %s",msg.c_str());
#endif

}

void DrRobot_MotionSensorDriver::DrRobotMotionSensorDriver::getDrRobotMotionDriverConfig(DrRobotMotionConfig* driverConfig)
{

  strcpy(driverConfig->robotID,_robotConfig->robotID);
  driverConfig->portNum = _robotConfig->portNum;
  strcpy(driverConfig->robotIP, _robotConfig->robotIP);

  return;
}

int DrRobot_MotionSensorDriver::DrRobotMotionSensorDriver::setDrRobotMotionDriverConfig(DrRobotMotionConfig* driverConfig)
{
  // Temporary variable for debug string
  char temp[512];
  // Checks if the ip is valid
  if (inet_pton(_addr.sin_family,driverConfig->robotIP, &_addr.sin_addr) == 0)
  {
    sprintf(temp, "DrRobot Motion/Sensor Driver Error Message: invalid IP address: %s\n", driverConfig->robotIP);
    debug_ouput(temp);
    // Clears the bad data, just in case
    bzero(&_addr, sizeof(_addr));
    // Revert to previous valid ip
    inet_pton(_addr.sin_family,_robotConfig->robotIP, &_addr.sin_addr);
    return -2;
    }
  // Checks if the port number is valid
  if (driverConfig->portNum <= 0)
  {
    sprintf(temp, "DrRobot Motion/Sensor Driver Error Message: invalid Port number: %i\n", driverConfig->portNum);
    debug_ouput(temp);
    return -1;
    }
  // Loads in the validated ip, port, and the new robot ID
  strcpy(_robotConfig->robotID,driverConfig->robotID);
  _robotConfig->portNum = driverConfig->portNum;
  strcpy(_robotConfig->robotIP,driverConfig->robotIP);
  // Ensures that the portNum is in big Endian notation to comply with network protocol
  _addr.sin_port = htons(_robotConfig->portNum);
  
  return 0;
}

int DrRobot_MotionSensorDriver::DrRobotMotionSensorDriver::readMotorSensorData(MotorSensorData* motorSensorData)
{
  // Pauses callback thread to extract data (I think)
  pthread_mutex_lock(&_mutex_Data_Buf);
  memcpy(motorSensorData,&_motorSensorData,sizeof(MotorSensorData));
  pthread_mutex_unlock(&_mutex_Data_Buf);
  return 0;
}


int DrRobot_MotionSensorDriver::DrRobotMotionSensorDriver::readCustomSensorData(CustomSensorData* customSensorData)
{
  // Pauses callback thread to extract data (I think)
  pthread_mutex_lock(&_mutex_Data_Buf);
  memcpy(customSensorData, &_customSensorData, sizeof(CustomSensorData));
  pthread_mutex_unlock(&_mutex_Data_Buf);
  return 0;
 }

int DrRobot_MotionSensorDriver::DrRobotMotionSensorDriver::readStandardSensorData(StandardSensorData* standardSensorData)
{
  // Pauses callback thread to extract data (I think)
  pthread_mutex_lock(&_mutex_Data_Buf);
  memcpy(standardSensorData,&_standardSensorData,sizeof(StandardSensorData));
  pthread_mutex_unlock(&_mutex_Data_Buf);
  return 0;
}


int DrRobot_MotionSensorDriver::DrRobotMotionSensorDriver::sendMotorCtrlAllCmd(CtrlMethod ctrlMethod, const int cmd1, const int cmd2, const int cmd3, const int cmd4, const int cmd5, const int cmd6, const int time)
{
  unsigned char msg[255];
  short tempCmd = 0;
  short tempTime = time;
  if (tempTime <= 0) tempTime = 1;
  msg[0] = COM_STX0;
  msg[1] = COM_STX1;
  msg[2] = COM_TYPE_MOT;
  msg[3] = 0;
  if (ctrlMethod == PWM)
  {
    msg[4] = MOTORPWMCTRLALL;
  }
  else if(ctrlMethod == Velocity)
  {
    msg[4] = MOTORVELOCITYCTRLALL;
  }
  else if(ctrlMethod == Position)
  {
    msg[4] = MOTORPOSITIONCTRLALL;
  }

    msg[5] = 14; // number of data bytes 2*6 motor cmds + 2 time
  // TODO: Simplify this garbage
  for (int i = 0; i < 6; i ++)
  {
    if (i == 0)
    {
      tempCmd = cmd1;
    }
    if (i == 1)
    {
      tempCmd = cmd2;
    }
    if (i == 2)
    {
      tempCmd = cmd3;
    }
    if (i == 3)
    {
      tempCmd = cmd4;
    }
    if (i == 4)
    {
      tempCmd = cmd5;
    }
    if (i == 5)
    {
      tempCmd = cmd6;
    }
    memcpy(&msg[ 6 + i *2], &tempCmd, sizeof(short));
  }

  memcpy(&msg[18], &tempTime, sizeof(short));
  msg[20] = CalculateCRC(&msg[2], msg[5] + 4);
  msg[21] = COM_ETX0; msg[22] = COM_ETX1;


  return sendCommand(msg, 23);
}

int DrRobot_MotionSensorDriver::DrRobotMotionSensorDriver::sendMotorCtrlAllCmd(CtrlMethod ctrlMethod, const int cmd1, const int cmd2, const int cmd3, const int cmd4, const int cmd5, const int cmd6)
{
  unsigned char msg[255];
   short tempCmd = 0;
   msg[0] = COM_STX0;
   msg[1] = COM_STX1;
   msg[2] = COM_TYPE_MOT;
   msg[3] = 0;
   if (ctrlMethod == PWM)
    {
      msg[4] = MOTORPWMCTRLALL;
    }
    else if(ctrlMethod == Velocity)
    {
      msg[4] = MOTORVELOCITYCTRLALL;
    }
    else if(ctrlMethod == Position)
    {
      msg[4] = MOTORPOSITIONCTRLALL;
    }

    msg[5] = 12;
   for (int i = 0; i < 6; i ++)
   {
     if (i == 0)
     {
       tempCmd = cmd1;
     }
     if (i == 1)
     {
       tempCmd = cmd2;
     }
     if (i == 2)
     {
       tempCmd = cmd3;
     }
     if (i == 3)
     {
       tempCmd = cmd4;
     }
     if (i == 4)
     {
       tempCmd = cmd5;
     }
     if (i == 5)
     {
       tempCmd = cmd6;
     }
     memcpy(&msg[ 6 + i *2], &tempCmd, sizeof(short));
   }
   msg[18] = CalculateCRC(&msg[2], msg[5] + 4);
   msg[19] = COM_ETX0; msg[20] = COM_ETX1;
   return sendCommand(msg, 21);
}

int DrRobot_MotionSensorDriver::DrRobotMotionSensorDriver::sendMotorCtrlCmd(CtrlMethod ctrlMethod, const int channel, const int cmd, const int time)
{
  unsigned char msg[255];
  short tempTime = (short)( time & 0xffff);
  if ((channel < 0) || (channel > 5))
    return -1;
  msg[0] = COM_STX0;
  msg[1] = COM_STX1;
  msg[2] = COM_TYPE_MOT;
  msg[3] = 0;
  if (ctrlMethod == PWM)
  {
    msg[4] = MOTORPWMCTRL;
  }
  else if(ctrlMethod == Velocity)
  {
    msg[4] = MOTORVELOCITYCTRL;
  }
  else if(ctrlMethod == Position)
  {
    msg[4] = MOTORPOSITIONCTRL;
  }
  msg[5] = 6;
  msg[6] = (unsigned char)(channel & 0xff);
  msg[7] = (unsigned char)(cmd % 256);
  msg[8] = (unsigned char)(cmd /256);
  msg[9] = TIMEFLAG;
  memcpy(&msg[10], &tempTime, sizeof(short));
  msg[12] = CalculateCRC(&msg[2], msg[5] + 4);
  msg[13] = COM_ETX0; msg[14] = COM_ETX1;
  return sendCommand(msg, 15);
}

int DrRobot_MotionSensorDriver::DrRobotMotionSensorDriver::sendMotorCtrlCmd(CtrlMethod ctrlMethod,const int channel, const int cmd)
{
  unsigned char msg[255];
  if ((channel < 0) || (channel > 5))
     return -1;
  msg[0] = COM_STX0;
  msg[1] = COM_STX1;
  msg[2] = COM_TYPE_MOT;
  msg[3] = 0;
  if (ctrlMethod == PWM)
  {
    msg[4] = MOTORPWMCTRL;
  }
  else if(ctrlMethod == Velocity)
  {
    msg[4] = MOTORVELOCITYCTRL;
  }
  else if(ctrlMethod == Position)
  {
    msg[4] = MOTORPOSITIONCTRL;
  }
  msg[5] = 3;
  msg[6] = (unsigned char)(channel & 0xff);
  msg[7] = (unsigned char)(cmd % 256);
  msg[8] = (unsigned char)(cmd /256);
  msg[9] = CalculateCRC(&msg[2], msg[5] + 4);
  msg[10] = COM_ETX0; msg[11] = COM_ETX1;
  return sendCommand(msg, 12);
}


int DrRobot_MotionSensorDriver::DrRobotMotionSensorDriver::sendServoCtrlAllCmd(const int cmd1, const int cmd2, const int cmd3, const int cmd4, const int cmd5, const int cmd6, const int time)
{
  // TODO: Check if it's even used/needed...
  unsigned char msg[255];
  short tempCmd = 0;
  short tempTime = time;
  if (tempTime <= 0) tempTime = 1;
  msg[0] = COM_STX0;
  msg[1] = COM_STX1;
  msg[2] = COM_TYPE_MOT;
  msg[3] = 0;

  msg[4] = SERVOCTRLALL;
  msg[5] = 14;

  for (int i = 0; i < 6; i ++)
  {
    if (i == 0)
    {
      tempCmd = cmd1;
    }
    if (i == 1)
    {
      tempCmd = cmd2;
    }
    if (i == 2)
    {
      tempCmd = cmd3;
    }
    if (i == 3)
    {
      tempCmd = cmd4;
    }
    if (i == 4)
    {
      tempCmd = cmd5;
    }
    if (i == 5)
    {
      tempCmd = cmd6;
    }
    memcpy(&msg[ 6 + i *2], &tempCmd, sizeof(short));

  }

  
  memcpy(&msg[18], &tempTime, sizeof(short));
  msg[20] = CalculateCRC(&msg[2], msg[5] + 4);
  msg[21] = COM_ETX0; msg[22] = COM_ETX1;
  return sendCommand(msg, 23);

}

int DrRobot_MotionSensorDriver::DrRobotMotionSensorDriver::sendServoCtrlAllCmd(const int cmd1, const int cmd2, const int cmd3, const int cmd4, const int cmd5, const int cmd6)
{
  // TODO: Check if it's even used/needed...
  unsigned char msg[255];
  short tempCmd = 0;
  msg[0] = COM_STX0;
  msg[1] = COM_STX1;
  msg[2] = COM_TYPE_MOT;
  msg[3] = 0;

  msg[4] = SERVOCTRLALL;
  msg[5] = 12;

  for (int i = 0; i < 6; i ++)
  {
   if (i == 0)
   {
     tempCmd = cmd1;
   }
   if (i == 1)
   {
     tempCmd = cmd2;
   }
   if (i == 2)
   {
     tempCmd = cmd3;
   }
   if (i == 3)
   {
     tempCmd = cmd4;
   }
   if (i == 4)
   {
     tempCmd = cmd5;
   }
   if (i == 5)
   {
     tempCmd = cmd6;
   }
   memcpy(&msg[ 6 + i *2], &tempCmd, sizeof(short));

  }
  msg[18] = CalculateCRC(&msg[2], msg[5] + 4);
  msg[19] = COM_ETX0; msg[20] = COM_ETX1;
  return sendCommand(msg, 21);
}
int DrRobot_MotionSensorDriver::DrRobotMotionSensorDriver::sendServoCtrlCmd(const int channel, const int cmd, const int time)
{
  // TODO: Check if it's even used/needed...
  unsigned char msg[255];
  short tempTime = (short)( time & 0xffff);
  if ((channel < 0) || (channel > 5))
  return -1;
  msg[0] = COM_STX0;
  msg[1] = COM_STX1;
  msg[2] = COM_TYPE_MOT;
  msg[3] = 0;

  msg[4] = SERVOCTRL;

  msg[5] = 6;
  msg[6] = (unsigned char)(channel & 0xff);
  msg[7] = (unsigned char)(cmd % 256);
  msg[8] = (unsigned char)(cmd /256);
  msg[9] = TIMEFLAG;
  memcpy(&msg[10], &tempTime, sizeof(short));
  msg[12] = CalculateCRC(&msg[2], msg[5] + 4);
  msg[13] = COM_ETX0; msg[14] = COM_ETX1;
  return sendCommand(msg, 15);
}
int DrRobot_MotionSensorDriver::DrRobotMotionSensorDriver::sendServoCtrlCmd(const int channel, const int cmd)
{
  // TODO: Check if it's even used/needed...
  unsigned char msg[255];
  if ((channel < 0) || (channel > 5))
    return -1;
  msg[0] = COM_STX0;
  msg[1] = COM_STX1;
  msg[2] = COM_TYPE_MOT;
  msg[3] = 0;
  msg[4] = SERVOCTRL;
  msg[5] = 3;
  msg[6] = (unsigned char)(channel & 0xff);
  msg[7] = (unsigned char)(cmd % 256);
  msg[8] = (unsigned char)(cmd /256);
  msg[9] = CalculateCRC(&msg[2], msg[5] + 4);
  msg[10] = COM_ETX0; msg[11] = COM_ETX1;
  return sendCommand(msg, 12);
}

int DrRobot_MotionSensorDriver::DrRobotMotionSensorDriver::disableMotorCmd(const int channel)
{
  // TODO: Check if it's even used/needed...
  unsigned char msg[255];
  if ((channel < 0) || (channel > 5))
    return -1;
  msg[0] = COM_STX0;
  msg[1] = COM_STX1;
  msg[2] = COM_TYPE_MOT;
  msg[3] = 0;
  msg[4] = MOTORENABLE;
  msg[5] = 2;
  msg[6] = 0;
  msg[7] = (unsigned char)(channel & 0x0ff);

  msg[8] = CalculateCRC(&msg[2], msg[5] + 4);
  msg[9] = COM_ETX0; msg[10] = COM_ETX1;
  return sendCommand(msg, 11);
}
int DrRobot_MotionSensorDriver::DrRobotMotionSensorDriver::disableServoCmd(const int channel)
{
  // TODO: Check if it's even used/needed...
  unsigned char msg[255];
  if ((channel < 0) || (channel > 5))
    return -1;

  msg[0] = COM_STX0;
  msg[1] = COM_STX1;
  msg[2] = COM_TYPE_MOT;
  msg[3] = 0;
  msg[4] = MOTORENABLE;
  msg[5] = 2;
  msg[6] = 0;
  msg[7] = (unsigned char)( (channel & 0x0ff) + 6);

  msg[8] = CalculateCRC(&msg[2], msg[5] + 4);
  msg[9] = COM_ETX0; msg[10] = COM_ETX1;
  return sendCommand(msg, 11);
}

int DrRobot_MotionSensorDriver::DrRobotMotionSensorDriver::setMotorPositionCtrlPID(const int channel, const int kp, const int kd, const int ki)
{
  // TODO: Check if it's even used/needed...
  unsigned char msg[255];
  if ( (channel < 0) || (channel > 5) )
    return -1;
  msg[0] = COM_STX0;
  msg[1] = COM_STX1;
  msg[2] = COM_TYPE_MOT;
  msg[3] = 0;
  msg[4] = MOTORPARAMETERSETTING;
  msg[5] = 11;
  msg[6] = POSITIONPID;
  msg[7] = (unsigned char)(channel & 0x0ff);
  msg[8] = KP_ID;
  memcpy(&msg[9],&kp, sizeof(short));
  msg[11] = KD_ID;
  memcpy(&msg[12],&kd, sizeof(short));
  msg[14] = KI_ID;
  memcpy(&msg[15],&ki, sizeof(short));
  msg[17] = CalculateCRC(&msg[2], msg[5] + 4);
  msg[18] = COM_ETX0; msg[19] = COM_ETX1;
  return sendCommand(msg, 20);
}

int DrRobot_MotionSensorDriver::DrRobotMotionSensorDriver::setMotorVelocityCtrlPID(const int channel, const int kp, const int kd, const int ki)
{
  // TODO: Check if it's even used/needed...
  unsigned char msg[255];
  if ( (channel < 0) || (channel > 5) )
  return -1;

  msg[0] = COM_STX0;
  msg[1] = COM_STX1;
  msg[2] = COM_TYPE_MOT;
  msg[3] = 0;
  msg[4] = MOTORPARAMETERSETTING;
  msg[5] = 11;
  msg[6] = VELOCITYPID;
  msg[7] = (unsigned char)(channel & 0x0ff);
  msg[8] = KP_ID;
  memcpy(&msg[9],&kp, sizeof(short));
  msg[11] = KD_ID;
  memcpy(&msg[12],&kd, sizeof(short));
  msg[14] = KI_ID;
  memcpy(&msg[15],&ki, sizeof(short));
  msg[17] = CalculateCRC(&msg[2], msg[5] + 4);
  msg[18] = COM_ETX0; msg[19] = COM_ETX1;
  return sendCommand(msg, 20);
}
int DrRobot_MotionSensorDriver::DrRobotMotionSensorDriver::setMotorFricCompensation(const int cmd1, const int cmd2, const int cmd3, const int cmd4, const int cmd5, const int cmd6)
{
  // TODO: Check if it's even used/needed...
  unsigned char msg[255];

  msg[0] = COM_STX0;
  msg[1] = COM_STX1;
  msg[2] = COM_TYPE_MOT;
  msg[3] = 0;

  msg[4] = MOTORFRICCOMP;
  msg[5] = 12;

  memcpy(&msg[6], &cmd1, sizeof(short));
  memcpy(&msg[8], &cmd2, sizeof(short));
  memcpy(&msg[10], &cmd3, sizeof(short));
  memcpy(&msg[12], &cmd4, sizeof(short));
  memcpy(&msg[14], &cmd5, sizeof(short));
  memcpy(&msg[16], &cmd6, sizeof(short));


  msg[18] = CalculateCRC(&msg[2], msg[5] + 4);
  msg[19] = COM_ETX0; msg[20] = COM_ETX1;
  return sendCommand(msg, 21);
}

int DrRobot_MotionSensorDriver::DrRobotMotionSensorDriver::setCustomIO(const int cmd)
{
  unsigned char msg[255];

  msg[0] = COM_STX0;
  msg[1] = COM_STX1;
  msg[2] = COM_TYPE_MOT;
  msg[3] = 0;
  msg[4] = CUSTOMIO;
  msg[5] = 1;
  msg[6] = (unsigned char)(cmd & 0x0000ff);
  msg[7] = CalculateCRC(&msg[2], msg[5] + 4);
  msg[8] = COM_ETX0; msg[9] = COM_ETX1;
  return sendCommand(msg, 10);
}

// Sends the actual message
// Message format
// Index  Purpose      Source
// 0      STX0         DrRobotCommConst.hpp
// 1      STX1         DrRobotCommConst.hpp
// 2      Comm type    DrRobotCommConst.hpp
//        (PC or Mot)
// 3      Unknown      Any message function
//        (Always 0)
// 4      msg type     DrRobotCommConst.hpp
// 5      data length  DrRobotCommConst.hpp
// 6      data         Any message function
// ...    data         Any message function
// last-2 CRC          Any function
// last-1 ETX0         DrRobotCommConst.hpp
// last   ETX1         DrRobotCommConst.hpp
int DrRobot_MotionSensorDriver::DrRobotMotionSensorDriver::sendCommand(const unsigned char* msg, const int nLen)
{
  ssize_t retval = 0;
  if (!_stopComm)
  {

    if (_sockfd > 0)
    {
      int retval = sendto(_sockfd, msg, nLen, 0,(const struct sockaddr *)&_addr,sizeof(_addr));
      if (retval > 0)
      {
         return retval;
       }
       else
       {
         perror("sendto");
         return -1;
       }
    }
  }
  return -1;
}



