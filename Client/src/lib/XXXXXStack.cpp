/*
 * XXXXXStack.cpp
 *
 *                      The BSD License
 *
 *           Copyright (c) 2014, tomoaki@tomy-tech.com
 *                    All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 *
 *  Created on: 2014/06/01
 *    Modified:
 *      Author: Tomoaki YAMAGUCHI
 *     Version: 0.0.0
 */

#ifndef ARDUINO
        #include "MQTTSN_Application.h"
        #include "Network.h"
#else
        #include <MQTTSN_Application.h>
        #include <Network.h>
#endif

#ifdef NETWORK_XXXXX

#ifdef ARDUINO
  #include <XXXXXStack.h>
  #include <mqUtil.h>
  #include <SPI.h>
  #include <Ethernet.h>
  #include <EthernetUdp.h>

  #if defined( NW_DEBUG) || defined(MQTTSN_DEBUG)
        #include <SoftwareSerial.h>
        extern SoftwareSerial debug;
  #endif

#endif  /* ARDUINO */

#ifdef MBED
        #include "mbed.h"
        #include "XXXXXStack.h"
        #include "mqUtil.h"
#endif /* MBED */

#ifdef LINUX
        #include "XXXXXStack.h"
        #include "mqUtil.h"
        #include <stdio.h>
        #include <sys/time.h>
        #include <sys/types.h>
        #include <sys/socket.h>
        #include <sys/stat.h>
        #include <unistd.h>
        #include <stdlib.h>
        #include <string.h>
        #include <fcntl.h>
        #include <errno.h>
        #include <termios.h>

#endif /* LINUX */

using namespace std;
using namespace tomyClient;

extern uint16_t getUint16(uint8_t* pos);
extern uint32_t getUint32(uint8_t* pos);
extern void setUint16(uint8_t* pos, uint16_t val);
extern void setUint32(uint8_t* pos, uint32_t val);

/*=========================================
       Class Network
 =========================================*/
Network::Network():_rxCallbackPtr(0),_returnCode(0){
}

Network::~Network(){

}

void Network::unicast(NWAddress64* addr64, uint16_t addr16, uint8_t* payload, uint16_t payloadLength){
	XXXXXPort::unicast( );
}

void Network::broadcast(uint8_t* payload, uint16_t payloadLength){
	XXXXXPort::multicast( );
}

bool Network::getResponse(NWResponse* response){

}

int Network::initialize(XXXXXConfig  config){
	return XXXXXPort::initialize(config);
}



void Network::setRxHandler(void (*callbackPtr)(NWResponse* data, int* returnCode)){
    _rxCallbackPtr = callbackPtr;
}
void Network::send(uint8_t* xmitData, uint8_t dataLen, SendReqType type)
{
   printf("MsgOut(Type%d,len%d):",type,dataLen);
   for(int i=0;i<dataLen;i++)
       printf("%0.2X ",xmitData[i]);
   printf("\n");
}//todo;
int  Network::readPacket(uint8_t type){
    _returnCode = 0;
     printf("Wait something from the net...type %X\n",type);
    return _returnCode;
}//todo
void Network::setGwAddress(){
    //todo: Jahka ymmärtää miten verkkoa osoitetaan nrlf2401 verkossa..

}
void Network::resetGwAddress(void){
//todo: Jahka ymmärtää miten verkkoa osoitetaan nrlf2401 verkossa..
}
/*=========================================
       Class XXXXXPort
 =========================================*/

XXXXXPort::XXXXXPort(){


}

XXXXXPort::~XXXXXPort(){
    close();
}

void XXXXXPort::close(){

}

int XXXXXPort::initialize(){
	return initialize(_config);
}

int XXXXXPort::initialize(XXXXXConfig config){

	_config.param1 = config.param1;
	_config.param2 = config.param2;
	_config.param3 = config.param3;


}


int XXXXXPort::unicast( ){

}

int XXXXXPort::multicast(){

}

int XXXXXPort::recv(){

}


/*=========================================
             Class NLLongAddress
 =========================================*/
NWAddress64::NWAddress64(){
    _msb = _lsb = 0;
}

NWAddress64::NWAddress64(uint32_t msb, uint32_t lsb){
    _msb = msb;
    _lsb = lsb;
}

uint32_t NWAddress64::getMsb(){
    return _msb;
}

uint32_t NWAddress64::getLsb(){
    return _lsb;
}

void NWAddress64::setMsb(uint32_t msb){
    _msb = msb;
}

void NWAddress64::setLsb(uint32_t lsb){
    _lsb = lsb;
}

bool NWAddress64::operator==(NWAddress64& addr){
	if(_msb == addr.getMsb() && _lsb == addr.getLsb()){
		return true;
	}else{
		return false;
	}
}

/*=========================================
             Class ZBResponse
 =========================================*/
NWResponse::NWResponse(){
    _addr16 = 0;
    memset( _frameDataPtr, 0, MQTTSN_MAX_FRAME_SIZE);
}

uint8_t  NWResponse::getFrameLength(){
	return _len;
}

void NWResponse::setLength(uint16_t len){
	_len = len;
}

NWAddress64*  NWResponse::getClientAddress64(){
    return &_addr64;
}

uint16_t NWResponse::getClientAddress16(){
  return _addr16;
}

void  NWResponse::setClientAddress64(uint32_t msb, uint32_t lsb){
    _addr64.setMsb(msb);
    _addr64.setLsb(lsb);
}

void  NWResponse::setClientAddress16(uint16_t addr16){
	_addr16 = addr16;
}

void NWResponse::setMsgType(uint8_t type){
	_type = type;
}


uint8_t NWResponse::getMsgType(){
	if(_len > 255){
		return _frameDataPtr[3];
	}else{
		return _frameDataPtr[1];
	}
}

uint8_t* NWResponse::getBody(){
	if(_len > 255){
		return _frameDataPtr + 4;
	}else{
		return _frameDataPtr + 2;
	}
}

uint16_t NWResponse::getBodyLength(){
	if(_len > 255){
		return getPayloadLength() - 4;
	}else{
		return getPayloadLength() - 2;
	}
}

uint8_t NWResponse::getPayload(uint8_t index){
		return _frameDataPtr[index + 2];

}

uint8_t* NWResponse::getPayloadPtr(){

		return _frameDataPtr;

}

uint8_t NWResponse::getPayloadLength(){

	return _len;
}

#endif /* NETWORK_XXXXX */
