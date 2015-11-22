/*
 * Nrf24Stack.cpp
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

#include "Defines.h"

#ifdef NETWORK_NRF24

#include <stdio.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/socket.h>
#include <netdb.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include "ProcessFramework.h"
#include "NRF24Stack.h"

using namespace std;
using namespace tomyGateway;

extern uint16_t getUint16(uint8_t* pos);
extern uint32_t getUint32(uint8_t* pos);
extern void setUint16(uint8_t* pos, uint16_t val);
extern void setUint32(uint8_t* pos, uint32_t val);

/*=========================================
       Class Network
 =========================================*/
Network::Network(){
    _port = Nrf24Port::getInstance();

}

Network::~Network(){

}

void Network::unicast(NWAddress64* addr64, uint16_t addr16, uint8_t* payload, uint16_t payloadLength){
    
     _port->unicast( addr16,payload,payloadLength);
}

void Network::broadcast(uint8_t* payload, uint16_t payloadLength){

    _port->multicast( payload, payloadLength);
}

bool Network::getResponse(NWResponse* response){
     uint8_t* buf = response->getPayloadPtr();
     uint16_t nodeId;
     uint16_t msgLen;
     uint8_t  msgType;
     int      recvLen;

    recvLen = _port->recv(buf,MQTTSN_MAX_FRAME_SIZE,nodeId);
    if(recvLen <= 0){
        return false;
    }else{
        if(buf[0] == 0x01){
            msgLen = getUint16(buf + 1);
            msgType = *(buf + 3);
        }else{
            msgLen = (uint16_t)*(buf);
            msgType = *(buf + 1);
        }

        if(msgLen != recvLen){
           return false;
        }

        response->setLength(msgLen);
	response->setMsgType(msgType);
	response->setClientAddress16(nodeId);
	//response->setClientAddress64(0, ipAddress);

	return true;
    }
return false;
}

int Network::initialize(Nrf24Config  config){
    return  _port->initialize(config);
}


/*=========================================
       Class Nrf24Port
 =========================================*/
std::mutex  Nrf24Port::_m;
Nrf24Port::Nrf24Port():
_radio(RPI_V2_GPIO_P1_15, BCM2835_SPI_CS0, BCM2835_SPI_SPEED_8MHZ), 
_network(_radio),
_mesh(_radio,_network),
_inited(false)
{
;
}

Nrf24Port::~Nrf24Port(){
    close();
}

void Nrf24Port::close(){

}

int Nrf24Port::initialize(Nrf24Config config){
   if (!_inited ){
       std::lock_guard<std::mutex> lock(_m);
       if( _inited)
           return 0;
       _inited = true;
       // Set the nodeID to 0 for the master node
       _mesh.setNodeID(0);
       // Connect to the mesh
       printf("start\n");
       _mesh.begin();
       _radio.printDetails();
       _config.tbd = config.tbd;
   }
    return 0;
}


int Nrf24Port::unicast(  uint16_t addr16, uint8_t* payload, uint16_t payloadLength ){
    bool    dataSent=false;

    std::lock_guard<std::mutex> lock(_m);

    _mesh.update();

    int16_t meshAdr = _mesh.getAddress(addr16);
    if (meshAdr< 0)
       return -1;
    //RF24NetworkHeader header(0,'Q');
   
    dataSent = _mesh.write(payload, 'Q', payloadLength, addr16); 

//_network.multicast(header,payload,payloadLength,1);

    printf("Unicast: MQTT to Node 0%o adr:%o data:",addr16,meshAdr);
                for (int i = 0;i < payloadLength;i++)
                   printf("%02X",payload[i]);
               printf("\n");

    return dataSent?payloadLength:0;
}

int Nrf24Port::multicast( uint8_t* payload, uint16_t payloadLength){
    std::lock_guard<std::mutex> lock(_m);
    RF24NetworkHeader header(0,'Q');
    _mesh.update();
  
    //_mesh.write(payload, 'Q', payloadLength, 4);
    _network.multicast(header,payload,payloadLength,1);

    printf("Multicast: MQTT data:");
                for (int i = 0;i < payloadLength;i++)
                   printf("%02X",payload[i]);
               printf("\n");
return 0;
}

int Nrf24Port::recv( uint8_t * buf, const uint16_t maxLen, uint16_t &nodeId){
    std::lock_guard<std::mutex> lock(_m);
    ///\todo Maybe these should be some kind of network maintenace func.
    // Call network.update as usual to keep the network updated
    _mesh.update();

    // In addition, keep the 'DHCP service' running on the master node so addresses will
    // be assigned to the sensor nodes
    _mesh.DHCP();
    RF24NetworkHeader header;
    int count=0;

   if( !_network.available() )
       return 0;

    _network.peek(header);

    switch(header.type){
        case 'Q':
        {
        count = _network.read(header,buf,maxLen); 
        if( count>0 ){
            printf("MQTT from 0%o data:",header.from_node);
                for (int i = 0;i < count;i++)
                   printf("%02X",buf[i]);
               printf("\n");

            nodeId = _mesh.getNodeID(header.from_node);
        }
        }
        break;
        default:
        count = _network.read(header,buf,maxLen); 
        if( count>0 ){
            printf("Unknown Msgtype %c from 0%o data:",header.type,header.from_node);
                for (int i = 0;i < count;i++)
                   printf("%02X",buf[i]);
               printf("\n");
        }
        count = 0;//Dont push up as Mqtt msg.
        break;
    };//swtch

    _mesh.update();

    return count;
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

#endif /* NETWORK_NRF24 */

