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
DEBUG_NWSTACK("Network const");
}

Network::~Network(){

}

void Network::unicast(NWAddress64* addr64, uint16_t addr16, uint8_t* payload, uint16_t payloadLength){
    Nrf24Port::unicast( );
}

void Network::broadcast(uint8_t* payload, uint16_t payloadLength){
    Nrf24Port::multicast( );
}

bool Network::getResponse(NWResponse* response){
return false;
}

int Network::initialize(Nrf24Config  config){
    return Nrf24Port::initialize(config);
}


/*=========================================
       Class Nrf24Port
 =========================================*/

Nrf24Port::Nrf24Port():
_radio(RPI_V2_GPIO_P1_15, BCM2835_SPI_CS0, BCM2835_SPI_SPEED_8MHZ), 
_network(_radio),
_mesh(_radio,_network)
{

DEBUG_NWSTACK("Constructor");
}

Nrf24Port::~Nrf24Port(){
    close();
}

void Nrf24Port::close(){

}

int Nrf24Port::initialize(){
   // Set the nodeID to 0 for the master node
   _mesh.setNodeID(0);
   // Connect to the mesh
   DEBUG_NWSTACK("start\n");
   _mesh.begin();
   _radio.printDetails();

    return initialize(_config);
}

int Nrf24Port::initialize(Nrf24Config config){

    _config.tbd = config.tbd;
    return 0;
}


int Nrf24Port::unicast( ){
return 0;
}

int Nrf24Port::multicast(){
return 0;
}
#define MAXSIZE 66
unsigned char buffer[MAXSIZE]={0};

int Nrf24Port::recv(){
// Call network.update as usual to keep the network updated
  _mesh.update();

  // In addition, keep the 'DHCP service' running on the master node so addresses will
  // be assigned to the sensor nodes
  _mesh.DHCP();
RF24NetworkHeader header;
int count = _network.read(header,buffer,MAXSIZE); 
                DEBUG_NWSTACK("MQTT from 0%o data:",header.from_node);
                for (int i = 0;i < count;i++)
                   DEBUG_NWSTACK("%02X",buffer[i]);
                DEBUG_NWSTACK("\n");

  return 0;
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

