//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
// 

#ifndef __LORA_OMNET_SIMPLELORAAPP_H_
#define __LORA_OMNET_SIMPLELORAAPP_H_

#include <omnetpp.h>
#include <vector>
#include "inet/common/lifecycle/ILifecycle.h"
#include "inet/common/lifecycle/NodeStatus.h"
#include "inet/common/ModuleAccess.h"
#include "inet/common/lifecycle/LifecycleOperation.h"

#include "LoRaAppPacket_m.h"
#include "LoRa/LoRaMacFrame_m.h"
#include "LoRa/LoRaMacControlInfo_m.h"

using namespace omnetpp;

namespace inet {

/**
 * TODO - Generated class
 */
class receivedPacketNode
{
    public:
        LoRaAppPacket *recPktNode;
};

class receivedPacketInfo
{
    public:
        int retransmissionSeen;
        int sequenceNum;
};


class INET_API SimpleLoRaApp : public cSimpleModule, public ILifecycle
{
    protected:
        virtual void initialize(int stage) override;
        void finish() override;
        virtual int numInitStages() const override { return NUM_INIT_STAGES; }
        virtual void handleMessage(cMessage *msg) override;
        virtual bool handleOperationStage(LifecycleOperation *operation, int stage, IDoneCallback *doneCallback) override;

        void handleMessageFromLowerLayer(cMessage *msg);
        std::pair<double,double> generateUniformCircleCoordinates(double radius, double gatewayX, double gatewayY);
        void sendJoinRequest();
        void sendDownMgmtPacket();
        void sendRetransmit(LoRaAppPacket *packetForRetransmission);
        void addPacketToSeenPackets(int sequenceNum);
        bool isPacketSeen4Times();
        void copyAndSavePacketForRetransmit(LoRaAppPacket *packet);

        bool packetExists;
        int retransmissionCnt;
        int receivedRetransmits;
        int data;
        int numberOfPacketsToSend;
        int sentPackets;
        int receivedADRCommands;
        int lastSentMeasurement;
        int sentRetransmits;
        int numberOfCanceledRetransmits;
        simtime_t timeToFirstPacket;
        simtime_t timeToNextPacket;
        simtime_t timeToSendRetransmit;

        cMessage *sendRetransmission;
        cMessage *configureLoRaParameters;
        cMessage *sendMeasurements;
        cMessage *goToIdle;

        std::vector<receivedPacketInfo> receivedPackets;
        std::vector<receivedPacketNode> packetsForRetransmission;

        std::vector<LoRaAppPacket*> crazy;

        LoRaAppPacket packetForRetransmission;
        LoRaAppPacket packetForRetransmissio;


        //history of sent packets;
        cOutVector sfVector;
        cOutVector tpVector;


        //variables to control ADR
        bool evaluateADRinNode;
        int ADR_ACK_CNT = 0;
        int ADR_ACK_LIMIT = 64; //64;
        int ADR_ACK_DELAY = 32; //32;
        bool sendNextPacketWithADRACKReq = false;
        void increaseSFIfPossible();

    public:
        SimpleLoRaApp() {}
        simsignal_t LoRa_AppPacketSent;
        //LoRa physical layer parameters
        double loRaTP;
        units::values::Hz loRaCF;
        int loRaSF;
        units::values::Hz loRaBW;
        int loRaCR;
        bool loRaUseHeader;
};

}

#endif
