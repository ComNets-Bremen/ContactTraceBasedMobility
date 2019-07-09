//
// @date: 08-11-2015
// @author: Asanga Udugama (adu@comnets.uni-bremen.de)
//

#include "KWirelessInterface.h"
#include "KBaseNodeInfo.h"
#include "alphanum.hpp"
#include <fstream>
#include <sstream>
#include <string>
#include <iostream>
#include <vector>
#include <map>
#include <regex>
#include <bits/stdc++.h>

int nodeIdx = 0;
std::map<std::string, std::ifstream*> fileMap;
std::map<std::string, std::list<string>> nodeRowsMap;
std::map<std::string, int> mytime;
ifstream* filesToBeRead = NULL;


Define_Module(KWirelessInterface);
int numOfLines=50;
bool initilizeAtFirst = true;

void KWirelessInterface::initialize(int stage)
{    
    if(initilizeAtFirst){
	int totalNumNodes = getParentModule()->getParentModule()->par("numNodes");
	filesToBeRead = new ifstream[totalNumNodes];

	initilizeAtFirst = false;
    }
    


    
    if (stage == 0) {

        // get parameters

        ownMACAddress = par("ownMACAddress").stringValue();
        wirelessRange = par("wirelessRange");
        expectedNodeTypes = par("expectedNodeTypes").stringValue();
        neighbourScanInterval = par("neighbourScanInterval");
        bandwidthBitRate = par("bandwidthBitRate");
        wirelessHeaderSize = par("wirelessHeaderSize");
        logging = par("logging");
     
        // set other parameters
        broadcastMACAddress = "FF:FF:FF:FF:FF:FF";


    } else if (stage == 1) {

        
        // get own module info
        ownNodeInfo = new KBaseNodeInfo();
        
        ownNodeInfo->nodeModule = getParentModule();
        
        for (cModule::SubmoduleIterator it(getParentModule()); !it.end(); ++it) {
            ownNodeInfo->nodeMobilityModule = dynamic_cast<inet::IMobility*>(*it);
            if (ownNodeInfo->nodeMobilityModule != NULL) {
                break;
            }
        }
        
        
        ownNodeInfo->nodeWirelessIfcModule = this;
        string nodeName = ownNodeInfo->nodeModule->getFullName();
       
	string fileName = "Result_" + nodeName + ".txt";
	
	
	filesToBeRead[nodeIdx].open(fileName, std::ifstream::in);

	fileMap[nodeName] = &filesToBeRead[nodeIdx];
	nodeIdx++;
       
        

    } else if (stage == 2) {

        // get module info of all other nodes in network
        for (int id = 0; id <= getSimulation()->getLastComponentId(); id++) {
            cModule *unknownModule = getSimulation()->getModule(id);
            if (unknownModule == NULL) {
                continue;
            }

            // has to be a node type module given in expectedNodeTypes parameter
            if(strstr(expectedNodeTypes.c_str(), unknownModule->getModuleType()->getName()) == NULL) {
                continue;
            }

            // if module is a KNode or KHeraldNode but is yourself
            if (unknownModule == ownNodeInfo->nodeModule) {
                continue;
            }

            KBaseNodeInfo *nodeInfo = new KBaseNodeInfo();
            nodeInfo->nodeModule = unknownModule;
            

            // find the wireless ifc module & mobility module
            for (cModule::SubmoduleIterator it(unknownModule); !it.end(); ++it) {

                if (dynamic_cast<inet::IMobility*>(*it) != NULL) {
                    nodeInfo->nodeMobilityModule = dynamic_cast<inet::IMobility*>(*it);
                }
                if (dynamic_cast<KWirelessInterface*>(*it) != NULL) {
                    nodeInfo->nodeWirelessIfcModule = dynamic_cast<KWirelessInterface*>(*it);
                }
            }

            // wireless ifc module & mobility module must be present, else something wrong
            if (nodeInfo->nodeMobilityModule == NULL || nodeInfo->nodeWirelessIfcModule == NULL) {
                delete nodeInfo;
                continue;
            }
	    
            string nodeFullName=nodeInfo->nodeModule->getFullName();
            string nodeID2 = nodeFullName.substr(nodeFullName.find("[")+1);
	    nodeID2 = nodeID2.substr(0, nodeID2.find("]"));
            allNodeInfoList[stoi(nodeID2)]=nodeInfo;


       

            
        }
       
        // setup first event to build neighbourhood node list and send to forwarding layer
        cMessage *sendNeighEvent = new cMessage("Send Neighbourhood Event");
        sendNeighEvent->setKind(KWIRELESSINTERFACE_NEIGH_EVENT_CODE);
        scheduleAt(simTime() + neighbourScanInterval, sendNeighEvent);

        // setup pkt send event message
        sendPacketTimeoutEvent = new cMessage("Send Packet Timeout Event");
        sendPacketTimeoutEvent->setKind(KWIRELESSINTERFACE_PKTSEND_EVENT_CODE);


    } else {
        EV_FATAL <<  KWIRELESSINTERFACE_SIMMODULEINFO << "Something is radically wrong\n";
    }

}

int KWirelessInterface::numInitStages() const
{
    return 3;
}

void KWirelessInterface::handleMessage(cMessage *msg)
{
    
    // find and send neighbour list to upper layer
    if (msg->isSelfMessage() && msg->getKind() == KWIRELESSINTERFACE_NEIGH_EVENT_CODE) {


    if(simTime().dbl() == 1 ||simTime().dbl()==triggerTime.front()){        				
            				
        
		while (currentNeighbourNodeInfoList1.size() > 0) {
		    list<KBaseNodeInfo*>::iterator iteratorCurrentNeighbourNodeInfo1 = currentNeighbourNodeInfoList1.begin();
		    KBaseNodeInfo *nodeInfo = *iteratorCurrentNeighbourNodeInfo1;
		    currentNeighbourNodeInfoList1.remove(nodeInfo);
	       	 }

		
		string line;
		string row;
	    	regex regex("\\ ");
	   
		int lastSimTime=mytime[ownNodeInfo->nodeModule->getFullName()];	
		
		if(simTime().dbl() == 1 || lastSimTime == simTime().dbl()){

		       ifstream* infile = fileMap[ownNodeInfo->nodeModule->getFullName()];    

			for(int i = 0 ; i < numOfLines ; i++){
				if(getline(*infile, row)){
					nodeRowsMap[ownNodeInfo->nodeModule->getFullName()].push_back(row);
					int contactStartTime;
					int contactEndTime;
					vector <string> out1(
		                    		sregex_token_iterator(row.begin(), row.end(), regex, -1),
		                    		sregex_token_iterator()
		                    		);

					contactStartTime = stod(out1.at(0));
					contactEndTime = stod(out1.at(2));
					
					triggerTime.push_back(contactStartTime);
					triggerTime.push_back(contactEndTime + 1);
				
				}else{
					break;
				}
			
			}
			sort(triggerTime.begin(), triggerTime.end());
			
			triggerTime.erase( unique( triggerTime.begin(),triggerTime.end()),triggerTime.end());
			
		        if(nodeRowsMap[ownNodeInfo->nodeModule->getFullName()].size()>0){
			//get last row (simtime)
			string lastLine = nodeRowsMap[ownNodeInfo->nodeModule->getFullName()].back();
		        mytime[ownNodeInfo->nodeModule->getFullName()]=stod(lastLine.substr(0, lastLine.find(" ")));	
			 }

		}

			if(simTime().dbl()==triggerTime.front()){
		    				
						triggerTime.erase(triggerTime.begin());
		        			
		    				}	
			
		

		std::list<string>::iterator it = nodeRowsMap[ownNodeInfo->nodeModule->getFullName()].begin();
		while(it != nodeRowsMap[ownNodeInfo->nodeModule->getFullName()].end()){
			line = it->c_str();

	       		int nodeName;
			int time1;
			int time2;

			vector <string> out(
		                    sregex_token_iterator(line.begin(), line.end(), regex, -1),
		                    sregex_token_iterator()
		                    );

			time1 = stod(out.at(0));
			time2 = stod(out.at(2));
			nodeName = stoi(out.at(1));

		        if(simTime().dbl() < time1){
		   		break;
	       		}

			if(simTime().dbl() > (time2)){
				it = nodeRowsMap[ownNodeInfo->nodeModule->getFullName()].erase(it);
			
			}else{
				++it;
			}

		 	//break while loop if relevant node is met
	  		if(time1 <= simTime().dbl() && simTime().dbl() <= (time2)){
		    		if (find(selectedNodes.begin(), selectedNodes.end(), nodeName) == selectedNodes.end()) {

					
		        	selectedNodes.push_back(nodeName);
		        
		    		}
				
			   }
		}
		        
			sort(selectedNodes.begin(), selectedNodes.end());
	    		for(auto const& value: selectedNodes) {
		              
		               currentNeighbourNodeInfoList1.push_back(allNodeInfoList[value]);   
	    		} 

		
	 	selectedNodes.clear();
	}
        
       
	
        // if there are neighbours, send message
        if (currentNeighbourNodeInfoList1.size() > 0) {

            // build message
            int neighCount = 0;

            KNeighbourListMsg *neighListMsg = new KNeighbourListMsg("Neighbour List Msg");
            neighListMsg->setNeighbourNameListArraySize(currentNeighbourNodeInfoList1.size());
            neighListMsg->setNeighbourNameCount(currentNeighbourNodeInfoList1.size());

            list<KBaseNodeInfo*>::iterator iteratorCurrentNeighbourNodeInfo = currentNeighbourNodeInfoList1.begin();
            while (iteratorCurrentNeighbourNodeInfo != currentNeighbourNodeInfoList1.end()) {
                KBaseNodeInfo *nodeInfo = *iteratorCurrentNeighbourNodeInfo;
		
		string nodeAddress = nodeInfo->nodeModule->par("ownAddress").stringValue();
                neighListMsg->setNeighbourNameList(neighCount, nodeAddress.c_str());

                if (logging) {EV_INFO << KWIRELESSINTERFACE_SIMMODULEINFO << ">!<NI>!<" << ownMACAddress << ">!<" << nodeAddress << ">!<" 
                    << nodeInfo->nodeModule->getFullName() << "\n";}

                neighCount++;
                iteratorCurrentNeighbourNodeInfo++;
            }

            // send msg to upper layer
            send(neighListMsg, "upperLayerOut");

        }

        // setup next event to build neighbourhood node list and send to forwarding layer
        cMessage *sendNeighEvent = new cMessage("Send Neighbourhood Event");
        sendNeighEvent->setKind(KWIRELESSINTERFACE_NEIGH_EVENT_CODE);
        scheduleAt(simTime() + neighbourScanInterval, sendNeighEvent);

        delete msg;

    // trigger to send pending packet and setup new send
    } else if (msg->isSelfMessage() && msg->getKind() == KWIRELESSINTERFACE_PKTSEND_EVENT_CODE) {

        // send the pending packet out
        sendPendingMsg();

        // if there are queued packets, setup for sending the next one at top of queue
        if (!packetQueue.empty()) {

            // get next at the top of queue
            cMessage *nextMsg = packetQueue.front();
            packetQueue.pop();

            // setup for next message sending and start timer
            setupSendingMsg(nextMsg);
        }

    // process a packet (arriving from upper or lower layers)
    } else {

        cGate *gate;
        char gateName[32];

       // get message arrival gate name
        gate = msg->getArrivalGate();
        strcpy(gateName, gate->getName());

        // msg from upper layer
        if (strstr(gateName, "upperLayerIn") != NULL) {

            // if currently there is a pending msg, then queue this msg
            if (sendPacketTimeoutEvent->isScheduled()) {

                packetQueue.push(msg);

            // no queued msgs
            } else {

                // so setup for next message sending and start timer
                setupSendingMsg(msg);

              }

        // from lowerLayerIn
        } else {

            // send msg to upper layer
            send(msg, "upperLayerOut");

        }
    }
}

void KWirelessInterface::setupSendingMsg(cMessage *msg)
{   

    
    string destinationAddress = getDestinationAddress(msg);
    bool isBroadcastMsg = FALSE;
    if (destinationAddress == broadcastMACAddress) {
        isBroadcastMsg = TRUE;
    }

    // make the neighbour list at begining of msg tx (to check later if those neighbours are still there)
    list<KBaseNodeInfo*>::iterator iteratorCurrentNeighbourNodeInfo = currentNeighbourNodeInfoList1.begin();
    while (iteratorCurrentNeighbourNodeInfo != currentNeighbourNodeInfoList1.end()) {
        KBaseNodeInfo *nodeInfo = *iteratorCurrentNeighbourNodeInfo;
        string nodeAddress = nodeInfo->nodeModule->par("ownAddress").stringValue();

        // if broadcast, add all addresses to tx time neighbour list
        // if unicast, add only the specific address
        if (isBroadcastMsg || destinationAddress == nodeAddress) {
            atTxNeighbourNodeInfoList.push_back(nodeInfo);
        
        }

        iteratorCurrentNeighbourNodeInfo++;
    }



    // save the msg to send
    currentPendingMsg = msg;

    // compute transmission duration
    cPacket *currentPendingPkt = dynamic_cast<cPacket*>(currentPendingMsg);
    double bitsToSend = (currentPendingPkt->getByteLength() * 8) + (wirelessHeaderSize * 8);
    double txDuration = bitsToSend / bandwidthBitRate;

    // setup timer to trigger at tx duration
    scheduleAt(simTime() + txDuration, sendPacketTimeoutEvent);

}

void KWirelessInterface::sendPendingMsg()
{    
    //cout<<simTime()<<endl;
    // check if nodes to deliver are still in neighbourhood, if so send the packet
    list<KBaseNodeInfo*>::iterator iteratorAtTxNeighbourNodeInfo = atTxNeighbourNodeInfoList.begin();
    while (iteratorAtTxNeighbourNodeInfo != atTxNeighbourNodeInfoList.end()) {
        KBaseNodeInfo *atTxNeighbourNodeInfo = *iteratorAtTxNeighbourNodeInfo;
        string atTxNeighbourNodeAddress = atTxNeighbourNodeInfo->nodeModule->par("ownAddress").stringValue();

        list<KBaseNodeInfo*>::iterator iteratorCurrentNeighbourNodeInfo = currentNeighbourNodeInfoList1.begin();
        while (iteratorCurrentNeighbourNodeInfo != currentNeighbourNodeInfoList1.end()) {
            KBaseNodeInfo *currentNeighbourNodeInfo = *iteratorCurrentNeighbourNodeInfo;
            string currentNeighbourNodeAddress = currentNeighbourNodeInfo->nodeModule->par("ownAddress").stringValue();
            
            // check if node is still in neighbourhood
            if (atTxNeighbourNodeAddress == currentNeighbourNodeAddress) {
               
               cPacket *outPktCopy =  dynamic_cast<cPacket*>(currentPendingMsg->dup());
                
               sendDirect(outPktCopy, currentNeighbourNodeInfo->nodeModule, "radioIn");
                

                break;
            }

            iteratorCurrentNeighbourNodeInfo++;
        }

        iteratorAtTxNeighbourNodeInfo++;
    }

    // remove original message
    delete currentPendingMsg;
    currentPendingMsg = NULL;

    // remove entries in list used to check neighbour list at begining of msg tx
    while (atTxNeighbourNodeInfoList.size() > 0) {
        list<KBaseNodeInfo*>::iterator iteratorAtTxNeighbourNodeInfo = atTxNeighbourNodeInfoList.begin();
        KBaseNodeInfo *nodeInfo = *iteratorAtTxNeighbourNodeInfo;
        atTxNeighbourNodeInfoList.remove(nodeInfo);
    }


}

string KWirelessInterface::getDestinationAddress(cMessage *msg)
{
    KDataMsg *dataMsg = dynamic_cast<KDataMsg*>(msg);
    if (dataMsg) {
        return dataMsg->getDestinationAddress();
    }

    KFeedbackMsg *feedbackMsg = dynamic_cast<KFeedbackMsg*>(msg);
    if (feedbackMsg) {
        return feedbackMsg->getDestinationAddress();
    }

    KSummaryVectorMsg *summaryVectorMsg = dynamic_cast<KSummaryVectorMsg*>(msg);
    if (summaryVectorMsg) {
        return summaryVectorMsg->getDestinationAddress();
    }

    KDataRequestMsg *dataRequestMsg = dynamic_cast<KDataRequestMsg*>(msg);
    if (dataRequestMsg) {
        return dataRequestMsg->getDestinationAddress();
    }

    EV_FATAL <<  KWIRELESSINTERFACE_SIMMODULEINFO << ">!<Unknown message type. Check \"string KWirelessInterface::getDestinationAddress(cMessage *msg)\"\n";

    throw cRuntimeError("Unknown message type in KWirelessnterface");

    return string();
}

void KWirelessInterface::finish()
{
    // remove send msg timeout
    if (sendPacketTimeoutEvent->isScheduled()) {
        cancelEvent(sendPacketTimeoutEvent);
    }
    delete sendPacketTimeoutEvent;

    // remove all messages
    while(!packetQueue.empty()) {
        cMessage *nextMsg = packetQueue.front();
        packetQueue.pop();
        delete nextMsg;
    }
    if (currentPendingMsg != NULL) {
        delete currentPendingMsg;
        currentPendingMsg = NULL;
    }

	string nodeName = ownNodeInfo->nodeModule->getFullName();
	string fileName = "Result_" + nodeName + ".txt";

	ifstream* toBeClosed = fileMap[nodeName];
	if(toBeClosed->is_open()){
		
		toBeClosed->close();
	}
	
}




