
#include "WirelessInterface.h"
#include <iostream>
#include <fstream>
#include <time.h>
#include <string>
#include <iomanip>
#include <cstdlib>
#include <sstream>
#include <dirent.h>
#include <errno.h>


Define_Module(WirelessInterface);

time_t t = time(0);
struct tm * now = localtime(&t);
char buffer [80];
std::string filePath = "";
std::string folderPath = "";
double Time= 604800;
ofstream myfile;


map<int, multimap<double, string>> nodeMapList;


void WirelessInterface::initialize(int stage)
{               
                
	  	strftime (buffer,sizeof(buffer),"%Y-%m-%d_",now);
		folderPath = "/home/ramesh/ContactMMCheck/results/Temp_" + std::string(buffer) + (string)__TIME__;
		DIR* dir = opendir(folderPath.c_str());
		if(dir){
			closedir(dir);
		}else if (ENOENT == errno){
			const int dir_err = system(("mkdir "+folderPath).c_str());
		}
		
        if (stage == 0) {

        nodeIndex = par("nodeIndex");
        wirelessRange = par("wirelessRange");

        ownName = getParentModule()->getFullName();
        for (cModule::SubmoduleIterator it(getParentModule()); !it.end(); ++it) {
            ownMobilityModule = dynamic_cast<inet::IMobility*>(*it);
            if (ownMobilityModule != NULL) {
                break;
            }
        }

        if (nodeIndex == 1) {
            numContacts = 0;
            sumContactDurations = 0.0;
            sumNeighbourhoodSize = 0;
            totNeighbourhoodReportingTimes = 0;
            
        }

    } else if (stage == 1) {

        for (int id = 0; id <= getSimulation()->getLastComponentId(); id++) {
            cModule *unknownModule = getSimulation()->getModule(id);
            if (unknownModule == NULL) {
                continue;
            }
            inet::IMobility *mobilityModule = dynamic_cast<inet::IMobility*>(unknownModule);
            if (mobilityModule == NULL) {
                continue;
            }

            if (strstr(unknownModule->getParentModule()->getFullName(), ownName.c_str()) != NULL) {
                continue;
            }

            NodeInfo *nodeInfo = new NodeInfo();
            nodeInfo->nodeMobilityModule = mobilityModule;
            nodeInfo->nodeName = unknownModule->getParentModule()->getFullName();
            nodeInfo->contactStartTime = 0.0;
            nodeInfo->contactStarted = true;
            allNodeInfoList.push_back(nodeInfo);
            

        }

        cMessage *checkNeighboursEvent = new cMessage("Check Neighbours Event");
        scheduleAt(simTime() + 1.0, checkNeighboursEvent);
    }

}

int WirelessInterface::numInitStages() const
{
    return 2;
}



void WirelessInterface::handleMessage(cMessage *msg)
{
    if (msg->isSelfMessage()) {
       

        // init the new list
        while (newNeighbourNodeInfoList.size() > 0) {
            list<NodeInfo*>::iterator iteratorNeighbourNodeInfo = newNeighbourNodeInfoList.begin();
            NodeInfo *nodeInfo = *iteratorNeighbourNodeInfo;
            newNeighbourNodeInfoList.remove(nodeInfo);
        }

        // get current position of self
        inet::Coord ownCoord = ownMobilityModule->getCurrentPosition();
        // make the new neighbour list
        list<NodeInfo*>::iterator iteratorNeighbourNodeInfo = allNodeInfoList.begin();
        while (iteratorNeighbourNodeInfo != allNodeInfoList.end()) {
            NodeInfo *nodeInfo = *iteratorNeighbourNodeInfo;
            inet::Coord neighCoord = nodeInfo->nodeMobilityModule->getCurrentPosition();

            double l = ((neighCoord.x - ownCoord.x) * (neighCoord.x - ownCoord.x))
                + ((neighCoord.y - ownCoord.y) * (neighCoord.y - ownCoord.y));
            double r = wirelessRange * wirelessRange;
             
   
            if (l <= r) {
                 
                newNeighbourNodeInfoList.push_back(nodeInfo);
                
               
            }
            iteratorNeighbourNodeInfo++;
        }
	if (newNeighbourNodeInfoList.size() > 0) {
	    sumNeighbourhoodSize += newNeighbourNodeInfoList.size();
            totNeighbourhoodReportingTimes++;
            // ANS = accumulated neighbourhood size
            // TNRT = total neighbourhood reporting times
            //EV_INFO<< " " << simTime() << " " << ownName << " ANS " << sumNeighbourhoodSize << " TNRT " << totNeighbourhoodReportingTimes << "\n";
        }


        // check and update left neighbours
        list<NodeInfo*>::iterator iteratorOldNeighbourNodeInfo = currentNeighbourNodeInfoList.begin();
        while (iteratorOldNeighbourNodeInfo != currentNeighbourNodeInfoList.end()) {
            NodeInfo *oldNodeInfo = *iteratorOldNeighbourNodeInfo;

            bool found = false;
            list<NodeInfo*>::iterator iteratorNewNeighbourNodeInfo = newNeighbourNodeInfoList.begin();
            while (iteratorNewNeighbourNodeInfo != newNeighbourNodeInfoList.end()) {
                NodeInfo *newNodeInfo = *iteratorNewNeighbourNodeInfo;

                if (strstr(oldNodeInfo->nodeName.c_str(), newNodeInfo->nodeName.c_str()) != NULL) {
                    found = true;

		    if(simTime().dbl() == Time){
			double contactDuration = simTime().dbl() - oldNodeInfo->contactStartTime  ;
                        double contactEnd = simTime().dbl();
		 	string nodeID = ownName.substr(ownName.find("[")+1);
			nodeID = nodeID.substr(0, nodeID.find("]"));
			int nodeIDInt = stoi(nodeID);
			
			string nodeID1 = oldNodeInfo->nodeName.substr(ownName.find("[")+1);
			nodeID1 = nodeID1.substr(0, nodeID1.find("]"));

			double contactSecond = simTime().dbl()-contactDuration  ;

			if(nodeMapList.find(nodeIDInt) != nodeMapList.end()){
				//map contains the key and relevant map
				nodeMapList.at(nodeIDInt).insert(pair<double, string>(contactSecond, nodeID1 +" " + to_string(contactEnd) ));

			}else{
				//insert new map
				multimap<double, string> nodeRows;
				nodeRows.insert(pair<double, string>(contactSecond, nodeID1 +" " + to_string(contactEnd) ));
				nodeMapList.insert(pair<int, multimap<double, string>>(nodeIDInt, nodeRows));
			}
	            }	

                    break;
                    

                }
                iteratorNewNeighbourNodeInfo++;
            }

            if (!found) {

                double contactDuration = simTime().dbl() - oldNodeInfo->contactStartTime -1.0 ;
                double contactEnd = simTime().dbl()-1.0;
           
		
                //EV_INFO << " " << ownName << " says: Contact with " << oldNodeInfo->nodeName << " ended at " << simTime().dbl() << " seconds - Contact duration was " << contactDuration << " seconds \n";

	 	string nodeID = ownName.substr(ownName.find("[")+1);
		nodeID = nodeID.substr(0, nodeID.find("]"));
		int nodeIDInt = stoi(nodeID);


		string nodeID1 = oldNodeInfo->nodeName.substr(ownName.find("[")+1);
		nodeID1 = nodeID1.substr(0, nodeID1.find("]"));

		double contactSecond = simTime().dbl()-contactDuration -1.0;

		if(nodeMapList.find(nodeIDInt) != nodeMapList.end()){
			//map contains the key and relevant map
			nodeMapList.at(nodeIDInt).insert(pair<double, string>(contactSecond, nodeID1 +" " + to_string(contactEnd)));
		

		}else{
			//insert new map
			multimap<double, string> nodeRows;
			nodeRows.insert(pair<double, string>(contactSecond, nodeID1 +" " + to_string(contactEnd)));
			
			nodeMapList.insert(pair<int, multimap<double, string>>(nodeIDInt, nodeRows));
		}

           
                oldNodeInfo->contactStarted = false;
                oldNodeInfo->contactStartTime = 0.0;
                currentNeighbourNodeInfoList.remove(oldNodeInfo);

                if (contactDuration > 0.0) {
                    sumContactDurations += contactDuration;
                    numContacts++;
                    // ACD = accumilated contact durations
                    // TNC = total number of contacts upto now
                   
                   //EV_INFO << " " << simTime() << " " << ownName <<" "<<ownCoord.x <<" "<<ownCoord.y<<" ACD " << sumContactDurations << " TNC " << numContacts << "\n";
 
                }
            }

            if (!found) {
                iteratorOldNeighbourNodeInfo = currentNeighbourNodeInfoList.begin();
            } else {
                iteratorOldNeighbourNodeInfo++;
            }
        }

        // check and update new neighbours
        list<NodeInfo*>::iterator iteratorNewNeighbourNodeInfo = newNeighbourNodeInfoList.begin();
        while (iteratorNewNeighbourNodeInfo != newNeighbourNodeInfoList.end()) {
            NodeInfo *newNodeInfo = *iteratorNewNeighbourNodeInfo;

            bool found = false;
            list<NodeInfo*>::iterator iteratorOldNeighbourNodeInfo = currentNeighbourNodeInfoList.begin();
            while (iteratorOldNeighbourNodeInfo != currentNeighbourNodeInfoList.end()) {
                NodeInfo *oldNodeInfo = *iteratorOldNeighbourNodeInfo;

                if (strstr(newNodeInfo->nodeName.c_str(), oldNodeInfo->nodeName.c_str()) != NULL) {
                    found = true;
                    break;
                }
                iteratorOldNeighbourNodeInfo++;
            }

            if (!found) {
                //EV_INFO << ownName << " says: Contact with " << newNodeInfo->nodeName << " started at " << simTime().dbl() << " seconds \n";

                //double contactDuration = simTime().dbl() - newNodeInfo->contactStartTime;

                //EV_INFO << " " << ownName << " says: Contact with " << oldNodeInfo->nodeName << " ended at " << simTime().dbl() << " seconds - Contact duration was " << contactDuration << " seconds \n";

                newNodeInfo->contactStarted = true;
                newNodeInfo->contactStartTime = simTime().dbl();
                currentNeighbourNodeInfoList.push_back(newNodeInfo);
            }
            iteratorNewNeighbourNodeInfo++;
        }

        scheduleAt(simTime() + 1.0, msg);
    }
 
}

void WirelessInterface::finish(){
   
    
	string nodeID = ownName.substr(ownName.find("[")+1);
	nodeID = nodeID.substr(0, nodeID.find("]"));
	int nodeIDInt = stoi(nodeID);                
	multimap<double, string> nodeRows = nodeMapList.at(nodeIDInt);
	filePath = folderPath + "/Result_" + ownName + ".txt";
	myfile.open (filePath, ios::app);              
	for(map<double, string>::iterator it = nodeRows.begin(); it != nodeRows.end(); it++){		
	     myfile << it->first << " " << it->second << "\n";
	     myfile.close();
        }
}



