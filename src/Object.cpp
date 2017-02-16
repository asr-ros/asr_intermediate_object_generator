/**

Copyright (c) 2016, Borella Jocelyn, Meißner Pascal
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "Object.h"


Object::Object():ObjectName("")
{
    Average = ObjectPointSharedPointer(new ObjectPoint(0,0,0,-1)); //-1 is the ID for AveragePos point
	ObjectsCount= 0;
	PosVar = 0.0;
	AverageDistanceToOtherObjects = 0.0;
	PresenceInScene = 0.0;
	NormalizedPosVar = 0.0;
	NormalizedAverageDistanceToOtherObjects = 0.0;
	rankValue = 0.0;
	RankingMethod = 0;
	Alpha = 0;
	Beta = 0;
	Gamma = 0;
}

Object::Object(std::string objectName, std::string observedId, int rankingMethodEntry, double alphaEntry, double betaEntry, double gammaEntry)
:ObjectName(objectName),ObservedId(observedId),RankingMethod(rankingMethodEntry),Alpha(alphaEntry),Beta(betaEntry),Gamma(gammaEntry)
{
    Average = ObjectPointSharedPointer(new ObjectPoint(0,0,0,-1)); //-1 is the ID for AveragePos point
	ObjectsCount= 0;
	PosVar = 0.0;
	AverageDistanceToOtherObjects = 0.0;
	PresenceInScene = 0.0;
	NormalizedPosVar = 0.0;
	NormalizedAverageDistanceToOtherObjects = 0.0;
	rankValue = 0.0;
}

Object::~Object()
{

}

void Object::CalcAveragePos()
{
	double mx = 0;
	double my = 0;
	double mz = 0;
	//To get the average pos we "sum" the points, that compose the Object.
    for (std::vector<ObjectPointSharedPointer>::iterator pointIterator = PointList.begin() ; pointIterator != PointList.end(); ++pointIterator)
		{
                mx += (*pointIterator)->getX();
                my += (*pointIterator)->getY();
                mz += (*pointIterator)->getZ();
		}
    Average->updatePoint(mx/ObjectsCount,my/ObjectsCount,mz/ObjectsCount); //divide by the number of points recorded for the object.
    ROS_DEBUG_STREAM("IObjects::CalcAveragePos object " << this->ObjectName << " average position (x,y,z):("
                     << Average->getX() << ", " << Average->getY() << ", " << Average->getZ() << ")");
}

void Object::CalcPosVar()
{

	double vx;
	double vy;
	double vz;
	//for each point, calc the average distance to average position.
    for (std::vector<ObjectPointSharedPointer>::iterator pointIterator = PointList.begin() ; pointIterator != PointList.end(); ++pointIterator)
	{
        vx = pow((*pointIterator)->getX() - Average->getX(),2);
        vy = pow((*pointIterator)->getY() - Average->getY(),2);
        vz = pow((*pointIterator)->getZ() - Average->getZ(),2);
        ROS_DEBUG_STREAM("IObjects::CalcPosVar object " << this->ObjectName <<
                         " got values for position variation (vx,vy,vz):("
                         << vx << ", " << vy << ", " << vz << ")");
		PosVar += sqrt(vx + vy + vz);
        ROS_DEBUG_STREAM("IObjects::CalcPosVar incremented position variation " << this->PosVar);

	}
	PosVar = PosVar/ObjectsCount; //divide result by number of points.
    ROS_DEBUG_STREAM("IObjects::CalcPosVar final position variation " << this->PosVar);
}

void Object::rank()
{
	//Average the different criteria to get the rank value ( between 0 and 1 ) for the object.
	if(RankingMethod == 0)
	{
		rankValue = Alpha*PresenceInScene / (Beta*NormalizedPosVar * Gamma*NormalizedAverageDistanceToOtherObjects);
	}
	else if(RankingMethod == 1)
	{
		rankValue = Alpha*PresenceInScene + Beta/NormalizedPosVar + Beta/NormalizedAverageDistanceToOtherObjects;
	}
}

void Object::DisplayStats()
{
    ROS_DEBUG_STREAM("IObjects::DisplayStats displaying stats for object" << ObjectName);
    ROS_DEBUG_STREAM("IObjects::DisplayStats presence in scene " << PresenceInScene);
    ROS_DEBUG_STREAM("IObjects::DisplayStats average position variation " << PosVar);
    ROS_DEBUG_STREAM("IObjects::DisplayStats normalized average position variation " << NormalizedPosVar);
    ROS_DEBUG_STREAM("IObjects::DisplayStats average distance to other objects "
                     << AverageDistanceToOtherObjects);
    ROS_DEBUG_STREAM("IObjects::DisplayStats normalized average distance to other objects "
                     << NormalizedAverageDistanceToOtherObjects);
    ROS_DEBUG_STREAM("IObjects::DisplayStats rank value " << rankValue);
}

void Object::publishLogs(std::string filePath)
{
	std::ofstream file(filePath, std::ios::out | std::ios::app);
	if(file)
	{
		file << std::endl <<"Object " << this->ObjectName << std::endl;
		file << "Pos Var " << PosVar << std::endl;
		file << "Presence in scene " << PresenceInScene << std::endl;
		file << "AverageDistanceToOtherObjects " << AverageDistanceToOtherObjects << std::endl;
		file << "Normalized Pos " << NormalizedPosVar << std::endl;
		file << "Normalized distance " << NormalizedAverageDistanceToOtherObjects << std::endl;
		file << "Rank value " << rankValue << std::endl;
		file.close();
	}
}
