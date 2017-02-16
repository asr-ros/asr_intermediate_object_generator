/**

Copyright (c) 2016, Borella Jocelyn, Meißner Pascal
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#pragma once

#include "ObjectPoint.h"
#include <iostream>
#include <fstream>
#include <vector>
#include <algorithm>
#include <boost/lexical_cast.hpp>
#include <math.h>

typedef std::shared_ptr<ObjectPoint> ObjectPointSharedPointer;

class Object {
private:
	std::string ObjectName;
    std::string ObservedId;
    ObjectPointSharedPointer Average;
	int ObjectsCount;
    int RankingMethod;
	double PosVar;
	double AverageDistanceToOtherObjects;
	double PresenceInScene;
	double NormalizedPosVar;
	double NormalizedAverageDistanceToOtherObjects;
    double rankValue;
    double Alpha;
    double Beta;
    double Gamma;
    std::vector<ObjectPointSharedPointer> PointList;

public:
    Object(); //default constructor
    Object(std::string objectName, std::string observedId,int rankingMethodEntry, double alphaEntry, double betaEntry, double gammaEntry);
    virtual ~Object();

	//getter
    ObjectPointSharedPointer getAverage() const {return this->Average;}
	double getPosVar() const {return this->PosVar;}
    ObjectPointSharedPointer getPoint(int i) const {return this->PointList[i];}
	int getObjetsCount() const {return this->ObjectsCount;}
	std::string getObjectName() const {return this->ObjectName;}
    std::string getObservedId() const {return this->ObservedId;}
	double getAverageDistanceToOtherObjects() const {return this->AverageDistanceToOtherObjects;}
	double getRankValue() const {return this->rankValue;}
	double getNormalizedPosVar() {return this->NormalizedPosVar;}
	double getNormalizedAverageDistanceToOtherObjects() {return this->NormalizedAverageDistanceToOtherObjects;}

	//setter
	void setPresenceInScene(double presence){this->PresenceInScene = presence;}
	void setNormalizedPosVar(double value){this->NormalizedPosVar = value;}
	void setNormalizedAverageDistanceToOtherObjects(double value){ this->NormalizedAverageDistanceToOtherObjects = value;}

	void CalcAveragePos();
	void CalcPosVar();
    void AddPoint(ObjectPointSharedPointer p) {this->PointList.push_back(p); ObjectsCount++;}
	void AddDistance(double d){ this->AverageDistanceToOtherObjects += d;}
	void AverageDistance(){ this->AverageDistanceToOtherObjects = this->AverageDistanceToOtherObjects/ObjectsCount;}
    void DisplayStats();
	void rank();
	void publishLogs(std::string filePath);
};

