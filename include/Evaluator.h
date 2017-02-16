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

#include <string>
#include <tuple>
#include <list>
#include <unordered_map>
#include <fstream>
#include <memory>
#include "ros/ros.h"
#include <sqlite3.h>


// Rank value, Scene name, Object meta name, object name, object observed id
typedef std::tuple<double,std::string,std::string,std::string,std::string> evalulatorTuple;
typedef std::shared_ptr<evalulatorTuple> evalulatorTupleSharedPtr;
// Rank value, Object meta name, object name, object observed id
typedef std::tuple<double,std::string,std::string,std::string> objectTuple;
typedef std::shared_ptr<objectTuple> objectTupleSharedPtr;

class Evaluator {
private:
    bool Validation(std::list<objectTupleSharedPtr> listToValidate);
    bool Evaluate(std::list<objectTupleSharedPtr> oldList, std::list<objectTupleSharedPtr> newList);
    double CalcAverageF(std::list<objectTupleSharedPtr> list);
    void PublishIntermediateObjectForWorldModel();
    void PublishIntermediateObjectInDomainTable();
    int ObjectListSize;
    std::unordered_map<std::string,std::list<std::string> > SceneMap;
    std::list<objectTupleSharedPtr> ProcessedObjectList;
	double GainValueThreshold;
    std::string NBVPath;
    std::string DbPath;

public:
	Evaluator();
    Evaluator(std::list<evalulatorTupleSharedPtr> objectListEntry, std::list<std::string> sceneListEntry, double GainValueThresholdEntry, std::string NBVPath, std::string dbPath);
	virtual ~Evaluator();
    std::list<objectTupleSharedPtr> getIntermediateObjects();
};

