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

#include <ISM/utility/TableHelper.hpp>
#include "Evaluator.h"
#include "Scene.h"
#include "ros/ros.h"

typedef std::shared_ptr<Scene> SceneSharedPointer;

class Domain {
private:
	std::string DomainName;
    std::vector<SceneSharedPointer> SceneList;
	double GainValueThreshold;
	int RankingMethod;
	double Alpha;
	double Beta;
	double Gamma;
    std::string LogPath;
    std::string NBVPath;
    std::string AutomatPath;
    std::string Scene_path;
    std::list<objectTupleSharedPtr> IntermediateOjects;
    ros::NodeHandle LocalHandle;

public:
	Domain();
	virtual ~Domain();

	void calcIntermediateObjectsForDomain();
    void getDomainFromParams();
	void publishLogs();
    std::list<objectTupleSharedPtr> getIntermediateOjects() const{return this->IntermediateOjects;}
    void publishIntermediateObjectToAutomat();
};

