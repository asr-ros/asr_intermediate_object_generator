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

#include <iostream>
#include <sqlite3.h>
#include <vector>
#include <unordered_map>
#include <algorithm>
#include <functional>
#include <boost/lexical_cast.hpp>
#include <math.h>
#include <sstream>
#include "Object.h"

typedef std::shared_ptr<Object> ObjectSharedPointer;

class Scene {
private:
    std::unordered_map<std::string,ObjectSharedPointer> ObjectMap;
	std::string DbName;
	std::string SceneName;
	int SceneObjectsCount;
	int RankingMethod;
	double Alpha;
	double Beta;
	double Gamma;
public:
	Scene();
    Scene(std::string dbName, std::string sceneName, int rankingMethodEntry, double alphaEntry, double betaEntry, double gammaEntry);
	virtual ~Scene();

    //Getter
	std::string const getSceneName(){return this->SceneName;}
	std::string const getDbName(){return this->DbName;}
    std::unordered_map<std::string,ObjectSharedPointer> const getObjectMap(){return this->ObjectMap;}
	void getObjectFromDb();
	void getNumberSet();

	void calcAveragePositionForEachObject();
	void getObjectAverageDistance();
	void calcPresenceInSceneForEachObject();
    void displayStats();
	void normalize();
	void rank();
	void publishLogs(std::string filePath);
};
