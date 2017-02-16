/**

Copyright (c) 2016, Borella Jocelyn, Meißner Pascal
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/


#include "Domain.h"
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string.hpp>
#include <fstream>
#include <chrono>
#include <ctime>


Domain::Domain()
{
    getDomainFromParams();
}

Domain::~Domain()
{
}

void Domain::calcIntermediateObjectsForDomain()
{
    //Calculate intermediate object ranking for each object in each scene
    for (std::vector<SceneSharedPointer>::iterator sceneIterator = SceneList.begin(); sceneIterator != SceneList.end(); ++sceneIterator)
	{
        (*sceneIterator)->getObjectFromDb();
        (*sceneIterator)->calcAveragePositionForEachObject();
        (*sceneIterator)->calcPresenceInSceneForEachObject();
        (*sceneIterator)->getObjectAverageDistance();
        (*sceneIterator)->normalize();
        (*sceneIterator)->rank();
        (*sceneIterator)->displayStats();
	}
    std::list<evalulatorTupleSharedPtr> evaluatorTupleList;
	std::list<std::string> sceneList;
    ROS_DEBUG_STREAM("Domain::calcIntermediateObjectsForDomain begin scene fusion");
    //Creation of an evaluator tuple for all object in all scenes. (1 evaluator tuple per object per scene)
    for (std::vector<SceneSharedPointer>::iterator sceneIterator = SceneList.begin(); sceneIterator != SceneList.end(); ++sceneIterator)
	{
        std::string sceneName = (*sceneIterator)->getSceneName();
		sceneList.push_back(sceneName);
        std::unordered_map<std::string,ObjectSharedPointer> bufferObjectMap = (*sceneIterator)->getObjectMap();
        for (std::unordered_map<std::string,ObjectSharedPointer>::iterator objectMapIterator=bufferObjectMap.begin(); objectMapIterator!=bufferObjectMap.end(); ++objectMapIterator)
		{
            evaluatorTupleList.push_back(evalulatorTupleSharedPtr(new evalulatorTuple(objectMapIterator->second->getRankValue(),sceneName,objectMapIterator->first,
                                           objectMapIterator->second->getObjectName(),objectMapIterator->second->getObservedId())));
		}
	}
    ROS_DEBUG_STREAM("Domain::calcIntermediateObjectsForDomain scene fusion completed");
    ROS_DEBUG_STREAM("Domain::calcIntermediateObjectsForDomain begin from evaluation");
    Evaluator evaluator = Evaluator(evaluatorTupleList,sceneList,GainValueThreshold,NBVPath,Scene_path);
	IntermediateOjects = evaluator.getIntermediateObjects();
    ROS_DEBUG_STREAM("Domain::calcIntermediateObjectsForDomain end from evaluation");
}

void Domain::getDomainFromParams()
{
    LocalHandle.getParam(LocalHandle.getNamespace() + "/asr_intermediate_object_generator/RankingMethod",RankingMethod);
    LocalHandle.getParam(LocalHandle.getNamespace() + "/asr_intermediate_object_generator/Alpha",Alpha);
    LocalHandle.getParam(LocalHandle.getNamespace() + "/asr_intermediate_object_generator/Beta",Beta);
    LocalHandle.getParam(LocalHandle.getNamespace() + "/asr_intermediate_object_generator/Gamma",Gamma);
    LocalHandle.getParam(LocalHandle.getNamespace() + "/asr_intermediate_object_generator/GainValueThreshold",GainValueThreshold);
    LocalHandle.getParam(LocalHandle.getNamespace() + "/asr_intermediate_object_generator/DomainName",DomainName);
    LocalHandle.getParam(LocalHandle.getNamespace() + "/asr_intermediate_object_generator/AutomatPath", AutomatPath);
    LocalHandle.getParam(LocalHandle.getNamespace() + "/asr_intermediate_object_generator/LogPath", LogPath);
    LocalHandle.getParam(LocalHandle.getNamespace() + "/asr_intermediate_object_generator/NBVPath", NBVPath);

    LocalHandle.getParam(LocalHandle.getNamespace() +  "/asr_intermediate_object_generator/dbfilename", Scene_path);


    std::vector<std::string> pathSplit;
    boost::split(pathSplit, Scene_path, boost::is_any_of("/"));
    std::string dbName = pathSplit[pathSplit.size()-1];

    const std::string placeholder = "XXX";
    std::string::size_type pos;
    if ((pos = AutomatPath.find(placeholder)) != std::string::npos) {
        AutomatPath.replace(pos, placeholder.length(), dbName);
    }

    if ((pos = NBVPath.find(placeholder)) != std::string::npos) {
        NBVPath.replace(pos, placeholder.length(), dbName);
    }


    ROS_DEBUG_STREAM("Domain::getDomainFromParams BEGIN");
    ROS_DEBUG_STREAM("Domain::getDomainFromParams RankingMethod : " << RankingMethod);
    ROS_DEBUG_STREAM("Domain::getDomainFromParams Alpha : " << Alpha);
    ROS_DEBUG_STREAM("Domain::getDomainFromParams Beta : " << Beta);
    ROS_DEBUG_STREAM("Domain::getDomainFromParams Gamma : " << Gamma);
    ROS_DEBUG_STREAM("Domain::getDomainFromParams GainValueThreshold : " << GainValueThreshold);
    ROS_DEBUG_STREAM("Domain::getDomainFromParams AutomatPath : " << AutomatPath);
    ROS_DEBUG_STREAM("Domain::getDomainFromParams LogPath : " << LogPath);
    ROS_DEBUG_STREAM("Domain::getDomainFromParams NBVPath : " << NBVPath);


    ISM::TableHelper tabelHelper(Scene_path);
    std::vector<std::string> Scene_patterns = tabelHelper.getRecordedPatternNames();
    for(const std::string &name : Scene_patterns)
    {
        SceneList.push_back(SceneSharedPointer(new Scene(Scene_path, name, RankingMethod, Alpha, Beta, Gamma)));
        ROS_DEBUG_STREAM("Domain::getDomainFromParams Adding scene : " << name);
        ROS_DEBUG_STREAM("Domain::getDomainFromParams with path : " << Scene_path);
    }
    ROS_DEBUG_STREAM("Domain::getDomainFromParams END");
}

void Domain::publishLogs()
{
    //Publish logs in the log file folder.
	std::chrono::time_point<std::chrono::system_clock> start;
	start = std::chrono::system_clock::now();
	std::time_t start_time = std::chrono::system_clock::to_time_t(start);
    std::string filePath = this->LogPath + this->DomainName + ".log";
	std::ofstream file(filePath, std::ios::out | std::ios::app);
	if(file)
	{
		file << std::endl <<this->DomainName << " Log file" << std::endl;
		file << "Timestamp for the Log " << std::ctime(&start_time) << std::endl;
		file << std::endl << "Intermediate Objects for The Domain " << std::endl;
        for (const objectTupleSharedPtr &tuple : IntermediateOjects)
		{
            file << std::get<1>(*tuple) << " : " << std::get<0>(*tuple) << std::endl;
		}
		file.close();
	}
    for (std::vector<SceneSharedPointer>::iterator sceneIterator = SceneList.begin(); sceneIterator != SceneList.end(); ++sceneIterator)
	{
        (*sceneIterator)->publishLogs(filePath);
	}
}

void Domain::publishIntermediateObjectToAutomat()
{
    //Publish the XML data containing intermediate object for the direct search.
    std::string output = "<InterObj>";
    for (const objectTupleSharedPtr &tuple : IntermediateOjects)
    {
        output += "<obj type=\"" + std::get<2>(*tuple) + "\" identifier=\""+ std::get<3>(*tuple) + "\"/>";
    }
    output += "</InterObj>";
    std::ofstream file(AutomatPath, std::ios::out | std::ios::trunc);
    if(file)
    {
        file << output;
        file.close();
    }
}
