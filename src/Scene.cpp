/**

Copyright (c) 2016, Borella Jocelyn, Meißner Pascal
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "Scene.h"

Scene::Scene()
{
	SceneObjectsCount = 0;
	DbName = "void";
	SceneName = "void";
	RankingMethod = 0;
	Alpha = 0;
	Beta = 0;
	Gamma = 0;
}

Scene::Scene(std::string dbName, std::string sceneName, int rankingMethodEntry, double alphaEntry, double betaEntry, double gammaEntry)
:DbName(dbName),SceneName(sceneName),RankingMethod(rankingMethodEntry),Alpha(alphaEntry),Beta(betaEntry),Gamma(gammaEntry)
{
	getNumberSet();
}

Scene::~Scene()
{
}

void Scene::getObjectFromDb()
{
	sqlite3 *database;
    ROS_DEBUG_STREAM("Scene::getObjectFromDb BEGIN");
	sqlite3_open(DbName.c_str(), &database);
	sqlite3_stmt *statement;
    std::stringstream sqlReq;
    sqlReq << "SELECT type,px,py,pz,setId,observedId FROM recorded_objects "
           << "INNER JOIN recorded_patterns on recorded_sets.patternId = recorded_patterns.id "
           << "INNER JOIN recorded_sets on recorded_objects.setId = recorded_sets.id "
           << "WHERE recorded_patterns.name = '"
           << SceneName
           << "';";
    const std::string& tmp = sqlReq.str();
    const char* SQLREQ = tmp.c_str();
    int returnPrepare = sqlite3_prepare_v2(database, SQLREQ, -1,&statement, 0);
    ROS_DEBUG_STREAM("Scene::getObjectFromDb prepare database returned code : " << returnPrepare);
    if(returnPrepare == SQLITE_OK)
	{
		int cols = sqlite3_column_count(statement);
		int result = 0;
		while(true)
		{
			result = sqlite3_step(statement);
            if(cols != 6)
            {
                ROS_ERROR_STREAM("Scene::getObjectFromDb something went terribly wrong when recovering objects for scene  : " << SceneName);
                continue;
            }
            if(result == SQLITE_ROW)
			{
                ObjectPointSharedPointer objectPoint = ObjectPointSharedPointer(new ObjectPoint);
                std::string name = (char*)sqlite3_column_text(statement, 0);
                std::string X = (char*)sqlite3_column_text(statement, 1);
                std::string Y = (char*)sqlite3_column_text(statement, 2);
                std::string Z = (char*)sqlite3_column_text(statement, 3);
                std::string setId = (char*)sqlite3_column_text(statement, 4);
                std::string observedId = (char*)sqlite3_column_text(statement, 5);
                std::string objectMetaName = name + observedId;
                if(this->ObjectMap.find(objectMetaName)== this->ObjectMap.end())
                {
                    this->ObjectMap[objectMetaName] = ObjectSharedPointer(new Object(name, observedId, RankingMethod, Alpha, Beta, Gamma));
                    ROS_DEBUG_STREAM("Scene::getObjectFromDb found object " << name
                                     << " with observed Id " << observedId
                                     << " in scene " << SceneName);
                }
                objectPoint->setX(boost::lexical_cast<double>(X));
                objectPoint->setY(boost::lexical_cast<double>(Y));
                objectPoint->setZ(boost::lexical_cast<double>(Z));
                objectPoint->setSetId(boost::lexical_cast<int>(setId));
                this->ObjectMap[objectMetaName]->AddPoint(objectPoint);
			}
			else
			{
				break;
			}
		}

		sqlite3_finalize(statement);
	}
	sqlite3_close(database);
    ROS_ERROR_COND(ObjectMap.size() == 0, "Scene::getObjectFromDb Could not get any Object. Please check the database location and the pattern name");
    ROS_DEBUG_STREAM("Scene::getObjectFromDb END");
}


void Scene::getNumberSet()
{
    ROS_DEBUG_STREAM("Scene::getNumberSet BEGIN");
    //Get the number of sets recorded for the scene.
    std::stringstream sqlReq;
    sqlReq << "SELECT COUNT(DISTINCT recorded_sets.id) FROM recorded_sets "
           << "INNER JOIN recorded_patterns on recorded_sets.patternId = recorded_patterns.id "
           << "WHERE recorded_patterns.name = '"
           << SceneName
           << "';";
    const std::string& tmp = sqlReq.str();
    const char* SQLREQ = tmp.c_str();
	sqlite3 *database;
	sqlite3_open(DbName.c_str(), &database);
	sqlite3_stmt *statement;
    if(sqlite3_prepare_v2(database, SQLREQ, -1, &statement, 0) == SQLITE_OK)
	{
		int cols = sqlite3_column_count(statement);
		int result = sqlite3_step(statement);
		if(result == SQLITE_ROW)
		{
			for(int col = 0; col < cols; col++)
			{
				std::string s = (char*)sqlite3_column_text(statement, col);
				SceneObjectsCount = boost::lexical_cast<int>(s);
			}
		}
		sqlite3_finalize(statement);
	}
	sqlite3_close(database);
    ROS_DEBUG_STREAM_COND(SceneObjectsCount > 0,"Scene::getNumberSet Got "<< SceneObjectsCount <<" sets for scene " << SceneName);
    ROS_ERROR_STREAM_COND(SceneObjectsCount == 0, "Scene::getNumberSet Could not get the amount of Sets in the scene"
                          << SceneName);
    ROS_DEBUG_STREAM("Scene::getNumberSet END");
}

void Scene::calcAveragePositionForEachObject()
{
    for (std::unordered_map<std::string,ObjectSharedPointer>::iterator objectIterator=ObjectMap.begin(); objectIterator!=ObjectMap.end(); ++objectIterator)
	{
        objectIterator->second->CalcAveragePos();
        objectIterator->second->CalcPosVar();
	}
}

void Scene::getObjectAverageDistance()
{
	bool safetyTest = false;

    for (std::unordered_map<std::string,ObjectSharedPointer>::iterator objectIterator=ObjectMap.begin(); objectIterator!=ObjectMap.end(); ++objectIterator) //Parcours liste Obj
	{
        for(int point=0; point<objectIterator->second->getObjetsCount(); point++) //All the point for one object
		{
			double distance=0;
			int counter=0;
            for (std::unordered_map<std::string,ObjectSharedPointer>::iterator objectIterator2=ObjectMap.begin(); objectIterator2!=ObjectMap.end(); ++objectIterator2)
			{
                if(objectIterator2->first != objectIterator->first)  //If objects are different
				{
					safetyTest = true;
                    for(int k=0; k<objectIterator2->second->getObjetsCount(); k++) //Other points for other objects
					{
                        if((objectIterator2->second->getPoint(k))->getSetId() == (objectIterator->second->getPoint(point))->getSetId())
						{
                            //Add the distance to the variable
                            distance += (objectIterator->second->getPoint(point))->calcDistance(*(objectIterator2->second->getPoint(k)));
							counter++;
						}
					}
				}
			}
            if(safetyTest==true){objectIterator->second->AddDistance(distance/counter);safetyTest=false;}
		}
        objectIterator->second->AverageDistance();
	}
}

void Scene::calcPresenceInSceneForEachObject()
{
    ROS_DEBUG_STREAM("Scene::calcPresenceInSceneForEachObject BEGIN");
    for (std::unordered_map<std::string,ObjectSharedPointer>::iterator objectIterator=ObjectMap.begin(); objectIterator!=ObjectMap.end(); ++objectIterator)
	{
        double presenceInScene = (boost::lexical_cast<double>(objectIterator->second->getObjetsCount())/boost::lexical_cast<double>(this->SceneObjectsCount));
        ROS_DEBUG_STREAM("Scene::calcPresenceInSceneForEachObject Object occurence in scene : "
                         << objectIterator->second->getObjetsCount()
                         << ", amount of sets recorded in the scene : " << this->SceneObjectsCount
                         << ", presence in scene : " << presenceInScene
                         << ", for object : " << objectIterator->second->getObjectName()
                         << " with observed Id " << objectIterator->second->getObservedId());
        objectIterator->second->setPresenceInScene(presenceInScene);
	}
    ROS_DEBUG_STREAM("Scene::calcPresenceInSceneForEachObject END");
}

void Scene::displayStats()
{
    ROS_DEBUG_STREAM("Scene::DisplayStats Stats For Scene " << SceneName);
    for (std::unordered_map<std::string,ObjectSharedPointer>::iterator objectIterator=ObjectMap.begin(); objectIterator!=ObjectMap.end(); ++objectIterator)
	{
        objectIterator->second->DisplayStats();
	}
}


void Scene::normalize()
{
    std::function<bool(const std::pair<std::string,ObjectSharedPointer>,const std::pair<std::string,ObjectSharedPointer>)> compare_distance_min =
            [](const std::pair<std::string,ObjectSharedPointer>& i, const std::pair<std::string,ObjectSharedPointer>& j)
    { return i.second->getAverageDistanceToOtherObjects() < j.second->getAverageDistanceToOtherObjects();};

    std::function<bool(const std::pair<std::string,ObjectSharedPointer>,const std::pair<std::string,ObjectSharedPointer>)> compare_position_min =
            [](const std::pair<std::string,ObjectSharedPointer>& i, const std::pair<std::string,ObjectSharedPointer>& j)
    { return i.second->getPosVar() < j.second->getPosVar();};

    //Get the min values for all parameters
    double minValueDist = (std::min_element(ObjectMap.begin(),ObjectMap.end(),compare_distance_min))->second->getAverageDistanceToOtherObjects();
    double minValuePos = (std::min_element(ObjectMap.begin(),ObjectMap.end(),compare_position_min))->second->getPosVar();

    ROS_DEBUG_STREAM("Scene::normalize minimum average distance value found " << minValueDist);
    ROS_DEBUG_STREAM("Scene::normalize minimum average position value found " << minValuePos);

    for (std::unordered_map<std::string,ObjectSharedPointer>::iterator objectIterator=ObjectMap.begin(); objectIterator!=ObjectMap.end(); ++objectIterator)
	{
        double bufferMin = (objectIterator->second->getAverageDistanceToOtherObjects()-minValueDist);
        ROS_DEBUG_STREAM("Scene::normalize difference to min value for object "
                         << objectIterator->second->getObjectName() << " : " << bufferMin);
        (objectIterator->second)->setNormalizedAverageDistanceToOtherObjects(bufferMin);
        (objectIterator->second)->setNormalizedPosVar((objectIterator->second->getPosVar()-minValuePos));
	}

    std::function<bool(const std::pair<std::string,ObjectSharedPointer>,const std::pair<std::string,ObjectSharedPointer>)> compare_distance_max =
            [](const std::pair<std::string,ObjectSharedPointer>& i, const std::pair<std::string,ObjectSharedPointer>& j)
    { return i.second->getNormalizedAverageDistanceToOtherObjects() < j.second->getNormalizedAverageDistanceToOtherObjects();};

    std::function<bool(const std::pair<std::string,ObjectSharedPointer>,const std::pair<std::string,ObjectSharedPointer>)> compare_position_max =
            [](const std::pair<std::string,ObjectSharedPointer>& i, const std::pair<std::string,ObjectSharedPointer>& j)
    { return i.second->getNormalizedPosVar() < j.second->getNormalizedPosVar();};

    //get the max values for all paramters
    double maxValueDist = (std::max_element(ObjectMap.begin(),ObjectMap.end(),compare_distance_max))->second->getNormalizedAverageDistanceToOtherObjects();
    double maxValuePos = (std::max_element(ObjectMap.begin(),ObjectMap.end(),compare_position_max))->second->getNormalizedPosVar();

    ROS_DEBUG_STREAM("Scene::normalize max average distance value found " << maxValueDist);
    ROS_DEBUG_STREAM("Scene::normalize max average position value found " << maxValuePos);

	double valueDistCoef = 1/maxValueDist;
	double posVarCoef = 1/maxValuePos;
    //Normalize each value.
    for (std::unordered_map<std::string,ObjectSharedPointer>::iterator objectIterator=ObjectMap.begin(); objectIterator!=ObjectMap.end(); ++objectIterator)
		{
            objectIterator->second->setNormalizedAverageDistanceToOtherObjects(objectIterator->second->getNormalizedAverageDistanceToOtherObjects()*valueDistCoef+0.1);
            objectIterator->second->setNormalizedPosVar(objectIterator->second->getNormalizedPosVar()*posVarCoef+0.1);
        }

}

void Scene::rank()
{
    for (std::unordered_map<std::string,ObjectSharedPointer>::iterator objectIterator=ObjectMap.begin(); objectIterator!=ObjectMap.end(); ++objectIterator)
		{
            objectIterator->second->rank();
		}
}

void Scene::publishLogs(std::string filePath)
{
	std::ofstream file(filePath, std::ios::out | std::ios::app);
	if(file)
	{
		file << std::endl << std::endl <<"Scene " << this->SceneName << std::endl;
		file.close();
	}
    for (std::unordered_map<std::string,ObjectSharedPointer>::iterator objectIterator=ObjectMap.begin(); objectIterator!=ObjectMap.end(); ++objectIterator)
	{
        objectIterator->second->publishLogs(filePath);
	}
}


