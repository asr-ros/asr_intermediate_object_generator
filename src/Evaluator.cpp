/**

Copyright (c) 2016, Borella Jocelyn, Meißner Pascal
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "Evaluator.h"
#include <iostream>

bool compare_evaluator_tuple(const objectTupleSharedPtr &first, const objectTupleSharedPtr &second)
{
  return (std::get<0>(*first) < std::get<0>(*second));
}

Evaluator::Evaluator()
{
	GainValueThreshold = 0;
    NBVPath = "";
}

Evaluator::Evaluator(std::list<evalulatorTupleSharedPtr> objectListEntry, std::list<std::string> sceneListEntry,
                     double GainValueThresholdEntry, std::string NBVPath, std::string dbPath):
    GainValueThreshold(GainValueThresholdEntry),NBVPath(NBVPath),DbPath(dbPath)
    {
	for (const std::string &it : sceneListEntry)
	{
        SceneMap[it].push_back("root-obj");
	}

    std::list<evalulatorTupleSharedPtr> bufferObjectList;

    //Fusion all evaluator tuples into object tuple, so that all object are only represented once.
	while(objectListEntry.size()>0)
	{
		double fValue = 0;
        std::string objectMetaName =std::get<2>(*(*objectListEntry.begin()));
        std::string objectName =std::get<3>(*(*objectListEntry.begin()));
        std::string objectObservedId =std::get<4>(*(*objectListEntry.begin()));
        fValue += std::get<0>(*(*objectListEntry.begin()));
        //Add the object in the scene map
        SceneMap[std::get<1>(*(*objectListEntry.begin()))].push_back(std::get<2>(*(*objectListEntry.begin())));
		objectListEntry.pop_front();
        bufferObjectList = objectListEntry;
        for (const evalulatorTupleSharedPtr &tuple : objectListEntry)
		{
            //Search for the same object in other scene
            if(std::get<2>(*tuple) == objectMetaName)
			{
                SceneMap[std::get<1>(*tuple)].push_back(std::get<2>(*tuple));
                fValue += std::get<0>(*tuple);
                bufferObjectList.remove(tuple);
			}
		}
        //Add the tuple in the tuple list
        ProcessedObjectList.push_back(objectTupleSharedPtr(new objectTuple(fValue,objectMetaName,objectName,objectObservedId)));
        objectListEntry = bufferObjectList;
	}
    ObjectListSize = ProcessedObjectList.size();

    PublishIntermediateObjectForWorldModel();
    PublishIntermediateObjectInDomainTable();
}

Evaluator::~Evaluator()
{
}


double Evaluator::CalcAverageF(std::list<objectTupleSharedPtr> list)
{
    double ret = 0;
    for (const objectTupleSharedPtr &it : list)
	{
        ret += std::get<0>(*it);
	}
    return ret/list.size();
}


bool Evaluator::Validation(std::list<objectTupleSharedPtr> listToValidate)
{
	bool test = false;
    //Validity of criteria : at least one object per scene
    for (std::unordered_map<std::string,std::list<std::string>>::iterator it=SceneMap.begin(); it!=SceneMap.end(); ++it)
	{
        for (const objectTupleSharedPtr &it2 : listToValidate)
		{
			for (const std::string &it3 : it->second)
			{
                if(it3 == std::get<1>(*it2))
				{
					test = true;
					break;
				}
			}
			if(test) break;
		}
		if(!test) return false;
	}
	return true;
}

bool Evaluator::Evaluate(std::list<objectTupleSharedPtr> oldList, std::list<objectTupleSharedPtr> newList)
{
    if ((CalcAverageF(newList) - CalcAverageF(oldList))/CalcAverageF(newList) < GainValueThreshold)
	{
		return true;
	}
	else
	{
		return false;
	}

}

std::list<objectTupleSharedPtr> Evaluator::getIntermediateObjects()
{
    ROS_DEBUG_STREAM("Evaluator::getIntermediateObjects BEGIN");
    objectTupleSharedPtr bufferTuple;
    ProcessedObjectList.sort(compare_evaluator_tuple);
    ROS_DEBUG_STREAM("Evaluator::getIntermediateObjects Weight value are now sorted");
    std::list<objectTupleSharedPtr> bufferTupleList;
    ROS_DEBUG_STREAM("Evaluator::getIntermediateObjects begin from filtering iteration process");
    //remove object ad long as the gain threshold is valid and the validation true is.
    for (int i=0; i<ObjectListSize; i++)
	{
        bufferTupleList = ProcessedObjectList;
        bufferTuple = *(ProcessedObjectList.begin());
        ProcessedObjectList.pop_front();

        if (Validation(ProcessedObjectList) == false)
		{
            ProcessedObjectList.push_back(bufferTuple);
			continue;
		}

        if(Evaluate(bufferTupleList, ProcessedObjectList) == false)
		{
			break;
		}
	}
    ROS_DEBUG_STREAM("Evaluator::getIntermediateObjects end from filtering iteration process");
    ROS_DEBUG_STREAM("Evaluator::getIntermediateObjects gotten list from intermediate object");
    for (const objectTupleSharedPtr &tuple : ProcessedObjectList)
		{
            ROS_DEBUG_STREAM("Evaluator::getIntermediateObjects Object " <<std::get<2>(*tuple)
                             << " with observed Id " << std::get<3>(*tuple)
                             << " and value " << std::get<0>(*tuple));
		}
    ROS_DEBUG_STREAM("Evaluator::getIntermediateObjects END");
    return ProcessedObjectList;
}

void Evaluator::PublishIntermediateObjectForWorldModel()
{
    std::string output = "<InterObj>";
    for (const objectTupleSharedPtr &tuple : ProcessedObjectList)
    {
        ROS_DEBUG_STREAM("Evaluator::PublishIntermediateObjectForWorldModel Object " << std::get<2>(*tuple) << " with observed Id " << std::get<3>(*tuple)
                         << " got intermediate object weight " << std::get<0>(*tuple));
        output += "<obj name=\""+std::get<2>(*tuple)+"\" observedId=\""+std::get<3>(*tuple)+"\">" + std::to_string(std::get<0>(*tuple)) + "</obj>";
    }
    output += "</InterObj>";

    std::ofstream file(NBVPath, std::ios::out | std::ios::trunc);
    if(file)
    {
        file << output;
        file.close();
    }
}

static int callback(void *NotUsed, int argc, char **argv, char **azColName){
    //Callback for sqlite requests.
    int i;
    for(i=0; i<argc; i++){
       ROS_DEBUG("Evaluator::PublishIntermediateObjectInDomainTable %s = %s\n", azColName[i], argv[i] ? argv[i] : "NULL");
    }
    return 0;
}

void Evaluator::PublishIntermediateObjectInDomainTable()
{
    std::stringstream sqlReq;
    char *zErrMsg = 0;
    sqlReq << "CREATE TABLE model_weight(id INT PRIMARY KEY NOT NULL,type VARCHAR(100),observedId "
           << "VARCHAR(100), weight VARCHAR(100));";
    const std::string& sqlReqString = sqlReq.str();
    const char* SQLREQ = sqlReqString.c_str();
    sqlite3 *database;
    sqlite3_open(DbPath.c_str(), &database);
    int sqliteReturn = sqlite3_exec(database, SQLREQ, callback, 0, &zErrMsg);
    if( sqliteReturn != SQLITE_OK )
    {
        ROS_WARN_STREAM("Evaluator::PublishIntermediateObjectInDomainTable SQL error : " << zErrMsg);
        sqlite3_free(zErrMsg);
    }
    else
    {
       ROS_DEBUG_STREAM("Evaluator::PublishIntermediateObjectInDomainTable Model weight table created.");
    }

    std::stringstream clearReq;
    clearReq << "DELETE FROM model_weight";

    const std::string& clearReqString = clearReq.str();
    const char* SQLCLEAR = clearReqString.c_str();
    sqliteReturn = sqlite3_exec(database, SQLCLEAR, callback, 0, &zErrMsg);
    if( sqliteReturn != SQLITE_OK )
    {
        ROS_WARN_STREAM("Evaluator::PublishIntermediateObjectInDomainTable SQL error : " << zErrMsg);
        sqlite3_free(zErrMsg);
    }
    else
    {
       ROS_DEBUG_STREAM("Evaluator::PublishIntermediateObjectInDomainTable table cleared.");
    }

    std::stringstream insertReq;
    unsigned int counter = 0;
    for (const objectTupleSharedPtr &tuple : ProcessedObjectList)
    {
        insertReq << "INSERT INTO model_weight (id,type,observedId,weight)"
                  << "VALUES (" << counter <<", '"<< std::get<2>(*tuple) <<"', '"<< std::get<3>(*tuple) <<"', '"
                  << std::get<0>(*tuple) <<"');";
        counter++;
    }
    const std::string& insertReqString = insertReq.str();
    const char* SQLINSERT = insertReqString.c_str();
    sqliteReturn = sqlite3_exec(database, SQLINSERT, callback, 0, &zErrMsg);
    if( sqliteReturn != SQLITE_OK )
    {
        ROS_WARN_STREAM("Evaluator::PublishIntermediateObjectInDomainTable SQL error : " << zErrMsg);
        sqlite3_free(zErrMsg);
    }
    else
    {
       ROS_DEBUG_STREAM("Evaluator::PublishIntermediateObjectInDomainTable Weights inserted.");
    }
    sqlite3_close(database);
}
