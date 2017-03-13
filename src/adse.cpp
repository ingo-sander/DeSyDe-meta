/**
 * Copyright (c) 2013-2016, Katrhin Rosvall  <krosvall@kth.se>
 *  						Nima Khalilzad   <nkhal@kth.se>
 * 							George Ungureanu <ugeorge@kth.se>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
  
 /** ! \file adse.cpp
 \brief The file with the main function.

 Does all the housekeeping for the DSE tool.
 */

#include <vector>

#include "applications/sdfgraph.hpp"
#include "platform/platform.hpp"
#include "system/mapping.hpp"
//#include "cp_model/model.hpp"
#include "cp_model/sdf_pr_online_model.hpp"
#include "presolving/oneProcMappings.hpp"
#include "execution/execution.cpp"
#include "presolving/presolver.cpp"
#include "settings/input_reader.hpp"
#include "cp_model/schedulability.hpp"
#include "validation/validation.hpp"

#include "metaheuristic/swarm.hpp"
#include "metaheuristic/ga_population.hpp"

#include "xml/xmldoc.hpp"
#include "settings/config.hpp"
#include "exceptions/exception.h"

using namespace Gecode;
using namespace Int;


int main(int argc, const char* argv[]) {

  cout << "DeSyDe - Analytical Design Space Exploration Tool\n";
  int exit_status = 0;

  Config cfg;

  try {
    if (cfg.parse(argc, argv))
      return 0;
  } catch (DeSyDe::Exception& ex) {
    cout << ex.toString() << endl;
    return 1;
  }

  try {
	  
	  TaskSet* taskset;
	  Platform* platform;
	  string WCET_path;
	  string desConst_path;
	  for (const auto& path : cfg.settings().inputs_paths) {
       /// Reading taskset
       size_t found_taskset=path.find("taskset");
       if(found_taskset != string::npos){
			XMLdoc xml(path);
			LOG_INFO("Parsing taskset XML files...");
			xml.read(false);
			taskset =  new TaskSet(xml);
			if (taskset->getNumberOfTasks() > 0) {
			  taskset->SetRMPriorities();
			  cout << *taskset;
			} else {
			  cout << "did not import any periodic tasks!" << endl;
			}
	   }
	   /// Reading platform
       size_t found_platform=path.find("platform");
       if(found_platform != string::npos){
			XMLdoc xml(path);
			LOG_INFO("Parsing platform XML files...");
			xml.read(false);
			platform =  new Platform(xml);
			cout << *platform << endl;
	   }
	   /// Storing WCET xml path
       size_t found_wcet=path.find("WCETs");
       if(found_wcet != string::npos){
		   WCET_path = path;
			LOG_INFO("Storing WCET XML file...");
	   }
	   /// Storing design constraints xml path
       size_t found_desConst=path.find("desConst");
       if(found_desConst != string::npos){
		   desConst_path = path;
			LOG_INFO("Storing desConst XML file...");
	   }
     }
	
    
    vector<SDFGraph*> sdfs;
    for (const auto& path : cfg.settings().inputs_paths) {
		 
       if(path.find("/sdfs/") != string::npos){		
           XMLdoc xml(path);
           LOG_INFO("Parsing SDF3 graphs...");
           xml.readXSD("sdf3", "noNamespaceSchemaLocation");
           sdfs.push_back(new SDFGraph(xml));
       }
     }
    /*for(auto &i : sdfXMLs)
     sdfs.push_back(new SDFGraph(i));*/

    
	XMLdoc xml_const(desConst_path);
	xml_const.read(false);
    LOG_INFO("Creating an application object ... ");
    Applications* appset = new Applications(sdfs, taskset, xml_const);
    cout << *appset;

	LOG_INFO("Creating a mapping object ... " );
    XMLdoc xml_wcet(WCET_path);
    xml_wcet.read(false);
    Mapping* map = new Mapping(appset, platform, xml_wcet);
    
    LOG_INFO("Sorting pr tasks based on utilization ... ");
    //map->PrintWCETs();
    map->SortTasksUtilization();
    cout << *taskset;

    
//Testing the design class --------------------------------------------
/*
    shared_ptr<Mapping> map_ptr(new Mapping(appset, platform, xml_wcet));
    shared_ptr<Applications> appset_ptr(new Applications(sdfs, taskset, xml_const));

    vector<int> proc_mappings = {3, 3, 3, 0, 1, 1, 2, 0, 0, 2, 3, 3, 0, 0, 0, 0};
    vector<int> proc_modes{1, 0, 0, 1};
    vector<int> next = {10, 2, 19, 16, 5, 17, 9, 12, 13, 18, 11, 1, 8, 14, 15, 3, 4, 6, 0, 7};
    vector<int> sendNext = {12, 2, 3, 20, 18, 4, 9, 14, 10, 8, 19, 0, 1, 15, 13, 16, 17, 5, 6, 11, 7};
    vector<int> recNext = {1, 20, 3, 17, 18, 19, 12, 9, 11, 14, 16, 0, 7, 15, 13, 10, 2, 4, 5, 8, 6};
    vector<int> tdmaAlloc = {1,1,1,1};
    Design* design = new Design(map_ptr, appset_ptr, proc_mappings, proc_modes, 
                                next, sendNext, recNext, tdmaAlloc);
    vector<int> periods = design->get_periods();
    for(auto p:periods)
        cout << "period=" << p << ", ";
    cout << endl << "energy=" << design->get_energy() << endl
        << *design << endl;    
     
    proc_mappings = {};
    //next = {}; 
    sendNext = {}; 
    recNext = {}; 
    //tdmaAlloc = {}; 
    
    SDFPROnlineModel* cp_model;
    LOG_INFO("Creating a constraint model object ... ");
    cp_model = new SDFPROnlineModel(map, &cfg);
    cp_model->set_design(proc_mappings, proc_modes, tdmaAlloc, next, sendNext, recNext);
    
    LOG_INFO("Creating an execution object ... ");
    Execution<SDFPROnlineModel> cp_execObj(cp_model, cfg);

    LOG_INFO("Running the model object ... ");
    cp_execObj.Execute();
    
    LOG_INFO("End of testing design class ... ");
    
    return exit_status;
*/   
    if(cfg.settings().search == Config::GA)
    {
       
        shared_ptr<Mapping> map_ptr(new Mapping(appset, platform, xml_wcet));
        shared_ptr<Applications> appset_ptr(new Applications(sdfs, taskset, xml_const));

        
        GA_Population p(map_ptr, appset_ptr, cfg);
        p.search();
        return exit_status;    
 
    }
    if(cfg.settings().search == Config::PSO)
    {
       
        shared_ptr<Mapping> map_ptr(new Mapping(appset, platform, xml_wcet));
        shared_ptr<Applications> appset_ptr(new Applications(sdfs, taskset, xml_const));

        
        Swarm s(map_ptr, appset_ptr, cfg);
        s.search();
        return exit_status;    
    }


    SDFPROnlineModel* model;
    //PRESOLVING +++

    if(sdfs.size() > 0)
    {
    LOG_INFO("Creating PRESOLVING constraint model object ... ");
    //OneProcModel* pre_model = new OneProcModel(map, cfg);

        LOG_INFO("Creating PRESOLVING execution object ... ");
        Presolver presolver(cfg);

        LOG_INFO("Running PRESOLVING model object ... ");
        model = (SDFPROnlineModel*)presolver.presolve(map);

        vector<vector<tuple<int,int>>> mappings = presolver.getMappingResults();
        cout << "Presolver found " << mappings.size() << " isolated mappings." << endl;
    }
    else
    {
        LOG_INFO("Creating a constraint model object ... ");
        model = new SDFPROnlineModel(map, &cfg);
    }
    LOG_INFO("Creating an execution object ... ");
    Execution<SDFPROnlineModel> execObj(model, cfg);

    LOG_INFO("Running the model object ... ");
    execObj.Execute();

//    Validation* val = new Validation(map, cfg);
//    val->Validate();

    return exit_status;
  } catch (DeSyDe::Exception& ex) {
    cout << ex.toString() << endl;
    return 1;
  }

}
/*
 * Example in which the critical cycle is detected while mcr is incorrect
    vector<int> proc_mappings = {1, 2, 0, 1, 3, 3, 3, 1, 1, 0, 3, 2, 2, 2, 1, 2};
    vector<int> proc_modes{1,0,1,1};
    vector<int> next = {8, 12, 16, 7, 5, 6, 19, 17, 14, 2, 4, 1, 13, 15, 3, 18, 0, 11, 10, 9};
    vector<int> sendNext = {1,16,14,17,5,6,20,18,9,10,3,4,2,15,13,19,7,0,12,11,8};
    vector<int> recNext = {14,17,13,15,5,20,7,2,4,10,19,0,11,3,16,18,9,6,12,8,1};
    vector<int> tdmaAlloc = {1,1,1,1};
*/
