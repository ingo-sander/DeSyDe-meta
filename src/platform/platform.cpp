#include "platform.hpp"

using namespace std;

Platform::Platform(size_t p_nodes, int p_cycle, size_t p_memSize, int p_buffer, enum InterconnectType p_type, int p_dps, int p_tdma, int p_roundLength){
  for (size_t i=0; i<p_nodes; i++){
    compNodes.push_back(new PE("pe"+i,"gp",vector<double>(1,p_cycle),
                               vector<int>(1,p_memSize), vector<int>(1,10),
                               vector<int>(1,10),vector<int>(1,1),p_buffer));
  }

  interconnect = Interconnect(p_type, p_dps, p_tdma, p_roundLength, p_nodes, 1);
}

Platform::Platform(std::vector<PE*> p_nodes, enum InterconnectType p_type, int p_dps, int p_tdma, int p_roundLength){
  compNodes = p_nodes;
  interconnect = Interconnect(p_type, p_dps, p_tdma, p_roundLength, (int)p_nodes.size(), 1);
}

Platform::Platform(std::vector<PE*> p_nodes, enum InterconnectType p_type, int p_dps, int p_tdma, int p_roundLength, int p_col, int p_row){
  compNodes = p_nodes;
  if(p_type == NOC){
    interconnect = Interconnect(p_type, p_dps, p_tdma, p_roundLength, p_col, p_row);
  }else{
    interconnect = Interconnect(p_type, p_dps, p_tdma, p_roundLength, (int)p_nodes.size(), 1);
  }
}

Platform::Platform(XMLdoc& xml)
{
    const char* my_xpathString = "///@name";
	LOG_DEBUG("running xpathString  " + tools::toString(my_xpathString) + " on platform xml ...");
	for (const auto& name : xml.xpathStrings(my_xpathString)){
		LOG_DEBUG("Platform class is parsing the following platform: " + name + "...");
	}
	load_xml(xml);
    ///Assigning default interconnect
    interconnect = Interconnect(TDMA_BUS, 32, (int) compNodes.size(), 1, (int)compNodes.size(), 1);    
}
void Platform::load_xml(XMLdoc& xml)
{
	const char* my_xpathString = "///platform/processor";
	LOG_DEBUG("running xpathString  " + tools::toString(my_xpathString) + " platform xml ...");
	auto xml_procs = xml.xpathNodes(my_xpathString);
	int proc_id = 0;
	for (const auto& proc : xml_procs)
	{
		string proc_model = xml.getProp(proc, "model");
		string proc_number = xml.getProp(proc, "number");
		LOG_DEBUG("Reading processor model: " + proc_model + "...");		
		for(int i=0; i<atoi(proc_number.c_str()); i++)
		{
			PE* pe = new PE(proc_model, i);
            /// Parsing the modes
            string query = "///platform/processor[@model=\'" + proc_model + "\']/mode";
            auto   modes = xml.xpathNodes(query.c_str());
            for (auto mode : modes) {
              string mode_name = xml.getProp(mode, "name");
              string mode_cycle = xml.getProp(mode, "cycle");
              string mode_mem = xml.getProp(mode, "mem");
              string mode_power = xml.getProp(mode, "power");
              string mode_area = xml.getProp(mode, "area");
              string mode_monetary = xml.getProp(mode, "monetary");
              pe->AddMode(atof(mode_cycle.c_str()), atoi(mode_mem.c_str()),
                        atoi(mode_power.c_str()), atoi(mode_area.c_str()),
                        atoi(mode_monetary.c_str()));
              LOG_DEBUG("Reading processor mode: " + mode_name + "...");		
            }
            
			proc_id++;
			compNodes.push_back(pe);
		}
	}	
}
Platform::~Platform(){
  //compNodes (they were potentially created with 'new'')
  for (size_t i=0; i<compNodes.size(); i++){
    delete compNodes[i];
  }
}

size_t Platform::nodes() const {
  return compNodes.size();
}

InterconnectType Platform::getInterconnectType() const{
  return interconnect.type;
}

int Platform::tdmaSlots() const{
  return interconnect.tdmaSlots;
}

int Platform::dataPerSlot() const{
  return interconnect.dataPerSlot;
}

int Platform::dataPerRound() const{
  return interconnect.dataPerRound;
}

double Platform::speedUp(int node, int mode) const {
  return compNodes[node]->cycle_length[mode];
}

//maximum communication time (=blocking+sending) on the TDM bus for a token of size tokSize
//for different TDM slot allocations (index of vector = number of slots
//for use with the element constraint in the model
const vector<int> Platform::maxCommTimes(int tokSize) const{
  double slotLength = (double)interconnect.roundLength/interconnect.tdmaSlots;
  double slotsNeeded = ((double)tokSize/interconnect.dataPerSlot);
  double activeSendingTime = slotsNeeded * slotLength;

  std::vector<int> sendingTimes;
  sendingTimes.push_back(-1); //for tdma_alloc=0, sendingTime = -1 (used in CP model)
  for (auto i=1; i<=interconnect.tdmaSlots; i++){ //max sendingTime depends on allocated TDM slots
    double initBlock = interconnect.roundLength - (i-1)*slotLength;
    int roundsNeeded = ceil(slotsNeeded/i);
    double slotDelay = (roundsNeeded-1) * (interconnect.roundLength - (i*slotLength));
    //cout << "tdmAlloc = " << i << " , commTime = " << initBlock+activeSendingTime+slotDelay << endl;

    sendingTimes.push_back(ceil(initBlock+activeSendingTime+slotDelay));
  }
  return sendingTimes;
}

//maximum blocking time on the TDM bus for a token
//for different TDM slot allocations (index of vector = number of slots
//for use with the element constraint in the model
const vector<int> Platform::maxBlockingTimes() const{
  double slotLength = (double)interconnect.roundLength/interconnect.tdmaSlots;

  //cout << "TMDA slot length = " << slotLength << endl;

  std::vector<int> blockingTimes;
  blockingTimes.push_back(0); //for tdma_alloc=0, blockingTime = 0 (used in CP model)
  for (auto i=1; i<=interconnect.tdmaSlots; i++){ //max blocking depends on allocated TDM slots
    double initBlock = interconnect.roundLength - (i-1)*slotLength;
    blockingTimes.push_back(ceil(initBlock));
  }
  return blockingTimes;
}

//maximum transfer time on the TDM bus for a token of size tokSize
//for different TDM slot allocations (index of vector = number of slots
//for use with the element constraint in the model
const vector<int> Platform::maxTransferTimes(int tokSize) const{
  double slotLength = (double)interconnect.roundLength/interconnect.tdmaSlots;
  double slotsNeeded = ((double)tokSize/interconnect.dataPerSlot);
  double activeSendingTime = slotsNeeded * slotLength;

//  cout << "token size = " << tokSize << endl;
//  cout << "TMDA slot length = " << slotLength << endl;
//  cout << "TMDA slots needed = " << slotsNeeded << endl;
//  cout << "active transfer time = " << activeSendingTime << endl;

  std::vector<int> transferTimes;
  transferTimes.push_back(0); //for tdma_alloc=0, transerTime = -1 (used in CP model)
  for (auto i=1; i<=interconnect.tdmaSlots; i++){ //max sendingTime depends on allocated TDM slots
    int roundsNeeded = ceil(slotsNeeded/i);
    double slotDelay = (roundsNeeded-1) * (interconnect.roundLength - (i*slotLength));

    transferTimes.push_back(ceil(activeSendingTime+slotDelay));
  }
  return transferTimes;
}


size_t Platform::memorySize(int node, int mode) const{
  return compNodes[node]->memorySize[mode];
}

size_t Platform::getModes(int node) const{
  return compNodes[node]->n_types;
}

size_t Platform::getMaxModes() const
{
  size_t max_mode = 0;
  for (size_t k = 0; k < nodes(); k++)
    {
      if(getModes(k) > max_mode)
        max_mode = getModes(k);
    }
  return max_mode;
}

vector<int> Platform::getMemorySize(int node) const{
  return compNodes[node]->memorySize;
}

vector<int> Platform::getPowerCons(int node) const{
  return compNodes[node]->powerCons;
}

vector<int> Platform::getAreaCost(int node) const{
  return compNodes[node]->areaCost;
}

vector<int> Platform::getMonetaryCost(int node) const{
  return compNodes[node]->monetaryCost;
}

int Platform::bufferSize(int node) const{
  return compNodes[node]->NI_bufferSize;
}

bool Platform::isFixed() const{
  for (size_t j = 0; j < nodes(); j++){
    if(compNodes[j]->n_types > 1) return false;
  }

  return true;
}

// True if the nodes are homogeneous
bool Platform::homogeneousNodes(int node0, int node1) const{
  if(compNodes[node0]->n_types > 1 || compNodes[node1]->n_types > 1 ||
     compNodes[node0]->type != compNodes[node1]->type ||
     compNodes[node0]->cycle_length != compNodes[node1]->cycle_length ||
     compNodes[node0]->memorySize[0] != compNodes[node1]->memorySize[0]) {
    return false;
  }
  return true;
}


bool Platform::homogeneousModeNodes(int node0, int node1) const{
  if(compNodes[node0]->n_types != compNodes[node1]->n_types){
      return false;
  }

  for (auto m=0; m<compNodes[node0]->n_types; m++){
    if(compNodes[node0]->cycle_length[m] != compNodes[node1]->cycle_length[m] ||
       compNodes[node0]->memorySize[m] != compNodes[node1]->memorySize[m] ||
       compNodes[node0]->powerCons[m] != compNodes[node1]->powerCons[m] ||
       compNodes[node0]->areaCost[m] != compNodes[node1]->areaCost[m] ||
       compNodes[node0]->monetaryCost[m] != compNodes[node1]->monetaryCost[m]){
        return false;
    }
  }
  return true;
}

// True if the platform is homogeneous
bool Platform::homogeneous() const{
  for (size_t j = 1; j < nodes(); j++) {
    if(!homogeneousNodes(j-1, j)) return false;
  }
  return true;
}

// True if all processors only have one mode
bool Platform::allProcsFixed() const{
  for (size_t j = 0; j < nodes(); j++) {
    if(compNodes[j]->type == "flexProc" ) return false;
  }
  return true;
}

string Platform::getProcModel(size_t id)
{
  if(id > compNodes.size())
      THROW_EXCEPTION(InvalidArgumentException,"processor id out of bound\n");  
  return compNodes[id]->model;
}

std::ostream& operator<< (std::ostream &out, const Platform &p)
{
  for (size_t i=0;i<p.compNodes.size();i++)
    {
      out << "PE[" << i << "]: " << *p.compNodes[i] << endl;
    }

  return out;

}
