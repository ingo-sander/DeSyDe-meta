/**
 * Copyright (c) 2013-2016, Nima Khalilzad   <nkhal@kth.se>
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

#include <vector>
#include <algorithm>

#include "mapping.hpp"

#include "../exceptions/runtimeexception.h"

#include <boost/graph/graphviz.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/howard_cycle_ratio.hpp>


using namespace std;

//using namespace boost;
namespace b = boost;

//! alias for actor ID property (check BGL documentation)
//<http://www.boost.org/doc/libs/1_53_0/libs/graph/doc/using_adjacency_list.html#sec:adjacency-list-properties>

enum vertex_actorida_t { vertex_actorida };
namespace boost {
  BOOST_INSTALL_PROPERTY(vertex, actorida);
}

using actor_prop_des = b::property<vertex_actorida_t, int>;
using chan_prop_des  = b::property<b::edge_weight_t, int, b::property<b::edge_weight2_t, int> >;
using boost_msag_des = b::adjacency_list<b::vecS, b::vecS, b::directedS, actor_prop_des, chan_prop_des>;

/**
 * This class is used for storing a particular design (i.e. solution).
 * We use this class to perform performance analysis on complete designs.
 */
class Design {
    class SuccessorNode{
    public:
      int successor_key;
      int min_tok;
      int max_tok;
      int delay;
      int channel; //in case of sending/receiving node
      int recOrder; //in case of receiving node
      int destination; //in case of sending node

      SuccessorNode():successor_key(-1){};

  };
public:
    Design(Mapping*, Applications*, vector<int>, vector<int>, vector<int>,
           vector<int>, vector<int>, vector<int>, vector<int>, vector<int>);    
    ~Design(){};
    vector<int> get_periods();
    int get_energy();
private:
    Mapping* mapping; /**< reference to the mapping class. */
    Applications* applications; /**< reference to the applications class. */
    const size_t no_entities; /**< total number of actors and tasks. */
    size_t no_actors; /**< total number of actors and tasks. */
    size_t no_channels; /**< total number of channels. */
    size_t no_processors; /**< total number of processors. */
    vector<int> proc_mappings; /**< current mappings of entities (actors and tasks) to processors. */
    vector<int> proc_modes; /**< current processors modes. */
    vector<int> next; /**< current communication orders */
    vector<int> sendingTime; /**< communication latencies (same proc=0). */
    vector<int> wcet; /**< Worst-Case Execution Times (WCET) of actors. */
    vector<int> memCons; /**< memory consumption on each proc. */
    vector<int> sendingLatency; /**< Worst-Case Blocking Times. */
    vector<int> receivingTime; /**< WC receiving times (0 on TDMA). */
    vector<int> sendingNext; /**< a schedule for sent messages on interconnect. */
    vector<int> receivingNext; /**< a schedule for sent messages on interconnect. */
    vector<int> tdmaAlloc; /**< mappings of TDMA slots to processors. */
    vector<int> sendbufferSz; /**< send buffer sizes (same proc=0). */
    vector<int> recbufferSz; /**< receive buffer sizes (same proc=0). */
    vector<int> appIndex; /**< appIndex[i] is index of last actor of application i.*/
    unordered_map<int,vector<SuccessorNode>> msaGraph;/**< for construction of the mapping and scheduling aware graph. */
    boost_msag_des b_msag; /** MSAG representation for boost. */
    vector<boost_msag_des*> b_msags; /**< MSAG representation for boost. */
    vector<int> channelMapping;/**< for mapping from msag send/rec actors to appG-channels.*/
    vector<int> receivingActors;/**< receivingActors: for storing/finding the first receiving actor for each dst. */
    vector<int> periods;
    int energy;
    size_t n_msagActors;
    /**
     * Constructs Mapping and Scheduling Aware Graph (MSAG).
     * The code is adopted from the ThroughputMCR class
     */ 
    void constructMSAG();
    void constructMSAG(vector<int> &msagMap);
    int getBlockActor(int ch_id) const;
    int getSendActor(int ch_id) const;
    int getRecActor(int ch_id) const;
    int getApp(int msagActor_id) const;   
    
    /**
     * Initiates the following internal vectors to be used by throughput analysis:
     * (i) sendingTime, (ii) sendingLatency, (iii) receivingTime (iv) wcet (v) memCons
     */ 
    void init_vectors();
    void calc_periods();
    void calc_energy();
    void print_vector(vector<int>);
    void printThroughputGraphAsDot(const string &dir) const;
};
