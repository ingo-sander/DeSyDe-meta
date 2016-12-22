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
#include <random>
#include <iterator>
#include <iostream>
#include <functional>

#include "../exceptions/runtimeexception.h"
#include "../system/design.hpp"
using namespace std;

class Schedule{
public:
    Schedule(vector<int>, int);/** creates a random schedule out of input elements and dummy node. */
    friend std::ostream& operator<< (std::ostream &out, const Schedule &sched);
    void set_rank(int index, int value);
    void set_rank(vector<int> _rank);
    vector<int> get_rank();
    vector<int> get_next();
    vector<int> rank_diff(vector<int>);/** element-wise difference of rank and input vector. */
    void rank_add(vector<float>);/** addes the rank with the input (speed). */
    static int random_round(float);/** randomly slecets ceil or floor. */
    int get_next(int);
    void switch_ranks(int, int);    
    vector<int> get_elements();
    int get_element_by_rank(int) const;
    int get_rank_by_id(int) const;
    int get_rank_by_element(int) const;
    void set_rank_by_element(int, int);
    Schedule& operator=(const Schedule& s)
    {
        rank = s.rank;
        dummy = s.dummy;
        elements= s.elements;
        return *this;
    }
    ~Schedule()
    {
        rank.clear();
        elements.clear();
    }
private:
    vector<int> elements;/** set of the elements (actor id or channel id) in this schedule.*/
    int dummy;/** id of dummy actor or dummy channel. */
    /** 
     * stores the position of the items in the sequence. 
     * rank[i] is the position of elements[i]
     */
    vector<int> rank;
    /**
     * Repairs the schedules which violate the distinct constraint.
     */ 
    void repair_dist();    
};
struct Position{
public:    
   ~Position()
    {
        proc_sched.clear();
        send_sched.clear();
        rec_sched.clear();            
    }
    vector<Schedule> proc_sched;
    vector<Schedule> send_sched;
    vector<Schedule> rec_sched;
    vector<int> proc_mappings;
    vector<int> proc_modes;        
    vector<int> tdmaAlloc;     
    vector<int> get_actors_by_proc(int) const;    
    vector<int> fitness;
    friend std::ostream& operator<< (std::ostream &out, const Position &p);
    
    Position& operator=(const Position& p)
    {
        proc_mappings = p.proc_mappings;
        proc_modes = p.proc_modes;
        tdmaAlloc = p.tdmaAlloc;
        fitness = p.fitness;
        proc_sched.clear();
        send_sched.clear();
        rec_sched.clear();
        for(auto s: p.proc_sched)
            proc_sched.push_back(std::move(s));
        for(auto s: p.send_sched)
            send_sched.push_back(std::move(s));
        for(auto s: p.rec_sched)
            rec_sched.push_back(std::move(s));               
        return *this;
    }

 
    bool empty() const
    {
        return fitness.empty();
    }
    static int weighted_sum(int a, int b, float w) 
    {
        float diff = w*(b - a);
        return Schedule::random_round((float) a + diff);
    }
 
  
};
class Particle{
public: 
    Particle(shared_ptr<Mapping>, shared_ptr<Applications>);
    /** 
     * Returns the fitness value of the particle with respect to different objectives.
     */ 
    vector<int> get_fitness();
    void calc_fitness();
    vector<int> get_next(vector<Schedule>, int);
    void set_best_global(Position);
    Position get_current_position();
    Position get_best_local_position();
    void update_position();/** updates the current position based on the local best and global best.*/
    friend std::ostream& operator<< (std::ostream &out, const Particle &particle);
    static std::ostream& print_vector (std::ostream &out, const vector<int> &v) ;
private:    
    shared_ptr<Mapping> mapping;
    shared_ptr<Applications> applications;
    const size_t no_entities; /**< total number of actors and tasks. */
    const size_t no_actors; /**< total number of actors and tasks. */
    const size_t no_channels; /**< total number of channels. */
    const size_t no_processors; /**< total number of processors. */
    const size_t no_tdma_slots; /**< total number of TDMA slots. */
    Position current_position;
    Position best_local_position;
    Position best_global_position;
    void init_random();
    void build_schedules(Position&);/** builds proc_sched, send_sched and rec_sched based on the mappings.*/        
    void repair_tdma(Position&);
    void repair_sched(Position&);
    void repair_send_sched(Position&);
    void repair_rec_sched(Position&);    
    void repair(Position&);
    vector<int> get_channel_by_src(Position&, int) const;
    vector<int> get_channel_by_dst(Position&, int) const;    
    /**
     * Adds position p2 to p1.
     * Uses the weight in adding.
     */ 
    void add_positions(Position& p1, const Position& p2, float w) ;
    bool is_better_than_lb(vector<int> f);
    int bring_to_bound(int, int, int);
    vector<int> bring_v_to_bound(vector<int>, int, int);
};

