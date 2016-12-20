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
    int get_element_by_rank(int);
    int get_rank_by_id(int);
    int get_rank_by_element(int);
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

class Particle{
public: 
    Particle(shared_ptr<Mapping>, shared_ptr<Applications>);
    /** 
     * Returns the fitness value of the particle with respect to different objectives.
     */ 
    vector<int> get_fitness();
    vector<int> calc_fitness();
    vector<int> get_next(vector<shared_ptr<Schedule>>, int);
    friend std::ostream& operator<< (std::ostream &out, const Particle &particle);
private:    
    shared_ptr<Mapping> mapping;
    shared_ptr<Applications> applications;
    const size_t no_entities; /**< total number of actors and tasks. */
    const size_t no_actors; /**< total number of actors and tasks. */
    const size_t no_channels; /**< total number of channels. */
    const size_t no_processors; /**< total number of processors. */
    const size_t no_tdma_slots; /**< total number of TDMA slots. */
    vector<shared_ptr<Schedule>> proc_sched;
    vector<shared_ptr<Schedule>> send_sched;
    vector<shared_ptr<Schedule>> rec_sched;
    vector<int> proc_mappings;
    vector<int> proc_modes;    
    vector<int> sendNext;
    vector<int> recNext;
    vector<int> tdmaAlloc;    
    vector<int> fitness;
    void init_random();
    void repair_tdma();
    void repair_sched(vector<shared_ptr<Schedule>>, int);
    void repair_send_sched(vector<shared_ptr<Schedule>>, int);
    void repair_rec_sched(vector<shared_ptr<Schedule>>, int);    
    void repair();
    vector<int> get_actors_by_proc(int);
    vector<int> get_channel_by_src(int);
    vector<int> get_channel_by_dst(int);
};

