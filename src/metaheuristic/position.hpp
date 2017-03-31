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
#include "../tools/tools.hpp"
#include "../tools/meta_tools.cpp"
using namespace std;

/**
 * \class Schedule
 *
 * \brief Stores Schedules.
 * 
 * Each element is associated with a rank which is the order of that 
 * element in the order-based schedule.
 *
 */
class Schedule{
public:
    /** Creates a random schedule out of input elements and dummy node. */
    Schedule(vector<int>, int);
    friend std::ostream& operator<< (std::ostream &out, const Schedule &sched);
    void set_rank(int index, int value);
    void set_rank(vector<int> _rank);
    vector<int> get_rank();
    vector<int> get_next();
    /** Element-wise difference of rank and input vector. */
    vector<int> rank_diff(vector<int>);
    /** Addes the rank with the input (speed). */
    void rank_add(vector<float>);
    /** Randomly slecets ceil or floor. */
    static int random_round(float);
    int get_next(int);
    void switch_ranks(int, int);    
    vector<int> get_elements();
    int get_element_by_rank(int) const;
    int get_index_by_element(int) const;
    int get_rank_by_id(int) const;
    int get_rank_by_element(int) const;
    float get_relative_rank_by_element(int) const;
    void set_rank_by_element(int, int);
    int random_unused_rank();
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
    size_t size()
    {
        return elements.size();
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
/**
 * \class Speed
 *
 * \brief Stores speed of the particles (\f$ V(t) \f$).
 *
 */
struct Speed{
    Speed(int no_actors, int no_channels, int no_processors, int no_entities)
    {
        //proc_sched = random_v(no_actors, -no_actors/no_processors, no_actors/no_processors);
        //send_sched = random_v(no_channels, -no_channels/no_processors, no_channels/no_processors);
        //rec_sched =  random_v(no_channels, -no_channels/no_processors, no_channels/no_processors);
        //proc_mappings = random_v(no_actors, -no_processors/2, no_processors/2);
        proc_sched = random_v(no_actors, -2, 2);
        send_sched = random_v(no_channels, -2, 2);
        rec_sched =  random_v(no_channels, -2, 2);
        proc_mappings = random_v(no_actors, -2, 2);
        proc_modes = random_v(no_actors, -1, 1);
        tdmaAlloc = random_v(no_processors, -1, 1);
        app_group = random_v(no_entities, -1, 1);
        proc_group = random_v(no_processors, -1, 1);
    }
    vector<float> proc_sched;
    vector<float> send_sched;
    vector<float> rec_sched;
    vector<float> proc_mappings;
    vector<float> proc_modes;
    vector<float> tdmaAlloc;
    vector<float> app_group;
    vector<float> proc_group;
    static vector<float> random_v(size_t s, float min, float max) 
    {
        random_device rnd_device;
        mt19937 mersenne_engine(rnd_device());
        uniform_real_distribution<> dist(min, max);

        /// -# The engine has to be reset after crreating the distribution
        auto gen = std::bind(dist, mersenne_engine);
        vector<float> v(s, 0);
        generate(begin(v), end(v), gen); 
        return v;
    }   
    float average() const;
    void apply_bounds();
    friend std::ostream& operator<< (std::ostream &out, const Speed &s);
};
/**
 * \struct Domain
 * 
 * \brief Stores domain and value of a decision variable.
 */ 
struct Domain{
private:
    int val_indx = 0;
public:    
    set<int> domain;    
    int value() const;    
    int index() const;
    void set_index(int indx);
    Domain& operator=(const Domain& d);
};
/**
 * \struct Position
 *
 * \brief Stores the position of particles.
 */
struct Position{
public:
    Position(bool _multi_obj, vector<float> _w);
   ~Position();
    bool multi_obj;
    vector<Schedule> proc_sched;
    vector<Schedule> send_sched;
    vector<Schedule> rec_sched;
    vector<Domain> proc_mappings;
    vector<int> proc_modes;        
    vector<int> tdmaAlloc;     
    vector<int> get_actors_by_proc(int) const;    
    vector<int> fitness;
    int penalty;
    vector<float> weights;
    int cnt_violations;
    vector<int> app_group;/*!< mapping of apps to co-mapping groups.*/
    vector<int> proc_group;/*!< mapping of co-mapping groups to apps.*/
    friend std::ostream& operator<< (std::ostream &out, const Position &p);
    
    Position(const Position &obj);
    void print_multi_obj();
    bool operator==(const Position& p_in) const;
    bool operator!=(const Position& p_in) const;
    Position& operator=(const Position& p);
    /**
     * Do I dominate p_in?
     */ 
    bool dominate(Position& p_in) const;
    bool dominate(Position& p_in, vector<float>) const;
    float fitness_func() const;
    bool empty() const;
    bool invalid() const;
    static int weighted_sum(int a, int b, float w);
    static int weighted_sum(int a, int b, int c, float w1, float w2);  
    void opposite();/*!< moves the position to the opposite side.*/
    vector<int> actors_by_mapping(int proc);
    vector<int> opposite_availabe_procs(int actor, vector<int> new_proc_mappings);
    int select_random(vector<int> v);
    vector<int> get_proc_mappings() const;
};
