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
    static bool random_bool();
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
/**
 * \class Speed
 *
 * \brief Stores speed of the particles (\f$ V(t) \f$).
 *
 */
struct Speed{
    Speed(int no_actors, int no_channels, int no_processors)
    {
        proc_sched.resize(no_actors, 0);
        send_sched.resize(no_channels, 0);
        rec_sched.resize(no_channels, 0);
        proc_mappings.resize(no_actors, 0);
        proc_modes.resize(no_processors, 0);
        tdmaAlloc.resize(no_processors, 0);
    }
    vector<float> proc_sched;
    vector<float> send_sched;
    vector<float> rec_sched;
    vector<float> proc_mappings;
    vector<float> proc_modes;
    vector<float> tdmaAlloc;
    friend std::ostream& operator<< (std::ostream &out, const Speed &s);
};
/**
 * \class Position
 *
 * \brief Stores the position of particles.
 */
struct Position{
public:
    Position(bool _multi_obj, vector<float> _w):
            multi_obj(_multi_obj),
            weights(_w)
            {
            };
   ~Position()
    {
        proc_sched.clear();
        send_sched.clear();
        rec_sched.clear();            
    }
    bool multi_obj;
    vector<Schedule> proc_sched;
    vector<Schedule> send_sched;
    vector<Schedule> rec_sched;
    vector<int> proc_mappings;
    vector<int> proc_modes;        
    vector<int> tdmaAlloc;     
    vector<int> get_actors_by_proc(int) const;    
    vector<int> fitness;
    vector<float> weights;
    friend std::ostream& operator<< (std::ostream &out, const Position &p);
    
    Position(const Position &obj) 
    {
        multi_obj = obj.multi_obj;
        proc_mappings = obj.proc_mappings;
        proc_modes = obj.proc_modes;
        tdmaAlloc = obj.tdmaAlloc;
        proc_sched = obj.proc_sched;
        send_sched = obj.send_sched;
        rec_sched = obj.rec_sched;
        fitness = obj.fitness;   
        weights = obj.weights;              
    }
    void print_multi_obj()
    {
        if(multi_obj)
        {
            cout << "multi objective\n";
            return;
        }
        if(!multi_obj)
        {
            cout << "single objective\n";    
            return;
        }
        cout << "unknown objective\n";    
    }
    bool operator==(const Position& p_in)
    {
        if(multi_obj)
        {
            for(size_t i=0;i<fitness.size();i++)
            {
                if(fitness[i] != p_in.fitness[i])
                    return false;
            }
            return true;
        }
        else
        {
            return (fitness_func() == p_in.fitness_func());
        }
    }
    Position& operator=(const Position& p)
    {
        proc_mappings = p.proc_mappings;
        proc_modes = p.proc_modes;
        tdmaAlloc = p.tdmaAlloc;
        fitness = p.fitness;
        weights = p.weights;
        multi_obj = p.multi_obj;
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
   
    /**
     * Do I dominate p_in?
     */ 
    bool dominate(Position& p_in)
    {
        if(empty())
            return false;
        /** -# fitness can not be negative. */            
        if(invalid())
            return false;      
            
        if(p_in.empty() || p_in.invalid())
            return true;
                  
        if(multi_obj)
        {
            for(size_t i=0;i<fitness.size();i++)
            {
                if(fitness[i] > p_in.fitness[i])
                    return false;
            }
            return true;
        }
        else
        {
            if(fitness_func() < p_in.fitness_func())
                return true;
            else
                return false;    
        }
    }
    float fitness_func() const
    {
        float f;
        for(size_t i=0;i<weights.size();i++)
            f += (float) fitness[i] * weights[i];
        
        return f;
    }
    bool empty() const
    {
        return fitness.empty();
    }
    bool invalid() const
    {
        for(auto f : fitness)
            if(f < 0)
                return true;   
       
       return false;         
    }
    static int weighted_sum(int a, int b, float w) 
    {
        float diff = w*(b - a);
        return Schedule::random_round((float) a + diff);
    }
    static int weighted_sum(int a, int b, int c, float w1, float w2) 
    {
        float diff1 = w1*(b - a);
        float diff2 = w2*(c - a);
        return Schedule::random_round((float) a + diff1 + diff2);
    }
 
  
};
/**
 * \class Particle
 *
 * \brief The particle class in PSO.
 *
 * This class abstracts a particle. Note that each particle contains 
 * 3 positions: (1) current position; (2) best local position, i.e., 
 * individual memory; (3) best global position, i.e., social memory.
 *
 * \note Each particle searchs for a particular objective.
 * \note The objective is set at the initialization. 
 *
 */
class Particle{
public:
    /**
     * Constructor method.
     * @param _mapping
     *        Pointer to the mapping object.
     * @param _application
     *        Pointer to the application object.
     * @param _objective
     *        Particle objective.
     * @param _w_t
     *        Maximum and initial weight of the current speed \c w_t.
     * @param _w_lb
     *        Weight of individual memory.
     * @param _w_gb
     *        Weight of social memory.
     */ 
    Particle(shared_ptr<Mapping>, shared_ptr<Applications>, int, float, float, float, bool, vector<float>);
    /** 
     * Returns the fitness value of the current position of the particle.
     * @return Vector of fitness values for all aobjectives.
     */ 
    vector<int> get_fitness();
    /** Calculate the fitness of the current position.*/
    void calc_fitness();
    /** Create the next vector for a vector of schedule objects which 
     * can be either proc_sched, next_sched or rec_sched.
     * @param no_elements
     *        The number of elements in the schedule.
     * @return Next vector.
     */ 
    vector<int> get_next(vector<Schedule>, int);
    /** 
     * The swarm object uses this function to update the social memory.
     * The particle keeps a copy of the best global position.
     * @param position 
     *        \c Position object.
     */   
    void set_best_global(Position);
    /**
     * Returns the current position of the particle.
     * @return position
     *         \c Position object.
     */ 
    Position get_current_position();
    /** Updates the current position based on the local best and global best.*/
    void update_position();
    /**
     * This is used by swarm in order to see which social memory is 
     * relevant for the particle.
     * @return \c objective of the particle.
     */ 
    int get_objective();
    /**
     * @return the speed of the objective.
     */ 
    Speed get_speed();
    /**
     * Implements a strategy to avoid stagnation.
     */
    void avoid_stagnation(); 
    /**
     * Overloads the << operator.
     */      
    friend std::ostream& operator<< (std::ostream &out, const Particle &particle);    
private:    
    shared_ptr<Mapping> mapping;/*!< Pointer to \c Mapping object.*/
    shared_ptr<Applications> applications;/*!< Pointer to \c Application object.*/
    const size_t no_entities; /*!< Total number of actors and tasks. */
    const size_t no_actors; /*!< Total number of actors and tasks. */
    const size_t no_channels; /*!< Total number of channels. */
    const size_t no_processors; /*!< Total number of processors. */
    const size_t no_tdma_slots; /*!< Total number of TDMA slots. */
    const int objective;/*!< Objective of the particle. */
    Position current_position;/*!< Current \c position.*/
    Position best_local_position;/*!< Individual memory.*/
    Position best_global_position;/*!< Social memory.*/
    Speed speed;/*!< Particle \c speed \f$ V(t) \f$.*/
    float w_t;/*!< Weight of current speed. \f$ w \f$ in \ref update_speed. */
    float w_lb;/*!< Weight of local best. \f$ c_1 \f$ in \ref update_speed. */
    float w_gb;/*!< Weight of global best.\f$ c_2 \f$ in \ref update_speed. */
    int no_invalid_moves;/*!< Number of moves in which the schedule resulted in deadlock (negative fitness).*/
    const int thr_invalid_mov = 30;/*!< Threshold for the number of invalid moves.*/
    const float delta_w_t = 0.05; /*!< Delta for decreasing \c w_t.*/
    const float min_w_t = 0.1;/*!< Minimum \c w_t.*/ 
    const float max_w_t;/*!< Maximum \c w_t.*/ 
    const bool multi_obj;/*!< True if we are solving a multiobjective optimization problem.*/
    vector<float> fitness_weights;
    void init_random();/*!< Randomly initializes the particle.*/
    void build_schedules(Position&);/*!< builds proc_sched, send_sched and rec_sched based on the mappings.*/        
    void repair_tdma(Position&);/*!< Repairs the \c tdmaAlloc vector in \ref Position.*/
    void repair_sched(Position&);/*!< Repairs the \c proc_sched in \ref Position.*/
    void repair_send_sched(Position&);/*!< Repairs the \c send_sched in \ref Position.*/
    void repair_rec_sched(Position&);/*!< Repairs the \c rec_sched in \ref Position.*/
    void repair(Position&);/*!< Calls all other repair functions.*/
    /**
     * Updates the speed of the particle using the following equation:
     * \f[
     * V(t+1) = wV(t) + c_1 y_1 (X(t)-Y(t)) + c_2 y_2 (X(t)-Y(t))
     * \f]
     */ 
    void update_speed();
    void move();/*!< Moves the particle \f$ X(t+1) = X(t)+V(t+1) \f$.*/
    /**
     * @param src_proc_id Processor of the source of the channels.
     * @return Vector of channels where \c source is on \c src_proc_id.
     */ 
    vector<int> get_channel_by_src(Position&, int) const;
    /**
     * @param dst_proc_id Processor of the destination of the channels.
     * @return Vector of channels where \c destination is on \c dst_proc_id.
     */ 
    vector<int> get_channel_by_dst(Position&, int) const;    
    /**
     * Compares \c f with the individual memory #best_local_position.
     * @return true if \c f is better than #best_local_position with 
     * respect to #objective and false otherwise.
     * @param f Fitness vector.
     */ 
    bool dominate(vector<int> f);
    /**
     * Moves \c v to a value between \c l and \c u.
     * @param v Value.
     * @param l Lower bound.
     * @param u Upper bound.
     * @return in bound value.
     */ 
    int bring_to_bound(int, int, int);
    /**
     * Moves all values in \c v in bound.
     * @param v Vector of values.
     * @return in bound vector.
     */ 
    vector<int> bring_v_to_bound(vector<int>, int, int);
    /**
     * Creates a random weight between 0 and 1 which is 
     * used by \ref update_speed function.
     * @return Random number between 0 and 1.
     */ 
    float random_weight();
};

