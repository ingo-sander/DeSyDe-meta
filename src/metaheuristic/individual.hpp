#pragma once
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

#include "position.hpp"
using namespace std;

/**
 * \class Individual
 *
 * \brief The individual class in population-based metaheuristics.
 *
 */
class Individual{
public:
    /**
     * Constructor method.
     * @param _mapping
     *        Pointer to the mapping object.
     * @param _application
     *        Pointer to the application object.
     * @param _multi_obj
     *        True if we are solving a multiobjective problem.
     * @param _obj_weights
     *        Weights used for calculating the objective function.
     */ 
    Individual(shared_ptr<Mapping>, shared_ptr<Applications>, bool, vector<float>);
    Individual(const Individual&);
    ~Individual(){};
    /** 
     * Returns the fitness value of the current position of the individual.
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
     * Returns the current position of the particle.
     * @return position
     *         \c Position object.
     */ 
    Position get_current_position() const;
    virtual void update(){};/*!< Updates the individual based on an internal algorithm.*/
    /**
     * Overloads the << operator.
     */  
    bool dominate(const shared_ptr<Individual>) const;     
    /**
     * Move the individual's current position to the opposite position.
     */
     void opposite(); 
protected:    
    shared_ptr<Mapping> mapping;/*!< Pointer to \c Mapping object.*/
    shared_ptr<Applications> applications;/*!< Pointer to \c Application object.*/
    const size_t no_entities; /*!< Total number of actors and tasks. */
    const size_t no_actors; /*!< Total number of actors and tasks. */
    const size_t no_channels; /*!< Total number of channels. */
    const size_t no_processors; /*!< Total number of processors. */
    const size_t no_tdma_slots; /*!< Total number of TDMA slots. */
    Position current_position;/*!< Current \c position.*/
    int no_invalid_moves;/*!< Number of moves in which the schedule resulted in deadlock (negative fitness).*/
    const int thr_invalid_mov = 25;/*!< Threshold for the number of invalid moves.*/
    const bool multi_obj;/*!< True if we are solving a multiobjective optimization problem.*/
    vector<float> obj_weights;/*!< Used for calculating the objective functions.*/
    void init_random();/*!< Randomly initializes the individual.*/
    void build_schedules(Position&);/*!< builds proc_sched, send_sched and rec_sched based on the mappings.*/        
    void repair_tdma(Position&);/*!< Repairs the \c tdmaAlloc vector in \ref Position.*/
    void repair_sched(Position&);/*!< Repairs the \c proc_sched in \ref Position.*/
    void repair_send_sched(Position&);/*!< Repairs the \c send_sched in \ref Position.*/
    void repair_rec_sched(Position&);/*!< Repairs the \c rec_sched in \ref Position.*/
    void repair(Position&);/*!< Calls all other repair functions.*/
    vector<int> get_channel_by_src(Position&, int) const;
    /**
     * @param dst_proc_id Processor of the destination of the channels.
     * @return Vector of channels where \c destination is on \c dst_proc_id.
     */ 
    vector<int> get_channel_by_dst(Position&, int) const;    
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
