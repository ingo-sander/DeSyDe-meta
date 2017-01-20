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
#include <thread>
#include <chrono>

#include "../exceptions/runtimeexception.h"
#include "particle.hpp"
using namespace std;
/**
 * \struct ParetoFront
 *
 * \brief Stores the pareto front of the \ref Swarm.
 *
 */
struct ParetoFront
{
    ParetoFront();
    vector<Position> pareto;
    /**
     * Does pareto[indx] dominate p?
     */ 
    bool dominate(Position&, int);
    /**
     * Does pareto dominate p?
     */ 
    bool dominate(Position& p);
    /**
     * Compares the input position with the current front 
     * and replaces if it dominates
     */ 
     /**
     * Does p1 dominate p2?
     */ 
    bool dominate(Position& p1, Position& p2);
    bool update_pareto(Position);
    /**
     * @return True if the pareto front is empty.
     */ 
    bool empty();
    friend std::ostream& operator<< (std::ostream &out, const ParetoFront &p);
};
/**
 * \struct Memory
 *
 * \brief Stores the memory of the \ref Swarm.
 *
 */
struct Memory
{
    Memory(){};
    vector<Position> mem;
    bool empty();
    bool update_memory(Position);
    void remove_worst();
    bool exists_in_mem(Position&);
    std::chrono::duration<double>  ins_time;
    const size_t max_size=1;
    friend std::ostream& operator<< (std::ostream &out, const Memory &m);
};
/**
 * \class Swarm
 *
 * \brief The swarm class in PSO.
 *
 */
class Swarm{
public: 
    Swarm(shared_ptr<Mapping>, shared_ptr<Applications>, Config&);
    ~Swarm();
    void search();
    friend std::ostream& operator<< (std::ostream &out, const Swarm &swarm);
private:    
    Config& cfg;
    shared_ptr<Mapping> mapping;
    shared_ptr<Applications> applications;
    vector<shared_ptr<Particle>> particle_set;
    vector<shared_ptr<Particle>> opposition_set;
    const size_t no_objectives; /**< total number of objectives. */
    const size_t no_particles; /**< total number of particles. */
    const size_t no_generations; /**< total number of particles. */
    const int no_threads;
    int particle_per_thread;
    ParetoFront par_f;
    Memory long_term_memory;/** used in case of single objective.*/
    Memory short_term_memory;/** used in case of single objective.*/
    vector<Memory> memory_hist;/** used for outputing the development of the solution.*/
    ofstream out, out_csv, out_tex;;
    bool stagnation;
    const bool multi_obj = false;
    typedef std::chrono::high_resolution_clock runTimer; /**< Timer type. */
    runTimer::time_point t_start, t_endAll; /**< Timer objects for start and end of experiment. */
    int random_indx(int);/** returns index of a random index. */
    void calc_fitness(int);/** calculates the fitness for particles in a thread. */ 
    void update_position(int);/** updates the best position of particles in a thread. */ 
    void init();/*!< Initializes the particles. */    
    void evaluate_oppositions();/*! Evaluates the opposition particles and adds the good ones to the particle set.*/
    void merge_main_opposite();/*! Merges the opposition set with the main particle set.*/
    void print();
    float average_speed();
    int no_converged_particles();
    void replace_converged_particles();
    int no_reinits;
};

