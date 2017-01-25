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
#include "population.cpp"
#include "population_data.hpp"
using namespace std;
/**
 * \class Swarm
 *
 * \brief The swarm class in PSO.
 *
 */
class Swarm : public Population<Particle>{
public: 
    Swarm(shared_ptr<Mapping>, shared_ptr<Applications>, Config&);
    ~Swarm();
    friend std::ostream& operator<< (std::ostream &out, const Swarm &swarm);
private:    
    vector<shared_ptr<Particle>> opposition_set;
    void update(int);/** updates the best position of particles in a thread. */ 
    void init();/*!< Initializes the particles. */    
    void evaluate_oppositions();/*! Evaluates the opposition particles and adds the good ones to the particle set.*/
    void merge_main_opposite();/*! Merges the opposition set with the main particle set.*/
    void print_results();
    bool termination();
    bool is_converged();
    float average_speed();
    int no_converged_particles();
    void replace_converged_particles();
    int no_reinits;
};

