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


#include "individual.hpp"

using namespace std;

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
class Particle: public Individual{
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
    Particle(shared_ptr<Mapping>, shared_ptr<Applications>, int, float, float, float, bool, vector<float>, vector<int>);
    Particle(const Particle&);
    ~Particle(){};   
    /** Updates the current position based on the local best and global best.*/
    /**
     * Updates the position based on the speed.
     */ 
    void update();
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
    const int objective;/*!< Objective of the particle. */
    Position best_local_position;/*!< Individual memory.*/
    Speed speed;/*!< Particle \c speed \f$ V(t) \f$.*/
    float w_t;/*!< Weight of current speed. \f$ w \f$ in \ref update_speed. */
    float w_lb;/*!< Weight of local best. \f$ c_1 \f$ in \ref update_speed. */
    float w_gb;/*!< Weight of global best.\f$ c_2 \f$ in \ref update_speed. */
    const float delta_w_t = 0.01; /*!< Delta for decreasing \c w_t.*/
    const float min_w_t = 0.1;/*!< Minimum \c w_t.*/ 
    const float max_w_t;/*!< Maximum \c w_t.*/ 
    /**
     * Updates the speed of the particle using the following equation:
     * \f[
     * V(t+1) = wV(t) + c_1 y_1 (X(t)-Y(t)) + c_2 y_2 (X(t)-Y(t))
     * \f]
     */ 
    void update_speed();
    void move();/*!< Moves the particle \f$ X(t+1) = X(t)+V(t+1) \f$.*/    
};

