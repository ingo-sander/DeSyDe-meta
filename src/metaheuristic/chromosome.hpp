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
 * \class Chromosome
 *
 * \brief The chromosome class in GA.
 *
 */
class Chromosome: public Individual{
public:
    /**
     * Constructor method.
     * @param _mapping
     *        Pointer to the mapping object.
     * @param _application
     *        Pointer to the application object.
     * @param _objective
     *        Chromosome objective.
     * @param If we are solving a multiobjective problem
     * @param _o_w
     *        Weight of objective function.
     */ 
    Chromosome(shared_ptr<Mapping>, shared_ptr<Applications>, bool, vector<float>, vector<int>);
    Chromosome(const Chromosome&);
    ~Chromosome(){};
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
    void avoid_stagnation(){}; 
    /**
     * Overloads the << operator.
     */      
    //friend std::ostream& operator<< (std::ostream &out, const Chromosome &ch);        
private:    
    void crossover();/*!< The cross-over operation between this chromosome and best global chromosome.*/    
    void cross_mut();/*!< The cross-over and mutation operation at the same time.*/    
    vector<int> mutation(vector<int>);
    vector<Domain> mutation(vector<Domain>);
    /**
     * Performes crossover on the two input int vectors.
     * @return int vector which is the crossover result.
     */ 
    template<class T> 
    vector<T> crossover(vector<T>, vector<T>);
    /**
     * Performes crossover on the two input \b Schedule vectors.
     * @return \b Schedule vector which is the crossover result.
     */
    vector<Schedule> crossover(vector<Schedule> new_s, vector<Schedule> s1, vector<int> map1, vector<Schedule> s2, vector<int> map2, int no_elems);
};

