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
#include <thread>
#include <chrono>

#include "individual.hpp"

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
    Memory();
    vector<Position> mem;
    std::chrono::duration<double>  last_update; 
    bool empty();
    bool update_memory(Position, std::chrono::duration<double>);
    void remove_worst();
    bool exists_in_mem(Position&);
    void set_mem_size(int);   
    friend std::ostream& operator<< (std::ostream &out, const Memory &m);
private:    
    size_t max_size;    
};

