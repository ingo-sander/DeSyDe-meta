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


#ifndef TOOLS_METATOOLS_HPP
#define TOOLS_METATOOLS_HPP

#include <algorithm>
#include <sstream>
#include <fstream>
#include <istream>
#include <cstdlib>
#include <list>
#include <random>
#include <iterator>
#include <iostream>
#include <functional>
#include <vector>

using namespace std;
namespace tools {


template<class T>
static T bring_to_bound(T v, T l, T u)
{
    if(v < l)
        return l;
    if(v > u)
        return u;
    return v;    
}

template<class T>
static vector<T> bring_v_to_bound(vector<T> v, T l, T u)
{
    vector<T> out;
    for(auto i: v)
        out.push_back(tools::bring_to_bound(i, l, u));
    return out;    
}

template <class T>
static float average(vector<T> v) 
{
    float avg = 0.0;
    for(auto e : v)
    {
        avg+= e;
    }
    return avg/v.size();
}


}//namespace

#endif //TOOLS_METATOOLS_HPP
