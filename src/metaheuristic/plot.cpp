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

#include "../exceptions/runtimeexception.h"
#include "../tools/stringtools.hpp"

using namespace std;
/**
 * \class ParetoFront
 *
 * \brief Stores the pareto front of the \ref Swarm.
 *
 */
class Plot{
public:    
    Plot(vector<string>& _titles, vector<vector<int>> _values):
         titles(_titles),
         values(_values)
    {
        /*if(titles.size() != values.size())
                    THROW_EXCEPTION(RuntimeException, "title and values sizes mismatch!");*/
        create_csv();
        create_tex();
    }
    vector<string> titles;
    vector<vector<int>> values;
    string csv;
    string tex;
    const string sep = ",";
private:
    void create_csv()
    {
        csv = "";
        for(auto t : titles)
           csv += t + sep;
        csv += "\n";   
        for(auto vec : values)
        {
            for(auto val : vec)
            {
                csv += tools::toString(val) + sep;
            }
            csv += "\n";
        }
    }
    void create_tex()
    {
        tex = "";
        tex += "\\documentclass{standalone}\n";
        tex += "\\usepackage{filecontents}\n";
        tex += "\\usepackage{pgfplots, pgfplotstable}\n";
        tex += "\\tikzset{mark options={mark size=.1, line width=.3pt}}\n";
        tex += "\\begin{document}\n";
        for(size_t i=0;i<titles.size()-2;i++)
        {
            string fig = "";
            fig += "\\begin{tikzpicture}[only marks, y=.5cm]\n";
            fig += "\\begin{axis}[ymin=0, title={},xlabel = {Energy},ylabel = {";
            fig += titles[i] + "}, cycle list name=black white, smooth,legend entries={";
            fig += titles[i] + sep;
            fig += "},legend pos=outer north east,]\n";
            fig += "\\addplot table [col sep = comma, x = energy, y =" + titles[i] + ", ] {data.csv};\n";
            fig += "\\end{axis}\n";
            fig += "\\end{tikzpicture}\n";
            tex += fig;
        }
        tex += "\\end{document}";
        
    }    
    
};
