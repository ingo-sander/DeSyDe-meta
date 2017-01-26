#include <vector>
#include <algorithm>
#include <random>
#include <iterator>
#include <iostream>
#include <functional>
#include <thread>
#include <chrono>
#include "../exceptions/runtimeexception.h"
#include "individual.hpp"
#include "population_data.hpp"
#include "plot.cpp"
/**
 * \class Population
 *
 * \brief The population class for population-based metaheuristics.
 *
 */
template <class T>
class Population{
public: 
Population(shared_ptr<Mapping> _mapping, shared_ptr<Applications> _application, Config& _cfg):
                    cfg(_cfg),
                    mapping(_mapping),
                    applications(_application),
                    no_objectives(mapping->getNumberOfApps()+2),
                    no_individulas(no_objectives*cfg.settings().particle_per_obj),
                    no_generations(cfg.settings().generation),
                    no_threads(std::thread::hardware_concurrency()),
                    current_generation(0),
                    last_update(0),
                    par_f(),                    
                    stagnation(false)
{   
    individual_per_thread = no_individulas / no_threads;
    no_reinits = 0;
}
~Population()
{
    population.clear();
}

void search()
{
    out.open(cfg.settings().output_path+"out/out_meta.txt");
    out_csv.open(cfg.settings().output_path+"out/data.csv");
    out_tex.open(cfg.settings().output_path+"out/plot.tex");
    
    t_start = runTimer::now();
    auto dur_fitness = runTimer::now() - runTimer::now();
    auto dur_update = runTimer::now() - runTimer::now();
    std::thread t[no_individulas];
    
    size_t g = 0;
    while(is_timedout() == false && termination() == false)
    {
        if(is_converged())
        {
            init();
            short_term_memory.mem.clear();
        }
        if(par_f.empty() && long_term_memory.empty())
            init();
             
        auto start_fitness = runTimer::now();
        for (int i = 0; i < no_threads; i++) 
        {
            t[i] = std::thread(&Population::calc_fitness, this, i);                   
        }
        
        //Join the threads 
        for (int i = 0; i < no_threads; i++) 
        {
             t[i].join();
        }
        
        dur_fitness += runTimer::now() - start_fitness;

        ///Find the best individual
        for(size_t p=0;p<population.size();p++)
        {
            if(cfg.settings().multi_obj)
            {
                short_term_memory.update_memory(population[p]->get_current_position());
                if(par_f.update_pareto(population[p]->get_current_position()))
                    last_update = g;
            }
            else
            {
                short_term_memory.update_memory(population[p]->get_current_position());
                if(long_term_memory.update_memory(population[p]->get_current_position()))
                {
                    last_update = g;
                    long_term_memory.ins_time = runTimer::now() - t_start;
                    memory_hist.push_back(long_term_memory);
                     out << long_term_memory.mem[0] << endl;   
                }
            }
        }
            
        auto start_update = runTimer::now();
        if(g+1- last_update < no_generations)        
        {
            LOG_DEBUG("updating positions ");
            /// update the positions
            for (int i = 0; i < no_threads; i++) 
            {
                t[i] = std::thread(&Population::update, this, i);            
            }
            //Join the threads 
            for (int i = 0; i < no_threads; i++) 
            {
                 t[i].join();
            }
        }   
        //merge_main_opposite();
        dur_update += runTimer::now() - start_update;     
        g++;
        
    }
    auto durAll = runTimer::now() - t_start;
    auto durAll_s = std::chrono::duration_cast<std::chrono::seconds>(durAll).count();
    auto dur_fitness_s = std::chrono::duration_cast<std::chrono::milliseconds>(dur_fitness).count();
    auto dur_update_s = std::chrono::duration_cast<std::chrono::milliseconds>(dur_update).count();
    auto durAll_ms = std::chrono::duration_cast<std::chrono::milliseconds>(durAll).count();
    std::stringstream stat;
    stat    << "===== search ended after: " << durAll_s/3600 << " h " 
            << (durAll_s%3600)/60 << " m "
            << (durAll_s%60) << " s "
            << "(" << durAll_ms 
            << " ms)\nfitness=" << dur_fitness_s << "ms update=" << dur_update_s << "ms \n"
            << "no threads=" << no_threads 
            << " last update in generation " << last_update
            << " pareto size " << par_f.pareto.size()
            << endl;
   cout << stat.str() << endl;
   out << stat.str() << endl;         
   string sep="";         
   for(size_t i=0;i<100;i++)
       sep+="=";
    
   out << sep << endl;
   if(cfg.settings().multi_obj) 
   {
            out << par_f << sep << endl;               
   }
   else
   {
        out << long_term_memory << sep << endl;       
        cout << long_term_memory << sep << endl;       
   }
   print_results();
   out.close();
   
   print();
   out_csv.close();
   out_tex.close();
   
}


protected:    
Config& cfg;
shared_ptr<Mapping> mapping;
shared_ptr<Applications> applications;
vector<shared_ptr<T>> population;
const size_t no_objectives; /**< total number of objectives. */
const size_t no_individulas; /**< total number of particles. */
const size_t no_generations; /**< total number of generations. */
const int no_threads;
size_t individual_per_thread;
size_t current_generation;
size_t last_update;
ParetoFront par_f;
Memory long_term_memory;/** used in case of single objective.*/
Memory short_term_memory;/** used in case of single objective.*/
vector<Memory> memory_hist;/** used for outputing the development of the solution.*/
ofstream out, out_csv, out_tex;;
bool stagnation;
const bool multi_obj = false;
typedef std::chrono::high_resolution_clock runTimer; /**< Timer type. */
runTimer::time_point t_start, t_endAll; /**< Timer objects for start and end of experiment. */
int no_reinits;


virtual void update(int){};/** updates the population in a thread. */ 
virtual void init(){};/*!< Initializes the particles. */    


virtual bool termination(){return false;};
virtual bool is_converged(){return false;};
virtual void print_results(){};
bool is_timedout()
{
    if(cfg.settings().timeout_first == 0)
        return false;
    auto duration = runTimer::now() - t_start;
    return (std::chrono::duration_cast<std::chrono::milliseconds>(duration).count() > cfg.settings().timeout_first);
}
    


int random_indx(int max)
{
    random_device rnd_device;
    uniform_int_distribution<int> dist(0, max);
    mt19937 mersenne_engine(rnd_device());  
    auto gen = std::bind(dist, mersenne_engine);
    int i = gen();
    return i;    
}
/** calculates the fitness for individuals in a thread. */ 
void calc_fitness(int t_id)
{
    int start_id = t_id * individual_per_thread;
    int end_id = start_id + individual_per_thread;
    if(t_id == no_threads -1)///Last thread takes care of all remaining particles
        end_id = no_individulas;
    for(int i=start_id;i<end_id;i++)    
    {
        population[i]->calc_fitness();        
    }
}

void print()
{
   vector<vector<int>> data;
   
   for(auto p : par_f.pareto)
   {
       vector<int> tmp;
       tmp.push_back(p.fitness_func());
       tmp.insert(tmp.end(), p.fitness.begin(), p.fitness.end());       
       data.push_back(tmp);
   }
   for(auto m : memory_hist)
       for(auto p : m.mem)
       {
           vector<int> tmp;
           tmp.push_back(p.fitness_func());
           tmp.insert(tmp.end(), p.fitness.begin(), p.fitness.end());
           auto durAll_ms = std::chrono::duration_cast<std::chrono::milliseconds>(m.ins_time).count();
           tmp.push_back(durAll_ms);
           data.push_back(tmp);
       }

   std::sort (data.begin(), data.end()); 
   vector<string> titles;
   titles.push_back("fitness");    
   for(int i=0;i<mapping->getNumberOfApps();i++)
       titles.push_back("app"+tools::toString(i)+"-period");
   titles.push_back("energy");    
   titles.push_back("memory");
   titles.push_back("time");
   Plot pl(titles, data);       
   out_csv << pl.csv;
   out_tex << pl.tex;
}

};



