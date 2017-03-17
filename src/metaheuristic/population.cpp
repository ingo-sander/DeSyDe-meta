#pragma once
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
                    no_individulas(cfg.settings().no_individulas),
                    no_generations(cfg.settings().generation),
                    current_generation(0),
                    last_update(0),
                    last_short_term_update(0),
                    par_f(),      
                    stagnation(false),
                    no_reinits(0),
                    last_reinit(0)
{   
    if(cfg.settings().threads == 0)
        no_threads = std::thread::hardware_concurrency();
    else
        no_threads = cfg.settings().threads;
    individual_per_thread = no_individulas / no_threads;    
    init_penalty();
}
~Population()
{
    population.clear();
}

void search()
{
    string txt_file = cfg.settings().output_path+"out/out_"+name+".txt";
    string csv_file = cfg.settings().output_path+"out/data_"+name+".csv";
    string tex_file = cfg.settings().output_path+"out/plot_"+name+".tex";
    out.open(txt_file);
    out_csv.open(csv_file);
    out_tex.open(tex_file);
    
    cout << "output files:\n" << txt_file << endl << csv_file << endl << tex_file << endl;
    
    
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
        
        evaluate();
        sort_population();
        auto start_update = runTimer::now();
        if(g+1- last_update < no_generations)        
        {
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
        new_population();
        dur_update += runTimer::now() - start_update;     
        g++;
        
    }
    auto durAll = runTimer::now() - t_start;
    auto durAll_s = std::chrono::duration_cast<std::chrono::seconds>(durAll).count();
    auto dur_fitness_s = std::chrono::duration_cast<std::chrono::milliseconds>(dur_fitness).count();
    auto dur_update_s = std::chrono::duration_cast<std::chrono::milliseconds>(dur_update).count();
    auto durAll_ms = std::chrono::duration_cast<std::chrono::milliseconds>(durAll).count();
    std::stringstream stat;
    stat    << "===== search ended after: " << durAll_s/3600 << "h " 
            << (durAll_s%3600)/60 << "m "
            << (durAll_s%60) << "s "
            << "(" << durAll_ms 
            << " ms)\nfitness=" << dur_fitness_s << "ms update=" << dur_update_s << "ms \n"
            << "no threads=" << no_threads 
            << " last update in generation " << last_update
            << " last update time:" << std::chrono::duration_cast<std::chrono::seconds>(last_update_time).count()/60 << "m and "
            << (std::chrono::duration_cast<std::chrono::seconds>(last_update_time).count()%60) << "s"
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
        out << "long term memory:" << endl
            << long_term_memory << sep << endl
            << "short term memory:" << endl
            << short_term_memory << sep << endl;       
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
int no_threads;
size_t individual_per_thread;
size_t current_generation;
size_t last_update;
size_t last_short_term_update;
ParetoFront par_f;
Memory long_term_memory;/** used in case of single objective.*/
Memory short_term_memory;/** used in case of single objective.*/
vector<Memory> memory_hist;/** used for outputing the development of the solution.*/
ofstream out, out_csv, out_tex;;
bool stagnation;
const bool multi_obj = false;
typedef std::chrono::high_resolution_clock runTimer; /**< Timer type. */
runTimer::time_point t_start, t_endAll; /**< Timer objects for start and end of experiment. */
std::chrono::duration<double> last_update_time;
int no_reinits;
int last_reinit;
string name="meta";
vector<int> penalty;/*!< Vector of penalities, used when there is violations in the solution.*/
virtual void update(int){};/** updates the population in a thread. */ 
virtual void init(){};/*!< Initializes the particles. */    
virtual bool termination(){return false;};/*!< @return true if the termination conditions are true. */    
virtual void new_population(){};/*!< Updates the population. */ 
virtual void sort_population(){};/*!< Sorts the population from best to worst. */ 
/**
 * Evaluates the population. 
 */ 
void evaluate()
{
    if(cfg.settings().multi_obj)
    {
        for(size_t p=0;p<population.size();p++)
        {
            short_term_memory.update_memory(population[p]->get_current_position(), runTimer::now() - t_start);
            if(par_f.update_pareto(population[p]->get_current_position()))
            {
                last_update = current_generation;
                last_short_term_update = current_generation;
                last_update_time = runTimer::now() - t_start;
                if(par_f.pareto.size() % 100 == 0)
                    cout << "pareto size:" << par_f.pareto.size() << endl;
            }
        }
    }
    else
    {
        for(size_t p=0;p<population.size();p++)
        {
            if(short_term_memory.update_memory(population[p]->get_current_position(), runTimer::now() - t_start))
            {
                last_short_term_update = current_generation;
            }
            if(long_term_memory.update_memory(population[p]->get_current_position(), runTimer::now() - t_start))
            {
                last_update = current_generation;
                last_update_time = runTimer::now() - t_start;
                memory_hist.push_back(long_term_memory);
                 out << "reinit:" << no_reinits << endl                    
                     << "gen after reinit:" << current_generation - last_reinit << endl
                     << "gen:" << current_generation << endl
                     << "last reinit:" << last_reinit << endl
                     << "time:" << std::chrono::duration_cast<std::chrono::seconds>(runTimer::now() - t_start).count() << "s\n"
                     << long_term_memory.mem[0] << endl
                     << "penalty:" << long_term_memory.mem[0].penalty << endl
                     << "total_fitness:" << long_term_memory.mem[0].fitness_func() << endl;   
                 ///#- print the next variables
                 out << "proc_sched:" << tools::toString(population[p]->get_next(long_term_memory.mem[0].proc_sched, applications->n_SDFActors())) << endl;
                 out << "send_sched:" << tools::toString(population[p]->get_next(long_term_memory.mem[0].send_sched, applications->n_SDFchannels())) << endl;
                 out << "rec_sched:" << tools::toString(population[p]->get_next(long_term_memory.mem[0].rec_sched, applications->n_SDFchannels())) << endl;
                 out << endl;
                
            }
        }
    }
}
/**
 * @return true if the search is timed out.
 */ 
bool is_timedout()
{
    if(cfg.settings().timeout_first == 0)
        return false;
    auto duration = runTimer::now() - t_start;
    return (std::chrono::duration_cast<std::chrono::milliseconds>(duration).count() > cfg.settings().timeout_first);
}
    
/** 
 * Calculates the fitness for a number of individuals in the population
 * starting from \c t_id (\f$  \times \f$) \c individual_per_thread.  
 */ 
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
/**
 * Prints the paretor front or memory_history.
 */ 
void print()
{
   vector<vector<int>> data;
   
   for(auto p : par_f.pareto)
   {
       if(p.cnt_violations == 0 && p.penalty == 0)
       {
           vector<int> tmp;
           tmp.push_back(p.fitness_func());
           tmp.insert(tmp.end(), p.fitness.begin(), p.fitness.end());                  
           data.push_back(tmp);
       }
   }
   for(auto m : memory_hist)
       for(auto p : m.mem)
       {
           vector<int> tmp;
           if(p.cnt_violations == 0 && p.penalty == 0)
           {
               tmp.push_back(p.fitness_func());
               tmp.insert(tmp.end(), p.fitness.begin(), p.fitness.end());
               auto durAll_ms = std::chrono::duration_cast<std::chrono::milliseconds>(m.last_update).count();
               tmp.push_back(durAll_ms);
               data.push_back(tmp);
           }
       }

   std::sort (data.begin(), data.end()); 
   vector<string> titles;
   titles.push_back("fitness");    
   for(int i=0;i<mapping->getNumberOfApps();i++)
       titles.push_back("app"+tools::toString(i)+"-period");
   titles.push_back("energy");
   titles.push_back("time");
   Plot pl(titles, data);       
   out_csv << pl.csv;
   out_tex << pl.tex;
}
void print_results()
{
    string sep="";         
   for(size_t i=0;i<100;i++)
       sep+="=";
    for(auto p : par_f.pareto)
       out << p << endl << sep << endl;
   for(auto p : population)
        out << "individual:\n" << *p << endl << sep << endl;    
}

void init_penalty()
{
    penalty.clear();
    for(size_t i=0;i<mapping->getNumberOfApps();i++)
    {
        penalty.push_back(mapping->getSumWCETs(i) + mapping->getSumWCCTs(i));        
    }
    int max_pow = 0;
    for(size_t i=0;i<mapping->getPlatform()->nodes();i++)
    {
        
        vector<int> pow = mapping->getPlatform()->getPowerCons(i);
        int max = (*max_element(pow.begin(), pow.end()));
        if(max > max_pow)
            max_pow = max;
    }
    penalty.push_back(max_pow*mapping->getPlatform()->nodes()*mapping->max_utilization);
    
    
    cout << "sum_wcet = " << tools::toString(penalty) << endl;
}
/*!< @return true if the population is converged. */    
bool is_converged()
{
    return ((int) current_generation - (int)last_short_term_update - (int)last_reinit) > (int) cfg.settings().restart_generation;    
}
};



