#include "ga_population.hpp"

GA_Population::GA_Population(shared_ptr<Mapping> _mapping, shared_ptr<Applications> _application, Config& _cfg):
            Population(_mapping, _application, _cfg) 
{
    short_term_memory.set_mem_size(1);
    name = "GA";
}

void GA_Population::init()
{
    LOG_INFO("Initializing the GA population reinit:"+tools::toString(no_reinits)); 
    no_reinits++;
    last_reinit = current_generation;
    population.clear();
    old_population.clear();
    for(size_t i=0;i<no_individulas;i++)
    {
        shared_ptr<Chromosome> c(new Chromosome(mapping, applications, 
                                cfg.settings().multi_obj, cfg.settings().fitness_weights));        
        population.push_back(c);        
        old_population.push_back(c);        
    }       
}
GA_Population::~GA_Population()
{}

std::ostream& operator<< (std::ostream &out, const GA_Population &pop)
{
    out << "no individuals=" << pop.no_individulas << endl;
    return out;
}


void GA_Population::update(int t_id)
{
    int start_id = t_id * individual_per_thread;
    int end_id = start_id + individual_per_thread;
    if(t_id == no_threads -1)///Last thread takes care of all remaining particles
        end_id = no_individulas;
    for(int i=start_id;i<end_id;i++)    
    {
        if(cfg.settings().multi_obj && !par_f.pareto.empty())
        {
            int par_indx = random::random_indx(par_f.pareto.size()-1);        
            population[i]->set_best_global(par_f.pareto[par_indx]);
           
            old_population[i] = population[i];
            population[i]->update();
        }
        if(!cfg.settings().multi_obj && !short_term_memory.empty())
        {
            int mem_indx = random::random_indx(short_term_memory.mem.size()-1);        
            population[i]->set_best_global(short_term_memory.mem[mem_indx]);
           
            old_population[i] = population[i];
            population[i]->update();
        }
    }
    
}

int GA_Population::no_converged_individuals()
{
    int cnt = 0;
    return cnt;
}
bool GA_Population::termination()
{
    current_generation++;
    return (current_generation - last_update > no_generations);
}
bool GA_Population::is_converged()
{
    //return (no_converged_individuals() > (int)no_individulas/4);
    const int thresh = no_generations/10;
    const int delta_update = current_generation - last_update;
    const int delta_reinit = current_generation - last_reinit;
    return (delta_update > thresh && delta_reinit > thresh);
}
void GA_Population::print_results()
{
    string sep="";         
   for(size_t i=0;i<100;i++)
       sep+="=";
    for(auto p : par_f.pareto)
       out << p << endl << sep << endl;
   for(auto p : population)
        out << "position:\n" << *p << endl << sep << endl;    
}
void GA_Population::select_fittest()
{
    population.insert(population.end(), old_population.begin(), old_population.end());
    std::sort(population.begin(), population.end(),
                    [](shared_ptr<Chromosome> const a, shared_ptr<Chromosome> const b) -> bool 
                    { return a->dominate(b); } );
    population.erase (population.begin()+no_individulas,population.end());   
}
void GA_Population::new_population()
{
    select_fittest();
}
