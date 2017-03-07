#include "ga_population.hpp"

GA_Population::GA_Population(shared_ptr<Mapping> _mapping, shared_ptr<Applications> _application, Config& _cfg):
            Population(_mapping, _application, _cfg) 
{
    short_term_memory.set_mem_size(1);
    name = "GA";
    
    int thresh = sqrt(no_individulas)+1;
    for(int i=0;i<thresh;i++)
    {
        for(int j=0;j<thresh;j++)
        {
            pair<int,int> par (i,j);
            //if(i != j)
                possible_parents.push_back(par);
        }
    }
   cout << "no_ind:" << no_individulas << " parents:" << possible_parents.size() << endl;
}

void GA_Population::init()
{
    LOG_INFO("Initializing the GA population reinit:"+tools::toString(no_reinits)); 
    no_reinits++;
    last_reinit = current_generation;
    population.clear();
    old_population.clear();
    next_population.clear();
    for(size_t i=0;i<no_individulas;i++)
    {
        shared_ptr<Chromosome> c(new Chromosome(mapping, applications, 
                                cfg.settings().multi_obj, cfg.settings().fitness_weights, penalty));        
        population.push_back(c);        
        old_population.push_back(c);        
        next_population.push_back(c);        
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
        /// Copy parent one because it will be updated
        shared_ptr<Chromosome> parent1(new Chromosome(*population[parents[i].first]));
        
        shared_ptr<Chromosome> parent2 = population[parents[i].second];
        
        parent1->set_best_global(parent2->get_current_position());
        parent1->update();
        
        next_population[i] = parent1;        
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
    //select_fittest();
    population = next_population;
}
void GA_Population::sort_population()
{
    std::sort(population.begin(), population.end(),
                    [](shared_ptr<Chromosome> const a, shared_ptr<Chromosome> const b) -> bool 
                    { return a->dominate(b); } );    
    
    //We also slecet parents here
    parents.clear();
    vector<int> all_indices;
    for(size_t i=0;i<possible_parents.size();i++)
        all_indices.push_back(i);
    vector<int> slected_indices;
    for(size_t i=0;i<no_individulas;i++)
    {
        int rand = random::random_indx(all_indices.size()-1);        
        int indx = all_indices[rand];
        parents.push_back(possible_parents[indx]);
        all_indices.erase(all_indices.begin()+rand);
    }     
}
