#include "swarm.hpp"

Swarm::Swarm(shared_ptr<Mapping> _mapping, shared_ptr<Applications> _application, Config& _cfg):
            Population(_mapping, _application, _cfg) {}

void Swarm::init()
{
    no_reinits++;
    population.clear();
    opposition_set.clear();
    for(size_t i=0;i<no_individulas;i++)
    {
        shared_ptr<Particle> p(new Particle(mapping, applications, i%no_objectives,
                                cfg.settings().w_current, 
                                cfg.settings().w_individual, cfg.settings().w_social,
                                cfg.settings().multi_obj, cfg.settings().fitness_weights));        
        population.push_back(p);
        opposition_set.push_back(p);
        
        //insert the opposite as well
        /*shared_ptr<Particle> tmp_p(new Particle(*p));        
        tmp_p->opposite();
        population.push_back(tmp_p);        */
        
    }   
    LOG_INFO("Initializing the swarm"); 
    //THROW_EXCEPTION(RuntimeException, "intit finished" );
}
Swarm::~Swarm()
{}

std::ostream& operator<< (std::ostream &out, const Swarm &swarm)
{
    out << "no particles=" << swarm.no_individulas << endl;
    return out;
}


void Swarm::update(int t_id)
{
    int start_id = t_id * individual_per_thread;
    int end_id = start_id + individual_per_thread;
    if(t_id == no_threads -1)///Last thread takes care of all remaining particles
        end_id = no_individulas;
    for(int i=start_id;i<end_id;i++)    
    {
        if(cfg.settings().multi_obj && !par_f.pareto.empty())
        {
            int par_indx = random_indx(par_f.pareto.size()-1);        
            population[i]->set_best_global(par_f.pareto[par_indx]);
            if(stagnation)
            {
                population[i]->avoid_stagnation();                  
            }
            population[i]->update();
        }
        if(!cfg.settings().multi_obj && !short_term_memory.empty())
        {
            int mem_indx = random_indx(short_term_memory.mem.size()-1);        
            population[i]->set_best_global(short_term_memory.mem[mem_indx]);
            if(stagnation)
            {
                population[i]->avoid_stagnation();                         
            }
            population[i]->update();
        }
        /*
        /// copy the particle
        shared_ptr<Particle> tmp_p(new Particle(*population[i]));        
        tmp_p->opposite();
        opposition_set[i] = tmp_p;*/
    }
    
}

void Swarm::evaluate_oppositions()
{
    vector<shared_ptr<Particle>> opposition_set;
    for(auto p :population)
    {
        /// copy the particle
        shared_ptr<Particle> tmp_p(new Particle(*p));        
        tmp_p->opposite();
        opposition_set.push_back(tmp_p);
    }
    /*
    for(auto p: population)
        cout << p->get_current_position().fitness_func() << endl;
    cout << "size=" << population.size() << endl;    
    THROW_EXCEPTION(RuntimeException, "sort completed");    */
}
void Swarm::merge_main_opposite()
{
    population.insert(population.end(), opposition_set.begin(), opposition_set.end());
    std::sort(population.begin(), population.end(),
                    [](shared_ptr<Particle> const a, shared_ptr<Particle> const b) -> bool 
                    { return a->dominate(b); } );
    population.erase (population.begin()+no_individulas,population.end());   
}

float Swarm::average_speed()
{
    vector<float> p_speeds;
    for(auto p : population)
        p_speeds.push_back(p->get_speed().average());
    if(p_speeds.empty())
        return 1; 
    return Speed::average<float>(p_speeds);    
}
int Swarm::no_converged_particles()
{
    int cnt = 0;
    for(auto p : population)
    {
        if(p->get_speed().average() == 0)
        {
            cnt++;
        }
    }
    return cnt;
}
void Swarm::replace_converged_particles()
{
    /// copy the particle
    cout << "replace_converged_particles\n";
    for(auto p : population)
    {
        if(p->get_speed().average() == 0 || p->get_current_position().fitness_func() == short_term_memory.mem[0].fitness_func())
        {    
            shared_ptr<Particle> tmp_p(new Particle(*p));        
            tmp_p->opposite();
            /*cout << "p:" << *p << endl;
            p = tmp_p;
            cout << "new_p:" << *p << endl;
            THROW_EXCEPTION(RuntimeException, "replaced" );*/
        }
    }
}
bool Swarm::termination()
{
    current_generation++;
    return (current_generation - last_update < no_generations);
}
bool Swarm::is_converged()
{
    current_generation++;
    return (no_converged_particles() > (int)no_individulas/4);
}
void Swarm::print_results()
{
    //dynamic_cast<DerivedType*>(&m_baseType);
    string sep="";         
   for(size_t i=0;i<100;i++)
       sep+="=";
    for(auto p : par_f.pareto)
       out << p << endl << sep << endl;
   for(auto p : population)
        out << "position:\n" << *p << endl << sep << endl;    
}
