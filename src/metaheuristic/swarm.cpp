#include "swarm.hpp"
Swarm::Swarm(shared_ptr<Mapping> _mapping, shared_ptr<Applications> _application):
                    mapping(_mapping),
                    applications(_application),
                    no_objectives(mapping->getNumberOfApps()+1),
                    no_particles(20),
                    no_generations(100),
                    pareto(no_objectives)                    
{   
    for(size_t i=0;i<no_particles;i++)
    {
        shared_ptr<Particle> p(new Particle(mapping, applications));
        particle_set.push_back(p);
    }    
}
Swarm::~Swarm()
{
    particle_set.clear();
}
void Swarm::search()
{
    for(size_t g=0;g<no_generations;g++)
    {
        LOG_INFO("Generation "+tools::toString(g));
        for(auto p: particle_set)
        {
            p->calc_fitness();
        }
        LOG_DEBUG("finding the best particle ");
        ///Find the best particle
        for(auto p: particle_set)
        {
            pareto.update_pareto(p->get_current_position());
        }
        cout << "pareto front--------------------------\n"
             << pareto << endl;  
        
        if(g+1 < no_generations)        
        {
            LOG_DEBUG("updating positions ");
            /// update the positions
            for(auto p: particle_set)
            {
                int obj = random_obj();
                if(!pareto.pareto[obj].empty())
                {
                    p->set_best_global(pareto.pareto[obj]);
                    p->update_position();
                }
            }
        }        
    }    
}
std::ostream& operator<< (std::ostream &out, const Swarm &swarm)
{
    out << "no particles=" << swarm.no_particles << endl;
    return out;
}
int Swarm::random_obj()
{
    random_device rnd_device;
    uniform_int_distribution<int> dist(0, no_objectives-1);
    mt19937 mersenne_engine(rnd_device());  
    auto gen = std::bind(dist, mersenne_engine);
    int obj = gen();
    return obj;    
}
ParetoFront::ParetoFront(int no_obj)
{
    for(int i=0;i<no_obj;i++)
        pareto.push_back(Position());
}
std::ostream& operator<< (std::ostream &out, const ParetoFront &p)
{
    for(auto po : p.pareto)
        out << Particle::print_vector(out, po.fitness) << endl;
    return out;    
}
bool ParetoFront::dominate(Position& p, int obj)
{
    if(p.empty() || p.fitness[obj] <= 0)
        return false;
        
    if(pareto[obj].empty() && !p.empty() && p.fitness[obj] > 0)
        return true;    
    
    if(pareto[obj].fitness[obj] > p.fitness[obj])
        return true;
    else
        return false;        
}
void ParetoFront::update_pareto(Position p)
{
    for(size_t obj=0;obj<pareto.size();obj++)
    {
        if(dominate(p, obj))
        {
            pareto[obj] = p;
        }
    }
}

