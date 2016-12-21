#include "swarm.hpp"
Swarm::Swarm(shared_ptr<Mapping> _mapping, shared_ptr<Applications> _application):
                    mapping(_mapping),
                    applications(_application),
                    no_particles(30),
                    no_generations(50)
{   
    for(size_t i=0;i<no_particles;i++)
    {
        shared_ptr<Particle> p(new Particle(mapping, applications));
        particle_set.push_back(p);
    }
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
            if(is_particle_better_gb(p->get_current_position()))
            {
                best_position = p->get_current_position();
            }
        }
        cout << "best fitness: ";
        for(auto f : best_position.fitness)
            cout << f << " ";
        cout << endl;  
        
        if(g+1 < no_generations)        
        {
            LOG_DEBUG("updating positions ");
            /// update the positions
            for(auto p: particle_set)
            {
                p->set_best_global(best_position);
                p->update_position();
            }
        }        
    }
    
    cout << "best position:" << best_position << endl;
   
    cout << endl;    
}
std::ostream& operator<< (std::ostream &out, const Swarm &swarm)
{
    out << "no particles=" << swarm.no_particles << endl;
    return out;
}
bool Swarm::is_particle_better_gb(Position p)
{

    vector<int> fitness = p.fitness;
    if(best_position.empty() && fitness[0] > 0)
        {return true;}
    if(best_position.empty() && fitness[0] < 0)
        return false;
        
    vector<int> best_fitness = best_position.fitness;
        
    if(fitness[0] < 0) ///There is a deadlock
        return false;
    
    bool better_pr = true;
    bool better_eng = true;
    for(int i=0;i<mapping->getNumberOfApps();i++)
    {
        if(fitness[i] > best_fitness[i])
        {
            better_pr = false;
            break;
        }
    }
    if(fitness[fitness.size()-1] > best_fitness[best_fitness.size()-1])
    {
        //better_eng = false;
    }
    if(better_pr && better_eng)
        {return true;}
    else    
        return false;
}
