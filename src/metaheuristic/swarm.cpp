#include "swarm.hpp"
Swarm::Swarm(shared_ptr<Mapping> _mapping, shared_ptr<Applications> _application, Config& _cfg):
                    cfg(_cfg),
                    mapping(_mapping),
                    applications(_application),
                    no_objectives(mapping->getNumberOfApps()+2),
                    no_particles(no_objectives*cfg.settings().particle_per_obj),
                    no_generations(cfg.settings().generation),
                    no_threads(std::thread::hardware_concurrency()),
                    par_f(no_objectives)
{   
    for(size_t i=0;i<no_particles;i++)
    {
        shared_ptr<Particle> p(new Particle(mapping, applications, i%no_objectives,
                                cfg.settings().w_current, 
                                cfg.settings().w_individual, cfg.settings().w_social));        
        particle_set.push_back(p);
    }    
    particle_per_thread = no_particles / no_threads;
}
Swarm::~Swarm()
{
    particle_set.clear();
}
void Swarm::search()
{
    out.open(cfg.settings().output_path+"out/out.txt");
    
    auto last_update = 0;
    t_start = runTimer::now();
    auto dur_fitness = runTimer::now() - runTimer::now();
    auto dur_update = runTimer::now() - runTimer::now();
    std::thread t[no_particles];
    //for(size_t g=0;g<no_generations;g++)
    size_t g = 0;
    while(g - last_update < no_generations)
    {
        LOG_INFO("Generation "+tools::toString(g)+" last_update "+tools::toString(last_update));
        auto start_fitness = runTimer::now();
        for (int i = 0; i < no_threads; i++) 
        {
            t[i] = std::thread(&Swarm::calc_fitness, this, i);            
            //p->calc_fitness();
        }
        
        //Join the threads 
        for (int i = 0; i < no_threads; i++) 
        {
             t[i].join();
        }
        
        dur_fitness += runTimer::now() - start_fitness;


        LOG_DEBUG("finding the best particle ");
        ///Find the best particle
        for(auto p: particle_set)
        {
            if(par_f.update_pareto(p->get_current_position()))
                last_update = g;
            //cout << tools::toString(p->get_current_position().fitness) << endl;
        }
        cout << "pareto front--------------------------\n"
             << par_f << endl;  
        auto start_update = runTimer::now();
        if(g+1- last_update < no_generations)        
        {
            LOG_DEBUG("updating positions ");
            /// update the positions
            for (int i = 0; i < no_threads; i++) 
            {
                t[i] = std::thread(&Swarm::update_position, this, i);            
            }
            //Join the threads 
            for (int i = 0; i < no_threads; i++) 
            {
                 t[i].join();
            }
        }   
        dur_update += runTimer::now() - start_update;     
        g++;
    }
    auto durAll = runTimer::now() - t_start;
    auto durAll_s = std::chrono::duration_cast<std::chrono::seconds>(durAll).count();
    auto dur_fitness_s = std::chrono::duration_cast<std::chrono::milliseconds>(dur_fitness).count();
    auto dur_update_s = std::chrono::duration_cast<std::chrono::milliseconds>(dur_update).count();
    auto durAll_ms = std::chrono::duration_cast<std::chrono::milliseconds>(durAll).count();
    std::stringstream stat;
    stat    << "===== search ended after: " << durAll_s << " s (" << durAll_ms 
            << " ms)\nfitness=" << dur_fitness_s << "ms update=" << dur_update_s << "ms \n"
            << "no threads=" << no_threads 
            << " last update in generation " << last_update
            << endl;
   cout << stat.str() << endl;
   out << stat.str() << endl;         
   string sep="";         
   for(size_t i=0;i<100;i++)
       sep+="=";
   out << sep << endl;    
   for(auto p : par_f.pareto)
       out << p << endl << sep << endl;
   for(auto p : particle_set)
        out << p->get_speed() << endl << sep << endl;
   out.close();
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
        out << tools::toString(po.fitness) << endl;
    return out;    
}
bool ParetoFront::dominate(Position& p, int obj)
{
    if(p.empty())
        return false;
    for(auto f : p.fitness)    
        if(f < 0)
            return false;
        
    if(pareto[obj].empty() && !p.empty() && p.fitness[obj] >= 0)
        return true;    
    if(pareto[obj].fitness[obj] > p.fitness[obj])
        return true;
    else
        return false;        
}
bool ParetoFront::update_pareto(Position p)
{
    bool is_updated = false;
    for(size_t obj=0;obj<pareto.size();obj++)
    {
        if(dominate(p, obj))
        {
            pareto[obj] = p;
            is_updated = true;
        }
    }
    return is_updated;
}

void Swarm::calc_fitness(int t_id)
{
    int start_id = t_id * particle_per_thread;
    int end_id = start_id + particle_per_thread;
    if(t_id == no_threads -1)///Last thread takes care of all remaining particles
        end_id = no_particles;
    for(int i=start_id;i<end_id;i++)    
    {
        particle_set[i]->calc_fitness();
    }
}
void Swarm::update_position(int t_id)
{
    int start_id = t_id * particle_per_thread;
    int end_id = start_id + particle_per_thread;
    if(t_id == no_threads -1)///Last thread takes care of all remaining particles
        end_id = no_particles;
        
    for(int i=start_id;i<end_id;i++)    
    {
        int obj = particle_set[i]->get_objective();//random_obj();
        if(!par_f.pareto[obj].empty())
        {
            particle_set[i]->set_best_global(par_f.pareto[obj]);
            particle_set[i]->update_position();
        }
    }
    
}
