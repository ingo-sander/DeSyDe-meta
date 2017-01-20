#include "swarm.hpp"
#include "plot.cpp"
Swarm::Swarm(shared_ptr<Mapping> _mapping, shared_ptr<Applications> _application, Config& _cfg):
                    cfg(_cfg),
                    mapping(_mapping),
                    applications(_application),
                    no_objectives(mapping->getNumberOfApps()+2),
                    no_particles(no_objectives*cfg.settings().particle_per_obj),
                    no_generations(cfg.settings().generation),
                    no_threads(std::thread::hardware_concurrency()),
                    par_f(),
                    stagnation(false)
{   
    particle_per_thread = no_particles / no_threads;
    no_reinits = 0;
}
Swarm::~Swarm()
{
    particle_set.clear();
}
void Swarm::init()
{
    no_reinits++;
    particle_set.clear();
    opposition_set.clear();
    for(size_t i=0;i<no_particles;i++)
    {
        shared_ptr<Particle> p(new Particle(mapping, applications, i%no_objectives,
                                cfg.settings().w_current, 
                                cfg.settings().w_individual, cfg.settings().w_social,
                                cfg.settings().multi_obj, cfg.settings().fitness_weights));        
        particle_set.push_back(p);
        opposition_set.push_back(p);
        
        //insert the opposite as well
        /*shared_ptr<Particle> tmp_p(new Particle(*p));        
        tmp_p->opposite();
        particle_set.push_back(tmp_p);        */
        
    }   
    LOG_INFO("Initializing the swarm"); 
    //THROW_EXCEPTION(RuntimeException, "intit finished" );
}
void Swarm::search()
{
    out.open(cfg.settings().output_path+"out/out_meta.txt");
    out_csv.open(cfg.settings().output_path+"out/data.csv");
    out_tex.open(cfg.settings().output_path+"out/plot.tex");
    string sep="";         
   for(size_t i=0;i<100;i++)
       sep+="=";
    auto last_update = 0;
    t_start = runTimer::now();
    auto dur_fitness = runTimer::now() - runTimer::now();
    auto dur_update = runTimer::now() - runTimer::now();
    std::thread t[no_particles];
    //for(size_t g=0;g<no_generations;g++)
    size_t g = 0;
    while(g - last_update < no_generations)
    //while(average_speed() != 0)
    {
        LOG_INFO("Generation "+tools::toString(g)+" last_update "+
                  tools::toString(last_update) + " pareto size: "+
                  tools::toString(par_f.pareto.size()) +
                  " memory size: "+
                  tools::toString(long_term_memory.mem.size()) +
                  " no converged:" +
                  tools::toString(no_converged_particles()) +
                  " out of:" +
                  tools::toString(no_particles) +
                  " no re-inits:" +
                  tools::toString(no_reinits)
                  );
         cout << "short_term_memory:" << short_term_memory << endl
              << "long_term_memory:" << long_term_memory << endl;              
        if(no_converged_particles() > (int)no_particles/4)
        {
            init();
            //replace_converged_particles();
            short_term_memory.mem.clear();
        }
        if(par_f.empty() && long_term_memory.empty())
            init();
        
        
             
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
        for(size_t p=0;p<particle_set.size();p++)
        {
            if(cfg.settings().multi_obj)
            {
                short_term_memory.update_memory(particle_set[p]->get_current_position());
                if(par_f.update_pareto(particle_set[p]->get_current_position()))
                    last_update = g;
            }
            else
            {
                short_term_memory.update_memory(particle_set[p]->get_current_position());
                if(long_term_memory.update_memory(particle_set[p]->get_current_position()))
                {
                    last_update = g;
                    long_term_memory.ins_time = runTimer::now() - t_start;
                    memory_hist.push_back(long_term_memory);
                }
            }
        }
        /*cout << "pareto front--------------------------\n"
             << par_f << endl;  */
            
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
   for(auto p : par_f.pareto)
       out << p << endl << sep << endl;
   for(auto p : particle_set)
        out << "position:\n" << *p << endl << sep << endl;
   out.close();
   
   print();
   out_csv.close();
   out_tex.close();
   
}
std::ostream& operator<< (std::ostream &out, const Swarm &swarm)
{
    out << "no particles=" << swarm.no_particles << endl;
    return out;
}
int Swarm::random_indx(int max)
{
    random_device rnd_device;
    uniform_int_distribution<int> dist(0, max);
    mt19937 mersenne_engine(rnd_device());  
    auto gen = std::bind(dist, mersenne_engine);
    int i = gen();
    return i;    
}
ParetoFront::ParetoFront()
{  
}
bool ParetoFront::empty()
{
    return pareto.empty();
}
std::ostream& operator<< (std::ostream &out, const ParetoFront &p)
{
    for(auto po : p.pareto)
        out << tools::toString(po.fitness_func()) << " " 
            << tools::toString(po.fitness) 
            << endl;
    return out;    
}
/**
 * Does pareto[indx] dominate p?
*/
bool ParetoFront::dominate(Position& p, int indx)
{
    if(p.empty())
        return true;
    for(auto f : p.fitness)    
        if(f < 0)
            return true;
        
    if(pareto.empty())    
        return false;
        
    //return dominate(pareto[indx], p);        
    return p.dominate(pareto[indx]);
} 
/**
 * Does p1 dominate p2?
 
bool ParetoFront::dominate(Position& p1, Position& p2)
{
    for(size_t i=0;i<p1.fitness.size();i++)
    {
        if(p1.fitness[i] > p2.fitness[i])
            return false;
    }
    return true;
} */
bool ParetoFront::dominate(Position& p)
{
    /// -# when pareto is empty directly call dominate for indx=0
    if(pareto.empty())
            return dominate(p, 0);
    for(size_t i=0;i<pareto.size();i++)
    {
        if(pareto[i].dominate(p) || pareto[i] == p)
        {
             return true;
        }
    }
    return false;
}
bool ParetoFront::update_pareto(Position p)
{
    bool is_updated = false;
    if(!dominate(p))
    {
        for(size_t i=0;i<pareto.size();i++)
        {
            if(p.dominate(pareto[i]))
            {
                 pareto.erase (pareto.begin()+i);
            }           
        }
        pareto.push_back(p);
        is_updated = true;
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
        if(cfg.settings().multi_obj && !par_f.pareto.empty())
        {
            int par_indx = random_indx(par_f.pareto.size()-1);        
            particle_set[i]->set_best_global(par_f.pareto[par_indx]);
            if(stagnation)
            {
                particle_set[i]->avoid_stagnation();                  
            }
            particle_set[i]->update_position();
        }
        if(!cfg.settings().multi_obj && !short_term_memory.empty())
        {
            int mem_indx = random_indx(short_term_memory.mem.size()-1);        
            particle_set[i]->set_best_global(short_term_memory.mem[mem_indx]);
            if(stagnation)
            {
                particle_set[i]->avoid_stagnation();                         
            }
            particle_set[i]->update_position();
        }
        /*
        /// copy the particle
        shared_ptr<Particle> tmp_p(new Particle(*particle_set[i]));        
        tmp_p->opposite();
        opposition_set[i] = tmp_p;*/
    }
    
}

bool Memory::empty()
{
    return mem.empty();
}
bool Memory::update_memory(Position new_p)
{
    if(mem.size() < max_size && !new_p.invalid())
    {
        mem.push_back(new_p);
        return true;
    }
    if(exists_in_mem(new_p))
        return false;
        
    bool added = false;
    for(auto p: mem)
    {
        if((new_p.dominate(p)))
        {
            mem.push_back(new_p);
            added = true;
            break;
        }
    }
    if(added)
       remove_worst();
    return added;   
}
bool Memory::exists_in_mem(Position& new_p)
{
    for(auto p: mem)
    {
        if(new_p == p)
            return true;
    }
    return false;
}
void Memory::remove_worst()
{
    int worst_i=0;
    for(size_t i=1;i<mem.size();i++)
    {
        if(mem[worst_i].dominate(mem[i]))
        {
            worst_i = i;            
        }
    }
    mem.erase(mem.begin()+worst_i);
}
std::ostream& operator<< (std::ostream &out, const Memory &m)
{
    for(auto po : m.mem)
        out << tools::toString(po.fitness_func()) << " " 
            << tools::toString(po.fitness) 
            << " updated at:" << std::chrono::duration_cast<std::chrono::seconds>(m.ins_time).count() << "s"
            << endl;
            
    return out;    
}
void Swarm::evaluate_oppositions()
{
    vector<shared_ptr<Particle>> opposition_set;
    for(auto p :particle_set)
    {
        /// copy the particle
        shared_ptr<Particle> tmp_p(new Particle(*p));        
        tmp_p->opposite();
        opposition_set.push_back(tmp_p);
    }
    /*
    for(auto p: particle_set)
        cout << p->get_current_position().fitness_func() << endl;
    cout << "size=" << particle_set.size() << endl;    
    THROW_EXCEPTION(RuntimeException, "sort completed");    */
}
void Swarm::merge_main_opposite()
{
    particle_set.insert(particle_set.end(), opposition_set.begin(), opposition_set.end());
    std::sort(particle_set.begin(), particle_set.end(),
                    [](shared_ptr<Particle> const a, shared_ptr<Particle> const b) -> bool 
                    { return a->dominate(b); } );
    particle_set.erase (particle_set.begin()+no_particles,particle_set.end());   
}
void Swarm::print()
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
float Swarm::average_speed()
{
    vector<float> p_speeds;
    for(auto p : particle_set)
        p_speeds.push_back(p->get_speed().average());
    if(p_speeds.empty())
        return 1; 
    return Speed::average<float>(p_speeds);    
}
int Swarm::no_converged_particles()
{
    int cnt = 0;
    for(auto p : particle_set)
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
    for(auto p : particle_set)
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
