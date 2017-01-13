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
}
Swarm::~Swarm()
{
    particle_set.clear();
}
void Swarm::init()
{
    particle_set.clear();
    for(size_t i=0;i<no_particles;i++)
    {
        shared_ptr<Particle> p(new Particle(mapping, applications, i%no_objectives,
                                cfg.settings().w_current, 
                                cfg.settings().w_individual, cfg.settings().w_social,
                                cfg.settings().multi_obj, cfg.settings().fitness_weights));        
        particle_set.push_back(p);
        LOG_INFO("Initializing the swarm");
    }    
}
void Swarm::search()
{
    out.open(cfg.settings().output_path+"out/out_meta.txt");
    ofstream out_csv, out_tex;
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
    {
        LOG_INFO("Generation "+tools::toString(g)+" last_update "+
                  tools::toString(last_update) + " pareto size: "+
                  tools::toString(par_f.pareto.size()) +
                  " memory size: "+
                  tools::toString(memory.mem.size())
                  );
        if(par_f.empty() && memory.empty())
            init();
        /*if(stagnation)    
            stagnation = false;        
        if(g - last_update > no_generations/2)
            stagnation = true;
        else
            stagnation = false;        
            */ 
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
            if(cfg.settings().multi_obj)
            {
                if(par_f.update_pareto(p->get_current_position()))
                    last_update = g;
            }
            else
            {
                if(memory.update_memory(p->get_current_position()))
                    last_update = g;
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
        out << par_f << sep << endl;   
   else
        out << memory << sep << endl;       
   for(auto p : par_f.pareto)
       out << p << endl << sep << endl;
   for(auto p : particle_set)
        out << "position:\n" << *p << endl << sep << endl;
   out.close();
   
   vector<vector<int>> data;
   for(auto p : par_f.pareto)
   {
       vector<int> tmp;
       tmp.push_back(p.fitness_func());
       tmp.insert(tmp.end(), p.fitness.begin(), p.fitness.end());
       data.push_back(tmp);
   }
   for(auto p : memory.mem)
   {
       vector<int> tmp;
       tmp.push_back(p.fitness_func());
       tmp.insert(tmp.end(), p.fitness.begin(), p.fitness.end());
       data.push_back(tmp);
   }
   std::sort (data.begin(), data.end()); 
   vector<string> titles;
   titles.push_back("fitness");    
   for(int i=0;i<mapping->getNumberOfApps();i++)
       titles.push_back("app"+tools::toString(i)+"-period");
   titles.push_back("energy");    
   titles.push_back("memory");
   Plot pl(titles, data);    

   
   out_csv << pl.csv;
   out_csv.close();
   
   out_tex << pl.tex;
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
        if(!cfg.settings().multi_obj && !memory.empty())
        {
            int mem_indx = random_indx(memory.mem.size()-1);        
            particle_set[i]->set_best_global(memory.mem[mem_indx]);
            if(stagnation)
            {
                particle_set[i]->avoid_stagnation();                
            }
            particle_set[i]->update_position();
        }
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
            << endl;
    return out;    
}
