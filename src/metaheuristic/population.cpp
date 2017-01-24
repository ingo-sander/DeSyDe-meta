#include "population.hpp"
#include "plot.cpp"
Population::Population(shared_ptr<Mapping> _mapping, shared_ptr<Applications> _application, Config& _cfg):
                    cfg(_cfg),
                    mapping(_mapping),
                    applications(_application),
                    no_objectives(mapping->getNumberOfApps()+2),
                    no_individulas(no_objectives*cfg.settings().particle_per_obj),
                    no_generations(cfg.settings().generation),
                    no_threads(std::thread::hardware_concurrency()),
                    par_f(),
                    stagnation(false)
{   
    individual_per_thread = no_individulas / no_threads;
    no_reinits = 0;
}
Population::~Population()
{
    population.clear();
}

void Population::search()
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
    std::thread t[no_individulas];
    
    size_t g = 0;
    while(!termination())
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

int Population::random_indx(int max)
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

void Population::calc_fitness(int t_id)
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
void Population::print()
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

