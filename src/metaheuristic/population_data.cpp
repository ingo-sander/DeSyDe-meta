#include "population_data.hpp"
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
    vector<int> remove_indx={};
    if(!dominate(p))
    {
        for(size_t i=0;i<pareto.size();i++)
        {
            if(p.dominate(pareto[i]))
            {
                 remove_indx.push_back(i);
            }     
        }
        for(int i=remove_indx.size()-1;i>=0;i--)
        {
            int indx = remove_indx[i];            
            pareto.erase (pareto.begin()+indx);           
         }
        pareto.push_back(p);
        is_updated = true;
    }
    return is_updated;
}
void Memory::set_mem_size(int s)
{
    max_size = s;
}
Memory::Memory():
               last_update(0),
               max_size(1){}
bool Memory::empty()
{
    return mem.empty();
}
bool Memory::update_memory(Position new_p, std::chrono::duration<double>  _time)
{
    if(exists_in_mem(new_p))
        return false;
    
    if(mem.size() < max_size && !new_p.invalid())
    {
        mem.push_back(new_p);
        last_update = _time;
        return true;
    }
        
    bool added = false;
    for(auto p: mem)
    {
        if((new_p.dominate(p)))
        {
            mem.push_back(new_p);
            last_update = _time;
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
            << " updated at:" << std::chrono::duration_cast<std::chrono::seconds>(m.last_update).count() << "s"
            << endl;
            
    return out;    
}
