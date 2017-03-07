#include "chromosome.hpp"
Chromosome::Chromosome(shared_ptr<Mapping> _mapping, shared_ptr<Applications> _application, 
                    bool _multi_obj, vector<float> _o_w, vector<int> _penalty):
                    Individual(_mapping, _application, _multi_obj, _o_w, _penalty)
                    {}
                    
Chromosome::Chromosome(const Chromosome& _c):
                    Individual(_c){}
                    
void Chromosome::crossover()
{
    if(best_global_position.empty())
        THROW_EXCEPTION(RuntimeException, "best_global_position is empty!" );     
    Position new_pos(current_position.multi_obj, current_position.weights);
    new_pos.proc_mappings = crossover(current_position.proc_mappings, 
                    best_global_position.proc_mappings);
    new_pos.proc_modes = crossover(current_position.proc_modes, 
                    best_global_position.proc_modes);
    new_pos.tdmaAlloc = crossover(current_position.tdmaAlloc, 
                    best_global_position.tdmaAlloc);         
    
    build_schedules(new_pos);      
    
    new_pos.proc_sched = crossover(new_pos.proc_sched, 
                                    current_position.proc_sched, current_position.proc_mappings,
                                    best_global_position.proc_sched, best_global_position.proc_mappings, no_actors);

    vector<int> curr_src_mappings(no_channels, 0);
    vector<int> bg_src_mappings(no_channels, 0);
    for(size_t i=0;i<no_channels;i++)
    {
        int src = applications->getChannel(i)->source;
        curr_src_mappings[i] = current_position.proc_mappings[src];
        bg_src_mappings[i] = best_global_position.proc_mappings[src];
    }
         
    new_pos.send_sched = crossover(new_pos.send_sched, 
                                        current_position.send_sched, curr_src_mappings,
                                        best_global_position.send_sched, bg_src_mappings, no_channels);                                        

    vector<int> curr_dst_mappings(no_channels, 0);
    vector<int> bg_dst_mappings(no_channels, 0);    
    for(size_t i=0;i<no_channels;i++)
    {
        int dst = applications->getChannel(i)->destination;
        curr_dst_mappings[i] = current_position.proc_mappings[dst];
        bg_dst_mappings[i] = best_global_position.proc_mappings[dst];
    }
    new_pos.rec_sched = crossover(new_pos.rec_sched, 
                                        current_position.rec_sched, curr_dst_mappings,
                                        best_global_position.rec_sched, bg_dst_mappings, no_channels);                                        
    
    
    repair(new_pos);
    current_position = new_pos;                                 
                                        
}
void Chromosome::cross_mut()
{
    if(best_global_position.empty())
        THROW_EXCEPTION(RuntimeException, "best_global_position is empty!" );     
    Position new_pos(current_position.multi_obj, current_position.weights);
    ///#- mappings
    new_pos.proc_mappings = crossover(current_position.proc_mappings, 
                    best_global_position.proc_mappings);
    new_pos.proc_mappings = mutation(new_pos.proc_mappings);
    ///#- modes
    new_pos.proc_modes = crossover(current_position.proc_modes, 
                    best_global_position.proc_modes);
    new_pos.proc_modes = mutation(new_pos.proc_modes);
    ///#- tdma                
    new_pos.tdmaAlloc = crossover(current_position.tdmaAlloc, 
                    best_global_position.tdmaAlloc);         
    new_pos.tdmaAlloc = mutation(new_pos.tdmaAlloc);                
    ///#- build schedules
    repair(new_pos);
    build_schedules(new_pos);      
    ///#- proc schedules
    new_pos.proc_sched = crossover(new_pos.proc_sched, 
                                    current_position.proc_sched, current_position.proc_mappings,
                                    best_global_position.proc_sched, best_global_position.proc_mappings, no_actors);

    vector<int> curr_src_mappings(no_channels, 0);
    vector<int> bg_src_mappings(no_channels, 0);
    for(size_t i=0;i<no_channels;i++)
    {
        int src = applications->getChannel(i)->source;
        curr_src_mappings[i] = current_position.proc_mappings[src];
        bg_src_mappings[i] = best_global_position.proc_mappings[src];
    }
    ///#- send schedules         
    new_pos.send_sched = crossover(new_pos.send_sched, 
                                        current_position.send_sched, curr_src_mappings,
                                        best_global_position.send_sched, bg_src_mappings, no_channels);                                        

    vector<int> curr_dst_mappings(no_channels, 0);
    vector<int> bg_dst_mappings(no_channels, 0);    
    for(size_t i=0;i<no_channels;i++)
    {
        int dst = applications->getChannel(i)->destination;
        curr_dst_mappings[i] = current_position.proc_mappings[dst];
        bg_dst_mappings[i] = best_global_position.proc_mappings[dst];
    }
    ///#- recive schedules
    new_pos.rec_sched = crossover(new_pos.rec_sched, 
                                        current_position.rec_sched, curr_dst_mappings,
                                        best_global_position.rec_sched, bg_dst_mappings, no_channels);                                        
    
    
    ///#- mutation for scehdules
    for(size_t proc=0;proc<no_processors;proc++)
    {
        new_pos.proc_sched[proc].set_rank(mutation(new_pos.proc_sched[proc].get_rank()));
        new_pos.send_sched[proc].set_rank(mutation(new_pos.send_sched[proc].get_rank()));
        new_pos.rec_sched[proc].set_rank(mutation(new_pos.rec_sched[proc].get_rank()));        
    }
    
    repair(new_pos);
    current_position = new_pos;                                 
                                        
}
vector<int> Chromosome::crossover(vector<int> v1, vector<int> v2)
{
    size_t cr_point = random::random_indx(v1.size()-1);
    vector<int> result;
    for(size_t i=0;i<cr_point;i++)
    {
        result.push_back(v1[i]);
    }
    for(size_t i=cr_point;i<v1.size();i++)
    {
        result.push_back(v2[i]);
    }
    return result;
}

vector<int> Chromosome::mutation(vector<int> v)
{
    ///#- randomly ignor mutation
    if(random::random_bool() || v.empty())
        return v;
        
    size_t mut_point = random::random_indx(v.size()-1);
    int mut_val = random::random_int(-2,2);
    v[mut_point] += mut_val;
    
    return v;
}
vector<Schedule> Chromosome::crossover(vector<Schedule> new_s, vector<Schedule> s1, 
                            vector<int> map1, vector<Schedule> s2, vector<int> map2, int no_elems)
{
    int cr_point = random::random_indx(no_elems-1);
    
    for(size_t proc=0;proc<new_s.size();proc++)
    {
        for(size_t i=0;i<new_s[proc].size();i++)
        {
            int elem = new_s[proc].get_elements()[i];
            if(elem < cr_point)
                new_s[proc].set_rank_by_element(elem, s1[map1[elem]].get_rank_by_element(elem));
            else
                new_s[proc].set_rank_by_element(elem, s2[map2[elem]].get_rank_by_element(elem));    
        }
    }
    return new_s;
}

void Chromosome::update()
{
    cross_mut();
}
