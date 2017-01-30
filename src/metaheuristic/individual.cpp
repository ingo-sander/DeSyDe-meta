#include "individual.hpp"
Individual::Individual(shared_ptr<Mapping> _mapping, shared_ptr<Applications> _application, 
                    bool _multi_obj, vector<float> _o_w):
                    mapping(_mapping),
                    applications(_application),
                    no_entities(mapping->getNumberOfApps()),
                    no_actors(applications->n_SDFActors()),
                    no_channels(applications->n_SDFchannels()),
                    no_processors(mapping->getPlatform()->nodes()),
                    no_tdma_slots(mapping->getPlatform()->tdmaSlots()),
                    current_position(_multi_obj, _o_w),
                    best_global_position(_multi_obj, _o_w),
                    no_invalid_moves(0),
                    multi_obj(_multi_obj),
                    obj_weights(_o_w)                                        
{   
    if(obj_weights.size() != no_entities + 2)
        THROW_EXCEPTION(RuntimeException, tools::toString(no_entities + 2) +
                        " obj_weights needed while " + tools::toString(obj_weights.size()) +
                        " provided");
    init_random();
}
Individual::Individual(const Individual& _p):
                    mapping(_p.mapping),
                    applications(_p.applications),
                    no_entities(mapping->getNumberOfApps()),
                    no_actors(applications->n_SDFActors()),
                    no_channels(applications->n_SDFchannels()),
                    no_processors(mapping->getPlatform()->nodes()),
                    no_tdma_slots(mapping->getPlatform()->tdmaSlots()),
                    current_position(_p.current_position),
                    best_global_position(_p.best_global_position),
                    no_invalid_moves(0),
                    multi_obj(_p.multi_obj),
                    obj_weights(_p.obj_weights)
{}
void Individual::build_schedules(Position& p)
{
    p.proc_sched.clear();
    p.send_sched.clear();
    p.rec_sched.clear();
    for(size_t i=0;i<no_processors;i++)
    {
        p.proc_sched.push_back(Schedule(p.get_actors_by_proc(i), i+no_actors));
        
        p.send_sched.push_back(Schedule(get_channel_by_src(p, i), i+no_channels));
        
        p.rec_sched.push_back(Schedule(get_channel_by_dst(p, i), i+no_channels));
    }
}
void Individual::repair(Position &p)
{
    p.proc_mappings = tools::bring_v_to_bound(p.proc_mappings, 0, (int)no_processors-1);
     for(size_t i=0;i<no_processors;i++)
    {
       int no_proc_modes = mapping->getPlatform()->getModes(i);
       p.proc_modes[i] = tools::bring_to_bound(p.proc_modes[i], 0, no_proc_modes-1);
    }
    repair_tdma(p);    
    repair_sched(p);
    repair_send_sched(p);
    repair_rec_sched(p);
}
void Individual::repair_sched(Position& p)
{
    for(size_t proc=0;proc<p.proc_sched.size();proc++)
    {
        p.proc_sched[proc].set_rank( tools::bring_v_to_bound(p.proc_sched[proc].get_rank(), 0, (int)p.proc_sched[proc].get_rank().size()-1) );        
        for(size_t i=0;i<p.proc_sched[proc].get_elements().size();i++)
        {
            int a = p.proc_sched[proc].get_elements()[i];
            for(size_t j=i;j<p.proc_sched[proc].get_elements().size();j++)
            {                
                int b = p.proc_sched[proc].get_elements()[j];
                int rank_a = p.proc_sched[proc].get_rank_by_id(i);
                int rank_b = p.proc_sched[proc].get_rank_by_id(j);
                /**
                 * if a and b are not dummy and the dependency is violated
                 */ 
                if((a < (int) no_actors && b < (int) no_actors) && rank_a > rank_b && applications->dependsOn(a, b))
                {
                    if(random::random_bool())
                        p.proc_sched[proc].switch_ranks(i, j);
                }
            }
        }
    }
    
}
void Individual::repair_send_sched(Position& p)
{
    for(size_t proc=0;proc<p.send_sched.size();proc++)
    {
        p.send_sched[proc].set_rank( tools::bring_v_to_bound(p.send_sched[proc].get_rank(), 0, (int)p.send_sched[proc].get_rank().size()-1) );
        for(size_t i=0;i<p.send_sched[proc].get_elements().size();i++)
        {
            int a = p.send_sched[proc].get_elements()[i];
            for(size_t j=0;j<p.send_sched[proc].get_elements().size();j++)
            {                
                int b = p.send_sched[proc].get_elements()[j];
                if(a < (int) no_channels && b < (int) no_channels)
                {
                    int dst_b = applications->getChannel(b)->destination;
                    int dst_a = applications->getChannel(a)->destination;
                    int src_b = applications->getChannel(b)->source;
                    int src_a = applications->getChannel(a)->source;
                    int rank_a = p.send_sched[proc].get_rank_by_id(i);
                    int rank_b = p.send_sched[proc].get_rank_by_id(j);
                    int rank_src_a = p.proc_sched[p.proc_mappings[src_a]].get_rank_by_element(src_a);
                    int rank_src_b = p.proc_sched[p.proc_mappings[src_b]].get_rank_by_element(src_b);
                    /**
                     * if a and b are sending to the same destination
                     * and the dependency is violated
                     * OR 
                     * if their source is on the same proc and 
                     * rank_src_a < rank_src_b and rank_a > rank_b
                     */ 
                    if(
                      (dst_a == dst_b
                        && rank_a < rank_b && applications->dependsOn(dst_a, dst_b))
                       ||
                       (rank_src_a < rank_src_b && rank_a > rank_b) 
                      )
                    {
                        if(random::random_bool())
                            p.send_sched[proc].switch_ranks(i, j);                        
                    }
                }
            }
        }
    }
    
}
void Individual::repair_rec_sched(Position& p)
{
    /**
     * if the source of a and b are on the same proc
     * and rank_src_a < rank_src_b and rank_a > rank_b then switch
     */ 
    for(size_t proc=0;proc<p.rec_sched.size();proc++)
    {
        p.rec_sched[proc].set_rank( tools::bring_v_to_bound(p.rec_sched[proc].get_rank(), 0, (int)p.rec_sched[proc].get_rank().size()-1) );
        for(size_t i=0;i<p.rec_sched[proc].get_elements().size();i++)
        {
            int a = p.rec_sched[proc].get_elements()[i];
            for(size_t j=0;j<p.rec_sched[proc].get_elements().size();j++)
            {                
                int b = p.rec_sched[proc].get_elements()[j];
                if(a < (int) no_channels && b < (int) no_channels)
                {
                    int src_b = applications->getChannel(b)->source;
                    int src_a = applications->getChannel(a)->source;
                    if(p.proc_mappings[src_a] == p.proc_mappings[src_b])
                    {
                        int rank_src_a = p.send_sched[p.proc_mappings[src_a]].get_rank_by_element(a);
                        int rank_src_b = p.send_sched[p.proc_mappings[src_b]].get_rank_by_element(b);       
                        int rank_a = p.rec_sched[proc].get_rank_by_id(i);
                        int rank_b = p.rec_sched[proc].get_rank_by_id(j);            
                    
                        if(rank_src_a < rank_src_b && rank_a > rank_b)
                        {
                            if(random::random_bool())
                                p.rec_sched[proc].switch_ranks(i, j);
                        }
                    }
                }
            }
        }
    }
    
}

vector<int> Individual::get_channel_by_src(Position& p, int src_proc_id) const
{
    vector<int> channels;
    for(size_t i=0;i<no_channels;i++)
    {
        if(p.proc_mappings[applications->getChannel(i)->source] == src_proc_id)
            channels.push_back(i);
    }
    return channels;
}
vector<int> Individual::get_channel_by_dst(Position& p, int dst_proc_id) const
{
    vector<int> channels;
    for(size_t i=0;i<no_channels;i++)
    {
        if(p.proc_mappings[applications->getChannel(i)->destination] == dst_proc_id)
            channels.push_back(i);
    }
    return channels;
}
void Individual::init_random()
{
    /**  -# Clear vectors. */
    current_position.proc_mappings.clear();
    current_position.proc_modes.clear();
    current_position.tdmaAlloc.clear();
    current_position.fitness.clear();
    current_position.proc_sched.clear();
    current_position.send_sched.clear();
    current_position.rec_sched.clear();
    no_invalid_moves = 0;
    
    random_device rnd_device;
    mt19937 mersenne_engine(rnd_device());
    uniform_int_distribution<int> dist_proc(0, no_processors-1);
    uniform_int_distribution<int> dist_actor(0, no_actors-1);
    uniform_int_distribution<int> dist_channel(0, no_channels-1);
    uniform_int_distribution<int> dist_tdma(0, no_tdma_slots);

    /// -# The engine has to be reset after crreating the distribution
    auto gen_proc = std::bind(dist_proc, mersenne_engine);mersenne_engine();
    auto gen_tdma = std::bind(dist_proc, mersenne_engine);mersenne_engine();
    
    current_position.proc_mappings.resize(no_actors, 0);
    current_position.proc_modes.resize(no_processors, 0);
    current_position.tdmaAlloc.resize(no_processors, 0);
    
    generate(begin(current_position.proc_mappings), end(current_position.proc_mappings), gen_proc); 
    generate(begin(current_position.tdmaAlloc), end(current_position.tdmaAlloc), gen_tdma);
    
    /// -# Random proc_modes based on the number of modes for each processors
    for(size_t i=0;i<no_processors;i++)
    {
       int no_proc_modes = mapping->getPlatform()->getModes(i);
       std::uniform_int_distribution<int> uni_dist(0,no_proc_modes-1);
       current_position.proc_modes[i] = uni_dist(mersenne_engine);       
    }
        
    build_schedules(current_position);
    current_position.fitness.resize(no_entities + 2,0);///energy + memory violations + throughputs    
    repair(current_position);         
}
void Individual::repair_tdma(Position& p)
{
    p.tdmaAlloc = tools::bring_v_to_bound(p.tdmaAlloc, 0, (int)no_tdma_slots);
    vector<int> no_inout_channels(no_processors, 0);
    ///Random # tdma slots based on src and dst of channels
    for(size_t i=0;i<no_channels;i++)
    {
        int src_i = applications->getChannel(i)->source;
        int dest_i = applications->getChannel(i)->destination;
        int proc_src_i = p.proc_mappings[src_i];
        int proc_dest_i = p.proc_mappings[dest_i];    
        if(proc_src_i != proc_dest_i)
        {
            no_inout_channels[proc_src_i]++;
            no_inout_channels[proc_dest_i]++;
        }      
    }
    for(size_t i=0;i<no_inout_channels.size();i++)
    {
        if(no_inout_channels[i] == 0)
            p.tdmaAlloc[i] = 0;
        else
        {
            if(p.tdmaAlloc[i] == 0)
            {
                p.tdmaAlloc[i]++;
            }
        }    
    }
    /**
     * \note If the total number of allocated slots is more than the number of 
     * available slots, then take the difference away from some processors.
     */     
    int sum_of_elems = std::accumulate(p.tdmaAlloc.begin(), p.tdmaAlloc.end(), 0);
    int diff = sum_of_elems - no_tdma_slots;
    while(diff > 0)
    {
        for(size_t i=0;i<p.tdmaAlloc.size();i++)
        {
            if(p.tdmaAlloc[i] > 1)
            {
                p.tdmaAlloc[i]--;
                diff--;
                if(diff <= 0)
                    break;
            }
        }
    }
}
vector<int> Individual::get_next(vector<Schedule> sched_set, int no_elements)
{
    vector<int> next(no_elements+no_processors, 0);
    vector<int> low_ranks;
    for (auto s : sched_set)
    {
        for(auto e: s.get_elements())
        {
            next[e] = s.get_next(e);
        }
        low_ranks.push_back(s.get_element_by_rank(0));                    
    }
    ///\note Last dummy node should point to the highest rank of the first proc.
    next[no_elements+no_processors-1] = low_ranks[0];
    for(size_t i=0;i<low_ranks.size()-1;i++)
    {
        next[no_elements+i] = low_ranks[i+1];
    }
    return next;
}
void Individual::calc_fitness()
{   
    try{ 
        Design design(mapping, applications, current_position.proc_mappings, 
                      current_position.proc_modes, get_next(current_position.proc_sched, no_actors),
                      get_next(current_position.send_sched, no_channels),
                      get_next(current_position.rec_sched, no_channels), 
                      current_position.tdmaAlloc);
        
        vector<int> prs = design.get_periods();
      

        current_position.fitness.clear();
        current_position.fitness.resize(no_entities + 2,0);
        int eng = design.get_energy();
        for(size_t i=0;i< prs.size();i++)
            current_position.fitness[i] = prs[i];
            
        current_position.fitness[current_position.fitness.size()-2] = eng;
        int no_mem_violations = 0;
        for(auto m : design.get_slack_memory())
            if(m < 0)
                no_mem_violations++;
                    
        current_position.fitness[current_position.fitness.size()-1] = no_mem_violations;        
    }
    catch(std::exception const& e)
    {
        THROW_EXCEPTION(RuntimeException, e.what() );
    }
    if(current_position.fitness[0] <= 0)
    {
        no_invalid_moves++;
    }
    else
        no_invalid_moves = 0;        
    
}
vector<int> Individual::get_fitness()
{
    return current_position.fitness;
}
Position Individual::get_current_position() const
{
    return current_position;
}

float Individual::random_weight()
{
    random_device rnd_device;
    uniform_int_distribution<int> dist(0, 100);
    mt19937 mersenne_engine(rnd_device());  
    auto gen = std::bind(dist, mersenne_engine);
    float w = ((float)gen())/100;
    return w;    
}
bool Individual::dominate(const shared_ptr<Individual> in_p)const
{
    Position new_pos = in_p->get_current_position();
    return current_position.dominate(new_pos);
}
void Individual::opposite()
{
    current_position.opposite();
    build_schedules(current_position);
    repair(current_position);       
}
void Individual::set_best_global(Position p)
{
    best_global_position = p; 
}
std::ostream& operator<< (std::ostream &out, const Individual &ind)
{
    out << "current position: ====================\n" << ind.current_position << endl;
    out << "best g position: ====================\n" << ind.best_global_position << endl;
    return out;
}
