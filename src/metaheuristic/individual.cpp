#include "individual.hpp"
Individual::Individual(shared_ptr<Mapping> _mapping, shared_ptr<Applications> _application, 
                    bool _multi_obj, vector<float> _o_w, vector<int> _penalty):
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
                    obj_weights(_o_w),
                    penalty(_penalty)                                        
{   
    if(obj_weights.size() != no_entities + 1)
        THROW_EXCEPTION(RuntimeException, tools::toString(no_entities + 1) +
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
                    obj_weights(_p.obj_weights),
                    penalty(_p.penalty)
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
bool Individual::is_dep_sched_violation(Position &p, int proc, int a, int i, int b, int j)
{
    int rank_a = p.proc_sched[proc].get_rank_by_id(i);
    int rank_b = p.proc_sched[proc].get_rank_by_id(j);
    /**
     * if a and b are not dummy and the dependency is violated
     * or there's a channel from a to b with initial tokens
     */ 
    if(((a < (int) no_actors && b < (int) no_actors) && rank_a > rank_b && applications->dependsOn(a, b))
    || (applications->getTokensOnChannel(a, b) > 0))
        return true;
    
    return false;
}
bool Individual::is_dep_send_sched_violation(Position &p, int proc, int a, int i, int b, int j)
{
    if(a < (int) no_channels && b < (int) no_channels)
    {
        int dst_b = applications->getChannel(b)->destination;
        int dst_a = applications->getChannel(a)->destination;
        int src_b = applications->getChannel(b)->source;
        int src_a = applications->getChannel(a)->source;
        int rank_a = p.send_sched[proc].get_rank_by_id(i);
        int rank_b = p.send_sched[proc].get_rank_by_id(j);
        /**
         * if src_a and src_b are on the same proc,
         * then the order of sending should be same as order of actor firings
         */ 
        if(p.proc_mappings[src_a] == p.proc_mappings[src_b]) 
        {
            int rank_src_a = p.proc_sched[p.proc_mappings[src_a]].get_rank_by_element(src_a);
            int rank_src_b = p.proc_sched[p.proc_mappings[src_b]].get_rank_by_element(src_b);       
            
            if(rank_src_a < rank_src_b && rank_a > rank_b)
                return true;
            if(rank_src_a > rank_src_b && rank_a < rank_b)
                return true;    
        }
        /**
         * if a and b have same destination proccessor
         * and a is received first while we first send b
         * or vice versa
         */ 
        if(p.proc_mappings[dst_a] == p.proc_mappings[dst_b])
        {
            int rank_dst_a = p.rec_sched[p.proc_mappings[dst_a]].get_rank_by_element(a);
            int rank_dst_b = p.rec_sched[p.proc_mappings[dst_b]].get_rank_by_element(b);       
            int rank_a = p.send_sched[proc].get_rank_by_id(i);
            int rank_b = p.send_sched[proc].get_rank_by_id(j);            
            if((rank_dst_a < rank_dst_b && rank_a > rank_b)
            || (rank_dst_a > rank_dst_b && rank_a < rank_b))
            {
                return true;
            }
        }         
    }
    
    return false;
}
bool Individual::is_dep_rec_sched_violation(Position &p, int proc, int a, int i, int b, int j)
{
    /**
     * if 2 channels have the same destination proc,
     * then the rec order should be in the same order as the dst_actor firing order
     */ 
    if(a < (int) no_channels && b < (int) no_channels)
    {
        int dst_b = applications->getChannel(b)->destination;
        int dst_a = applications->getChannel(a)->destination;
        if(p.proc_mappings[dst_a] == p.proc_mappings[dst_b])
        {            
            int rank_dst_a = p.proc_sched[p.proc_mappings[dst_a]].get_rank_by_element(dst_a);
            int rank_dst_b = p.proc_sched[p.proc_mappings[dst_b]].get_rank_by_element(dst_b);       
            
            int rank_a = p.rec_sched[proc].get_rank_by_id(i);
            int rank_b = p.rec_sched[proc].get_rank_by_id(j);            
        
            if(rank_dst_a < rank_dst_b && rank_a > rank_b)
            {
                return true;
            }
            if(rank_dst_a > rank_dst_b && rank_a < rank_b)
            {
                return true;
            }
        }
    }
    return false;
}
int Individual::count_proc_sched_violations(Position &p)
{
    int cnt = 0;
    for(size_t proc=0;proc<p.proc_sched.size();proc++)
    {
        for(size_t i=0;i<p.proc_sched[proc].get_elements().size();i++)
        {
            int a = p.proc_sched[proc].get_elements()[i];
            for(size_t j=i;j<p.proc_sched[proc].get_elements().size();j++)
            {                
                int b = p.proc_sched[proc].get_elements()[j];
                if(is_dep_sched_violation(p, proc, a, i, b, j))
                {
                    cnt++;
                }
            }
        }
    }
    return cnt;
}
int Individual::count_send_sched_violations(Position& p)
{
    /**
     * if the source of a and b are on the same proc
     * and rank_src_a < rank_src_b and rank_a > rank_b then switch
     */ 
    int cnt = 0;
    for(size_t proc=0;proc<p.send_sched.size();proc++)
    {
        for(size_t i=0;i<p.send_sched[proc].get_elements().size();i++)
        {
            int a = p.send_sched[proc].get_elements()[i];
            for(size_t j=0;j<p.send_sched[proc].get_elements().size();j++)
            {                
                int b = p.send_sched[proc].get_elements()[j];
                if(is_dep_send_sched_violation(p, proc, a, i, b, j))
                {
                    cnt++;                    
                }
            }
        }
    }
    return cnt;
}

int Individual::count_rec_sched_violations(Position& p)
{
    int cnt = 0;
    for(size_t proc=0;proc<p.rec_sched.size();proc++)
    {
        p.rec_sched[proc].set_rank( tools::bring_v_to_bound(p.rec_sched[proc].get_rank(), 0, (int)p.rec_sched[proc].get_rank().size()-1) );
        for(size_t i=0;i<p.rec_sched[proc].get_elements().size();i++)
        {
            int a = p.rec_sched[proc].get_elements()[i];
            for(size_t j=0;j<p.rec_sched[proc].get_elements().size();j++)
            {                
                int b = p.rec_sched[proc].get_elements()[j];
                if(is_dep_rec_sched_violation(p, proc, a, i, b, j))
                {
                    cnt++;                    
                }
            }
        }
    }
    return cnt;
}

int Individual::count_sched_violations(Position& p)
{
    int cnt = 0;
    /*
    for(size_t i=0;i<applications->n_SDFApps();i++)
    {
        if(cross_proc_deadlock(i, p))
        cnt++;
    }
    */
    
    cnt += count_proc_sched_violations(p)
        + count_send_sched_violations(p) + count_rec_sched_violations(p);
    
    p.cnt_violations = cnt;       
    
    if(cross_proc_deadlock(p))
    {
        cnt +=1;//larger penalty for cross proc deadlocks     
        /*
        cout << *this << endl
        << "actors:" << tools::toString(cross_proc_deadlock_actors) << endl;
        THROW_EXCEPTION(RuntimeException, "cross_proc_deadlock!");
        */ 
    }
    
    return cnt;
}
int Individual::estimate_sched_violations(Position& p)
{
    int cnt = 0;
    cnt += count_proc_sched_violations(p);
    
    if(cnt == 0)
        cnt += count_send_sched_violations(p) + count_rec_sched_violations(p);
    
    if(cnt == 0 && cross_proc_deadlock(p))
        cnt ++;     
    
    p.cnt_violations = cnt;       
    
    return cnt;
}
bool Individual::cross_proc_deadlock(int app_id, Position &p)
{
    cross_proc_deadlock_actors.empty();
    vector<int> roots = applications->get_root(app_id);
    set<int> can_fire(roots.begin(), roots.end());
    set<int> next_in_sched;
    for(auto s : p.proc_sched)
    {
        int a = s.get_element_by_rank(0);
        if((int) applications->getSDFGraph(a) != app_id)
        {
            vector<int> next = get_next_app(a, app_id, s);        
            for(auto n : next)
                next_in_sched.insert(n); 
        }
        else
            next_in_sched.insert(a);
    }
    set<int> fired;
    set<int> intersect_can_sched;
    set_intersection(can_fire.begin(),can_fire.end(),next_in_sched.begin(),next_in_sched.end(),
                        std::inserter(intersect_can_sched,intersect_can_sched.begin()));
    ///# if the interserction is already empty, then we have deadlock
    if(intersect_can_sched.empty() && !can_fire.empty())
    {
        cross_proc_deadlock_actors = can_fire;
        return true;                 
    }

    while(!intersect_can_sched.empty())
    {
        ///# select one actor from intersect_can_sched
        int a = *intersect_can_sched.begin();
        can_fire.erase(a);
        next_in_sched.erase(a);
        ///# fire actor a
        fired.insert(a);
        ///# find next of a and add to next_in_sched set
        int proc_a = p.proc_mappings[a];
        vector<int> next = get_next_app(a, app_id, p.proc_sched[proc_a]);
        for(auto n : next)
            next_in_sched.insert(n); 
        ///# add successors of a to can_fire set
        vector<int> succ = applications->getSuccessors(a);
        can_fire.insert(succ.begin(), succ.end());
        intersect_can_sched.clear();
        set_intersection(can_fire.begin(),can_fire.end(), next_in_sched.begin(), next_in_sched.end(),
                            std::inserter(intersect_can_sched,intersect_can_sched.begin()));              
        ///# if there are actors that can be fired but the intersection is empty
        if(intersect_can_sched.empty() && !can_fire.empty())
        {
            /*
            cout << "intersection is empty while " << tools::toString(can_fire)                 
                 << " needs to be fired" << endl
                 << "next_in_sched:" << tools::toString(next_in_sched) << endl ;
            cout << "schedule is:" << tools::toString(p.proc_sched) << endl<< endl;
            */
            cross_proc_deadlock_actors = can_fire;
            return true;
        }
    }    
    return false;
}
bool Individual::cross_proc_deadlock(Position &p)
{
    cross_proc_deadlock_actors.empty();
    vector<int> roots;
    for(size_t i=0;i<applications->n_SDFApps();i++)
    {
        vector<int> tmp_r = applications->get_root(i);
        roots.insert(roots.end(), tmp_r.begin(), tmp_r.end());
    }
     
    set<int> can_fire(roots.begin(), roots.end());
    set<int> next_in_sched;
    for(auto s : p.proc_sched)
    {
        int a = s.get_element_by_rank(0);
        next_in_sched.insert(a);
    }
    set<int> fired;
    set<int> intersect_can_sched;
    set_intersection(can_fire.begin(),can_fire.end(),next_in_sched.begin(),next_in_sched.end(),
                        std::inserter(intersect_can_sched,intersect_can_sched.begin()));

    ///# if the interserction is already empty, then we have deadlock
    if(intersect_can_sched.empty() && !can_fire.empty()) 
    {
        cross_proc_deadlock_actors = can_fire;
        return true;
    }
    while(!intersect_can_sched.empty())
    {
        ///# select one actor from intersect_can_sched
        int a = *intersect_can_sched.begin();
        can_fire.erase(a);
        next_in_sched.erase(a);
        ///# fire actor a
        fired.insert(a);
        ///# find next of a and add to next_in_sched set
        int proc_a = p.proc_mappings[a];
        next_in_sched.insert(p.proc_sched[proc_a].get_next(a));     
        ///# add successors of a to can_fire set
        vector<int> succ = applications->getSuccessors(a);
        for(auto s : succ)
        {
            vector<int> pred = applications->getPredecessors(s);   
            bool ready = true;
            for(auto p : pred)         
            {
                bool is_in = fired.find(p) != fired.end();
                if(!is_in)
                {
                    ready = false;
                    //cout << s << " is not ready to be fired!\n";
                    break;
                }   
            }
            if(ready)
                can_fire.insert(s);
        }
        //can_fire.insert(succ.begin(), succ.end());
        intersect_can_sched.clear();
        set_intersection(can_fire.begin(),can_fire.end(), next_in_sched.begin(), next_in_sched.end(),
                            std::inserter(intersect_can_sched,intersect_can_sched.begin()));              
        /*
           cout << "firing:" << a 
             << " next:" << p.proc_sched[proc_a].get_next(a)
             << " can:" << tools::toString(succ)
             << " next_set:" << tools::toString(next_in_sched)
             << " can_set:" << tools::toString(can_fire)
             << endl;
        */ 
        ///# if there are actors that can be fired but the intersection is empty
        if(intersect_can_sched.empty() && !can_fire.empty())
        {
            /*
            cout << "intersection is empty while " << tools::toString(can_fire)                 
                 << " needs to be fired" << endl
                 << "next_in_sched:" << tools::toString(next_in_sched) << endl ;
            cout << "schedule is:" << tools::toString(p.proc_sched) << endl<< endl;
            */            
            cross_proc_deadlock_actors = can_fire;
            return true;
        }
    }    
    return false;
}
vector<int> Individual::get_next_app(int elem, int app_id,  Schedule &s)
{
    vector<int> next_elem;
    if(elem >= (int) no_actors)
        return next_elem;
        
    size_t next = s.get_next(elem);    
    while(next < no_actors)
    {
        if((int) applications->getSDFGraph(next) == app_id)
        {
            next_elem.push_back(next);
            return next_elem;
        }
        next = s.get_next(next);
    }
    return next_elem;
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
    //repair_cross_deadlock(p);  
    repair_send_sched(p);
    repair_rec_sched(p);     
}
void Individual::repair_cross_deadlock(Position& p)
{
    cross_proc_deadlock(p);
    for(auto a : cross_proc_deadlock_actors)
    {
        ///# set rank of a to random lower value 
        int proc = p.proc_mappings[a];
        int rank_a = p.proc_sched[proc].get_rank_by_element(a);
        int new_rank = random::random_indx(rank_a);
        p.proc_sched[proc].set_rank_by_element(a, new_rank);
    }
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
                if(is_dep_sched_violation(p, proc, a, i, b, j))
                {
                    //if(random::random_bool())
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
                if(is_dep_send_sched_violation(p, proc, a, i, b, j))                
                {
                    //if(random::random_bool())
                        p.send_sched[proc].switch_ranks(i, j);                        
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
                if(is_dep_rec_sched_violation(p, proc, a, i, b, j))
                {
                    //if(random::random_bool())
                        p.rec_sched[proc].switch_ranks(i, j);
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
    current_position.fitness.resize(no_entities + 1,0);///energy + memory violations + throughputs    
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
vector<int> Individual::get_next(vector<Schedule> sched_set, int no_elements) const
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
        current_position.fitness.clear();
        current_position.fitness.resize(no_entities + 1,0);
        int no_sched_vio = count_sched_violations(current_position);
        //int no_sched_vio = estimate_sched_violations(current_position);
        
        if(no_sched_vio == 0)
        {            
            Design design(mapping, applications, current_position.proc_mappings, 
                          current_position.proc_modes, get_next(current_position.proc_sched, no_actors),
                          get_next(current_position.send_sched, no_channels),
                          get_next(current_position.rec_sched, no_channels), 
                          current_position.tdmaAlloc);
            
            vector<int> prs = design.get_periods();
            int eng = design.get_energy();
            
            int no_mem_violations = 0;
            for(auto m : design.get_slack_memory())
                if(m < 0)
                    no_mem_violations++;
            
            current_position.cnt_violations = no_sched_vio+no_mem_violations;
            
            float penalty_ratio = ((float)current_position.cnt_violations);
            for(size_t i=0;i< prs.size();i++)
            {
                if(prs[i] <= 0)
                    current_position.fitness[i] = INT_MAX;
                else
                    current_position.fitness[i] = prs[i] + (prs[i])*penalty_ratio;    
               
               if(current_position.cnt_violations == 0 && prs[i] < 0)
               {
                   Design tmp_design(mapping, applications, current_position.proc_mappings, 
                          current_position.proc_modes, get_next(current_position.proc_sched, no_actors),
                          get_next(current_position.send_sched, no_channels),
                          get_next(current_position.rec_sched, no_channels), 
                          current_position.tdmaAlloc);
            
                   tmp_design.set_print_debug(true);
                   tmp_design.get_periods();
                   cout << endl << *this << endl;
                   cout << "communication violations:" << count_send_sched_violations(current_position) + count_rec_sched_violations(current_position) << endl;
                   THROW_EXCEPTION(RuntimeException, "period is negative while there is zero violation!");
               }    
                
            }
            if(eng < 0)
                 current_position.fitness[current_position.fitness.size()-1] = INT_MAX;
            else    
                current_position.fitness[current_position.fitness.size()-1] = eng + eng*penalty_ratio;
        }    
        else//if there is scheduling violations
        {
            int max_penalty = *max_element(penalty.begin(), penalty.end()-1);      
            for(size_t i=0;i< penalty.size()-1;i++)
                current_position.fitness[i] = (1+no_sched_vio) * max_penalty;//penalty[i];
            
            current_position.fitness[current_position.fitness.size()-1] = *(penalty.end()-1) * no_sched_vio;
            
        }
      
        //current_position.fitness[current_position.fitness.size()-1] = no_mem_violations;        
    }
    catch(std::exception const& e)
    {
        cout << "calc fitness failed!\n";
        THROW_EXCEPTION(RuntimeException, e.what() );
    }
    if(current_position.cnt_violations > 0)
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
    out << "proc_sched:" << tools::toString(ind.get_next(ind.current_position.proc_sched, ind.no_actors)) << endl;
    out << "send_sched:" << tools::toString(ind.get_next(ind.current_position.send_sched, ind.no_channels)) << endl;
    out << "rec_sched:" << tools::toString(ind.get_next(ind.current_position.rec_sched, ind.no_channels)) << endl;
    out << "best g position: ====================\n" << ind.best_global_position << endl;
    return out;
}
