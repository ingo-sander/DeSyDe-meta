#include "particle.hpp"
Particle::Particle(shared_ptr<Mapping> _mapping, shared_ptr<Applications> _application):
                    mapping(_mapping),
                    applications(_application),
                    no_entities(mapping->getNumberOfApps()),
                    no_actors(applications->n_SDFActors()),
                    no_channels(applications->n_SDFchannels()),
                    no_processors(mapping->getPlatform()->nodes()),
                    no_tdma_slots(mapping->getPlatform()->tdmaSlots())
{   
    init_random();
    for(size_t i=0;i<no_processors;i++)
    {
        shared_ptr<Schedule> s(new Schedule(get_actors_by_proc(i), i+no_actors));
        proc_sched.push_back(s);
        
        shared_ptr<Schedule> snd(new Schedule(get_channel_by_src(i), i+no_channels));
        send_sched.push_back(snd);
        
        shared_ptr<Schedule> rc(new Schedule(get_channel_by_dst(i), i+no_channels));
        rec_sched.push_back(rc);
    }
    LOG_INFO("before repair ...");
    cout << "next: ";
    for(auto n : get_next(proc_sched, no_actors))
        cout << n << " ";
    cout << endl;        
    
    repair();     
    
    LOG_INFO("after repair ...");
    cout << "next: ";
    for(auto n : get_next(proc_sched, no_actors))
        cout << n << " ";
    cout << endl;        
    
    for(auto n : get_next(send_sched, no_channels))
        cout << n << " ";
    cout << endl;        
    
    for(auto n : get_next(rec_sched, no_channels))
        cout << n << " ";
    cout << endl;        
}
void Particle::repair()
{
    repair_tdma();    
    repair_sched(proc_sched, no_actors);
    repair_send_sched(send_sched, no_channels);
    repair_rec_sched(rec_sched, no_channels);
}
void Particle::repair_sched(vector<shared_ptr<Schedule>> sched_set, int no_elements)
{
    for(size_t proc=0;proc<sched_set.size();proc++)
        for(size_t i=0;i<sched_set[proc]->get_elements().size();i++)
        {
            int a = sched_set[proc]->get_elements()[i];
            for(size_t j=i;j<sched_set[proc]->get_elements().size();j++)
            {                
                int b = sched_set[proc]->get_elements()[j];
                int rank_a = sched_set[proc]->get_rank_by_id(i);
                int rank_b = sched_set[proc]->get_rank_by_id(j);
                /**
                 * if a and b are not dummy and the dependency is violated
                 */ 
                if((a < no_elements && b < no_elements) && rank_a > rank_b && applications->dependsOn(a, b))
                {
                    sched_set[proc]->switch_ranks(i, j);
                    LOG_DEBUG("switching " + tools::toString(a) +
                              " and " + tools::toString(b));
                }
            }
        }
    
}
void Particle::repair_send_sched(vector<shared_ptr<Schedule>> sched_set, int no_elements)
{
    for(size_t proc=0;proc<sched_set.size();proc++)
        for(size_t i=0;i<sched_set[proc]->get_elements().size();i++)
        {
            int a = sched_set[proc]->get_elements()[i];
            for(size_t j=0;j<sched_set[proc]->get_elements().size();j++)
            {                
                int b = sched_set[proc]->get_elements()[j];
                if(a < no_elements && b < no_elements)
                {
                    int dst_b = applications->getChannels()[b]->destination;
                    int dst_a = applications->getChannels()[a]->destination;
                    int src_b = applications->getChannels()[b]->source;
                    int src_a = applications->getChannels()[a]->source;
                    int rank_a = sched_set[proc]->get_rank_by_id(i);
                    int rank_b = sched_set[proc]->get_rank_by_id(j);
                    int rank_src_a = proc_sched[proc_mappings[src_a]]->get_rank_by_element(src_a);
                    int rank_src_b = proc_sched[proc_mappings[src_b]]->get_rank_by_element(src_b);
                    /**
                     * if a and b are sending to the same destination
                     * and the dependency is violated
                     * OR 
                     * if their source is on the same proc and 
                     * rank_src_a < rank_src_b and rank_a > rank_b
                     */ 
                    if(
                      (dst_a == dst_b
                        && rank_a > rank_b && applications->dependsOn(a, b))
                       ||
                       (rank_src_a < rank_src_b && rank_a > rank_b) 
                      )
                    {
                        sched_set[proc]->switch_ranks(i, j);
                        LOG_DEBUG("switching send_next " + tools::toString(a) +
                                  " and " + tools::toString(b));
                    }
                }
            }
        }
    
}
void Particle::repair_rec_sched(vector<shared_ptr<Schedule>> sched_set, int no_elements)
{
    /**
     * if the source of a and b are on the same proc
     * and rank_src_a < rank_src_b and rank_a > rank_b then switch
     */ 
    for(size_t proc=0;proc<sched_set.size();proc++)
        for(size_t i=0;i<sched_set[proc]->get_elements().size();i++)
        {
            int a = sched_set[proc]->get_elements()[i];
            for(size_t j=0;j<sched_set[proc]->get_elements().size();j++)
            {                
                int b = sched_set[proc]->get_elements()[j];
                if(a < no_elements && b < no_elements)
                {
                    int src_b = applications->getChannels()[b]->source;
                    int src_a = applications->getChannels()[a]->source;
                    if(proc_mappings[src_a] == proc_mappings[src_b])
                    {
                        int rank_src_a = send_sched[proc_mappings[src_a]]->get_rank_by_element(a);
                        int rank_src_b = send_sched[proc_mappings[src_b]]->get_rank_by_element(b);       
                        int rank_a = sched_set[proc]->get_rank_by_id(i);
                        int rank_b = sched_set[proc]->get_rank_by_id(j);            
                    
                        if(rank_src_a < rank_src_b && rank_a > rank_b)
                        {
                            sched_set[proc]->switch_ranks(i, j);
                            LOG_DEBUG("switching rec_next " + tools::toString(a) +
                                      " and " + tools::toString(b));
                        }
                    }
                }
            }
        }
    
}
vector<int> Particle::get_actors_by_proc(int proc_id)
{
    vector<int> actors;
    for(size_t i=0;i<proc_mappings.size();i++)
    {
        if(proc_mappings[i] == proc_id)
            actors.push_back(i);
    }
    return actors;
}
vector<int> Particle::get_channel_by_src(int src_proc_id)
{
    vector<int> channels;
    for(size_t i=0;i<no_channels;i++)
    {
        if(proc_mappings[applications->getChannels()[i]->source] == src_proc_id)
            channels.push_back(i);
    }
    return channels;
}
vector<int> Particle::get_channel_by_dst(int dst_proc_id)
{
    vector<int> channels;
    for(size_t i=0;i<no_channels;i++)
    {
        if(proc_mappings[applications->getChannels()[i]->destination] == dst_proc_id)
            channels.push_back(i);
    }
    return channels;
}
void Particle::init_random()
{
    // First create an instance of an engine.
    random_device rnd_device;
    // Specify the engine and distribution.
    mt19937 mersenne_engine(rnd_device());
    uniform_int_distribution<int> dist_proc(0, no_processors-1);
    uniform_int_distribution<int> dist_actor(0, no_actors-1);
    uniform_int_distribution<int> dist_channel(0, no_channels-1);
    uniform_int_distribution<int> dist_tdma(0, no_tdma_slots);

    /// the engine has to be reset after crreating the distribution
    auto gen_proc = std::bind(dist_proc, mersenne_engine);mersenne_engine();
    auto gen_tdma = std::bind(dist_proc, mersenne_engine);mersenne_engine();
    
    proc_mappings.resize(no_actors, 0);
    proc_modes.resize(no_processors, 0);
    tdmaAlloc.resize(no_processors, 0);
    
    generate(begin(proc_mappings), end(proc_mappings), gen_proc); 
    generate(begin(tdmaAlloc), end(tdmaAlloc), gen_tdma);
    
    ///Random proc_modes based on the number of modes for each processors
    for(size_t i=0;i<no_processors;i++)
    {
       int no_proc_modes = mapping->getPlatform()->getModes(i);
       std::uniform_int_distribution<int> uni_dist(0,no_proc_modes-1);
       proc_modes[i] = uni_dist(mersenne_engine);
    }
}
void Particle::repair_tdma()
{
    vector<int> no_inout_channels(no_processors, 0);
    ///Random # tdma slots based on src and dst of channels
    for(size_t i=0;i<no_channels;i++)
    {
        int src_i = applications->getChannels()[i]->source;
        int dest_i = applications->getChannels()[i]->destination;
        int proc_src_i = proc_mappings[src_i];
        int proc_dest_i = proc_mappings[dest_i];    
        if(proc_src_i != proc_dest_i)
        {
            no_inout_channels[proc_src_i]++;
            no_inout_channels[proc_dest_i]++;
        }      
    }
    for(size_t i=0;i<no_inout_channels.size();i++)
    {
        if(no_inout_channels[i] == 0)
            tdmaAlloc[i] = 0;
        else
        {
            if(tdmaAlloc[i] == 0)
            {
                tdmaAlloc[i]++;
            }
        }    
    }
    /**
     * If the total number of allocated slots is more than the number of 
     * available slots, then take the difference away from some processors
     */     
    int sum_of_elems = std::accumulate(tdmaAlloc.begin(), tdmaAlloc.end(), 0);
    int diff = sum_of_elems - no_tdma_slots;
    while(diff > 0)
    {
        for(size_t i=0;i<tdmaAlloc.size();i++)
        {
            if(tdmaAlloc[i] > 1)
            {
                tdmaAlloc[i]--;
                diff--;
                if(diff <= 0)
                    break;
            }
        }
    }
}
vector<int> Particle::get_next(vector<shared_ptr<Schedule>> sched_set, int no_elements)
{
    vector<int> next(no_elements+no_processors, 0);
    vector<int> low_ranks;
    for (auto s : sched_set)
    {
        for(auto e: s->get_elements())
        {
            next[e] = s->get_next(e);
        }
        //if(s->get_elements().size() > 0)
        {
            low_ranks.push_back(s->get_element_by_rank(0));            
        }
    }
    ///Last dummy node should point to the highest rank of the first proc
    next[no_elements+no_processors-1] = low_ranks[0];
    for(size_t i=0;i<low_ranks.size()-1;i++)
    {
        next[no_elements+i] = low_ranks[i+1];
    }
    return next;
}
vector<int> Particle::get_fitness()
{
    LOG_INFO("get_fitness start");
    vector<int> fitness;
    Design design(mapping, applications, proc_mappings, proc_modes, 
                  get_next(proc_sched, no_actors), get_next(send_sched, no_channels),
                  get_next(rec_sched, no_channels), tdmaAlloc);
    
    //cout << design << endl;
    
    vector<int> prs = design.get_periods();
    
    int eng = design.get_energy();
    for(auto p: prs)
        fitness.push_back(p);
        
    fitness.push_back(eng);
    
    cout << "fitness: ";
    for(auto f : fitness)
        cout << f << " ";
        
    LOG_INFO("get_fitness end");    
    return fitness;
}
std::ostream& operator<< (std::ostream &out, const Particle &particle)
{
    out << "proc_mappings: ";
    for(auto m : particle.proc_mappings)
        out << m << " ";
    out << endl << "proc_modes:";    
    for(auto m : particle.proc_modes)
        out << m << " ";   
    out << endl << "tdmaAlloc:";    
    for(auto t : particle.tdmaAlloc)
        out << t << " ";
    out << endl << "proc_sched -----";    
    for(auto s : particle.proc_sched)
        out << *s;
    out << endl << "send_sched -----";        
    for(auto s : particle.send_sched)
        out << *s;
    out << endl << "rec_sched -----";        
    for(auto r : particle.rec_sched)
        out << *r << " ";        
    return out;
}
std::ostream& operator<< (std::ostream &out, const Schedule &sched)
{
    out << endl << "elements:";    
    for(auto e : sched.elements)
        out << e << " ";   
    out << " ->" << sched.dummy;     
    out << endl << "rank:";    
    for(auto r : sched.rank)
        out << r << " ";    
    out << endl;    
    return out;
}
Schedule::Schedule(vector<int> _elems, int _dummy):
                   elements(_elems),
                   dummy(_dummy)
{
    for(size_t i=0;i<elements.size();i++)
    {
        rank.push_back(i);
    }
    std::random_device rd;
    std::mt19937 g(rd());
 
    std::shuffle(rank.begin(), rank.end(), g);
}
void Schedule::set_rank(int index, int value)
{
    rank[index] = value;
}
void Schedule::set_rank(vector<int> _rank)
{
    if(rank.size() != _rank.size())
        THROW_EXCEPTION(RuntimeException, "rank.size() != _rank.size()" );
        
    for(size_t i=0;i<_rank.size();i++)
        set_rank(i, _rank[i]);
}
int Schedule::get_rank_by_id(int elem_id)
{
    if((size_t) elem_id >= elements.size())
        THROW_EXCEPTION(RuntimeException, "element is not in the set");
           
    return rank[elem_id];
}
int Schedule::get_rank_by_element(int elem)
{
    for(size_t i=0;i<rank.size();i++)
    {
        if(elements[i] == elem)
            return rank[i];
    }
    THROW_EXCEPTION(RuntimeException, "element " + tools::toString(elem) + " is not in the set");
           
    return -1;
}
vector<int> Schedule::get_rank()
{
    return rank;
}
vector<int> Schedule::get_elements()
{
    return elements;
}

int Schedule::get_next(int elem)
{
    int elem_id = -1;
    ///First find the id of element
    for(size_t i=0;i<elements.size();i++)
    {
        if(elements[i] == elem)
            elem_id = i;
    }
    /**
     * If the input is in the elements list, then we return the element with rank[elem_id]+1
     */ 
    if(elem_id < 0)
        THROW_EXCEPTION(RuntimeException, "element is not in the set");
    
    return get_element_by_rank(rank[elem_id] + 1) ;
    
}
vector<int> Schedule::get_next()
{
    vector<int> next;
    for(auto e: elements)
        next.push_back(get_next(e));        
    return next;    
}
int Schedule::get_element_by_rank(int _rank)
{
    /**
     * if _rank is the largest rank, then we return the dummy ellement.
     * othersiwe we find the element with the given rank.
     */ 
    if(_rank == (int) rank.size())
        return dummy;
    for(size_t i=0;i<rank.size();i++)
    {
        if(rank[i] == _rank)
            return elements[i];        
    }
    THROW_EXCEPTION(RuntimeException, "could not find the element with input rank="+tools::toString(_rank));
    return -1;
}
vector<int> Schedule::rank_diff(vector<int> _rank)
{
    if(rank.size() != _rank.size())
        THROW_EXCEPTION(RuntimeException, "rank.size() != _rank.size()" );
    vector<int> diff;
    for(size_t i;i<rank.size();i++)
        diff.push_back(rank[i] - _rank[i]);
    
    return diff;    
}
void Schedule::rank_add(vector<float> _speed)
{
    for(size_t i;i<rank.size();i++)
        set_rank(i, rank[i] + random_round(_speed[i]));
}
int Schedule::random_round(float f)
{
  random_device rnd_device;
  uniform_int_distribution<int> dist(0, 1);
  mt19937 mersenne_engine(rnd_device());  
  auto gen = std::bind(dist, mersenne_engine);
  auto tmp = gen();
  if(tmp == 0)
      return ceil(f);
  return floor(f);  
}
void Schedule::switch_ranks(int i, int j)
{
    int tmp = rank[i];
    set_rank(i, rank[j]);
    set_rank(j, tmp);
}
void Schedule::repair_dist()
{
    for(size_t i=0;i<rank.size();i++)
    {
        int cnt = std::count(rank.begin(), rank.end(), rank[i] );
        if(cnt > 1)
        {
            ///Find first unused rank
            size_t j;
            for(j=0;j<rank.size();j++)
            {
                int cnt_j = std::count(rank.begin(), rank.end(), j);
                if(cnt_j == 0)
                {
                    set_rank(i, j);
                    break;
                }
            }
                        
        }
    }
}
