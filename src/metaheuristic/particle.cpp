#include "particle.hpp"
Particle::Particle(Mapping* _mapping, Applications* _application):
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
        shared_ptr<Schedule> s(new Schedule(get_actors_by_proc(i), i+no_processors));
        proc_sched.push_back(s);
        
        shared_ptr<Schedule> snd(new Schedule(get_channel_by_src(i), i+no_processors));
        send_sched.push_back(snd);
        
        shared_ptr<Schedule> rc(new Schedule(get_channel_by_dst(i), i+no_processors));
        rec_sched.push_back(rc);
    }
    repair();     
}
void Particle::repair()
{
    repair_tdma();    
    repair_proc_sched();
}
void Particle::repair_proc_sched()
{
    for(size_t proc=0;proc<proc_sched.size();proc++)
        for(size_t i=0;i<proc_sched[proc]->get_elements().size();i++)
            for(size_t j=i;j<proc_sched[proc]->get_elements().size();j++)
            {
                int a = proc_sched[proc]->get_elements()[i];
                int b = proc_sched[proc]->get_elements()[j];
                int rank_a = proc_sched[proc]->get_rank_by_id(a);
                int rank_b = proc_sched[proc]->get_rank_by_id(b);
                if(rank_a > rank_b && applications->dependsOn(a, b))
                {
                    //should switch i and j
                    proc_sched[proc]->switch_ranks(i, j);
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
    auto gen_actor = std::bind(dist_proc, mersenne_engine);mersenne_engine();
    auto gen_channel_s = std::bind(dist_proc, mersenne_engine);mersenne_engine();
    auto gen_channel_r = std::bind(dist_proc, mersenne_engine);mersenne_engine();
    auto gen_tdma = std::bind(dist_proc, mersenne_engine);mersenne_engine();
    
    proc_mappings.resize(no_actors, 0);
    proc_modes.resize(no_processors, 0);
    next.resize(no_actors, 0);
    sendNext.resize(no_channels, 0);
    recNext.resize(no_channels, 0);
    tdmaAlloc.resize(no_processors, 0);
    
    generate(begin(proc_mappings), end(proc_mappings), gen_proc); 
    generate(begin(next), end(next), gen_actor);
    generate(begin(sendNext), end(sendNext), gen_channel_s);
    generate(begin(recNext), end(recNext), gen_channel_r);
    generate(begin(tdmaAlloc), end(tdmaAlloc), gen_tdma);
    
    ///Random proc_modes based on the number of modes for each processors
    for(size_t i=0;i<no_processors;i++)
    {
       int no_proc_modes = mapping->getPlatform()->getModes(i);
       std::uniform_int_distribution<int> uni_dist(0,no_proc_modes);
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
std::ostream& operator<< (std::ostream &out, const Particle &particle)
{
    out << "proc_mappings: ";
    for(auto m : particle.proc_mappings)
        out << m << " ";
    out << endl << "proc_modes:";    
    for(auto m : particle.proc_modes)
        out << m << " ";
    out << endl << "next:";    
    for(auto n : particle.next)
        out << n << " ";
    out << endl << "sendNext:";    
    for(auto s : particle.sendNext)
        out << s << " ";
    out << endl << "recNext:";    
    for(auto r : particle.recNext)
        out << r << " ";
    out << endl << "tdmaAlloc:";    
    for(auto t : particle.tdmaAlloc)
        out << t << " ";
    out << endl << "proc_sched -----\n";    
    for(auto s : particle.proc_sched)
        out << *s;
    out << endl << "send_sched -----\n";        
    for(auto s : particle.send_sched)
        out << *s;
    out << endl << "rec_sched -----\n";        
    for(auto r : particle.rec_sched)
        out << *r << "\n";        
    return out;
}
std::ostream& operator<< (std::ostream &out, const Schedule &sched)
{
    out << endl << "elements:";    
    for(auto e : sched.elements)
        out << e << " ";    
    out << endl << "rank:";    
    for(auto r : sched.rank)
        out << r << " ";    
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
    const bool is_in = std::find(elements.begin(), elements.end(),elem_id) != elements.end();
    if(!is_in)
        THROW_EXCEPTION(RuntimeException, "element is not in the set");
           
    return rank[elem_id];
}
vector<int> Schedule::get_rank()
{
    return rank;
}
vector<int> Schedule::get_elements()
{
    return elements;
}

int Schedule::get_next(int elem_id)
{
    /**
     * If the input is in the elements list, then we return the element with rank[elem_id]+1
     */ 
    if(elem_id >= (int) elements.size())
        THROW_EXCEPTION(RuntimeException, "element is not in the set");
    
    return get_element_by_rank(rank[elem_id] + 1) ;
    
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
