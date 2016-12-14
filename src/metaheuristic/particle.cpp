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
    //init_random();
    Schedule actor_sched(no_actors);
    cout << actor_sched;
    for(int i=0;i<20;i++)
        cout << Schedule::random_round(1.2) << " ";
    cout << endl;    
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
    uniform_int_distribution<int> dist_tdma(0, no_tdma_slots-1);

    /// the engine has to be reset after crreating the distribution
    auto gen_proc = std::bind(dist_proc, mersenne_engine);mersenne_engine();
    auto gen_actor = std::bind(dist_proc, mersenne_engine);mersenne_engine();
    auto gen_channel_s = std::bind(dist_proc, mersenne_engine);mersenne_engine();
    auto gen_channel_r = std::bind(dist_proc, mersenne_engine);mersenne_engine();
    auto gen_tdma = std::bind(dist_proc, mersenne_engine);mersenne_engine();
    
    proc_mappings.resize(no_actors, 0);
    proc_modes.resize(no_processors, 0);//TODO mode based on each proc mode rand
    next.resize(no_actors, 0);
    sendNext.resize(no_channels, 0);
    recNext.resize(no_channels, 0);
    tdmaAlloc.resize(no_processors, 0);//TODO
    
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
    ///Random # tdma slots based on src and dst of channels
    for(size_t i=0;i<no_channels;i++)
    {
         int src_i = applications->getChannels()[i]->source;
        int dest_i = applications->getChannels()[i]->destination;
        int proc_src_i = proc_mappings[src_i];
        int proc_dest_i = proc_mappings[dest_i];    
        if(proc_src_i != proc_dest_i)
        {
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
     
    return out;
}
std::ostream& operator<< (std::ostream &out, const Schedule &sched)
{
    out << "next: ";
    for(auto i : sched.next)
        out << i << "->";
    out << endl << "rank:";    
    for(auto r : sched.rank)
        out << r << " ";    
    out << endl;         
    return out;
}
Schedule::Schedule(size_t size)
{
    for(size_t i=0;i<size;i++)
    {
        next.push_back(i);
        rank.push_back(i);
    }
    std::random_shuffle (rank.begin(), rank.end());
    update_next();
}
void Schedule::update_next()
{
    for(size_t i=0;i<rank.size();i++)
    {
        next[rank[i]] = i;
    }
}
void Schedule::set_rank(int index, int value)
{
    rank[index] = value;
    next[value] = index;
}
void Schedule::set_rank(vector<int> _rank)
{
    if(rank.size() != _rank.size())
        THROW_EXCEPTION(RuntimeException, "rank.size() != _rank.size()" );
        
    for(size_t i=0;i<_rank.size();i++)
        set_rank(i, _rank[i]);
}
vector<int> Schedule::get_rank()
{
    return rank;
}
vector<int> Schedule::get_next()
{
    return next;
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
