#include "particle.hpp"

Particle::Particle(shared_ptr<Mapping> _mapping, shared_ptr<Applications> _application, 
                    int _objective, float _w_t, float _w_lb, float _w_gb, 
                    bool _multi_obj, vector<float> _o_w, vector<int> _penalty):
                    Individual(_mapping, _application, _multi_obj, _o_w, _penalty),
                     objective(_objective),
                    best_local_position(_multi_obj, _o_w),
                    speed(no_actors, no_channels, no_processors),                    
                    w_t(_w_t),
                    w_lb(_w_lb),
                    w_gb(_w_gb),
                    max_w_t(_w_t){}
                    
Particle::Particle(const Particle& _p):
                    Individual(_p),
                    objective(_p.objective),
                    best_local_position(_p.best_local_position),
                    speed(_p.speed),                    
                    w_t(_p.w_t),
                    w_lb(_p.w_lb),
                    w_gb(_p.w_gb),
                    max_w_t(_p.max_w_t) {}

Speed Particle::get_speed()
{
    return speed;
}
void Particle::update()
{   
    if(current_position.dominate(best_local_position) || best_local_position.empty() || best_local_position == current_position)
    {   
        best_local_position = current_position;
    }
    /**
     * \note - In case the number of invalid moves is more than the threshold,
     * then reinitialize the particle.
     */
    if(no_invalid_moves > thr_invalid_mov)  
    {
        init_random();
        Speed s(no_actors, no_channels, no_processors);
        speed = s;
        best_local_position = current_position;
        best_global_position = current_position;
        w_t = max_w_t;
        //cout << "the particle is randomly initiated again\n";
    }
    /// -# Update speed.    
    update_speed();    
    /// -# Move the particle based on the speed.
    move();
    /// -# Repair the particle.
    repair(current_position);       
}

void Particle::update_speed()
{
    if(best_global_position.empty())
        THROW_EXCEPTION(RuntimeException, "update_speed: best_global is empty" );
    if(best_local_position.empty())
        THROW_EXCEPTION(RuntimeException, "update_speed: best_local is empty" );
    float y1 = random_weight();    
    float y2 = random_weight();
    
    for(size_t i=0;i<speed.proc_mappings.size();i++)
    {
        speed.proc_mappings[i] = w_t * speed.proc_mappings[i] +
                                 y1 * w_lb * (best_local_position.proc_mappings[i] - current_position.proc_mappings[i]) +
                                 y2 * w_gb * (best_global_position.proc_mappings[i] - current_position.proc_mappings[i]);      
    }
    speed.proc_mappings = tools::round_2(speed.proc_mappings);
    for(size_t i=0;i<speed.proc_modes.size();i++)
    {
        speed.proc_modes[i] = w_t * speed.proc_modes[i] +
                                 y1 * w_lb * (best_local_position.proc_modes[i] - current_position.proc_modes[i]) +
                                 y2 * w_gb * (best_global_position.proc_modes[i] - current_position.proc_modes[i]);
    }
    speed.proc_modes = tools::round_2(speed.proc_modes);
    for(size_t i=0;i<speed.tdmaAlloc.size();i++)
    {
        if( speed.tdmaAlloc.size() != current_position.tdmaAlloc.size() ||
            speed.tdmaAlloc.size() != best_local_position.tdmaAlloc.size() ||
            speed.tdmaAlloc.size() != best_global_position.tdmaAlloc.size() )
        {
            cout << "s:" << speed.tdmaAlloc.size()
                 << " c:" << current_position.tdmaAlloc.size()
                 << " l:" << best_local_position.tdmaAlloc.size()
                 << " g:" << best_global_position.tdmaAlloc.size()
                 << endl;
            cout << "update_speed failed!\n";
        }    
        try{
        speed.tdmaAlloc[i] = w_t * speed.tdmaAlloc[i] +
                                 y1 * w_lb * (best_local_position.tdmaAlloc[i] - current_position.tdmaAlloc[i]) +
                                 y2 * w_gb *  (best_global_position.tdmaAlloc[i] - current_position.tdmaAlloc[i]);
         }
          catch(std::exception const& e)
        {
            cout << "s:" << speed.tdmaAlloc.size()
                 << " c" << current_position.tdmaAlloc.size()
                 << " l" << best_local_position.tdmaAlloc.size()
                 << " g:" << best_global_position.tdmaAlloc.size()
                 << endl;
            cout << "update_speed failed!\n";
            THROW_EXCEPTION(RuntimeException, e.what() );
        }
                                 
    }
    speed.tdmaAlloc = tools::round_2(speed.tdmaAlloc);
    for(size_t i=0;i<speed.proc_sched.size();i++)
    {
        int cu_p = current_position.proc_mappings[i];
        int bl_p = best_local_position.proc_mappings[i];
        int bg_p = best_global_position.proc_mappings[i];
        speed.proc_sched[i] = w_t * speed.proc_sched[i] +
                                 y1 * w_lb * (best_local_position.proc_sched[bl_p].get_relative_rank_by_element(i) - current_position.proc_sched[cu_p].get_relative_rank_by_element(i)) +
                                 y2 * w_gb * (best_global_position.proc_sched[bg_p].get_relative_rank_by_element(i) - current_position.proc_sched[cu_p].get_relative_rank_by_element(i));       
    }
    speed.proc_sched = tools::round_2(speed.proc_sched);
    for(size_t i=0;i<speed.send_sched.size();i++)
    {
        int src = applications->getChannel(i)->source;
        int cu_p = current_position.proc_mappings[src];
        int bl_p = best_local_position.proc_mappings[src];
        int bg_p = best_global_position.proc_mappings[src];
        speed.send_sched[i] = w_t * speed.send_sched[i] +
                                 y1 * w_lb * (best_local_position.send_sched[bl_p].get_relative_rank_by_element(i) - current_position.send_sched[cu_p].get_relative_rank_by_element(i)) +
                                 y2 * w_gb * (best_global_position.send_sched[bg_p].get_relative_rank_by_element(i) - current_position.send_sched[cu_p].get_relative_rank_by_element(i));
    }
    speed.send_sched = tools::round_2(speed.send_sched);
    for(size_t i=0;i<speed.rec_sched.size();i++)
    {
        int dst = applications->getChannel(i)->destination;
        int cu_p = current_position.proc_mappings[dst];
        int bl_p = best_local_position.proc_mappings[dst];
        int bg_p = best_global_position.proc_mappings[dst];
        speed.rec_sched[i] = w_t * speed.rec_sched[i] +
                                 y1 * w_lb * (best_local_position.rec_sched[bl_p].get_relative_rank_by_element(i) - current_position.rec_sched[cu_p].get_relative_rank_by_element(i)) +
                                 y2 * w_gb * (best_global_position.rec_sched[bg_p].get_relative_rank_by_element(i) - current_position.rec_sched[cu_p].get_relative_rank_by_element(i));
    }
    speed.rec_sched = tools::round_2(speed.rec_sched);
    ///#- applying the speed bounds
    speed.apply_bounds();
}
void Particle::move() 
{
    for(size_t i=0;i<current_position.proc_mappings.size();i++)
    {
        current_position.proc_mappings[i] = Schedule::random_round((float) current_position.proc_mappings[i] + speed.proc_mappings[i]);                 
    }
    current_position.proc_mappings = tools::bring_v_to_bound(current_position.proc_mappings, 0, (int)no_processors-1);
    for(size_t i=0;i<current_position.proc_modes.size();i++)
    {
        current_position.proc_modes[i] = Schedule::random_round((float) current_position.proc_modes[i] + speed.proc_modes[i]);                 
    }
    for(size_t i=0;i<current_position.tdmaAlloc.size();i++)
    {
        current_position.tdmaAlloc[i] = Schedule::random_round((float) current_position.tdmaAlloc[i] + speed.tdmaAlloc[i]);                 
    }

    build_schedules(current_position);
    /** \li Add proc_sched. */    
    for(size_t proc;proc<current_position.proc_sched.size();proc++)
    {
        for(auto e : current_position.proc_sched[proc].get_elements())
        {
            current_position.proc_sched[proc].set_rank_by_element(e, 
                    Schedule::random_round((float)current_position.proc_sched[proc].get_rank_by_element(e)+
                                            speed.proc_sched[e] * current_position.proc_sched[proc].size()) );
        }
    }  
    /** \li Add send_sched. */  
    for(size_t proc;proc<current_position.send_sched.size();proc++)
    {
        for(auto e : current_position.send_sched[proc].get_elements())
        {
            current_position.send_sched[proc].set_rank_by_element(e, 
                    Schedule::random_round((float)current_position.send_sched[proc].get_rank_by_element(e)+
                                            speed.send_sched[e] * current_position.proc_sched[proc].size()) );
        }
    } 
    /** \li Add rec_sched. */        
    for(size_t proc;proc<current_position.rec_sched.size();proc++)
    {
        for(auto e : current_position.rec_sched[proc].get_elements())
        {
            current_position.rec_sched[proc].set_rank_by_element(e, 
                    Schedule::random_round((float)current_position.rec_sched[proc].get_rank_by_element(e)+
                                            speed.rec_sched[e] * current_position.proc_sched[proc].size()) );
        }
    }              
    /** \li Decrease w_t.*/
    if(w_t > min_w_t)
        w_t = w_t - delta_w_t;    
}

int Particle::get_objective()
{
    return objective;
}
void Particle::avoid_stagnation()
{
    current_position.opposite();
    build_schedules(current_position);
    repair(current_position);       
    cout << "moving to opposite\n";    
}
std::ostream& operator<< (std::ostream &out, const Speed &s)
{
    out << "proc_mappings: ";
    for(auto m : s.proc_mappings)
        out << m << ", ";
    out << endl << "proc_modes:";    
    for(auto m : s.proc_modes)
        out << m << ", ";  /* 
    out << endl << "tdmaAlloc:";    
    for(auto t : s.tdmaAlloc)
        out << t << ", ";
    out << endl << "proc_sched:";    
    for(auto sc : s.proc_sched)
        out << sc << ", ";
    out << endl << "send_sched:";        
    for(auto se : s.send_sched)
        out << se << ", ";
    out << endl << "rec_sched:";        
    for(auto r : s.rec_sched)
        out << r << ", ";    */    
    return out;
}
std::ostream& operator<< (std::ostream &out, const Particle &particle)
{
    out << "current position: ====================\n" << particle.current_position << endl;
    out << "best l position: ====================\n" << particle.best_local_position << endl;
    out << "best g position: ====================\n" << particle.best_global_position << endl;
    out << "speed: ====================\n" << particle.speed << endl;
    out << "w_t:" << particle.w_t;
    return out;
}


