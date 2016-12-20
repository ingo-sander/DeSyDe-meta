#include "swarm.hpp"
Swarm::Swarm(shared_ptr<Mapping> _mapping, shared_ptr<Applications> _application):
                    mapping(_mapping),
                    applications(_application),
                    no_particles(10)
{   
}
void Swarm::search()
{}
std::ostream& operator<< (std::ostream &out, const Swarm &swarm)
{
    out << "no particles=" << swarm.no_particles << endl;
    return out;
}
