#include "meta_tools.hpp"

class random {
public:
/**
 * @return a random int between 0 and max.
 */
static int random_indx(int max)
{
    random_device rnd_device;
    uniform_int_distribution<int> dist(0, max);
    mt19937 mersenne_engine(rnd_device());  
    auto gen = std::bind(dist, mersenne_engine);
    int i = gen();
    return i;    
}
/**
 * @return a random int between min and max.
 */
static int random_int(int min, int max)
{
    random_device rnd_device;
    uniform_int_distribution<int> dist(min, max);
    mt19937 mersenne_engine(rnd_device());  
    auto gen = std::bind(dist, mersenne_engine);
    int i = gen();
    return i;    
}
/**
 * @return a random boolean.
 */
static bool random_bool()
{
  random_device rnd_device;
  uniform_int_distribution<int> dist(0, 1);
  mt19937 mersenne_engine(rnd_device());  
  auto gen = std::bind(dist, mersenne_engine);
  auto tmp = gen();
  if(tmp == 0)
      return true;
  return false;  
}
};
