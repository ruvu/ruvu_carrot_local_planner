#pragma once

namespace ruvu_carrot_local_planner
{
/**
 * Find element in iterator with the minimum calculated value
 */
template <typename Iter, typename Getter>
Iter min_by(Iter begin, Iter end, Getter getCompareVal)
{
  if (begin == end)
    return end;
  auto lowest = getCompareVal(*begin);
  Iter lowest_it = begin;
  for (Iter it = ++begin; it != end; ++it)
  {
    auto comp = getCompareVal(*it);
    if (comp < lowest)
    {
      lowest = comp;
      lowest_it = it;
    }
  }
  return lowest_it;
}

/**
 * Compute the sign of a number
 * https://stackoverflow.com/questions/1903954/is-there-a-standard-sign-function-signum-sgn-in-c-c
 */
template <typename T>
int sgn(T val)
{
  return (T(0) < val) - (val < T(0));
}
}
