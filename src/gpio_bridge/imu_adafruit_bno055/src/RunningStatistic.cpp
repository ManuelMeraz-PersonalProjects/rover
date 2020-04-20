#include "gpio_bridge/imu/RunningStatistic.hpp"

#include <cmath>

void RunningStatistic::clear()
{
   m_num_data_values = 0;
}
void RunningStatistic::push(double x)
{
   m_num_data_values++;

   // See Knuth TAOCP vol 2, 3rd edition, page 232
   if (m_num_data_values == 1) {
      m_old_mean = m_new_mean = x;
      m_old_variance = 0.0;
   } else {
      m_new_mean = m_old_mean + (x - m_old_mean) / m_num_data_values;
      m_new_variance = m_old_variance + (x - m_old_mean) * (x - m_new_mean);

      // set up for next iteration
      m_old_mean = m_new_mean;
      m_old_variance = m_new_variance;
   }
}
auto RunningStatistic::num_data_values() const -> int
{
   return m_num_data_values;
}
auto RunningStatistic::mean() const -> double
{
   return (m_num_data_values > 0) ? m_new_mean : 0.0;
}
auto RunningStatistic::variance() const -> double
{
   return ((m_num_data_values > 1) ? m_new_variance / (m_num_data_values - 1) : 0.0);
}
auto RunningStatistic::standard_deviation() const -> double
{
   return sqrt(variance());
}
