#ifndef IMU_ADAFRUIT_BNO055_RUNNINGSTATISTIC_HPP
#define IMU_ADAFRUIT_BNO055_RUNNINGSTATISTIC_HPP

class RunningStatistic
{
 public:
   RunningStatistic() = default;

   void clear();
   void push(double x);

   [[nodiscard]] auto num_data_values() const -> int;
   [[nodiscard]] auto mean() const -> double;
   [[nodiscard]] auto variance() const -> double;
   [[nodiscard]] auto standard_deviation() const -> double;

 private:
   int m_num_data_values{0};
   double m_old_mean{0.0};
   double m_new_mean{0.0};
   double m_old_variance{0.0};
   double m_new_variance{0.0};
};

#endif // IMU_ADAFRUIT_BNO055_RUNNINGSTATISTIC_HPP
