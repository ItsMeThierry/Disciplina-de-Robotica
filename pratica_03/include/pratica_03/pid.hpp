#ifndef PID_HPP
#define PID_HPP

#include <algorithm>
#include <chrono>

namespace pratica
{

class PID
{
public:
    PID(double kp, double ki, double kd,
        double i_min = -1.0, double i_max = 1.0,
        double out_min = -1.0, double out_max = 1.0)
    : kp_(kp), ki_(ki), kd_(kd),
      i_min_(i_min), i_max_(i_max),
      out_min_(out_min), out_max_(out_max),
      integral_(0.0), prev_error_(0.0), first_run_(true)
    {}

    double compute(double error, double dt)
    {
        if (dt <= 0.0) {
            return 0.0;
        }

        // Proporcional
        double p = kp_ * error;

        // Integral com anti-windup (clamp)
        integral_ += error * dt;
        integral_ = std::clamp(integral_, i_min_, i_max_);
        double i = ki_ * integral_;

        // Derivativo (evita "kick" no primeiro ciclo)
        double d = 0.0;
        if (!first_run_) {
            d = kd_ * (error - prev_error_) / dt;
        } else {
            first_run_ = false;
        }
        prev_error_ = error;

        // Soma e saturação final
        double output = p + i + d;
        return std::clamp(output, out_min_, out_max_);
    }

    void reset()
    {
        integral_ = 0.0;
        prev_error_ = 0.0;
        first_run_ = true;
    }

    // Setters úteis se quiser ajustar ganhos em runtime (via parâmetros)
    void set_gains(double kp, double ki, double kd) { kp_ = kp; ki_ = ki; kd_ = kd; }

private:
    double kp_, ki_, kd_;
    double i_min_, i_max_;
    double out_min_, out_max_;
    double integral_;
    double prev_error_;
    bool first_run_;
};

}  

#endif  