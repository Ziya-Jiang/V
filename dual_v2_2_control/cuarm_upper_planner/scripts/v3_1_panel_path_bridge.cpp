// Calls the deployed controller source, including duplicate filtering and retiming.
// This is an offline sampler, never a robot command interface.
#include "interpolation.h"
#include <exception>
#include <cmath>
#include <string>
static thread_local std::string error;
extern "C" const char* v31_path_error() { return error.c_str(); }
// Stateful offline RAW path: runs the same InterpolatorFactory::kNone filter.
struct RawFilter {
    int joints;
    VectorXd current;
    InterpolatorFactory factory;
    RawFilter(int n, const double* q, const double* limits, double dt, double time):
        joints(n), current(Eigen::Map<const VectorXd>(q,n)),
        factory(InterpolationMethod::kQuintic, static_cast<float>(dt), n, "v31_raw_test") {
        factory.updateParam(InterpolationMethod::kNone, static_cast<float>(time), 0.f,
                            Eigen::Map<const VectorXd>(limits,n));
    }
};
extern "C" void* v31_raw_create(int n,const double* q,const double* limits,double dt,double time) {
    try { return new RawFilter(n,q,limits,dt,time); }
    catch (const std::exception& e) { error=e.what(); return nullptr; }
}
extern "C" void v31_raw_destroy(void* handle) { delete static_cast<RawFilter*>(handle); }
extern "C" int v31_raw_step(void* handle,const double* target,int ticks,double* output) {
    try {
        auto& r=*static_cast<RawFilter*>(handle);
        VectorXd goal=Eigen::Map<const VectorXd>(target,r.joints);
        for(int i=0;i<ticks;++i) {
            r.current=r.factory.getNextPos(r.current,goal);
            Eigen::Map<VectorXd>(output+i*r.joints,r.joints)=r.current;
        }
        return ticks;
    } catch (const std::exception& e) { error=e.what(); return -1; }
}
extern "C" int v31_sample_path(int joints, int count, const double* start,
    const double* points, double interval, const double* limits, double sample_dt,
    int capacity, double* output, double* duration) {
    try {
        if (joints < 1 || count < 1 || interval <= 0 || sample_dt <= 0) return -1;
        VectorXd q = Eigen::Map<const VectorXd>(start, joints);
        VectorXd vmax = Eigen::Map<const VectorXd>(limits, joints);
        std::vector<VectorXd> waypoints, speeds;
        std::vector<float> times;
        for (int i=0; i<count; ++i) {
            waypoints.emplace_back(Eigen::Map<const VectorXd>(points+i*joints, joints));
            speeds.push_back(vmax); times.push_back(static_cast<float>(interval));
        }
        InterpolatorFactory factory(InterpolationMethod::kQuintic,
                                    static_cast<float>(sample_dt), joints, "v31_validation");
        factory.updateParam(InterpolationMethod::kQuinticPath, 5.0f, 0.0f, vmax);
        factory.getNextPos(q, waypoints, times, speeds); // same zero v/a defaults
        *duration = factory.getInterpolationT();
        int n = static_cast<int>(std::ceil(*duration/sample_dt))+1;
        if (n > capacity || !std::isfinite(*duration)) return -2;
        for (int i=0; i<n; ++i) {
            auto value = factory.getPosAtTime(static_cast<float>(std::min(i*sample_dt,*duration)));
            Eigen::Map<VectorXd>(output+i*joints,joints) = value;
        }
        return n;
    } catch (const std::exception& e) { error=e.what(); return -3; }
}
