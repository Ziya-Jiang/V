#pragma once
#include <cmath>

// GUI-thread lease. Use the receiver's monotonic arrival time, not the time a
// queued command is eventually consumed. A timeout latches until explicit Start.
class RemoteCommandLease {
public:
    void start() { seen_ = false; stopped_ = false; last_ = 0.; }
    void stop() { stopped_ = true; }
    bool expired(double now) {
        if (!std::isfinite(now) || (seen_ && now - last_ > timeout_)) stopped_ = true;
        return stopped_;
    }
    bool accept(double received, double now) {
        if (expired(now) || !std::isfinite(received) || received < 0. ||
            received > now || now - received > timeout_) {
            stopped_ = true;
            return false;
        }
        if (seen_ && received < last_) {
            stopped_ = true;
            return false;
        }
        seen_ = true;
        last_ = received;
        return true;
    }
private:
    const double timeout_ = .5;
    bool seen_ = false;
    bool stopped_ = true;
    double last_ = 0.;
};
