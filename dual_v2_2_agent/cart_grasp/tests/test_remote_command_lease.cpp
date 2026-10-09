#include "../hardware_patch/remote_command_lease.h"
#include <cassert>
#include <limits>
int main() {
    RemoteCommandLease lease;
    assert(lease.expired(0.));
    lease.start();
    assert(!lease.expired(10.));  // no first command: no trajectory to replay
    assert(lease.accept(10., 10.01));
    assert(!lease.expired(10.49));
    assert(lease.expired(10.51));
    assert(!lease.accept(11., 11.));  // reconnection cannot restart
    lease.start();
    assert(!lease.accept(12., 12.6)); // queued old command is not a heartbeat
    lease.start();
    assert(!lease.accept(14., 13.));
    lease.start();
    assert(lease.accept(15., 15.));
    assert(!lease.accept(14.9, 15.1));
    lease.start();
    assert(!lease.accept(std::numeric_limits<double>::quiet_NaN(), 20.));
    lease.start();
    assert(lease.accept(21., 21.));
    lease.stop();
    assert(!lease.accept(21.1, 21.1));
}
