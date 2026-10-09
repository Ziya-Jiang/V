#include "circular_queue.h"

#include <mutex>

// Roughly 40 seconds at 100 Hz. A larger static queue previously reserved
// hundreds of MiB and made memory locking unreliable.
static constexpr int kSaveBufferSize = 4096;
static int queue_head = 0, queue_tail = 0, queue_size = 0;
static std::mutex queue_lock;
static save_data queue_buffer[kSaveBufferSize];

bool queue_insert(const save_data& new_node)
{   
    bool success = false;
    if (queue_lock.try_lock()) {
        if (queue_size < kSaveBufferSize) {
            queue_buffer[queue_head ++] = new_node;
            if (queue_head >= kSaveBufferSize) {
                queue_head = 0;
            }
            queue_size ++;
            success = true;
        }
        queue_lock.unlock();
    }
    return success;
}

bool queue_poptop(save_data* new_node) 
{   
    bool success = false;
    if (queue_lock.try_lock()) {
        if (queue_size > 0) {
            queue_size--;
            *new_node = queue_buffer[queue_tail ++];
            if (queue_tail >= kSaveBufferSize) {
                queue_tail = 0;
            }
            success = true;
        }
        queue_lock.unlock();
    }
    return success;
}

int queue_get_size(){
    std::lock_guard<std::mutex> lock(queue_lock);
    return queue_size;
}
