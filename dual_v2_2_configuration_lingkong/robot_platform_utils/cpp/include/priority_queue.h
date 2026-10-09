#ifndef PRIORITY_QUEUE_H
#define PRIORITY_QUEUE_H

#include <queue>
#include <chrono>
#include <vector>

template <typename T>
class PriorityQueue{
    public:
        struct QueueData {
            int id = -1;
            int priority = 0;
            uint64_t seq = 0;
            long received_at = 0;
            T data;
        };

        struct QueueDataCompare {
            bool operator()(const QueueData& lhs, const QueueData& rhs) const {
                if (lhs.priority != rhs.priority) {
                    return lhs.priority > rhs.priority;
                }

                if (lhs.seq != rhs.seq) {
                    return lhs.seq < rhs.seq;
                }
                return lhs.received_at < rhs.received_at;
            }
        };

        void push(QueueData data);
        QueueData pop();
        bool empty() const;
        void clear();
        void print() const;
    private:
        std::priority_queue<QueueData, std::vector<QueueData>, QueueDataCompare> queue_;
};

template <typename T>
void PriorityQueue<T>::push(QueueData data){
    queue_.push(std::move(data));
}

template <typename T>
typename PriorityQueue<T>::QueueData PriorityQueue<T>::pop(){
    QueueData top = std::move(const_cast<QueueData&>(queue_.top()));
    queue_.pop();
    return top;
}

template <typename T>
bool PriorityQueue<T>::empty() const{
    return queue_.empty();
}

template <typename T>
void PriorityQueue<T>::clear(){
    queue_ = std::priority_queue<QueueData, std::vector<QueueData>, QueueDataCompare>();
}

template <typename T>
void PriorityQueue<T>::print() const{
    std::priority_queue<QueueData, std::vector<QueueData>, QueueDataCompare> temp_queue = queue_;
    int queue_order = 0;
    while (!temp_queue.empty()){
        QueueData qd = temp_queue.top();
        std::cout << "queue order: " << queue_order << " id: " << qd.id << " priority: " << qd.priority << " seq: " << qd.seq << " received_at: " << qd.received_at << std::endl;
        queue_order++;
        temp_queue.pop();
    }
}

#endif