#ifndef CONTROL_INTERFACE_H
#define CONTROL_INTERFACE_H

class ControlInterface{
    public:
        virtual void setInFront(bool in_front) = 0;
        virtual bool sendVal() = 0;
};

#endif