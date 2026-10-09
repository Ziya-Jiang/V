#ifndef SPEED_INTERFACE_H
#define SPEED_INTERFACE_H

class SpeedInterface{
    public:
        virtual void setStep(float step) = 0;
        virtual void setSliderResol(float resol) = 0;
};
#endif