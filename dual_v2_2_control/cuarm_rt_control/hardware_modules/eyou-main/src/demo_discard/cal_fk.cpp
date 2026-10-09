#include <iostream>
#include <thread>
#include <iomanip>
#include <csignal>
#include <math.h>
#include <unistd.h>

#include "kinect.h"
#include "torque_sensor.h"
#include "eyou_joints.h"
#include "kinematics.h"



#define CLAMP(x,low_lim,high_lim) ((x)<(low_lim)?low_lim:((x)>(high_lim)?(high_lim):(x)))
//#define PULSE2DEG 360/65536 
uint32_t motor_id[14] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14};
bool enable_motor_index[14] = {1, 1, 1, 1, 1, 1, 1, 
                                0, 0, 0, 0, 0, 0, 0};//indicate which joint is enbled
uint32_t sensor_id[14] = {0x15, 0x16, 0x17, 0x18, 0x19, 0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28};
double joint_pos[14] = {0};
EyouJoints dual_arm(motor_id, enable_motor_index);
FILE *fwrite_data = fopen("data.txt", "wb"); // save data
hint32 joint_pos_pulse[14];
double left_joint_pos[7] = {0};
int sign[14] = {1, -1, 1, 1, 1, 1, 1,
                1, 1, 1, -1, 1, -1, -1};
int gear_ratio[14] = {121, 121, 101, 101, 101, 101, 101, 121, 121, 101, 101, 101, 101, 101};
bool exit_requested = false;
hint32 m_init_mdhpos[14] = {
    2326350 + static_cast<int>((0. / PULSE2DEG) * gear_ratio[0]),
    2963120 + static_cast<int>((0.329 / PULSE2DEG) * gear_ratio[1]),
    1123510 + static_cast<int>((-3.788 / PULSE2DEG) * gear_ratio[2]),
    -1727964 + static_cast<int>((-4.132 / PULSE2DEG) * gear_ratio[3]),
    138168 + static_cast<int>((2.977 / PULSE2DEG) * gear_ratio[4]),
    -1304367 + static_cast<int>((0.863 / PULSE2DEG) * gear_ratio[5]),
    -48581 + static_cast<int>((5.622 / PULSE2DEG) * gear_ratio[6]),

    1974960 + static_cast<int>((0. / PULSE2DEG) * gear_ratio[0]),
    // (-3909510 + static_cast<int> ((-10.557 / PULSE2DEG) * gear_ration[1])),
    -3909510 + static_cast<int>((-10.557 / PULSE2DEG) * gear_ratio[1]),
    1063830 + static_cast<int>((-2.855 / PULSE2DEG) * gear_ratio[2]),
    1771340 + static_cast<int>((-2.462 / PULSE2DEG) * gear_ratio[3]),
    -1520960 + static_cast<int>((-3.738 / PULSE2DEG) * gear_ratio[4]),
    1999800 + static_cast<int>((-4.831 / PULSE2DEG) * gear_ratio[5]),
    1965460 + static_cast<int>((5.855 / PULSE2DEG) * gear_ratio[6])};

void signal_handler(int signal)
{
    exit_requested = true;
}



int main()
{   
    int n = 28;
    double jpos[n][7] = {
        {-4.907109, 1.741548, -1.834449, 1.089414, 0.619057, 0.860659, 0.230481} , 
        {-4.872306, 1.741548, -1.491221, 1.089414, 0.540345, 0.915020, 0.230672} , 
        {-4.872498, 1.741548, -1.491221, 1.089414, 0.307467, 0.555876, 0.216483} , 
        {-4.963099, 1.741548, -1.251632, 1.089414, 0.304303, 0.664981, 0.216291} , 
        {-4.961757, 1.741548, -1.254029, 1.089414, 0.147742, 0.913006, -0.225879} , 
        {-4.960798, 1.741548, -1.253262, 1.089414, 0.647532, 0.864398, -0.225879} , 
        {-4.959743, 1.741643, -0.916170, 1.089510, 0.343132, 0.804573, -0.213415} , 
        {-4.957922, 1.741548, -0.808887, 1.089414, 0.006136, 0.746761, -0.184461} , 
        {-4.942678, 1.741356, -0.811284, 1.106384, 0.024256, 0.567573, -0.913773} , 
        {-4.928489, 1.741356, -0.595664, 1.109547, -0.439869, 0.659228, 0.198459} , 
        {-4.927434, 1.741260, -0.858358, 1.115875, -0.004314, 0.975037, 0.087437} , 
        {-5.167694, 1.740109, -1.286051, 1.113670, 0.209005, 0.487902, -0.161739} , 
        {-5.187923, 1.741452, -1.011756, 1.115492, 0.219455, 0.635739, -0.139976} , 
        {-5.075655, 1.741356, -0.686361, 1.092003, -0.193090, 0.683293, -0.121568} , 
        {-5.330104, 1.741356, -0.965257, 1.058543, -0.161068, 0.544276, -0.216387} , 
        {-5.231258, 1.741452, -0.573996, 1.059885, 0.056374, 0.824131, -0.181393} , 
        {-5.098473, 1.740781, -0.602087, 0.974174, -0.264899, 0.585693, -0.258380} , 
        {-5.076901, 1.740781, -0.633342, 0.958546, -0.411874, 0.594322, -0.664597} , 
        {-5.081407, 1.741068, -0.634013, 0.971777, -0.272665, 0.482725, -0.969380} , 
        {-5.090132, 1.741164, -0.628165, 0.988171, 0.051292, 0.371032, -0.941385} , 
        {-5.192525, 1.295639, -1.096317, 1.180015, 0.187625, 0.860947, 0.196925} , 
        {-4.924366, 1.380103, -0.957396, 1.179823, -0.091272, 0.919430, 0.412545} , 
        {-5.122921, 1.594765, -1.083662, 1.153554, -0.280910, 0.472754, 0.316863} , 
        {-5.044975, 1.593231, -1.377035, 1.151444, 0.221564, 0.695469, 0.155507} , 
        {-4.860897, 1.740205, -1.341562, 1.069376, 0.258188, 0.573709, 0.389919} , 
        {-5.305368, 1.231115, -0.994020, 1.069472, -0.118117, 0.717807, 0.265570} , 
        {-5.720214, 1.224404, -0.592117, 1.068322, -0.277651, 0.710041, 0.260297} , 
        {-5.110361, 1.476744, -0.561149, 1.067171, -0.736598, 0.682046, 0.093285} , 
    };
    //Get joint data
    for (int i = 0;i<n;++i)
    {  
        double left_joint_pos[7];
        double left_l7_world[6];
        for (int j=0; j<7; j++){
            left_joint_pos[j] = jpos[i][j];
            if (j==0){
                left_joint_pos[j] += 2*PI;
                std::cout <<i <<", "<<j <<", "<<left_joint_pos[j]<<"\n";
            }
        }
        kinematics::forwardKinematics(0, left_joint_pos, left_l7_world);

        //Write joint position
        for (int i = 0; i<7; ++i)
        {
            fprintf(fwrite_data, "%lf ", left_joint_pos[i]);
        }

        //Write task position
        for (int i = 0; i<6; ++i){
            fprintf(fwrite_data, "%lf ", left_l7_world[i]);
        }
        fprintf(fwrite_data, "\n");
    }

    fclose(fwrite_data);

    return 0;
}