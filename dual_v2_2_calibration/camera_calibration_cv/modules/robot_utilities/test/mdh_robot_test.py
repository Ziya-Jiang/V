import numpy
from matplotlib import pyplot as plt
from scripts.robot_utilities import load_MDH_robot

if __name__ == "__main__":
    config = "config/cuarm_v2_2_mdh.json"
    robot = load_MDH_robot(config)
    q = numpy.array([0,30,0,60,90,30,0],dtype="float64")*numpy.pi/180
    q1 = numpy.array([0,20,50,70,90,30,0],dtype="float64")*numpy.pi/180
    #forward kinematics
    T1 = robot.FK(q1)
    
    #Inverse kinematics
    q2 = robot.IK(T1[:3,:3], T1[:3,3], q)
    
    #compare result
    T2 = robot.FK(q2)
    print("Equal result: ", robot.equal(T1, T2, eps=0.001))
    
    #draw robot
    fig = plt.figure()
    robot.draw(q, fig)
    plt.show()