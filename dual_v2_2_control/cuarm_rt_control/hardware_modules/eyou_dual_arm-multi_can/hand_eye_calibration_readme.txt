在远程连接的terminal中：
ssh vlmdemo@10.30.13.21
pw:vlm-demo
如何连接如图所示
conda activate robotram2
run test_display_zed.py按空格键保存图片
un cam_single_calibration.py进行标定
修改cam_single_calibration.py中的exclude_list中的参数，然后重新标定直到标定结果小于0.008
在标定结果中保存内外参到result.txt文件中，外参的T一般选取第一种方法或者第二种方法
run damp_json.py

在本地termainal中：
sudo su
password:1
cd ~/eyou_dual_arm/eyou_dual_arm-multi_can/build
export LD_LIBRARY_PATH=/home/cuarm/eyou_dual_arm/lib/eyou_harmonic/x86:$LD_LIBRARY_PATH
./camera_calibration_record_new_point 运行机械臂零力矩控制模式 按任意按键保存当前关节角度值到joint_pos_data.txt（需同时打开相机观察当前图片是否为good image）
./joint_position_control_from_file_revised 读取关节位置文件(joint_pos_data.txt) 运行到指定角度后自动保存当前T
