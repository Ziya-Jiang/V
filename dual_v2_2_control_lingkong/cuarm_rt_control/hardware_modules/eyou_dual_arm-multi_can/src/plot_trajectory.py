import sys
import subprocess

def generate_gnuplot_commands(control_mode, file_path, plot_pos=False, plot_vel=False, plot_tor=False, plot_error=False):
    """生成gnuplot命令字符串"""
    commands = []
    
    # 设置通用样式
    commands.append("set grid")
    commands.append("set key outside center top horizontal")
    
    if control_mode == "pos_control":
        # 位置控制模式 - 原始格式
        # 列索引计算：(电机索引 * 4) + 列偏移
        #   期望位置：i*4 + 1
        #   实际位置：i*4 + 2
        #   速度：i*4 + 3
        #   力矩：i*4 + 4
        
        # 位置图
        if plot_pos:
            pos_cmd = "plot "
            for i in range(14):  # 最多14个电机
                target_col = i * 6 + 1
                actual_col = i * 6 + 2
                pos_cmd += f'"{file_path}" u 0:{target_col} w l t "M{i+1} Target", '
                #pos_cmd += f'"" u 0:{actual_col} w l dt 2 t "M{i+1} Actual", '
                pos_cmd += f'"" u 0:{actual_col} w l t "M{i+1} Actual", '
            commands.append(pos_cmd.rstrip(', '))  # 移除最后一个逗号
            commands.append("")  # 空行分隔
        
        # 速度图
        if plot_vel:
            
            vel_cmd = "plot "
            for i in range(14):  # 最多14个电机
                vel_col = i * 4 + 3
                vel_cmd += f'"{file_path}" u 0:{vel_col} w l t "M{i+1} Velocity", '
            
            commands.append(vel_cmd.rstrip(', '))  # 移除最后一个逗号
            commands.append("")  # 空行分隔
        
        # 力矩图
        if plot_tor:
           
            tor_cmd = "plot "
            for i in range(14):  # 最多14个电机
                tor_col = i * 4 + 4
                tor_cmd += f'"{file_path}" u 0:{tor_col} w l t "M{i+1} Torque", '
            
            commands.append(tor_cmd.rstrip(', '))  # 移除最后一个逗号
    
    elif control_mode == "vel_control":
        # 速度控制模式 - 新格式
        # 列索引计算：(电机索引 * 4) + 列偏移
        #   实际位置：i*4 + 1
        #   期望速度：i*4 + 2
        #   实际速度：i*4 + 3
        #   位置误差：i*4 + 4
        
        # 实际位置图
        if plot_pos:
            
            pos_cmd = "plot "
            for i in range(14):  # 最多14个电机
                actual_pos_col = i * 4 + 1
                pos_cmd += f'"{file_path}" u 0:{actual_pos_col} w l t "M{i+1} Actual Pos", '
            
            commands.append(pos_cmd.rstrip(', '))  # 移除最后一个逗号
            commands.append("")  # 空行分隔
        
        # 速度图
        if plot_vel:
            
            vel_cmd = "plot "
            for i in range(14):  # 最多14个电机
                target_vel_col = i * 4 + 2
                actual_vel_col = i * 4 + 3
                vel_cmd += f'"{file_path}" u 0:{target_vel_col} w l t "M{i+1} Target Vel", '
                vel_cmd += f'"" u 0:{actual_vel_col} w l t "M{i+1} Actual Vel", '
            
            commands.append(vel_cmd.rstrip(', '))  # 移除最后一个逗号
            commands.append("")  # 空行分隔
        
        # 位置误差图
        if plot_error:
           
            error_cmd = "plot "
            for i in range(14):  # 最多14个电机
                error_col = i * 4 + 4
                error_cmd += f'"{file_path}" u 0:{error_col} w l t "M{i+1} Pos Error", '
            
            commands.append(error_cmd.rstrip(', '))  # 移除最后一个逗号
    
    return commands

def main():
    # 解析命令行参数
    if len(sys.argv) < 4:
        print("Usage:")
        print("  python plot_motors.py <control_mode> <data_file> [options]")
        print("Control modes:")
        print("  pos_control - Position control mode (original format)")
        print("  vel_control - Velocity control mode (new format)")
        print("Options for pos_control mode:")
        print("  --pos : Plot position data")
        print("  --vel : Plot velocity data")
        print("  --tor : Plot torque data")
        print("Options for vel_control mode:")
        print("  --pos    : Plot actual position data")
        print("  --vel    : Plot velocity data")
        print("  --error  : Plot position error data")
        print("Examples:")
        print("  python plot_motors.py pos_control data.txt --pos")
        print("  python plot_motors.py vel_control data.txt --pos --vel --error")
        sys.exit(1)
    
    control_mode = sys.argv[1]
    file_path = sys.argv[2]
    
    # 检查控制模式是否有效
    if control_mode not in ["pos_control", "vel_control"]:
        print(f"Error: Invalid control mode '{control_mode}'. Must be 'pos_control' or 'vel_control'")
        sys.exit(1)
    
    # 提取绘图选项
    plot_options = sys.argv[3:]
    plot_pos = '--pos' in plot_options
    plot_vel = '--vel' in plot_options
    plot_tor = '--tor' in plot_options
    plot_error = '--error' in plot_options
    
    # 在速度控制模式下，扭矩选项无效
    if control_mode == "vel_control" and plot_tor:
        print("Warning: Torque data not available in velocity control mode. Ignoring --tor option.")
        plot_tor = False
    
    # 在位置控制模式下，误差选项无效
    if control_mode == "pos_control" and plot_error:
        print("Warning: Position error data not available in position control mode. Ignoring --error option.")
        plot_error = False
    
    # 检查是否选择了任何有效选项
    valid_options = plot_pos or plot_vel or plot_tor or plot_error
    if not valid_options:
        print("Error: Must specify at least one valid plot option for the selected control mode")
        if control_mode == "pos_control":
            print("Valid options: --pos, --vel, --tor")
        else:
            print("Valid options: --pos, --vel, --error")
        sys.exit(1)
    
    # 生成gnuplot命令
    gnuplot_commands = generate_gnuplot_commands(
        control_mode, file_path, 
        plot_pos, plot_vel, plot_tor, plot_error
    )
    
    # 执行gnuplot命令
    try:
        # 启动gnuplot进程
        process = subprocess.Popen(
            ['gnuplot', '-persist'],
            stdin=subprocess.PIPE,
            text=True
        )
        
        # 发送命令
        for cmd in gnuplot_commands:
            process.stdin.write(cmd + '\n')
        
        # 刷新并关闭输入
        process.stdin.flush()
        process.stdin.close()
        
        # 等待用户关闭窗口
        print(f"Plots displayed in {control_mode} mode. Close plot windows to exit...")
        process.wait()
    
    except FileNotFoundError:
        print("Error: gnuplot not found. Please install gnuplot:")
        print("  Linux: sudo apt-get install gnuplot")
        print("  macOS: brew install gnuplot")
        print("  Windows: Download from https://sourceforge.net/projects/gnuplot/")
        sys.exit(1)

if __name__ == "__main__":
    main()