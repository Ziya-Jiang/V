#!/bin/bash

#Tested on Jetson Orin, Unbuntu 22.04, Linux kernel 5.15.148-tegra (3rd Feb 2025)
platform='unknown'
unamestr=$(uname)
architecture=$(uname -m)
if  [ $unamestr == 'Linux' ] && [ $architecture == 'aarch64' ] ; then
platform='linux-aarch64'
echo "$platform"

#Install yaml-cpp
cd ~ && git clone https://github.com/jbeder/yaml-cpp.git
cd yaml-cpp && mkdir build && cd build
cmake .. && make && sudo make install

#Install pinocchio
sudo apt-get install liburdfdom-dev libeigen3-dev libboost-all-dev
sudo apt-get install python3-pip
#eigenpy > 3.5.1 requires cmeel_boost=1.87 which is depended on numpy=2.2(given python3.10 on Ubuntu22.04). This is conflicted with opencv4.12 wchihc require numpy<=2.0 (Tried on 3rd Feb2025).
pip install eigenpy==3.5.1 
cd ~ && git clone https://github.com/stack-of-tasks/pinocchio.git
#Remove "2.7.10" from add_project_dependency(eigenpy 2.7.10 REQUIRED) manually
cd pinocchio && mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local 
make -j4
sudo make install

#Add pathadd() function for easy path add
cat <<'EOF'>>~/.bashrc
pathadd() {
    if [ -d "$1" ] && [[ ":${!2}:" != *":$1:"* ]]; then
        export $2="$1${!2:+":${!2}"}"
    fi
}
EOF

#Add pinocchio environment variables to ~/.bashrc
python_version=$(python -V 2>&1 | grep -Po '(?<=Python )[0-9]\.[0-9]')
echo "$python_version"
cat <<'EOF'>>~/.bashrc
#Pinocchio config
pathadd /usr/local/bin PATH
pathadd /usr/local/lib/pkgconfig PKG_CONFIG_PATH
pathadd /usr/local/lib LD_LIBRARY_PATH
pathadd /usr/local/lib/python${python_version}/site-packages PYTHONPATH
pathadd /usr/local CMAKE_PREFIX_PATH
EOF

# # Modify rt_runtime from 0 to 950000(for ubuntu20.04)
# echo 950000 >/sys/fs/cgroup/cpu/user.slice/cpu.rt_runtime_us
