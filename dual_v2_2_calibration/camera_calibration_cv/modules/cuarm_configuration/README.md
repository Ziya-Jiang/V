
# How to build 
### Build cpp test
```bash
cd share/cpp/
mkdir build & cd build
cmake -DBUILD_CPP_TESTS=ON ..
make
```

### Build python lib
#### 1. Install pybind11
```bash
pip install pybind11
```
#### 2. Build from cpp source
```bash
cd share/cpp/
mkdir build & cd build
cmake -DBUILD_PYTHON_LIB=ON -DPython3_EXECUTABLE=$(which python3) ..
make
```