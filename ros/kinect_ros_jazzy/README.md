# Installation

You need to install freenect python bindings

Install dependencies (ubuntu 24.04)
```bash
sudo apt install python3-numpy python3-dev cython3 libusb-1.0-0 libusb-1.0-0-dev
```

Clone and apply a patch
```bash
git clone https://github.com/OpenKinect/libfreenect
cd libfreenect
git checkout 09a1f09
git apply /path/to/CMPSC488FA24Sec2Team1/ros/kinect_ros_jazzy/fix_make_install.patch
```

Build it
```bash
mkdir build && cd build
cmake .. -DBUILD_PYTHON3=ON
sudo make install
```

Fix your python env
```bash
python3 -c 'import freenect' 2>/dev/null || echo 'export PYTHONPATH="/usr/local/lib/python3/dist-packages:$PYTHONPATH"' >> ~/.bashrc
```
