# Installations

You need to install freenect python bindings

install dependencies (ubuntu 24.04)
```bash
sudo apt install python3-numpy python3-dev cython3 libusb-1.0-0 libusb-1.0-0-dev
```

clone and build it
```bash
git clone https://github.com/OpenKinect/libfreenect
cd libfreenect
mkdir build && cd build
cmake .. -DBUILD_PYTHON3=ON
sudo make install
```
