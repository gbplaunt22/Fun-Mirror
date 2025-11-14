# Fun-Mirror

This is a project where I will attempt to use my Xbox360 kinnect with a rasberry pi, hooked up to a monitor with a two-way mirror infront, or a "Smart Mirror". This is a big WIP. This desc needs to be filled!



Note that libfreenect is an essential part of this project. Please visit their repo \[(https://github.com/OpenKinect/libfreenect)] for troubleshooting and extra information on dependices and cmake list options. I will be scripting with java/python, but other languages are permitted. 



\## \*\*Raspberry Pi Setup\*\*



Install system dependencies:



```bash

sudo apt update

sudo apt install -y \\

\- git cmake build-essential \\

\- libusb-1.0-0-dev freeglut3-dev libopengl-dev \\

\- python3-dev python3-numpy cython3

```



\## Build and install libfreenect



```bash

cd ~

git clone https://github.com/OpenKinect/libfreenect.git

cd libfreenect

mkdir build \&\& cd build

cmake .. -DBUILD\_EXMAPLES=ON -DBUILD\_PYTHON3=ON

make -j4

sudo make install

sudo ldconfig

```



\## Install Python Bindings:



```bash

cd ~/libfreenect/wrappers/python

sudo python3 setup.py install

```



\## Test

```bash

python3 -c "import freenect; print('freenect imported OK')"

```





