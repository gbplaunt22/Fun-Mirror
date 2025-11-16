# THIS README IS DEPERCATED IN THE CONTEXT OF THIS REPOSITORY!


# Fun-Mirror

This is a project where I will attempt to use my Xbox 360 Kinect with a Raspberry Pi, hooked up to a monitor with a two-way mirror in front, or a "Smart Mirror". This is a big WIP. This desc needs to be filled!



Note that libfreenect is an essential part of this project. Please visit their repo https://github.com/OpenKinect/libfreenect for troubleshooting and extra information on dependencies and CMake list options. I will be scripting with Java/Python, but other languages are permitted. 

Also, note that this is my first project repo, and **I do not fully understand dependencies/installation for machines other than my own!** Please install dependencies with scrutiny in mind. 

## Raspberry Pi Setup

These steps assume a fresh Raspberry Pi OS with an Xbox 360 Kinect V1 plugged in via USB. 

Install system dependencies:

```bash
sudo apt update
sudo apt install -y \
  git cmake build-essential \
  libusb-1.0-0-dev freeglut3-dev libopengl-dev \
  python3-dev python3-numpy cython3
```

## Build and install libfreenect
Please refer to the libfreenect repo for better instructions!

```bash
cd ~
git clone https://github.com/OpenKinect/libfreenect.git
cd libfreenect
mkdir build && cd build
cmake .. -DBUILD_EXMAPLES=ON -DBUILD_PYTHON3=ON
make -j4
sudo make install
sudo ldconfig
```

## Install Python Bindings:

```bash
cd ~/libfreenect/wrappers/python
sudo python3 setup.py install
```

## Test
```bash
python3 -c "import freenect; print('freenect imported OK')"
```

## Build the native head tracker

The `native/headtrack.c` helper uses `lrint` from libm, so make sure to link
against `-lm` (either explicitly or via `pkg-config`). From the repo root:

```bash
gcc native/headtrack.c -o native/headtrack \
  $(pkg-config --cflags --libs libfreenect) -lm
```

If your `pkg-config` already expands to a flag list that includes `-lm`, you do
not need to add the final `-lm` again.
----------------------------------------------------------------------
## Java Wrapper usage explanation

This repo already includes OpenKinect.jar and jna-5.13.0.jar
As long as you have a JDK installed, you can compile and run Java programs that talk to the Kinect.

## Example: running a Java class on the pi
Assuming you have a class like mirror.SimpleKinect under src/

```bash
cd ~/Fun-Mirror

#compile
javac -cp lib/OpenKinect.jar:lib/jna-5.13.0.jar src/mirror/SimpleKinect.java

# Run (With Kinect plugged in)
java -cp lib/OpenKinect.jar:lib/jna-5.13.0.jar:src mirror.SimpleKinect
```

The same JARs are used via your IDE by adding:
+ lib/OpenKinect.jar
+ lib/jna-5.13.0.jar

to the project's build path.

----------------------------------------------------
## Java Wrapper Setup
**THESE ARE MAINTAINER NOTES!!! YOU WILL NOT NEED TO DO THIS!!!**

Building the Java wrapper with Maven:

```bash
cd ~/libfreenect/wrappers/java
nano pom.xml
```

In the maven-compiler-plugin section, make sure the Java version is at least 1.8:

```xmi
<plugin>
  <groupId>org.apache.maven.plugins</groupId>
  <artifactId>maven-compiler-plugin</artifactId>
  <version>2.3.2</version>
  <configuration>
    <source>1.8</source>
    <target>1.8</target>
  </configuration>
</plugin>
```

Save and exit, then build the JAR without running the tests (I'm not 100% why, but it works for me! :D)

```bash
cd ~/libfreenect/wrappers/java
mvn -DskipTests package
```

Now you should see a JAR in target/

```bash
ls target
# freenect-1.0.jar (name may vary)
```

## Copy the wrapper + JNA jars into Fun-Mirror repo
```bash
# From the Java wrapper build dir:
cd ~/libfreenect/wrappers/java/target

mkdir -p ~/Fun-Mirror/lib

# Copy the freenect Java wrapper
cp freenect-1.0.jar ~/Fun-Mirror/lib/OpenKinect.jar

# Copy the JNA jar from the system install
cp /usr/share/java/jna-5.13.0.jar ~/Fun-Mirror/lib #(jna jar name may vary!)
```






