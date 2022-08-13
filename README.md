# ORB-SLAM3 
### [Link to original ORB-SLAM3's README.md](https://github.com/UZ-SLAMLab/ORB_SLAM3)

This fork incorporates the changes that I find necessary to make it easier and more straightforward to install and run ORB-SLAM3 on Ubuntu 20.04.

## 1. Install

### Dependencies
```
sudo add-apt-repository "deb http://security.ubuntu.com/ubuntu xenial-security main"
sudo apt update

sudo apt-get install build-essential
sudo apt-get install cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev

sudo apt-get install python-dev python-numpy libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libdc1394-22-dev libjasper-dev

sudo apt-get install libglew-dev libboost-all-dev libssl-dev

sudo apt install libeigen3-dev
```
### 1.1 Pangolin:
```
git clone https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin
mkdir build && cd build
cmake ..
make
sudo make install
```
### 1.2 OpenCV

Check the OpenCV version on your computer (required at leat 3.0 as stated in the original `README.md`):
```
python3 -c "import cv2; print(cv2.__version__)" 
```
On a freshly installed Ubuntu 20.04.4 LTS with desktop image, OpenCV 4.2.0 is included so we can skip to the next step. If a newer version is required (>= 3.0), follow the instrucions:
- [General installation instruction](https://docs.opencv.org/4.x/d0/d3d/tutorial_general_install.html). 
- If you want CUDA to be included: [How to install OpenCV 4.5.2 with CUDA 11.2 and CUDNN 8.2 in Ubuntu 20.04](https://gist.github.com/raulqf/f42c718a658cddc16f9df07ecc627be7)

For example, the main commands for OpenCV 4.5.1 without CUDA and other bells and whistles:
```
git clone https://github.com/opencv/opencv
git -C opencv checkout 4.5.1

cd opencv
mkdir build
cd build
cmake ..
make -j4
sudo make install
```

## 2. Build

```
# Clone the repo:
git clone https://github.com/thien94/ORB_SLAM3.git ORB_SLAM3

# Build
cd ORB_SLAM3
chmod +x build.sh
./build.sh
```

## 3. Run examples

[EuRoC datset](https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets):
```
# Download
cd ~
mkdir -p Datasets/EuRoC
cd Datasets/EuRoC/
wget -c http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/machine_hall/MH_01_easy/MH_01_easy.zip
mkdir MH01
unzip MH_01_easy.zip -d MH01/

# Run in mono-inertial mode
./Examples/Monocular-Inertial/mono_inertial_euroc ./Vocabulary/ORBvoc.txt ./Examples/Monocular-Inertial/EuRoC.yaml ~/Datasets/EuRoC/MH01 ./Examples/Monocular-Inertial/EuRoC_TimeStamps/MH01.txt dataset-MH01_monoimu

```


## Changelog:
### 13-Aug-2022
Work with Ubuntu 20.04, no additional installation of OpenCV or C++ required:
- Update CMakeLists.txt to use OpenCV 4.2 mimimum.
- Update CMakeLists.txt to use C++14 instead of C++11.
