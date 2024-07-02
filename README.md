# VIVE Calibration

## Setting udev rules

Copy  `60-HTC-Vive-perms.rules` to the folder `/etc/udev/rules.d` and run:

```
sudo udevadm control --reload-rules && sudo udevadm trigger
```

## Install OpenVR SDK

```
cd ~
mkdir librarires && cd libraries
git clone https://github.com/ValveSoftware/openvr.git
cd openvr
mkdir build && cd build
cmake ..
make 
sudo make install
```





