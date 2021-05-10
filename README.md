## Kleinvoet - Firmware

Firmware for _Kleinvoet_ hardware project. Written in embedded C and compiled 
with ARM GCC.

### Platform (Tested)
- Kleinvoet (STM32F446RC/ET)

### Setup

#### Formatting SD Card
The SD card needs to be formatted using a modern version of `mkfs.vfat` and `parted`. The following commands were used:

[OPTIONAL]
Zeroing the SD card helps that the controller doesn't have to clear pages wile 
running. This improves performance but drastically diminishes the lif span of 
the medium. Optionally you can specify the number of blocks(`count=N`) if you
don't want to zero the whole drive. Good for testing purposes.
```shell script
sudo dd if=/dev/zero of=/dev/sdX bs=4096 status=progress
```

```shell script
# Create an DOS partition table
sudo parted /dev/sdX --script --align optimal -- mklabel msdos
# Specify partition as 'FAT32' with 1MiB clearance
sudo parted /dev/sdX --script --align optimal -- mkpart primary fat32 1MiB 100%
# Format partition as 'FAT32'
sudo mkfs.vfat -F32 /dev/sdXX
# Check to make sure everything is as it should be
sudo parted /dev/sdX --script -- print
```

#### Requirements
- CubeMx (>= v6.0.1)
- OpenOCD (>= v0.10.0)
- gdb (>= 9.1-6)
- arm-none-eabi-gcc (>= v9.2.0-4)

#### OpenOCD
If using an ST-Link V3 then OpenOCD requires _stlink.cfg_ which isn't shipped 
with most linux distros. It is thus recommended that one compiles it from 
scratch. [See this tutorial for a guide on how to compile](https://mbd.kleier.net/integrating-st-link-v3.html)

##### Requirements
###### Ubuntu
```shell script
sudo apt install make libtool pkg-config autoconf automake texinfo libusb libusb-dev
```

###### Fedora
```shell script
sudo dnf install make libtool pkg-config autoconf automake texinfo libusb libusb-devel
```

##### Building
```shell script
git clone git://git.code.sf.net/p/openocd/code openocd
cd openocd
git pull
./bootstrap
./configure
make
sudo make install
```

#### SVD
The SVD file was downloaded from [this link](https://www.st.com/resource/en/svd/stm32f4_svd.zip). Currently using version 1.2

#### IDE
Recommended IDE would be between setting up and amazing _VIM_ experience. (DIY) 
Or using _CLion_, for most of this project _CLion 2020.2_ was used. _CubeIDE_ 
seems to also be a good alternative if one is struggling with getting all the 
tooling to work. _CubeIDE_ also has support for OpenOCD but uses a shelf shipped
version.

#### libusb_open() failed with LIBUSB_ERROR_ACCESS 
This is due to a udev rule problem. Installing _CubeIDE_ installs these rules. 
But creating them manually should work too. Place the appropriate file in the 
`/etc/udev/rules.d` directory.

**49-stlinkv3.rules**
```
# ST_PKG_VERSION 1.0.2-3
# stlink-v3 boards (standalone and embedded) in usbloader mode and standard (debug) mode

SUBSYSTEMS=="usb", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="374d", \
    MODE="660", GROUP="plugdev", TAG+="uaccess", ENV{ID_MM_DEVICE_IGNORE}="1", \
    SYMLINK+="stlinkv3loader_%n"

SUBSYSTEMS=="usb", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="374e", \
    MODE="660", GROUP="plugdev", TAG+="uaccess", ENV{ID_MM_DEVICE_IGNORE}="1", \
    SYMLINK+="stlinkv3_%n"

SUBSYSTEMS=="usb", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="374f", \
    MODE="660", GROUP="plugdev", TAG+="uaccess", ENV{ID_MM_DEVICE_IGNORE}="1", \
    SYMLINK+="stlinkv3_%n"

SUBSYSTEMS=="usb", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="3753", \
    MODE="660", GROUP="plugdev", TAG+="uaccess", ENV{ID_MM_DEVICE_IGNORE}="1", \
    SYMLINK+="stlinkv3_%n"
```

**49-stlinkv2-1.rules**
```
# ST_PKG_VERSION 1.0.2-3
# stm32 nucleo boards, with onboard st/linkv2-1
# ie, STM32F0, STM32F4.

SUBSYSTEMS=="usb", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="374b", \
    MODE="660", GROUP="plugdev", TAG+="uaccess", ENV{ID_MM_DEVICE_IGNORE}="1", \
    SYMLINK+="stlinkv2-1_%n"

SUBSYSTEMS=="usb", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="3752", \
    MODE="660", GROUP="plugdev", TAG+="uaccess", ENV{ID_MM_DEVICE_IGNORE}="1", \
    SYMLINK+="stlinkv2-1_%n"
```
