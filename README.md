# Kleinvoet - Firmware

[![DOI](https://zenodo.org/badge/284653068.svg)](https://zenodo.org/badge/latestdoi/284653068)

Firmware for _Kleinvoet_ hardware project. Written in embedded C and compiled
with ARM GCC.

## Platform (Tested)
- Kleinvoet (STM32F446RC/ET)

## Setup
### Formatting SD Card
The SD card needs to be formatted using a modern version of `mkfs.vfat` and
`parted`. The following commands were used:

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

### Requirements
- CubeMx (>= v6.0.1)
- OpenOCD (>= v0.10.0)
- gdb (>= 9.1-6)
- arm-none-eabi-gcc (>= v9.2.0-4, < v11.2)

Recommend using [ARM Build tools](https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads) from their website, as these are more up to date.

#### Ubuntu
```shell script
sudo apt install make libtool pkg-config autoconf automake texinfo libusb libusb-dev
```

#### Fedora
```shell script
sudo dnf install make libtool pkg-config autoconf automake texinfo libusb libusb-devel
```

### OpenOCD
If using an ST-Link V3 then OpenOCD requires _stlink.cfg_ which isn't shipped
with most linux distros. It is thus recommended that one compiles it from
scratch. [See this tutorial for a guide on how to compile](https://mbd.kleier.net/integrating-st-link-v3.html)

Alternatively make use of the STM32 fork of OpenOCD:

```shell script
git clone https://github.com/STMicroelectronics/OpenOCD.git
cd OpenOCD
git pull
# [Recommended] Optionally checkout a specific version
git checkout v0.X.Y
./bootstrap
./configure --enable-stlink
make
sudo make install
```

### SVD
The SVD file was downloaded from
[this link](https://www.st.com/resource/en/svd/stm32f4_svd.zip).
Currently using version 1.2

### IDE
Recommended IDE would be between setting up and amazing _VIM_ experience. (DIY)
Or using _CLion_, for most of this project _CLion 2020.2_ was used. _CubeIDE_
seems to also be a good alternative if one is struggling with getting all the
tooling to work. _CubeIDE_ also has support for OpenOCD but uses a shelf shipped
version.

#### Command-line Tools
To build a release run, replacing `$BUILD_DIR` with the target output directory 
of your choice:
```shell
cmake --build $BUILD_DIR --target all -- -j $(nproc)
```
To flash an `*.elf` file using `openocd` run:
```shell
openocd -f OCD_kleinvoet.cfg -c "tcl_port disabled" -c "gdb_port disabled" -c "tcl_port disabled" -c "program \"$BUILD_DIR/firmware.elf\"" -c reset -c shutdown
```

### libusb_open() failed with LIBUSB_ERROR_ACCESS
This is due to a udev rule problem. Installing _CubeIDE_ installs these rules.
But creating them manually should work too. Place the appropriate file in the
`/etc/udev/rules.d` directory.

**49-stlinkv3.rules**
```
# ST_PKG_VERSION 1.0.2-3
# stlink-v3 boards (standalone and embedded) in usbloader mode and
# standard (debug) mode

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

## Troubleshooting
### Incorrect file length
If the `WAVE_STATIC_FILE_ALLOC` is enabled and the file was not correctly ended
it will result in an corrupted header. To repair the file one can simply run:
```shell
sox --ignore-length corrupted.wav fixed.wav
```
### Useful tools for WAV/Serialisation analysis
- `shntool info` : Useful for checking the PCM data
- `exiftool` : Useful for checking _chunk_/_subchunk_ info
- `mediainfo` : General purpose tool

### Illegal Instruction Float
[See forum post](https://community.arm.com/support-forums/f/compilers-and-libraries-forum/52623/gcc-11-2-arm-none-eabi-internal-compiler-error-illegal-instruction). Seems to be an problem with gcc >= 11.2
