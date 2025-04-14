- [Library Information](#org20b6c79)
- [Host Computer Setup](#orgfc98e19)

    <!-- This file is generated automatically from metadata -->
    <!-- File edits may be overwritten! -->


<a id="org20b6c79"></a>

# Library Information

-   **Name:** ClusterController
-   **Version:** 0.1.0
-   **License:** BSD
-   **URL:** <https://github.com/janelia-arduino/ClusterController>
-   **Author:** Peter Polidoro
-   **Email:** peter@polidoro.io
-   **PCB:** <https://github.com/janelia-kicad/cluster-pcb>


## Description

Firmware for each cluster of prisms in the Voigts Lab honeycomb maze.

Commands would be like a few bytes, sent to one IP: one byte for command, one byte for prism id, two for arguments. Commands would be e.g. Beep (no arg, no arg), Calibrate to bottom position (prism, no arg), set target (prism, position), start move (prism, no arg), stop move (prism, no arg), report position (prism, no arg), set parameters (param id, param value)

| Command Name         | Command Format | Protocol Version | Command Number | Arg 0       | Arg 1 | Arg 2 | Response   | Response Format |
|-------------------- |-------------- |---------------- |-------------- |----------- |----- |----- |---------- |--------------- |
| read-cluster-address | '<BB'          | 0x01             | 0x01           |             |       |       | 0x00-0xFF  | '<B'            |
| check-communication  | '<BB'          | 0x01             | 0x02           |             |       |       | 0x12345678 | '<L'            |
| reset                | '<BB'          | 0x01             | 0x03           |             |       |       | 0x03       | '<B'            |
| beep                 | '<BBH'         | 0x01             | 0x04           | duration-ms |       |       | 0x04       | '<B'            |
| led-off              | '<BB'          | 0x01             | 0x05           |             |       |       | 0x05       | '<B'            |
| led-on               | '<BB'          | 0x01             | 0x06           |             |       |       | 0x06       | '<B'            |
| power-off            | '<BB'          | 0x01             | 0x07           |             |       |       | 0x07       | '<B'            |
| power-on             | '<BB'          | 0x01             | 0x08           |             |       |       | 0x08       | '<B'            |
|                      |                |                  |                |             |       |       |            |                 |


<a id="orgfc98e19"></a>

# Host Computer Setup


## Download this repository

<https://github.com/janelia-arduino/ClusterController.git>

```sh
git clone https://github.com/janelia-arduino/ClusterController.git
```


## PlatformIO


### Install PlatformIO Core

<https://docs.platformio.org/en/latest/core/installation/index.html>

```sh
python3 -m venv .venv
source .venv/bin/activate
pip install platformio
pio --version
```


### 99-platformio-udev.rules

Linux users have to install udev rules for PlatformIO supported boards/devices.

1.  Download udev rules file to /etc/udev/rules.d

    ```sh
    curl -fsSL https://raw.githubusercontent.com/platformio/platformio-core/develop/platformio/assets/system/99-platformio-udev.rules | sudo tee /etc/udev/rules.d/99-platformio-udev.rules
    ```

2.  Restart udev management tool

    ```sh
    sudo service udev restart
    ```

3.  Add user to groups

    ```sh
    sudo usermod -a -G dialout $USER && sudo usermod -a -G plugdev $USER
    ```

4.  Remove modemmanager

    ```sh
    sudo apt-get purge --auto-remove modemmanager
    ```

5.  After setting up rules and groups

    You will need to log out and log back in again (or reboot) for the user group changes to take effect.
    
    After this file is installed, physically unplug and reconnect your board.


### Compile the firmware

1.  Gnu/Linux

    ```sh
    make firmware
    ```

2.  Other

    ```sh
    pio run -e pico
    ```


### Upload the firmware

1.  Gnu/Linux

    ```sh
    make upload
    ```

2.  Other

    ```sh
    pio run -e pico -t upload
    ```


### Serial Terminal Monitor

1.  Gnu/Linux

    ```sh
    make monitor
    ```

2.  Other

    ```sh
    pio device monitor --echo --eol=LF
    ```


## Arduino Ide


### Download

<https://www.arduino.cc/en/software>


### Additional Boards Manager URLs

File > Preferences

    https://github.com/earlephilhower/arduino-pico/releases/download/global/package_rp2040_index.json


### Add Board Support Packages

-   Raspberry Pi Pico/RP2040 by Earle F Philhower, III
