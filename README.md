- [Library Information](#org2f7a740)
- [Host Computer Setup](#org2eccd74)

    <!-- This file is generated automatically from metadata -->
    <!-- File edits may be overwritten! -->


<a id="org2f7a740"></a>

# Library Information

-   **Name:** ClusterController
-   **Version:** 3.0.0
-   **License:** BSD
-   **URL:** <https://github.com/janelia-arduino/ClusterController>
-   **Author:** Peter Polidoro
-   **Email:** peter@polidoro.io
-   **PCB:** <https://github.com/janelia-kicad/cluster-pcb>


## Description

Firmware for each cluster of prisms in the Voigts Lab honeycomb maze.

-   protocol-version = 0x03
-   prism-count = 7
-   command = protocol-version command-length command-number command-parameters
-   response = protocol-version response-length command-number response-parameters
-   duration units = ms
-   position units = mm
-   speed units = mm/s
-   current units = percent
-   stall-threshold -> higher value = lower sensitivity, 0 indifferent value, 1..63 less sensitivity, -1..-64 higher sensitivity

| command-name           | command-format | command-length | command-number | command-parameters                                           | response-format | response-length | response-parameters    |
|---------------------- |-------------- |-------------- |-------------- |------------------------------------------------------------ |--------------- |--------------- |---------------------- |
| invalid-command        |                |                |                |                                                              | '<BBB'          | 3               | 0xEE                   |
| read-cluster-address   | '<BBB'         | 3              | 0x01           |                                                              | '<BBBB'         | 4               | 0x00..0xFF             |
| communicating-cluster  | '<BBB'         | 3              | 0x02           |                                                              | '<BBBL'         | 7               | 0x12345678             |
| reset-cluster          | '<BBB'         | 3              | 0x03           |                                                              | '<BBB'          | 3               |                        |
| beep-cluster           | '<BBBH'        | 5              | 0x04           | duration                                                     | '<BBB'          | 3               |                        |
| led-off-cluster        | '<BBB'         | 3              | 0x05           |                                                              | '<BBB'          | 3               |                        |
| led-on-cluster         | '<BBB'         | 3              | 0x06           |                                                              | '<BBB'          | 3               |                        |
| power-off-cluster      | '<BBB'         | 3              | 0x07           |                                                              | '<BBB'          | 3               |                        |
| power-on-cluster       | '<BBB'         | 3              | 0x08           |                                                              | '<BBB'          | 3               |                        |
| home-prism             | '<BBBBHBBb'    | 9              | 0x09           | prism-address, travel-limit, speed, current, stall-threshold | '<BBBB'         | 4               | prism-address          |
| home-cluster           | '<BBBHBBb'     | 8              | 0x0A           | travel-limit, speed, current, stall-threshold                | '<BBB'          | 3               |                        |
| homed-cluster          | '<BBB'         | 3              | 0x0B           |                                                              | '<BBBBBBBBBB'   | 10              | 0..1[prism-count]      |
| write-target-prism     | '<BBBBH'       | 6              | 0x0C           | prism-address, position                                      | '<BBBB'         | 4               | prism-address          |
| write-targets-cluster  | '<BBBHHHHHHH'  | 17             | 0x0D           | position[prism-count]                                        | '<BBB'          | 3               |                        |
| pause-prism            | '<BBBB'        | 4              | 0x0E           | prism-address                                                | '<BBBB'         | 4               | prism-address          |
| pause-cluster          | '<BBB'         | 3              | 0x0F           |                                                              | '<BBB'          | 3               |                        |
| resume-prism           | '<BBBB'        | 4              | 0x10           | prism-address                                                | '<BBBB'         | 4               | prism-address          |
| resume-cluster         | '<BBB'         | 3              | 0x11           |                                                              | '<BBB'          | 3               |                        |
| read-positions-cluster | '<BBB'         | 3              | 0x12           |                                                              | '<BBBhhhhhhh'   | 17              | -1..32767[prism-count] |
| write-speed-cluster    | '<BBBB'        | 4              | 0x13           | speed                                                        | '<BBB'          | 3               |                        |
| write-current-cluster  | '<BBBB'        | 4              | 0x14           | current                                                      | '<BBB'          | 3               |                        |


<a id="org2eccd74"></a>

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
