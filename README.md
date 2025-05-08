- [Library Information](#orgd444e10)
- [Host Computer Setup](#org4daa1b9)

    <!-- This file is generated automatically from metadata -->
    <!-- File edits may be overwritten! -->


<a id="orgd444e10"></a>

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

protocol-version = 0x02 prism-count = 7 command = protocol-version command-length command-number command-parameters response = protocol-version response-length command-number response-parameters

| command-name               | command-format | command-length | command-number | command-parameters         | response-format | response-length | response-parameters      |
|-------------------------- |-------------- |-------------- |-------------- |-------------------------- |--------------- |--------------- |------------------------ |
| invalid-command            |                |                |                |                            | '<BBB'          | 3               | 0xEE                     |
| read-cluster-address       | '<BBB'         | 3              | 0x01           |                            | '<BBBB'         | 4               | 0x00..0xFF               |
| check-communication        | '<BBB'         | 3              | 0x02           |                            | '<BBBL'         | 7               | 0x12345678               |
| reset                      | '<BBB'         | 3              | 0x03           |                            | '<BBB'          | 3               |                          |
| beep                       | '<BBBH'        | 5              | 0x04           | duration-ms                | '<BBB'          | 3               |                          |
| led-off                    | '<BBB'         | 3              | 0x05           |                            | '<BBB'          | 3               |                          |
| led-on                     | '<BBB'         | 3              | 0x06           |                            | '<BBB'          | 3               |                          |
| power-off-all              | '<BBB'         | 3              | 0x07           |                            | '<BBB'          | 3               |                          |
| power-on-all               | '<BBB'         | 3              | 0x08           |                            | '<BBB'          | 3               |                          |
| home                       | '<BBBB'        | 4              | 0x09           | prism-address              | '<BBBB'         | 4               | prism-address            |
| home-all                   | '<BBB'         | 3              | 0x0A           |                            | '<BBB'          | 3               |                          |
| write-target-position      | '<BBBBH'       | 6              | 0x0B           | prism-address, position-mm | '<BBBB'         | 4               | prism-address            |
| write-all-target-positions | '<BBBHHHHHHH'  | 17             | 0x0C           | position-mm[prism-count]   | '<BBB'          | 3               |                          |
| pause                      | '<BBBB'        | 4              | 0x0D           | prism-address              | '<BBBB'         | 4               | prism-address            |
| pause-all                  | '<BBB'         | 3              | 0x0E           |                            | '<BBB'          | 3               |                          |
| resume                     | '<BBBB'        | 4              | 0x0F           | prism-address              | '<BBBB'         | 4               | prism-address            |
| resume-all                 | '<BBB'         | 3              | 0x10           |                            | '<BBB'          | 3               |                          |
| read-actual-position       | '<BBBB'        | 4              | 0x11           | prism-address              | '<BBBhB'        | 6               | prism-address, -1..32767 |
| read-all-actual-positions  | '<BBB'         | 3              | 0x12           |                            | '<BBBhhhhhhh'   | 17              | -1..32767[prism-count]   |
|                            |                |                |                |                            |                 |                 |                          |


<a id="org4daa1b9"></a>

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
