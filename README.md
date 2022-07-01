# lib_espidf_canbus - a qCAN ESP-IDF Library

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT) [![Lib Version](https://img.shields.io/badge/Version-1.0.0-green.svg)](include/canbus_version.hpp) [![Main - Page](https://img.shields.io/badge/Project-Zakhar%20the%20Robot-yellow)](https://github.com/Zakhar-the-Robot "See project sources on Github")

This library implements a qCAN standard support for ESP-IDF devices. qCAN is a simple CAN extension for [Zakhar the Robot](https://zakhar-the-robot.github.io/doc/). See [the qCAN standard documentation](https://zakhar-the-robot.github.io/doc/docs/communication-protocols/canbus/) for details.

## Usage

The library is mostly callback-driven.

There are three types of Callbacks:

- Tx - the program can modify a message before sending
- Rx - the program can handle any incoming message
- Cmd - the program can handle only Command messages

There is also an option to configure a library for periodic data transmission providing a set of data pointers.

For the full public method description see the [canbus.hpp](include/canbus.hpp) file.

See [examples](#examples) for more guidance.
## Installation

**Option 1**

Add the folder of the repository to the `EXTRA_COMPONENT_DIRS` environment variable

**Option 2**

Clone the repository into your `components` directory of your ESP-IDF project. E.g.: `Idf_Blink_Project/components/lib_espidf_canbus`

## Examples

There are two projects in the [examples](examples/) directory. To use them you should have two ESP32 devices connected to the same CAN network. Modify `TX_GPIO_NUM` and `RX_GPIO_NUM` defines according to your setup.
