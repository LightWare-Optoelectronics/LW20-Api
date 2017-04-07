# LW20 / SF20 API
Version 0.7.0

## About

Visit [http://www.lightware.co.za](http://www.lightware.co.za) for more information about the products we provide.

## Supported products

* Model: LW20 - Firmware: 2.0 - Software: 2.1
* Model: LW20 - Firmware: 2.0 - Software: 2.2

## Overview

The layered structure of the API opens a variety of options for integration within your own application framework. You can select a single layer to work with, or use components from each that accomplish your goals.

Most applications will already have some internal transport layer that they wish to use for communicating with external devices (Serial, TCP / UDP, etc). This API does not include a transport layer directly, however the Linux, Windows & Arduinio samples all include basic serial communication examples.

The primary API file is lw20api.h as located in the root directory of this repository. There are also Windows and Linux usage examples in their respective directories. The Arduino directory contains a wrapper that is ready to be loaded into the Arduino IDE as a library. See the README in each directory for more specific information.

## Features & characteristics

* Support for Arduino, RaspberryPI, Windows & Linux. (8bit, 32bit, 64bit)
* Single header file library
* Zero external dependencies
* Zero dynamic memory allocations
* Minimal C++ style

## How to use single file header only libraries
You can include this file as normal when you would include any other header file. However, you need to define the implementation in one C or C++ compilation unit file. You can do this with:
```c++
#define LW20_API_IMPLEMENTATION
#include "lw20api.h"
```

## Layers

Each layer provides increasingly lower level access to the API. 

You need to create an instance of lwLW20 before using most API functions. This object should live throughout the lifetime of your communication with the sensor.

```c++
lwLW20 lw20 = lw20CreateLW20();
```

### Simple command execution

You can execute commands directly and get a response with a single call. This is a blocking operation. By passing callbacks to your IO functions this layer will automatically manage the event loops.

A service context is required to specify callbacks for command execution.

```c++
lwServiceContext serviceContext = {};
serviceContext.sendPacketCallback = sendPacket;
serviceContext.getPacketCallback = getPacket;
serviceContext.sleepCallback = sleep;
serviceContext.streamCallback = streamResponse;
```

You can then execute commands as follows:

```c++
executeCommand_GetProduct(&lw20, &serviceContext);
```

### Event loop execution

By passing callbacks to your IO functions this layer will automatically manage the event loops. You have control over when and how the loops execute, allowing you to capture streaming data.

You need to create an lwLW20 and lwServiceContext as above. You can use the packet writer to write into the LW20 send buffer, and then use runEventLoop to carry out the execution.

```c++
packetWriteLaserMode(&lw20.command, LWMS_48);
runEventLoop(&lw20, &serviceContext);
```

### Custom event loop

The custom loop lets you build and manage responses from the event pump manually, giving exact control on where and how to implement blocking / non-blocking portions of code. The event system requires continuous pumping and easily integrates into various architectural designs. Every command executed requires an event pump cycle to be managed to completion. You should hit the pump until you get recevie a LWELR_COMPLETED Status.

You can look at the implementation of runEventLoop in lw20api.h for examples on how to build your own.

### Packet writer & resolver

The lowest form of interacting with the sensor is writing and reading raw packets. You can use the packet write functions to build up commands that can be directly sent to the sensor over your transport layer. By feeding received data into the packet resolver you can reconstruct received packets with type and data.

## Other notes

When using the packet writer / resolver directly: After issuing a request you must wait for a response before issuing the next request.

## License
**Unlicense (http://unlicense.org) **

This is free and unencumbered software released into the public domain.

Anyone is free to copy, modify, publish, use, compile, sell, or
distribute this software, either in source code form or as a compiled
binary, for any purpose, commercial or non-commercial, and by any
means.

In jurisdictions that recognize copyright laws, the author or authors
of this software dedicate any and all copyright interest in the
software to the public domain. We make this dedication for the benefit
of the public at large and to the detriment of our heirs and
successors. We intend this dedication to be an overt act of
relinquishment in perpetuity of all present and future rights to this
software under copyright law.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
IN NO EVENT SHALL THE AUTHORS BE LIABLE FOR ANY CLAIM, DAMAGES OR
OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
OTHER DEALINGS IN THE SOFTWARE.

For more information, please refer to <http://unlicense.org>