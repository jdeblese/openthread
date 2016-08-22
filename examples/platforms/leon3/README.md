# OpenThread on Leon3 Example

This directory contains example platform drivers for the [Gaisler Aeroflex
Leon3 Processor][leon3].

[leon3]: http://www.gaisler.com/index.php/products/processors/leon3

The example platform drivers are intended to present the minimal code
necessary to support OpenThread.  As a result, the example platform
drivers do not necessarily highlight the platform's full capabilities.
This example also does not implement a working radio, as the Leon3
processor does not contain one.

## Toolchain

Download and install the Gaisler BCC toolchain.

## Build Examples

```bash
$ cd <path-to-openthread>
$ ./bootstrap
$ make -f examples/Makefile-leon3
```

After a successful build, the `elf` files are found in
`<path-to-openthread>/output/bin`.  These can be uploaded to
a Leon3 SOC using grmon2.

## Interact

1. Open terminal to `/dev/ttyUSB0` (serial port settings: 115200 8-N-1).
2. Type `help` for list of commands.

```bash
> help
help
channel
childtimeout
contextreusedelay
extaddr
extpanid
ipaddr
keysequence
leaderweight
masterkey
mode
netdataregister
networkidtimeout
networkname
panid
ping
prefix
releaserouterid
rloc16
route
routerupgradethreshold
scan
start
state
stop
whitelist
```
