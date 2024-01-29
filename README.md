# Keyboard (software part - key scanner)

hardware part

- https://github.com/sb-child/keyboard-hardware

## build

- install vscode, then install platformio plugin

- clone this repo, open in vscode, then click `PlatformIO: Build` button at the bottom

## upload

this firmware need to upload to MCU on the **2nd** and the **6rd** board.

you need to prepare a `WCH-LinkE-R0-1v3` debugger, and solder three wires `VCC, GND, SWDIO` from the board which need to upload, and connect the wires to the debugger

click `PlatformIO: Upload` button at the bottom

disconnect these wires
