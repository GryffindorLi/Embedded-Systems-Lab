In case you want to fly wireless via a gnu/linux pc, you can use ble_pty.
It creates a pseudo serial port after connecting to the fcb/quadurpel, that you can 
connect to as you are connecting to the serial port in tethered mode.

ble_pty is built using gattlib, and a packaged version is provided in the .deb
(sudo dpkg -i "file.deb")
