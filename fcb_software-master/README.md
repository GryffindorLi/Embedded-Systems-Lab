# Welcome to the Embedded Systems Laboratory software pack!

For starters cd to the in4073 folder, and read the in4073.c file. This is the quad side.
In the pc_terminal folder exists a sample pc terminal. Study this as well.

After that read the Makefile to find out what rules are available (modify them as you see fit!).
"make upload-run" is the most interesting one as it will compile both sides, upload the code to the quad and execute the pc side.

You are recommended to setup your own **private** gitlab or github repository for your team.

Have fun and remember: Google it before asking the TAs!

## Requirements (Ubuntu 20.04)
* build-essential (gcc and make)
* python2 (for the upload script)
* pip2 (to install pyserial)
* pyserial (`pip2 install pyserial`)
* git (optional)

## Installing pip2 on Ubuntu 20.04
Ubuntu wanted to make it more difficult to install pip2, so it's not downloadable using the package manager.
To install pip2, run:
```
$ wget https://bootstrap.pypa.io/pip/2.7/get-pip.py
$ sudo python2.7 get-pip.py 
```

Next install pyserial by running:
```
$ pip2 install pyserial
```

## Uploading without sudo
To access `/dev/tty` devices on ubuntu you need to be part of the `dialout` group,
you can add yourself to the group by running:
```
$ sudo adduser YOUR_USER_NAME dialout
```
You need to reboot for the change to take effect.

## Building the FCB code
Once you have installed the required packages, you can use the Makefile to
compile and upload the code to the FCB. This is done from within the `in4073`
directory.
```
$ cd in4073
$ make upload-run
```
