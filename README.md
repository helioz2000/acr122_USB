# acr122_USB
A simple implementation of the ACR122 NFC reader with USB.

### Overview
This code is a standalone implementation which requires only libusb.
It does not require libnfc or other kernel based driver.

### Purpose
This code was written as a lightweight implementation designed to retrieve the UID of the target (card, fob, …) to be used for low security access applications.

### Target Device
Raspberry Pi B+ V2 running Raspbian Wheezy, a Debian flavour of Linux

- - -
NOTE:
Raspbian Wheezy comes with a NFC driver loadable kernel module (LKM). The LKM will claim the USB interface and will deny acr122 access to the USB port. You will recognise this problem as the LED on the reader will light up within a few seconds of the reader being connected to the USB port. 
You can confirm the KLM’s with `sudo lsmod` which will list show `pn533` and `nfc` modules. The modules can be temporarily unloaded with `sudo rmmod pn533` and `sudo rmmod nfc` but reconnecting the ACR122U will re-load the modules.

For a more permanent solution place a file named `blacklist-libnfc.conf` into `/etc/modprobe.d/` with the following contents:
    blacklist nfc
    blacklist pn533
- - -

### Acknowledgements
A lot of my code was developed by studying libnfc. 
My thanks go to Ludovic Rousseau <ludovic.rousseau@gmail.com>, most of my knowledge comes from studying his code. 
Some header files have been taken unmodified from libnfc and used by this application with  parts commented out.









