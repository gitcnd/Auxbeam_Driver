# Auxbeam_Driver
This software lets you connect a 3.3v Arduino Pro-mini (8mhz atmega328p) in-line to an Auxbeam switch panel to control the 8-gang output and read the switch presses


# SYNOPSIS

```C

  #define pcklen 5
  #define datbyte 2
  unsigned char buf[pcklen] = { 0b00001101, 0b01010001, 0x55, 0b00111111, 0b11100000};

  ...

  buf[datbyte] = 0b01010101; // Select which relays to turn on and off
  ret=bitsend(buf, pcklen, baud115200 ); // Send the signals to the Auxbeam

```


# HARDWARE

This was built to control the 8-gang panel kit from here: https://auxbeam.com/collections/8-gang-switch-panel

# NOTICE

We are not affiliated in any way whatsoever with Auxbeam.  This is no commercial relationship or benefit involved at all.

