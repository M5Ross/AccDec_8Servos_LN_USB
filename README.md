# AccDec_8Servos_LN_USB

DCC accessory decoder for 8 servo with Loconet and USB interfaces.

![image1](https://github.com/M5Ross/AccDec_8Servos_LN_USB/blob/main/doc/IMG_1.jpg)

Derived from mixing some examples resurces of library NmraDcc and Loconet.

Library required:

- [NmraDcc] (public)
- [Loconet] (public)
- [ConfCVlib] (manual installation)
- [DccSerialCom] (manual installation)

Features:

- 8 Servo outputs.
- DCC optoisolated input, 2 Loconet sockets, USB connector.
- Servo customizable start/stop points, travelling speed, default position ad power on, save last position.
- Separate external power socket or autopower via DCC or Power from/to Loconet bus, Loconet pull-up integrated.
- Sigle address mode (continuos address for each output) or multiple address mode (each output has it own address).
- Direct USB connection, instead of custom SerialCom inteface.
- USB Loconet Gateway integrated (see [LocoLinx]).
- Most powerfull Arduino Leonardo processor instead of standard Arduono Uno core.
- Commands source from DCC and/or Loconet and/or USB.

Designed to be configured with the custom PC tool [DecoderConfigurator], or in standardize way via CV or LNCV (Loconet CV).

![image2](https://github.com/M5Ross/AccDec_8Servos_LN_USB/blob/main/doc/IMG_2.jpg)
![image3](https://github.com/M5Ross/AccDec_8Servos_LN_USB/blob/main/doc/IMG_3.jpg)

[NmraDcc]: https://github.com/mrrwa/NmraDcc
[Loconet]: https://github.com/mrrwa/LocoNet
[LocoLinx]: https://github.com/mrrwa/LocoNet/tree/master/examples/LocoLinx32U4
[ConfCVlib]: https://github.com/M5Ross/ConfCVlib
[DccSerialCom]: https://github.com/M5Ross/DccSerialCom
[DecoderConfigurator]: https://github.com/M5Ross/DecoderConfigurator
