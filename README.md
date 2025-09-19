# Mercury 236 RS-485 demo

This project demonstrates how to drive a *Mercury 236 ART / ART-02* energy
meter over the native "Mercury" single-packet protocol while sharing the same
RS-485 bus with the existing Modbus master component.

The demo application configures UART1 (RS-485 half-duplex) through
`modbus_handler_init()`, opens a level 1 session with the meter and periodically
prints the following data to the serial console:

* Device serial number and production date (read once on start-up).
* Suggested network address derived from the serial number.
* Phase voltages, currents, and power factors.
* Grid frequency along with active, reactive, and apparent power sums.

## Hardware requirements

* ESP32 series board with an RS-485 transceiver (tested with MAX485/MAX3485).
* Mercury 236 ART / ART-02 electricity meter connected to the same RS-485 line.
* Optional USB-to-RS-485 adapter for monitoring traffic.

UART1 pins and DE/~RE control lines are configured inside `modbus_handler.c`
(`TXD=GPIO17`, `RXD=GPIO18`, `RTS/DE=GPIO10` by default). Adjust these constants
if your hardware uses different wiring.

## Configuration

Use the project menuconfig to point the demo at your device:

```
idf.py menuconfig
```

Navigate to **Mercury 236 demo configuration** and set:

* **Mercury 236 network address** – RS-485 address of your meter (47 for serial
  `51664847` without the "D" suffix).
* **Meter has "D" index (ASCII passwords)** – enable for models that expect
  ASCII passwords (factory default `"111111"`/`"222222"`). Leave disabled for
  standard HEX-password devices (`0x111111`/`0x222222`).
* **Polling interval** – how often instantaneous values are read while the
  channel is open.
* **Retry delay** – wait time before re-opening the channel after a failure.
* **Demo task stack size / priority** – FreeRTOS parameters for the polling task
  in case you need to tune them for your firmware.

## Building and running

Build, flash and monitor as usual:

```
idf.py -p PORT flash monitor
```

Example console output after a successful poll:

```
I (3400) MERCURY_DEMO: Mercury 236 demo started (addr=47, variant=standard)
I (3410) MERCURY_DEMO: Testing link...
I (3430) MERCURY_DEMO: Channel L1 opened
I (3430) MERCURY_DEMO: Serial: 51664847 manufactured on 15-04-2023
I (3430) MERCURY_DEMO: Suggested address from serial: 47
I (3450) MERCURY_DEMO: U[V]: A=229.7  B=230.1  C=228.9
I (3450) MERCURY_DEMO: I[A]: A=3.214  B=3.105  C=3.287
I (3450) MERCURY_DEMO: PF  : A=0.996  B=0.995  C=0.997  Σ=0.996
I (3450) MERCURY_DEMO: Freq=49.98 Hz  P=5.40 kW  Q=0.12 kvar  S=5.41 kVA
```

If the link test or any subsequent request fails, the task logs the error and
retries after the configured delay. Use these logs to verify wiring, address,
password settings and general communication health.
