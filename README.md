# Concentric Tube Robot Controller

Closed-loop control system for a concentric push-pull tube robot with rotational, linear, and stepper twist axes.

## Structure

- **firmware/robot_controller.ino** - Arduino firmware (PID control, encoder filtering, slew-rate ramped motor drive)
- **gui/slider.py** - Python/Tkinter GUI for real-time control over serial

## Setup

### Firmware
Open firmware/robot_controller.ino in the Arduino IDE. Requires Encoder and PID_v1 libraries.

### GUI
``npip install pyserial
python gui/slider.py
``n
## Serial Protocol (9600 baud, newline-terminated)

- R:<deg> - Rotation absolute
- L:<mm> - Linear absolute
- LR:<mm> - Linear relative
- S:<deg> - Stepper twist absolute
- RKP/RKI/RKD/LKP/LKI/LKD:<val> - Live PID tuning
- HOME - Zero all axes
