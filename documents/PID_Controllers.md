# PID Controllers

## Dynamic PID Controllers

In common with Betaflight, Protoflight uses dynamic PID controllers: that is the PID constants are changed during the course of flight.
These changes are detailed below.

## ITerm Relax

ITerm Relax is used to reduce the amount of overshoot and bounce-back at the end of a roll or flip manuever.
It works by reducing the ITerm on an axis when the setpoint on that axis exceeds a certain value.
