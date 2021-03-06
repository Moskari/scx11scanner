# scx11scanner

## Synopsis

This Python 3 package is for controlling a (custom) scanner, which uses two SCX11 universal controllers and stepping motors.

Only serial connection is supported.

The package enables easy connectivity and creation of concurrent control scripts for the motors.

![alt tag](https://www.dropbox.com/s/vf7n8kt68gjvl6x/skanneri.jpg?dl=1)

## Code Example:

Move scanner to two user defined positions.

```python
from scx11scanner.scanner import Scanner
# Specify serial ports
scan = Scanner(x_motor='COM5', y_motor='COM4', log_path='log.log')
with scan.open() as s:
    s.calibrate()  # Moves scanner to max position and sets HOME position
    s.move_pos(x=50, y=50)
    s.move_pos(x=25, y=25)
# Both motors start moving immediately and block execution until the movement is finished.
```

**Note**
`move_pos()` function requires calibration or at least `reset_home()` to set (x=0, y=0) position for the motors.
Otherwise, use `move_up()`, `move_down()`, `move_left()`, `move_right()` or `Motor()` class' methods directly.

**Warning!**
Use `calibrate()` to move scanner/motor to maximum position ONLY if there is an mechanism to stop motor movement at maximum position with outside alerting signal.

Individual motors can also be controlled separately from the `Scanner()`.

```python
from scx11scanner.scanner import Motor

motor = Motor()
with motor.open as m:
    s.move(dis=10)  # Specify moving distance if HOME position is not set.
    s.move(dis=-10)
```

## Installation

Tested only with Python 3.5 (Windows), but underlying PySerial module should support linux also.

`pip install <directory path to setup.py>`


## License

The MIT License (MIT)

**Disclaimer**
The author disclaims all responsibility for possible damage to machines and/or people. Use the software with your own risk.
