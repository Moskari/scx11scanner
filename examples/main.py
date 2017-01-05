'''
Created on 21.10.2016

An example test script.

@author: Samuli Rahkonen
'''

from scx11scanner.scanner import Scanner


if __name__ == '__main__':
    # Specify serial ports
    scan = Scanner(x_motor='COM5', y_motor='COM4', log_path='log.log')
    with scan.open() as s:
        s.calibrate()  # Moves scanner to max position and sets HOME position
        s.move_pos(x=50, y=50)
        s.move_pos(x=25, y=25)
    # Both motors start moving immediately and block execution until the
    # movement is finished.
