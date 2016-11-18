'''
Created on 1.11.2016

A script for measuring times the Scanner takes while moving.

@author: Samuli Rahkonen
'''

import scx11scanner.scanner as scanner
import time


if __name__ == '__main__':
    scan = scanner.Scanner(x_motor='COM5', y_motor='COM4')

    with scan.open() as s:
        times = []
        positions = []
        s.calibrate()

        i = 0
        while True:
            i += 1
            asd = input('Press enter')
            if asd != '':
                break
            start = time.time()
            s.move_pos(30*i, 0)
            end = time.time()
            times.append(end-start)
            positions.append((30*(i-1), 30*i))
        print('Times: ', times)
        print('Positions: ', positions)
