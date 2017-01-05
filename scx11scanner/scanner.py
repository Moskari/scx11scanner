'''
Created on 24.10.2016

@author: Samuli Rahkonen
'''
import serial
import re
import threading
import time
import logging
import os
import sys
from contextlib import contextmanager
from scx11scanner.utils import kbinterrupt_decorate
from logging.handlers import RotatingFileHandler


class ControllerBase(object):

    def __init__(self, logger=None, log_path=None):
        self.name = None
        self.id = None
        if logger is not None:
            self.logger = logger
        else:
            self.logger = logging.getLogger(__name__)
            # Using realpath() to resolve a possible symbolic link
            cur_dir = os.path.dirname(os.path.realpath(__file__))
            log_p = os.path.join(cur_dir, 'log.log')
            path = log_path or log_p
            # Handler for writing log messages to file.
            # All messages are written to log file.
            # file_handler = logging.FileHandler(log_path)
            file_handler = logging.handlers.RotatingFileHandler(
                path,
                mode='a',
                maxBytes=50*1024*1024,  # 50 MB
                backupCount=2,
                encoding=None,
                delay=0)
            file_formatter = logging.Formatter(
                '%(asctime)s [%(name)s] [%(levelname)-5s] %(message)s')
            file_handler.setFormatter(file_formatter)
            self.logger.addHandler(file_handler)

        # Handler for printing log messages to console.
        # Only level INFO or above messages are printed.
        console_handler = logging.StreamHandler(sys.stdout)
        console_handler.setLevel(logging.INFO)

        console_formatter = logging.Formatter(
                '%(asctime)s [%(name)s] [%(levelname)-8s] %(message)s')
        console_handler.setFormatter(console_formatter)
        self.logger.addHandler(console_handler)
        self.logger.setLevel(logging.DEBUG)

    def log(self, name, *args, level=logging.INFO):
        '''
        Prints given arguments and prints them to file.

        @param name: Defines a prefix for where the message comes from.
        @param *args: Arguments to be logged
        @param level: Defines the message level. I.e. logging.INFO,
                      logging.WARNING, logging.ERROR, logging.DEBUG
        '''
        n = str(name)
        text = ' '.join(map(lambda x: str(x), args))
        prefix_len = 25
        space = prefix_len - len(n)
        msg = ''.join([type(self).__name__,
                       '::',
                       str(self.name),
                       '::',
                       str(self.id),
                       '::',
                       n[:prefix_len],
                       ': ',
                       ' ' * (max(space, 0)),
                       str(text)])
        self.logger.log(level, msg)

    def _abort(self):
        raise NotImplementedError()


class Motor(ControllerBase):
    '''
    Class for giving motor instructions and executing them in sequence.
    '''

    def __init__(self, port, name=None, baudrate=9600, write_timeout=5,
                 logger=None, log_path=None):
        '''
        Constructor.

        @param port: string describing the serial port. I.e. for Windows 'COM4'
        @param name: string giving the motor a name
        @param baudrate: integer defining serial connection's baud rate
        @param write_timeout: Timeout period for connecting
        @param logger: Python Logging object
        '''
        super().__init__(logger=logger)
        self.defaults = [
            'UU=mm',
            'CURRENT=1']

        self.settings = {'VS': '2',  # Starting velocity
                         'VR': '2'}  # Rotation velocity

        self._queue = []  # Job queue for programs to be executed on SCX11
        self._running = False  # Is the motor moving (don't assign by hand)
        self._enabled = False  # Used to stop threads

        self.port = port
        self.id = port
        self.name = name or 'port-' + port

        # PySerial connection to motor
        self.ser = serial.Serial(
            baudrate=baudrate,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            write_timeout=write_timeout
        )

        self.connected = False

        # Lock to avoid using serial port at the same time
        self.thread_lock = threading.Lock()
        # Lock to avoid pushing and popping to program queue at the same time
        self.queue_lock = threading.Lock()
        # Lock to enable closing the threads from outside
        self.enabled_lock = threading.Lock()
        # Lock to check is program running in motor
        self.running_lock = threading.Lock()

        # Events for interthread communication
        event_motor_stopped = threading.Event()
        event_program_running = threading.Event()
        # Threads for polling the motor state and inputing command sequences
        self.poller_thread = threading.Thread(name='poll motor',
                                              target=self.motor_poller,
                                              args=(event_motor_stopped,
                                                    event_program_running))
        self.runner_thread = threading.Thread(name='run sequence',
                                              target=self.sequence_runner,
                                              args=(event_motor_stopped,
                                                    event_program_running))

    @property
    def enabled(self):
        '''
        Are threads enabled.
        '''
        self.enabled_lock.acquire()
        val = self._enabled
        self.enabled_lock.release()
        return val

    @enabled.setter
    def enabled(self, value):
        self.enabled_lock.acquire()
        self._enabled = value
        self.enabled_lock.release()

    @property
    def running(self):
        '''
        Is program running.
        '''
        self.running_lock.acquire()
        val = self._running
        self.running_lock.release()
        return val

    @running.setter
    def running(self, value):
        self.running_lock.acquire()
        self._running = value
        self.running_lock.release()

    def open(self, alarm_clear=False):
        '''
        Open serial connection to the motor.

        Finds a single device id and connects to it.
        '''
        # Open serial connection
        self.ser.port = self.port
        self.ser.open()
        self.connected = self.ser.isOpen()
        self.log('open', 'Serial connection to %s is %s' %
                 (self.id, 'open' if self.connected else 'closed'))

        if self.connected:
            self.dev_id = self.find_device_id()
            if not self.connect_dev_id(self.dev_id):
                raise Exception(
                    'Couldn\'t connect to device id=%s' % self.dev_id)

        # Enable threads
        self.enabled = self.connected

        # Clear motor alarm during startup
        if alarm_clear:
            self.clear_alarm()
            if not self.check_alarm():
                raise Exception('Alarm was not cleared during startup')

        # Start threads
        self.poller_thread.start()
        self.runner_thread.start()
        return self

    def close(self):
        self.log('close', 'Closing...')
        self._clear_queue()
        self.running = False
        self.enabled = False  # Shutdown threads
        # Wait threads to shut down
        self.runner_thread.join()
        self.poller_thread.join()
        # Close serial connection
        self.ser.close()
        self.connected = self.ser.isOpen()
        self.log('close', 'Serial connection to %s is %s' %
                 (self.port, 'open' if self.connected else 'closed'))
        if self.connected:
            raise Exception(
                'Serial connection to %s didn\'t close' % self.port)
        self.log('close', '...closed.')

    def _abort(self):
        self.log('abort', 'ABORT!', level=logging.WARNING)
        self.running = False
        self.enabled = False
        program = ['ABORT']
        self.thread_lock.acquire()
        self._run_program(program, name='abort')
        self.thread_lock.release()
        self.close()
        self.log('abort', 'Finished.', level=logging.WARNING)

    def find_device_id(self):
        self.thread_lock.acquire()
        self.ser.write(b'\\ID\r\n')
        time.sleep(0.1)
        output = self.ser.read_all().decode('ascii')
        self.thread_lock.release()
        m1 = re.search(r'^[0-9]+$', output, re.MULTILINE)
        m2 = re.search(r'^ *ID=[0-9]+.*$', output, re.MULTILINE)
        if m1 is not None:
            return m1.group(0)
        elif m2 is not None:
            return m2.group(0).split('=')[1].strip()
        else:
            raise Exception(
                'Read output wasn\'t something expected: %s' % output)

    def get_position(self):
        '''
        Returns motor position in relation to the current HOME.

        Not tested!
        '''
        self.thread_lock.acquire()
        self.ser.write(b'\\PC\r\n')
        time.sleep(0.1)
        output = self.ser.read_all().decode('ascii')
        m = re.search(r'^ *PC=[0-9]+.*$', output, re.MULTILINE)
        if m is not None:
            position = m.group(0).split('=')[1].strip()
        else:
            raise Exception(
                'Read output wasn\'t something expected: %s' % output)
        self.thread_lock.release()
        return position

    def connect_dev_id(self, dev_id):
        self.thread_lock.acquire()
        self.ser.write(('@%s\r\n' % dev_id).encode('ascii'))
        time.sleep(0.1)
        output = self.ser.read_all().decode('ascii')
        self.thread_lock.release()
        m = re.search(r'^%s>$' % dev_id, output, re.MULTILINE)
        self.log('connect_dev_id', m, level=logging.DEBUG)
        # self.log('connect_dev_id', output)
        return m is not None

    def motor_poller(self,
                     out_event_motor_stopped,
                     in_event_program_running):
        '''
        Thread for polling is the motor running.
        Sends out event telling that motor has stopped.

        @param out_event_motor_stopped: threading.Event which is set/cleared
                                        when motor is noticed to be
                                        stopped/moving.
        @param in_event_program_running: threading.Event which is
                                         waited before polling can start.
        '''
        name = 'motor_poller'
        self.log(name, 'Thread started.')
        out_event_motor_stopped.set()  # Motor is not moving
        while self.enabled:
            # Wait for event that tells that now motor is moving
            self.log(name, 'Waiting for program to run',
                     level=logging.DEBUG)
            program_running = in_event_program_running.wait(1)
            self.log(name, 'program_running=%s' % str(program_running),
                     level=logging.DEBUG)
            while program_running:
                self.thread_lock.acquire()
                if not self.enabled:
                    self.thread_lock.release()
                    break
                time.sleep(0.5)
                self.log(name, 'Polling motor.', level=logging.DEBUG)
                self.ser.write(b'SIGEND\r\n')
                time.sleep(0.5)
                output = self.ser.read_all().decode('ascii')
                self.log(name, '    ' + output, level=logging.DEBUG)
                m = re.search('.*SIGEND=[01].*', output)
                if m is None:
                    continue
                splits = m.group(0).split('=')
                value = splits[1].strip()
                if value == '0':
                    self.log(name, 'Motor is moving.', level=logging.DEBUG)
                    out_event_motor_stopped.clear()
                    # self.running = True
                elif value == '1':
                    self.log(name, 'Motor stopped.', level=logging.DEBUG)
                    self.running = False
                    out_event_motor_stopped.set()
                    self.thread_lock.release()
                    break
                self.thread_lock.release()
        self.log(name, 'Thread stopped.')

    def sequence_runner(self,
                        in_event_motor_stopped,
                        out_event_program_running):
        '''
        Thread for taking jobs (programs) from the job queue and running them.
        Sends out event telling that program is running

        @param in_event_motor_stopped: threading.Event which is waited before
                                       running a program can start
        @param out_event_program_running: threading.Event which is set/cleared
                                          when program is sent to SCX11
        '''
        name = 'sequence_runner'
        self.log(name, 'Thread started.')
        out_event_program_running.clear()
        while self.enabled:
            self.log(name, 'Waiting for motor to stop.', level=logging.DEBUG)
            motor_stopped = in_event_motor_stopped.wait(1)
            if not motor_stopped:
                continue
            # Wait until we have something to execute
            while self.enabled:
                time.sleep(0.1)
                sequence_list = self._pop_next_program()
                if sequence_list is not None:
                    break
            # Break out the loop if we are closing the thread
            if not self.enabled:
                break

            # Acquire lock because we will access serial port
            self.thread_lock.acquire()
            self.log(name, 'Running sequence:', level=logging.DEBUG)
            for s in sequence_list:
                self.log(name, '    ' + s, level=logging.DEBUG)

            self.running = True
            self._run_program(sequence_list, name=name)

            time.sleep(0.5)
            out_event_program_running.set()
            self.thread_lock.release()
        out_event_program_running.clear()
        self.log(name, 'Thread stopped.')

    def _sequence_list_to_bytes(self, seq_list):
        '''
        Returns new list of strings converted to ASCII.

        @param seq_list: A list of Python 3 (unicode) strings.
        @return: A list of strings converted to ASCII (bytes)
        '''
        return list(map(lambda x: (x + '\r\n').encode('ascii'),
                        seq_list))

    def _run_program(self, sequence_list, name='_run_program'):
        '''
        Runs sequence list (list of strings defining the program)

        @param sequence_list: A list of byte strings.
        @param name: A title for logging (optional).
        '''
        program = self._sequence_list_to_bytes(sequence_list)
        # Run each program code line
        for line in program:
            l = line
            # Try to run it
            for retry in range(10):
                self.ser.write(line)
                time.sleep(0.05)
                output = self.ser.read_all().decode('ascii')
                # self.log(name, '    ' + output)
                m = re.match('.*Error.*', output)
                if m is None:
                    break
                else:
                    self.log(name,
                             'Error with command', l,
                             'Retry', retry+1, '...')
                    if retry == 9:
                        self.running = False
                        raise Exception(
                            'Error: Couldn\'t run command %s' % l)

    def _push_program(self, program):
        '''
        Add new program to the job queue.

        @param program: A list of unicode strings (commands).
        '''
        self.queue_lock.acquire()
        self._queue.append(program)
        self.queue_lock.release()

    def _pop_next_program(self):
        '''
        Return the oldest program from the job queue.
        If  there is none, return None.

        @return: A list containing the oldest program in the job queue.
                 Otherwise None.
        '''
        self.queue_lock.acquire()
        if len(self._queue) > 0:
            program = self._queue.pop(0)
        else:
            program = None
        self.queue_lock.release()
        return program

    def _is_queue_empty(self):
        '''
        Checks is the job queue empty. Returns True/False.

        @return: True/False
        '''
        self.queue_lock.acquire()
        size = len(self._queue)
        self.queue_lock.release()
        return size == 0

    def _clear_queue(self):
        '''
        Clears the job queue.
        '''
        self.queue_lock.acquire()
        self._queue = []
        self.queue_lock.release()

    @kbinterrupt_decorate
    def wait_to_finish(self):
        '''
        Blocks execution until job queue is empty.
        '''
        name = 'wait_to_finish'
        self.log(name,
                 'Wait until job queue is empty and motor has stopped.')
        is_empty = self._is_queue_empty()
        while not is_empty or self.running:
            self.log(name,
                     'Queue is%s empty.' % ('' if is_empty else ' NOT'),
                     'running =', self.running, level=logging.DEBUG)
            time.sleep(0.1)
            is_empty = self._is_queue_empty()
        self.log(name, 'Finished waiting. Running =', self.running)

    def move(self, dis=None, pos=None, settings=None, wait=True):
        '''
        Pushes a move sequence to the job queue where it will be executed
        in FIFO (first in first out) order.

        Use only one of the 'distance' or 'position' arguments at once.

        @param dis: Distance to be moved.
        @param pos: Position to which motor should move.
        @param settings: Additional settings dict for the controller.
                         See SCX11 manual for full list of option.
        @param wait: True/False, is execution blocked until motor stops.
        '''
        if dis is not None and pos is None:
            command = ['DIS=%d' % dis, 'MI']
        elif pos is not None and dis is None:
            command = ['MA %d' % pos]
        else:
            raise Exception(
                'Use only one of arguments \'distance\' or \'position\'')

        if settings is None:
            sets = self.settings
        else:
            sets = settings

        if isinstance(sets, dict):
            settings_list = \
                list(map(lambda x: ''.join([x[0], '=', x[1]]),
                         sets.items()))
        else:
            raise Exception('Illegal settings: %s' % str(settings))

        program = self.defaults + settings_list + command
        self._push_program(program)
        self.log('move', 'Pushed job to queue.')
        if wait:
            self.wait_to_finish()

    def _calibrate(self, direction=-1, offset=10):
        '''
        TODO: Untested. Work in progress
        @param direction: 1 or -1, otherwise raises Exception
        @param offset: Distance which motor will move after hitting
                       max position.
        '''
        if direction not in [1, -1]:
            raise Exception()

        self.move(direction*9999, settings={'VS': '3', 'VR': '3'})
        self.wait_to_finish()
        alarm = self.check_alarm()
        self.log('calibrate', 'Expected alarm=True, was %s' % str(alarm),
                 level=logging.DEBUG)
        if alarm:
            self.clear_alarm()
            time.sleep(0.5)
            alarm = self.check_alarm()
            self.log('calibrate', 'Expected alarm=False, was %s' % str(alarm),
                     level=logging.DEBUG)
            if not alarm:
                self.move(-direction*offset)
                self.wait_to_finish()
                self.reset_home()

    def reset_home(self):
        '''
        Sets the SCX11 controller's home position to be the current position.
        '''
        self.thread_lock.acquire()
        self.ser.write(b'PRESET\r\n')
        self.thread_lock.release()

    def clear_alarm(self):
        '''
        Clear alarm if any.
        '''
        self.thread_lock.acquire()
        self.ser.write(b'ALMCLR\r\n')
        self.thread_lock.release()

    def check_alarm(self):
        '''
        Checks is alarm raised in the controller.
        '''
        self.thread_lock.acquire()
        self.ser.write(b'ALM\r\n')
        # self.log(name, 'Still alive!')
        time.sleep(0.5)
        output = self.ser.read_all().decode('ascii')
        self.thread_lock.release()
        self.log('check_alarm', output, level=logging.DEBUG)
        m = re.search('.*ALARM =[0-9]+ .*', output)
        if m is None:
            raise Exception(
                'Read output wasn\'t something expected: %s' % output)
        splits = m.group(0).split('=')
        value = splits[1].split(' ')[0].strip()
        self.log('check_alarm', 'Alarm value:', value, level=logging.DEBUG)
        if value == '00':
            return False
        else:
            return True


class Scanner(ControllerBase):
    '''
    Scanner object controls two SCX11 controllers and two step motors.

    Usage example:

    scan = Scanner('COM5', 'COM4')
    with scan.open() as s:
        s.calibrate() # Moves scanner to max position and sets HOME
        s.move_pos(x=50, y=50)
        s.move_pos(x=25, y=25)
    # Moving starts immediately
    '''

    def __init__(self, x_motor, y_motor, name=None,
                 logger=None, log_path=None):
        super().__init__(logger=logger, log_path=log_path)
        self.name = 'S' if name is None else name
        self.x = Motor(x_motor, name='X')
        self.y = Motor(y_motor, name='Y')
        self.id = '/'.join([str(self.x.id), str(self.y.id)])

    @contextmanager
    def open(self):
        try:
            self.x.open()
            self.y.open()
            yield self
        except KeyboardInterrupt:
            self._abort()
        finally:
            self.close()

    def close(self):
        self.log('close', 'Closing...')
        self.x.close()
        self.y.close()
        self.log('close', '...closed.')

    def _abort(self):
        self.log('abort', 'ABORT!')
        self.x.enabled = False
        self.y.enabled = False
        if self.x.connected:
            self.x._abort()
        if self.y.connected:
            self.y._abort()
        self.log('abort', 'Finished.')

    def move_up(self, distance, settings=None, wait=True):
        self.y.move(dis=distance, settings=settings, wait=wait)

    def move_down(self, distance, settings=None, wait=True):
        self.y.move(dis=-distance, settings=settings, wait=wait)

    def move_right(self, distance, settings=None, wait=True):
        self.x.move(dis=distance, settings=settings, wait=wait)

    def move_left(self, distance, settings=None, wait=True):
        self.x.move(dis=-distance, settings=settings, wait=wait)

    def move_pos(self, x, y, settings=None, wait=True):
        '''
        Moves to the given coordinate x, y. Moves both motors
        simultaneously.
        '''
        self.x.move(pos=x, settings=settings, wait=False)
        self.y.move(pos=y, settings=settings, wait=False)
        if wait:
            self.wait_to_finish()

    def reset_home(self):
        self.x.reset_home()
        self.y.reset_home()

    def clear_alarms(self):
        self.x.clear_alarm()
        self.y.clear_alarm()

    @kbinterrupt_decorate
    def calibrate(self,
                  x_direction=-1,
                  y_direction=-1,
                  offset=10,
                  retries=5,
                  settings={'UU': 'mm', 'VS': '3', 'VR': '3'}):
        '''
        Moves the scanner to maximum position until alert is raised
        and then changes direction to move 'offset' distance. Then the home
        position is set.
        '''
        name = 'calibrate'
        alarm_x = self.x.check_alarm()
        alarm_y = self.y.check_alarm()
        if alarm_x:
            self.x.clear_alarm()
        if alarm_y:
            self.y.clear_alarm()
        # Move to maximum position until alarm raises and motion stops.
        self.x.move(dis=x_direction*1000,
                    settings=settings, wait=False)
        self.y.move(dis=y_direction*1000,
                    settings=settings, wait=False)
        self.wait_to_finish()
        alarm_x = self.x.check_alarm()
        alarm_y = self.y.check_alarm()
        self.log(name,
                 'Expected X motor alarm=True, was %s' % str(alarm_x))
        self.log(name,
                 'Expected Y motor alarm=True, was %s' % str(alarm_y))
        if alarm_x and alarm_y:
            self.x.clear_alarm()
            self.y.clear_alarm()
            time.sleep(0.1)
            alarm_x = self.x.check_alarm()
            alarm_y = self.y.check_alarm()
            self.log(name,
                     'Expected X motor alarm=False, was %s' % str(alarm_x))
            self.log(name,
                     'Expected Y motor alarm=False, was %s' % str(alarm_y))
            if alarm_x or alarm_y:
                raise Exception(
                    'Didn\'t receive alarm! X: %s, Y: %s' % (str(alarm_x),
                                                             str(alarm_y)))
            self.x.move(dis=-x_direction*offset,
                        settings=settings,
                        wait=False)
            self.y.move(dis=-y_direction*offset,
                        settings=settings,
                        wait=False)
            self.wait_to_finish()
        else:
            raise Exception(
                'Didn\'t receive alarm! X: %s, Y: %s' % (str(alarm_x),
                                                         str(alarm_y)))
        # Scanner could get stuck, retry 5 times
        for retry in range(retries):
            alarm_x = self.x.check_alarm()
            alarm_y = self.y.check_alarm()
            if not alarm_x and not alarm_y:
                break
            self.log(name,
                     'Alarm didn\'t clear. Retrying %d...' % retry+1)
            if alarm_x:
                self.x.clear_alarm()
            if alarm_y:
                self.y.clear_alarm()
            time.sleep(0.1)
            if alarm_x:
                self.x.move(dis=-x_direction*offset,
                            settings=settings,
                            wait=False)
            if alarm_y:
                self.y.move(dis=-x_direction*offset,
                            settings=settings,
                            wait=False)
            self.wait_to_finish()
        if retry+1 >= retries:
            raise Exception('Calibration failed!')
        # TODO: Check position is it correct
        self.reset_home()

    @kbinterrupt_decorate
    def wait_to_finish(self):
        self.x.wait_to_finish()
        self.y.wait_to_finish()


if __name__ == '__main__':
    scan = Scanner(x_motor='COM5', y_motor='COM4')
    with scan.open() as s:
        s.calibrate()
        s.move_pos(25, 25)

    # cd \MyTemp\envs\scanner35\Scripts
    # activate
    # cd \MyTemp\koodit\eclipse\workspace\Scanner\examples
    # python scanner.py
