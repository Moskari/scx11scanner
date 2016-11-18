'''
Created on 1.11.2016

@author: Samuli Rahkonen
'''


def kbinterrupt_decorate(func):
    '''
    Decorator.

    Adds KeyboardInterrupt handling to ControllerBase methods.
    '''
    def func_wrapper(*args, **kwargs):
        try:
            return func(*args, **kwargs)
        except KeyboardInterrupt:
            this = args[0]
            this._abort()
            raise
    return func_wrapper


def wait_decorate(func):
    '''
    Decorator.

    Adds waiting
    '''
    def func_wrapper(*args, **kwargs):
        this = args[0]
        wait = kwargs['wait']
        func(*args, **kwargs)
        if wait:
            this.x.wait_to_finish()
    return func_wrapper
