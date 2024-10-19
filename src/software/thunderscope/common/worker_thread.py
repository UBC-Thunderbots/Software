from pyqtgraph.Qt import QtCore
from typing import Callable


class WorkerThread(QtCore.QThread):
    """Runs a function in a thread and emits a signal to alert the parent when done.
    The return value of the function is emitted in the signal.

    Call start() on the WorkerThread to begin execution of the thread.

    IMPORTANT: You must save a reference to the WorkerThread instance or else it
    will be immediately garbage collected and destroyed once it goes out of scope,
    terminating the thread before it completes.
    """

    finished = QtCore.pyqtSignal(object)
    """Signal emitted once the thread has completed running the function.
    The signal is emitted with the return value of the function.
    """

    def __init__(self, func: Callable[[], object]):
        """Initialize the WorkerThread with a function to run

        :param func: the function to run on the thread
        """
        super().__init__()
        self.func = func

    def run(self):
        """Override QThread.run"""
        try:
            result = self.func()
        except Exception as e:
            result = e
        finally:
            self.finished.emit(result)
