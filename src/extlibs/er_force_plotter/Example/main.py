from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
from PyAnalogClock import AnalogClock
        
if __name__=="__main__":
    import sys
    
    a=QApplication(sys.argv)
    w=AnalogClock()
    w.show()
    sys.exit(a.exec_())