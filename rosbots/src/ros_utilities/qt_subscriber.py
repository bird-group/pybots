import rospy
from PyQt4.QtCore import QObject, pyqtSignal

class RosQtSubscriber(QObject, rospy.Subscriber):

    dataUpdated = pyqtSignal(object)
    
    def __init__(self, name, dataClass, parent=None):
        QObject.__init__(self, parent)
        rospy.Subscriber.__init__(self, name, dataClass,
            callback=self._callback)
            
    def _callback(self, data):
        self.dataUpdated.emit(data)
