import sys

from pyqtgraph.Qt import QtGui, QtCore
import pyqtgraph as pg
import numpy as np

class ImageView(QtGui.QWidget):
    """ Image View without histograms
    
    This reimplements a bunch of the pyqtgraph ImageView class to exlude the
    histograms, ROI, and suff we don't need most of the time.
    """
    
    def __init__(self):
        """ Constructor for the ImageView
        
        Argument:
            no arguments
            
        Returns:
            the object
        """
        super(ImageView, self).__init__()
        self.ui = Ui_Form()
        self.ui.setupUi(self)
        self.scene = self.ui.graphicsView.scene()
        self.view = pg.ViewBox()
        self.ui.graphicsView.setCentralItem(self.view)
        self.view.setAspectLocked(True)
        self.img = None
        
    def setImage(self, img):
        """ Set the image to display
        
        Arguments:
            img: numpy array
        
        Returns:
            no returns
        """        
        if self.img is None:
            self.img = pg.ImageItem(img)
            self.view.addItem(self.img)
            return
        
        self.img.setImage(img)
        
class Ui_Form(object):
    """ A class to handle some under the hood stuff setting up for the ImageView
    """
    def setupUi(self, Form):
        """ Set up the ui
        
        Arguments:
            Form: the container for the ui
        
        Returns:
            no returns
        """
        Form.setObjectName(QtCore.QString.fromUtf8("Form"))
        self.gridLayout = QtGui.QGridLayout(Form)
        self.gridLayout.setSpacing(0)
        self.gridLayout.setMargin(0)
        self.graphicsView = pg.GraphicsView(Form)
        self.graphicsView.setObjectName(QtCore.QString.fromUtf8("graphicsView"))
        self.gridLayout.addWidget(self.graphicsView, 1, 1)

        QtCore.QMetaObject.connectSlotsByName(Form)
