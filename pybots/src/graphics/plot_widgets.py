import sys
from PyQt4 import QtGui, QtCore

from pyqtgraph.Qt import QtGui, QtCore
import pyqtgraph as pg
import numpy

from math import sqrt, sin, cos, pi

from geometry.quaternion import Quaternion

class StripChart(QtGui.QWidget):
    """ a class to implement a stripchart using the pyqtgraph plotting
    utilities
    """
    def __init__(self, plt=None, dim=1, relative_time=False, max_pts=200):
        super(StripChart, self).__init__()
        self.plt = plt
        self.curve = []
        self.xdata = []
        self.ydata = []
        for i in range(0,dim):
            self.curve.append(plt.plot(pen='w'))
            self.ydata.append([])
            self.xdata.append([])
        self.dim = dim
        self.max_pts = max_pts
        self._npts = [0,] * dim
        self.pens = [None,] * dim
        self.brushes = [None,] * dim

        self._use_relative_time = relative_time

    def _update_plot(self):
        offset = 0.0
        if self._use_relative_time:
            for xd in self.xdata:
                if len(xd) == 0:
                    continue
                offset = max(numpy.amax(numpy.array(xd)), offset)

        for xd,yd,c,i  in zip(self.xdata, self.ydata, self.curve, range(self.dim)):
            if numpy.isscalar(xd):
                xd = [xd,]
                yd = [yd,]

            plot_xdata = numpy.array(xd) - offset
            plot_ydata = numpy.array(yd)

            c.setData(x=plot_xdata, y=plot_ydata)

            if self.brushes is not None:
                assert len(self.brushes) == self.dim, "Number of brush\
                    collections must match number of samples"
                nbrush = 0
                if self.brushes[i] is not None:
                    c.setBrush(self.brushes[i])

            if self.pens is not None:
                assert len(self.pens) == self.dim, "Number of pens\
                    collections must match number of samples"
                npen = 0
                if self.pens[i] is not None:
                    c.setPen(self.pens[i])

    def update_data(self, x_new, y_new, idx=None, brushes=None, pens=None):
        if idx is None:
            idx = range(0,self.dim)
        if self.dim == 1:
            if not isinstance(x_new, tuple):
                x_new = (x_new,)
            if not isinstance(y_new, tuple):
                y_new = (y_new,)

        for x,y,i in zip(x_new, y_new, idx):
            if self._npts[i] < self.max_pts:
                if numpy.isnan(x) or numpy.isnan(y):
                    continue
                self.xdata[i] = numpy.append(self.xdata[i], x)
                self.ydata[i] = numpy.append(self.ydata[i], y)
                self._npts[i] += 1
            else:
                if numpy.isnan(x) or numpy.isnan(y):
                    continue
                self.xdata[i] = numpy.append(self.xdata[i][1:], x)
                self.ydata[i] = numpy.append(self.ydata[i][1:], y)

        if brushes is None:
            brushes = [None,]*len(idx)
        if pens is None:
            pens = [None,]*len(idx)
        for b,p,i in zip(brushes, pens, idx):
            self.brushes[i] = b
            self.pens[i] = p

        self._update_plot()

class ImageDisplay(QtGui.QWidget):
    """ image view widget
    """
    def __init__(self, img=None, img_data=None, is_colorize=False):
        super(ImageDisplay, self).__init__()
        self._img_view = img
        if img_data is not None:
            self.img_data = img_data
        else:
            self.img_data = numpy.zeros((2,2))
        self.cmax = 1.0
        self.cmin = 0.0
        self.is_colorize = is_colorize

    def update_data(self, img_new=None):
        if img_new is None or self._img_view is None:
            return
        self.img_data = img_new
        if self.is_colorize:
            self._img_view.setImage(self.colorize())
        else:
            self._img_view.setImage(self.img_data)

    def colorize(self):
        len_x = self.img_data.shape[0]
        len_y = self.img_data.shape[1]
        c = numpy.zeros((len_x, len_y, 3))
        crange = self.cmax - self.cmin
        c[:,:,0] = (self.img_data - self.cmin)/crange
        c[:,:,1] = 1 - abs(self.img_data/crange)
        c[:,:,2] = -(self.img_data - self.cmax)/crange
        return c

class xyPlot(QtGui.QWidget):
    """ Plot Widget for x-y data
    """
    def __init__(self, plt=None, dim=1, xlim=None, ylim=None):
        super(xyPlot, self).__init__()
        self.plt = plt
        self.curve = []
        self.xdata = []
        self.ydata = []
        self.pens = [None,] * dim
        self.brushes = [None,] * dim

        self._xlim = xlim
        self._ylim = ylim

        self.dim = dim

    def _update_plot(self):
        for xd,yd,c,i  in zip(self.xdata, self.ydata, self.curve, range(self.dim)):
            if numpy.isscalar(xd):
                xd = [xd,]
                yd = [yd,]

            if self.size is not None:
                c.setData(x=xd, y=yd, size=self.size)
            else:
                c.setData(x=xd, y=yd)

            if self.brushes is not None:
                assert len(self.brushes) == self.dim, "Number of brush\
                    collections must match number of samples"
                nbrush = 0
                if self.brushes[i] is not None:
                    c.setBrush(self.brushes[i])

            if self.pens is not None:
                assert len(self.pens) == self.dim, "Number of pens\
                    collections must match number of samples"
                npen = 0
                if self.pens[i] is not None:
                    c.setPen(self.pens[i])

        if self._xlim:
            self.plt.setXRange(self._xlim[0], self._xlim[1])
        if self._ylim:
            self.plt.setYRange(self._ylim[0], self._ylim[1])

    def update_data(self, x_new, y_new, curve_index=None, auto_update=True,
        brushes=None, pens=None, size=None):
        """Update the xy plot

        Arguments:
            x_new: new x data to update. Must either be a numpy array or a tuple
                of numpy arrays
            y_new: new y data to update. Must either be a numpy array or a tuple
                of numpy arrays
            curve_index: tuple of indices which indicate the curves which the
                tuples in x_new and y_new should update
            auto_update: optional, boolean indicating if we should redraw the
                plot, defaults to True
            brushes: tuple of brushes corresponding to the data to update
            pens: tuple of pens corresponding to the data to update
            size: not really sure

        Returns:
            no returns
        """
        assert type(x_new) is type(y_new), "x and y data must either be\
            numpy arrays or tuples containing them"
        if type(x_new) is not tuple:
            assert self.dim == 1, "must specify tuple of data if there is\
                more htan one data series"
            x_new = (x_new,)
            y_new = (y_new,)
            curve_index = (0,)
        assert curve_index is not None, "must specify the data series that\
            correspond to data in x_new and y_new tuples"

        if brushes is None:
            brushes = [None,]*len(curve_index)
        if pens is None:
            pens = [None,]*len(curve_index)
        for xd,yd,i,b,p in zip(x_new, y_new, curve_index, brushes, pens):
            self.xdata[i] = xd
            self.ydata[i] = yd
            if b is not None:
                self.brushes[i] = b
            if p is not None:
                self.pens[i] = p

        self.size = size

        if auto_update:
            self._update_plot()

class ScatterPlot(xyPlot):
    """ Widget for scatterplots. Inherits from xyPlot
    """
    def __init__(self, plt=None, dim=1):
        super(ScatterPlot, self).__init__(plt, dim)

        for i in range(0, self.dim):
            self.curve.append(pg.ScatterPlotItem(pen='w'))
            plt.addItem(self.curve[-1])
            self.ydata.append(numpy.zeros((1,)))
            self.xdata.append(numpy.zeros((1,)))

class LinePlot(xyPlot):
    """ Widget for lineplots. Inherits from xyPlot
    """
    def __init__(self, plt=None, dim=1, xlim=None, ylim=None):
        super(LinePlot, self).__init__(plt, dim, xlim, ylim)

        for i in range(0, self.dim):
            self.curve.append(pg.PlotCurveItem(pen='w'))
            plt.addItem(self.curve[-1])
            self.ydata.append(numpy.zeros((1,)))
            self.xdata.append(numpy.zeros((1,)))
