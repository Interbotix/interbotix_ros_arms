from python_qt_binding.QtGui import QColor
from python_qt_binding.QtGui import QBrush
from python_qt_binding.QtGui import QPainter
from python_qt_binding.QtCore import QLine
from python_qt_binding.QtCore import QRect
from python_qt_binding.QtCore import QPoint
from python_qt_binding.QtWidgets import QPushButton

class JoystickButton(QPushButton):

    ### @brief Initialization of the JoystickButton class; it builds up from a QPushButton
    ### @param gui - the parent widget
    ### @details - code inspired from the JoystickButton class created in 'pyqtgraph'
    def __init__(self, gui):
        super(JoystickButton, self).__init__()
        self.gui = gui                                              # parent widget
        self.size = 360                                             # size in pixels of the push button
        self.state = None                                           # a QPoint representing the position of the crosshair in pixels
        self.pan_limits = (0, 360)                                  # width of the push button in pixels
        self.tilt_limits = (0, 360)                                 # height of the push button in pixels
        self.setCheckable(True)                                     # allow the push button to be 'checked'
        self.setFixedWidth(self.size)
        self.setFixedHeight(self.size)
        self.center = QPoint(self.size/2.0, self.size/2.0)          # (0,0) in pixels start from the upper left corner of the push button, so set the center to the middle (QPoint type)
        self.current_pos = QPoint(self.size/2.0, self.size/2.0)     # a QPoint representing the current position of the turret in pixels
        self.pan_deg_limits = [self.gui.name_map["pan"]["min_lower_limit"], self.gui.name_map["pan"]["max_upper_limit"]]                # Pan joint limits in degrees
        self.tilt_deg_limits = [self.gui.name_map["tilt"]["min_lower_limit"], self.gui.name_map["tilt"]["max_upper_limit"]]             # Tilt joint limits in degrees
        self.setJointCommands([float(self.gui.name_map["pan"]["display"].text()), float(self.gui.name_map["tilt"]["display"].text())])  # set the default crosshair position to the current turret position

    ### @brief Returns whether the joystick button is currently pressed
    ### @param <boolean> [out] - current button state
    def isActive(self):
        if self.isChecked():
            return True
        else:
            return False

    ### @brief Redefines what happens when the Joystick button is pressed
    ### @param ev - QMouseEvent
    def mousePressEvent(self, ev):
        ev.accept()
        self.setChecked(True)
        self.setState(ev)

    ### @brief Redefines what happens when the mouse is moved
    ### @param ev - QMouseEvent
    def mouseMoveEvent(self, ev):
        ev.accept()
        self.setState(ev)

    ### @brief Redefines what happens when the mouse is released
    ### @param ev - QMouseEvent
    def mouseReleaseEvent(self, ev):
        ev.accept()
        self.setChecked(False)

    ### @brief Draws a circle in the QPushButton
    ### @param pnt - QPoint specifying where the circle should be drawn
    ### @param color - QColor specifying the circle color
    ### @param fill - whether the inside of the circle should be colored
    def drawCircle(self, pnt, color, fill=True):
        p = QPainter(self)
        p.setPen(color)
        if (fill):
            p.setBrush(QBrush(color))
        p.drawEllipse(pnt,3,3)

    ### @brief Draws a crosshair centered at the specified point
    ### @param pnt - QPoint specifying the point at which to draw the crosshair
    def drawCrosshair(self, pnt):
        p = QPainter(self)
        horz_line = QLine(0, pnt.y(), self.size, pnt.y())
        vert_line = QLine(pnt.x(), 0, pnt.x(), self.size)
        p.drawLines(horz_line, vert_line)

    ### @brief Draws a green rectangle showing the valid positions that the pan and tilt motors can be commanded
    def drawRect(self):
        p = QPainter(self)
        in_bounds = QRect(self.pan_limits[0], self.size - self.tilt_limits[1], self.pan_limits[1] - self.pan_limits[0], self.tilt_limits[1] - self.tilt_limits[0])
        p.fillRect(in_bounds, QColor(0, 200, 0, 128))
        p.drawRect(in_bounds)

    ### @brief Redefines a paint event (essentially what is drawn on the QPushButton)
    ### @param ev - QPaintEvent
    def paintEvent(self, ev):
        QPushButton.paintEvent(self, ev)
        self.drawRect()
        self.drawCircle(self.state, QColor(0,0,0), False)
        self.drawCircle(self.center, QColor(0,0,0))
        self.drawCircle(self.current_pos, QColor(255,0,0))
        self.drawCrosshair(self.state)

    ### @brief Converts from degrees to pixels
    ### @param value - values in degrees to convert to pixels
    ### @param min_lower_limit - absolute minimum angle that the motor can reach in degrees
    ### @param max_upper_limit - absolute maximum angle that the motor can reach in degrees
    ### @param new_val - value converted to pixels
    def deg2Pxl(self, value, min_lower_limit, max_upper_limit):
        degree_span = max_upper_limit - min_lower_limit
        value_scaled = float(value - min_lower_limit) / degree_span
        new_val = value_scaled * self.size
        return new_val

    ### @brief Converts from pixels to degrees
    ### @param value - values in pixels to convert to degrees
    ### @param min_lower_limit - absolute minimum angle that the motor can reach in degrees
    ### @param max_upper_limit - absolute maximum angle that the motor can reach in degrees
    ### @param new_val - value converted to degrees
    def pxl2Deg(self, value, min_lower_limit, max_upper_limit):
        value_scaled = float(value) / self.size
        new_val = min_lower_limit + value_scaled * (max_upper_limit - min_lower_limit)
        return new_val

    ### @brief Sets the state of the crosshair in pixels
    ### @param ev - QMouseEvent containing the position of the cursor in pixels
    ### @details - checks to make sure that the desired crosshair position is within the pan and tilt limits before setting it; it
    ###            also converts the cursor position from pixels to degrees and updates the parent widget's slider bars accordingly
    def setState(self, ev):
        pnt = QPoint(ev.x(), ev.y())
        # Since (0,0) is in the upper left corner, that means that increasingly negative Tilt values will be closer to the top of the push button; However,
        # However, it makes visual sense to flip this so that increasingly negative Tilt values are closer to the bottom of the push button; thus, the funky math... :)
        if (pnt.x() < self.pan_limits[0] or pnt.y() < (self.size - self.tilt_limits[1]) or pnt.x() > self.pan_limits[1] or pnt.y() > (self.size - self.tilt_limits[0])):
            return
        if self.state == pnt:
            return
        self.state = pnt
        self.update()
        if self.isChecked():
            cmd_x = self.pxl2Deg(self.state.x(), self.pan_deg_limits[0], self.pan_deg_limits[1])
            cmd_y = -self.pxl2Deg(self.state.y(), self.tilt_deg_limits[0], self.tilt_deg_limits[1])
            self.gui.name_map["pan"]["display"].setText("%.1f" % cmd_x)
            self.gui.name_map["tilt"]["display"].setText("%.1f" % cmd_y)
            self.gui.update_slider_bar("pan")
            self.gui.update_slider_bar("tilt")

    ### @brief If the user changes the default Pan min/max values, this function converts the values from degrees to pixels
    ### @param min_value - new lower limit in degrees
    ### @param max_value - new upper limit in degrees
    def setPanMinMax(self, min_value, max_value):
        new_min = round(self.deg2Pxl(min_value, self.pan_deg_limits[0], self.pan_deg_limits[1]))
        new_max = round(self.deg2Pxl(max_value, self.pan_deg_limits[0], self.pan_deg_limits[1]))
        self.pan_limits = (new_min, new_max)
        self.update()

    ### @brief If the user changes the default Tilt min/max values, this function converts the values from degrees to pixels
    ### @param min_value - new lower limit in degrees
    ### @param max_value - new upper limit in degrees
    def setTiltMinMax(self, min_value, max_value):
        new_min = round(self.deg2Pxl(min_value, self.tilt_deg_limits[0], self.tilt_deg_limits[1]))
        new_max = round(self.deg2Pxl(max_value, self.tilt_deg_limits[0], self.tilt_deg_limits[1]))
        self.tilt_limits = (new_min, new_max)
        self.update()

    ### @brief Converts the current joint positions from degrees to pixels
    ### @param positions - list containing the pan and tilt values in degrees
    def setJointStates(self, positions):
        new_pan = round(self.deg2Pxl(positions[0], self.pan_deg_limits[0], self.pan_deg_limits[1]))
        new_tilt = round(self.size - self.deg2Pxl(positions[1], self.tilt_deg_limits[0], self.tilt_deg_limits[1]))
        # Since this function is called at around 100 Hz, only update the QPushButton if the current position has significantly changed
        if (abs(new_pan - self.current_pos.x()) >= 1 or abs(new_tilt - self.current_pos.y()) >= 1):
            self.current_pos = QPoint(new_pan, new_tilt)
            self.update()

    ### @brief Converts the current joint commands from degrees to pixels
    ### @param commands - list containing the pan and tilt values in degrees
    def setJointCommands(self, commands):
        new_pan = round(self.deg2Pxl(commands[0], self.pan_deg_limits[0], self.pan_deg_limits[1]))
        new_tilt = round(self.size - self.deg2Pxl(commands[1], self.tilt_deg_limits[0], self.tilt_deg_limits[1]))
        self.setState(QPoint(new_pan, new_tilt))
        self.update()
