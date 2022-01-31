class Vector2D():

    def __init__(self, x, y):
        self._x = x
        self._y = y

    @property
    def x(self):
        return self._x

    @property
    def y(self):
        return self._y


class Point():

    def __init__(self, x, y, color, label, display=True):
        self._x = x
        self._y = y
        self._color = color
        self._label = label
        self.display = display

    @property
    def position(self):
        return Vector2D(self._x, self._y)

    @property
    def x(self):
        return self._x

    @property
    def y(self):
        return self._y

    @property
    def color(self):
        return self._color
    
    @property
    def label(self):
        return self._label


class Label():

    def __init__(self, text, color, offset=Vector2D(0, 0)):
        self._text = text
        self._color = color
        self._offset = offset

    @property
    def text(self):
        return self._text

    @property
    def color(self):
        return self._color

    @property
    def offset(self):
        return self._offset


class Line():

    def __init__(self, start, end, color=(255,255,255), linestyle='solid', linewidth=1):
        self._start = start
        self._end = end
        self._color = color
        self._linestyle = linestyle
        self._linewidth = linewidth

    @property
    def start(self):
        return self._start

    @property
    def end(self):
        return self._end

    @property
    def color(self):
        return self._color

    @property
    def linestyle(self):
        return self._linestyle

    @property
    def linewidth(self):
        return self._linewidth