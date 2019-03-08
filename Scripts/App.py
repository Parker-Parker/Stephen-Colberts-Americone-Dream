from kivy.config import Config
Config.set('input', 'mouse', 'mouse,multitouch_on_demand')

###############################################################################
###############################################################################
###############################################################################

from random import random
from kivy.app import App
from kivy.uix.widget import Widget
from kivy.uix.button import Button
from kivy.graphics import Color, Ellipse, Line

from kivy.uix.slider import Slider
from kivy.uix.filechooser import FileChooserIconView
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.label import Label

# class MyPaintWidget(Widget):
#
#     def on_touch_down(self, touch):
#         color = (random(), 1, 1)
#         with self.canvas:
#             Color(*color, mode='hsv')
#             d = 30.
#             Ellipse(pos=(touch.x - d / 2, touch.y - d / 2), size=(d, d))
#             touch.ud['line'] = Line(points=(touch.x, touch.y))
#
#     def on_touch_move(self, touch):
#         touch.ud['line'].points += [touch.x, touch.y]


class PositionSlider(BoxLayout):
    def __init__(self):
        super().__init__(orientation='horizontal')
        self.add_widget(Slider(min=-100, max=100, value=25))
        self.add_widget(Label(text="000", size_hint_max_x = 50))
        self.add_widget(Label(text="000", size_hint_max_x = 50))


class SerialCommsPanel(BoxLayout):
    pass


class PositionalJointPanel(BoxLayout):
    def __init__(self):
        super().__init__(orientation='vertical')#, size_hint=(.5,.5))
        self.add_widget(PositionSlider())
        self.add_widget(PositionSlider())
        self.add_widget(PositionSlider())
        self.add_widget(PositionSlider())
        self.add_widget(PositionSlider())


class PositionalWorldPanel(Widget):
    pass


class NubInfoPanel(Widget):
    pass


class TrajectoryPanel(Widget):
    pass


class GUIApp(App):

    def build(self):
        parent = PositionalJointPanel()

        return parent



if __name__ == '__main__':
    GUIApp().run()
