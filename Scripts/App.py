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
from kivy.uix.dropdown import DropDown

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

########################################################################
#                Primitives
#########################################################################


class PositionSlider(BoxLayout):
    def __init__(self):
        super().__init__(orientation='horizontal')
        self.add_widget(Slider(min=-100, max=100, value=25))
        self.add_widget(Label(text="000", size_hint_max_x = 50))
        self.add_widget(Label(text="000", size_hint_max_x = 50))
# class ComDropDown(DropDown):
#     def __init__(self):
#         super().__init__()
#         self.refresh()
#         self.dismiss()
#     def refresh(self):
#         list = [DropButton(text="UWU", size_hint_y=None, height=44),DropButton(text=";D", size_hint_y=None, height=44),DropButton(text="yeet", size_hint_y=None, height=44),]
#         self.children = [i for i in list]
#
# class DropButton(Button):
#     def __init__(self):
#         super().__init__(orientation='horizontal')
#         self.add_widget(Slider(min=-100, max=100, value=25))
#         self.add_widget(Label(text="000", size_hint_max_x = 50))
#         self.add_widget(Label(text="000", size_hint_max_x = 50))
#     def refresh(self):


class ComDropDown(DropDown):
    def __init__(self):
        super().__init__()
        self.refresh()
        self.dismiss()
    def refresh(self):
        list = [DropButton("UWU"),DropButton(";D"),DropButton("yeet")]
        self.children = [i for i in list]


class DropButton(Button):
    def __init__(self, name):
        super().__init__(text = name, size_hint_y=None, height=44)
        self.name = name
    def on_press(self):
        self.parent.select(self)


class DropMainButton(Button):
    def __init__(self):
        super().__init__(text = "text", size_hint_y=None, height=44)
        self.dd = ComDropDown()
        self.add_widget(self.dd)
    def on_press(self):
        self.dd.open(self)
        print ("wheee")


########################################################################
#                Panels
#########################################################################

class SerialCommsPanel(BoxLayout):
    def __init__(self):
        super().__init__(orientation='horizontal')
        self.add_widget(DropMainButton())
        self.add_widget(DropMainButton())
        self.add_widget(DropMainButton())
        self.add_widget(DropMainButton())
        self.add_widget(DropMainButton())
        self.add_widget(DropMainButton())


class PositionalJointPanel(BoxLayout):
    def __init__(self):
        super().__init__(orientation='vertical')
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
        # parent = PositionalJointPanel()

        parent = SerialCommsPanel()

        return parent



if __name__ == '__main__':
    GUIApp().run()
