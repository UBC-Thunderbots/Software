from bokeh.server.server import Server

from random import random

from bokeh.layouts import column
from bokeh.models import Button
from bokeh.palettes import RdYlBu3
from bokeh.plotting import figure, curdoc


def visualizer(doc):
    # the code below is just an example that's supposed to display a grid and buttons,
    # to be replaced by our actual code once the server actually works
    
    # create a plot and style its properties
    p = figure(x_range=(0, 100), y_range=(0, 100), toolbar_location=None)
    p.border_fill_color = 'black'
    p.background_fill_color = 'black'
    p.outline_line_color = None
    p.grid.grid_line_color = None

    # add a text renderer to the plot (no data yet)
    r = p.text(x=[], y=[], text=[], text_color=[], text_font_size="26px",
            text_baseline="middle", text_align="center")

    i = 0

    ds = r.data_source

    # create a callback that adds a number in a random location
    def callback():
        global i

        # BEST PRACTICE --- update .data in one step with a new dict
        new_data = dict()
        new_data['x'] = ds.data['x'] + [random()*70 + 15]
        new_data['y'] = ds.data['y'] + [random()*70 + 15]
        new_data['text_color'] = ds.data['text_color'] + [RdYlBu3[i%3]]
        new_data['text'] = ds.data['text'] + [str(i)]
        ds.data = new_data

        i = i + 1

    # add a button widget and configure with the call back
    button = Button(label="Press Me")
    button.on_click(callback)

    # put the button and plot in a layout and add to the document
    curdoc().add_root(column(button, p))


if __name__ == '__main__':
    # Setting num_procs here means we can't touch the IOLoop before now, we must
    # let Server handle that. If you need to explicitly handle IOLoops then you
    # will need to use the lower level BaseServer class.
    server = Server({'/': visualizer})
    server.start()

    print('Opening Bokeh application on http://localhost:5006/')
    server.io_loop.add_callback(server.show, "/")
    server.io_loop.start()