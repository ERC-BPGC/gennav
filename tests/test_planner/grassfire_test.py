from __future__ import division
import math
'''
reference:referance: https://nrsyed.com/2017/12/30/animating-the-grassfire-path-planning-algorithm/
'''
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from grassfire import Grassfire

# Initialize grid rows, columns, and obstacle probability.
rows = 8
cols = 8
obstlist=[(5,5),(3,4),(4,4),(3,3),(2,2),(2,7),(6,6)]
start=(1,1)
end=(6,7)

# Instantiate Grassfire class. Initialize a grid and colorGrid.
Grassfire = Grassfire()
grid = Grassfire.select_grid(rows, cols, obstlist, start, end)

colorGrid = Grassfire.color_grid(grid)

# Initialize figure, imshow object, and axis.
fig = plt.figure()
gridPlot = plt.imshow(colorGrid, interpolation='nearest')
ax = gridPlot._axes
ax.grid(visible=True, ls='solid', color='k', lw=1.5)
ax.set_xticklabels([])
ax.set_yticklabels([])

# Initialize text annotations to display obstacle probability, rows, cols.
obstText = ax.annotate('', (0.15, 0.01), xycoords='figure fraction')
colText = ax.annotate('', (0.15, 0.04), xycoords='figure fraction')
rowText = ax.annotate('', (0.15, 0.07), xycoords='figure fraction')

def set_axis_properties(rows, cols):
    '''Set axis/imshow plot properties based on number of rows, cols.'''
    ax.set_xlim((0, cols))
    ax.set_ylim((rows, 0))
    ax.set_xticks(np.arange(0, cols+1, 1))
    ax.set_yticks(np.arange(0, rows+1, 1))
    gridPlot.set_extent([0, cols, 0, rows])

def update_annotations(rows, cols):
    '''Update annotations with obstacle probability, rows, cols.'''
    
    colText.set_text('Rows: {:d}'.format(rows))
    rowText.set_text('Columns: {:d}'.format(cols))

set_axis_properties(rows, cols)
update_annotations(rows, cols)

# Disable default figure key bindings.
fig.canvas.mpl_disconnect(fig.canvas.manager.key_press_handler_id)



# Functions init_anim() and update_anim() are for use with FuncAnimation.
def init_anim():
    '''Plot grid in its initial state by resetting "grid".'''
    Grassfire.reset_grid(grid)
    colorGrid = Grassfire.color_grid(grid)
    gridPlot.set_data(colorGrid)

def update_anim(dummyFrameArgument):
    '''Update plot based on values in "grid" ("grid" is updated
        by the generator--this function simply passes "grid" to
        the color_grid() function to get an image array).
    '''
    colorGrid = Grassfire.color_grid(grid)
    gridPlot.set_data(colorGrid)

# Create animation object. Supply generator function to frames.
ani = animation.FuncAnimation(fig, update_anim,
    init_func=init_anim, frames=Grassfire.find_path(grid),
    repeat=False, interval=150)

# Turn on interactive plotting and show figure.
plt.ion()
plt.show(block=True)