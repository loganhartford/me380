import tkinter as tk
from tkinter import messagebox
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import math

from ik import *

# Function to plot the arm
def plot_arm(angles):
    fig, ax = plt.subplots()
    theta1, theta2 =angles[0], angles[1]
    x0, y0 = 0, 0
    x1, y1 = LINK_1 * math.cos(theta1), LINK_1 * math.sin(theta1)
    x2, y2 = x1 + LINK_2 * math.cos(theta1 + theta2), y1 + LINK_2 * math.sin(theta1 + theta2)

    ax.plot([x0, x1, x2], [y0, y1, y2], marker='o')

    # Annotate end effector coordinates
    ax.annotate(f'({x2:.1f}, {y2:.1f})', (x2, y2), textcoords="offset points", xytext=(10,-10))

    ax.set_xlim(-300, 300)
    ax.set_ylim(-300, 300)
    ax.set_aspect('equal', adjustable='box')
    ax.grid(True)  # Enable grid
    return fig

# Function called when the button is pressed
def on_button_click():
    #try:
    abs_x = entry_abs_x.get()
    abs_y = entry_abs_y.get()
    rel_x = entry_rel_x.get()
    rel_y = entry_rel_y.get()

    if abs_x and abs_y:
        abs_x = float(abs_x)
        abs_y = float(abs_y)
        angles = abs_request(abs_x, abs_y)
        plot(angles)
    
    elif rel_x and rel_y:
        rel_x = float(rel_x)
        rel_y = float(rel_y)
        angles = rel_request(rel_x, rel_y)
        plot(angles)

    # except Exception as e:
    #     messagebox.showerror("Error", str(e))

# Repetative code
def plot(angles):
    fig = plot_arm(angles)
    canvas.figure = fig
    canvas.draw()

# Starts the visualizer in a default state
def start_vis():
    try:
        fig = plot_arm([joint_state["theta1"], joint_state["theta2"]])
        canvas.figure = fig
        canvas.draw()
    except Exception as e:
        messagebox.showerror("Error", str(e))

# Function to handle window close event
def on_closing():
    root.quit()
    root.destroy()

# Function to be called when Enter key is pressed
def on_enter_key(event):
    on_button_click()

# Enables arrow key controls   
class KeyTracker:
    def __init__(self):
        self.keys_pressed = set()
        self.DELTA = 5

    def key_pressed(self, event):
        self.keys_pressed.add(event.keysym)
        self.check_combination()

    def key_released(self, event):
        self.keys_pressed.discard(event.keysym)

    def check_combination(self):
        if {'Up', 'Left'} <= self.keys_pressed:
            angles = rel_request(-self.DELTA, self.DELTA)
            plot(angles)
        elif {'Up', 'Right'} <= self.keys_pressed:
            angles = rel_request(self.DELTA, self.DELTA)
            plot(angles)
        elif {'Down', 'Right'} <= self.keys_pressed:
            angles = rel_request(self.DELTA, -self.DELTA)
            plot(angles)
        elif {'Down', 'Left'} <= self.keys_pressed:
            angles = rel_request(-self.DELTA, -self.DELTA)
            plot(angles)
        elif {'Down'} <= self.keys_pressed:
            angles = rel_request(0, -self.DELTA)
            plot(angles)
        elif {'Up'} <= self.keys_pressed:
            angles = rel_request(0, self.DELTA)
            plot(angles)
        elif {'Left'} <= self.keys_pressed:
            angles = rel_request(-self.DELTA, 0)
            plot(angles)
        elif {'Right'} <= self.keys_pressed:
            angles = rel_request(self.DELTA, 0)
            plot(angles)

        

# Create the main window
root = tk.Tk()
key_tracker = KeyTracker()
root.title("Robot Arm Visualization")

## Layout
right_frame = tk.Frame(root)
ui_frame = tk.Frame(right_frame)


left_ui = tk.Frame(ui_frame, bg="red")

top_left_ui = tk.Frame(left_ui)
label_x = tk.Label(top_left_ui, text="Abs X: ")
label_x.pack(side="left")
entry_abs_x = tk.Entry(top_left_ui)
entry_abs_x.pack(side="right")
top_left_ui.pack()

bottom_left_ui = tk.Frame(left_ui)
label_x = tk.Label(bottom_left_ui, text="Abs Y: ")
label_x.pack(side="left")
entry_abs_y = tk.Entry(bottom_left_ui)
entry_abs_y.pack(side="right")
bottom_left_ui.pack()

left_ui.pack(side="left")

right_ui = tk.Frame(ui_frame, bg="green")

top_right_ui = tk.Frame(right_ui)
label_x = tk.Label(top_right_ui, text="Rel X: ")
label_x.pack(side="left")
entry_rel_x = tk.Entry(top_right_ui)
entry_rel_x.pack(side="right")
top_right_ui.pack()

bottom_right_ui = tk.Frame(right_ui)
label_x = tk.Label(bottom_right_ui, text="Rel Y: ")
label_x.pack(side="left")
entry_rel_y = tk.Entry(bottom_right_ui)
entry_rel_y.pack(side="right")
bottom_right_ui.pack()

right_ui.pack(side="right")

# Create and place widgets
button = tk.Button(right_frame, text="Update", command=on_button_click)
button.pack(side="bottom")

ui_frame.pack(side="top")
right_frame.pack(side="bottom", padx=20, pady=20)

# Canvas for matplotlib plot
fig, ax = plt.subplots()
canvas = FigureCanvasTkAgg(fig, master=root)
canvas_widget = canvas.get_tk_widget()
canvas_widget.pack(side="left", fill="y", expand=True)

# Bind the Enter key to the on_enter_key function
root.bind('<Return>', on_enter_key)

# Allows for arrow key controls
root.bind('<KeyPress>', key_tracker.key_pressed)
root.bind('<KeyRelease>', key_tracker.key_released)


# Bind window close event to handler
root.protocol("WM_DELETE_WINDOW", on_closing)

start_vis()

root.mainloop()
