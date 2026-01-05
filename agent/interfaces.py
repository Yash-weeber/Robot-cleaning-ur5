import tkinter as tk
from tkinter import messagebox
import numpy as np

class DrawingInterface:
    def __init__(self, width=400, height=300, title="Draw Trajectory"):
        self.width = width
        self.height = height
        self.title = title
        self.trajectory = []
        self.drawing = False
        self.completed = False

        # Create window
        self.root = tk.Toplevel()
        self.root.title(title)
        self.root.geometry(f"{width}x{height + 100}")
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)

        # Canvas for drawing
        self.canvas = tk.Canvas(self.root, width=width, height=height, bg='white')
        self.canvas.pack(pady=10)

        # Bind mouse events
        self.canvas.bind("<Button-1>", self.start_draw)
        self.canvas.bind("<B1-Motion>", self.draw)
        self.canvas.bind("<ButtonRelease-1>", self.stop_draw)

        # Buttons
        button_frame = tk.Frame(self.root)
        button_frame.pack(pady=10)

        tk.Button(button_frame, text="Clear", command=self.clear).pack(side=tk.LEFT, padx=5)
        tk.Button(button_frame, text="Done", command=self.done).pack(side=tk.LEFT, padx=5)
        tk.Button(button_frame, text="Cancel", command=self.cancel).pack(side=tk.LEFT, padx=5)

        # Instructions
        instructions = tk.Label(self.root, text="Draw with mouse. Click 'Done' when finished.")
        instructions.pack()

        # Coordinate transformation parameters (robot workspace)
        self.x_min, self.x_max = -1.0, 1.0  # Robot workspace in meters
        self.y_min, self.y_max = -0.6, 0.6

    def canvas_to_robot_coords(self, canvas_x, canvas_y):
        # Convert canvas coordinates to robot workspace coordinates
        x = self.x_min + (canvas_x / self.width) * (self.x_max - self.x_min)
        y = self.y_max - (canvas_y / self.height) * (self.y_max - self.y_min)  # Flip Y
        return x, y

    def start_draw(self, event):
        self.drawing = True
        self.trajectory = []
        x, y = self.canvas_to_robot_coords(event.x, event.y)
        self.trajectory.append([x, y])

    def draw(self, event):
        if self.drawing:
            # Draw on canvas
            if len(self.trajectory) > 0:
                last_canvas_x = int((self.trajectory[-1][0] - self.x_min) / (self.x_max - self.x_min) * self.width)
                last_canvas_y = int((self.y_max - self.trajectory[-1][1]) / (self.y_max - self.y_min) * self.height)

                self.canvas.create_line(last_canvas_x, last_canvas_y, event.x, event.y,
                                        width=2, fill='blue', capstyle=tk.ROUND)

            # Add to trajectory
            x, y = self.canvas_to_robot_coords(event.x, event.y)
            self.trajectory.append([x, y])

    def stop_draw(self, event):
        self.drawing = False

    def clear(self):
        self.canvas.delete("all")
        self.trajectory = []

    def done(self):
        if len(self.trajectory) < 2:
            messagebox.showwarning("Warning", "Please draw a trajectory first!")
            return
        self.completed = True
        self.root.quit()

    def cancel(self):
        self.trajectory = []
        self.completed = False
        self.root.quit()

    def on_closing(self):
        self.completed = False
        self.root.quit()

    def get_trajectory(self):
        """Run the drawing interface and return the trajectory"""
        self.root.mainloop()
        if self.completed and len(self.trajectory) > 1:
            return np.array(self.trajectory)
        return None

class RealTimeMouseControl:
    def __init__(self, width=600, height=600):
        self.width = width
        self.height = height
        self.active = False
        self.current_pos = np.array([-0.2, 0.0])  # Default position
        self.trajectory_log = []

        # Create window
        self.root = tk.Toplevel()
        self.root.title("Real-time Mouse Control")
        self.root.geometry(f"{width}x{height + 140}")
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)

        # Canvas for mouse control
        self.canvas = tk.Canvas(self.root, width=width, height=height, bg='lightgray')
        self.canvas.pack(pady=10)

        # Bind mouse events
        self.canvas.bind("<Motion>", self.mouse_move)
        self.canvas.bind("<Button-1>", self.toggle_control)

        # Control state
        self.control_frame = tk.Frame(self.root)
        self.control_frame.pack(pady=10)

        self.status_label = tk.Label(self.control_frame, text="Click to start control")
        self.status_label.pack()

        tk.Button(self.control_frame, text="Stop Control", command=self.stop_control).pack(side=tk.LEFT, padx=5)
        tk.Button(self.control_frame, text="Save Log", command=self.save_trajectory).pack(side=tk.LEFT, padx=5)

        # Position display
        self.pos_label = tk.Label(self.root, text="Position: (0.30, 0.00)")
        self.pos_label.pack()

        # Coordinate transformation parameters
        self.x_min, self.x_max = -1.05, 1.05
        self.y_min, self.y_max = -0.65, 0.65

        # SENSITIVITY
        self.sensitivity = 2.0  # 1.0 = original mapping
        sens_frame = tk.Frame(self.root)
        sens_frame.pack(pady=4)
        tk.Label(sens_frame, text="Sensitivity").pack(side=tk.LEFT, padx=6)
        self.sens_slider = tk.Scale(
            sens_frame, from_=0.5, to=5.0, resolution=0.1,
            orient=tk.HORIZONTAL, length=220, command=self._on_sens_change
        )
        self.sens_slider.set(self.sensitivity)
        self.sens_slider.pack(side=tk.LEFT)

        self.running = True

    def _on_sens_change(self, val):
        try:
            self.sensitivity = float(val)
        except Exception:
            pass

    def canvas_to_robot_coords(self, canvas_x, canvas_y):
        nx = np.clip(canvas_x / self.width, 0.0, 1.0)
        ny = np.clip(canvas_y / self.height, 0.0, 1.0)

        xrange = (self.x_max - self.x_min) * self.sensitivity
        yrange = (self.y_max - self.y_min) * self.sensitivity

        x = self.x_min + nx * xrange
        y = self.y_max - ny * yrange

        # Clamp to original workspace limits
        x = float(np.clip(x, min(self.x_min, self.x_max), max(self.x_min, self.x_max)))
        y = float(np.clip(y, min(self.y_min, self.y_max), max(self.y_min, self.y_max)))
        return np.array([y,x])

    def mouse_move(self, event):
        if self.active:
            self.current_pos = self.canvas_to_robot_coords(event.x, event.y)
            self.trajectory_log.append(self.current_pos.copy())
            self.pos_label.config(text=f"Position: ({self.current_pos[0]:.3f}, {self.current_pos[1]:.3f})")

    def toggle_control(self, event):
        self.active = not self.active
        if self.active:
            self.status_label.config(text="Control ACTIVE - Move mouse to control robot")
            self.canvas.config(bg='lightgreen')
            self.trajectory_log = []
        else:
            self.status_label.config(text="Control STOPPED - Click to resume")
            self.canvas.config(bg='lightgray')

    def stop_control(self):
        self.active = False
        self.running = False
        self.status_label.config(text="Control stopped")
        self.canvas.config(bg='lightgray')

    def save_trajectory(self):
        if len(self.trajectory_log) > 0:
            import numpy as np
            np.savetxt("mouse_trajectory.csv", self.trajectory_log, delimiter=",",
                       header="x,y", comments="")
            messagebox.showinfo("Saved", f"Trajectory saved with {len(self.trajectory_log)} points")

    def on_closing(self):
        self.running = False
        self.active = False
        self.root.quit()

    def get_current_position(self):
        return self.current_pos.copy()

    def is_active(self):
        return self.active and self.running