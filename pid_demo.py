import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button
import matplotlib.animation as animation

class PIDResponseGraph:
    def __init__(self):
        #  Values used for PID response
        self._errors = []
        self._integrals = []
        self._derivatives = []
        self._values = []
        self._t = []

        # Parameters used for PID controller
        self._setpoint = 1.0
        self._Kp = 4.0
        self._Ki = 0.0
        self._Kd = 0.0

        self.simulate_pid_response()
        self._init_plots()
        self._init_ui()

        self._update_plot()
        plt.show()

    def _init_plots(self):
        self._figure, self._ax = plt.subplots(figsize=(10, 6))
        plt.subplots_adjust(left=0.1, bottom=0.35, right=0.85)  # Make room for text on right
        self._output_line, = self._ax.plot(self._t, self._values, label='Output')
        self._ax.plot(self._t, np.ones_like(self._t), '--', label='Setpoint')
        self._time_marker = self._ax.axvline(0, color='red', lw=1)

        # Add tangent line for derivative
        self._tangent_line, = self._ax.plot([], [], 'g--', label='Derivative', alpha=0.7, linewidth=2)

        # Add integral area
        self._integral_area = self._ax.fill_between([], [], [], alpha=0.3, color='orange', label='Integral')

        # Add text box for current values
        self._value_text = self._ax.text(
            1.02, 0.5, '', transform=self._ax.transAxes, 
            bbox=dict(facecolor='white', alpha=0.8, edgecolor='gray'),
            verticalalignment='center'
        )

        self._ax.set_ylim(0, 2)
        self._ax.set_title('PID Response')
        self._ax.set_xlabel('Time (s)')
        self._ax.set_ylabel('Output')
        self._ax.legend()

    def _init_ui(self):
        # Create axes for sliders with more vertical space
        axcolor = 'lightgoldenrodyellow'
        self._ax_kp = plt.axes([0.1, 0.25, 0.3, 0.03], facecolor=axcolor)
        self._ax_ki = plt.axes([0.1, 0.20, 0.3, 0.03], facecolor=axcolor)
        self._ax_kd = plt.axes([0.1, 0.15, 0.3, 0.03], facecolor=axcolor)
        self._ax_time = plt.axes([0.1, 0.05, 0.8, 0.03], facecolor=axcolor)
        self._ax_play = plt.axes([0.45, 0.15, 0.08, 0.04])

        # Create sliders
        self._slider_kp = Slider(self._ax_kp, 'Kp', 0.0, 10.0, valinit=self._Kp)
        self._slider_ki = Slider(self._ax_ki, 'Ki', 0.0, 5.0, valinit=self._Ki)
        self._slider_kd = Slider(self._ax_kd, 'Kd', 0.0, 5.0, valinit=self._Kd)
        self._slider_time = Slider(self._ax_time, 'Time', 0.0, self._t[-1], valinit=0.0, valstep=self._t[1])
        self._btn_play = Button(self._ax_play, 'Play')

        self._is_playing = False
        self._frame_idx = 0

        # Connect callbacks
        self._slider_kp.on_changed(self._update_plot)
        self._slider_ki.on_changed(self._update_plot)
        self._slider_kd.on_changed(self._update_plot)
        self._slider_time.on_changed(self._on_time_change)
        self._btn_play.on_clicked(self._toggle_play)

        self._ani = animation.FuncAnimation(self._figure, self._animate, interval=0.1)

    def simulate_pid_response(self, duration=10.0, dt=0.01):
        t = np.arange(0, duration, dt)
        x = 0.0  # position
        v = 0.0  # velocity
        integral = 0.0
        previous_error = 0.0
        values, errors, integrals = [], [], []

        for _ in range(len(t)):
            error = self._setpoint - x
            integral += error * dt
            derivative = (error - previous_error) / dt
            u = self._Kp * error + self._Ki * integral + self._Kd * derivative

            # Second-order system: d²x/dt² = -x - 0.1*dx/dt + u
            # This is equivalent to a mass-spring-damper system
            dx = v
            dv = -x - 0.1*v + u
            x += dx * dt
            v += dv * dt

            values.append(x)
            errors.append(error)
            integrals.append(integral)
            previous_error = error

        self._t = t
        self._values = np.array(values)
        self._errors = np.array(errors)
        self._integrals = np.array(integrals)

    def _update_data(self, idx):
        # Calculate PID components
        error = self._errors[idx]
        Kp, Ki, Kd = self._slider_kp.val, self._slider_ki.val, self._slider_kd.val
        p_term = Kp * error
        i_term = Ki * self._integrals[idx]
        
        # Calculate derivative using central difference
        if idx > 0 and idx < len(self._t) - 1:
            derivative = (self._errors[idx+1] - self._errors[idx-1]) / (2 * (self._t[1] - self._t[0]))
        else:
            derivative = 0

        d_term = Kd * derivative

        self._value_text.set_text(
            f'Current Values:\n'
            f'Error: {error:.3f}\n'
            f'Integral: {self._integrals[idx]:.3f}\n'
            f'Derivative: {derivative:.3f}\n'
            f'P-term: {p_term:.3f}\n'
            f'I-term: {i_term:.3f}\n'
            f'D-term: {d_term:.3f}'
        )

        # Update tangent line for derivative
        if idx > 0 and idx < len(self._t) - 1:
            slope = (self._values[idx+1] - self._values[idx-1]) / (2 * (self._t[1] - self._t[0]))
            x_tangent = np.array([self._t[idx] - 0.5, self._t[idx] + 0.5])
            y_tangent = self._values[idx] + slope * (x_tangent - self._t[idx])
            self._tangent_line.set_data(x_tangent, y_tangent)
        else:
            self._tangent_line.set_data([], [])

        # Update integral area
        try:
            self._integral_area.remove()
        except:
            pass
        self._integral_area = self._ax.fill_between(
            self._t[:idx+1],
            self._setpoint,
            self._values[:idx+1],
            alpha=0.3,
            color='orange'
        )

    def _update_plot(self, val=None):
        # Update PID values from sliders
        self._Kp = self._slider_kp.val
        self._Ki = self._slider_ki.val
        self._Kd = self._slider_kd.val
        
        # Recalculate the response with new values
        self.simulate_pid_response()
        
        # Update the plot
        self._output_line.set_ydata(self._values)
        self._slider_time.valmax = self._t[-1]
        self._slider_time.ax.set_xlim(self._slider_time.valmin, self._slider_time.valmax)
        
        # Clear integral area when parameters change
        try:
            self._integral_area.remove()
        except:
            pass
        self._integral_area = self._ax.fill_between([], [], [], alpha=0.3, color='orange')
        
        # Update the current time point
        self._on_time_change(self._slider_time.val)
        self._figure.canvas.draw_idle()

    def _on_time_change(self, val):
        idx = int(val / (self._t[1] - self._t[0]))
        idx = min(max(0, idx), len(self._t) - 1)  # Ensure idx is within bounds
        self._update_data(idx)
        self._time_marker.set_xdata([val])
        self._figure.canvas.draw_idle()

    def _toggle_play(self, _):
        self._is_playing = not self._is_playing
        self._btn_play.label.set_text('Pause' if self._is_playing else 'Play')

    def _animate(self, _):
        if self._is_playing:
            idx = self._frame_idx
            if idx < len(self._t) - 1:
                self._slider_time.set_val(self._t[idx])
                self._frame_idx += 10
            else:
                self._is_playing = False
                self._frame_idx = 0
                self._btn_play.label.set_text('Play')

PIDResponseGraph()
