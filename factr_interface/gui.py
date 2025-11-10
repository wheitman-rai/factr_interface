from functools import partial
from io import TextIOWrapper
import os
import subprocess
import yaml
from collections import deque
import time

import numpy as np
import viser
from viser.extras import ViserUrdf
import yourdfpy
from robot_descriptions.loaders.yourdfpy import load_robot_description
import plotly.graph_objects as go


class Gui:

    def __init__(self, num_joints=7, history_length=500):
        self.server = viser.ViserServer()

        self.num_joints = num_joints
        self.history_length = history_length

        # Initialize torque history storage (deques for efficient append/pop)
        self.torque_history = [deque(maxlen=history_length) for _ in range(num_joints)]
        self.time_history = deque(maxlen=history_length)
        self.time_start = None

        self.cal_started = False

        # Add a Franka to the scene

        urdf = load_robot_description(
            "panda_description",
            load_meshes=True,
            build_scene_graph=True,
            load_collision_meshes=False,
            build_collision_scene_graph=False,
        )

        urdf = yourdfpy.URDF.load(
            "/home/wheitman/wheitman_ws/src/external/FACTR_Teleop/src/factr_teleop/factr_teleop/urdf/factr_teleop_franka.urdf",
            build_scene_graph=True,
            build_collision_scene_graph=False,
            load_meshes=True,
            load_collision_meshes=False,
            filename_handler=partial(
                yourdfpy.filename_handler_magic,
                dir="/home/wheitman/wheitman_ws/src/external/FACTR_Teleop/src/factr_teleop/factr_teleop/urdf/",
            ),
        )

        self.viser_urdf = ViserUrdf(
            self.server,
            urdf_or_path=urdf,
            load_meshes=True,
            load_collision_meshes=False,
            collision_mesh_color_override=(1.0, 0.0, 0.0, 0.5),
        )

        self.cal_button = self.server.gui.add_button("Start Calibration")
        self.explanatory_text = self.server.gui.add_markdown(
            "Click the button to calibrate FACTR's joint angles"
        )
        self.freq_text = self.server.gui.add_markdown("null Hz")
        self.gripper_pos_bar = self.server.gui.add_progress_bar(0.0)
        self.gripper_pos_bar.label = "Gripper"
        self.gripper_pos_bar.hint = "Open/Closed"

        # Add torque progress bars for each joint
        self.torque_bars = []
        self.max_torque_display = 10.0  # Max torque for progress bar scaling (Nm)
        for i in range(self.num_joints):
            
            bar_label = self.server.gui.add_markdown(f"Joint {i+1} Torque")
            
            bar = self.server.gui.add_progress_bar(0.0)
            bar.label = f"Joint {i+1} Torque"
            bar.hint = "Current applied torque (Nm)"
            self.torque_bars.append(bar)

        # Add torque plot using Plotly
        # Create initial empty figure
        self.fig = go.Figure()
        self.fig.update_layout(
            title="Joint Torques Over Time",
            xaxis_title="Time (s)",
            yaxis_title="Torque (Nm)",
            showlegend=True,
            height=400,
        )
        # Add traces for each joint
        for i in range(self.num_joints):
            self.fig.add_trace(
                go.Scatter(
                    x=[],
                    y=[],
                    mode='lines',
                    name=f'Joint {i+1}',
                    line=dict(width=2)
                )
            )

        self.torque_plot = self.server.gui.add_plotly(figure=self.fig)

        initial_config: list[float] = []
        for joint_name, (
            lower,
            upper,
        ) in self.viser_urdf.get_actuated_joint_limits().items():
            lower = lower if lower is not None else -np.pi
            upper = upper if upper is not None else np.pi
            initial_pos = 0.0 if lower < -0.1 and upper > 0.1 else (lower + upper) / 2.0
            initial_config.append(initial_pos)

        # Start in zero position until told otherwise
        self.viser_urdf.update_cfg(np.zeros(7))

        # @self.cal_button.on_click
        # def _(_) -> None:
        #     print("Button clicked!")

        #     self.cal_started = not self.cal_started

        #     if self.cal_started:  # Now in calibration mode
        #         self.explanatory_text.content = "Move FACTR to match the zero configuration shown. Be sure it matches!"
        #         self.cal_button.label = "Finish Calibration"
        #         start_cal_cb()

        #     else:  # Calibration just finished
        #         self.cal_button.label = "Start Calibration"
        #         self.explanatory_text.content = (
        #             "Click the button to calibrate FACTR's joint angles"
        #         )
        #         finish_cal_cb()

    def update(self, leader_arm_pos, leader_gripper_pos, joint_torques=None):
        if self.cal_started:
            self.viser_urdf.update_cfg(np.zeros(7))
            return  # Ignore current config, only show zero config

        self.viser_urdf.update_cfg(leader_arm_pos)

        self.gripper_pos_bar.value = leader_gripper_pos * (100 / 0.8)  # 0.8 = closed

        # Update torque visualization if torque data is provided
        if joint_torques is not None:
            self._update_torque_visualization(joint_torques)

    def _update_torque_visualization(self, joint_torques):
        """Update torque progress bars and plot with new torque data."""
        # Initialize time if this is the first update
        if self.time_start is None:
            self.time_start = time.time()

        # Calculate current time
        current_time = time.time() - self.time_start

        # Add to history
        self.time_history.append(current_time)
        for i in range(self.num_joints):
            self.torque_history[i].append(joint_torques[i])

        # Update progress bars
        for i in range(self.num_joints):
            # Map torque to 0-100 range, handling both positive and negative torques
            torque_normalized = (abs(joint_torques[i]) / self.max_torque_display) * 100
            torque_normalized = min(100, torque_normalized)  # Cap at 100%
            self.torque_bars[i].value = torque_normalized
            self.torque_bars[i].hint = f"{joint_torques[i]:.2f} Nm"

        # Update plot every N frames to avoid performance issues
        if len(self.time_history) % 5 == 0:  # Update every 5 frames
            # Convert deques to numpy arrays for plotting
            time_array = np.array(list(self.time_history))

            # Update each trace
            for i in range(self.num_joints):
                torque_array = np.array(list(self.torque_history[i]))
                self.fig.data[i].x = time_array
                self.fig.data[i].y = torque_array

            # Update the plot
            self.torque_plot.figure = self.fig

    def record_freq(self, new_freq):
        self.freq_text.content = f"{int(new_freq)} Hz"
