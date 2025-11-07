from functools import partial
from io import TextIOWrapper
import os
import subprocess
import yaml

import numpy as np
import viser
from viser.extras import ViserUrdf
import yourdfpy
from robot_descriptions.loaders.yourdfpy import load_robot_description


class Gui:

    def __init__(self):
        self.server = viser.ViserServer()

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

    def update(self, leader_arm_pos, leader_gripper_pos):
        if self.cal_started:
            self.viser_urdf.update_cfg(np.zeros(7))
            return  # Ignore current config, only show zero config

        self.viser_urdf.update_cfg(leader_arm_pos)

        self.gripper_pos_bar.value = leader_gripper_pos * (100 / 0.8)  # 0.8 = closed

    def record_freq(self, new_freq):
        self.freq_text.content = f"{int(new_freq)} Hz"
