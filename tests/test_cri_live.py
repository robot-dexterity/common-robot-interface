"""
Use (shift, ctrl, arrow keys) to move robot
"""
import hydra
import numpy as np
import time
from omegaconf import DictConfig

from cri.transforms import inv_transform_euler
from tactile_bench.data.utils.sensors import setup_embodiment

import tactile_bench.utils.keyboard as k

np.set_printoptions(precision=1, suppress=True)


def run_live_loop(robot, num_iterations=10000):

    sign = -1 if robot.name == "mg400" else 1          # MG400 has reversed coordinates
    reset_pose = (0, 0, -20 * sign, 0, 0, 0)

    # reset robot 
    robot.move_linear(reset_pose)                       # move to safe reset pose
    # robot.move_joints([*robot.joint_angles[:-1], 0])    # zero last joint
    pose_robot = reset_pose
    keyboard = k.Keyboard()

    # ==== demo loop ====
    with keyboard:
        for i in range(num_iterations):

            delta_pose = keyboard.get_state()
            pose_robot = inv_transform_euler(delta_pose, pose_robot)      # transform control output to robot frame
            robot.move_linear(pose_robot)   
            time.sleep(0.01) 

            print(f"{i+1}: " + "{}".format(robot.pose))

    # reset robot
    robot.close()
    
    robot.move_linear(reset_pose)                     # finish at reset pose
    # robot.move_joints((*robot.joint_angles[:-1], 0))  # zero last joint
    robot.close()


cfg_path, cfg_name = "../cfg", "app/collect"

@hydra.main(config_path=cfg_path, config_name=cfg_name, version_base=None)
def main(cfg: DictConfig):

    robot, _ = setup_embodiment(cfg.robot, cfg.sensor, cfg.environment)
    
    run_live_loop(robot)


if __name__ == "__main__":
    main()
