from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("ur5", package_name="ur5_moveit_config").to_moveit_configs()
    return generate_move_group_launch(moveit_config)
