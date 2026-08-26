from launch import LaunchDescription
from launch.actions import LogInfo


def generate_launch_description():
    # Modification: 2026-08-21, WenSheng Xu.
    # Reason: Keep the launch graph composable while making the unimplemented
    # status explicit. No fake optimized path or TF is published.
    return LaunchDescription(
        [
            LogInfo(
                msg=(
                    "fusionlynx_loop_closure interface is reserved; "
                    "no loop-closure node is implemented or started."
                )
            )
        ]
    )
