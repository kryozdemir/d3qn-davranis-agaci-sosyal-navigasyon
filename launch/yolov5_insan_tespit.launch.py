from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="d3qn_sosyal_navigasyon",
            executable="insan_tespit_node",
            name="yolov5_insan_tespit",
            output="screen",
            parameters=[{
                "gorsel_topic": "/camera/image_raw",
                "yolov5_repo_yolu": "~/ros2_ws/src/yolov5",
                "agirlik_yolu": "",
                "guven_esigi": 0.35,
                "iou_esigi": 0.45,
                "goruntu_boyutu": 640,
                "cihaz": "auto",
                "insan_var_esigi": 1,
            }],
        ),
    ])
