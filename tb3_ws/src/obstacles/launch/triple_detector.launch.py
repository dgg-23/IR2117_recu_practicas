from launch import LaunchDescription
from launch_ros.actions import Node
import launch.actions

def generate_launch_description():
	return LaunchDescription([
		Node(
			package='obstacles',
			executable='detector',
			namespace='front',
			remappings = [
			  ('/front/scan', '/scan')
			],
			parameters=[
			  {"obs_angle_min": -0.3927},
			  {"obs_angle_max": 0.3927},
			  {"obs_angle_": 1.0}]
		),
		Node(
			package='obstacles',
			executable='detector',
			namespace='left',
			remappings = [
			  ('/left/scan', '/scan'),
			],
			parameters=[
			  {"obs_angle_min": 0.3927},
			  {"obs_angle_max": 1.1781},
			  {"obs_angle_": 1.0}]
		),
		Node(
			package='obstacles',
			executable='detector',
			namespace='right',
			remappings = [
			  ('/right/scan', '/scan'),
			],
			parameters=[
			  {"obs_angle_min": -1.1781},
			  {"obs_angle_max": -0.3927},
			  {"obs_angle_": 1.0}]
		),
		Node(
		package='obstacles',
		executable='avoidance'
		)
	])
