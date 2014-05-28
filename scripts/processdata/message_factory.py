from collections import namedtuple

GRIPPER_COMMAND_TYPE = 'pr2_controllers_msgs/Pr2GripperCommand'
MARKER_FEEDBACK_TYPE = 'visualization_msgs/InteractiveMarkerFeedback'
POSE_STAMPED_TYPE = 'geometry_msgs/PoseStamped'
POSE_TYPE = 'geometry_msgs/Pose'
SCREEN_POSITION_TYPE = 'rviz_experiment/ScreenPosition'
STRING_TYPE = 'std_msgs/String'

Point = namedtuple('Point', ['x', 'y', 'z'])
Quaternion = namedtuple('Quaternion', ['w', 'x', 'y', 'z'])
Pose = namedtuple('Pose', ['position', 'orientation'])
Pr2GripperCommand = namedtuple('Pr2GripperCommand', ['position'])
InteractiveMarkerFeedback = namedtuple(
  'InteractiveMarkerFeedback',
  ['marker_name', 'control_name', 'event_type', 'pose']
)
ScreenPosition = namedtuple('ScreenPosition', ['x', 'y'])
String = namedtuple('String', ['data'])

def make_pose(message):
  position = Point(message.position.x, message.position.y, message.position.z)
  orientation = Quaternion(
    message.orientation.w,
    message.orientation.x,
    message.orientation.y,
    message.orientation.z
  )
  return Pose(position, orientation)

def model(message):
  if message._type == POSE_TYPE:
    return make_pose(message)
  elif message._type == POSE_STAMPED_TYPE:
    return make_pose(message.pose)
  elif message._type == GRIPPER_COMMAND_TYPE:
    return Pr2GripperCommand(message.position)
  elif message._type == MARKER_FEEDBACK_TYPE:
    return InteractiveMarkerFeedback(
      message.marker_name,
      message.control_name,
      message.event_type,
      make_pose(message.pose)
    )
  elif message._type == SCREEN_POSITION_TYPE:
    return ScreenPosition(message.x, message.y)
  elif message._type == STRING_TYPE:
    return String(message.data)
  else:
    return None

