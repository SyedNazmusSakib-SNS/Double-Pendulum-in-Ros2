import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/media/sns/Study/1_RME/Ros2/ros2_ws/install/nodes_topics_demo'
