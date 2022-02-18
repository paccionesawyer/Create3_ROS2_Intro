import os

os.system("export RMW_IMPLEMENTATION=rmw_fastrtps_cpp")
os.system("export FASTRTPS_DEFAULT_PROFILES_FILE=/home/ubuntu/unicast.xml")

os.system("pkill -9 _ros2_daemon")
os.system("ros2 topic list --no-daemon --spin-time 10")