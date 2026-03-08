# formatter.sh
find ros2bridge/ rosbag2rawlog/ -iname *.h -o -iname *.hpp -o -iname *.cpp -o -iname *.c | xargs clang-format-14 -i
