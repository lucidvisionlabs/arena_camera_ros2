# docker build --rm -f "ros_eloquent_desktop.dockerfile" -t ros_eloquent_desktop:latest "."

# linux/amd64 only for now
#https://hub.docker.com/layers/osrf/ros/eloquent-desktop/images/sha256-742948bc521573ff962f5a7f084ba1562a319e547c3938603f8dff5d33d3466e?context=explore
FROM osrf/ros:eloquent-desktop

