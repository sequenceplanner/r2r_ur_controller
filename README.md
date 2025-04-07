# r2r_ur_controller


maybe each robot should have a docker container. Then we have a redis interface to the state, a loop, which writes the robot state to the redis, and which also receives commands to move the robot. In the docker, we run r2r and ros2, and we do the action interface between the ur_script_driver and the redis interface, and we expoes the robot states and the rest to redis via it. Then, ideally, we could just launch several instances of robots with these dockers.?