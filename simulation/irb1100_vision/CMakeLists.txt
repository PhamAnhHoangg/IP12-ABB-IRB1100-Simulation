cmake_minimum_required(VERSION 3.8)
project(irb1100_vision)
find_package(ament_cmake REQUIRED)
install(DIRECTORY urdf launch config rviz worlds meshes DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/fov_markers.py scripts/pick_sequence.py scripts/pick_panel.py scripts/gripper.py scripts/points_relay.py
                 scripts/ip12_sim.py scripts/box_detector.py scripts/make_dataset.py scripts/handeye_calib.py DESTINATION lib/${PROJECT_NAME})
ament_package()
