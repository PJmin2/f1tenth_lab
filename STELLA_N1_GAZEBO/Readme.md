#### STELLA N1 GAZEBO

STELLA N1 GAZEBO is a package for running STELLA N1 in a simulation environment.

STELLA N1 GAZEBO는 STELLA N1을 시뮬레이션 환경에서 구동하기 위한 패키지 입니다.

It was developed based on ros humble.

ros humble을 기반으로 개발되었습니다.

---

##### ROS command

+ ros2 launch stella_gazebo start_world.launch.py

Commands that run empty Gazebo World.

빈 가제보 월드를 실행하는 명령어 입니다.

---

+ ros2 launch stella_gazebo spawn_stella_slam.launch.py

Command that spawn STELLA N1 for SLAM in Gazebo World.

가제보 월드에 SLAM용 STELLA N1을 불러오는 명령어 입니다.

---

+ ros2 launch stella_gazebo spawn_stella_ros2.launch.py

Command that spawn STELLA N1 for Navigation2 or Teleoperation in Gazebo World.

가제보 월드에 Navigation2 혹은 Teleoperation을 위한 STELLA N1을 불러오는 명령어 입니다.

---

+ ros2 launch stella_cartographer cartographer.launch.py

Command to execute Cartographer SLAM. Some code has been changed for Gazebo.

Cartographer SLAM을 실행하는 명령어 입니다. 가제보를 위해 일부 코드가 변경되었습니다.

---

+ ros2 run nav2_map_server map_saver_cli -f ~/map

Command to store the SLAM map in the home directory.

SLAM 진행 중인 맵을 홈 디렉토리에 저장하는 명령어 입니다.

---

+ ros2 launch stella_navigation2 navigation2.launch.py map:=$HOME/map.yaml

Command to import a map of the home directory and execute navigation2.

홈 디렉토리의 맵을 불러와 Navigation2를 실행하는 명령어 입니다.

---

+ ros2 run stella_teleop teleop_keyboard

Command to run keyboard teleoperation.

키보드 Teleoperation을 실행하는 명령어 입니다. 

---

We will upload detailed instructions through [idearobot gitbook](https://idearobot.gitbook.io/idearobot), [Blog](https://blog.naver.com/idea_robot), and [YouTube](https://www.youtube.com/@idearobot).

[idearobot gitbook](https://idearobot.gitbook.io/idearobot)과 [블로그](https://blog.naver.com/idea_robot), [유튜브](https://www.youtube.com/@idearobot) 등을 통하여 자세한 사용 방법을 업로드할 예정입니다.


If you have any questions, please feel free to ask our [forum](https://idea.synology.me/).

문의 사항이 있으신 경우 저희 [포럼](https://idea.synology.me/)에 질문하여 주세요.
