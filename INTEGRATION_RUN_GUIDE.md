# 통합 실행 가이드

이 문서는 팀 기능 브랜치를 모두 머지한 뒤 실제로 `drive.exe`에서 테스트할 때 필요한 실행 순서를 정리한 문서입니다.

## 1. 구현 대상 기능 브랜치

팀에서 통합해야 하는 기능은 다음과 같습니다.

| 기능 | 역할 |
| --- | --- |
| 신호등 | 신호등 상태를 인식하고 출발, 정지, 좌회전, 감속 판단 |
| 차선 | 카메라 기반 차선 인식 및 기본 주행 angle 생성 |
| 라바콘 | 라바콘 구간 감지 및 회피/주행 판단 |
| 지름길 | 지름길 구간 진입/주행 판단 |
| 주취자 | 파란색/남색 옷 주취자 감지, 정지/회피 명령 생성 |
| 차량 피하기 | 전방/주변 차량 감지 및 회피 판단 |

## 2. 전체 구조

`ros2 launch ros_tcp_endpoint endpoint.py`만 실행하면 전체 주행이 동작하지 않습니다.

이 명령은 Unity 또는 `drive.exe`와 ROS2를 연결하는 TCP 서버만 실행합니다.

통합 테스트에서는 각 기능 노드와 최종 주행 노드를 함께 실행해야 합니다.

기본 구조:

```text
drive.exe
  |
ros_tcp_endpoint
  |
센서 토픽들
  |
기능 노드들
  |
track_drive
  |
xycar_motor
```

## 3. 빌드

머지 후 먼저 빌드합니다.

```bash
cd ~/xycar_ws
colcon build --packages-select track_drive
source install/setup.bash
```

빌드가 실패하면 실행 테스트를 하지 말고 먼저 충돌 파일을 확인합니다.

자주 확인할 파일:

```text
track_drive/track_drive.py
track_drive/lane_detector_node.py
track_drive/traffic_light_node.py
track_drive/drunk_avoid_node.py
setup.py
package.xml
```

## 4. 실행 파일 확인

빌드 후 실행 가능한 노드 목록을 확인합니다.

```bash
ros2 pkg executables track_drive
```

예상 예시:

```text
track_drive track_drive
track_drive lane_detector
track_drive traffic_light
track_drive drunk_avoid_node
```

라바콘, 지름길, 차량 피하기 노드가 별도 노드로 구현되었다면 그 실행 명령도 여기 보여야 합니다.

예:

```text
track_drive cone_detector
track_drive shortcut_node
track_drive car_avoid_node
```

실행 파일이 안 보이면 `setup.py`의 `console_scripts`에 누락된 것입니다.

## 5. 기본 실행 순서

터미널을 여러 개 열고 각각 실행합니다.

### 터미널 1: ROS-TCP Endpoint

```bash
cd ~/xycar_ws
source install/setup.bash
ros2 launch ros_tcp_endpoint endpoint.py
```

이 노드는 `drive.exe`와 ROS2를 연결합니다.

### 터미널 2: 차선 인식

```bash
cd ~/xycar_ws
source install/setup.bash
ros2 run track_drive lane_detector
```

차선 인식 결과 토픽을 publish합니다.

예상 토픽:

```text
/lane_angle
/lane_departure
```

### 터미널 3: 신호등 인식

```bash
cd ~/xycar_ws
source install/setup.bash
ros2 run track_drive traffic_light
```

신호등 판단 결과를 publish합니다.

예상 토픽:

```text
/traffic_action
```

### 터미널 4: 주취자 회피

```bash
cd ~/xycar_ws
source install/setup.bash
ros2 run track_drive drunk_avoid_node
```

주취자 감지 및 회피 명령을 publish합니다.

예상 토픽:

```text
/drunk_avoid_active
/drunk_avoid_cmd
```

### 터미널 5: 최종 주행 노드

```bash
cd ~/xycar_ws
source install/setup.bash
ros2 run track_drive track_drive
```

최종적으로 `xycar_motor`를 publish하는 노드입니다.

이 노드는 차선, 신호등, 주취자 회피 등의 결과를 받아 최종 angle/speed를 결정합니다.

### 터미널 6 이후: 추가 기능 노드

라바콘, 지름길, 차량 피하기가 별도 노드로 구현되었다면 각각 추가로 실행합니다.

예시:

```bash
ros2 run track_drive cone_detector
```

```bash
ros2 run track_drive shortcut_node
```

```bash
ros2 run track_drive car_avoid_node
```

실제 명령어 이름은 각 기능이 `setup.py`에 등록한 `console_scripts` 이름을 따릅니다.

## 6. drive.exe 실행

ROS 노드들을 먼저 실행한 뒤 `drive.exe`를 실행합니다.

권장 순서:

1. `ros_tcp_endpoint`
2. 기능 노드들
3. `track_drive`
4. `drive.exe`

`drive.exe`를 먼저 켜도 연결될 수 있지만, 테스트할 때는 ROS 노드를 먼저 켜는 편이 상태 확인이 쉽습니다.

## 7. 토픽 확인 명령어

전체 토픽 목록:

```bash
ros2 topic list
```

센서 입력 확인:

```bash
ros2 topic hz /usb_cam/image_raw/front
ros2 topic hz /scan
```

차선 토픽:

```bash
ros2 topic echo /lane_angle
ros2 topic echo /lane_departure
```

신호등 토픽:

```bash
ros2 topic echo /traffic_action
```

주취자 회피 토픽:

```bash
ros2 topic echo /drunk_avoid_active
ros2 topic echo /drunk_avoid_cmd
```

최종 모터 명령:

```bash
ros2 topic echo /xycar_motor
```

## 8. 주취자 회피 적용 기준

`drunk_avoid_node`는 아래 토픽을 publish합니다.

```text
/drunk_avoid_active
/drunk_avoid_cmd
```

`track_drive`는 최종 motor publish 직전에만 이 값을 확인해야 합니다.

기준:

```text
drunk_avoid_active == True
  -> drunk_avoid_cmd를 xycar_motor로 publish

drunk_avoid_active == False
  -> 기존 차선/신호등/미션 주행 명령을 그대로 publish
```

중요:
- `drunk_avoid_active=False`일 때 `/drunk_avoid_cmd`가 `angle=0`, `speed=0`이어도 무시해야 합니다.
- 기존 차선, 신호등, 라바콘, 지름길, 차량 피하기 로직 중간에 주취자 회피 조건을 끼워 넣지 않습니다.
- 최종 publish 직전 override만 유지합니다.

## 9. 통합 시 우선순위 정하기

여러 기능이 동시에 명령을 낼 수 있으므로 우선순위를 정해야 합니다.

추천 우선순위:

1. 긴급 정지
2. 차량 피하기
3. 주취자 회피
4. 라바콘 회피
5. 신호등 정지/좌회전/감속
6. 지름길 판단
7. 기본 차선 주행

팀에서 다른 우선순위를 정했다면 `track_drive.py`의 최종 결정 부분에 명확히 반영합니다.

단, 각 기능의 감지 로직은 가능하면 독립 노드에 두고, `track_drive.py`에서는 최종 선택만 하도록 유지합니다.

## 10. 머지 후 체크리스트

실행 전:

```bash
git status
python3 -m py_compile track_drive/*.py
colcon build --packages-select track_drive
source install/setup.bash
ros2 pkg executables track_drive
```

실행 후:

```bash
ros2 node list
ros2 topic list
ros2 topic echo /xycar_motor
```

확인할 것:
- 필요한 노드가 모두 실행 중인가
- 센서 토픽이 들어오는가
- 각 기능 토픽이 publish되는가
- `xycar_motor`가 최종적으로 publish되는가
- 특정 기능 active가 False일 때 기존 주행이 막히지 않는가

## 11. 자주 생기는 문제

### `ros2 launch ros_tcp_endpoint endpoint.py`만 실행했는데 차가 안 움직임

정상입니다.

이 명령은 TCP 연결만 실행합니다.

차선, 신호등, 주취자 회피, 최종 주행 노드를 별도로 실행해야 합니다.

### `ros2 run track_drive drunk_avoid_node`가 멈춘 것처럼 보임

정상입니다.

ROS 노드는 실행 후 계속 살아 있으면서 토픽을 기다립니다.

### `ros2 run track_drive <노드명>`이 안 됨

`setup.py`의 `console_scripts`에 등록되어 있는지 확인합니다.

수정 후 다시 빌드합니다.

```bash
colcon build --packages-select track_drive
source install/setup.bash
```

### 머지 후 누군가의 기능이 사라짐

대부분 `track_drive.py` 또는 `setup.py` 충돌 해결 중 한쪽 코드를 통째로 선택해서 생깁니다.

해결:
- `setup.py`의 `console_scripts`에 모든 노드가 남아 있는지 확인
- `track_drive.py`의 callback, subscriber, 최종 판단 로직이 모두 남아 있는지 확인
- 파일 전체 덮어쓰기 대신 필요한 블록만 병합

