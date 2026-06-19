# drunk_avoid 브랜치 머지 절차

이 문서는 `drunk_avoid` 기능을 다른 사람들 브랜치와 합칠 때 충돌을 줄이기 위한 절차입니다.

핵심 원칙:
- `track_drive.py`를 통째로 덮어쓰지 않는다.
- 각자 맡은 노드는 가능하면 독립 파일로 유지한다.
- 기존 차선 인식, 신호등, 미션 로직 중간에 회피 코드를 끼워 넣지 않는다.
- `drunk_avoid`는 최종 motor publish 직전에만 override한다.

## 1. 머지 전에 각 브랜치 최신화

각자 자기 브랜치에서 먼저 원격 최신 상태를 받습니다.

```bash
git fetch origin
git checkout <내_브랜치>
git pull origin <내_브랜치>
```

작업 중인 변경사항이 있으면 먼저 커밋하거나 stash합니다.

```bash
git status
git add <내가_수정한_파일>
git commit -m "내 작업 설명"
```

## 2. 공통 기준 브랜치 정하기

팀에서 기준 브랜치 하나를 정합니다.

추천:
- 최종 제출용 브랜치가 있다면 그 브랜치
- 없다면 현재 통합이 가장 많이 된 브랜치

예:

```bash
git checkout drunk_avoid
git pull origin drunk_avoid
```

## 3. 다른 사람 브랜치를 하나씩 머지

여러 브랜치를 한 번에 합치지 말고 하나씩 합칩니다.

```bash
git checkout drunk_avoid
git merge origin/<상대_브랜치>
```

충돌이 나면 바로 해결하고, 빌드/실행 확인 후 다음 브랜치를 머지합니다.

## 4. `track_drive.py` 충돌 해결 기준

`track_drive.py`에서 충돌이 나면 아래 구조를 유지합니다.

유지해야 하는 기존 로직:
- 차선 인식 결과 구독
- 신호등 상태 구독
- 미션별 speed/angle 계산
- `main_loop()` 안의 기존 판단 흐름

`drunk_avoid`에서 추가되어야 하는 최소 로직:
- `/drunk_avoid_active` 구독
- `/drunk_avoid_cmd` 구독
- `drunk_avoid_active_callback()`
- `drunk_avoid_cmd_callback()`
- `drive()` 함수의 최종 publish 직전 override

`drive()` 함수는 아래 형태가 되어야 합니다.

```python
def drive(self, angle, speed):
    self.motor_msg.angle = float(angle)
    self.motor_msg.speed = float(speed)
    if self.drunk_avoid_active:
        self.motor_pub.publish(self.drunk_avoid_cmd)
    else:
        self.motor_pub.publish(self.motor_msg)
```

중요:
- `main_loop()` 중간에 `drunk_avoid` 조건문을 넣지 않습니다.
- 기존 차선/신호등/미션 판단 코드를 삭제하지 않습니다.
- `active=False`일 때 `/drunk_avoid_cmd` 값은 무시되어야 합니다.

## 5. `setup.py` 충돌 해결 기준

`console_scripts`에는 각자 노드가 모두 남아야 합니다.

예:

```python
entry_points={
    'console_scripts': [
        'track_drive = track_drive.track_drive:main',
        'lane_detector = track_drive.lane_detector_node:main',
        'traffic_light = track_drive.traffic_light_node:main',
        'drunk_avoid_node = track_drive.drunk_avoid_node:main',
    ],
},
```

충돌 해결 시 한 사람의 entry만 남기지 말고, 필요한 실행 명령을 모두 보존합니다.

## 6. 새 파일은 그대로 보존

`drunk_avoid` 기능은 아래 파일을 사용합니다.

```text
track_drive/drunk_avoid_node.py
```

이 파일은 다른 기능과 독립된 노드이므로, 특별한 이유 없이 다른 브랜치 코드로 덮어쓰지 않습니다.

패키지 구조상 필요하면 아래 파일도 유지합니다.

```text
package.xml
resource/track_drive
track_drive/__init__.py
```

## 7. 충돌 해결 후 검증

머지 충돌을 해결한 뒤 반드시 문법 검사를 먼저 합니다.

```bash
python3 -m py_compile track_drive/track_drive.py
python3 -m py_compile track_drive/drunk_avoid_node.py
python3 -m py_compile track_drive/lane_detector_node.py
python3 -m py_compile track_drive/traffic_light_node.py
```

그 다음 빌드합니다.

```bash
colcon build --packages-select track_drive
source install/setup.bash
```

실행 명령이 보이는지 확인합니다.

```bash
ros2 pkg executables track_drive
```

아래 실행 파일들이 보여야 합니다.

```text
track_drive track_drive
track_drive drunk_avoid_node
track_drive lane_detector
track_drive traffic_light
```

## 8. 실행 테스트 순서

터미널을 나누어 실행합니다.

```bash
ros2 launch ros_tcp_endpoint endpoint.py
```

```bash
ros2 run track_drive lane_detector
```

```bash
ros2 run track_drive traffic_light
```

```bash
ros2 run track_drive drunk_avoid_node
```

```bash
ros2 run track_drive track_drive
```

그 다음 `drive.exe`를 실행합니다.

## 9. 동작 확인 토픽

회피 노드 상태:

```bash
ros2 topic echo /drunk_avoid_active
ros2 topic echo /drunk_avoid_cmd
```

차선/신호등 토픽:

```bash
ros2 topic echo /lane_angle
ros2 topic echo /lane_departure
ros2 topic echo /traffic_action
```

센서 입력:

```bash
ros2 topic hz /usb_cam/image_raw/front
ros2 topic hz /scan
```

## 10. 최종 push 전 체크리스트

```bash
git status
git diff --stat
```

확인할 것:
- 내가 의도한 파일만 수정되었는가
- `track_drive.py`에서 다른 사람 로직을 삭제하지 않았는가
- `setup.py`에서 다른 사람 console script를 지우지 않았는가
- `drunk_avoid_node.py`가 포함되어 있는가
- 빌드가 통과했는가

문제 없으면 커밋 후 push합니다.

```bash
git add <수정한_파일들>
git commit -m "Merge drunk avoid integration"
git push origin <브랜치명>
```

## 11. 추천 머지 전략

가장 안전한 방식:
1. 통합 담당자 한 명이 기준 브랜치를 최신화한다.
2. 다른 사람 브랜치를 하나씩 merge한다.
3. 충돌이 나면 해당 기능 담당자와 같이 해결한다.
4. 매 merge마다 빌드와 최소 실행 테스트를 한다.
5. 마지막에 전체 노드를 동시에 켜서 drive.exe에서 통합 테스트한다.

피해야 할 방식:
- 여러 명이 동시에 같은 통합 브랜치에 push
- `track_drive.py`를 파일 전체 복사로 덮어쓰기
- 충돌 마커를 대충 지우고 빌드 없이 push
- `setup.py`에서 자기 노드 entry만 남기는 것

