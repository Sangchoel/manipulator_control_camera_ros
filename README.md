# 🤖 YOLOv7 ROS Depth Control System

YOLOv7 객체 감지 + Depth 카메라를 기반으로  
3D 공간 인식 → 좌표 추출 → 조건 기반 자동 명령 전달(Action A/B/C)을 수행하는  
**지능형 시각 기반 ROS 제어 시스템**입니다.

UI 키 입력도 함께 처리하며, 객체 중심 위치가 지정된 구간에 진입하면  
자동으로 명령 메시지를 `/keyboard_action_result`로 퍼블리시합니다.

---

## 📦 전체 구성 개요

| 모듈 | 설명 |
|------|------|
| `YOLOv7 ROS 노드` | RGB 영상으로 객체 감지 + 중심점 PointCenter 퍼블리시 |
| `Depth 연동 노드` | 중심 좌표(x, y)에 해당하는 depth 값을 참조하여 3D 좌표 계산 |
| `Action 판단` | X, Y, Z 좌표가 특정 범위 내에 들어오면 Action A/B/C 실행 |
| `UI 통합` | `/UI_result`를 수신하여 사전 정의된 문자 시퀀스를 전송 |

---

## 🧩 주요 토픽 구조

| 토픽명 | 메시지 타입 | 설명 |
|--------|--------------|------|
| `/camera/depth/image_raw` | `sensor_msgs/Image` | Depth 영상 |
| `/yolov7/pointcenter_topic` | `yolov7_ros/PointCenterArray` | YOLO 감지된 객체 중심 좌표 |
| `/keyboard_action_result` | `std_msgs/String` | Action 키 전송 (ex. 'g', 'i' 등) |
| `/UI_result` | `std_msgs/String` | 외부 UI 입력 ('a', 'b', 'c') |

---

## 🎯 동작 로직 요약

### ✅ Depth + YOLO 기반 자동 액션 처리

1. YOLOv7이 객체 감지 → 중심 좌표 발행  
2. Depth 이미지에서 (x, y)에 해당하는 거리 `Z` 추출  
3. 아래 조건에 따라 액션 구분 후 `/keyboard_action_result`에 전송

```cpp
if (-80 <= X <= -50 && -120 <= Y <= -80 && Z <= 1000)
    Action A
else if (60 <= X <= 120 && -160 <= Y <= -90 && Z <= 1000)
    Action B
else if (190 <= X <= 250 && -100 <= Y <= -70 && Z <= 1000)
    Action C
