## 기존 legacy 구조의 한계
 - God Object (신 객체) 안티 패턴:
   - `ErrorMonitorNode`가 모든 데이터를 구독하고, 모든 모니터의 업데이트 상태를 `bool update_...` 변수로 관리하며, 모든 퍼블리셔를 소유하고 있음.
   - 에러 종류를 하나 추가할 때마다 노드의 헤더와 소스 코드(변수 선언, 초기화, 타이머 로직, 파라미터 로드 등)를 최소 10군데 이상 수정해야 함. 이는 `개방-폐쇄 원칙(OCP)`에 정면으로 위배

 - 수동 타이머 스케줄링:
   - `publish_cnt_... += 10;` 형태로 10ms 단일 타이머 안에서 각 모니터의 실행 주기를 수동으로 계산하고 음.
   - 이는 ROS2가 기본적으로 제공하는 강력한 `timer_` 스케줄링 기능을 낭비하는 것이며, 연산 부하가 큰 모니터가 생기면 전체 10ms 주기가 밀릴(Delay) 위험이 있음.

 - static 지역 변수 남용 (가장 치명적):
   - `FallDownErrorMonitor`, `LowBatteryErrorMonitor` 등의 **checkError 함수 내부**에 `static 변수`(예: static bool prev_state, static rclcpp::Clock clock)가 선언.
   - 이는 객체의 상태를 함수 스코프에 종속시켜버려 단위 테스트(Unit Test)를 불가능하게 만들고, 동일한 모니터 객체를 두 개 이상 생성할 수 없게 (재사용성 파괴) 만듦.
   - 상태는 반드시 클래스의 멤버 변수로 관리되어야 함.

 - 복잡한 타입 캐스팅:
   - `monitors_` "map"이 `std::shared_ptr<void>`를 값으로 갖고 있어, runMonitor 템플릿 함수에서 런타임 타입 캐스팅을 강제하고 있음.


## 개선 Point!
 - **`Blackboard 패턴` + `다형성(Polymorphism)`**
 - Blackboard 데이터 저장소:
   - 노드는 단순히 센서 데이터를 구독하여 "**하나의 중앙 구조체(Blackboard)에 최신화**"만 하면 됨.
 - 공통 인터페이스 단일화:
   - 모든 `error_monitor`는 `Blackvoard` 에만 접근하여 자신이 필요한 데이터를 읽어옴. -> std::tuple 등으로 복잡하게 데이터 넘겨줄 필요 없음
 - `ros2 timer` 사용:
   - 각 `error monitor` 의 주기는 `rclcpp::TimerBase` 로 각각 독립적으로 관리된다. (timer 는 타이머일 뿐 스레드 같은 개념이 아니다.)

## Upgrade history Note.

