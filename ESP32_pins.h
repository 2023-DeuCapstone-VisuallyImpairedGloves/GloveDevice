#define I2C_SDA 14
#define I2C_SCL 15
#define MOTOR_RIGHT 13
#define MOTOR_LEFT 12
#define NAV_BUTTON 2

#define FRONT_OBSTACLE 0 //장애물
#define MOVE_LEFT 1 // 왼쪽이동
#define MOVE_RIGHT 2 // 오른쪽이동
#define START 3 // 시작
#define BATTERY_EMPTY 4 // 배터리 부족
#define STAIR_UPHILL 5 //오르막
#define STAIR_DOWNHILL 6 // 내리막
#define MODE_CHANGE 7
#define MODE_START 8
#define MODE_END 9

enum { None, SingleClick, DoubleClick, YesSingle};
