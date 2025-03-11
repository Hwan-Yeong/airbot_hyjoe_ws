#include "error_monitor/battery_monitor/battery_monitor.hpp"

BatteryMonitor::BatteryMonitor()
{
}

BatteryMonitor::~BatteryMonitor()
{
}

bool BatteryMonitor::batteryMonitor(robot_custom_msgs::msg::BatteryStatus batterystatus)
{
    // 배터리 잔량 표시  // 15% 이하일 경우

    double battery_remaining_amount;
//    int count=0;  // 30초 유지관련 카운트 값
//    int maintenance = 30000;  // 단위 mms
    // 베터리 잔량 관련되서 15%이하이면  복귀 불가능
    battery_remaining_amount = batterystatus.battery_percent;

    if(battery_remaining_amount <= 15 || battery_remaining_amount >= 10)
    {
        return true;    // 베터리가 10프로 이상 15프로 이하일 경우
    }
    else
    {
        return false;      // 베터리가 10프로 이상 15프로 이상일 경우
    }

    // 배터리 잔량 표시  // 10% 이하일 경우

}
