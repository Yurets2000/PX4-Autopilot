#include <px4_platform_common/log.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <px4_platform_common/tasks.h>
#include <drivers/drv_hrt.h>
#include <uORB/uORB.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/vehicle_status.h>
#include <mathlib/mathlib.h>

extern "C" __EXPORT int fly_100m_main(int argc, char *argv[]);

class Fly100m : public px4::ScheduledWorkItem
{
public:
    Fly100m();
    virtual ~Fly100m();

    void start();
    void stop();
    void run() override;

private:
    int _vehicle_status_sub = -1;
    orb_advert_t _vehicle_command_pub = nullptr;
    orb_advert_t _local_pos_sp_pub = nullptr;

    void publish_setpoint(float x, float y, float z, float yaw);
    void arm_and_takeoff();
    void land();
};

Fly100m::Fly100m() : ScheduledWorkItem(px4::wq_configurations::nav_and_controllers)
{
}

Fly100m::~Fly100m()
{
    stop();
}

void Fly100m::start()
{
    PX4_INFO("Fly100m starting");
    _vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status));
    schedule_on_interval(100_ms);
}

void Fly100m::stop()
{
    PX4_INFO("Fly100m stopping");
    orb_unsubscribe(_vehicle_status_sub);
    schedule_clear();
}

void Fly100m::run()
{
    // Check vehicle status
    vehicle_status_s vehicle_status;
    if (orb_copy(ORB_ID(vehicle_status), _vehicle_status_sub, &vehicle_status) == PX4_OK)
    {
        if (vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED)
        {
            publish_setpoint(100.0f, 0.0f, -10.0f, 0.0f); // Fly 100m forward, 10m height
        }
    }
}

void Fly100m::publish_setpoint(float x, float y, float z, float yaw)
{
    vehicle_local_position_setpoint_s sp = {};
    sp.x = x;
    sp.y = y;
    sp.z = z;
    sp.yaw = yaw;
    sp.timestamp = hrt_absolute_time();

    if (_local_pos_sp_pub == nullptr)
    {
        _local_pos_sp_pub = orb_advertise(ORB_ID(vehicle_local_position_setpoint), &sp);
    }
    else
    {
        orb_publish(ORB_ID(vehicle_local_position_setpoint), _local_pos_sp_pub, &sp);
    }
}

void Fly100m::arm_and_takeoff()
{
    // Send command to arm and take off
    vehicle_command_s cmd = {};
    cmd.command = vehicle_command_s::VEHICLE_CMD_NAV_TAKEOFF;
    cmd.param7 = 10.0f; // Altitude 10m
    cmd.timestamp = hrt_absolute_time();

    if (_vehicle_command_pub == nullptr)
    {
        _vehicle_command_pub = orb_advertise(ORB_ID(vehicle_command), &cmd);
    }
    else
    {
        orb_publish(ORB_ID(vehicle_command), _vehicle_command_pub, &cmd);
    }
}

void Fly100m::land()
{
    // Send command to land
    vehicle_command_s cmd = {};
    cmd.command = vehicle_command_s::VEHICLE_CMD_NAV_LAND;
    cmd.timestamp = hrt_absolute_time();

    if (_vehicle_command_pub == nullptr)
    {
        _vehicle_command_pub = orb_advertise(ORB_ID(vehicle_command), &cmd);
    }
    else
    {
        orb_publish(ORB_ID(vehicle_command), _vehicle_command_pub, &cmd);
    }
}

int fly_100m_main(int argc, char *argv[])
{
    Fly100m fly100m;
    fly100m.start();

    while (!should_exit())
    {
        usleep(1);
    }

    fly100m.stop();
    return 0;
}
