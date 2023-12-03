package org.firstinspires.ftc.teamcode.opmode;

//import statements
import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.kauailabs.navx.ftc.AHRS;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

public class NavXCollisionDetection extends OpMode {

    // variable initialization
    private final double COLLISION_THRESHOLD_DELTA_G = 0.5;
    double last_world_linear_accel_x;
    double last_world_linear_accel_y;
    private ElapsedTime runtime = new ElapsedTime();
    private AHRS navx_device;
    private boolean collision_state;
    private final String COLLISION = "COLLISION";
    private final String NONCOLLISION = "NON-COLLISION";
    private long last_system_timestamp = 0;
    private long last_sensor_timestamp = 0;
    private long sensor_timestamp_delta = 0;
    private long system_timestamp_delta = 0;

    //Functions
    @Override
    public void init() {
        navx_device = AHRS.getInstance(hardwareMap.get(NavxMicroNavigationSensor.class, "navx"), AHRS.DeviceDataType.kProcessedData);
        last_world_linear_accel_x = 0.0;
        last_world_linear_accel_y = 0.0;
        setCollisionState(false);
    }
    @Override
    public void start() {
        navx_device.registerCallback(this);
    }        
}
