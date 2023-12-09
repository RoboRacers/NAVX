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
    @Override
    public void stop() {
        navx_device.close();
    }
    @Override
    public void init_loop() {
        telemetry.addData("navX Op Init Loop", runtime.toString());
    }
    @Override
    public void loop() {

      boolean connected = navx_device.isConnected();
      telemetry.addData("1 navX-Device", connected ?
              "Connected" : "Disconnected" );
      String gyrocal, motion;
      DecimalFormat df = new DecimalFormat("#.##");

      if (connected) {
          gyrocal = (navx_device.isCalibrating() ?
                  "CALIBRATING" : "Calibration Complete");
          motion = (navx_device.isMoving() ? "Moving" : "Not Moving");
          if (navx_device.isRotating()) {
              motion += ", Rotating";
          }
      } 
      else {
          gyrocal =
            motion = "-------";
      }
      telemetry.addData("2 GyroAccel", gyrocal );
      telemetry.addData("3 Motion", motion);
      telemetry.addData("4 Collision", getCollisionString());
      telemetry.addData("5 Timing", Long.toString(sensor_timestamp_delta) + ", " +
                                    Long.toString(system_timestamp_delta) );
      telemetry.addData("6 Events", Double.toString(navx_device.getUpdateCount()));
  }
  private String getCollisionString() {
      return (this.collision_state ? COLLISION : NO_COLLISION);
  }

  private void setCollisionState( boolean newValue ) {
      this.collision_state = newValue;
  }
  @Override
  public void timestampedDataReceived(long curr_system_timestamp, long curr_sensor_timestamp, Object o) {
        boolean collisionDetected = false;
        sensor_timestamp_delta = curr_sensor_timestamp - last_sensor_timestamp;
        system_timestamp_delta = curr_system_timestamp - last_system_timestamp;
        double curr_world_linear_accel_x = navx_device.getWorldLinearAccelX();
        double currentJerkX = curr_world_linear_accel_x - last_world_linear_accel_x;
        last_world_linear_accel_x = curr_world_linear_accel_x;
        double curr_world_linear_accel_y = navx_device.getWorldLinearAccelY();
        double currentJerkY = curr_world_linear_accel_y - last_world_linear_accel_y;
        last_world_linear_accel_y = curr_world_linear_accel_y;
        if ( ( Math.abs(currentJerkX) > COLLISION_THRESHOLD_DELTA_G ) ||
                ( Math.abs(currentJerkY) > COLLISION_THRESHOLD_DELTA_G) ) {
            collisionDetected = true;
        }
        setCollisionState( collisionDetected );
    }
    @Override
    public void untimestampedDataReceived(long l, Object o) {

    }

    @Override
    public void yawReset() {
    }
}
