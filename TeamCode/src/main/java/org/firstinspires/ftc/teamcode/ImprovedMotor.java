class ImprovedMotor extends com.qualcomm.hardware.DcMotor {
    ImprovedMotor() {
        this.setPower(0)
    }
    void initialize() {
        m = hwMap.get(DcMotor.class, "m"+motors.indexOf(m));
        if (motors.indexOf(m) % 2 == 0) {
            m.setDirection(DCMotor.Direction.FORWARD);
        } else {
            m.setDirection(DCMotor.Direction.REVERSE);
        }
        m.setPower(0);
        m.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
    }
}