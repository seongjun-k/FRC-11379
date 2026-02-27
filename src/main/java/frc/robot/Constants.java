package frc.robot;

public final class Constants {

    public static final class DriveConstants {
        // CAN IDs (Phoenix Tuner X와 일치)
        public static final int LEFT_FRONT_ID  = 1;
        public static final int LEFT_REAR_ID   = 2;
        public static final int RIGHT_FRONT_ID = 3;
        public static final int RIGHT_REAR_ID  = 4;

        // 텔레옵 속도 스케일
        public static final double DRIVE_SPEED_SCALE    = 0.7;
        public static final double DRIVE_ROTATION_SCALE = 0.6;

        // 전류 제한 (A)
        public static final double SUPPLY_CURRENT_LIMIT_AMPS = 40.0;

        // [가정] 물리 파라미터 — 실측 후 수정
        public static final double WHEEL_DIAMETER_METERS = 0.1524;   // 6인치 휠
        public static final double GEAR_RATIO             = 5.95;    // AM14U6 기본 감속비
        public static final double TRACK_WIDTH_METERS     = 0.561;   // ~24인치 (좌우 바퀴 중심 간격)

        // 엔코더 변환 계수: TalonFX 회전수(rot) → 이동 거리(m)
        public static final double ENCODER_POSITION_FACTOR =
            (Math.PI * WHEEL_DIAMETER_METERS) / GEAR_RATIO;

        // [가정] PathPlanner용 최대 속도 — 실측 권장
        public static final double MAX_SPEED_MPS = 5.45;
    }

    public static final class OIConstants {
        public static final int DRIVER_CONTROLLER_PORT = 0;
    }
    
    public static final class ShooterConstants {
    // CAN ID — Phoenix Tuner X에서 Kraken X60 설정과 반드시 일치
    public static final int SHOOTER_MOTOR_5_ID = 5;
    public static final int SHOOTER_MOTOR_6_ID = 6;
    public static final int SHOOTER_MOTOR_7_ID = 7;

    // 전류 제한 (A) — Kraken X60 연속 권장: 60~80A
    public static final double SUPPLY_CURRENT_LIMIT_AMPS = 60.0;
}
}
