package Hardware;

import com.acmerobotics.dashboard.config.Config;

@Config
public class VariableStorage {

    /** Varibile pentru facut automatizare la teleop */

    public static double PI = 3.1415;
    public double GEAR_MOTOR_40_TICKS = 1120;
    public double GEAR_MOTOR_ORBITAL20_TICKS = 537.6;
    public static double GEAR_MOTOR_GOBILDA5202_TICKS = 537.7;
    public static double WHEEL_DIAMETER_CM = 4;
    //double TICKS_PER_CM_Z = GEAR_MOTOR_40_TICKS / (WHEEL_DIAMETER_CM * PI);
    public static double TICKS_PER_CM_Z = GEAR_MOTOR_GOBILDA5202_TICKS / (WHEEL_DIAMETER_CM * PI);

   /** Exemplu de Config in variable storage
    * public static class ClesteConfig {
        public double cOpen1;
        public double cOpen2;
        public double cClosed1;
        public double cClosed2;
        public ClesteConfig(double cOpen1, double cOpen2, double cClosed1, double cClosed2){
            this.cOpen1 = cOpen1;
            this.cOpen2 = cOpen2;
            this.cClosed1 = cClosed1;
            this.cClosed2 = cClosed2;
        }
    } */

    public VariableStorage(){}
}
