package Drone;

public class PID {

    public boolean first_run ;
    public double P, I, D, max_i, integral, last_error;

    public PID() {}

    public PID(double p, double i, double d, double max_i) {
        P = p;
        I = i;
        D = d;
        this.max_i = max_i;
        this.integral = 0;
        first_run= true;
    }

    public double update(double error, double dt){
        if(first_run){
            last_error=error;
            first_run=false;
        }
        integral+= I * error * dt;
        double diff = (error -last_error)/dt;
        double const_integral = constrain(integral, max_i,-max_i);
        //double control_out = P * error + D * diff + const_integral;
        double control_out = P * error + D * diff + integral;

        last_error =error;


//        System.out.println("p: "+ P+",i: "+I+",d: "+D+",max_i: "+max_i +" integral: "+integral +" last_error: "+last_error);
        return control_out;
    }

    private double constrain(double integral, double max_i, double v) {

        if(integral>max_i){
            return max_i;
        } else if (integral<v) {
            return v;
        }
        return integral;
    }


}
