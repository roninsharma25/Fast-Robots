float error;

struct PID {
    float setpoint, k_p, k_i, k_d;
}

struct PID setPID(float a, float b, float c, float d) {
    struct PID p;

    p.setpoint = a;
    p.k_p = a;
    p.k_i = b;
    p.k_d = d;

    return p;
}

// UPDATE WITH ACTUAL INPUT INFO FOR PID CONTROLLER
struct INFO {
    float x;
}

struct INFO setINFO(float x) {
    struct INFO x;

    x.x = x;

    return x
}

float PIDController(PID p, INFO i, int PID_type) {

    // Calculate error using i
    error = i.x;

    switch (PID_type) {
        
        case 0: // P
            return p.k_p * error;
            break;

        case 1: // PI
            break;

        case 2: // PD
            break;

        case 3: // PID
            break;

        default: // P
            return p.k_p * error;
            break;
    }
}