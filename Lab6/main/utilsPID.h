
struct PID {
    float setpoint, k_p, k_i, k_d, prev_error;
};

struct PID setPID(float a, float b, float c, float d, float e) {
    struct PID p;

    p.setpoint = a;
    p.k_p = a;
    p.k_i = b;
    p.k_d = d;
    p.prev_error = e;

    return p;
}

// UPDATE WITH MORE INPUT INFO FOR PID CONTROLLER
struct PIDInfo {
    struct motorSpeeds;
    float sensorVal1, sensorVal2;
};

struct PIDInfo setPIDInfo(struct motorSpeeds *m, float sv1, float sv2) {
    struct PIDInfo x;

    //x->motorSpeeds = m;
    x.sensorVal1 = sv1;
    x.sensorVal2 = sv2;

    return x;
}

float PIDController(struct PID *p, struct PIDInfo *i, int PID_type) {

    // Calculate error using i
    float error = i->sensorVal1;

    switch (PID_type) {
        
        case 0: // P
            return p->k_p * error;
            break;

        case 1: // PI
            break;

        case 2: // PD
            break;

        case 3: // PID
            break;

        default: // P
            return p->k_p * error;
            break;
    }
}
