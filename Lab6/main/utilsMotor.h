struct motorSpeeds {
    float speed1a, speed1b, speed2a, speed2b;
};

struct motorSpeeds setMotorSpeeds(float a, float b, float c, float d) {
    struct motorSpeeds m;

    m.speed1a = a;
    m.speed1b = b;
    m.speed2a = c;
    m.speed2b = d;

    return m;
}
