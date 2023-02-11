

#define POINTS_2_INTEGRATE 3   // USAR BUFFER CIRCULAR SI SON MUCHOS PUNTOS

static double Pgyro[POINTS_2_INTEGRATE]; // P[0] -> Pk ; P[1] -> Pk-1 ...
static double Qgyro[POINTS_2_INTEGRATE];
static double Rgyro[POINTS_2_INTEGRATE];

static uint8_t simpsonCounter;

void pushGyroMeasurement();

static double simpson(double *f, double h, int n) {
    return (f[0] + 4*f[1] + f[2])*h/3;
}



void cbGyro(){



}
