#include <bulletWorld.h>
#include <iostream>

int main(int argc, char* argv[]) {
  BulletWorld world;

  /// Make the terrain
  int row_count = 10;
  int col_count = 10;
  double grad = 1;
  double min_ht = -2.2674;
  double max_ht = 2.2674;
  double X[] = {
    -4.7935,-3.7282,-2.6630,-1.5978,-0.5326, 0.5326, 1.5978, 2.6630, 3.7282, 4.7935,
    -4.7935,-3.7282,-2.6630,-1.5978,-0.5326, 0.5326, 1.5978, 2.6630, 3.7282, 4.7935,
    -4.7935,-3.7282,-2.6630,-1.5978,-0.5326, 0.5326, 1.5978, 2.6630, 3.7282, 4.7935,
    -4.7935,-3.7282,-2.6630,-1.5978,-0.5326, 0.5326, 1.5978, 2.6630, 3.7282, 4.7935,
    -4.7935,-3.7282,-2.6630,-1.5978,-0.5326, 0.5326, 1.5978, 2.6630, 3.7282, 4.7935,
    -4.7935,-3.7282,-2.6630,-1.5978,-0.5326, 0.5326, 1.5978, 2.6630, 3.7282, 4.7935,
    -4.7935,-3.7282,-2.6630,-1.5978,-0.5326, 0.5326, 1.5978, 2.6630, 3.7282, 4.7935,
    -4.7935,-3.7282,-2.6630,-1.5978,-0.5326, 0.5326, 1.5978, 2.6630, 3.7282, 4.7935,
    -4.7935,-3.7282,-2.6630,-1.5978,-0.5326, 0.5326, 1.5978, 2.6630, 3.7282, 4.7935,
    -4.7935,-3.7282,-2.6630,-1.5978,-0.5326, 0.5326, 1.5978, 2.6630, 3.7282, 4.7935};
  double Y[] =
      { -4.9575,-4.9575,-4.9575,-4.9575,-4.9575,-4.9575,-4.9575,-4.9575,-4.9575,-4.9575,
        -3.8558,-3.8558,-3.8558,-3.8558,-3.8558,-3.8558,-3.8558,-3.8558,-3.8558,-3.8558,
        -2.7542,-2.7542,-2.7542,-2.7542,-2.7542,-2.7542,-2.7542,-2.7542,-2.7542,-2.7542,
        -1.6525,-1.6525,-1.6525,-1.6525,-1.6525,-1.6525,-1.6525,-1.6525,-1.6525,-1.6525,
        -0.5508,-0.5508,-0.5508,-0.5508,-0.5508,-0.5508,-0.5508,-0.5508,-0.5508,-0.5508,
        0.5508, 0.5508, 0.5508, 0.5508, 0.5508, 0.5508, 0.5508, 0.5508, 0.5508, 0.5508,
        1.6525, 1.6525, 1.6525, 1.6525, 1.6525, 1.6525, 1.6525, 1.6525, 1.6525, 1.6525,
        2.7542, 2.7542, 2.7542, 2.7542, 2.7542, 2.7542, 2.7542, 2.7542, 2.7542, 2.7542,
        3.8558, 3.8558, 3.8558, 3.8558, 3.8558, 3.8558, 3.8558, 3.8558, 3.8558, 3.8558,
        4.9575, 4.9575, 4.9575, 4.9575, 4.9575, 4.9575, 4.9575, 4.9575, 4.9575, 4.9575};
  double Z[] =
      { -0.3165,-0.0505, 0.0426,-0.1839,-0.3582,-0.5377,-0.7223,-0.9120,-0.8758, 1.4057,
        -0.7318,-1.1311, 0.5175, 0.0630,-0.0150,-0.6302,-1.0422,-0.7355,-1.8573,-1.4281,
        -0.5714,-1.7493,-1.0203, 0.1016,-0.7434,-0.4039,-1.5088,-0.8884,-1.8881,-1.2869,
        -0.6168,-0.1142,-0.7253,-0.8923,-1.0681,-0.8642,-0.9886,-0.1629,-0.3332,-1.1358,
        -0.6667,-0.6750,-0.6718,-0.9922,-1.8549,-0.3782,-0.3454,-1.7289,-1.5314,-0.9746,
        -0.7210,-0.6499,-1.6811,-1.7250,-1.5584, 0.0163, 0.3797,-0.6971,-0.5469,-0.8036,
        -0.7798,-0.1799,-0.3584,-1.4845,-0.9507,-0.7062,-0.5254,-0.3992, 0.2913,-0.6225,
        -0.8431,-1.2576,-1.1905,-0.7280,-0.3380,-1.2152,-0.7732,-2.1436,-0.5971,-0.5624,
        -0.0506,-1.7827,-1.3489,-1.1618,-1.7606,-1.3008,-0.7487, 0.1320,-1.3031,-1.1524,
        2.2674, 1.3542, 0.7818, 0.2537,-0.2303,-0.6701,-1.0658,-1.4172,-1.7245,-2.2674};
  double normal[] = {0, 0, 1};

  ////////////
  double radius = 3;
  double mass = 5;
  double restitution = 2;
  double position[] = {10, 10, 8};
  double rotation[] = {1, 0, 0,
                       0, 1, 0,
                       0, 0, 1};

  ////////////
  double id = 1;
  double pivot[] = {4, 0, 0};
  double axis[] = {0, 1, 0};
  double limits[] = {-1, 1, .9, .3, 1};

  int i = world.AddTerrain(row_count, col_count, grad, min_ht, max_ht,
                           X, Y, Z, normal);

  i = world.AddSphere(radius, mass, restitution, position, rotation);

  i = world.Hinge_one_pivot(id, pivot, axis, limits);

  while (1) {
    world.StepSimulation();
    std::vector<double> pose = world.GetShapeTransform(0);
    std::vector<double> pose2 = world.GetShapeTransform(1);
    double position[3];
    //Position
    memcpy( position, &pose2[0], sizeof( double ) * 3 );
  }

  return 0;
}
