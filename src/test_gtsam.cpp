#include <gtsam/geometry/Point3.h>
#include <iostream>

int main() {
    gtsam::Point3 p(1, 2, 3);
    std::cout << p << std::endl;
    return 0;
}
