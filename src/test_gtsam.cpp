#include <iostream>
#include <gtsam/config.h>

int main()
{
    std::cout << "GTSAM Version: "
              << GTSAM_VERSION_MAJOR << "."
              << GTSAM_VERSION_MINOR << "."
              << GTSAM_VERSION_PATCH << std::endl;
    return 0;
}