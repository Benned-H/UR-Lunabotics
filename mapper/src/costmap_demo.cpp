#include <iostream>
#include "mapper/costmap.h"

int main() {
    CostMap map();

    std::cout << "Here is the default costmap:" << map << std::endl;

    return 0;
}
