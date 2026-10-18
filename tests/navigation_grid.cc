#include "core/maps/navigation_grid.h"
#include "core/maps/grid_budget.h"
#include <cassert>
#include <filesystem>
#include <iostream>
#include <limits>

int main(int argc, char** argv) {
    assert(argc == 2);
    using namespace lightning::navigation;
    for (const auto resolution : {0.05, 0.2}) {
        const auto root = std::filesystem::path(argv[1]) / (resolution < .1 ? "fine" : "coarse");
        std::filesystem::create_directories(root);
        GridSpec grid;
        grid.width=3; grid.height=2; grid.resolution=resolution;
        grid.origin={-1.25, 2.5, .3};
        SaveGrid(root.string(), grid, {0,100,-1,100,0,-1});
    }
    GridSpec invalid;
    bool caught=false;
    try { SaveGrid(argv[1], invalid, {}); } catch (const std::invalid_argument&) { caught=true; }
    assert(caught);
    invalid.width=1; invalid.height=1; invalid.resolution=std::numeric_limits<double>::quiet_NaN();
    assert(!invalid.Valid(1));
    invalid.resolution=.05; invalid.origin[0]=std::numeric_limits<double>::infinity();
    assert(!invalid.Valid(1));
    GridSpec mismatch; mismatch.width=2; mismatch.height=2; mismatch.resolution=.05;
    assert(!mismatch.Valid(3));
    CheckGridBudget(10,20,16,51200);
    for (const auto test : {0, 1, 2}) {
        caught=false;
        try {
            if (test==0) CheckGridBudget(10,20,16,51199);
            if (test==1) CheckGridBudget(1e30,20,16,0);
            if (test==2) CheckGridBudget(100000,100000,16,0);
        } catch (const std::length_error&) { caught=true; }
        assert(caught);
    }
    std::cout << "navigation geometry, scale, occupancy and allocation-budget tests passed\n";
}
