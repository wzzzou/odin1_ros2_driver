#include "odin_local/parent_death_guard.hpp"

int main(int argc, char* argv[]) {
    return odin_local::run_with_parent_death_signal(argc, argv);
}
