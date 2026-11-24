#include <cstdio>

#include "core/config.h"

int main(int argc, char** argv) {
    if (argc != 2) {
        std::fprintf(stderr, "usage: %s <config.json>\n", argv[0]);
        return 2;
    }
    try {
        const dev_board::Config config = dev_board::load_config(argv[1]);
        std::printf("config ok: Zw = %g m (%d cells), r = %g m (%d cells), s = %g m\n",
                    config.zone.width_m, config.zone.width_cells, config.zone.ring_width_m,
                    config.zone.ring_cells, config.grid.cell_size_m);
    } catch (const dev_board::ConfigError& error) {
        std::fprintf(stderr, "config error: %s\n", error.what());
        return 1;
    }
    return 0;
}
