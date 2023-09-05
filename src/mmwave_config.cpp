#include "mmwave/mmwave_cfg.h"

#include <stdio.h>

int main(int argc, char **argv) {
    if (argc < 3) {
        printf("Usage: %s <device> <config>\n", argv[0]);
        return 1;
    }

    if (mmwave_cfg_write_file(argv[1], argv[2])) {
        printf("Failed to configure %s\n", argv[1]);
        return 1;
    }

    printf("Successfully configured %s\n", argv[1]);

    return 0;
}