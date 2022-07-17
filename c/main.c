#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "pico/stdlib.h"


/**
 * @brief The rust main function
 */
extern void rust_entry();


int main() {
    stdio_init_all();
    rust_entry();
    return 0;
}
