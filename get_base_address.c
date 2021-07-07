#include <bcm_host.h>

#include <stdio.h>
#include <bcm_host.h>

int main(void) {
    printf("bcm_host_get_peripheral_address -> 0x%08x\n", bcm_host_get_peripheral_address());
    printf("bcm_host_get_peripheral_size -> 0x%08x\n", bcm_host_get_peripheral_size());
    printf("bcm_host_get_sdram_address -> 0x%08x\n", bcm_host_get_sdram_address());

    return 0;
}
