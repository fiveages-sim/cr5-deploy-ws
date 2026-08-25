/*
 * Minimal Wuji SDK scanner for quick_start.sh.
 * Prints one device per line: SN\taddress\tmodel
 *
 * Build (quick_start caches the binary under .cache/):
 *   ARCH=$(uname -m); case $ARCH in aarch64|arm64) ARCH=aarch64;; *) ARCH=x86_64;; esac
 *   SDK=src/wujihand2-ros2-control/external/wuji-sdk-c
 *   gcc -o wuji_scan_devices wuji_scan_devices.c \
 *     -I"$SDK/include" -L"$SDK/lib/$ARCH" \
 *     -lwuji_sdk_c -Wl,-rpath,"$SDK/lib/$ARCH"
 */
#include <stdio.h>
#include <stdlib.h>

#include "wuji_sdk.h"

int main(void)
{
  if (wuji_init(NULL) != WUJI_STATUS_OK) {
    fprintf(stderr, "wuji_init failed: %s\n", wuji_last_error());
    return 1;
  }

  WujiDiscovered * found = NULL;
  size_t n = 0;
  const WujiStatus st = wuji_scan(&found, &n);
  if (st != WUJI_STATUS_OK) {
    fprintf(stderr, "wuji_scan failed: %s\n", wuji_last_error());
    wuji_shutdown();
    return 1;
  }

  for (size_t i = 0; i < n; ++i) {
    /* Only list Hand2; skip gloves / other devices for this workspace. */
    if (found[i].device_id != WUJI_DEVICE_TYPE_WUJI_HAND_2) {
      continue;
    }
    printf("%s\t%s\t%s\n", found[i].serial_number, found[i].address, found[i].model);
  }

  wuji_discovered_free(found, n);
  wuji_shutdown();
  return 0;
}
