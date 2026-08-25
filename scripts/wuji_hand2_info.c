/*
 * Pre-launch Hand2 identity check for quick_start.sh.
 * Connects (no motor enable), prints handedness / IP / online joints /
 * effort limits / optional diagnostics — mirrors official 2_hand_info.c.
 *
 * Usage:
 *   wuji_hand2_info --address IP:port [--expect left|right]
 *   wuji_hand2_info --sn SERIAL [--expect left|right]
 *
 * Exit: 0 ok (and expect match if given), 2 handedness mismatch, 1 error.
 * Human-readable output goes to stderr.
 *
 * Build (quick_start caches under .cache/):
 *   ARCH=x86_64  # or aarch64
 *   SDK=src/wujihand2-ros2-control/external/wuji-sdk-c
 *   gcc -O2 -o wuji_hand2_info wuji_hand2_info.c \
 *     -I"$SDK/include" -L"$SDK/lib/$ARCH" \
 *     -lwuji_sdk_c -Wl,-rpath,"$SDK/lib/$ARCH"
 */
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <stdatomic.h>

#include "wuji_sdk.h"

static int popcount32(uint32_t v)
{
  int c = 0;
  for (; v; v &= v - 1) {
    c++;
  }
  return c;
}

typedef struct {
  _Atomic int got;
  WujiJointDiagnosticsEntry entries[WUJI_HAND_2_JOINT_COUNT];
  uint8_t count;
} diag_snapshot_t;

static void on_diag(WujiFrameKind kind, const WujiJointDiagnosticsFrame * f, void * ud)
{
  diag_snapshot_t * s = (diag_snapshot_t *)ud;
  if (atomic_load_explicit(&s->got, memory_order_relaxed) || kind != WUJI_FRAME_KIND_OK || !f) {
    return;
  }
  uint8_t n = f->joints_len < WUJI_HAND_2_JOINT_COUNT ? (uint8_t)f->joints_len
                                                       : (uint8_t)WUJI_HAND_2_JOINT_COUNT;
  for (uint8_t i = 0; i < n; i++) {
    s->entries[i] = f->joints[i];
  }
  s->count = n;
  atomic_store_explicit(&s->got, 1, memory_order_release);
}

static void usage(const char * argv0)
{
  fprintf(stderr,
          "Usage:\n"
          "  %s --address IP:port [--expect left|right]\n"
          "  %s --sn SERIAL [--expect left|right]\n",
          argv0, argv0);
}

int main(int argc, char ** argv)
{
  const char * address = NULL;
  const char * sn = NULL;
  const char * expect = NULL; /* "left" | "right" | NULL */

  for (int i = 1; i < argc; i++) {
    if (strcmp(argv[i], "--address") == 0 && i + 1 < argc) {
      address = argv[++i];
    } else if (strcmp(argv[i], "--sn") == 0 && i + 1 < argc) {
      sn = argv[++i];
    } else if (strcmp(argv[i], "--expect") == 0 && i + 1 < argc) {
      expect = argv[++i];
    } else if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
      usage(argv[0]);
      return 0;
    } else {
      fprintf(stderr, "Unknown arg: %s\n", argv[i]);
      usage(argv[0]);
      return 1;
    }
  }

  if ((address == NULL && sn == NULL) || (address != NULL && sn != NULL)) {
    fprintf(stderr, "Provide exactly one of --address or --sn\n");
    usage(argv[0]);
    return 1;
  }
  if (expect != NULL && strcmp(expect, "left") != 0 && strcmp(expect, "right") != 0) {
    fprintf(stderr, "--expect must be left or right\n");
    return 1;
  }

  WujiInitOptions opts = {.log_level = 3};
  if (wuji_init(&opts) != WUJI_STATUS_OK) {
    fprintf(stderr, "wuji_init failed: %s\n", wuji_last_error());
    return 1;
  }

  WujiConnectTarget tgt;
  if (address != NULL) {
    tgt.kind = WUJI_CONNECT_TARGET_KIND_ADDR;
    tgt.value = address;
    fprintf(stderr, "Connecting to address %s ...\n", address);
  } else {
    tgt.kind = WUJI_CONNECT_TARGET_KIND_SN;
    tgt.value = sn;
    fprintf(stderr, "Connecting to SN %s ...\n", sn);
  }

  struct WujiDevice * dev = NULL;
  WujiStatus st = wuji_connect(&tgt, "hand2", NULL, &dev);
  if (st != WUJI_STATUS_OK) {
    fprintf(stderr, "wuji_connect failed: %s\n", wuji_last_error());
    wuji_shutdown();
    return 1;
  }

  int exit_code = 0;
  enum WujiHandedness hand = WUJI_HANDEDNESS_LEFT;
  const char * hand_str = "?";

  if (wuji_hand_2_get_handedness(dev, &hand) == WUJI_STATUS_OK) {
    hand_str = (hand == WUJI_HANDEDNESS_LEFT) ? "left" : "right";
    fprintf(stderr, "handedness: %s\n", hand_str);
  } else {
    fprintf(stderr, "handedness: (read failed: %s)\n", wuji_last_error());
  }

  size_t needed = 0;
  if (wuji_hand_2_get_ip(dev, NULL, 0, &needed) == WUJI_STATUS_ERR_BUFFER_TOO_SMALL) {
    char * ip = (char *)malloc(needed);
    if (ip && wuji_hand_2_get_ip(dev, ip, needed, NULL) == WUJI_STATUS_OK) {
      fprintf(stderr, "ip: %s\n", ip);
    }
    free(ip);
  }

  uint8_t count = 0;
  if (wuji_hand_2_online_joints_count(dev, &count) == WUJI_STATUS_OK) {
    fprintf(stderr, "online joints (count): %u\n", count);
  }

  float limits[WUJI_HAND_2_JOINT_COUNT];
  uint32_t online = 0;
  if (wuji_hand_2_get_all_effort_limit(dev, limits, &online) != WUJI_STATUS_OK) {
    fprintf(stderr, "get_all_effort_limit: %s\n", wuji_last_error());
    exit_code = 1;
    goto cleanup;
  }

  fprintf(stderr, "\nonline bitmap = 0x%08X  (%d joints online)\n", online, popcount32(online));
  fprintf(stderr, "  %-3s %-9s %-8s %12s\n", "i", "label", "state", "effort_lim");
  for (int i = 0; i < WUJI_HAND_2_JOINT_COUNT; i++) {
    char label[16];
    if (wuji_hand_2_joint_label((uint8_t)i, label, sizeof(label), NULL) != WUJI_STATUS_OK) {
      snprintf(label, sizeof(label), "joint_%d", i);
    }
    if (!WUJI_JOINT_ONLINE(online, i)) {
      fprintf(stderr, "  %-3d %-9s %-8s\n", i, label, "offline");
      continue;
    }
    fprintf(stderr, "  %-3d %-9s %-8s %10.3fA\n", i, label, "online", (double)limits[i]);
  }

  diag_snapshot_t snap = {0};
  struct WujiSub * sub = NULL;
  if (wuji_hand_2_subscribe_joint_diagnostics(dev, on_diag, &snap, &sub) == WUJI_STATUS_OK) {
    for (int i = 0; i < 100 && !atomic_load_explicit(&snap.got, memory_order_acquire); i++) {
      usleep(10000);
    }
    wuji_sub_close(sub);
  }
  if (atomic_load_explicit(&snap.got, memory_order_acquire)) {
    fprintf(stderr, "\njoint_diagnostics snapshot (%u joints):\n", snap.count);
    fprintf(stderr, "  %-4s %-12s %8s %7s %6s %8s\n", "nid", "status_word", "current", "vbus",
            "temp", "err_code");
    for (uint8_t i = 0; i < snap.count; i++) {
      const WujiJointDiagnosticsEntry * e = &snap.entries[i];
      fprintf(stderr, "  %-4u 0x%08X %7.2fA %6.1fV %5.1fC   0x%04X\n", e->nid, e->status_word,
              (double)e->current, (double)e->vbus_v_fb, (double)e->mcu_temp_c_fb,
              e->error_code_current);
    }
  } else {
    fprintf(stderr, "\n(no joint_diagnostics frame received)\n");
  }

  if (expect != NULL) {
    const int want_left = (strcmp(expect, "left") == 0);
    const int is_left = (hand == WUJI_HANDEDNESS_LEFT);
    if (want_left != is_left) {
      fprintf(stderr,
              "\n[MISMATCH] expected %s hand, device reports %s\n",
              expect, hand_str);
      exit_code = 2;
    } else {
      fprintf(stderr, "\n[OK] handedness matches expect=%s\n", expect);
    }
  }

cleanup:
  wuji_dev_disconnect(dev);
  wuji_dev_release(dev);
  wuji_shutdown();
  return exit_code;
}
