#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#define NDET 16
#define NBEACON 3
#define WINDOW_S 0.1
#define LISTEN_WINDOWS 30
#define NWHEEL 3

enum {
  BRANCH_LISTEN,
  BRANCH_STAY_STILL,
  BRANCH_ORBIT_ONE,
  BRANCH_MAP_THREE,
  BRANCH_MAP_TWO,
  BRANCH_FILTER_START,
  BRANCH_FILTER_CORRECT,
  BRANCH_FILTER_PREDICT,
};

typedef struct {
  double beacon_xy[NBEACON][2];
  double hoop_radius[NBEACON];
  double payload_radius;
  double detector_angle[NDET];
  double wheel_angle[NWHEEL];
  double top_speed;       // m/s
  double dodge_clearance; // m
  double taper_start;     // m
  double memory_time;     // s
  double hidden_margin;   // rad
  double resect_limit;    // m per rad
  double motion_trust;
  double fix_trust;
  double lost_limit; // m
} ctrl_cfg;

typedef struct {
  double magnitude[NDET][NBEACON];
  double quiet_magnitude[NDET];
} ctrl_in;

typedef struct {
  int windows; // since powering up
  int noise_started;
  double noise[NDET]; // smoothed noise
  int mem_have[NBEACON];
  double mem[NBEACON][2]; // bearing, fake distance
  double mem_age[NBEACON]; // s
  double lam_keep;    // m per fake brightness unit also smoothed
  int dodge_side[NBEACON];
  double last_vx, last_vy; // last command, filter needs this
  int kf_on;
  double kf_x[3];    // x, y, psi
  double kf_P[3][3]; // uncertainty matrix
} ctrl_state;

typedef struct {
  double vx, vy;
  double wheel_speed[NWHEEL];
  int arrived;
  int branch; // which branch ran
  int centroid_known;
  double centroid_x, centroid_y;
  int detected[NBEACON];
  int used[NBEACON];
} ctrl_out;

void ctrl_init(const ctrl_cfg *cfg, ctrl_state *st);
void ctrl_step(const ctrl_cfg *cfg, ctrl_state *st, const ctrl_in *in,
               ctrl_out *out);

#ifdef __cplusplus
}
#endif
