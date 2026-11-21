#include "controller.h"
#include "tables.h"

#include <math.h>
#include <string.h>

#define PI 3.14159265358979323846
#define DEG (PI / 180)
#define DETECTOR_SNR 3.0
#define BEACON_SNR 5.0
#define KF_BEARING_NOISE (4 * DEG)
#define KF_MOVE_NOISE_FRAC 0.07
#define KF_MOVE_NOISE_MIN 0.001
#define KF_TURN_NOISE_PER_M 0.3
#define KF_TURN_NOISE_MIN 0.002
#define STOP_RADIUS 0.01
#define TAPER_FLOOR 0.25
#define ORBIT_SPEED_FRAC (1.0 / 3.0)
#define DODGE_CENTROID_GAP 0.01

static double wrap(double a) { return atan2(sin(a), cos(a)); }
static double sq(double v) { return v * v; }

// todo: use each pair's calibrated spacing from detector_angle instead of an even 22.5 deg.
static double ratio_to_offset(double ratio) {
  double max_theta = PI / NDET;
  if (ratio <= RATIO_TABLE[0]) return 0;
  if (ratio >= RATIO_TABLE[RATIO_TABLE_N - 1]) return max_theta;
  int lo = 0, hi = RATIO_TABLE_N - 1;
  while (hi - lo > 1) {
    int mid = (lo + hi) / 2;
    if (RATIO_TABLE[mid] <= ratio) lo = mid; else hi = mid;
  }
  double f = (ratio - RATIO_TABLE[lo]) / (RATIO_TABLE[hi] - RATIO_TABLE[lo]);
  return max_theta * (lo + f) / (RATIO_TABLE_N - 1);
}

typedef struct {
  int used[NBEACON];
  double bearing[NBEACON];
  double brightness[NBEACON];
  double fake_distance[NBEACON];
} sensed;

static void sense(const ctrl_cfg *cfg, ctrl_state *st, const ctrl_in *in, sensed *s) {
  for (int d = 0; d < NDET; d++)
    st->noise[d] = st->noise_started ? 0.9 * st->noise[d] + 0.1 * in->quiet_magnitude[d]
                                     : in->quiet_magnitude[d];
  st->noise_started = 1;
  for (int b = 0; b < NBEACON; b++) {
    double sum = 0, noise_sum = 0;
    int n_counting = 0, best = -1, counts[NDET] = {0};
    for (int d = 0; d < NDET; d++) {
      double noise = st->noise[d], mag = in->magnitude[d][b];
      if (mag < DETECTOR_SNR * noise) continue;
      counts[d] = 1;
      sum += mag;
      noise_sum += noise;
      n_counting++;
      if (best < 0 || mag > in->magnitude[best][b]) best = d;
    }
    s->used[b] = n_counting > 0 &&
                 sum > BEACON_SNR * (noise_sum / n_counting) * sqrt((double)n_counting);
    s->brightness[b] = sum;
    s->fake_distance[b] = sum > 0 ? 1 / sqrt(sum) : 0;
    s->bearing[b] = 0;
    if (!s->used[b]) continue;
    int up = (best + 1) % NDET, down = (best + NDET - 1) % NDET, neighbour = -1;
    if (counts[up] && (!counts[down] || in->magnitude[up][b] >= in->magnitude[down][b])) neighbour = up;
    else if (counts[down]) neighbour = down;
    double axis = cfg->detector_angle[best];
    if (neighbour < 0) {
      s->bearing[b] = axis;
    } else {
      double theta = ratio_to_offset(in->magnitude[neighbour][b] / in->magnitude[best][b]);
      s->bearing[b] = wrap(axis + (neighbour == up ? theta : -theta));
    }
  }
}

static void place(const ctrl_cfg *cfg, double lam, int i, double bearing, double fake_distance, double out[2]) {
  double d = lam * fake_distance + cfg->hoop_radius[i];
  out[0] = d * cos(bearing);
  out[1] = d * sin(bearing);
}

static void hidden_rule(const ctrl_cfg *cfg, const ctrl_state *st, sensed *s) {
  if (st->lam_keep <= 0) return;
  if (!(s->used[0] && s->used[1] && s->used[2])) return;
  int drop = -1;
  double most = 0;
  for (int a = 0; a < NBEACON; a++)
    for (int b = a + 1; b < NBEACON; b++) {
      double da = st->lam_keep * s->fake_distance[a] + cfg->hoop_radius[a];
      double db = st->lam_keep * s->fake_distance[b] + cfg->hoop_radius[b];
      double ha = asin(fmin(1, cfg->hoop_radius[a] / da)), hb = asin(fmin(1, cfg->hoop_radius[b] / db));
      double over = ha + hb + cfg->hidden_margin - fabs(wrap(s->bearing[a] - s->bearing[b]));
      if (over > most) {
        most = over;
        drop = da > db ? a : b;
      }
    }
  if (drop >= 0) s->used[drop] = 0;
}

static void remember(const ctrl_cfg *cfg, ctrl_state *st, const sensed *s) {
  for (int b = 0; b < NBEACON; b++) {
    if (s->used[b]) {
      st->mem_have[b] = 1;
      st->mem_age[b] = 0;
      st->mem[b][0] = s->bearing[b];
      st->mem[b][1] = s->fake_distance[b];
      continue;
    }
    if (!st->mem_have[b]) continue;
    st->mem_age[b] += WINDOW_S;
    if (st->mem_age[b] > cfg->memory_time || st->lam_keep <= 0) {
      st->mem_have[b] = 0;
      continue;
    }
    double q[2];
    place(cfg, st->lam_keep, b, st->mem[b][0], st->mem[b][1], q);
    q[0] -= st->last_vx * WINDOW_S;
    q[1] -= st->last_vy * WINDOW_S;
    st->mem[b][0] = atan2(q[1], q[0]);
    st->mem[b][1] = fmax(1e-9, (hypot(q[0], q[1]) - cfg->hoop_radius[b]) / st->lam_keep);
  }
}

typedef struct {
  int branch;
  int centroid_known;
  double centroid_x, centroid_y;
  int have_hoops;
  double hoop_x[NBEACON], hoop_y[NBEACON];
  int move;
  double vx, vy;
} aim_out;

static double real_perimeter(const ctrl_cfg *cfg) {
  double p = 0;
  for (int i = 0; i < NBEACON; i++) {
    int j = (i + 1) % NBEACON;
    p += hypot(cfg->beacon_xy[i][0] - cfg->beacon_xy[j][0], cfg->beacon_xy[i][1] - cfg->beacon_xy[j][1]);
  }
  return p;
}

static void listen(aim_out *o) {
  o->branch = BRANCH_LISTEN;
  o->move = 1;
  o->vx = o->vy = 0;
}

static void stay_still(aim_out *o) {
  o->branch = BRANCH_STAY_STILL;
  o->move = 1;
  o->vx = o->vy = 0;
}

static void orbit_one(const ctrl_cfg *cfg, const ctrl_state *st, int hoop, double hoop_bearing, aim_out *o) {
  o->branch = BRANCH_ORBIT_ONE;
  o->move = 1;
  double a = hoop_bearing + (st->dodge_side[hoop] > 0 ? PI / 2 : -PI / 2);
  o->vx = ORBIT_SPEED_FRAC * cfg->top_speed * cos(a);
  o->vy = ORBIT_SPEED_FRAC * cfg->top_speed * sin(a);
}

typedef double (*measure_fn)(const ctrl_cfg *cfg, double lam, const double bearing[NBEACON],
                             const double fake_distance[NBEACON], int a, int b);

static double perimeter_at(const ctrl_cfg *cfg, double lam, const double bearing[NBEACON],
                           const double fake_distance[NBEACON], int a, int b) {
  (void)a;
  (void)b;
  double q[NBEACON][2], per = 0;
  for (int i = 0; i < NBEACON; i++) place(cfg, lam, i, bearing[i], fake_distance[i], q[i]);
  for (int i = 0; i < NBEACON; i++) {
    int j = (i + 1) % NBEACON;
    per += hypot(q[i][0] - q[j][0], q[i][1] - q[j][1]);
  }
  return per;
}

static double spacing_at(const ctrl_cfg *cfg, double lam, const double bearing[NBEACON],
                         const double fake_distance[NBEACON], int a, int b) {
  double qa[2], qb[2];
  place(cfg, lam, a, bearing[a], fake_distance[a], qa);
  place(cfg, lam, b, bearing[b], fake_distance[b], qb);
  return hypot(qa[0] - qb[0], qa[1] - qb[1]);
}

static double solve_scale(const ctrl_cfg *cfg, measure_fn f, double want, const double bearing[NBEACON],
                          const double fake_distance[NBEACON], int a, int b) {
  double lo = 0, hi = 1;
  if (f(cfg, 0, bearing, fake_distance, a, b) >= want) return 0;
  while (f(cfg, hi, bearing, fake_distance, a, b) < want && hi < 1e6) hi *= 2;
  for (int it = 0; it < 60; it++) {
    double mid = (lo + hi) / 2;
    if (f(cfg, mid, bearing, fake_distance, a, b) < want) lo = mid; else hi = mid;
  }
  return (lo + hi) / 2;
}

static void keep_scale(ctrl_state *st, double lam) {
  st->lam_keep = st->lam_keep > 0 ? 0.8 * st->lam_keep + 0.2 * lam : lam;
}

static void map_three(const ctrl_cfg *cfg, ctrl_state *st, const double bearing[NBEACON],
                      const double fake_distance[NBEACON], aim_out *o) {
  o->branch = BRANCH_MAP_THREE;
  double lam = solve_scale(cfg, perimeter_at, real_perimeter(cfg), bearing, fake_distance, 0, 0);
  keep_scale(st, lam);
  o->centroid_known = o->have_hoops = 1;
  o->centroid_x = o->centroid_y = 0;
  for (int i = 0; i < NBEACON; i++) {
    double q[2];
    place(cfg, lam, i, bearing[i], fake_distance[i], q);
    o->hoop_x[i] = q[0];
    o->hoop_y[i] = q[1];
    o->centroid_x += q[0] / NBEACON;
    o->centroid_y += q[1] / NBEACON;
  }
}

static void map_two(const ctrl_cfg *cfg, ctrl_state *st, int a, int b, const double bearing[NBEACON],
                    const double fake_distance[NBEACON], aim_out *o) {
  o->branch = BRANCH_MAP_TWO;
  int c = 3 - a - b;
  double ex = cfg->beacon_xy[b][0] - cfg->beacon_xy[a][0], ey = cfg->beacon_xy[b][1] - cfg->beacon_xy[a][1];
  double wx = cfg->beacon_xy[c][0] - cfg->beacon_xy[a][0], wy = cfg->beacon_xy[c][1] - cfg->beacon_xy[a][1];
  double ee = ex * ex + ey * ey, along = (wx * ex + wy * ey) / ee, left = (ex * wy - ey * wx) / ee;
  double lam = solve_scale(cfg, spacing_at, sqrt(ee), bearing, fake_distance, a, b);
  keep_scale(st, lam);
  double qa[2], qb[2];
  place(cfg, lam, a, bearing[a], fake_distance[a], qa);
  place(cfg, lam, b, bearing[b], fake_distance[b], qb);
  double dx = qb[0] - qa[0], dy = qb[1] - qa[1];
  o->centroid_known = o->have_hoops = 1;
  o->hoop_x[a] = qa[0];
  o->hoop_y[a] = qa[1];
  o->hoop_x[b] = qb[0];
  o->hoop_y[b] = qb[1];
  o->hoop_x[c] = qa[0] + along * dx - left * dy;
  o->hoop_y[c] = qa[1] + along * dy + left * dx;
  o->centroid_x = (o->hoop_x[0] + o->hoop_x[1] + o->hoop_x[2]) / 3;
  o->centroid_y = (o->hoop_y[0] + o->hoop_y[1] + o->hoop_y[2]) / 3;
}

static void map_aim(const ctrl_cfg *cfg, ctrl_state *st, aim_out *o) {
  double bearing[NBEACON] = {0}, fake_distance[NBEACON] = {0};
  int known[NBEACON], n = 0;
  for (int b = 0; b < NBEACON; b++) {
    known[b] = st->mem_have[b];
    if (known[b]) {
      bearing[b] = st->mem[b][0];
      fake_distance[b] = st->mem[b][1];
      n++;
    }
  }
  if (n == 3) {
    map_three(cfg, st, bearing, fake_distance, o);
  } else if (n == 2) {
    int a = known[0] ? 0 : 1, b = known[2] ? 2 : 1;
    map_two(cfg, st, a, b, bearing, fake_distance, o);
  } else if (n == 1) {
    int a = known[0] ? 0 : known[1] ? 1 : 2;
    orbit_one(cfg, st, a, bearing[a], o);
  } else {
    stay_still(o);
  }
}

static double bearing_from(const ctrl_cfg *cfg, const double p[3], int i, double J[3]) {
  double dx = cfg->beacon_xy[i][0] - p[0], dy = cfg->beacon_xy[i][1] - p[1], r2 = dx * dx + dy * dy;
  if (J) {
    J[0] = dy / r2;
    J[1] = -dx / r2;
    J[2] = -1;
  }
  return atan2(dy, dx) - p[2];
}

static int inv3(const double A[3][3], double B[3][3]) {
  double det = A[0][0] * (A[1][1] * A[2][2] - A[1][2] * A[2][1]) - A[0][1] * (A[1][0] * A[2][2] - A[1][2] * A[2][0]) +
               A[0][2] * (A[1][0] * A[2][1] - A[1][1] * A[2][0]);
  if (fabs(det) < 1e-12) return 0;
  B[0][0] = (A[1][1] * A[2][2] - A[1][2] * A[2][1]) / det;
  B[0][1] = (A[0][2] * A[2][1] - A[0][1] * A[2][2]) / det;
  B[0][2] = (A[0][1] * A[1][2] - A[0][2] * A[1][1]) / det;
  B[1][0] = (A[1][2] * A[2][0] - A[1][0] * A[2][2]) / det;
  B[1][1] = (A[0][0] * A[2][2] - A[0][2] * A[2][0]) / det;
  B[1][2] = (A[0][2] * A[1][0] - A[0][0] * A[1][2]) / det;
  B[2][0] = (A[1][0] * A[2][1] - A[1][1] * A[2][0]) / det;
  B[2][1] = (A[0][1] * A[2][0] - A[0][0] * A[2][1]) / det;
  B[2][2] = (A[0][0] * A[1][1] - A[0][1] * A[1][0]) / det;
  return 1;
}

static double worst_sd(double a, double b, double d) {
  double tr = a + d, det = a * d - b * b, disc = sqrt(fmax(0, tr * tr / 4 - det));
  return sqrt(fmax(0, tr / 2 + disc));
}

static int resection_solve(const ctrl_cfg *cfg, const double bearing[NBEACON], const double guess[3], double pose[3],
                           double *m_per_rad) {
  double p[3] = {guess[0], guess[1], guess[2]};
  double J[3][3], Ji[3][3], r[3];
  for (int it = 0; it < 30; it++) {
    double norm = 0;
    for (int i = 0; i < NBEACON; i++) {
      r[i] = wrap(bearing[i] - bearing_from(cfg, p, i, J[i]));
      norm += r[i] * r[i];
    }
    if (!inv3(J, Ji)) return 0;
    double step[3] = {0, 0, 0};
    for (int a = 0; a < 3; a++)
      for (int b = 0; b < 3; b++) step[a] += Ji[a][b] * r[b];
    double len = hypot(step[0], step[1]), k = len > 0.3 ? 0.3 / len : 1;
    for (int a = 0; a < 3; a++) p[a] += k * step[a];
    p[2] = wrap(p[2]);
    if (norm < 1e-14 && len < 1e-7) break;
  }
  double norm = 0;
  for (int i = 0; i < NBEACON; i++) {
    r[i] = wrap(bearing[i] - bearing_from(cfg, p, i, J[i]));
    norm += r[i] * r[i];
  }
  if (norm > 1e-8 || !inv3(J, Ji)) return 0;
  double C[2][2] = {{0, 0}, {0, 0}};
  for (int a = 0; a < 2; a++)
    for (int b = 0; b < 2; b++)
      for (int k = 0; k < 3; k++) C[a][b] += Ji[a][k] * Ji[b][k];
  *m_per_rad = worst_sd(C[0][0], C[0][1], C[1][1]);
  pose[0] = p[0];
  pose[1] = p[1];
  pose[2] = p[2];
  return 1;
}

static void guess_from_map(const ctrl_cfg *cfg, const aim_out *m, double g[3]) {
  double mx = 0, my = 0;
  for (int i = 0; i < NBEACON; i++) {
    mx += m->hoop_x[i] / NBEACON;
    my += m->hoop_y[i] / NBEACON;
  }
  double sdot = 0, scross = 0;
  for (int i = 0; i < NBEACON; i++) {
    double x0 = cfg->beacon_xy[i][0], x1 = cfg->beacon_xy[i][1], y0 = m->hoop_x[i] - mx, y1 = m->hoop_y[i] - my;
    sdot += x0 * y0 + x1 * y1;
    scross += x0 * y1 - x1 * y0;
  }
  double phi = atan2(scross, sdot);
  g[0] = -(cos(phi) * mx + sin(phi) * my);
  g[1] = -(-sin(phi) * mx + cos(phi) * my);
  g[2] = -phi;
}

static void from_pose(const ctrl_cfg *cfg, const double p[3], aim_out *o) {
  double c = cos(-p[2]), s = sin(-p[2]);
  o->centroid_known = o->have_hoops = 1;
  o->centroid_x = c * (0 - p[0]) - s * (0 - p[1]);
  o->centroid_y = s * (0 - p[0]) + c * (0 - p[1]);
  for (int i = 0; i < NBEACON; i++) {
    double dx = cfg->beacon_xy[i][0] - p[0], dy = cfg->beacon_xy[i][1] - p[1];
    o->hoop_x[i] = c * dx - s * dy;
    o->hoop_y[i] = s * dx + c * dy;
  }
}

static int try_fix(const ctrl_cfg *cfg, ctrl_state *st, const sensed *s, double pose[3], double *m_per_rad) {
  if (!(s->used[0] && s->used[1] && s->used[2])) return 0;
  aim_out m;
  memset(&m, 0, sizeof m);
  map_aim(cfg, st, &m);
  if (!m.have_hoops) return 0;
  double g[3];
  guess_from_map(cfg, &m, g);
  if (!resection_solve(cfg, s->bearing, g, pose, m_per_rad)) return 0;
  return *m_per_rad <= cfg->resect_limit;
}

static void filter_start(const ctrl_cfg *cfg, ctrl_state *st, const double pose[3], double m_per_rad, aim_out *o) {
  double b = cfg->fix_trust * KF_BEARING_NOISE, v = sq(m_per_rad * b) + 1e-6;
  memset(st->kf_P, 0, sizeof st->kf_P);
  st->kf_P[0][0] = st->kf_P[1][1] = v;
  st->kf_P[2][2] = sq(b) * 3;
  memcpy(st->kf_x, pose, sizeof st->kf_x);
  st->kf_on = 1;
  from_pose(cfg, st->kf_x, o);
  o->branch = BRANCH_FILTER_START;
}

static void filter_predict(const ctrl_cfg *cfg, ctrl_state *st) {
  double c = cos(st->kf_x[2]), s = sin(st->kf_x[2]);
  double vx = st->last_vx, vy = st->last_vy, dist = hypot(vx, vy) * WINDOW_S;
  st->kf_x[0] += (c * vx - s * vy) * WINDOW_S;
  st->kf_x[1] += (s * vx + c * vy) * WINDOW_S;
  double qm = sq(cfg->motion_trust * (KF_MOVE_NOISE_FRAC * dist + KF_MOVE_NOISE_MIN));
  double qt = sq(cfg->motion_trust * (KF_TURN_NOISE_PER_M * dist + KF_TURN_NOISE_MIN));
  double gx = (-s * vx - c * vy) * WINDOW_S, gy = (c * vx - s * vy) * WINDOW_S;
  double P[3][3];
  memcpy(P, st->kf_P, sizeof P);
  double F[3][3] = {{1, 0, gx}, {0, 1, gy}, {0, 0, 1}}, FP[3][3] = {{0}};
  for (int a = 0; a < 3; a++)
    for (int b = 0; b < 3; b++)
      for (int k = 0; k < 3; k++) FP[a][b] += F[a][k] * P[k][b];
  for (int a = 0; a < 3; a++)
    for (int b = 0; b < 3; b++) {
      double v = 0;
      for (int k = 0; k < 3; k++) v += FP[a][k] * F[b][k];
      st->kf_P[a][b] = v;
    }
  st->kf_P[0][0] += qm;
  st->kf_P[1][1] += qm;
  st->kf_P[2][2] += qt;
}

static int filter_correct(const ctrl_cfg *cfg, ctrl_state *st, const sensed *s) {
  double R = sq(cfg->fix_trust * KF_BEARING_NOISE);
  int n_used = 0;
  for (int i = 0; i < NBEACON; i++) {
    if (!s->used[i]) continue;
    double H[3];
    double h = bearing_from(cfg, st->kf_x, i, H);
    double miss = wrap(s->bearing[i] - h);
    double PH[3] = {0, 0, 0}, S = R;
    for (int a = 0; a < 3; a++) {
      for (int k = 0; k < 3; k++) PH[a] += st->kf_P[a][k] * H[k];
      S += H[a] * PH[a];
    }
    if (miss * miss > 9 * S) continue;
    for (int a = 0; a < 3; a++) st->kf_x[a] += PH[a] / S * miss;
    st->kf_x[2] = wrap(st->kf_x[2]);
    for (int a = 0; a < 3; a++)
      for (int b = 0; b < 3; b++) st->kf_P[a][b] -= PH[a] * PH[b] / S;
    n_used++;
  }
  return n_used;
}

static void kiwi_inverse(const ctrl_cfg *cfg, double vx, double vy, double *wheel_speed) {
  for (int i = 0; i < NWHEEL; i++) {
    double a = cfg->wheel_angle[i];
    wheel_speed[i] = -sin(a) * vx + cos(a) * vy;
  }
}

void ctrl_init(const ctrl_cfg *cfg, ctrl_state *st) {
  (void)cfg;
  memset(st, 0, sizeof *st);
}

void ctrl_step(const ctrl_cfg *cfg, ctrl_state *st, const ctrl_in *in, ctrl_out *out) {
  memset(out, 0, sizeof *out);
  sensed s;
  sense(cfg, st, in, &s);
  for (int b = 0; b < NBEACON; b++) out->detected[b] = s.used[b];
  hidden_rule(cfg, st, &s);
  for (int b = 0; b < NBEACON; b++) out->used[b] = s.used[b];

  aim_out o;
  memset(&o, 0, sizeof o);
  remember(cfg, st, &s);

  if (st->windows++ < LISTEN_WINDOWS) {
    listen(&o);
  } else {
    if (st->kf_on) {
      filter_predict(cfg, st);
      int n_used = filter_correct(cfg, st, &s);
      double sd = worst_sd(st->kf_P[0][0], st->kf_P[0][1], st->kf_P[1][1]);
      if (sd > cfg->lost_limit) st->kf_on = 0;
      if (st->kf_on) {
        from_pose(cfg, st->kf_x, &o);
        o.branch = n_used ? BRANCH_FILTER_CORRECT : BRANCH_FILTER_PREDICT;
      }
    }
    if (!st->kf_on) {
      double pose[3], m_per_rad;
      if (try_fix(cfg, st, &s, pose, &m_per_rad)) filter_start(cfg, st, pose, m_per_rad, &o);
      else map_aim(cfg, st, &o);
    }
  }

  out->branch = o.branch;
  out->centroid_known = o.centroid_known;
  out->centroid_x = o.centroid_x;
  out->centroid_y = o.centroid_y;

  double vx = 0, vy = 0;
  if (o.move) {
    vx = o.vx;
    vy = o.vy;
    if (o.branch != BRANCH_ORBIT_ONE)
      for (int i = 0; i < NBEACON; i++) st->dodge_side[i] = 0;
  } else if (o.centroid_known) {
    double d = hypot(o.centroid_x, o.centroid_y);
    if (d < STOP_RADIUS) {
      out->arrived = 1;
    } else {
      double dir = atan2(o.centroid_y, o.centroid_x);
      if (o.have_hoops) {
        int worst = -1;
        double worst_r = 0, worst_c = 0;
        for (int i = 0; i < NBEACON; i++) {
          double hx = o.hoop_x[i], hy = o.hoop_y[i], r = hypot(hx, hy);
          double c = cfg->payload_radius + cfg->hoop_radius[i] + cfg->dodge_clearance;
          double cap = hypot(hx - o.centroid_x, hy - o.centroid_y) - DODGE_CENTROID_GAP;
          if (c > cap) c = fmax(cap, 0);
          double along = (hx * o.centroid_x + hy * o.centroid_y) / d;
          double t = fmax(0, fmin(d, along));
          double off = hypot(hx - t * o.centroid_x / d, hy - t * o.centroid_y / d);
          int blocking = along > 0 && along < d && off < c;
          if (!blocking) {
            st->dodge_side[i] = 0;
            continue;
          }
          if (worst < 0 || r < worst_r) {
            worst = i;
            worst_r = r;
            worst_c = c;
          }
        }
        if (worst >= 0) {
          double hb = atan2(o.hoop_y[worst], o.hoop_x[worst]);
          if (!st->dodge_side[worst]) st->dodge_side[worst] = wrap(dir - hb) >= 0 ? 1 : -1;
          dir = hb + st->dodge_side[worst] * asin(fmin(1, worst_c / fmax(worst_r, 1e-9)));
        }
      }
      double v = cfg->top_speed;
      if (d < cfg->taper_start) v = fmax(TAPER_FLOOR * cfg->top_speed, cfg->top_speed * d / cfg->taper_start);
      vx = v * cos(dir);
      vy = v * sin(dir);
    }
  }

  kiwi_inverse(cfg, vx, vy, out->wheel_speed);
  double peak = 0;
  for (int i = 0; i < NWHEEL; i++) peak = fmax(peak, fabs(out->wheel_speed[i]));
  if (peak > cfg->top_speed) {
    double k = cfg->top_speed / peak;
    for (int i = 0; i < NWHEEL; i++) out->wheel_speed[i] *= k;
    vx *= k;
    vy *= k;
  }
  out->vx = vx;
  out->vy = vy;
  st->last_vx = vx;
  st->last_vy = vy;
}
