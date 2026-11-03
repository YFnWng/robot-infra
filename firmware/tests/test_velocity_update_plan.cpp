#include "../teensy_tekceleo/velocity_update_plan.h"

#include <assert.h>
#include <stdint.h>
#include <initializer_list>

namespace {

using VelocityUpdatePlan::AxisState;
using VelocityUpdatePlan::Plan;

AxisState idle() { return {false, false, false, false, '1', '1'}; }
AxisState running(uint8_t current, uint8_t target) {
  return {true, true, true, true, target, current};
}
AxisState from_rest(uint8_t target) { return {true, false, true, false, target, '1'}; }

void test_same_direction_speed_change_is_speed_only_in_both_policies() {
  AxisState axes[6] = {running('1', '1'), idle(), running('0', '0'), idle(), idle(), idle()};
  for (bool on_the_fly : {false, true}) {
    const Plan p = VelocityUpdatePlan::plan(axes, 6, on_the_fly);
    assert(p.speed_mask == 0x05 && p.reverse_mask == 0 && p.start_mask == 0);
  }
}

void test_legacy_restarts_every_active_axis_when_one_starts() {
  // Tendon (axis 2) running; insertion (axis 0) starts from rest.
  AxisState axes[6] = {from_rest('1'), idle(), running('1', '1'), idle(), idle(), idle()};
  const Plan p = VelocityUpdatePlan::plan(axes, 6, false);
  assert(p.start_mask == 0x05 && p.speed_mask == 0 && p.reverse_mask == 0);
}

void test_on_the_fly_starts_only_the_axis_from_rest() {
  AxisState axes[6] = {from_rest('1'), idle(), running('1', '1'), idle(), idle(), idle()};
  const Plan p = VelocityUpdatePlan::plan(axes, 6, true);
  assert(p.start_mask == 0x01 && p.speed_mask == 0x04 && p.reverse_mask == 0);
}

void test_legacy_restarts_on_reversal_on_the_fly_reverses_in_place() {
  AxisState axes[6] = {running('1', '1'), idle(), running('1', '0'), idle(), idle(), idle()};
  const Plan legacy = VelocityUpdatePlan::plan(axes, 6, false);
  assert(legacy.start_mask == 0x05 && legacy.speed_mask == 0 && legacy.reverse_mask == 0);
  const Plan live = VelocityUpdatePlan::plan(axes, 6, true);
  assert(live.reverse_mask == 0x04 && live.speed_mask == 0x01 && live.start_mask == 0);
}

void test_enabled_axis_with_unknown_mode_or_direction_is_restarted() {
  // A driver that lost its mode or direction cache is not treated as running.
  AxisState lost_mode = running('1', '1');
  lost_mode.mode_valid = false;
  AxisState lost_dir = running('1', '1');
  lost_dir.direction_valid = false;
  AxisState axes[6] = {lost_mode, idle(), lost_dir, idle(), idle(), idle()};
  const Plan p = VelocityUpdatePlan::plan(axes, 6, true);
  assert(p.start_mask == 0x05 && p.speed_mask == 0 && p.reverse_mask == 0);
}

void test_inactive_axes_never_appear() {
  AxisState axes[6] = {idle(), idle(), idle(), idle(), idle(), idle()};
  axes[3].enabled = true;  // enabled but commanded zero: stopped elsewhere
  for (bool on_the_fly : {false, true}) {
    const Plan p = VelocityUpdatePlan::plan(axes, 6, on_the_fly);
    assert(p.start_mask == 0 && p.speed_mask == 0 && p.reverse_mask == 0);
  }
}

}  // namespace

int main() {
  test_same_direction_speed_change_is_speed_only_in_both_policies();
  test_legacy_restarts_every_active_axis_when_one_starts();
  test_on_the_fly_starts_only_the_axis_from_rest();
  test_legacy_restarts_on_reversal_on_the_fly_reverses_in_place();
  test_enabled_axis_with_unknown_mode_or_direction_is_restarted();
  test_inactive_axes_never_appear();
  return 0;
}
