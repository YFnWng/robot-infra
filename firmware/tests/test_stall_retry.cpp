#include "../teensy_tekceleo/stall_retry.h"

#include <assert.h>
#include <stdint.h>

void test_two_retries_then_escalation() {
  StallRetryBudget budget;
  assert(budget.request(1000));
  assert(budget.retryCount() == 1);
  assert(budget.coolingDown(1100));
  assert(!budget.coolingDown(1150));
  assert(budget.request(2200));
  assert(budget.retryCount() == 2);
  assert(!budget.request(3300));
}

void test_budget_resets_after_quiet_window() {
  StallRetryBudget budget;
  assert(budget.request(1000));
  assert(budget.request(2000));
  assert(budget.request(7001));
  assert(budget.retryCount() == 1);
}

void test_explicit_reset_restores_budget() {
  StallRetryBudget budget;
  assert(budget.request(1000));
  assert(budget.request(2000));
  assert(!budget.request(3000));
  budget.reset();
  assert(budget.request(3001));
  assert(budget.retryCount() == 1);
}

void test_cooldown_handles_millis_rollover() {
  StallRetryBudget budget;
  constexpr uint32_t start = UINT32_MAX - 99U;
  assert(budget.request(start));
  assert(budget.coolingDown(20));
  assert(!budget.coolingDown(51));
}

int main() {
  test_two_retries_then_escalation();
  test_budget_resets_after_quiet_window();
  test_explicit_reset_restores_budget();
  test_cooldown_handles_millis_rollover();
  return 0;
}
