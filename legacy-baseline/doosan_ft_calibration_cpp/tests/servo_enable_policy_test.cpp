#include "servo_enable_policy.hpp"
#include <cassert>

int main() {
    ServoEnablePolicy already_ready(false);
    assert(already_ready.observe(ServoState::Standby,true)==ServoAction::Ready);

    ServoEnablePolicy acknowledged(true);
    assert(acknowledged.observe(ServoState::SafeOff,true)==ServoAction::EnableOnce);
    assert(acknowledged.observe(ServoState::SafeOff,false)==ServoAction::Reject);
    assert(acknowledged.observe(ServoState::SafeOff,true)==ServoAction::Wait);
    assert(acknowledged.observe(ServoState::Standby,true)==ServoAction::Ready);

    ServoEnablePolicy changed_after_confirmation(false);
    assert(changed_after_confirmation.observe(ServoState::SafeOff,true)==ServoAction::NeedConfirmation);
    changed_after_confirmation.confirm_safe_off();
    assert(changed_after_confirmation.observe(ServoState::SafeOff,true)==ServoAction::EnableOnce);
    assert(changed_after_confirmation.observe(ServoState::SafeOff,true)==ServoAction::Wait);

    ServoEnablePolicy denied(true);
    assert(denied.observe(ServoState::SafeOff,false)==ServoAction::Reject);
    assert(denied.observe(ServoState::Other,true)==ServoAction::Reject);
    assert(denied.observe(ServoState::Standby,false)==ServoAction::Reject);
}
