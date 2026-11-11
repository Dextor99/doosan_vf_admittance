#pragma once

// Only ordinary SAFE_OFF is eligible for a single operator-confirmed request.
enum class ServoState { Standby, SafeOff, Other };
enum class ServoAction { Ready, NeedConfirmation, EnableOnce, Wait, Reject };

class ServoEnablePolicy {
public:
    explicit ServoEnablePolicy(bool safe_off_confirmed) : confirmed_(safe_off_confirmed) {}
    void confirm_safe_off() { confirmed_=true; }
    ServoAction observe(ServoState state,bool conditions_ok) {
        if(!conditions_ok)return ServoAction::Reject;
        if(state==ServoState::Standby)return ServoAction::Ready;
        if(state!=ServoState::SafeOff)return ServoAction::Reject;
        if(!confirmed_)return ServoAction::NeedConfirmation;
        if(sent_)return ServoAction::Wait;
        sent_=true;
        return ServoAction::EnableOnce;
    }
private:
    bool confirmed_=false, sent_=false;
};
