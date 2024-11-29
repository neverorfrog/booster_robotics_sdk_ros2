#pragma once

#include "booster_interface/msg/booster_api_req_msg.hpp"
#include <booster/robot/common/robot_shared.hpp>
#include <booster/robot/b1/b1_api_const.hpp>
#include <booster/robot/b1/b1_loco_api.hpp>

namespace booster_interface {

msg::BoosterApiReqMsg ConstructMsg(booster::robot::b1::LocoApiId api_id, nlohmann::json json_body) {
    msg::BoosterApiReqMsg msg;
    msg.api_id = static_cast<int64_t>(api_id);
    msg.body = json_body.dump();
    return msg;
}

msg::BoosterApiReqMsg CreateChangeModeMsg(booster::robot::RobotMode mode) {
    nlohmann::json body;
    body["mode"] = static_cast<int>(mode);
    return ConstructMsg(booster::robot::b1::LocoApiId::kChangeMode, body);
}

msg::BoosterApiReqMsg CreateMoveMsg(float vx, float vy, float vyaw) {
    nlohmann::json body;
    body["vx"] = vx;
    body["vy"] = vy;
    body["vyaw"] = vyaw;
    return ConstructMsg(booster::robot::b1::LocoApiId::kMove, body);
}

msg::BoosterApiReqMsg CreateRotateHeadMsg(float pitch, float yaw) {
    nlohmann::json body;
    body["pitch"] = pitch;
    body["yaw"] = yaw;
    return ConstructMsg(booster::robot::b1::LocoApiId::kRotateHead, body);
}

msg::BoosterApiReqMsg CreateRotateHeadWithDirectionMsg(int pitch_direction, int yaw_direction) {
    nlohmann::json body;
    body["pitch_direction"] = pitch_direction;
    body["yaw_direction"] = yaw_direction;
    return ConstructMsg(booster::robot::b1::LocoApiId::kRotateHeadWithDirection, body);
}

msg::BoosterApiReqMsg CreateWaveHandMsg(booster::robot::b1::HandIndex hand_index, booster::robot::b1::HandAction hand_action) {
    nlohmann::json body;
    body["hand_index"] = static_cast<int>(hand_index);
    body["hand_action"] = static_cast<int>(hand_action);
    return ConstructMsg(booster::robot::b1::LocoApiId::kWaveHand, body);
}

msg::BoosterApiReqMsg CreateLieDownMsg() {
    return ConstructMsg(booster::robot::b1::LocoApiId::kLieDown, "");
}

msg::BoosterApiReqMsg CreateGetUpMsg() {
    return ConstructMsg(booster::robot::b1::LocoApiId::kGetUp, "");
}
} // namespace booster_interface