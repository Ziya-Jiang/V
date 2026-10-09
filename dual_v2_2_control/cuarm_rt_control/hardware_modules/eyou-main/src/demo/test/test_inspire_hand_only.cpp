#include "inspire_hand.h"
#include "zlg_hand_transport.h"


#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>
#include <unistd.h>

#ifdef ARM_ARCH
constexpr int kCanableDeviceIndex = 2;
#else
constexpr int kCanableDeviceIndex = 0;
#endif

namespace {
void printUsage(const char* program) {
    std::cout
        << "Usage:\n"
        << "  " << program << " [canable|zlg_fd|zlg_can20] [hand_idx] [hand0_id] [hand1_id] [dev_idx] [chn_idx] [open_home|joint_idx] [target] [hold_ms]\n\n"
        << "Defaults:\n"
        << "  mode=canable hand_idx=0 hand_ids={2,1} dev_idx=0 chn_idx=0\n"
        << "  Without joint_idx, run the legacy all-joint test.\n"
        << "  With joint_idx, move only that channel and restore its initial angle.\n\n"
        << "Examples:\n"
        << "  " << program << " canable\n"
        << "  " << program << " canable 0 2 1 0 0 open_home\n"
        << "  " << program << " canable 0 2 1 0 0 3 600 2000\n"
        << "  " << program << " zlg_fd 0 2 1 0 0\n"
        << "  " << program << " zlg_can20 1 2 1 0 0\n";
}

int parseIntArg(char** argv, int argc, int index, int default_value) {
    if (index >= argc) {
        return default_value;
    }
    return std::atoi(argv[index]);
}

void printJointData(const InspireHand& hand, int hand_idx) {
    for (int i = 0; i < 6; ++i) {
        std::cout << "joint " << i
                  << " angle=" << hand.joint_angle_[hand_idx][i]
                  << " force=" << hand.joint_force_[hand_idx][i]
                  << " current=" << hand.joint_current_[hand_idx][i]
                  << " error=" << hand.joint_error_[hand_idx][i]
                  << " status=" << hand.joint_status_[hand_idx][i]
                  << std::endl;
    }
}

std::shared_ptr<IHandCanTransport> makeTransport(const std::string& mode,
                                                 const int hand_ids[2],
                                                 int dev_idx,
                                                 int chn_idx) {
    if (mode == "canable") {
        return std::make_shared<CanableHandTransport>(hand_ids);
    }
    if (mode == "zlg_fd") {
        return std::make_shared<ZlgCanFdHandTransport>(dev_idx, chn_idx);
    }
    if (mode == "zlg_can20") {
        return std::make_shared<ZlgCan20HandTransport>(dev_idx, chn_idx);
    }
    return nullptr;
}
}

int main(int argc, char** argv) {
    const std::string mode = argc > 1 ? argv[1] : "canable";
    if (mode == "-h" || mode == "--help") {
        printUsage(argv[0]);
        return 0;
    }

    int hand_idx = parseIntArg(argv, argc, 2, 0);
    int hand_ids[2] = {
        parseIntArg(argv, argc, 3, 2),
        parseIntArg(argv, argc, 4, 1),
    };
    const int dev_idx = parseIntArg(argv, argc, 5, 0);
    const int chn_idx = parseIntArg(argv, argc, 6, 0);
    const std::string action = argc > 7 ? argv[7] : "";
    const bool open_home = action == "open_home";
    const bool single_joint_test = argc > 7 && !open_home;
    const int joint_idx = parseIntArg(argv, argc, 7, -1);
    const int target_angle = parseIntArg(argv, argc, 8, 600);
    const int hold_ms = parseIntArg(argv, argc, 9, 2000);

    if (hand_idx < 0 || hand_idx > 1) {
        std::cout << "hand_idx must be 0 or 1." << std::endl;
        return -1;
    }
    if (single_joint_test && (joint_idx < 0 || joint_idx >= 6)) {
        std::cout << "joint_idx must be between 0 and 5." << std::endl;
        return -1;
    }
    if (target_angle < 0 || target_angle > 1000 || hold_ms < 0) {
        std::cout << "target must be 0..1000 and hold_ms must be non-negative."
                  << std::endl;
        return -1;
    }

    auto transport = makeTransport(mode, hand_ids, dev_idx, chn_idx);
    if (!transport) {
        std::cout << "Unknown mode: " << mode << std::endl;
        printUsage(argv[0]);
        return -1;
    }

    InspireHand hand(hand_ids, transport);

    std::cout << "Testing InspireHand only." << std::endl;
    std::cout << "mode=" << mode
              << " hand_idx=" << hand_idx
              << " hand_id=" << hand_ids[hand_idx]
              << " dev_idx=" << dev_idx
              << " chn_idx=" << chn_idx
              << std::endl;

    int16_t speed[6] = {100, 100, 100, 100, 100, 100};
    int16_t force[6] = {200, 200, 200, 200, 200, 200};
    int16_t safe_open[6] = {200, 200, 200, 200, 200, 200};
    int16_t safe_mid[6] = {500, 500, 500, 500, 500, 500};

    if (hand.getAngles(hand_idx) != 0) {
        std::cout << "Initial getAngles failed. Check power, CAN wiring, hand ID, and transport mode." << std::endl;
        return -1;
    }
    std::cout << "Initial getAngles success." << std::endl;
    printJointData(hand, hand_idx);

    hand.setSpeed(hand_idx, speed);
    usleep(2000);
    hand.setForce(hand_idx, force);
    usleep(2000);

    if (open_home) {
        if (hand.getError(hand_idx) != 0 || hand.getStatus(hand_idx) != 0) {
            std::cout << "Failed to read hand fault/status before OPEN_HOME."
                      << std::endl;
            return -1;
        }

        bool has_recoverable_fault = false;
        bool has_non_recoverable_fault = false;
        for (int i = 0; i < 6; ++i) {
            const int error = static_cast<unsigned short>(
                hand.joint_error_[hand_idx][i]) & 0xff;
            const int status = static_cast<unsigned short>(
                hand.joint_status_[hand_idx][i]) & 0xff;
            has_recoverable_fault |=
                (error & (0x01 | 0x04)) != 0 || status == 4 || status == 5;
            has_non_recoverable_fault |=
                (error & (0x02 | 0x08 | 0x10)) != 0 || status == 6;
        }
        if (has_non_recoverable_fault) {
            std::cout << "OPEN_HOME blocked: non-recoverable hand fault is active."
                      << std::endl;
            printJointData(hand, hand_idx);
            return -1;
        }
        if (has_recoverable_fault) {
            std::cout << "Clearing recoverable stall/overcurrent fault before OPEN_HOME."
                      << std::endl;
            if (hand.clearError(hand_idx) != 0) {
                std::cout << "Failed to clear recoverable hand fault." << std::endl;
                return -1;
            }
            usleep(100000);
        }

        int16_t home_angles[6] = {1000, 1000, 1000, 1000, 1000, 0};
        bool reached = false;
        for (int attempt = 1; attempt <= 3; ++attempt) {
            std::cout << "OPEN_HOME attempt " << attempt
                      << "/3 target=[1000,1000,1000,1000,1000,0]"
                      << std::endl;
            hand.setAngles(hand_idx, home_angles);
            usleep(500000);
            if (hand.getAngles(hand_idx) != 0) {
                std::cout << "OPEN_HOME angle verification read failed."
                          << std::endl;
                continue;
            }

            reached = true;
            for (int i = 0; i < 5; ++i) {
                reached &= hand.joint_angle_[hand_idx][i] >= 900;
            }
            reached &= hand.joint_angle_[hand_idx][5] <= 100;
            printJointData(hand, hand_idx);
            if (reached) {
                std::cout << "OPEN_HOME verified." << std::endl;
                break;
            }
        }

        hand.getForces(hand_idx);
        hand.getCurrent(hand_idx);
        hand.getError(hand_idx);
        hand.getStatus(hand_idx);
        std::cout << "OPEN_HOME final state:" << std::endl;
        printJointData(hand, hand_idx);
        if (!reached) {
            std::cout << "OPEN_HOME failed: one or more channels did not reach "
                         "the verified open range."
                      << std::endl;
            return 2;
        }
        return 0;
    }

    if (single_joint_test) {
        int16_t initial_angles[6];
        int16_t test_angles[6];
        for (int i = 0; i < 6; ++i) {
            initial_angles[i] = hand.joint_angle_[hand_idx][i];
            test_angles[i] = initial_angles[i];
        }
        test_angles[joint_idx] = static_cast<int16_t>(target_angle);

        std::cout << "Single-joint test: channel=" << joint_idx
                  << " initial=" << initial_angles[joint_idx]
                  << " target=" << target_angle
                  << " hold_ms=" << hold_ms << std::endl;
        hand.setAngles(hand_idx, test_angles);
        usleep(static_cast<useconds_t>(hold_ms) * 1000);
        hand.getAngles(hand_idx);
        std::cout << "Angles at test target:" << std::endl;
        printJointData(hand, hand_idx);

        std::cout << "Restoring all initial angles." << std::endl;
        hand.setAngles(hand_idx, initial_angles);
        usleep(500000);
        hand.getAngles(hand_idx);
        std::cout << "Angles after restore:" << std::endl;
        printJointData(hand, hand_idx);
        return 0;
    }

    std::cout << "Set safe_open angles." << std::endl;
    hand.setAngles(hand_idx, safe_open);
    usleep(300000);

    std::cout << "Set safe_mid angles." << std::endl;
    hand.setAngles(hand_idx, safe_mid);
    usleep(300000);

    int ret = 0;
    ret |= hand.getAngles(hand_idx);
    usleep(2000);
    ret |= hand.getForces(hand_idx);
    usleep(2000);
    ret |= hand.getCurrent(hand_idx);
    usleep(2000);
    ret |= hand.getError(hand_idx);
    usleep(2000);
    ret |= hand.getStatus(hand_idx);

    if (ret != 0) {
        std::cout << "One or more read commands failed, ret=" << ret << std::endl;
        return -1;
    }

    std::cout << "Final read success." << std::endl;
    printJointData(hand, hand_idx);

    return 0;
}
