#include <iostream>
#include <iomanip>
#include <cstdint>
#include <cmath>
#include <unistd.h>

#include "i2c_bus.hpp"
#include "tca9548a.hpp"
#include "pololu_tic_i2c.hpp"

#define STEP_PER_CENTIMETER 1600.0f // 1/8 microstepping 기준

// 이동 완료 대기용. Tic Control Center의 "Max speed" 설정값(pulse/s)을 넣어야 한다.
// 실제보다 작게 잡으면 더 오래 기다릴 뿐이라 안전 방향이다.
#define STEP_RATE_HZ        2000.0f
#define WAIT_MARGIN         2.0f    // 가감속 구간 보정용 배수
#define WAIT_EXTRA_US       300000  // 고정 여유 0.3 s

struct Axis {
    const char* name;
    uint8_t mux_ch;
    float   limit_cm;   // 축별 스트로크 (원점 기준 +-)
    float   stepPerCm;  // 축별 분해능
    int32_t dirSign;    // +1: 입력 그대로, -1: 부호 반전
};

int main()
{
    const char* I2C_DEV = "/dev/i2c-1";
    const uint8_t MUX_ADDR7 = 0x70;
    const uint8_t TIC_ADDR7 = 0x0E;

    //              name  ch  limit_cm  stepPerCm            dirSign
    Axis axes[3] = {
        {"X", 0,  5.0f, STEP_PER_CENTIMETER, -1},
        {"Y", 1,  5.0f, STEP_PER_CENTIMETER, -1},
        {"Z", 2, 10.0f, STEP_PER_CENTIMETER, -1}, // Z만 반대로 가면 여기를 +1로
    };

    // ---------------- I2C init ----------------
    I2CBus bus(I2C_DEV);
    if (!bus.valid()) {
        std::cerr << "[ERROR] Failed to open I2C bus\n";
        return 1;
    }

    TCA9548A mux(MUX_ADDR7);
    TicI2C tic(TIC_ADDR7);

    std::cout << "=============================================\n";
    std::cout << "  AMBS Axis Origin Calibration (I2C)\n";
    std::cout << "=============================================\n\n";

    // -------- axis prepare lambda (INSIDE main) --------
    auto axis_prepare = [&](const Axis& a) -> bool {
        if (!mux.select(bus, a.mux_ch)) {
            std::cerr << "[" << a.name << "] MUX select failed\n";
            return false;
        }

        if (!bus.probeAck(TIC_ADDR7)) {
            std::cerr << "[" << a.name << "] Tic not responding\n";
            return false;
        }

        if (!tic.energize(bus)) {
            std::cerr << "[" << a.name << "] energize failed\n";
            return false;
        }

        if (!tic.exitSafeStart(bus)) {
            std::cerr << "[" << a.name << "] exit-safe-start failed\n";
            return false;
        }

        tic.resetCommandTimeout(bus);

        // 현재 위치를 0으로 '정의'만 한다 (모터는 움직이지 않음).
        // 이렇게 해야 아래 누적 목표의 기준점이 확정된다.
        tic.haltAndSetPosition(bus, 0);
        return true;
    };

    // 이동 소요시간 추정 후 대기.
    // 위치 읽기(Get variables) API가 없어서 시간 기반이다. 스텝 로스는 감지 못 한다.
    auto wait_move_done = [&](int32_t delta_pulse) {
        const float steps = std::fabs(static_cast<float>(delta_pulse));
        const float sec   = (steps / STEP_RATE_HZ) * WAIT_MARGIN;
        usleep(static_cast<useconds_t>(sec * 1e6f) + WAIT_EXTRA_US);
    };

    // ---------------- prepare all axes ----------------
    for (const auto& a : axes) {
        if (!axis_prepare(a)) return 1;
        std::cout << "[" << a.name << "] ready (MUX ch=" << int(a.mux_ch) << ")\n";
        usleep(10000);
    }

    // ---------------- calibration loop ----------------
    for (const auto& a : axes) {

        // 축 루프 '안', while 루프 '밖'에 둬야 한다.
        // setTargetPosition은 절대 위치 명령이므로, 상대 입력을 여기에 누적해서 보낸다.
        int32_t posTarget_pulse = 0;
        float   posTotal_cm     = 0.0f;

        while (true) {
            std::cout << "\n===== " << a.name << " axis calibration =====\n";
            std::cout << "Current offset: " << std::fixed << std::setprecision(2)
                      << posTotal_cm << " cm  (limit +-" << a.limit_cm << " cm)\n";
            std::cout << "Enter distance to move (relative) [cm]: ";

            float pos_cm;
            if (!(std::cin >> pos_cm)) {
                std::cin.clear();
                std::cin.ignore(1024, '\n');
                std::cout << "Invalid input.\n";
                continue;
            }

            // 한 번의 입력이 아니라 '누적 합'을 스트로크와 비교한다.
            const float posNext_cm = posTotal_cm + pos_cm;
            if (posNext_cm < -a.limit_cm || posNext_cm > a.limit_cm) {
                std::cout << "Rejected: total would become " << posNext_cm
                          << " cm, outside +-" << a.limit_cm << " cm\n";
                continue;
            }

            // 상대 이동량 -> 펄스 -> 절대 목표에 누적
            const int32_t delta_pulse =
                static_cast<int32_t>(pos_cm * a.stepPerCm) * a.dirSign;
            posTarget_pulse += delta_pulse;

            if (!mux.select(bus, a.mux_ch)) return 1;
            tic.resetCommandTimeout(bus);
            tic.exitSafeStart(bus);
            tic.setTargetPosition(bus, posTarget_pulse);

            // 도착할 때까지 대기. 안 기다리면 이동 중 halt가 걸려 덜 간 채로 원점이 잡힌다.
            wait_move_done(delta_pulse);
            tic.resetCommandTimeout(bus);

            posTotal_cm = posNext_cm;

            std::cout << "Moved " << pos_cm << " cm.  Total offset: "
                      << posTotal_cm << " cm\n";
            std::cout << "Finish " << a.name << " axis? (y/n): ";

            char ans;
            if (!(std::cin >> ans)) {
                std::cin.clear();
                std::cin.ignore(1024, '\n');
                std::cout << "Invalid input. Please enter 'y' or 'n'.\n";
                continue;
            }

            if (ans == 'y' || ans == 'Y') {
                mux.select(bus, a.mux_ch);
                tic.resetCommandTimeout(bus);
                tic.exitSafeStart(bus);
                tic.haltAndSetPosition(bus, 0);

                std::cout << a.name << " axis origin set.\n";
                break;
            } else if (ans == 'n' || ans == 'N') {
                continue;
            } else {
                std::cout << "Invalid input. Please enter 'y' or 'n'.\n";
            }
        }
    }

    std::cout << "\nCalibration completed.\n";
    return 0;
}
