// SPDX-FileCopyrightText: 2026 Encos
// SPDX-License-Identifier: MIT

#include "JointManager.h"

int main() {
    // Exercise the packaged solver DSO as well as the driver plugin.
    const couple_joint::MechanismSolver solver(couple_joint::CalibrationInput{});
    if (solver.GetCalibrationStatus() != couple_joint::CalibrationStatus::CoincidentJointCenters) {
        return 1;
    }
    encos::DefaultJointManager manager;
    manager.CreateAdapter(nlohmann::json{{"type", "Fake"}, {"name", "installed_consumer"}});
}
