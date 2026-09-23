// SPDX-FileCopyrightText: 2026 Encos
// SPDX-License-Identifier: MIT

#include "JointManager.h"

int main() {
    encos::DefaultJointManager manager;
    manager.CreateAdapter(nlohmann::json{{"type", "Fake"}, {"name", "installed_consumer"}});
}
