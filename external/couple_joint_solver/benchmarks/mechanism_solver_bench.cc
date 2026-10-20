#include <nanobench.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "mechanism_solver.h"

namespace {

constexpr std::array<int, 6> kFrequencySamplesHz = {100, 200, 300, 500, 800, 1000};
constexpr double kAngularSpeed = 30.0;
constexpr double kSearchTolerance = 1.0e-4;
constexpr int kSigmaSearchGridSamples = 512;

void Require(bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

bool CanUseSolveResult(couple_joint::SolveStatus status) {
    return status == couple_joint::SolveStatus::Success || status == couple_joint::SolveStatus::NearSingular;
}

const char* SolveStatusName(couple_joint::SolveStatus status) {
    switch (status) {
        case couple_joint::SolveStatus::Success:
            return "Success";
        case couple_joint::SolveStatus::NearSingular:
            return "NearSingular";
        case couple_joint::SolveStatus::InvalidCalibration:
            return "InvalidCalibration";
        case couple_joint::SolveStatus::DegenerateSideEquation:
            return "DegenerateSideEquation";
        case couple_joint::SolveStatus::UnreachablePose:
            return "UnreachablePose";
        case couple_joint::SolveStatus::SingularJacobian:
            return "SingularJacobian";
        case couple_joint::SolveStatus::ForwardNotConverged:
            return "ForwardNotConverged";
    }
    return "Unknown";
}

double Norm2(double value1, double value2) {
    return std::sqrt(value1 * value1 + value2 * value2);
}

couple_joint::CalibrationInput CreateValidCalibration() {
    couple_joint::CalibrationInput input;
    input.left_joint_center = {-35.462080740, 29.491263548, 58.546474855};
    input.right_joint_center = {-35.462080740, -29.491263548, 58.546474855};
    input.left_planar_endpoint = {-35.5, 11.61629736, 56.42889465};
    input.right_planar_endpoint = {-35.5, -11.61629736, 56.42889465};
    input.roll_center = {0.0, 0.0, 0.0};
    input.pitch_center = {0.0, 0.0, 0.0};
    input.left_spatial_endpoint = {-36.6, 26.0, -5.0};
    input.right_spatial_endpoint = {-36.6, -26.0, -5.0};
    input.rotation_order = couple_joint::RotationOrder::PitchThenRoll;
    return input;
}

couple_joint::MechanismSolver CreateCalibratedSolver() {
    couple_joint::MechanismSolver solver(CreateValidCalibration());
    Require(solver.GetCalibrationStatus() == couple_joint::CalibrationStatus::Success, "benchmark calibration failed");
    return solver;
}

couple_joint::MechanismSolver CreateOriginSolver() {
    couple_joint::MechanismSolver solver = CreateCalibratedSolver();
    const couple_joint::SolveResult origin_result = solver.SolveInverse(0.0, 0.0);
    Require(CanUseSolveResult(origin_result.status), "failed to establish theta origin from SolveInverse(0, 0)");
    return solver;
}

struct InverseProbe {
    double roll{0.0};
    double pitch{0.0};
    couple_joint::SolveResult result;
    couple_joint::MechanismState state;
    bool usable{false};
};

struct ForwardProbe {
    double theta1{0.0};
    double theta2{0.0};
    couple_joint::SolveResult result;
    couple_joint::MechanismState state;
    bool usable{false};
};

struct BoundarySamples {
    InverseProbe origin;
    InverseProbe sigma_0_2;
    InverseProbe sigma_0_1;
    InverseProbe boundary_inside;
    InverseProbe boundary_outside;
};

struct InverseSummaryRow {
    std::string label;
    InverseProbe probe;
    double roundtrip_error{0.0};
};

struct ForwardSummaryRow {
    std::string label;
    ForwardProbe probe;
    double roundtrip_error{0.0};
};

InverseProbe ProbeInverse(const couple_joint::MechanismSolver& origin_solver, double roll, double pitch) {
    couple_joint::MechanismSolver solver = origin_solver;
    const couple_joint::SolveResult result = solver.SolveInverse(roll, pitch);
    return {roll, pitch, result, solver.GetState(), CanUseSolveResult(result.status)};
}

ForwardProbe ProbeForward(const couple_joint::MechanismSolver& origin_solver, double theta1, double theta2) {
    couple_joint::MechanismSolver solver = origin_solver;
    const couple_joint::SolveResult result = solver.SolveForward(theta1, theta2);
    return {theta1, theta2, result, solver.GetState(), CanUseSolveResult(result.status)};
}

ankerl::nanobench::Bench MakeBench() {
    ankerl::nanobench::Bench bench;
    bench.epochs(11).minEpochTime(std::chrono::milliseconds(5));
    return bench;
}

void ConsumeSolveResult(const couple_joint::SolveResult& result, const couple_joint::MechanismState& state) {
    ankerl::nanobench::doNotOptimizeAway(static_cast<int>(result.status));
    ankerl::nanobench::doNotOptimizeAway(result.residual);
    ankerl::nanobench::doNotOptimizeAway(result.iterations);
    ankerl::nanobench::doNotOptimizeAway(result.transmission.sigma_min_j);
    ankerl::nanobench::doNotOptimizeAway(result.transmission.condition_j);
    ankerl::nanobench::doNotOptimizeAway(state.theta1);
    ankerl::nanobench::doNotOptimizeAway(state.theta2);
    ankerl::nanobench::doNotOptimizeAway(state.roll);
    ankerl::nanobench::doNotOptimizeAway(state.pitch);
}

std::string FormatDouble(double value) {
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(6) << value;
    return stream.str();
}

std::string FormatInteger(int value) {
    return std::to_string(value);
}

std::string FormatMarkdownCell(const std::string& text, size_t width, bool align_left) {
    std::ostringstream stream;
    if (align_left) {
        stream << std::left;
    } else {
        stream << std::right;
    }
    stream << std::setw(static_cast<int>(width)) << text;
    return stream.str();
}

std::string MakeMarkdownSeparator(size_t width, bool align_left) {
    Require(width >= 3, "markdown separator width must be at least 3");
    std::string separator(width, '-');
    if (align_left) {
        separator.front() = ':';
    } else {
        separator.back() = ':';
    }
    return separator;
}

void PrintNanobenchHeader() {
    std::cout << "|               ns/op |                op/s |    err% |     total | benchmark\n";
    std::cout << "|--------------------:|--------------------:|--------:|----------:|:----------\n";
}

void PrimeNanobenchHeaderState() {
    std::ostringstream sink;
    ankerl::nanobench::Bench bench = MakeBench();
    bench.output(&sink).run("header-prime", [] {});
}

void PrintInverseSummaryTable(const std::string& title, const std::vector<InverseSummaryRow>& rows) {
    size_t case_width = std::string("case").size();
    size_t roll_width = std::string("roll").size();
    size_t pitch_width = std::string("pitch").size();
    size_t status_width = std::string("status").size();
    size_t residual_width = std::string("residual").size();
    size_t iterations_width = std::string("iterations").size();
    size_t sigma_width = std::string("sigma_min_j").size();
    size_t roundtrip_width = std::string("roundtrip_error").size();

    for (const InverseSummaryRow& row : rows) {
        case_width = std::max(case_width, row.label.size());
        roll_width = std::max(roll_width, FormatDouble(row.probe.roll).size());
        pitch_width = std::max(pitch_width, FormatDouble(row.probe.pitch).size());
        status_width = std::max(status_width, std::string(SolveStatusName(row.probe.result.status)).size());
        residual_width = std::max(residual_width, FormatDouble(row.probe.result.residual).size());
        iterations_width = std::max(iterations_width, FormatInteger(row.probe.result.iterations).size());
        sigma_width = std::max(sigma_width, FormatDouble(row.probe.result.transmission.sigma_min_j).size());
        roundtrip_width = std::max(roundtrip_width, FormatDouble(row.roundtrip_error).size());
    }

    std::cout << "[stability][inverse][" << title << "]\n";
    std::cout << "| " << FormatMarkdownCell("case", case_width, true) << " | "
              << FormatMarkdownCell("roll", roll_width, false) << " | "
              << FormatMarkdownCell("pitch", pitch_width, false) << " | "
              << FormatMarkdownCell("status", status_width, true) << " | "
              << FormatMarkdownCell("residual", residual_width, false) << " | "
              << FormatMarkdownCell("iterations", iterations_width, false) << " | "
              << FormatMarkdownCell("sigma_min_j", sigma_width, false) << " | "
              << FormatMarkdownCell("roundtrip_error", roundtrip_width, false) << " |\n";
    std::cout << "|" << MakeMarkdownSeparator(case_width + 2, true) << "|"
              << MakeMarkdownSeparator(roll_width + 2, false) << "|" << MakeMarkdownSeparator(pitch_width + 2, false)
              << "|" << MakeMarkdownSeparator(status_width + 2, true) << "|"
              << MakeMarkdownSeparator(residual_width + 2, false) << "|"
              << MakeMarkdownSeparator(iterations_width + 2, false) << "|"
              << MakeMarkdownSeparator(sigma_width + 2, false) << "|"
              << MakeMarkdownSeparator(roundtrip_width + 2, false) << "|\n";
    for (const InverseSummaryRow& row : rows) {
        std::cout << "| " << FormatMarkdownCell(row.label, case_width, true) << " | "
                  << FormatMarkdownCell(FormatDouble(row.probe.roll), roll_width, false) << " | "
                  << FormatMarkdownCell(FormatDouble(row.probe.pitch), pitch_width, false) << " | "
                  << FormatMarkdownCell(SolveStatusName(row.probe.result.status), status_width, true) << " | "
                  << FormatMarkdownCell(FormatDouble(row.probe.result.residual), residual_width, false) << " | "
                  << FormatMarkdownCell(FormatInteger(row.probe.result.iterations), iterations_width, false) << " | "
                  << FormatMarkdownCell(FormatDouble(row.probe.result.transmission.sigma_min_j), sigma_width, false)
                  << " | " << FormatMarkdownCell(FormatDouble(row.roundtrip_error), roundtrip_width, false) << " |\n";
    }
}

void PrintForwardSummaryTable(const std::string& title, const std::vector<ForwardSummaryRow>& rows) {
    size_t case_width = std::string("case").size();
    size_t theta1_width = std::string("theta1").size();
    size_t theta2_width = std::string("theta2").size();
    size_t status_width = std::string("status").size();
    size_t residual_width = std::string("residual").size();
    size_t iterations_width = std::string("iterations").size();
    size_t sigma_width = std::string("sigma_min_j").size();
    size_t roundtrip_width = std::string("roundtrip_error").size();

    for (const ForwardSummaryRow& row : rows) {
        case_width = std::max(case_width, row.label.size());
        theta1_width = std::max(theta1_width, FormatDouble(row.probe.theta1).size());
        theta2_width = std::max(theta2_width, FormatDouble(row.probe.theta2).size());
        status_width = std::max(status_width, std::string(SolveStatusName(row.probe.result.status)).size());
        residual_width = std::max(residual_width, FormatDouble(row.probe.result.residual).size());
        iterations_width = std::max(iterations_width, FormatInteger(row.probe.result.iterations).size());
        sigma_width = std::max(sigma_width, FormatDouble(row.probe.result.transmission.sigma_min_j).size());
        roundtrip_width = std::max(roundtrip_width, FormatDouble(row.roundtrip_error).size());
    }

    std::cout << "[stability][forward][" << title << "]\n";
    std::cout << "| " << FormatMarkdownCell("case", case_width, true) << " | "
              << FormatMarkdownCell("theta1", theta1_width, false) << " | "
              << FormatMarkdownCell("theta2", theta2_width, false) << " | "
              << FormatMarkdownCell("status", status_width, true) << " | "
              << FormatMarkdownCell("residual", residual_width, false) << " | "
              << FormatMarkdownCell("iterations", iterations_width, false) << " | "
              << FormatMarkdownCell("sigma_min_j", sigma_width, false) << " | "
              << FormatMarkdownCell("roundtrip_error", roundtrip_width, false) << " |\n";
    std::cout << "|" << MakeMarkdownSeparator(case_width + 2, true) << "|"
              << MakeMarkdownSeparator(theta1_width + 2, false) << "|" << MakeMarkdownSeparator(theta2_width + 2, false)
              << "|" << MakeMarkdownSeparator(status_width + 2, true) << "|"
              << MakeMarkdownSeparator(residual_width + 2, false) << "|"
              << MakeMarkdownSeparator(iterations_width + 2, false) << "|"
              << MakeMarkdownSeparator(sigma_width + 2, false) << "|"
              << MakeMarkdownSeparator(roundtrip_width + 2, false) << "|\n";
    for (const ForwardSummaryRow& row : rows) {
        std::cout << "| " << FormatMarkdownCell(row.label, case_width, true) << " | "
                  << FormatMarkdownCell(FormatDouble(row.probe.theta1), theta1_width, false) << " | "
                  << FormatMarkdownCell(FormatDouble(row.probe.theta2), theta2_width, false) << " | "
                  << FormatMarkdownCell(SolveStatusName(row.probe.result.status), status_width, true) << " | "
                  << FormatMarkdownCell(FormatDouble(row.probe.result.residual), residual_width, false) << " | "
                  << FormatMarkdownCell(FormatInteger(row.probe.result.iterations), iterations_width, false) << " | "
                  << FormatMarkdownCell(FormatDouble(row.probe.result.transmission.sigma_min_j), sigma_width, false)
                  << " | " << FormatMarkdownCell(FormatDouble(row.roundtrip_error), roundtrip_width, false) << " |\n";
    }
}

double FindBoundaryMagnitude(const couple_joint::MechanismSolver& origin_solver, double tolerance,
                             double* outside_magnitude) {
    double low = 0.0;
    double high = 0.1;
    InverseProbe probe = ProbeInverse(origin_solver, -high, 0.0);
    while (probe.usable) {
        low = high;
        high *= 2.0;
        Require(high <= 8.0, "roll boundary search exceeded expected range");
        probe = ProbeInverse(origin_solver, -high, 0.0);
    }

    while (high - low > tolerance) {
        const double mid = 0.5 * (low + high);
        const InverseProbe mid_probe = ProbeInverse(origin_solver, -mid, 0.0);
        if (mid_probe.usable) {
            low = mid;
        } else {
            high = mid;
        }
    }

    *outside_magnitude = high;
    return low;
}

double FindSigmaTargetMagnitude(const couple_joint::MechanismSolver& origin_solver, double target_sigma_min_j,
                                double boundary_inside_magnitude, double tolerance) {
    InverseProbe previous_probe = ProbeInverse(origin_solver, 0.0, 0.0);
    Require(previous_probe.usable, "origin inverse probe is unexpectedly unusable");
    if (previous_probe.result.transmission.sigma_min_j <= target_sigma_min_j) {
        return 0.0;
    }

    double low = 0.0;
    double high = -1.0;
    for (int index = 1; index <= kSigmaSearchGridSamples; ++index) {
        const double magnitude = boundary_inside_magnitude * static_cast<double>(index) / kSigmaSearchGridSamples;
        const InverseProbe current_probe = ProbeInverse(origin_solver, -magnitude, 0.0);
        Require(current_probe.usable, "sigma search stepped outside usable boundary unexpectedly");
        if (current_probe.result.transmission.sigma_min_j <= target_sigma_min_j) {
            low = boundary_inside_magnitude * static_cast<double>(index - 1) / kSigmaSearchGridSamples;
            high = magnitude;
            break;
        }
        previous_probe = current_probe;
    }
    Require(high > 0.0, "failed to bracket sigma_min_j target before boundary");

    while (high - low > tolerance) {
        const double mid = 0.5 * (low + high);
        const InverseProbe mid_probe = ProbeInverse(origin_solver, -mid, 0.0);
        Require(mid_probe.usable, "sigma target refinement stepped outside usable boundary");
        if (mid_probe.result.transmission.sigma_min_j > target_sigma_min_j) {
            low = mid;
        } else {
            high = mid;
        }
    }
    return high;
}

BoundarySamples PrepareBoundarySamples(const couple_joint::MechanismSolver& origin_solver) {
    double boundary_outside_magnitude = 0.0;
    const double boundary_inside_magnitude =
        FindBoundaryMagnitude(origin_solver, kSearchTolerance, &boundary_outside_magnitude);
    const double sigma_0_2_magnitude =
        FindSigmaTargetMagnitude(origin_solver, 0.2, boundary_inside_magnitude, kSearchTolerance);
    const double sigma_0_1_magnitude =
        FindSigmaTargetMagnitude(origin_solver, 0.1, boundary_inside_magnitude, kSearchTolerance);

    BoundarySamples samples;
    samples.origin = ProbeInverse(origin_solver, 0.0, 0.0);
    samples.sigma_0_2 = ProbeInverse(origin_solver, -sigma_0_2_magnitude, 0.0);
    samples.sigma_0_1 = ProbeInverse(origin_solver, -sigma_0_1_magnitude, 0.0);
    samples.boundary_inside = ProbeInverse(origin_solver, -boundary_inside_magnitude, 0.0);
    samples.boundary_outside = ProbeInverse(origin_solver, -boundary_outside_magnitude, 0.0);

    Require(samples.sigma_0_2.usable, "sigma_min_j = 0.2 point is not usable");
    Require(samples.sigma_0_1.usable, "sigma_min_j = 0.1 point is not usable");
    Require(samples.boundary_inside.usable, "boundary inside point is not usable");
    Require(!samples.boundary_outside.usable, "boundary outside point is still usable");
    return samples;
}

void RunQStepBenchmarks(const couple_joint::MechanismSolver& origin_solver) {
    std::cout << "[suite] q-step\n";
    PrintNanobenchHeader();
    std::vector<InverseSummaryRow> summary_rows;
    summary_rows.reserve(kFrequencySamplesHz.size());
    for (const int frequency_hz : kFrequencySamplesHz) {
        const double delta_q = kAngularSpeed / static_cast<double>(frequency_hz);
        const double target_roll = -delta_q;
        const double target_pitch = 0.0;
        const InverseProbe inverse_probe = ProbeInverse(origin_solver, target_roll, target_pitch);
        Require(inverse_probe.usable, "q-step inverse probe is not usable");

        const ForwardProbe forward_roundtrip =
            ProbeForward(origin_solver, inverse_probe.state.theta1, inverse_probe.state.theta2);
        Require(forward_roundtrip.usable, "q-step roundtrip forward probe is not usable");
        const double roundtrip_error =
            Norm2(forward_roundtrip.state.roll - target_roll, forward_roundtrip.state.pitch - target_pitch);
        summary_rows.push_back({"q-step " + std::to_string(frequency_hz) + "Hz", inverse_probe, roundtrip_error});

        MakeBench().run("q-step " + std::to_string(frequency_hz) + "Hz", [&] {
            couple_joint::MechanismSolver solver = origin_solver;
            const couple_joint::SolveResult result = solver.SolveInverse(target_roll, target_pitch);
            ConsumeSolveResult(result, solver.GetState());
        });
    }
    PrintInverseSummaryTable("q-step", summary_rows);
}

void RunThetaStepBenchmarks(const couple_joint::MechanismSolver& origin_solver) {
    std::cout << "[suite] theta-step\n";
    PrintNanobenchHeader();
    const couple_joint::MechanismState theta_origin = origin_solver.GetState();
    std::vector<ForwardSummaryRow> summary_rows;
    summary_rows.reserve(kFrequencySamplesHz.size());
    for (const int frequency_hz : kFrequencySamplesHz) {
        const double delta_theta = kAngularSpeed / static_cast<double>(frequency_hz);
        const double target_theta1 = theta_origin.theta1 + delta_theta;
        const double target_theta2 = theta_origin.theta2 + delta_theta;
        const ForwardProbe forward_probe = ProbeForward(origin_solver, target_theta1, target_theta2);
        Require(forward_probe.usable, "theta-step forward probe is not usable");

        const InverseProbe inverse_roundtrip =
            ProbeInverse(origin_solver, forward_probe.state.roll, forward_probe.state.pitch);
        Require(inverse_roundtrip.usable, "theta-step roundtrip inverse probe is not usable");
        const double roundtrip_error =
            Norm2(inverse_roundtrip.state.theta1 - target_theta1, inverse_roundtrip.state.theta2 - target_theta2);
        summary_rows.push_back({"theta-step " + std::to_string(frequency_hz) + "Hz", forward_probe, roundtrip_error});

        MakeBench().run("theta-step " + std::to_string(frequency_hz) + "Hz", [&] {
            couple_joint::MechanismSolver solver = origin_solver;
            const couple_joint::SolveResult result = solver.SolveForward(target_theta1, target_theta2);
            ConsumeSolveResult(result, solver.GetState());
        });
    }
    PrintForwardSummaryTable("theta-step", summary_rows);
}

void RunBoundaryBenchmarks(const couple_joint::MechanismSolver& origin_solver) {
    std::cout << "[suite] boundary\n";
    PrintNanobenchHeader();
    const BoundarySamples samples = PrepareBoundarySamples(origin_solver);
    std::vector<InverseSummaryRow> inverse_summary_rows;
    inverse_summary_rows.reserve(5);

    const InverseProbe inverse_cases[] = {
        samples.origin, samples.sigma_0_2, samples.sigma_0_1, samples.boundary_inside, samples.boundary_outside,
    };
    const char* inverse_labels[] = {
        "boundary inverse origin",   "boundary inverse sigma0.2", "boundary inverse sigma0.1",
        "boundary inverse boundary", "boundary inverse outside",
    };
    for (size_t index = 0; index < std::size(inverse_cases); ++index) {
        const InverseProbe& probe = inverse_cases[index];
        inverse_summary_rows.push_back({inverse_labels[index], probe, probe.usable ? 0.0 : -1.0});
        MakeBench().run(inverse_labels[index], [&, roll = probe.roll, pitch = probe.pitch] {
            couple_joint::MechanismSolver solver = origin_solver;
            const couple_joint::SolveResult result = solver.SolveInverse(roll, pitch);
            ConsumeSolveResult(result, solver.GetState());
        });
    }

    std::vector<ForwardSummaryRow> forward_summary_rows;
    forward_summary_rows.reserve(4);
    const InverseProbe reachable_inverse_cases[] = {
        samples.origin,
        samples.sigma_0_2,
        samples.sigma_0_1,
        samples.boundary_inside,
    };
    const char* forward_labels[] = {
        "boundary forward origin",
        "boundary forward sigma0.2",
        "boundary forward sigma0.1",
        "boundary forward boundary",
    };
    for (size_t index = 0; index < std::size(reachable_inverse_cases); ++index) {
        const InverseProbe& inverse_probe = reachable_inverse_cases[index];
        const ForwardProbe forward_probe =
            ProbeForward(origin_solver, inverse_probe.state.theta1, inverse_probe.state.theta2);
        Require(forward_probe.usable, std::string(forward_labels[index]) + " probe is not usable");
        const double roundtrip_error =
            Norm2(forward_probe.state.roll - inverse_probe.roll, forward_probe.state.pitch - inverse_probe.pitch);
        forward_summary_rows.push_back({forward_labels[index], forward_probe, roundtrip_error});

        MakeBench().run(forward_labels[index],
                        [&, theta1 = inverse_probe.state.theta1, theta2 = inverse_probe.state.theta2] {
                            couple_joint::MechanismSolver solver = origin_solver;
                            const couple_joint::SolveResult result = solver.SolveForward(theta1, theta2);
                            ConsumeSolveResult(result, solver.GetState());
                        });
    }

    std::cout << "[boundary-search] origin_roll=" << FormatDouble(samples.origin.roll)
              << " sigma0.2_roll=" << FormatDouble(samples.sigma_0_2.roll)
              << " sigma0.1_roll=" << FormatDouble(samples.sigma_0_1.roll)
              << " boundary_inside_roll=" << FormatDouble(samples.boundary_inside.roll)
              << " boundary_outside_roll=" << FormatDouble(samples.boundary_outside.roll) << "\n";
    PrintInverseSummaryTable("boundary", inverse_summary_rows);
    PrintForwardSummaryTable("boundary", forward_summary_rows);
}

}  // namespace

int main() {
    try {
        PrimeNanobenchHeaderState();
        const couple_joint::MechanismSolver origin_solver = CreateOriginSolver();
        RunQStepBenchmarks(origin_solver);
        RunThetaStepBenchmarks(origin_solver);
        RunBoundaryBenchmarks(origin_solver);
        return EXIT_SUCCESS;
    } catch (const std::exception& exception) {
        std::cerr << "mechanism_solver_bench failed: " << exception.what() << "\n";
        return EXIT_FAILURE;
    }
}
