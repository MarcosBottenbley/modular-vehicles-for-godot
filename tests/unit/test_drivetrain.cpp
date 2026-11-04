#include "test_helpers.hpp"

#include "math/mv_drivetrain.hpp"

using namespace mv;

// Golden values: tests/unit/golden/golden_values.txt, [drivetrain].
namespace {
// vehicle.gd Gearbox / Motor defaults.
constexpr Real kGearRatios[] = { R(3.8), R(2.3), R(1.7), R(1.3), R(1.0), R(0.8) };
constexpr std::uint32_t kGearCount = 6;
constexpr Real kFinalDrive = R(3.2);
constexpr Real kReverseRatio = R(3.3);
constexpr Real kMaxTorque = 300;
constexpr Real kMaxRpm = 7000;
constexpr Real kMotorMoment = R(0.5);
constexpr Real kDelta = R(1.0 / 120.0);

// The demo torque curve (arcade_car.tscn Curve_bt7kc) sampled at 128 points by the golden script.
const TorqueTable kDemoTable = { { R(0.4945050), R(0.5104434), R(0.5260566), R(0.5413478), R(0.5563202), R(0.5709769), R(0.5853211), R(0.5993556), R(0.6130840), R(0.6265092), R(0.6396344), R(0.6524625), R(0.6649972), R(0.6772411), R(0.6891975), R(0.7008696), R(0.7122604), R(0.7233732), R(0.7342111), R(0.7447772), R(0.7550746), R(0.7651064), R(0.7748758), R(0.7843860), R(0.7936401), R(0.8026415), R(0.8113928), R(0.8198974), R(0.8281584), R(0.8361790), R(0.8439622), R(0.8515114), R(0.8588296), R(0.8659198), R(0.8727854), R(0.8794293), R(0.8858548), R(0.8920649), R(0.8980628), R(0.9038516), R(0.9094346), R(0.9148147), R(0.9199953), R(0.9249793), R(0.9297699), R(0.9343702), R(0.9387835), R(0.9430128), R(0.9470613), R(0.9509321), R(0.9546283), R(0.9581532), R(0.9615097), R(0.9647012), R(0.9677305), R(0.9706011), R(0.9733158), R(0.9758781), R(0.9782908), R(0.9805573), R(0.9826804), R(0.9846636), R(0.9865099), R(0.9882225), R(0.9898043), R(0.9912587), R(0.9925886), R(0.9937975), R(0.9948882), R(0.9958640), R(0.9967278), R(0.9974831), R(0.9981328), R(0.9986801), R(0.9991282), R(0.9994800), R(0.9997389), R(0.9999080), R(0.9999903), R(0.9999596), R(0.9996533), R(0.9990482), R(0.9981469), R(0.9969519), R(0.9954658), R(0.9936911), R(0.9916304), R(0.9892861), R(0.9866608), R(0.9837573), R(0.9805779), R(0.9771252), R(0.9734018), R(0.9694101), R(0.9651527), R(0.9606323), R(0.9558513), R(0.9508123), R(0.9455180), R(0.9399705), R(0.9341727), R(0.9281272), R(0.9218364), R(0.9153030), R(0.9085293), R(0.9015179), R(0.8942715), R(0.8867927), R(0.8790837), R(0.8711474), R(0.8629862), R(0.8546026), R(0.8459994), R(0.8371788), R(0.8281435), R(0.8188961), R(0.8094392), R(0.7997752), R(0.7899067), R(0.7798362), R(0.7695662), R(0.7590995), R(0.7484385), R(0.7375857), R(0.7265437), R(0.7153150), R(0.7039023), R(0.6923080) } };

TorqueTable flat_table(Real p_value) {
	TorqueTable table;
	table.fill(p_value);
	return table;
}
} // namespace

TEST_CASE("mv_drivetrain: unit constants") {
	CHECK(kAngularVelocityToRpm == approx(60.0 / 6.283185307179586));
	CHECK(angular_velocity_to_rpm(rpm_to_angular_velocity(4325)) == approx(4325.0));
	CHECK(rpm_to_angular_velocity(60) == approx(kTau));
	CHECK(kTorqueTableSize == 128);
}

TEST_CASE("mv_drivetrain: gear_ratio matches vehicle.gd get_gear_ratio()") {
	CHECK(gear_ratio(-1, kGearRatios, kGearCount, kFinalDrive, kReverseRatio) == approx(-10.56));
	CHECK(gear_ratio(0, kGearRatios, kGearCount, kFinalDrive, kReverseRatio) == approx(0.0));
	CHECK(gear_ratio(1, kGearRatios, kGearCount, kFinalDrive, kReverseRatio) == approx(12.16));
	CHECK(gear_ratio(2, kGearRatios, kGearCount, kFinalDrive, kReverseRatio) == approx(7.36));
	CHECK(gear_ratio(3, kGearRatios, kGearCount, kFinalDrive, kReverseRatio) == approx(5.44));
	CHECK(gear_ratio(4, kGearRatios, kGearCount, kFinalDrive, kReverseRatio) == approx(4.16));
	CHECK(gear_ratio(5, kGearRatios, kGearCount, kFinalDrive, kReverseRatio) == approx(3.2));
	CHECK(gear_ratio(6, kGearRatios, kGearCount, kFinalDrive, kReverseRatio) == approx(2.56));
	SUBCASE("out-of-range gears and an empty list return 0 instead of reading past the array") {
		CHECK(gear_ratio(7, kGearRatios, kGearCount, kFinalDrive, kReverseRatio) == 0);
		CHECK(gear_ratio(-2, kGearRatios, kGearCount, kFinalDrive, kReverseRatio) == 0);
		CHECK(gear_ratio(1, kGearRatios, 0, kFinalDrive, kReverseRatio) == 0);
		CHECK(gear_ratio(1, nullptr, kGearCount, kFinalDrive, kReverseRatio) == 0);
	}
	SUBCASE("a single-speed gearbox (N = 1)") {
		const Real single[] = { 4 };
		CHECK(gear_ratio(1, single, 1, 2, 3) == approx(8.0));
		CHECK(gear_ratio(2, single, 1, 2, 3) == 0);
		CHECK(gear_ratio(-1, single, 1, 2, 3) == approx(-6.0));
	}
}

TEST_CASE("mv_drivetrain: sample_torque_table interpolates a baked table") {
	SUBCASE("a flat table reproduces a null curve") {
		const TorqueTable ones = flat_table(1);
		for (Real x : { R(0.0), R(0.001), R(0.37), R(0.5), R(0.999), R(1.0), R(1.5), R(-1.0) }) {
			CHECK(sample_torque_table(ones, x) == approx(1.0));
		}
		CHECK(torque_at_rpm(3500, kMaxRpm, kMaxTorque, ones) == approx(300.0));
	}
	SUBCASE("knots are returned exactly and midpoints are linear") {
		TorqueTable ramp;
		for (std::uint32_t i = 0; i < kTorqueTableSize; ++i) {
			ramp[i] = static_cast<Real>(i);
		}
		CHECK(sample_torque_table(ramp, 0) == approx(0.0));
		CHECK(sample_torque_table(ramp, 1) == approx(127.0));
		CHECK(sample_torque_table(ramp, R(1.0 / 127.0)) == approx(1.0));
		CHECK(sample_torque_table(ramp, R(0.5)) == approx(63.5));
		CHECK(sample_torque_table(ramp, R(10.5 / 127.0)) == approx(10.5));
	}
	SUBCASE("out-of-range positions clamp to the ends") {
		CHECK(sample_torque_table(kDemoTable, -2) == approx(kDemoTable[0]));
		CHECK(sample_torque_table(kDemoTable, 2) == approx(kDemoTable[kTorqueTableSize - 1]));
	}
}

TEST_CASE("mv_drivetrain: torque_at_rpm stays within 1% of vehicle.gd get_torque_at_rpm() on the demo curve") {
	// Right-hand side: Curve.sample_baked(clamp(rpm / 7000, 0, 1)) * 300 from the golden script.
	const struct {
		Real rpm;
		double torque;
	} cases[] = {
		{ 0, 148.3514964581 },
		{ 500, 188.2321000099 },
		{ 1000, 220.7129359245 },
		{ 1500, 246.4963138103 },
		{ 2500, 280.7797908783 },
		{ 3500, 296.7008829117 },
		{ 4325, 299.9910414219 },
		{ 5000, 293.4281587601 },
		{ 5500, 280.6204855442 },
		{ 6000, 261.6816580296 },
		{ 6500, 237.1821820736 },
		{ 7000, 207.6924026012 },
		{ 7700, 207.6924026012 }, // beyond max_rpm: clamped like the GDScript
	};
	for (const auto &c : cases) {
		CAPTURE(c.rpm);
		CHECK(torque_at_rpm(c.rpm, kMaxRpm, kMaxTorque, kDemoTable) == approx(c.torque, 0.01));
		CHECK(torque_factor_at_rpm(c.rpm, kMaxRpm, kDemoTable) == approx(c.torque / 300.0, 0.01));
	}
	SUBCASE("non-positive max_rpm falls back to the first table entry") {
		CHECK(torque_at_rpm(3000, 0, kMaxTorque, kDemoTable) == approx(kDemoTable[0] * 300.0));
	}
}

TEST_CASE("mv_drivetrain: motor helpers match vehicle.gd process_motor()") {
	const Real drag = motor_drag_torque(4325, R(0.005));
	CHECK(drag == approx(21.625));
	const Real curve_torque = torque_at_rpm(4325, kMaxRpm, kMaxTorque, kDemoTable);
	CHECK(curve_torque == approx(299.9910414219, 0.01));
	// Use the GDScript's exact curve torque so the comparison is about the formula, not the table.
	const Real golden_curve_torque = R(299.9910414219);
	CHECK(motor_torque_output(golden_curve_torque, R(0.7), drag, 0) == approx(188.3687289953));
	CHECK(motor_torque_output(golden_curve_torque, R(0.2), drag, 1) == approx(21.0732082844));
	CHECK(motor_torque_output(golden_curve_torque, 0, drag, 1) == approx(-43.25));
	CHECK(motor_rpm_delta(150, kMotorMoment, kDelta) == approx(23.8732414638));
	CHECK(motor_rpm_delta(150, 0, kDelta) == 0);
}

TEST_CASE("mv_drivetrain: drive_inertia and clutch_torque match vehicle.gd process_clutch()") {
	const Real gear_inertia = R(0.02);
	const Real drive_axles_inertia = 2 * (R(0.5) * 15 * R(0.3) * R(0.3));
	const Real max_clutch = max_clutch_torque(kMaxTorque, R(1.6));
	CHECK(max_clutch == approx(480.0));

	const Real ratio_2 = gear_ratio(2, kGearRatios, kGearCount, kFinalDrive, kReverseRatio);
	const Real inertia_2 = drive_inertia(kMotorMoment, ratio_2, gear_inertia, drive_axles_inertia);
	CHECK(inertia_2 == approx(2.933392));
	CHECK(drive_inertia(kMotorMoment, ratio_2, gear_inertia, 0) == approx(1.583392));

	SUBCASE("motor spinning faster than the wheels: clamped to the clutch capacity") {
		CHECK(clutch_torque(kMotorMoment, inertia_2, ratio_2, rpm_to_angular_velocity(4325), 40, 120, 180, 0, max_clutch, kDelta) == approx(480.0));
	}
	SUBCASE("overrun: negative speed difference takes the -sqrt branch") {
		CHECK(clutch_torque(kMotorMoment, inertia_2, ratio_2, rpm_to_angular_velocity(2000), 60, -30, -8, 0, max_clutch, kDelta) == approx(-86.4409145434));
	}
	SUBCASE("half-pressed pedal halves the clamp") {
		CHECK(clutch_torque(kMotorMoment, inertia_2, ratio_2, rpm_to_angular_velocity(4325), 40, 120, 180, R(0.5), max_clutch, kDelta) == approx(240.0));
	}
	SUBCASE("nearly locked drivetrain") {
		const Real spin = rpm_to_angular_velocity(3000) / ratio_2 + R(0.5);
		CHECK(clutch_torque(kMotorMoment, inertia_2, ratio_2, rpm_to_angular_velocity(3000), spin, 90, 150, 0, max_clutch, kDelta) == approx(-7.6228605564, 1e-3));
	}
	SUBCASE("reverse gear") {
		const Real ratio_r = gear_ratio(-1, kGearRatios, kGearCount, kFinalDrive, kReverseRatio);
		const Real inertia_r = drive_inertia(kMotorMoment, ratio_r, gear_inertia, drive_axles_inertia);
		CHECK(clutch_torque(kMotorMoment, inertia_r, ratio_r, rpm_to_angular_velocity(1500), -5, -40, 60, 0, max_clutch, kDelta) == approx(427.2096108504));
	}
	SUBCASE("neutral, fully pressed pedal or zero delta give no clutch torque") {
		CHECK(clutch_torque(kMotorMoment, inertia_2, 0, rpm_to_angular_velocity(4325), 40, 120, 180, 0, max_clutch, kDelta) == 0);
		CHECK(clutch_torque(kMotorMoment, inertia_2, ratio_2, rpm_to_angular_velocity(4325), 40, 120, 180, 1, max_clutch, kDelta) == approx(0.0));
		CHECK(clutch_torque(kMotorMoment, inertia_2, ratio_2, rpm_to_angular_velocity(4325), 40, 120, 180, 0, max_clutch, 0) == 0);
	}
}
