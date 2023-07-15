#include <gtest/gtest.h>

#include <motion_planning/PositionSmoothing.hpp>


static constexpr float MAX_JERK = 4.f;
static constexpr float MAX_ACCELERATION = 3.f;

static constexpr float MAX_ALLOWED_HOR_ERR = 2.f;
static constexpr float VERTICAL_ACCEPTANCE_RADIUS = 0.8f;
static constexpr float CRUISE_SPEED = 5.f;
static constexpr float MAX_VELOCITY = CRUISE_SPEED;

static constexpr float HORIZONTAL_TRAJECTORY_GAIN = 0.5f;
static constexpr float TARGET_ACCEPTANCE_RADIUS = 0.5f;


class PositionSmoothingTest : public ::testing::Test
{

public:
	PositionSmoothing _position_smoothing;

	PositionSmoothingTest()
	{
		_position_smoothing.setMaxJerk({MAX_JERK, MAX_JERK, MAX_JERK});
		_position_smoothing.setMaxAcceleration({MAX_ACCELERATION, MAX_ACCELERATION, MAX_ACCELERATION});
		_position_smoothing.setMaxVelocity({MAX_VELOCITY, MAX_VELOCITY, MAX_VELOCITY});
		_position_smoothing.setMaxAllowedHorizontalError(MAX_ALLOWED_HOR_ERR);
		_position_smoothing.setVerticalAcceptanceRadius(VERTICAL_ACCEPTANCE_RADIUS);
		_position_smoothing.setCruiseSpeed(CRUISE_SPEED);
		_position_smoothing.setHorizontalTrajectoryGain(HORIZONTAL_TRAJECTORY_GAIN);
		_position_smoothing.setTargetAcceptanceRadius(TARGET_ACCEPTANCE_RADIUS);

		_position_smoothing.reset({0.f, 0.f, 0.f}, {0.f, 0.f, 0.f}, {0.f, 0.f, 0.f});
	}

	static bool _vectorNear(const Vector3f &a, const Vector3f &b, float EPS = 1e-4f)
	{
		return (fabsf(a(0) - b(0)) < EPS) && (fabsf(a(1) - b(1)) < EPS) && (fabsf(a(2) - b(2)) < EPS);
	}

	static void expectVectorEqual(const Vector3f &expected, const Vector3f &value, const char *name, float EPS)
	{
		EXPECT_TRUE(_vectorNear(expected, value, EPS)) <<
				"Vector \"" << name << "\" expected [" <<
				expected(0) << ", " <<
				expected(1) << ", " <<
				expected(2) << "] has value [" <<
				value(0) << ", " <<
				value(1) << ", " <<
				value(2) << "]\n";
	}

	static void expectDynamicsLimitsRespected(const PositionSmoothing::PositionSmoothingSetpoints &setpoints)
	{
		EXPECT_LE(fabsf(setpoints.velocity(0)), MAX_VELOCITY) << "Velocity in x too high\n";
		EXPECT_LE(fabsf(setpoints.velocity(1)), MAX_VELOCITY) << "Velocity in y too high\n";
		EXPECT_LE(fabsf(setpoints.velocity(2)), MAX_VELOCITY) << "Velocity in z too high\n";
		EXPECT_LE(fabsf(setpoints.acceleration(0)), MAX_ACCELERATION) << "Acceleration in x too high\n";
		EXPECT_LE(fabsf(setpoints.acceleration(1)), MAX_ACCELERATION) << "Acceleration in y too high\n";
		EXPECT_LE(fabsf(setpoints.acceleration(2)), MAX_ACCELERATION) << "Acceleration in z too high\n";
		EXPECT_LE(fabsf(setpoints.jerk(0)), MAX_JERK) << "Jerk in x too high\n";
		EXPECT_LE(fabsf(setpoints.jerk(1)), MAX_JERK) << "Jerk in y too high\n";
		EXPECT_LE(fabsf(setpoints.jerk(2)), MAX_JERK) << "Jerk in z too high\n";
	}
};


TEST_F(PositionSmoothingTest, reachesTargetPositionSetpoint)
{
	const float EPS = 1e-4;
	const int N_ITER = 2000;
	const float DELTA_T = 0.02f;
	const Vector3f INITIAL_POSITION{0.f, 0.f, 0.f};
	const Vector3f FF_VELOCITY{0.f, 0.f, 0.f};
	const Vector3f TARGET{12.f, 17.f, 8.f};

	Vector3f waypoints[3] = {INITIAL_POSITION, TARGET, TARGET};

	Vector3f position{0.f, 0.f, 0.f};

	PositionSmoothing::PositionSmoothingSetpoints out;

	int iteration = 0;

	for (; iteration < N_ITER; iteration++) {
		_position_smoothing.generateSetpoints(
			position,
			waypoints,
			FF_VELOCITY,
			DELTA_T,
			false,
			out
		);
		position = out.position;
		expectDynamicsLimitsRespected(out);

		if ((position - TARGET).length() < EPS) {
			printf("Converged in %d iterations\n", iteration);
			break;
		}
	}

	expectVectorEqual(TARGET, position, "position", EPS);
	EXPECT_LT(iteration, N_ITER) << "Took too long to converge\n";
}


TEST_F(PositionSmoothingTest, reachesTargetVelocityIntegration)
{
	const float EPS = 1e-4;
	const int N_ITER = 2000;
	const float DELTA_T = 0.02f;
	const Vector3f INITIAL_POSITION{0.f, 0.f, 0.f};
	const Vector3f FF_VELOCITY{0.f, 0.f, 0.f};
	const Vector3f TARGET{12.f, 17.f, 8.f};

	Vector3f waypoints[3] = {INITIAL_POSITION, TARGET, TARGET};

	Vector3f position{0.f, 0.f, 0.f};

	PositionSmoothing::PositionSmoothingSetpoints out;

	int iteration = 0;

	for (; iteration < N_ITER; iteration++) {
		_position_smoothing.generateSetpoints(
			position,
			waypoints,
			FF_VELOCITY,
			DELTA_T,
			false,
			out
		);
		position += out.velocity * DELTA_T;
		expectDynamicsLimitsRespected(out);


		if ((position - TARGET).length() < EPS) {
			printf("Converged in %d iterations\n", iteration);
			break;
		}
	}

	expectVectorEqual(TARGET, position, "position", EPS);
	EXPECT_LT(iteration, N_ITER) << "Took too long to converge\n";
}


TEST_F(PositionSmoothingTest, reachesTargetInitialVelocity)
{
	const float EPS = 1e-4;
	const int N_ITER = 2000;
	const float DELTA_T = 0.02f;
	const Vector3f INITIAL_POSITION{0.f, 0.f, 0.f};
	const Vector3f TARGET{12.f, 17.f, 8.f};
	const Vector3f NEXT_TARGET{8.f, 12.f, 80.f};


	Vector3f waypoints[3] = {INITIAL_POSITION, TARGET, NEXT_TARGET};
	Vector3f ff_velocity{1.f, 0.1f, 0.3f};

	Vector3f position{0.f, 0.f, 0.f};

	PositionSmoothing::PositionSmoothingSetpoints out;

	int iteration = 0;

	for (; iteration < N_ITER; iteration++) {
		_position_smoothing.generateSetpoints(
			position,
			waypoints,
			ff_velocity,
			DELTA_T,
			false,
			out
		);
		position = out.position;
		ff_velocity = {0.f, 0.f, 0.f};
		expectDynamicsLimitsRespected(out);

		if ((position - TARGET).length() < EPS) {
			printf("Converged in %d iterations\n", iteration);
			break;
		}
	}

	expectVectorEqual(TARGET, position, "position", EPS);
	EXPECT_LT(iteration, N_ITER) << "Took too long to converge\n";
}
