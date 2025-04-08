#include "fff.h"
#include <gtest/gtest.h>

extern "C" {
// add includes like freertos, hal, proc headers, etc
#include "application/estimator/estimator_types.h"
#include "application/estimator/pad_filter.h"
DEFINE_FFF_GLOBALS; // this must be called within the extern c block
}

// IMPORTANT: when running the matlab pad_filter in cmdline, remember to
// clear the persistent variables `clear pad_filter`

class PadFilterTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Reset all fakes before each test, for example:
        // RESET_FAKE(xQueueCreate);
        FFF_RESET_HISTORY();
    }

    void TearDown() override {}
};

/**
global IMU_select;
IMU_select = [1 1];
IMU_1 = [0.01; 0.02; -9.81; 0.001; -0.002; 0.0005; 0.3; 0.0; 0.4; 1013.25];
IMU_2 = [-0.02; 0.01; -9.78; 0.0005; 0.001; -0.001; 0.31; -0.01; 0.39; 1013.30];
[x_init, bias_1, bias_2] = pad_filter(IMU_1, IMU_2);

x_init[] = {0.813804, 0.185759, 0.403873, -0.374304, 0.000000, 0.000000, 0.000000, 0.000000,
0.000000, 0.000000, 35009.193288, 5.000000, 0.000000, }; bias_1[] = {0.000000, 0.000000,
0.000000, 3.960110, 4.950105, 5.940155, 13.362314, -3.419518, -0.130656, 0.000000, }; bias_2[] =
{0.000000, 0.000000, 0.000000, 13.860355, 14.850385, 15.840390, 29.906717, -7.509157, -2.028143,
0.000000, };
 */
TEST(PadFilterTest, NewContextOneIteration) {
    // Arrange
    pad_filter_ctx_t test_ctx = {0};

    y_imu_t imu1 = {
        .array = {0.01f, 0.02f, -9.81f, 0.001f, -0.002f, 0.0005f, 0.3f, 0.0f, 0.4f, 1013.25f}
    };

    y_imu_t imu2 = {
        .array = {-0.02f, 0.01f, -9.78f, 0.0005f, 0.001f, -0.001f, 0.31f, -0.01f, 0.39f, 1013.30f}
    };

    bool actual_is_dead_1 = false;
    bool actual_is_dead_2 = false;
    y_imu_t actual_bias_1 = {0};
    y_imu_t actual_bias_2 = {0};
    x_state_t actual_x_init = {0};

    // Act
    pad_filter(
        &test_ctx,
        &imu1,
        &imu2,
        actual_is_dead_1,
        actual_is_dead_2,
        &actual_x_init,
        &actual_bias_1,
        &actual_bias_2
    );

    // Assert
    x_state_t expect_x_init = {
        .array = {
            0.573781f,
            -0.413347f,
            0.573488f,
            0.413558f,
            0.0f,
            0.0f,
            0.0f,
            0.0f,
            0.0f,
            0.0f,
            25762.842421f,
            5.0f,
            0.0f
        }
    };

    y_imu_t expect_bias_1 = {
        .array = {
            0.0f,
            0.0f,
            0.0f,
            0.001000f,
            -0.002000f,
            0.000500f,
            0.126540f,
            0.379619f,
            -0.299796f,
            0.0f
        }
    };

    y_imu_t expect_bias_2 = {
        .array = {
            0.0f,
            0.0f,
            0.0f,
            0.000500f,
            0.001000f,
            -0.001000f,
            0.132866f,
            0.366974f,
            -0.309801f,
            0.0f
        }
    };

    // tolerance = 0.001% of the actual value
    // accounts for floats being ~6 significant digits of precision
    float tolerance = 0.00001f;

    for (int i = 0; i < 13; ++i) {
        EXPECT_NEAR(
            actual_x_init.array[i], expect_x_init.array[i], abs(expect_x_init.array[i] * tolerance)
        );
    }

    for (int i = 0; i < 10; ++i) {
        EXPECT_NEAR(
            actual_bias_1.array[i], expect_bias_1.array[i], abs(expect_bias_1.array[i] * tolerance)
        );
        EXPECT_NEAR(
            actual_bias_2.array[i], expect_bias_2.array[i], abs(expect_bias_2.array[i] * tolerance)
        );
    }
}

TEST(PadFilterTest, RunsThreeSequentialIterationsCorrectly) {
    // Arrange
    pad_filter_ctx_t test_ctx = {0};
    // tolerance is evaled as 0.5% of the given expect value
    float tolerance = 0.005f;

    // --- Iteration 1 ---
    {
        y_imu_t imu1 = {
            .array = {
                0.009697f,
                0.020294f,
                -9.810787f,
                0.001888f,
                -0.003147f,
                -0.000569f,
                0.299191f,
                -0.002944f,
                0.401438f,
                1013.250325f
            }
        };
        y_imu_t imu2 = {
            .array = {
                -0.020755f,
                0.011370f,
                -9.781712f,
                0.000398f,
                0.000759f,
                -0.000681f,
                0.310313f,
                -0.010865f,
                0.389970f,
                1013.299835f
            }
        };

        x_state_t expect_x = {
            .array = {
                0.576728f,
                -0.409241f,
                0.576403f,
                0.409472f,
                0.0f,
                0.0f,
                0.0f,
                0.0f,
                0.0f,
                0.0f,
                25762.842145f,
                5.0f,
                0.0f
            }
        };
        y_imu_t expect_bias_1 = {
            .array = {
                0.0f,
                0.0f,
                0.0f,
                0.001888f,
                -0.003147f,
                -0.000569f,
                0.135194f,
                0.378179f,
                -0.298964f,
                0.0f
            }
        };
        y_imu_t expect_bias_2 = {
            .array = {
                0.0f,
                0.0f,
                0.0f,
                0.000398f,
                0.000759f,
                -0.000681f,
                0.138892f,
                0.364747f,
                -0.310093f,
                0.0f
            }
        };

        x_state_t actual_x = {0};
        y_imu_t actual_bias_1 = {0};
        y_imu_t actual_bias_2 = {0};

        pad_filter(
            &test_ctx, &imu1, &imu2, false, false, &actual_x, &actual_bias_1, &actual_bias_2
        );

        for (int i = 0; i < 13; ++i) {
            EXPECT_NEAR(actual_x.array[i], expect_x.array[i], fabs(expect_x.array[i] * tolerance));
        }
        for (int i = 0; i < 10; ++i) {
            EXPECT_NEAR(
                actual_bias_1.array[i],
                expect_bias_1.array[i],
                fabs(expect_bias_1.array[i] * tolerance)
            );
            EXPECT_NEAR(
                actual_bias_2.array[i],
                expect_bias_2.array[i],
                fabs(expect_bias_2.array[i] * tolerance)
            );
        }
    }

    // --- Iteration 2 ---
    {
        y_imu_t imu1 = {
            .array = {
                0.010628f,
                0.021093f,
                -9.808891f,
                0.000136f,
                -0.001923f,
                -0.000714f,
                0.298886f,
                -0.000007f,
                0.401533f,
                1013.249230f
            }
        };
        y_imu_t imu2 = {
            .array = {
                -0.019629f,
                0.009774f,
                -9.778883f,
                -0.000589f,
                0.001033f,
                -0.000447f,
                0.311101f,
                -0.008456f,
                0.390086f,
                1013.298508f
            }
        };

        x_state_t expect_x = {
            .array = {
                0.576677f,
                -0.409313f,
                0.576352f,
                0.409544f,
                0.0f,
                0.0f,
                0.0f,
                0.0f,
                0.0f,
                0.0f,
                25762.842166f,
                5.0f,
                0.0f
            }
        };
        y_imu_t expect_bias_1 = {
            .array = {
                0.0f,
                0.0f,
                0.0f,
                0.001880f,
                -0.003141f,
                -0.000570f,
                0.135085f,
                0.378218f,
                -0.298963f,
                0.0f
            }
        };
        y_imu_t expect_bias_2 = {
            .array = {
                0.0f,
                0.0f,
                0.0f,
                0.000393f,
                0.000760f,
                -0.000680f,
                0.138790f,
                0.364786f,
                -0.310097f,
                0.0f
            }
        };

        x_state_t actual_x = {0};
        y_imu_t actual_bias_1 = {0};
        y_imu_t actual_bias_2 = {0};

        pad_filter(
            &test_ctx, &imu1, &imu2, false, false, &actual_x, &actual_bias_1, &actual_bias_2
        );

        for (int i = 0; i < 13; ++i) {
            EXPECT_NEAR(actual_x.array[i], expect_x.array[i], fabs(expect_x.array[i] * tolerance));
        }
        for (int i = 0; i < 10; ++i) {
            EXPECT_NEAR(
                actual_bias_1.array[i],
                expect_bias_1.array[i],
                fabs(expect_bias_1.array[i] * tolerance)
            );
            EXPECT_NEAR(
                actual_bias_2.array[i],
                expect_bias_2.array[i],
                fabs(expect_bias_2.array[i] * tolerance)
            );
        }
    }

    // --- Iteration 3 ---
    {
        y_imu_t imu1 = {
            .array = {
                0.009258f,
                0.018938f,
                -9.807650f,
                0.000384f,
                -0.001252f,
                0.000308f,
                0.300889f,
                -0.000765f,
                0.398598f,
                1013.248578f
            }
        };
        y_imu_t imu2 = {
            .array = {
                -0.019512f,
                0.009823f,
                -9.780196f,
                0.001919f,
                0.001292f,
                -0.000802f,
                0.311588f,
                -0.010804f,
                0.390697f,
                1013.300835f
            }
        };

        x_state_t expect_x = {
            .array = {
                0.576683f,
                -0.409304f,
                0.576358f,
                0.409535f,
                0.0f,
                0.0f,
                0.0f,
                0.0f,
                0.0f,
                0.0f,
                25762.842172f,
                5.0f,
                0.0f
            }
        };
        y_imu_t expect_bias_1 = {
            .array = {
                0.0f,
                0.0f,
                0.0f,
                0.001872f,
                -0.003132f,
                -0.000565f,
                0.135082f,
                0.378204f,
                -0.298971f,
                0.0f
            }
        };
        y_imu_t expect_bias_2 = {
            .array = {
                0.0f,
                0.0f,
                0.0f,
                0.000400f,
                0.000763f,
                -0.000680f,
                0.138802f,
                0.364785f,
                -0.310103f,
                0.0f
            }
        };

        x_state_t actual_x = {0};
        y_imu_t actual_bias_1 = {0};
        y_imu_t actual_bias_2 = {0};

        pad_filter(
            &test_ctx, &imu1, &imu2, false, false, &actual_x, &actual_bias_1, &actual_bias_2
        );

        for (int i = 0; i < 13; ++i) {
            EXPECT_NEAR(actual_x.array[i], expect_x.array[i], fabs(expect_x.array[i] * tolerance));
        }
        for (int i = 0; i < 10; ++i) {
            EXPECT_NEAR(
                actual_bias_1.array[i],
                expect_bias_1.array[i],
                fabs(expect_bias_1.array[i] * tolerance)
            );
            EXPECT_NEAR(
                actual_bias_2.array[i],
                expect_bias_2.array[i],
                fabs(expect_bias_2.array[i] * tolerance)
            );
        }
    }
}
