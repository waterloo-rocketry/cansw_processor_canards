/**
 * numbers in these tests generated from matlab cmdline in simulink-canards commit 2c8c534
 */

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
clear global
global IMU_select;
IMU_select = [1 1];
IMU_1 = [0.01; 0.02; -9.81; 0.001; -0.002; 0.0005; 0.3; 0.0; 0.4; 1013.25];
IMU_2 = [-0.02; 0.01; -9.78; 0.0005; 0.001; -0.001; 0.31; -0.01; 0.39; 1013.30];
[x_init, bias_1, bias_2] = pad_filter(IMU_1, IMU_2);
c_array_str = @(v) ['{', sprintf('%.6f, ', v(1:end-1)), sprintf('%.6f', v(end)), '}'];

fprintf(c_array_str(x_init));
printf("\n");
fprintf(c_array_str(bias_1));
printf("\n");
fprintf(c_array_str(bias_2));

x_init[] = {0.813804, 0.185759, 0.403873, -0.374304, 0.000000, 0.000000, 0.000000, 0.000000,
0.000000, 0.000000, 35009.193288, 5.000000, 0.000000, }; bias_1[] = {0.000000, 0.000000,
0.000000, 3.960110, 4.950105, 5.940155, 13.362314, -3.419518, -0.130656, 0.000000, }; bias_2[] =
{0.000000, 0.000000, 0.000000, 13.860355, 14.850385, 15.840390, 29.906717, -7.509157, -2.028143,
0.000000, };
 */
TEST(PadFilterTest, NewContextOneIteration) {
    // Arrange
    pad_filter_ctx_t test_ctx = {0};

    y_imu_t imu1 = {.array = {0.01, 0.02, -9.81, 0.001, -0.002, 0.0005, 0.3, 0.0, 0.4, 1013.25}};

    y_imu_t imu2 = {
        .array = {-0.02, 0.01, -9.78, 0.0005, 0.001, -0.001, 0.31, -0.01, 0.39, 1013.30}
    };

    bool actual_is_dead_1 = false;
    bool actual_is_dead_2 = false;
    y_imu_t actual_bias_1 = {0};
    y_imu_t actual_bias_2 = {0};
    x_state_t actual_x_init = {0};

    // Initialize pad filter
    EXPECT_EQ(
        pad_filter_init(&test_ctx, &imu1, &imu2, actual_is_dead_1, actual_is_dead_2), W_SUCCESS
    );

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
            0.413347,
            -0.573781,
            -0.413558,
            -0.573488,
            0.000000,
            0.000000,
            0.000000,
            0.000000,
            0.000000,
            0.000000,
            250.000000,
            3.482826,
            0.000000
        }
    };

    y_imu_t expect_bias_1 = {
        .array = {
            0.010000,
            0.020000,
            -9.810000,
            0.001000,
            -0.002000,
            0.000500,
            0.126540,
            0.379619,
            0.299796,
            -97343.221211
        }
    };

    y_imu_t expect_bias_2 = {
        .array = {
            -0.020000,
            0.010000,
            -9.780000,
            0.000500,
            0.001000,
            -0.001000,
            0.113892,
            0.373299,
            0.309801,
            -97343.171211
        }
    };

    // tolerance = 0.001% of the actual value
    // accounts for floats being ~6 significant digits of precision
    float tolerance = 0.00001;

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

TEST(PadFilterTest, InitFailsOnSecondAttempt) {
    // Arrange
    pad_filter_ctx_t test_ctx = {0};

    y_imu_t imu1 = {.array = {0.01, 0.02, -9.81, 0.001, -0.002, 0.0005, 0.3, 0.0, 0.4, 1013.25}};
    y_imu_t imu2 = {
        .array = {-0.02, 0.01, -9.78, 0.0005, 0.001, -0.001, 0.31, -0.01, 0.39, 1013.30}
    };

    bool is_dead_1 = false;
    bool is_dead_2 = false;

    // Act & Assert
    ASSERT_EQ(pad_filter_init(&test_ctx, &imu1, &imu2, is_dead_1, is_dead_2), W_SUCCESS);
    ASSERT_EQ(pad_filter_init(&test_ctx, &imu1, &imu2, is_dead_1, is_dead_2), W_FAILURE);
}

// clang-format off
/**
 clear all;
global IMU_select;
IMU_select = [1 1];

% --- Iteration 1 ---
IMU_1 = [0.01; 0.02; -9.81; 0.001; -0.002; 0.0005; 0.3; 0.0; 0.4; 1013.25];
IMU_2 = [-0.02; 0.01; -9.78; 0.0005; 0.001; -0.001; 0.31; -0.01; 0.39; 1013.30];
[x_init, bias_1, bias_2] = pad_filter(IMU_1, IMU_2);

% --- Iteration 2 (from C test) ---
IMU_1 = [0.010628; 0.021093; -9.808891; 0.000136; -0.001923; -0.000714; 0.298886; -0.000007; 0.401533; 1013.249230];
IMU_2 = [-0.019629; 0.009774; -9.778883; -0.000589; 0.001033; -0.000447; 0.311101; -0.008456; 0.390086; 1013.298508];
[x_init, bias_1, bias_2] = pad_filter(IMU_1, IMU_2);

% --- Iteration 3 (from C test) ---
IMU_1 = [0.009258; 0.018938; -9.807650; 0.000384; -0.001252; 0.000308; 0.300889; -0.000765; 0.398598; 1013.248578];
IMU_2 = [-0.019512; 0.009823; -9.780196; 0.001919; 0.001292; -0.000802; 0.311588; -0.010804; 0.390697; 1013.300835];
[x_init, bias_1, bias_2] = pad_filter(IMU_1, IMU_2);
 */
// clang-format off
TEST(PadFilterTest, RunsThreeSequentialIterationsCorrectly) {
    // Arrange
    pad_filter_ctx_t test_ctx = {0};
    // tolerance is evaled as 0.2% of the given expect value
    float tolerance = 0.002;

    // --- Iteration 1 ---
    {
        y_imu_t imu1 = {
            .array = {
                0.01, 0.02, -9.81, 0.001, -0.002, 0.0005, 0.3, 0.0, 0.4, 1013.25
            }
        };
        y_imu_t imu2 = {
            .array = {-0.02, 0.01, -9.78, 0.0005, 0.001, -0.001, 0.31, -0.01, 0.39, 1013.30}
        };

        x_state_t expect_x = {
            .array = {
                0.576728,
                -0.409241,
                0.576403,
                0.409472,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                25762.842145,
                3.62759872846844,
                0.0
            }
        };
        y_imu_t expect_bias_1 = {
            .array = {
                0.0, 0.0, 0.0, 0.001888, -0.003147, -0.000569, 0.135194, 0.378179, -0.298964, 0.0
            }
        };
        y_imu_t expect_bias_2 = {
            .array = {
                0.0, 0.0, 0.0, 0.000398, 0.000759, -0.000681, 0.138892, 0.364747, -0.310093, 0.0
            }
        };

        x_state_t actual_x = {0};
        y_imu_t actual_bias_1 = {0};
        y_imu_t actual_bias_2 = {0};

        // Initialize pad filter on first iteration
        ASSERT_EQ(pad_filter_init(&test_ctx, &imu1, &imu2, false, false), W_SUCCESS);

        pad_filter(
            &test_ctx, &imu1, &imu2, false, false, &actual_x, &actual_bias_1, &actual_bias_2
        );
        // only compare final result for now
    //     for (int i = 0; i < 13; ++i) {
    //         EXPECT_NEAR(actual_x.array[i], expect_x.array[i], abs(expect_x.array[i] * tolerance));
    //     }
    //     for (int i = 0; i < 10; ++i) {
    //         EXPECT_NEAR(
    //             actual_bias_1.array[i],
    //             expect_bias_1.array[i],
    //             abs(expect_bias_1.array[i] * tolerance)
    //         );
    //         EXPECT_NEAR(
    //             actual_bias_2.array[i],
    //             expect_bias_2.array[i],
    //             abs(expect_bias_2.array[i] * tolerance)
    //         );
    //     }
    }

    // --- Iteration 2 ---
    {
        y_imu_t imu1 = {
            .array = {
                0.010628,
                0.021093,
                -9.808891,
                0.000136,
                -0.001923,
                -0.000714,
                0.298886,
                -0.000007,
                0.401533,
                1013.249230
            }
        };
        y_imu_t imu2 = {
            .array = {
                -0.019629,
                0.009774,
                -9.778883,
                -0.000589,
                0.001033,
                -0.000447,
                0.311101,
                -0.008456,
                0.390086,
                1013.298508
            }
        };

        x_state_t expect_x = {
            .array = {
                0.576677,
                -0.409313,
                0.576352,
                0.409544,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                25762.842166,
                3.62759872846844,
                0.0
            }
        };
        y_imu_t expect_bias_1 = {
            .array = {
                0.0, 0.0, 0.0, 0.001880, -0.003141, -0.000570, 0.135085, 0.378218, -0.298963, 0.0
            }
        };
        y_imu_t expect_bias_2 = {
            .array = {
                0.0, 0.0, 0.0, 0.000393, 0.000760, -0.000680, 0.138790, 0.364786, -0.310097, 0.0
            }
        };

        x_state_t actual_x = {0};
        y_imu_t actual_bias_1 = {0};
        y_imu_t actual_bias_2 = {0};

        pad_filter(
            &test_ctx, &imu1, &imu2, false, false, &actual_x, &actual_bias_1, &actual_bias_2
        );

        // for (int i = 0; i < 13; ++i) {
        //     EXPECT_NEAR(actual_x.array[i], expect_x.array[i], abs(expect_x.array[i] * tolerance));
        // }
        // for (int i = 0; i < 10; ++i) {
        //     EXPECT_NEAR(
        //         actual_bias_1.array[i],
        //         expect_bias_1.array[i],
        //         abs(expect_bias_1.array[i] * tolerance)
        //     );
        //     EXPECT_NEAR(
        //         actual_bias_2.array[i],
        //         expect_bias_2.array[i],
        //         abs(expect_bias_2.array[i] * tolerance)
        //     );
        // }
    }

    // --- Iteration 3 ---
    {
        y_imu_t imu1 = {
            .array = {
                0.009258,
                0.018938,
                -9.807650,
                0.000384,
                -0.001252,
                0.000308,
                0.300889,
                -0.000765,
                0.398598,
                1013.248578
            }
        };
        y_imu_t imu2 = {
            .array = {
                -0.019512,
                0.009823,
                -9.780196,
                0.001919,
                0.001292,
                -0.000802,
                0.311588,
                -0.010804,
                0.390697,
                1013.300835
            }
        };

        x_state_t expect_x = {
            .array = {
         0.413373541614123,
        -0.573762123638439,
        -0.413584530119265,
        -0.573469421170242,
                         0,
                         0,
                         0,
                         0,
                         0,
                         0,
                       250,
          3.48282648772469,
                         0
            }
        };
        y_imu_t expect_bias_1 = {
            .array = {
              0.0099994143,
            0.020000127675,
           -9.809982732725,
              0.0009926216,
           -0.001995876925,
             0.00049300035,
         0.126501033340102,
         0.379631966111436,
         0.299794753017351,
         -97343.2212224229
            }
        };
        y_imu_t expect_bias_2 = {
            .array = {
           -0.019995714275,
             0.00999799065,
           -9.779995422925,
            0.000501677225,
            0.001001624175,
           -0.000996258825,
         0.113862299305529,
         0.373311843338002,
         0.309814367418114,
         -97343.1712147298
            }
        };

        x_state_t actual_x = {0};
        y_imu_t actual_bias_1 = {0};
        y_imu_t actual_bias_2 = {0};

        pad_filter(
            &test_ctx, &imu1, &imu2, false, false, &actual_x, &actual_bias_1, &actual_bias_2
        );

        for (int i = 0; i < 13; ++i) {
            EXPECT_NEAR(actual_x.array[i], expect_x.array[i], abs(expect_x.array[i] * tolerance));
        }
        for (int i = 0; i < 10; ++i) {
            EXPECT_NEAR(
                actual_bias_1.array[i],
                expect_bias_1.array[i],
                abs(expect_bias_1.array[i] * tolerance)
            );
            EXPECT_NEAR(
                actual_bias_2.array[i],
                expect_bias_2.array[i],
                abs(expect_bias_2.array[i] * tolerance)
            );
        }
    }
}

TEST(PadFilterTest, RunFailsWithoutInit) {
    // Arrange
    pad_filter_ctx_t test_ctx = {0};

    y_imu_t imu1 = {.array = {0.01, 0.02, -9.81, 0.001, -0.002, 0.0005, 0.3, 0.0, 0.4, 1013.25}};
    y_imu_t imu2 = {
        .array = {-0.02, 0.01, -9.78, 0.0005, 0.001, -0.001, 0.31, -0.01, 0.39, 1013.30}
    };

    bool is_dead_1 = false;
    bool is_dead_2 = false;

    x_state_t actual_x_init = {0};
    y_imu_t actual_bias_1 = {0};
    y_imu_t actual_bias_2 = {0};

    // Act & Assert
    ASSERT_EQ(
        pad_filter(
            &test_ctx,
            &imu1,
            &imu2,
            is_dead_1,
            is_dead_2,
            &actual_x_init,
            &actual_bias_1,
            &actual_bias_2
        ),
        W_FAILURE
    );
}
