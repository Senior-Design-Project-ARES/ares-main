#include <eigen3/Eigen/Dense>
#include <cmath>

/* ============================================================
   CONFIG & GLOBAL VARIABLES
   ============================================================ */

// Define fixed sizes to avoid dynamic memory allocation (Heap Fragmentation)
#define MAX_WAYPOINTS 20 

// Global variables to persist between timer interrupts
Eigen::Matrix<double, MAX_WAYPOINTS, 2> global_waypoints;
int last_refindex = 0;
bool waypoints_loaded = false;

/* ============================================================
   CONTROLLER WRAPPER
   ============================================================ */
// Note: We use Eigen::Ref to allow fixed-size matrices to be passed 
// into functions expecting generic types without copying data.
void codeGen_func(
    const Eigen::Vector3d& State,
    const Eigen::Ref<const Eigen::Matrix<double, Eigen::Dynamic, 2>>& waypoints,
    double& linVel,
    double& angVel,
    int& last_refindex
);

/* ============================================================
   STM32 HARDWARE INTERRUPT (The "Polling" Mechanism)
   ============================================================ */

/**
 * This function is called by the hardware timer (e.g., TIM2) 
 * at a fixed frequency (e.g., 100Hz / every 10ms).
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) 
{
    if (htim->Instance == TIM2) 
    {
        if (!waypoints_loaded) return;

        // 1. Get current state from Sensors (IMU, Encoders, or GPS)
        // Usually: [x_pos, y_pos, heading_rad]
        Eigen::Vector3d State = Sensor_GetRobotState(); 

        double linVel = 0.0;
        double angVel = 0.0;

        // 2. Run the Controller
        // Because we use fixed-size matrices, this execution time is deterministic
        codeGen_func(State, global_waypoints, linVel, angVel, last_refindex);

        // 3. Actuate Motors
        // Convert velocities to PWM or CAN commands for your motor drivers
        Motor_SetVelocity(linVel, angVel);
    }
}

/* ============================================================
   INITIALIZATION (Setup)
   ============================================================ */

int main(void) 
{
    // Standard STM32 HAL Init
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_TIM2_Init();

    // 4. Load Waypoints 
    // Instead of reading a CSV file, we populate the fixed-size Matrix.
    // In a real app, you might receive these over UART or SPI.
    global_waypoints << 0.0, 0.0,
                        1.0, 0.5,
                        2.0, 0.0,
                        3.0, -0.5; 
                        // ... fill the rest with zeros or use a sub-block
    
    waypoints_loaded = true;

    // 5. Start the Timer Interrupt
    HAL_TIM_Base_Start_IT(&htim2);

    while (1) 
    {
        // The main loop is now empty or handles low-priority tasks (LED blinking, UART logging)
        // The "Polling Rate" is handled entirely by the Timer Interrupt above.
    }
}