#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/vector3.hpp> // Using Vector3 for Kp, Ki, Kd
#include <raylib.h>

#define RAYGUI_IMPLEMENTATION
#include "raygui.h"              // Must be in the same folder or include path

class RobotControlNode : public rclcpp::Node {
public:
    // Variables for Visualization (Incoming)
    float current_roll = 0.0f;

    // Variables for Tuning (Outgoing)
    float kp = 0.0f;
    float ki = 0.0f;
    float kd = 0.0f;

    RobotControlNode() : Node("raylib_gui_node") {
        // 1. Subscribe to IMU (to see effect of tuning)
        sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu/data", 10,
            [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
                // Simplified: Just taking one axis of acceleration for demo
                current_roll = msg->linear_acceleration.x; 
            });

        // 2. Publish PID values (ESP32 will subscribe to this)
        pub_pid_ = this->create_publisher<geometry_msgs::msg::Vector3>("/tuning/pid", 10);
    }

    void publish_pid() {
        auto msg = geometry_msgs::msg::Vector3();
        msg.x = kp;
        msg.y = ki;
        msg.z = kd;
        pub_pid_->publish(msg);
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr pub_pid_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotControlNode>();

    const int screenWidth = 800;
    const int screenHeight = 450;
    InitWindow(screenWidth, screenHeight, "ROS 2 PID Tuner");

    // Initialize GUI styles (optional, makes it look larger)
    GuiSetStyle(DEFAULT, TEXT_SIZE, 20);

    SetTargetFPS(60);

    while (!WindowShouldClose()) {
        // --- ROS UPDATE ---
        rclcpp::spin_some(node);

        // --- DRAWING ---
        BeginDrawing();
        ClearBackground(RAYWHITE);

            // 1. Draw the Sliders (Immediate Mode GUI)
            // Format: GuiSlider((Rectangle){x, y, w, h}, "Text Left", "Text Right", &variable, min, max)
            
            // Kp Slider
            if (GuiSlider((Rectangle){ 100, 40, 200, 30 }, "Kp", TextFormat("%2.2f", node->kp), &node->kp, 0.0f, 10.0f)) {
                node->publish_pid(); // Publish immediately on change
            }
            
            // Ki Slider
            if (GuiSlider((Rectangle){ 100, 90, 200, 30 }, "Ki", TextFormat("%2.2f", node->ki), &node->ki, 0.0f, 10.0f)) {
                node->publish_pid();
            }

            // Kd Slider
            if (GuiSlider((Rectangle){ 100, 140, 200, 30 }, "Kd", TextFormat("%2.2f", node->kd), &node->kd, 0.0f, 10.0f)) {
                node->publish_pid();
            }

            // 2. Visual Feedback Area (Right side of screen)
            DrawRectangle(400, 0, 400, 450, LIGHTGRAY);
            DrawText("Robot Response", 420, 20, 20, DARKGRAY);

            // Draw a bar representing the robot's current tilt/acceleration
            // Center is 225. We multiply roll by 20 to scale it up visually.
            int barHeight = (int)(node->current_roll * 20.0f); 
            DrawRectangle(580, 225, 40, barHeight, MAROON);
            DrawLine(400, 225, 800, 225, DARKGRAY); // Zero line

        EndDrawing();
    }

    CloseWindow();
    rclcpp::shutdown();
    return 0;
}
