#include <ros/ros.h>
#include "raylib.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "battery_display");
    ros::NodeHandle nh;

    ROS_INFO_STREAM("Battery Display Node Started");

    //SetTraceLogLevel(LOG_NONE);
    InitWindow(800, 600, "Battery Display");

    int display = GetCurrentMonitor();
    int screenWidth = GetMonitorWidth(display);
    int screenHeight = GetMonitorHeight(display);

    ROS_INFO_STREAM("setting window size to (" << screenWidth << "x" << screenHeight << ")");
    SetWindowSize(screenWidth, screenHeight);

    ros::Duration(1.0).sleep();

    ToggleFullscreen();
    SetTargetFPS(10);               // Set our game to run at 60 frames-per-second

    while (!WindowShouldClose() && ros::ok())    // Detect window close button or ESC key
    {
        BeginDrawing();
        ClearBackground(RAYWHITE);
        int textWidth = MeasureText("Congrats! You created your first window!", 20);
        DrawText("Congrats! You created your first window!", (screenWidth - textWidth) / 2.0f, screenHeight / 2.0f, 20, LIGHTGRAY);
        EndDrawing();
    }

    CloseWindow();        // Close window and OpenGL context

    return 0;
}
