#pragma once
#include <raylib.h>

// ============================================================================
//  visr/systems/CameraSystem.hpp
//
//  Free-look camera with WASD + Q/E vertical + RMB drag.
//  Stateless: call update() every render frame before BeginMode3D.
//
//  Controls:
//    W/S          — forward / back
//    A/D          — strafe left / right
//    Q / LShift   — up
//    E / LCtrl    — down
//    RMB drag     — look (yaw + pitch)
//    Scroll wheel — zoom (FOV)
// ============================================================================

namespace visr
{
    struct CameraSystem
    {
        float move_speed  = 10.0f;
        float sensitivity = 0.1f;

        void update(Camera3D &camera) const
        {
            Vector3 movement = {0, 0, 0};
            Vector3 rotation = {0, 0, 0};

            // ── Translation ───────────────────────────────────────────────
            if (IsKeyDown(KEY_W))           movement.x += move_speed * GetFrameTime();
            if (IsKeyDown(KEY_S))           movement.x -= move_speed * GetFrameTime();
            if (IsKeyDown(KEY_A))           movement.y -= move_speed * GetFrameTime();
            if (IsKeyDown(KEY_D))           movement.y += move_speed * GetFrameTime();
            if (IsKeyDown(KEY_Q) || IsKeyDown(KEY_LEFT_SHIFT))
                                            movement.z += move_speed * GetFrameTime();
            if (IsKeyDown(KEY_E) || IsKeyDown(KEY_LEFT_CONTROL))
                                            movement.z -= move_speed * GetFrameTime();

            // ── Look (RMB drag) ───────────────────────────────────────────
            if (IsMouseButtonDown(MOUSE_BUTTON_RIGHT))
            {
                rotation.x = GetMouseDelta().x * sensitivity;
                rotation.y = GetMouseDelta().y * sensitivity;
            }

            // ── Zoom (scroll → FOV) ───────────────────────────────────────
            float scroll = GetMouseWheelMove();
            if (scroll != 0.0f)
            {
                camera.fovy -= scroll * 2.0f;
                if (camera.fovy < 5.0f)  camera.fovy = 5.0f;
                if (camera.fovy > 120.0f) camera.fovy = 120.0f;
            }

            UpdateCameraPro(&camera, movement, rotation, 0.0f);
        }

        // Resets camera to a sensible default for a physics scene.
        static Camera3D make_default()
        {
            Camera3D cam{};
            cam.position   = { 8.0f, 6.0f, 14.0f };
            cam.target     = { 0.0f, 2.0f,  0.0f };
            cam.up         = { 0.0f, 1.0f,  0.0f };
            cam.fovy       = 45.0f;
            cam.projection = CAMERA_PERSPECTIVE;
            return cam;
        }
    };

} // namespace visr