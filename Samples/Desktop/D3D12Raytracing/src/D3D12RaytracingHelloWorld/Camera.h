#pragma once

#include <DirectXMath.h>

class Camera
{
public:
    DirectX::XMVECTOR GetRight() const;
    DirectX::XMVECTOR GetUp() const;
    DirectX::XMVECTOR GetForward() const;
    DirectX::XMVECTOR GetLookFrom() const;
    DirectX::XMVECTOR GetLookAt() const;
    float GetVFOV() const;

    Camera(DirectX::XMVECTOR pos, DirectX::XMVECTOR lookAt, DirectX::XMVECTOR up, float vfov);
    void RotateRight(float d); // Yaw
    void RotateUp(float d);    // Pitch
    void MoveForward(float d);
    void MoveRight(float d);

private:
    DirectX::XMVECTOR lookFrom;
    DirectX::XMVECTOR lookAt;
    DirectX::XMVECTOR vup;
    float rotationSpeed = .01f;
    float moveSpeed = .1f;
    float vfov;
};
