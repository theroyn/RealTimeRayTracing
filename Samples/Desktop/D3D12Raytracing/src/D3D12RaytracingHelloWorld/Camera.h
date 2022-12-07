#pragma once

#include <DirectXMath.h>

class Camera
{
public:
    DirectX::XMVECTOR GetRight();
    DirectX::XMVECTOR GetUp();
    DirectX::XMVECTOR GetForward();
    DirectX::XMVECTOR GetLookFrom();
    DirectX::XMVECTOR GetLookAt();

    Camera(DirectX::XMVECTOR pos, DirectX::XMVECTOR lookAt, DirectX::XMVECTOR up);
    void RotateRight(float d); //Yaw
    void RotateUp(float d); //Pitch
    void MoveForward(float d);
    void MoveRight(float d);

private:
    DirectX::XMVECTOR lookFrom;
    DirectX::XMVECTOR lookAt;
    DirectX::XMVECTOR vup;
    float rotationSpeed = .01f;
    float moveSpeed = .1f;
};