#pragma once

#include <DirectXMath.h>

static constexpr float PI = 3.1415926535897932385f;

class Camera
{
public:
    DirectX::XMVECTOR GetRight() const;
    DirectX::XMVECTOR GetUp() const;
    DirectX::XMVECTOR GetForward() const;
    DirectX::XMVECTOR GetLookFrom() const;
    DirectX::XMVECTOR GetLookAt() const;
    DirectX::XMVECTOR GetLowerLeft(DirectX::XMVECTOR& vpHorizontal, DirectX::XMVECTOR& vpVertical) const;
    float GetLensRadius() const
    {
        return aperture * .5f;
    }

    Camera(DirectX::XMVECTOR pos, DirectX::XMVECTOR lookAt, DirectX::XMVECTOR up, float vfov, float aperture,
           float focusDist);
    void RotateRight(float d); // Yaw
    void RotateUp(float d);    // Pitch
    void MoveForward(float d);
    void MoveRight(float d);
    void Zoom(float d);
    void ChangeAperture(float d);
    void UpdateAspectRatio(float aspectRatio)
    {
        this->aspectRatio = aspectRatio;
    }

private:
    DirectX::XMVECTOR lookFrom;
    DirectX::XMVECTOR lookAt;
    DirectX::XMVECTOR vup;
    float rotationSpeed = .005f;
    float moveSpeed = .1f;
    float vfov;
    float aspectRatio = 1.f; // temporary value since there is no window yet
    float aperture;
    float focusDist;
};
