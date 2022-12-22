#include "stdafx.h"

#include "Camera.h"

using namespace DirectX;

static inline float degrees_to_radians(float degrees)
{
    return degrees * PI / 180.f;
}

DirectX::XMVECTOR Camera::GetRight() const
{
    return XMVector3Normalize(XMVector3Cross(GetForward(), vup));
}
DirectX::XMVECTOR Camera::GetUp() const
{
    return XMVector3Normalize(XMVector3Cross(GetRight(), GetForward()));
}

DirectX::XMVECTOR Camera::GetForward() const
{
    return XMVector3Normalize(lookAt - lookFrom);
}

DirectX::XMVECTOR Camera::GetLookFrom() const
{
    return lookFrom;
}

DirectX::XMVECTOR Camera::GetLookAt() const
{
    return lookAt;
}

DirectX::XMVECTOR Camera::GetLowerLeft(XMVECTOR& vpHorizontal, XMVECTOR& vpVertical) const
{
    XMVECTOR lookfrom = GetLookFrom();
    XMVECTOR forward = GetForward();
    XMVECTOR up = GetUp();
    XMVECTOR right = GetRight();

    float theta = degrees_to_radians(vfov);
    float h = tan(theta / 2);
    float vpHeight = 2.f * h;
    float vpWidth = aspectRatio * vpHeight;

    vpHorizontal = focusDist * vpWidth * right;
    vpVertical = focusDist * vpHeight * up;

    XMVECTOR leftCorner = lookfrom + focusDist * forward - 0.5f * vpHorizontal - 0.5 * vpVertical;

    return leftCorner;
}

Camera::Camera(DirectX::XMVECTOR pos, DirectX::XMVECTOR lookAt, DirectX::XMVECTOR up, float vfov, float aperture,
               float focusDist)
    : lookFrom(pos), lookAt(lookAt), vup(up), vfov(vfov), aperture(aperture), focusDist(focusDist)
{
    // transform = XMMatrixTranspose(XMMatrixLookAtRH(pos, lookAt, up));
}

void Camera::RotateRight(float d)
{
    float rotation = rotationSpeed * d;
    XMMATRIX rot = XMMatrixRotationAxis(GetUp(), rotation);
    lookFrom = XMVector3TransformCoord(lookFrom, rot);
}

void Camera::RotateUp(float d)
{
    float rotation = rotationSpeed * d;
    XMMATRIX rot = XMMatrixRotationAxis(GetRight(), rotation);
    lookFrom = XMVector3TransformCoord(lookFrom, rot);
}

void Camera::MoveForward(float d)
{
    float translation = moveSpeed * d;
    XMMATRIX trans = XMMatrixTranslationFromVector(GetForward() * translation);
    lookFrom = XMVector3TransformCoord(lookFrom, trans);
    lookAt = XMVector3TransformCoord(lookAt, trans);
}

void Camera::MoveRight(float d)
{
    float translation = moveSpeed * d;
    XMMATRIX trans = XMMatrixTranslationFromVector(GetRight() * translation);
    lookFrom = XMVector3TransformCoord(lookFrom, trans);
    lookAt = XMVector3TransformCoord(lookAt, trans);
}

void Camera::Zoom(float d)
{
    static constexpr float ZOOM_SPEED = .01f;
    vfov += ZOOM_SPEED * d;
}

void Camera::ChangeAperture(float d)
{
    static constexpr float APERTURE_SPEED = .003f;
    aperture += APERTURE_SPEED * d;
}

void Camera::ChangeFocusDistance(float d)
{
    static constexpr float FOCUS_SPEED = .8f;
    focusDist += FOCUS_SPEED * d;
}