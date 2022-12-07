#include "stdafx.h"

#include "Camera.h"

using namespace DirectX;

DirectX::XMVECTOR Camera::GetRight()
{
    return XMVector3Normalize(XMVector3Cross(GetForward(), vup));
}
DirectX::XMVECTOR Camera::GetUp()
{
    return XMVector3Normalize(XMVector3Cross(GetRight(), GetForward()));
}

DirectX::XMVECTOR Camera::GetForward()
{
    return XMVector3Normalize(lookAt - lookFrom);
}

DirectX::XMVECTOR Camera::GetLookFrom()
{
    return lookFrom;
}

DirectX::XMVECTOR Camera::GetLookAt()
{
    return lookAt;
}

Camera::Camera(DirectX::XMVECTOR pos, DirectX::XMVECTOR lookAt, DirectX::XMVECTOR up)
    : lookFrom(pos), lookAt(lookAt), vup(up)
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