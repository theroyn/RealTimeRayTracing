
# Real Time Ray-Tracing in One Weekend
This repo contains my implementation of the first book of Ray-Tracing in One Weekend using the DirectX 12 RayTracing(DXR) API.

## Ray-Tracing
Ray-tracing is a rendering technique that brings an extra level of realism to computer graphics. It emulates the way light reflects and refracts in the real world, providing a more believable environment than what’s typically seen using other rendering techniques.

The technique sends a ray from a virtual "eye" (or camera) through each pixel in the screen, let it hit the scene's objects and bounce from them. The ray might hit other objects and might hit a light source, collecting their illuminance and color. In practicality to avoid noise, many rays(samples) need to be sent for each pixel.

While the technique has been around for many decades, it was unusable for the gaming industry due to its very intensive processing demands. It was used mainly in the movie industry in which every frame can be rendered in advance.

Nowadays, thanks to advances in GPU hardware such as NVIDIA's RTX series and smart APIs such as Microsoft's DXR, real time ray-tracing becomes possible. This project will demonstrate it.

## DirectX 12 RayTracing (DXR)
At the 2018 Game Developer's Conference, Microsoft introduced an addition to DirectX 12 called DirectX Raytracing (DXR), an API allowing easy use of GPU-accelerated ray tracing in DirectX and allowing simple interoperability with traditional DirectX rasterization.

## Ray-Tracing in One Weekend
[Ray-Tracing in One Weekend](https://raytracing.github.io/books/RayTracingInOneWeekend.html) is an excellent online tutorial for implementing a ray-tracer(technically a path-tracer) with C++. The tutorial walks the user through:
* writing a basic camera.
* sending rays fron the camera and letting them bounce in the scene
* constructing a scene containing matte, metal, and glass (dielectric) materials.

The output result is a single image which can take even **an hour** to render. My project, depending on the GPU that runs it and the parameter number of samples per pixel, can render at **30 frames per second**.

## Building
Build the D3D12Raytracing.sln solution and run the executable.

The only dependency is [assimp](https://github.com/assimp/assimp), an asset loading library. The project currently build with version 5.2.5. You can build it from source and update the project's include directories and input libraries location.

## Youtube Demo:

[<img src="https://img.youtube.com/vi/N5Vw_xkd9WI/maxresdefault.jpg" width="50%">](https://youtu.be/N5Vw_xkd9WI)

## Resources I used
* [Ray-Tracing in One Weekend](https://raytracing.github.io/books/RayTracingInOneWeekend.html)
* [DirectX Raytracing (DXR) Functional Spec](https://microsoft.github.io/DirectX-Specs/d3d/Raytracing.html)
* [NVIDIA DXR Tutorial](https://developer.nvidia.com/rtx/raytracing/dxr/dx12-raytracing-tutorial-part-1)
* [DirectX-Graphics-Samples](https://github.com/Microsoft/DirectX-Graphics-Samples)
* [assimp](https://github.com/assimp/assimp)
