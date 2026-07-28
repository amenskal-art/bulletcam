# BulletCam

Offline object photogrammetry for Android. Orbit a physical object, and the app
turns the resulting pose-tagged photo set into a dense, textured 3D model
entirely on the phone.

There is no server, no upload step and no account. The app does not declare
`android.permission.INTERNET`, so it is not merely a policy that nothing leaves
the device: the platform will not let it.

---

## What it actually does

BulletCam is not a room scanner. It is a camera for objects, and the whole design
follows from one observation: **ARCore already solved the hard part.**

Classical photogrammetry spends most of its time on structure-from-motion -
detecting features across dozens of photos, matching them, and running bundle
adjustment to work out where each photo was taken from. That stage is slow,
fragile, and completely unnecessary on a phone that is already running a
visual-inertial odometry system at 30 Hz.

So every capture is tagged with ARCore's 6-DoF pose and the camera intrinsics at
the moment the shutter fired. The native pipeline takes those as given and jumps
straight to the dense stereo stage.

```
capture (Kotlin)              reconstruction (C++17, no third-party CV)
──────────────────            ─────────────────────────────────────────
ARCore VIO pose      ─┐
camera intrinsics    ─┼──►  1. sparse alignment      ORB + epipolar matching
YUV_420_888 planes   ─┘      2. view selection        co-visibility x baseline
      (zero-copy)            3. dense stereo          PatchMatch MVS
                             4. fusion                forward/backward check
                             5. clean-up              voxel, outliers, normals
                             6. surfacing             Poisson + marching tets
                             7. UV unwrap             chart growing (or xatlas)
                             8. texture bake          occlusion-aware projection
                             9. export                .obj + .mtl + .png, .glb
```

## Capture behaviour

- **Tap to set the subject.** An ARCore hit test if the scene offers one,
  otherwise a fixed distance along the tapped ray - which is the normal case for
  a small object on a plain table.
- **Angle-triggered auto-capture.** Photos fire when the camera has moved far
  enough *angularly* from every previous shot, not on a timer. Even coverage
  matters far more than photo count.
- **Coverage, not counts.** The viewing sphere is split into 24 azimuth sectors
  crossed with 3 elevation bands. The AR ring around the subject lights up as
  each sector is filled, and the strip at the top of the screen shows the same
  thing in a glance.
- **Blur rejection.** Every candidate frame is scored by the variance of its
  Laplacian on the native side and dropped if it is soft. Frames captured while
  the phone is moving too fast are skipped before they are even converted.
- **Exposure and focus lock.** ARCore has no auto-exposure lock of its own, and
  unlocked AE is the single biggest cause of blotchy photogrammetry textures -
  the same surface comes back a different brightness in every photo. BulletCam
  opens the camera through ARCore's shared-camera interop so Camera2's
  `CONTROL_AE_LOCK`, `CONTROL_AWB_LOCK` and a pinned `LENS_FOCUS_DISTANCE` can
  hold one photometric calibration for the whole orbit. Devices that refuse the
  interop path fall back to a plain ARCore session automatically.

## The reconstruction core

Written from scratch in dependency-free C++17. The algorithms follow established
open-source work - OpenMVG's pinhole and epipolar formulations, COLMAP's
patch-match stereo and geometric consistency filtering, OpenMVS's texturing
approach, Open3D's point-cloud operators, Kazhdan's Poisson surface
reconstruction - but nothing is vendored, so the build is hermetic and needs no
network at any point.

Two deliberate departures from the reference implementations, both because the
target is a phone:

- **Poisson on a uniform grid with geometric multigrid**, not an octree. Fixed
  memory, no pointer chasing, and the smoother threads trivially.
- **Marching tetrahedra** for isosurface extraction, not marching cubes. Sixteen
  cases, and none of them ambiguous.

UV unwrapping uses xatlas when the build opts in
(`-DBULLETCAM_USE_XATLAS=ON`, fetched at configure time). The default build uses
a built-in chart-growing packer instead: it region-grows near-planar charts
across the mesh and shelf packs them. More seams than xatlas, but no external
dependency.

### Verifying the core without a phone

`tools/host_test` builds the entire reconstruction pipeline for the host and runs
it against a synthetic scene: a procedurally textured 150 mm sphere rendered from
a 20-view orbit, fed through the real `FrameStore` and `runReconstruction` path.
It then re-reads the exported OBJ and checks every vertex against the true
surface.

```bash
cmake -S tools/host_test -B build/host -DCMAKE_BUILD_TYPE=Release
cmake --build build/host --parallel
./build/host/bulletcam_host_test /tmp/bulletcam_test
```

A passing run reports a median radial error under 3 mm; the reference run lands
around 0.7 mm. CI runs this before it builds the APK.

## Building

```bash
gradle wrapper --gradle-version 8.9   # once, if the wrapper jar is not committed
./gradlew assembleDebug
```

Requirements: JDK 17, Android SDK 35, NDK 27.0.12077973, CMake 3.22.1. The app is
`arm64-v8a` only - dense multi-view stereo is memory-hungry and a 32-bit address
space is not worth supporting for it.

## Quality presets

| Preset   | Stereo edge | PatchMatch iters | Source views | Poisson res | Atlas |
|----------|-------------|------------------|--------------|-------------|-------|
| Fast     | 384 px      | 3                | 4            | 128         | 1024  |
| Balanced | 512 px      | 5                | 6            | 160         | 2048  |
| High     | 768 px      | 6                | 8            | 224         | 4096  |

Triangle count is controlled by `poissonResolution` rather than by decimating
afterwards. Vertex-cluster decimation is available (`targetTriangles`) but is off
by default: it damages normals badly enough to starve the texture bake.

## Getting good results

- Fill the frame. Aim for the object covering at least a third of the view.
- Diffuse, even light. Hard shadows get baked into the texture as if they were
  paint.
- Matte, textured surfaces reconstruct well. Glass, chrome and flat white
  surfaces have nothing for a stereo matcher to lock onto.
- Two or three orbits at different heights beat one slow orbit at eye level.
- Keep 0.3-0.8 m from the subject and move steadily. The app will tell you when
  you are going too fast.

## Repository layout

```
app/src/main/cpp/          reconstruction core + JNI bridge
  core/                    image, camera model, frame storage, threading
  sparse/                  ORB features, epipolar matching, triangulation
  dense/                   view selection, PatchMatch MVS, depth fusion
  surface/                 point cloud, Poisson, marching tets, mesh, UV atlas
  texture/                 projective texture baking
  io/                      PNG encoder, OBJ and GLB writers
app/src/main/java/         Kotlin capture layer
  ar/                      ARCore session, GL background, shared-camera lock
  capture/                 orbit coverage model, capture controller
  nativebridge/            JNI binding
  ui/                      Compose screens
  viewer/                  OBJ loader and GL orbit preview
  vm/                      view model
tools/host_test/           host-side regression test for the core
.github/workflows/         CI
```

## Notes and limitations

- The shared-camera exposure-lock path follows Google's `SharedCameraActivity`
  ordering but has not been verified on physical hardware; it is written to fail
  soft, and a device that rejects it drops to a plain ARCore session with fixed
  focus.
- Frame conversion runs on the GL thread when the shutter fires, which drops one
  preview frame per capture. Reconstruction itself runs on `Dispatchers.Default`
  and never touches the UI thread.
- Reconstruction is CPU-only. A high-quality run on 40 photos takes several
  minutes and will make the phone warm.
