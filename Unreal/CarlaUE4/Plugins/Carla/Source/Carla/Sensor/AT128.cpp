// Copyright (c) 2017 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.
// Lidar implemented by Csonthó Mihály based on RayCastLidar code

#include <PxScene.h>
#include <cmath>
#include "Carla.h"
#include "Carla/Sensor/AT128.h"
#include "Carla/Actor/ActorBlueprintFunctionLibrary.h"
#include "carla/geom/Math.h"

#include <compiler/disable-ue4-macros.h>
#include "carla/geom/Math.h"
#include "carla/ros2/ROS2.h"
#include "carla/geom/Location.h"
#include <compiler/enable-ue4-macros.h>

#include "DrawDebugHelpers.h"
#include "Engine/CollisionProfile.h"
#include "Runtime/Engine/Classes/Kismet/KismetMathLibrary.h"
#include "Runtime/Core/Public/Async/ParallelFor.h"

// ==============================
// AT128P channel tables (from manual: §1.5 & Appendix A). We'll wire them later.
// Vertical angles are channel-ordered: Ch1 (top) → Ch128 (bottom). 
namespace {

static const float kAT128P_VertDeg[128] = {
  12.93f,12.73f,12.53f,12.33f,12.13f,11.93f,11.73f,11.53f,
  11.33f,11.13f,10.93f,10.73f,10.53f,10.33f,10.13f, 9.93f,
   9.73f, 9.53f, 9.33f, 9.13f, 8.93f, 8.73f, 8.53f, 8.33f,
   8.13f, 7.93f, 7.73f, 7.53f, 7.33f, 7.13f, 6.93f, 6.73f,
   6.53f, 6.33f, 6.13f, 5.93f, 5.73f, 5.53f, 5.33f, 5.13f,
   4.93f, 4.73f, 4.53f, 4.33f, 4.13f, 3.93f, 3.73f, 3.53f,
   3.33f, 3.13f, 2.93f, 2.73f, 2.53f, 2.33f, 2.13f, 1.93f,
   1.73f, 1.53f, 1.33f, 1.13f, 0.93f, 0.73f, 0.53f, 0.33f,
   0.13f,-0.07f,-0.27f,-0.47f,-0.67f,-0.87f,-1.07f,-1.27f,
  -1.47f,-1.67f,-1.87f,-2.07f,-2.27f,-2.47f,-2.67f,-2.87f,
  -3.07f,-3.27f,-3.47f,-3.67f,-3.87f,-4.07f,-4.27f,-4.47f,
  -4.67f,-4.87f,-5.07f,-5.27f,-5.47f,-5.67f,-5.87f,-6.07f,
  -6.27f,-6.47f,-6.67f,-6.87f,-7.07f,-7.27f,-7.47f,-7.67f,
  -7.87f,-8.07f,-8.27f,-8.47f,-8.67f,-8.87f,-9.07f,-9.27f,
  -9.47f,-9.67f,-9.87f,-10.07f,-10.27f,-10.47f,-10.67f,-10.87f,
  -11.07f,-11.27f,-11.47f,-11.67f,-11.87f,-12.07f,-12.27f,-12.47f
};

// Optional per-channel horizontal (azimuth) offsets, in degrees.
// Keep them for fidelity; if you don’t want them later, we can zero them. 
static const float kAT128P_HorizDeg[128] = {
  2.4f,-0.65f, 2.4f,-0.65f, 2.4f,-0.65f, 2.4f,-0.65f,
  2.4f,-0.65f, 2.4f,-0.65f, 2.4f,-0.65f, 2.4f,-0.65f,
 -2.4f, 0.65f,-2.4f, 0.65f,-2.4f, 0.65f,-2.4f, 0.65f,
 -2.4f, 0.65f,-2.4f, 0.65f,-2.4f, 0.65f,-2.4f, 0.65f,
  2.4f,-0.65f, 2.4f,-0.65f, 2.4f,-0.65f, 2.4f,-0.65f,
  2.4f,-0.65f, 2.4f,-0.65f, 2.4f,-0.65f, 2.4f,-0.65f,
 -2.4f, 0.65f,-2.4f, 0.65f,-2.4f, 0.65f,-2.4f, 0.65f,
 -2.4f, 0.65f,-2.4f, 0.65f,-2.4f, 0.65f,-2.4f, 0.65f,
  2.4f,-0.65f, 2.4f,-0.65f, 2.4f,-0.65f, 2.4f,-0.65f,
  2.4f,-0.65f, 2.4f,-0.65f, 2.4f,-0.65f, 2.4f,-0.65f,
 -2.4f, 0.65f,-2.4f, 0.65f,-2.4f, 0.65f,-2.4f, 0.65f,
 -2.4f, 0.65f,-2.4f, 0.65f,-2.4f, 0.65f,-2.4f, 0.65f,
  2.4f,-0.65f, 2.4f,-0.65f, 2.4f,-0.65f, 2.4f,-0.65f,
  2.4f,-0.65f, 2.4f,-0.65f, 2.4f,-0.65f, 2.4f,-0.65f,
 -2.4f, 0.65f,-2.4f, 0.65f,-2.4f, 0.65f,-2.4f, 0.65f,
 -2.4f, 0.65f,-2.4f, 0.65f,-2.4f, 0.65f,-2.4f, 0.65f
};
// --- Identify near-field-enabled channels (pattern from Appendix A) ---
static bool IsNearFieldEnabled(int ch) {
  // Matches 3,7,11,15 within every 16-channel block → 32 channels total (0.5 m min):contentReference[oaicite:1]{index=1}
  const int m = ch % 16;
  return (m == 3 || m == 7 || m == 11 || m == 15);
}

// --- Fill per-channel min / max ranges according to the manual (§ 1.5 + Appendix A) ---
static void FillAT128Ranges(std::vector<float>& minR, std::vector<float>& maxR) {
  minR.resize(128);
  maxR.resize(128);
  for (int i = 0; i < 128; ++i) {
    const int ch = i + 1;
    // Near-field-enabled → 0.5 m, others 7.2 m (min instrumented range)
    minR[i] = IsNearFieldEnabled(ch) ? 0.5f : 7.2f;
    // Channels 33–96 → 260 m max, the rest → 90 m max
    maxR[i] = (ch >= 33 && ch <= 96) ? 260.0f : 90.0f;
  }
}

} // namespace
// ==============================

FActorDefinition AAT128::GetSensorDefinition()
{
  return UActorBlueprintFunctionLibrary::MakeLidarDefinition(TEXT("at128"));
}


AAT128::AAT128(const FObjectInitializer& ObjectInitializer)
  : Super(ObjectInitializer) {
    
    RandomEngine = CreateDefaultSubobject<URandomEngine>(TEXT("RandomEngine"));
    SetSeed(Description.RandomSeed);
}

void AAT128::Set(const FActorDescription &ActorDescription)
{
  ASensor::Set(ActorDescription);
  FLidarDescription LidarDescription;
  UActorBlueprintFunctionLibrary::SetLidar(ActorDescription, LidarDescription);
  Set(LidarDescription);
}

void AAT128::Set(const FLidarDescription &LidarDescription)
{
  Description = LidarDescription;
  LidarData = FLidarData(Description.Channels);
  CreateLasers();
  // --- Replace default evenly spaced angles with AT128 vertical table ---
  LaserAngles.Empty();
  LaserAngles.Reserve(128);

  for (int i = 0; i < 128; ++i) {
    LaserAngles.Add(kAT128P_VertDeg[i]);   // nincs DegreesToRadians!
  }

  // UE_LOG(LogCarla, Warning, TEXT("AT128 vertical angles (deg):"));
  // for (int i = 0; i < 8; ++i) {
  //   UE_LOG(LogCarla, Warning, TEXT("  CH%03d = %.3f deg (%.3f rad)"),
  //         i, FMath::RadiansToDegrees(LaserAngles[i]), LaserAngles[i]);
  // }
  // UE_LOG(LogCarla, Warning, TEXT("  ... last = %.3f deg (%.3f rad)"),
  //       FMath::RadiansToDegrees(LaserAngles.Last()), LaserAngles.Last());

  // (Optional) horizontal offsets: store them in radians for later use
  ChannelHorizOffsetsRad.Empty();
  ChannelHorizOffsetsRad.Reserve(128);

  for (int i = 0; i < 128; ++i) {
    ChannelHorizOffsetsRad.Add(FMath::DegreesToRadians(kAT128P_HorizDeg[i]));
  }
  

    // --- Fill per-channel min / max range arrays (manual § 1.5 + Appendix A):contentReference[oaicite:2]{index=2} ---
  {
    std::vector<float> minR, maxR;
    FillAT128Ranges(minR, maxR);
    ChannelMinRange.Empty();
    ChannelMaxRange.Empty();
    ChannelMinRange.Reserve(128);
    ChannelMaxRange.Reserve(128);
    for (int i = 0; i < 128; ++i) {
      ChannelMinRange.Add(minR[i]);
      ChannelMaxRange.Add(maxR[i]);
    }
  }


  PointsPerChannel.resize(Description.Channels);

  // Compute drop off model parameters
  DropOffBeta = 1.0f - Description.DropOffAtZeroIntensity;
  DropOffAlpha = Description.DropOffAtZeroIntensity / Description.DropOffIntensityLimit;
  DropOffGenActive = Description.DropOffGenRate > std::numeric_limits<float>::epsilon();
}

void AAT128::PostPhysTick(UWorld *World, ELevelTick TickType, float DeltaTime)
{
  TRACE_CPUPROFILER_EVENT_SCOPE(AAT128::PostPhysTick);
  SimulateLidar(DeltaTime);

  auto DataStream = GetDataStream(*this);
  auto SensorTransform = DataStream.GetSensorTransform();

  {
    TRACE_CPUPROFILER_EVENT_SCOPE_STR("Send Stream");
    DataStream.SerializeAndSend(*this, LidarData, DataStream.PopBufferFromPool());
  }
  // ROS2
  #if defined(WITH_ROS2)
  auto ROS2 = carla::ros2::ROS2::GetInstance();
  if (ROS2->IsEnabled())
  {
    TRACE_CPUPROFILER_EVENT_SCOPE_STR("ROS2 Send");
    AActor* ParentActor = GetAttachParentActor();
    auto Transform = (ParentActor) ? GetActorTransform().GetRelativeTransform(ParentActor->GetActorTransform()) : GetActorTransform();
    ROS2->ProcessDataFromLidar(DataStream.GetSensorType(), Transform, LidarData, this);
  }
  #endif


}

float AAT128::ComputeIntensity(const FSemanticDetection& RawDetection) const
{
  const carla::geom::Location HitPoint = RawDetection.point;
  const float Distance = HitPoint.Length();

  const float AttenAtm = Description.AtmospAttenRate;
  const float AbsAtm = exp(-AttenAtm * Distance);

  const float IntRec = AbsAtm;

  return IntRec;
}

AAT128::FDetection AAT128::ComputeDetection(const FHitResult& HitInfo, const FTransform& SensorTransf, int32 ChannelIdx) const
{
  FDetection Detection;
  const FVector HitPoint = HitInfo.ImpactPoint;
  Detection.point = SensorTransf.Inverse().TransformPosition(HitPoint);

  const float Distance = Detection.point.Length();
  // --- Enforce per-channel range limits (manual § 1.5 + Appendix A):contentReference[oaicite:1]{index=1} ---
  const float dmin = ChannelMinRange.IsValidIndex(ChannelIdx) ? ChannelMinRange[ChannelIdx] : 0.5f;
  const float dmax = ChannelMaxRange.IsValidIndex(ChannelIdx) ? ChannelMaxRange[ChannelIdx] : Description.Range;

  if (Distance < dmin || Distance > dmax) {
    return {};             // discard this hit
  }

  const float AttenAtm = Description.AtmospAttenRate;
  const float AbsAtm = exp(-AttenAtm * Distance);

  const float IntRec = AbsAtm;

  Detection.intensity = IntRec;

  return Detection;
}

  void AAT128::PreprocessRays(uint32_t Channels, uint32_t MaxPointsPerChannel) {
    Super::PreprocessRays(Channels, MaxPointsPerChannel);

    for (auto ch = 0u; ch < Channels; ch++) {
      for (auto p = 0u; p < MaxPointsPerChannel; p++) {
        RayPreprocessCondition[ch][p] = !(DropOffGenActive && RandomEngine->GetUniformFloat() < Description.DropOffGenRate);
      }
    }
  }

  bool AAT128::PostprocessDetection(FDetection& Detection) const
  {
    if (Description.NoiseStdDev > std::numeric_limits<float>::epsilon()) {
      const auto ForwardVector = Detection.point.MakeUnitVector();
      const auto Noise = ForwardVector * RandomEngine->GetNormalDistribution(0.0f, Description.NoiseStdDev);
      Detection.point += Noise;
    }

    const float Intensity = Detection.intensity;
    if(Intensity > Description.DropOffIntensityLimit)
      return true;
    else
      return RandomEngine->GetUniformFloat() < DropOffAlpha * Intensity + DropOffBeta;
  }

  void AAT128::ComputeAndSaveDetections(const FTransform& SensorTransform) {
    for (auto idxChannel = 0u; idxChannel < Description.Channels; ++idxChannel)
      PointsPerChannel[idxChannel] = RecordedHits[idxChannel].size();

    LidarData.ResetMemory(PointsPerChannel);

    for (auto idxChannel = 0u; idxChannel < Description.Channels; ++idxChannel) {
      for (auto& hit : RecordedHits[idxChannel]) {
        FDetection Detection = ComputeDetection(hit, SensorTransform, static_cast<int32>(idxChannel));
        if (PostprocessDetection(Detection))
          LidarData.WritePointSync(Detection);
        else
          PointsPerChannel[idxChannel]--;
      }
    }

    LidarData.WriteChannelCount(PointsPerChannel);
    
  }
  /*
  SimulateLidar(float DeltaTime)

  Main simulation function called every tick. Casts rays across a configurable horizontal
  field of view (centered on the sensor's forward axis) with user-defined angular resolution,
  and for each vertical laser (from LaserAngles). No continuous rotation is used — this lidar
  performs a fixed sweep relative to the sensor's forward axis.

  Steps:
    - Validates laser/channel setup.
    - Prepares ray preprocessing and hit recording structures.
    - For each vertical channel, casts N rays (N = HorizontalFov / HorizontalResolution) across the horizontal sweep.
    - Computes the horizontal angle for each ray based on the configured FOV and resolution.
    - Records hits per channel if they pass preprocessing.
    - Computes and stores detections from the hit results.

  This function defines the horizontal scanning behavior of the sensor and adapts to any FOV/resolution
*/
static float SnapToStep(float value, float step) {
  return std::round(value / step) * step;
}

void AAT128::SimulateLidar(const float DeltaTime)
{
  TRACE_CPUPROFILER_EVENT_SCOPE(AAT128::SimulateLidar);
  const uint32 ChannelCount = Description.Channels;
  const float HorizontalResolution = SnapToStep(Description.HorizontalResolution, 0.01f);
  const uint32 PointsToScanWithOneLaser = Description.HorizontalFov / HorizontalResolution;




  if (PointsToScanWithOneLaser <= 0)
  {
    UE_LOG(
        LogCarla,
        Warning,
        TEXT("%s: no points requested this frame, try increasing the number of points per second."),
        *GetName());
    return;
  }

  check(ChannelCount == LaserAngles.Num());

  ResetRecordedHits(ChannelCount, PointsToScanWithOneLaser);
  PreprocessRays(ChannelCount, PointsToScanWithOneLaser);

  GetWorld()->GetPhysicsScene()->GetPxScene()->lockRead();
  {
    TRACE_CPUPROFILER_EVENT_SCOPE(ParallelFor);
    ParallelFor(ChannelCount, [&](int32 idxChannel) {
      TRACE_CPUPROFILER_EVENT_SCOPE(ParallelForTask);

      FCollisionQueryParams TraceParams(FName(TEXT("Laser_Trace")), true, this);
      TraceParams.bTraceComplex = true;
      TraceParams.bReturnPhysicalMaterial = false;

      const float VertAngle = LaserAngles[idxChannel];
      for (auto idxPtsOneLaser = 0u; idxPtsOneLaser < PointsToScanWithOneLaser; idxPtsOneLaser++) {
        FHitResult HitResult;
        
        // --- Correct horizontal sweep computation ---
        const float stepDeg = Description.HorizontalFov / static_cast<float>(PointsToScanWithOneLaser - 1);

        // Base horizontal sweep, centered on sensor’s forward axis
        const float baseHorizDeg =
            -Description.HorizontalFov * 0.5f + static_cast<float>(idxPtsOneLaser) * stepDeg;

        // Per-channel azimuth offset (manual ±2.4° / ±0.65° pattern)
        const float offsetDeg = ChannelHorizOffsetsRad.IsValidIndex(idxChannel)
                                  ? FMath::RadiansToDegrees(ChannelHorizOffsetsRad[idxChannel])
                                  : 0.0f;

        // Final horizontal angle in radians
        const float HorizAngle = baseHorizDeg + offsetDeg;   // ne DegreesToRadians


        // // --- Optional DEBUG: log first few channels once per sweep ---
        // if ((idxPtsOneLaser % 300) == 0 && idxChannel == 0) {
        //   UE_LOG(LogCarla, Warning,
        //         TEXT("CH%03d Vert=%.2f° BaseHoriz=%.2f° Step=%.3f° Offset=%.2f° Total=%.2f°"),
        //         idxChannel,
        //         FMath::RadiansToDegrees(VertAngle),
        //         baseHorizDeg,
        //         stepDeg,
        //         offsetDeg,
        //         baseHorizDeg + offsetDeg);
        // }
        // // --------------------------------------------------------------


        const bool PreprocessResult = RayPreprocessCondition[idxChannel][idxPtsOneLaser];

        if (PreprocessResult && ShootLaser(VertAngle, HorizAngle, HitResult, TraceParams)) {
          WritePointAsync(idxChannel, HitResult);
        }
      }
    });
  }
  GetWorld()->GetPhysicsScene()->GetPxScene()->unlockRead();

  FTransform ActorTransf = GetTransform();
  ComputeAndSaveDetections(ActorTransf);
}
