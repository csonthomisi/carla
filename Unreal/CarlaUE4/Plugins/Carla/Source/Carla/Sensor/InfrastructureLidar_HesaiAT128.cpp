// StaticMesh'/Game/Carla/Blueprints/Sensors/AT128PFOV.AT128PFOV'
// StaticMesh'/Game/Carla/Blueprints/Sensors/Ouster.Ouster'
// Copyright (c) 2017 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include <PxScene.h>
#include <cmath>
#include "Carla.h"
#include "Carla/Sensor/InfrastructureLidar_HesaiAT128.h"
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

FActorDefinition AInfrastructureLidar_HesaiAT128::GetSensorDefinition()
{
  return UActorBlueprintFunctionLibrary::MakeLidarDefinition(TEXT("hesai_at128"));
}


AInfrastructureLidar_HesaiAT128::AInfrastructureLidar_HesaiAT128(const FObjectInitializer& ObjectInitializer)
  : Super(ObjectInitializer) {
    // Create mesh component
    VisualMesh = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("VisualMesh"));

    // Attach it to root (or use SetRootComponent if you want)
    VisualMesh->SetupAttachment(RootComponent);
    // Optionally disable shadows for performance
    VisualMesh->SetCastShadow(false);
    // Disable all collisions on the mesh so Lidar rays don't hit it
    VisualMesh->SetCollisionEnabled(ECollisionEnabled::NoCollision);
    VisualMesh->SetGenerateOverlapEvents(false);
    VisualMesh->SetCanEverAffectNavigation(false);

    // Load the mesh asset by path (change the path to your actual imported mesh)
    static ConstructorHelpers::FObjectFinder<UStaticMesh> MeshAsset(TEXT("/Game/Carla/Blueprints/Sensors/AT128PFOV.AT128PFOV"));

    if (MeshAsset.Succeeded())
    {
        VisualMesh->SetStaticMesh(MeshAsset.Object);
        // Or combine in one transform:
        // Move mesh 50 units forward (X axis), 0 right (Y), 20 up (Z)
        // VisualMesh->SetRelativeLocation(FVector(0.0f, 0.0f, 0.0f));

        // // Rotate mesh 90 degrees around Z (Yaw)
        // VisualMesh->SetRelativeRotation(FRotator(0.0f, 0.0f, 0.0f));
    }
    
    RandomEngine = CreateDefaultSubobject<URandomEngine>(TEXT("RandomEngine"));
    SetSeed(Description.RandomSeed);
}

void AInfrastructureLidar_HesaiAT128::Set(const FActorDescription &ActorDescription)
{
  ASensor::Set(ActorDescription);
  FLidarDescription LidarDescription;
  UActorBlueprintFunctionLibrary::SetLidar(ActorDescription, LidarDescription);
  Set(LidarDescription);
}

void AInfrastructureLidar_HesaiAT128::Set(const FLidarDescription &LidarDescription)
{
  Description = LidarDescription;
  LidarData = FLidarData(Description.Channels);
  CreateLasers();
  PointsPerChannel.resize(Description.Channels);

  // Compute drop off model parameters
  DropOffBeta = 1.0f - Description.DropOffAtZeroIntensity;
  DropOffAlpha = Description.DropOffAtZeroIntensity / Description.DropOffIntensityLimit;
  DropOffGenActive = Description.DropOffGenRate > std::numeric_limits<float>::epsilon();
}

void AInfrastructureLidar_HesaiAT128::PostPhysTick(UWorld *World, ELevelTick TickType, float DeltaTime)
{
  TRACE_CPUPROFILER_EVENT_SCOPE(AInfrastructureLidar_HesaiAT128::PostPhysTick);
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

float AInfrastructureLidar_HesaiAT128::ComputeIntensity(const FSemanticDetection& RawDetection) const
{
  const carla::geom::Location HitPoint = RawDetection.point;
  const float Distance = HitPoint.Length();

  const float AttenAtm = Description.AtmospAttenRate;
  const float AbsAtm = exp(-AttenAtm * Distance);

  const float IntRec = AbsAtm;

  return IntRec;
}

AInfrastructureLidar_HesaiAT128::FDetection AInfrastructureLidar_HesaiAT128::ComputeDetection(const FHitResult& HitInfo, const FTransform& SensorTransf) const
{
  FDetection Detection;
  const FVector HitPoint = HitInfo.ImpactPoint;
  Detection.point = SensorTransf.Inverse().TransformPosition(HitPoint);

  const float Distance = Detection.point.Length();

  const float AttenAtm = Description.AtmospAttenRate;
  const float AbsAtm = exp(-AttenAtm * Distance);

  const float IntRec = AbsAtm;

  Detection.intensity = IntRec;

  return Detection;
}

  void AInfrastructureLidar_HesaiAT128::PreprocessRays(uint32_t Channels, uint32_t MaxPointsPerChannel) {
    Super::PreprocessRays(Channels, MaxPointsPerChannel);

    for (auto ch = 0u; ch < Channels; ch++) {
      for (auto p = 0u; p < MaxPointsPerChannel; p++) {
        RayPreprocessCondition[ch][p] = !(DropOffGenActive && RandomEngine->GetUniformFloat() < Description.DropOffGenRate);
      }
    }
  }

  bool AInfrastructureLidar_HesaiAT128::PostprocessDetection(FDetection& Detection) const
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

  void AInfrastructureLidar_HesaiAT128::ComputeAndSaveDetections(const FTransform& SensorTransform) {
    for (auto idxChannel = 0u; idxChannel < Description.Channels; ++idxChannel)
      PointsPerChannel[idxChannel] = RecordedHits[idxChannel].size();

    LidarData.ResetMemory(PointsPerChannel);

    for (auto idxChannel = 0u; idxChannel < Description.Channels; ++idxChannel) {
      for (auto& hit : RecordedHits[idxChannel]) {
        FDetection Detection = ComputeDetection(hit, SensorTransform);
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

  Main simulation function called every tick. Casts rays across a fixed horizontal
  field of view (-60° to +60°) with 0.1° resolution, and for each vertical laser
  (from LaserAngles). No continuous rotation is used — this lidar performs a fixed
  120° sweep relative to the sensor's forward axis.

  Steps:
    - Validates laser/channel setup.
    - Prepares ray preprocessing and hit recording structures.
    - For each vertical channel, casts 1200 rays (0.1° resolution) across horizontal sweep.
    - Records hits per channel if they pass preprocessing.
    - Computes and stores detections from the hit results.

  This function defines the horizontal scanning behavior of the sensor.
*/
void AInfrastructureLidar_HesaiAT128::SimulateLidar(const float DeltaTime)
{
  TRACE_CPUPROFILER_EVENT_SCOPE(AInfrastructureLidar_HesaiAT128::SimulateLidar);
  const uint32 ChannelCount = Description.Channels;
  const uint32 PointsToScanWithOneLaser = 1200; // 0.1° resolution over 120°

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
        const float HorizAngle = -60.0f + idxPtsOneLaser * 0.1f;
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

  //SemanticLidarData.SetHorizontalAngle(0.0f); // Fixed angle
}


