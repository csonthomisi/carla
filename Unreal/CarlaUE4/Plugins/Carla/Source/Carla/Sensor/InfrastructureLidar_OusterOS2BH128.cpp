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
#include "Carla/Sensor/InfrastructureLidar_OusterOS2BH128.h"
#include "Carla/Actor/ActorBlueprintFunctionLibrary.h"
#include "carla/geom/Math.h"
#include "Carla/Sensor/ouster_meta_parser.h"

#include <compiler/disable-ue4-macros.h>
#include "carla/geom/Math.h"
#include "carla/ros2/ROS2.h"
#include "carla/geom/Location.h"
#include <compiler/enable-ue4-macros.h>

#include "DrawDebugHelpers.h"
#include "Engine/CollisionProfile.h"
#include "Runtime/Engine/Classes/Kismet/KismetMathLibrary.h"

FActorDefinition AInfrastructureLidar_OusterOS2BH128::GetSensorDefinition()
{
  return UActorBlueprintFunctionLibrary::MakeLidarDefinition(TEXT("ouster_os2_bh_128"));
}


AInfrastructureLidar_OusterOS2BH128::AInfrastructureLidar_OusterOS2BH128(const FObjectInitializer& ObjectInitializer)
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
    static ConstructorHelpers::FObjectFinder<UStaticMesh> MeshAsset(TEXT("/Game/Carla/Blueprints/Sensors/Ouster.Ouster"));

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

void AInfrastructureLidar_OusterOS2BH128::LoadBeams()
{
    // Your custom logic
    FString Path = FPaths::ProjectContentDir() / TEXT("Carla/Blueprints/Sensors/OusterOS2BH128.json");
    // BeamAngles = LoadVerticalBeamAngles(Path);
    BeamAngles = LoadVerticalBeamAngles(Path, HorizontalResolution, RotationFrequency);

    if (BeamAngles.Num() > 0)
    {
      bUseMetadata = true;
      UE_LOG(LogTemp, Log, TEXT("Loaded %d beam angles from metadata."), BeamAngles.Num());
    }
    else
    {
      bUseMetadata = false;
      UE_LOG(LogTemp, Error, TEXT("Beam metadata not found or failed to parse. Falling back to default vertical angle calculation."));
    }
}

void AInfrastructureLidar_OusterOS2BH128::Set(const FActorDescription &ActorDescription)
{
  ASensor::Set(ActorDescription);
  FLidarDescription LidarDescription;
  UActorBlueprintFunctionLibrary::SetLidar(ActorDescription, LidarDescription);
  Set(LidarDescription);
}

void AInfrastructureLidar_OusterOS2BH128::Set(const FLidarDescription &LidarDescription)
{
  Description = LidarDescription;
  LoadBeams();
  if (bUseMetadata)
    {
      Description.Channels = BeamAngles.Num();
      UE_LOG(LogTemp, Warning, TEXT("Sanity Check: BeamAngles.Num() = %d, Channels = %d"), BeamAngles.Num(), Description.Channels);
      if (HorizontalResolution > 0 && RotationFrequency > 0)
      {
        UE_LOG(LogTemp, Log, TEXT("RotationFrequency=%d"),RotationFrequency);
        Description.RotationFrequency =static_cast<float>(RotationFrequency);// RotationFrequency;
        Description.PointsPerSecond = HorizontalResolution * RotationFrequency * Description.Channels;
        Description.UpperFovLimit = FMath::Max(BeamAngles);
        Description.LowerFovLimit = FMath::Min(BeamAngles);
        
        UE_LOG(LogTemp, Log, TEXT("Ouster Config used: RotationFrequency=%.2f"),Description.RotationFrequency);
        UE_LOG(LogTemp, Log, TEXT("Ouster Config used: HorizontalResolution=%d"),HorizontalResolution);
        UE_LOG(LogTemp, Log, TEXT("Ouster Config used: Channels=%d"),Description.Channels);
        UE_LOG(LogTemp, Log, TEXT("Ouster Config used: PointsPerSecond=%d"),Description.PointsPerSecond);
        UE_LOG(LogTemp, Log, TEXT("Ouster Config used: UpperFovLimit=%.2f"),Description.UpperFovLimit);
        UE_LOG(LogTemp, Log, TEXT("Ouster Config used: LowerFovLimit=%.2f"),Description.LowerFovLimit);
      }
    }
  LidarData = FLidarData(Description.Channels);
  CreateLasers();
  PointsPerChannel.resize(Description.Channels);

  // Compute drop off model parameters
  DropOffBeta = 1.0f - Description.DropOffAtZeroIntensity;
  DropOffAlpha = Description.DropOffAtZeroIntensity / Description.DropOffIntensityLimit;
  DropOffGenActive = Description.DropOffGenRate > std::numeric_limits<float>::epsilon();
}

void AInfrastructureLidar_OusterOS2BH128::PostPhysTick(UWorld *World, ELevelTick TickType, float DeltaTime)
{
  TRACE_CPUPROFILER_EVENT_SCOPE(AInfrastructureLidar_OusterOS2BH128::PostPhysTick);
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

float AInfrastructureLidar_OusterOS2BH128::ComputeIntensity(const FSemanticDetection& RawDetection) const
{
  const carla::geom::Location HitPoint = RawDetection.point;
  const float Distance = HitPoint.Length();

  const float AttenAtm = Description.AtmospAttenRate;
  const float AbsAtm = exp(-AttenAtm * Distance);

  const float IntRec = AbsAtm;

  return IntRec;
}

AInfrastructureLidar_OusterOS2BH128::FDetection AInfrastructureLidar_OusterOS2BH128::ComputeDetection(const FHitResult& HitInfo, const FTransform& SensorTransf) const
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

  void AInfrastructureLidar_OusterOS2BH128::PreprocessRays(uint32_t Channels, uint32_t MaxPointsPerChannel) {
    Super::PreprocessRays(Channels, MaxPointsPerChannel);

    for (auto ch = 0u; ch < Channels; ch++) {
      for (auto p = 0u; p < MaxPointsPerChannel; p++) {
        RayPreprocessCondition[ch][p] = !(DropOffGenActive && RandomEngine->GetUniformFloat() < Description.DropOffGenRate);
      }
    }
  }

  bool AInfrastructureLidar_OusterOS2BH128::PostprocessDetection(FDetection& Detection) const
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

  void AInfrastructureLidar_OusterOS2BH128::CreateLasers()
  {
    if (!bUseMetadata)
    {
      const auto NumberOfLasers = Description.Channels;
      check(NumberOfLasers > 0u);
      const float DeltaAngle = NumberOfLasers == 1u ? 0.f :
      (Description.UpperFovLimit - Description.LowerFovLimit) /
      static_cast<float>(NumberOfLasers - 1);
      LaserAngles.Empty(NumberOfLasers);
      for(auto i = 0u; i < NumberOfLasers; ++i)
      {
        const float VerticalAngle =
        Description.UpperFovLimit - static_cast<float>(i) * DeltaAngle;
        LaserAngles.Emplace(VerticalAngle);
      }
    }
    else
    {
      LaserAngles.Empty(BeamAngles.Num());
      for(int32 i = 0u; i < BeamAngles.Num(); ++i)
      {
        LaserAngles.Emplace(BeamAngles[i]);
      }
      // LaserAngles = BeamAngles;

    }
    UE_LOG(LogTemp, Log, TEXT("Beam used :%d"), LaserAngles.Num());
  }

  void AInfrastructureLidar_OusterOS2BH128::ComputeAndSaveDetections(const FTransform& SensorTransform) {
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
