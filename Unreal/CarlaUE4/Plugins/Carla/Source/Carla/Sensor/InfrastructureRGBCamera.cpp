// Copyright (c) 2017 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "Carla.h"
#include "Carla/Sensor/InfrastructureRGBCamera.h"
#include "Carla/Sensor/SceneCaptureCamera.h"
#include "Carla/Game/CarlaEngine.h"
#include <chrono>

#include "Runtime/RenderCore/Public/RenderingThread.h"

FActorDefinition AInfrastructureRGBCamera::GetSensorDefinition()
{
    constexpr bool bEnableModifyingPostProcessEffects = true;
    return UActorBlueprintFunctionLibrary::MakeCameraDefinition(
        TEXT("infrastructure"),
        bEnableModifyingPostProcessEffects);
}

AInfrastructureRGBCamera::AInfrastructureRGBCamera(const FObjectInitializer& ObjectInitializer)
    : Super(ObjectInitializer)
{
    // Create mesh component
    VisualMesh = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("VisualMesh"));

    // Attach it to root (or use SetRootComponent if you want)
    VisualMesh->SetupAttachment(RootComponent);
    // Optionally disable shadows for performance
    VisualMesh->SetCastShadow(false);

    // Load the mesh asset by path (change the path to your actual imported mesh)
    static ConstructorHelpers::FObjectFinder<UStaticMesh> MeshAsset(TEXT("/Game/Carla/Blueprints/Sensors/cctv2.cctv2"));

    if (MeshAsset.Succeeded())
    {
        VisualMesh->SetStaticMesh(MeshAsset.Object);
        // Or combine in one transform:
        // Move mesh 50 units forward (X axis), 0 right (Y), 20 up (Z)
        // VisualMesh->SetRelativeLocation(FVector(-10.0f, 0.0f, 0.0f));

        // // Rotate mesh 90 degrees around Z (Yaw)
        // VisualMesh->SetRelativeRotation(FRotator(0.0f, -90.0f, 0.0f));
    }
    AddPostProcessingMaterial(
        TEXT("Material'/Carla/PostProcessingMaterials/PhysicLensDistortion.PhysicLensDistortion'"));
}

void AInfrastructureRGBCamera::BeginPlay()
{
  Super::BeginPlay();
}

void AInfrastructureRGBCamera::OnFirstClientConnected()
{
}

void AInfrastructureRGBCamera::OnLastClientDisconnected()
{
}

void AInfrastructureRGBCamera::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
  Super::EndPlay(EndPlayReason);
}

void AInfrastructureRGBCamera::PostPhysTick(UWorld *World, ELevelTick TickType, float DeltaSeconds)
{
  TRACE_CPUPROFILER_EVENT_SCOPE(AInfrastructureRGBCamera::PostPhysTick);
  ENQUEUE_RENDER_COMMAND(MeasureTime)
  (
    [](auto &InRHICmdList)
    {
      std::chrono::time_point<std::chrono::high_resolution_clock> Time = 
          std::chrono::high_resolution_clock::now();
      auto Duration = std::chrono::duration_cast< std::chrono::milliseconds >(Time.time_since_epoch());
      uint64_t Milliseconds = Duration.count();
      FString ProfilerText = FString("(Render)Frame: ") + FString::FromInt(FCarlaEngine::GetFrameCounter()) + 
          FString(" Time: ") + FString::FromInt(Milliseconds);
      TRACE_CPUPROFILER_EVENT_SCOPE_TEXT(*ProfilerText);
    }
  );
  FPixelReader::SendPixelsInRenderThread<AInfrastructureRGBCamera, FColor>(*this);
}

void AInfrastructureRGBCamera::SendGBufferTextures(FGBufferRequest& GBuffer)
{
    SendGBufferTexturesInternal(*this, GBuffer);
}
