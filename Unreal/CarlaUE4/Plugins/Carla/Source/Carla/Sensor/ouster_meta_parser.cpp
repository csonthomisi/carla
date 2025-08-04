#include "Carla/Sensor/ouster_meta_parser.h"  // Include your header
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"
#include "Dom/JsonObject.h"
#include "Dom/JsonValue.h"

TArray<float> LoadVerticalBeamAngles(const FString& FilePath, int32& OutHorizontalResolution,int32& OutRotationFrequency)
{
    FString JsonRaw;
    TArray<float> BeamAngles;

    if (!FFileHelper::LoadFileToString(JsonRaw, *FilePath))
    {
        UE_LOG(LogTemp, Error, TEXT("Failed to load file: %s"), *FilePath);
        return BeamAngles;
    }

    TSharedPtr<FJsonObject> JsonParsed;
    TSharedRef<TJsonReader<>> Reader = TJsonReaderFactory<>::Create(JsonRaw);

    if (!FJsonSerializer::Deserialize(Reader, JsonParsed) || !JsonParsed.IsValid())
    {
        UE_LOG(LogTemp, Error, TEXT("Failed to parse JSON file"));
        return BeamAngles;
    }

    // Navigate to beam_altitude_angles
    TArray<TSharedPtr<FJsonValue>> AltitudeArray = JsonParsed
        ->GetObjectField("beam_intrinsics")
        ->GetArrayField("beam_altitude_angles");

    for (const auto& Value : AltitudeArray)
    {
        BeamAngles.Add(Value->AsNumber());
    }

    // Optional: Log them
    for (int32 i = 0; i < BeamAngles.Num(); ++i)
    {
        UE_LOG(LogTemp, Log, TEXT("Beam %d: %f degrees"), i, BeamAngles[i]);
    }

    // Parse lidar_mode from config_params
    OutHorizontalResolution = 0;
    OutRotationFrequency = 0;
    const TSharedPtr<FJsonObject> ConfigParams = JsonParsed->GetObjectField("config_params");
    FString LidarModeStr;
    if (ConfigParams->TryGetStringField("lidar_mode", LidarModeStr))
    {
        TArray<FString> ModeParts;
        LidarModeStr.ParseIntoArray(ModeParts, TEXT("x"), true);
        if (ModeParts.Num() == 2)
        {
            OutHorizontalResolution = FCString::Atoi(*ModeParts[0]);
            OutRotationFrequency = FCString::Atoi(*ModeParts[1]);
            UE_LOG(LogTemp, Log, TEXT("Parsed lidar_mode: %s → Horizontal: %d, Frequency: %d"), *LidarModeStr, OutHorizontalResolution, OutRotationFrequency);
        }
        else
        {
            UE_LOG(LogTemp, Warning, TEXT("Invalid lidar_mode format: %s"), *LidarModeStr);
        }
    }
    else
    {
        UE_LOG(LogTemp, Warning, TEXT("Could not find 'lidar_mode' in config_params"));
    }

    return BeamAngles;
}