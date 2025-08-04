#pragma once

#include "CoreMinimal.h"

// Expose the function to other classes
// TArray<float> LoadVerticalBeamAngles(const FString& FilePath);
TArray<float> LoadVerticalBeamAngles(const FString& FilePath, int32& OutHorizontalResolution, int32& OutRotationFrequency);
