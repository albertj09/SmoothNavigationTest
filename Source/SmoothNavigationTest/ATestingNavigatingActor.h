// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "Math/UnrealMathUtility.h"
#include "ATestingNavigatingActor.generated.h"

class UNavigationSystemV1;
class UDebugStringsComponent;
class AGoalActor;
class ARecastNavMesh;

UENUM(BlueprintType)
enum class ENavPathDrawType : uint8 {
	Points = 0	UMETA(DisplayName = "Only Points"),
	Lines = 1	UMETA(DisplayName = "Only Lines"),
	PointsAndLines = 2	UMETA(DisplayName = "Points And Lines"),
};

enum class EAngleUnits : uint8 {
	Degrees = 0,
	Radians = 1	
};

template <typename VectorType>
float GetAngleBetweenUnitVectors(const VectorType& a, const VectorType& b, EAngleUnits units = EAngleUnits::Radians)
{
	static_assert(std::is_same_v<FVector, VectorType>, "Only vector types supported");
	
	const float dotProduct = FVector::DotProduct(a, b);
	const float angleInRadians = FMath::Acos(dotProduct);
	return units == EAngleUnits::Radians ? angleInRadians : FMath::RadiansToDegrees(angleInRadians);
}

template<typename T>
inline T GetBezierPoint(float t, T P0, T P1, T P2) {
	float u = 1 - t;
	float tt = t * t;
	float uu = u * u;
	T P = uu * P0;
	P += 2 * u * t * P1;
	P += tt * P2;
	return P;
}

template<typename T>
inline T GetCubicBezierPoint(float t, T P0, T P1, T P2, T P3) {
	float u = 1 - t;
	float tt = t * t;
	float uu = u * u;
	float uuu = uu * u;
	float ttt = tt * t;
	T P = uuu * P0; 
	P += 3 * uu * t * P1; 
	P += 3 * u * tt * P2; 
	P += ttt * P3; 
	return P;
}

USTRUCT(BlueprintType)
struct FSmoothNavPathConfig
{
	GENERATED_BODY()

	// How far along the direction vector will the first bias point be offset (resulting distance = full distance * Bias1_DistanceScalar)
	UPROPERTY(EditAnywhere, Category="First Bias", meta=(ClampMin=0.1, ClampMax=1.f, UIMin = 0.1, UIMax = 1.f))
	float Bias1_DistanceScalar = 0.5f;

	// Max distance of how far along the direction vector will the second bias point be offset
	UPROPERTY(EditAnywhere, Category="Second Bias", meta=(ClampMin=0.0, UIMin = 0.0, UIMax = 1000.f))
	float Bias2_MaxDistanceOffset = 500.f;

	// Min distance of how far along the direction vector will the second bias point be offset
	UPROPERTY(EditAnywhere, Category="Second Bias", meta=(ClampMin=0.0, UIMin = 0.0, UIMax = 300.f))
	float Bias2_MinDistanceOffset = 50.f;

	// A configurable threshold for when the smooth path should attempt to skip certain nav points to maintain a smoother integrity (VERY EXPERIMENTAL)
	UPROPERTY(EditAnywhere, Category="Nav Point Skipping", meta=(InlineEditConditionToggle))
	bool bNavPointSkipping = true;

	// A configurable threshold for when the smooth path should attempt to skip certain nav points to maintain a smoother integrity (VERY EXPERIMENTAL)
	UPROPERTY(EditAnywhere, Category="Nav Point Skipping", meta=(EditCondition="bNavPointSkipping", ClampMin=0.f, UIMin = 0.f, UIMax = 90.f))
	float MinAngleSkipThreshold = 20.f;

	// A small offset to apply to next point in order to smooth out the turns more and avoid some inconsistencies
	UPROPERTY(EditAnywhere, meta=(ClampMin=0.f, UIMin = 0.f, UIMax = 100.f))
	float NextPointOffset = 50.0f;

	// Return to default smooth path config values
	UPROPERTY(EditAnywhere)
	bool bResetToDefaultConfigValues = false;

	// Extra debugging information. For now I am just toggling everything, but it's going to be separated out in the future
	UPROPERTY(EditAnywhere, Category="Debugging")
	bool bEnableExtraDebugInfo = false;

	void ResetToDefaults()
	{
		Bias1_DistanceScalar = 0.5f;
		Bias2_MaxDistanceOffset = 500.f;
		Bias2_MinDistanceOffset = 50.f;
		bNavPointSkipping = true;
		MinAngleSkipThreshold = 20.f;
		NextPointOffset = 50.0f;
		bResetToDefaultConfigValues = false;
		bEnableExtraDebugInfo = false;
	}
};

UCLASS()
class SMOOTHNAVIGATIONTEST_API AATestingNavigatingActor : public AActor
{
	GENERATED_BODY()
	
public:	

	AATestingNavigatingActor();

	UPROPERTY(EditAnywhere)
	TObjectPtr<UStaticMeshComponent> MeshComponent = nullptr;

	UPROPERTY()
	TObjectPtr<UDebugStringsComponent> DebugStringsComponent = nullptr;

	UPROPERTY(EditAnywhere, Category="Smooth Path")
	TObjectPtr<AGoalActor> GoalActor = nullptr;

	UPROPERTY(EditAnywhere, Category="Smooth Path")
	bool bRecalculateSmoothPath = false;

	UPROPERTY(EditAnywhere, Category="Smooth Path|Debug")
	ENavPathDrawType NavPathDrawType = ENavPathDrawType::Points;

	UPROPERTY(EditAnywhere, Category="Smooth Path")
	FSmoothNavPathConfig SmoothPathConfigurator;

	/** "None" will result in default filter being used */
	UPROPERTY(EditAnywhere, Category = Pathfinding)
	TSubclassOf<UNavigationQueryFilter> NavigationFilterClass;

protected:

#if WITH_EDITOR
	virtual void PostEditChangeProperty(struct FPropertyChangedEvent& PropertyChangedEvent) override;
#endif

	virtual void OnConstruction(const FTransform& Transform) override;

	UFUNCTION()
	void GeneratePath();
	
	TArray<FVector> SmoothPath(FNavPathSharedPtr path);

	// Simple debug draw for the generated path
	void DebugDrawNavigationPath(const TArray<FVector>& pathPoints, const FColor& color) const;
	void DebugDrawNavigationPath(const TArray<FNavPathPoint>& pathPoints, const FColor& color) const;

	// Custom helper functions
	void GetClosestPointOnNearbyPolys(NavNodeRef originalPoly, const FVector& testPt, FVector& pointOnPoly) const;
	bool IsSegmentIsFullyOnNavmesh(const FVector& segmentStart, const FVector& segmentEnd, FVector& hitLocation) const;
	bool IsSegmentIsFullyOnNavmesh(const FVector& segmentStart, const FVector& segmentEnd) const;
	void CalculateFirstBiasPoint(FVector& bias, const FNavPathPoint& currentPoint, const FNavPathPoint& nextPoint, const TArray<FNavPathPoint>& originalPathPoints, const TArray<FVector>& smoothPathPoints) const;
	
private:

	UPROPERTY()
	TObjectPtr<ANavigationData> NavigationData = nullptr;

	UPROPERTY()
	TObjectPtr<UNavigationSystemV1> NavSystem = nullptr;

	UPROPERTY()
	TObjectPtr<ARecastNavMesh> RecastNavMesh = nullptr;
};
