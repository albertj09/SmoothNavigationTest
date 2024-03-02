// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "Math/UnrealMathUtility.h"
#include "ATestingNavigatingActor.generated.h"

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


struct FCustomNavPathPoint 
{
	FNavPathPoint NavPathPoint;
	bool bSkipPathPoint = false;

	FCustomNavPathPoint(const FNavPathPoint& InNavPathPoint, bool InSkipPathPoint) : NavPathPoint(InNavPathPoint), bSkipPathPoint(InSkipPathPoint) {}
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

UCLASS()
class SMOOTHNAVIGATIONTEST_API AATestingNavigatingActor : public AActor
{
	GENERATED_BODY()
	
public:	

	AATestingNavigatingActor();

	UPROPERTY(EditAnywhere)
	TObjectPtr<UStaticMeshComponent> MeshComponent = nullptr;

	UPROPERTY(EditAnywhere)
	TObjectPtr<UDebugStringsComponent> DebugStringsComponent = nullptr;

	UPROPERTY(EditAnywhere)
	TObjectPtr<AGoalActor> GoalActor = nullptr;

	UPROPERTY(EditAnywhere)
	bool bRecalculateSmoothPath = false;

	UPROPERTY(EditAnywhere)
	ENavPathDrawType NavPathDrawType = ENavPathDrawType::Points;

	/** "None" will result in default filter being used */
	UPROPERTY(EditAnywhere, Category = Pathfinding)
	TSubclassOf<class UNavigationQueryFilter> NavigationFilterClass;

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
	void GetClosestPointOnNearbyPolys(ARecastNavMesh* recastNavMesh, NavNodeRef originalPoly, const FVector& testPt, FVector& pointOnPoly);
};
