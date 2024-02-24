// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "ATestingNavigatingActor.generated.h"

class AGoalActor;

UENUM(BlueprintType)
enum class ENavPathDrawType : uint8 {
	Points = 0	UMETA(DisplayName = "Only Points"),
	Lines = 1	UMETA(DisplayName = "Only Lines"),
	PointsAndLines = 2	UMETA(DisplayName = "Points And Lines"),
};

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

UCLASS()
class SMOOTHNAVIGATIONTEST_API AATestingNavigatingActor : public AActor
{
	GENERATED_BODY()
	
public:	

	AATestingNavigatingActor();

	UPROPERTY(EditAnywhere)
	TObjectPtr<UStaticMeshComponent> MeshComponent = nullptr;

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
};
