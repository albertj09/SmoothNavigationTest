// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "GoalActor.generated.h"

DECLARE_DYNAMIC_MULTICAST_DELEGATE(FOnConstructionDel);

UCLASS()
class SMOOTHNAVIGATIONTEST_API AGoalActor : public AActor
{
	GENERATED_BODY()
	
public:	
	AGoalActor();
	
	UPROPERTY(EditAnywhere)
	TObjectPtr<UStaticMeshComponent> MeshComponent = nullptr;

	UPROPERTY()
	FOnConstructionDel OnConstructionEvent;

protected:
	virtual void OnConstruction(const FTransform& Transform) override;
};
