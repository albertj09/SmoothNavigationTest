// Fill out your copyright notice in the Description page of Project Settings.

#include "GoalActor.h"

// Sets default values
AGoalActor::AGoalActor()
{
	PrimaryActorTick.bCanEverTick = false;
	MeshComponent = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("MeshComponent"));
	RootComponent = MeshComponent;
}

void AGoalActor::OnConstruction(const FTransform& Transform)
{
	Super::OnConstruction(Transform);

	OnConstructionEvent.Broadcast();
}
