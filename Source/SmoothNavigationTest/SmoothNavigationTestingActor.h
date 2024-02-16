// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "NavigationTestingActor.h"
#include "SmoothNavigationTestingActor.generated.h"

UCLASS()
class SMOOTHNAVIGATIONTEST_API ASmoothNavigationTestingActor : public ANavigationTestingActor
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere)
	bool bRecalculateSmoothPath = false;

protected:

#if WITH_EDITOR
	virtual void PostEditChangeProperty(struct FPropertyChangedEvent& PropertyChangedEvent) override;
#endif

	TArray<FVector> SmoothPath(FNavPathSharedPtr Path);
};
