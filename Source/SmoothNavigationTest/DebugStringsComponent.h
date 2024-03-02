// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "AIController.h"
#include "Debug/DebugDrawComponent.h"
#include "DebugStringsComponent.generated.h"

struct FDebugSceneProxyData
{
	struct FDebugText
	{
		FVector Location;
		FString Text;
		FColor Color;
		float FontSize = 1.f;

		FDebugText() {}
		FDebugText(const FVector& InLocation, const FString& InText, const FColor& InColor, const float InFontSize) : Location(InLocation), Text(InText), Color(InColor), FontSize(InFontSize)
		{
			if(!FAISystem::IsValidLocation(Location))
			{
				UE_LOG(LogTemp, Warning, TEXT("FDebugSceneProxyData::FDebugText constructor called with invalid location! Location: %s, Text: %s, Color: %s, FontSize: %f"), *Location.ToString(), *Text, *Color.ToString(), FontSize);
			}
		}
	};

	TArray<FDebugText>* DebugLabels;
};

class FDebugSceneProxy : public FDebugRenderSceneProxy
{
public:
	FDebugSceneProxy(const UPrimitiveComponent* InComponent, FDebugSceneProxyData* ProxyData);

	FDebugSceneProxyData ProxyData;
};

class FDebugTextDelegateHelper : public FDebugDrawDelegateHelper
{
public:
	virtual void DrawDebugLabels(UCanvas* Canvas, APlayerController*) override;

	void SetupFromProxy(const FDebugSceneProxy* InSceneProxy);

	TArray<FDebugSceneProxyData::FDebugText> DebugLabels;
};

UCLASS(ClassGroup = Custom, meta = (BlueprintSpawnableComponent))
class SMOOTHNAVIGATIONTEST_API UDebugStringsComponent : public UDebugDrawComponent
{
	GENERATED_BODY()
	
public:
	
	UDebugStringsComponent(const FObjectInitializer& ObjectInitializer = FObjectInitializer::Get());

	UFUNCTION(BlueprintCallable, Category = "Debug")
	void DrawDebugStringAtLocation(const FString& Text, const FColor& Color, const float FontSize, const FVector& Location);

	UFUNCTION(BlueprintCallable, Category = "Debug")
	void ClearDebugText();
	
	virtual FDebugRenderSceneProxy* CreateDebugSceneProxy() override;
	virtual FDebugDrawDelegateHelper& GetDebugDrawDelegateHelper() override { return DebugDrawDelegateManager; }
	virtual FBoxSphereBounds CalcBounds(const FTransform& LocalToWorld) const override;

private:
	
	FDebugTextDelegateHelper DebugDrawDelegateManager;
	TArray<FDebugSceneProxyData::FDebugText> DebugTexts;
};
