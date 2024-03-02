// Fill out your copyright notice in the Description page of Project Settings.
 
#include "DebugStringsComponent.h"
#include "Engine/Canvas.h"

FDebugSceneProxy::FDebugSceneProxy(const UPrimitiveComponent* InComponent, FDebugSceneProxyData* ProxyData)
 : FDebugRenderSceneProxy(InComponent)
{
	DrawType = EDrawType::SolidAndWireMeshes;
	ViewFlagName = TEXT("DebugText");
	this->ProxyData = ProxyData ? *ProxyData : FDebugSceneProxyData();
	
	if(TArray<FDebugSceneProxyData::FDebugText>* proxyDataDebugLabels = ProxyData->DebugLabels)
	{
		for(const FDebugSceneProxyData::FDebugText& Text : *proxyDataDebugLabels)
		{
			this->Texts.Emplace(FText3d(
				Text.Text,
				Text.Location,
				Text.Color
			));
		}
	}
}

void FDebugTextDelegateHelper::DrawDebugLabels(UCanvas* Canvas, APlayerController* PlayerController)
{
	if(!Canvas) return;

	const FColor OldDrawColor = Canvas->DrawColor;
	const FSceneView* View = Canvas->SceneView;
	const UFont* Font = UEngine::GetSmallFont();
	const FDebugSceneProxyData::FDebugText* DebugText = DebugLabels.GetData();

	for(int32 i = 0; i < DebugLabels.Num(); ++i, ++DebugText)
	{
		if(View->ViewFrustum.IntersectSphere(DebugText->Location, 1.0f))
		{
			const FVector ScreenLoc = Canvas->Project(DebugText->Location);
			Canvas->SetDrawColor(DebugText->Color);
			Canvas->DrawText(Font, DebugText->Text, ScreenLoc.X, ScreenLoc.Y, DebugText->FontSize, DebugText->FontSize);
		}
	}

	Canvas->SetDrawColor(OldDrawColor);
}

void FDebugTextDelegateHelper::SetupFromProxy(const FDebugSceneProxy* InSceneProxy)
{
	DebugLabels.Reset();
	if(const TArray<FDebugSceneProxyData::FDebugText>* debugLabels = InSceneProxy->ProxyData.DebugLabels)
	{
		DebugLabels.Append(*debugLabels);
	}
} 

UDebugStringsComponent::UDebugStringsComponent(const FObjectInitializer& ObjectInitializer) : Super(ObjectInitializer)
{
	FEngineShowFlags::RegisterCustomShowFlag(TEXT("DebugText"), false, SFG_Normal,
		FText::FromString("Debug Text"));
}

void UDebugStringsComponent::DrawDebugStringAtLocation(const FString& Text, const FColor& Color, const float FontSize, const FVector& Location)
{
	const FDebugSceneProxyData::FDebugText NewDebugString(Location, Text, Color, FontSize);
	DebugTexts.Emplace(NewDebugString);

	if(!IsRenderStateDirty())
	{
		MarkRenderStateDirty();
	}
}

void UDebugStringsComponent::ClearDebugText()
{
	DebugTexts.Empty();
	MarkRenderStateDirty();
}

FDebugRenderSceneProxy* UDebugStringsComponent::CreateDebugSceneProxy()
{
	FDebugSceneProxyData ProxyData;
	ProxyData.DebugLabels = &DebugTexts;
	FDebugSceneProxy* DebugSceneProxy = new FDebugSceneProxy(this, &ProxyData);
	DebugDrawDelegateManager.SetupFromProxy(DebugSceneProxy);

	return DebugSceneProxy;
}

FBoxSphereBounds UDebugStringsComponent::CalcBounds(const FTransform& LocalToWorld) const
{
	return FBoxSphereBounds(FBox(FVector(-1000, -1000, -1000), FVector(1000, 1000, 1000)));
}

