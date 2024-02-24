// Fill out your copyright notice in the Description page of Project Settings.

#include "ATestingNavigatingActor.h"
#include "GoalActor.h"
#include "NavigationSystem.h"
#include "NavMesh/RecastNavMesh.h"

AATestingNavigatingActor::AATestingNavigatingActor()
{
	PrimaryActorTick.bCanEverTick = false;
	MeshComponent = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("MeshComponent"));
	RootComponent = MeshComponent;
}

#if WITH_EDITOR
void AATestingNavigatingActor::PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent)
{
	Super::PostEditChangeProperty(PropertyChangedEvent);

	// Path recalculation triggered either by changes to bRecalculateSmoothPath, NavPathDrawType or GoalActor
	if ((PropertyChangedEvent.Property != nullptr && PropertyChangedEvent.Property->GetFName() == GET_MEMBER_NAME_CHECKED(AATestingNavigatingActor, bRecalculateSmoothPath)) 
		|| (PropertyChangedEvent.Property != nullptr && PropertyChangedEvent.Property->GetFName() == GET_MEMBER_NAME_CHECKED(AATestingNavigatingActor, NavPathDrawType))
		|| (PropertyChangedEvent.Property != nullptr && PropertyChangedEvent.Property->GetFName() == GET_MEMBER_NAME_CHECKED(AATestingNavigatingActor, GoalActor)))
	{
		GeneratePath();
	}
}
#endif

void AATestingNavigatingActor::OnConstruction(const FTransform& Transform)
{
	Super::OnConstruction(Transform);

	GeneratePath();
}

void AATestingNavigatingActor::GeneratePath()
{
	if (UNavigationSystemV1* navSys = FNavigationSystem::GetCurrent<UNavigationSystemV1>(GetWorld()))
	{
		const ANavigationData* navData = navSys->GetDefaultNavDataInstance();
		if (navData && IsValid(GoalActor))
		{
			if(!GoalActor->OnConstructionEvent.IsAlreadyBound(this, &AATestingNavigatingActor::GeneratePath))
			{
				GoalActor->OnConstructionEvent.AddUniqueDynamic(this, &AATestingNavigatingActor::GeneratePath);
			}
			
			// Get the optimal navigation path from the engine
			const FPathFindingResult pathFindingResult = navSys->FindPathSync(FPathFindingQuery(this, *navData, GetActorLocation(), GoalActor->GetActorLocation(), UNavigationQueryFilter::GetQueryFilter(*navData, this, NavigationFilterClass), nullptr, UE_BIG_NUMBER, true));
			if (pathFindingResult.IsSuccessful())
			{
				SmoothPath(pathFindingResult.Path);
			}
		}
	}

	// Reset the bool
	if (bRecalculateSmoothPath)
	{
		bRecalculateSmoothPath = false;
	}
}

TArray<FVector> AATestingNavigatingActor::SmoothPath(FNavPathSharedPtr path)
{
	
	if (FNavigationPath* navPath = path.Get()) 
	{
		GEngine->AddOnScreenDebugMessage(-1, 5.0f, FColor::Cyan, TEXT("SmoothPathRecalculation..."));

		UNavigationSystemV1* navSys = FNavigationSystem::GetCurrent<UNavigationSystemV1>(GetWorld());
		ARecastNavMesh* navMesh = Cast<ARecastNavMesh>(UNavigationSystemV1::GetCurrent(GetWorld())->GetDefaultNavDataInstance());

		ensure(navSys);
		ensure(navMesh);

		TArray<FNavPathPoint> navPathPoints = navPath->GetPathPoints();
		TArray<FVector> navPoints;
		navPoints.Reserve(navPathPoints.Num());
		int32 numLocations = navPoints.Num();
		
		// Transform to an array FVector points so it's more convenient to work with
		Algo::Transform(navPathPoints, navPoints, [](const FNavPathPoint& navPathPoint) { return navPathPoint.Location; });

		// Flush all previous debug drawing
		FlushPersistentDebugLines(GetWorld());

		// Draw the optimal non smoothed engine path
		DebugDrawNavigationPath(navPoints, FColor::Blue);

		////MIDPOINTS IMPLEMENTATION (shown in the report but unused)
		//TArray<FVector> polygonMidpointSegments;
		//TArray<FNavigationPortalEdge> pathCorridorEdges;
		//FNavMeshPath* NavMeshPath = static_cast<FNavMeshPath*>(Path.Get());
		//if (NavMeshPath)
		//{
		//	// Get the corridor edges from the path
		//	pathCorridorEdges = NavMeshPath->GetPathCorridorEdges();
		//}

		//// Generate an alternative path using corridor segment midpoints
		//polygonMidpointSegments.Emplace(navPoints[0]);
		//for (auto& pathCorridorEdge : pathCorridorEdges)
		//{
		//	polygonMidpointSegments.Emplace(pathCorridorEdge.GetMiddlePoint());
		//}
		//polygonMidpointSegments.Emplace(navPoints.Last());

		// Smoothing of the points
		//BEZIER
		TArray<FVector> bezierSmoothedLocations;
		for (int32 i = 0; i < navPoints.Num(); i++)
		{
			// We are generating the point from current to next, so the last point is already generated
			if (i + 1 == navPoints.Num()) {
				break;
			}

			FVector currentP = navPoints[i];
			FVector nextP = navPoints[i + 1];

			FVector bias = nextP - currentP;
			if (i != 0) {
				bias = bezierSmoothedLocations[bezierSmoothedLocations.Num() - 1] - bezierSmoothedLocations[bezierSmoothedLocations.Num() - 2];
			}

			const float halfDistance = FVector::Dist(currentP, nextP) / 2.0f;
			bias.Normalize();
			bias *= (halfDistance / 1.3);
			bias += currentP;

			/*DrawDebugLine(GetWorld(), currentP, bias, FColor::Red, true, -1.f, 0);
			DrawDebugPoint(GetWorld(), bias, 16.f, FColor::Red, true, -1.f, 0);*/

			for (float t = 0.0; t <= 1.0; t += 0.1f) {
				FVector pointOnCurve = GetBezierPoint(t, currentP, bias, nextP);
				FNavLocation projectedPointOnCurve;

				// Prevent out of navmesh boundaries... This is a bad way but I am not sure how to do it with corridor edges
				if (navSys->ProjectPointToNavigation(pointOnCurve, projectedPointOnCurve)) {
					bezierSmoothedLocations.Emplace(projectedPointOnCurve);
				}
			}
		}

		bezierSmoothedLocations.Emplace(navPoints.Last());

		// Debug draw the smoothed path
		DebugDrawNavigationPath(bezierSmoothedLocations, FColor::Cyan);

		return bezierSmoothedLocations;
	}

	// No nav path?
	return {};
}

void AATestingNavigatingActor::DebugDrawNavigationPath(const TArray<FVector>& pathPoints, const FColor& color) const
{
	if (!pathPoints.IsEmpty()) 
	{
		switch (NavPathDrawType)
		{
		case ENavPathDrawType::Points:
			{
				for (const FVector& pointLocation : pathPoints)
				{
					DrawDebugPoint(GetWorld(), pointLocation, 16.f, color, true);
				}
			}
			break;

		case ENavPathDrawType::Lines:
			{
				for (int32 i = 0; i < pathPoints.Num() - 1; ++i)
				{
					DrawDebugLine(GetWorld(), pathPoints[i], pathPoints[i + 1], color, true, -1.f, 0, 3.f);
				}
			}
			break;

		case ENavPathDrawType::PointsAndLines:
			{
				for (int32 i = 0; i < pathPoints.Num() - 1; ++i)
				{
					DrawDebugPoint(GetWorld(), pathPoints[i], 16.f, color, true);
					DrawDebugLine(GetWorld(), pathPoints[i], pathPoints[i + 1], color, true, -1.f, 0, 4.f);
				}
			}
			break;
		}	
	}
}
