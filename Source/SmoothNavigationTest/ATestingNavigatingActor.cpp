// Fill out your copyright notice in the Description page of Project Settings.

#include "ATestingNavigatingActor.h"
#include "GoalActor.h"
#include "NavigationSystem.h"
#include "NavMesh/RecastNavMesh.h"
#include "AITypes.h"
#include "DebugStringsComponent.h"

AATestingNavigatingActor::AATestingNavigatingActor()
{
	PrimaryActorTick.bCanEverTick = false;
	MeshComponent = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("MeshComponent"));
	DebugStringsComponent = CreateDefaultSubobject<UDebugStringsComponent>(TEXT("DebugStringsComponent"));
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
		UNavigationSystemV1* navSys = FNavigationSystem::GetCurrent<UNavigationSystemV1>(GetWorld());
		ARecastNavMesh* recastNavMesh = Cast<ARecastNavMesh>(UNavigationSystemV1::GetCurrent(GetWorld())->GetDefaultNavDataInstance());

		ensure(navSys);
		ensure(recastNavMesh);

		TArray<FNavPathPoint> navPathPoints = navPath->GetPathPoints();

		// Flush all previous debug drawing
		FlushPersistentDebugLines(GetWorld());
		if(DebugStringsComponent)
		{
			DebugStringsComponent->ClearDebugText();
		}
		
		// Draw the optimal non smoothed engine path
		DebugDrawNavigationPath(navPathPoints, FColor::Blue);

		// // Navmesh path and corridor edges
		// TArray<FNavigationPortalEdge> pathCorridorEdges;
		// FNavMeshPath* NavMeshPath = static_cast<FNavMeshPath*>(path.Get());
		// if (NavMeshPath)
		// {
		// 	// Get the corridor edges from the path
		// 	pathCorridorEdges = NavMeshPath->GetPathCorridorEdges();
		// }
		
		// Smoothing of the points with a custom algorithm including Bezier interpolation
		TArray<FVector> bezierSmoothedLocations;
		for (int32 i = 0; i < navPathPoints.Num(); i++)
		{
			// We are generating the point from current to next, so the last point is already generated
			if (i + 1 == navPathPoints.Num()) {
				break;
			}

			// Experimental bias. We need to start with some sort of curve.
			FNavPathPoint currentP = navPathPoints[i];
			FNavPathPoint nextP = navPathPoints[i + 1];
			FVector experimentalBias = nextP.Location - currentP.Location;

			// Sample experimental bias from actual plotted interpolated points instead if we already have some.
			if (bezierSmoothedLocations.Num() > 1) {
				experimentalBias = bezierSmoothedLocations[bezierSmoothedLocations.Num() - 1] - bezierSmoothedLocations[bezierSmoothedLocations.Num() - 2];
			}
			
			// We plot this bias point at (previous points direction * half of segment distance + current point location)
			const float halfDistance = FVector::Dist(currentP.Location, nextP.Location) / 2.0f;
			experimentalBias.Normalize();
			experimentalBias *= (halfDistance);
			experimentalBias += currentP.Location;

			// Next and nextNext points as well as the next segment's direction
			FVector experimentalBias2 = FAISystem::InvalidLocation;
			FVector nextSegmentDir = FAISystem::InvalidDirection;
			if(i + 2 < navPathPoints.Num())
			{
				FNavPathPoint nextNextP = navPathPoints[i + 2];
				nextSegmentDir = experimentalBias2 = nextNextP.Location - nextP.Location;
				nextSegmentDir.Normalize();
				experimentalBias2.Normalize();
				experimentalBias2 *= -1;
				experimentalBias2 *= 50.f;
				experimentalBias2 += nextP.Location;

				DrawDebugLine(GetWorld(), nextP.Location, experimentalBias2, FColor::Yellow, true, -1.f, 0, 4.f);
				DrawDebugPoint(GetWorld(), experimentalBias2, 22.f, FColor::Yellow, true, -1.f, 0);
			}
			
			// Direction bias at next point
			FVector experimentalPointOnCurve = FAISystem::IsValidLocation(experimentalBias2) ? GetCubicBezierPoint(0.9, currentP.Location, experimentalBias, experimentalBias2, nextP.Location) : GetBezierPoint(0.9, currentP.Location, experimentalBias, nextP.Location);
			FVector directionBias = nextP.Location - experimentalPointOnCurve;
			directionBias.Normalize();

			// Calculate the angle between directionBias and nextSegmentDir 
			float angle = GetAngleBetweenUnitVectors(directionBias, nextSegmentDir, EAngleUnits::Degrees);

			// Debug angles
			if(DebugStringsComponent)
			{
				DebugStringsComponent->DrawDebugStringAtLocation(FString::SanitizeFloat(angle), FColor::White, 1.f, nextP.Location + FVector(0,0, 50));
			}
			
			FNavLocation projectedBiasPoint;
			if (!navSys->ProjectPointToNavigation(experimentalBias, projectedBiasPoint))
			{
				
			}

			DrawDebugLine(GetWorld(), currentP.Location, experimentalBias, FColor::Red, true, -1.f, 0);
			DrawDebugPoint(GetWorld(), experimentalBias, 16.f, FColor::Red, true, -1.f, 0);
			
			for (float t = 0.0; t <= 1.0; t += 0.1f) {
				FVector pointOnCurve = FAISystem::IsValidLocation(experimentalBias2) ? GetCubicBezierPoint(t, currentP.Location, experimentalBias, experimentalBias2, nextP.Location) : GetBezierPoint(t, currentP.Location, experimentalBias, nextP.Location);
				FNavLocation projectedPointOnCurve;
				
				// Prevent out of navmesh boundaries...
				if (navSys->ProjectPointToNavigation(pointOnCurve, projectedPointOnCurve))
				{
					bezierSmoothedLocations.Emplace(projectedPointOnCurve.Location);
				}
			}
		}

		bezierSmoothedLocations.Emplace(navPathPoints.Last().Location);

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

void AATestingNavigatingActor::DebugDrawNavigationPath(const TArray<FNavPathPoint>& pathPoints, const FColor& color) const
{
	TArray<FVector> navPoints;
	navPoints.Reserve(pathPoints.Num());
	Algo::Transform(pathPoints, navPoints, [](const FNavPathPoint& navPathPoint) { return navPathPoint.Location; });
	DebugDrawNavigationPath(navPoints, color);
}
