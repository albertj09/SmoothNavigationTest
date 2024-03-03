// Fill out your copyright notice in the Description page of Project Settings.

#include "ATestingNavigatingActor.h"
#include "GoalActor.h"
#include "NavigationSystem.h"
#include "NavMesh/RecastNavMesh.h"
#include "AITypes.h"
#include "DebugStringsComponent.h"
#include "Kismet/KismetMathLibrary.h"

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

	// Path recalculation triggered either by changes to bRecalculateSmoothPath, NavPathDrawType, GoalActor or changes to smooth path configuration
	if ((PropertyChangedEvent.Property != nullptr && PropertyChangedEvent.Property->GetFName() == GET_MEMBER_NAME_CHECKED(AATestingNavigatingActor, bRecalculateSmoothPath)) 
		|| (PropertyChangedEvent.Property != nullptr && PropertyChangedEvent.Property->GetFName() == GET_MEMBER_NAME_CHECKED(AATestingNavigatingActor, NavPathDrawType))
		|| (PropertyChangedEvent.Property != nullptr && PropertyChangedEvent.Property->GetFName() == GET_MEMBER_NAME_CHECKED(AATestingNavigatingActor, GoalActor))
		|| (PropertyChangedEvent.Property != nullptr && PropertyChangedEvent.Property->GetFName() == GET_MEMBER_NAME_CHECKED(FSmoothNavPathConfig, Bias1_DistanceScalar))
		|| (PropertyChangedEvent.Property != nullptr && PropertyChangedEvent.Property->GetFName() == GET_MEMBER_NAME_CHECKED(FSmoothNavPathConfig, Bias2_MaxDistanceOffset))
		|| (PropertyChangedEvent.Property != nullptr && PropertyChangedEvent.Property->GetFName() == GET_MEMBER_NAME_CHECKED(FSmoothNavPathConfig, Bias2_MinDistanceOffset))
		|| (PropertyChangedEvent.Property != nullptr && PropertyChangedEvent.Property->GetFName() == GET_MEMBER_NAME_CHECKED(FSmoothNavPathConfig, bNavPointSkipping))
		|| (PropertyChangedEvent.Property != nullptr && PropertyChangedEvent.Property->GetFName() == GET_MEMBER_NAME_CHECKED(FSmoothNavPathConfig, MinAngleSkipThreshold))
		|| (PropertyChangedEvent.Property != nullptr && PropertyChangedEvent.Property->GetFName() == GET_MEMBER_NAME_CHECKED(FSmoothNavPathConfig, NextPointOffset))
		|| (PropertyChangedEvent.Property != nullptr && PropertyChangedEvent.Property->GetFName() == GET_MEMBER_NAME_CHECKED(FSmoothNavPathConfig, bResetToDefaultConfigValues))
		|| (PropertyChangedEvent.Property != nullptr && PropertyChangedEvent.Property->GetFName() == GET_MEMBER_NAME_CHECKED(FSmoothNavPathConfig, bEnableExtraDebugInfo)))
	{
		// Reset the toggle booleans
		bRecalculateSmoothPath = false;
		if(SmoothPathConfigurator.bResetToDefaultConfigValues)
		{
			SmoothPathConfigurator.ResetToDefaults();
		}

		// Proceed to generate path
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
	NavSystem = FNavigationSystem::GetCurrent<UNavigationSystemV1>(GetWorld());
	if (NavSystem)
	{
		NavigationData = NavSystem->GetDefaultNavDataInstance();
		if (NavigationData && IsValid(GoalActor))
		{
			if(!GoalActor->OnConstructionEvent.IsAlreadyBound(this, &AATestingNavigatingActor::GeneratePath))
			{
				GoalActor->OnConstructionEvent.AddUniqueDynamic(this, &AATestingNavigatingActor::GeneratePath);
			}
			
			// Get the optimal navigation path from the engine
			const FPathFindingResult pathFindingResult = NavSystem->FindPathSync(FPathFindingQuery(this, *NavigationData, GetActorLocation(), GoalActor->GetActorLocation(), UNavigationQueryFilter::GetQueryFilter(*NavigationData, this, NavigationFilterClass), nullptr, UE_BIG_NUMBER, true));
			if (pathFindingResult.IsSuccessful())
			{
				SmoothPath(pathFindingResult.Path);
			}
		}
	}
}

TArray<FVector> AATestingNavigatingActor::SmoothPath(FNavPathSharedPtr path)
{
	if (FNavigationPath* navPath = path.Get()) 
	{
		RecastNavMesh = Cast<ARecastNavMesh>(NavigationData);

		// Redundant really, but paranoia
		ensure(NavSystem);
		ensure(RecastNavMesh);
		ensure(NavigationData);

		TArray<FNavPathPoint> navPathPoints = navPath->GetPathPoints();

		// Flush all previous debug drawing
		FlushPersistentDebugLines(GetWorld());
		if(DebugStringsComponent)
		{
			DebugStringsComponent->ClearDebugText();
		}
		
		// Draw the optimal non smoothed engine path
		DebugDrawNavigationPath(navPathPoints, FColor::Blue);
		
		// Smoothing of the points with a custom algorithm including cubic Bezier interpolation
		TArray<FVector> bezierSmoothedLocations;
		for (int32 i = 0; i < navPathPoints.Num(); i++)
		{
			// We are generating the point from current to next, so the last point is already generated
			if (i + 1 == navPathPoints.Num()) {
				break;
			}
			
			// Experimental bias. We need to start with some sort of curve before we make any adjustments.
			FNavPathPoint currentP = navPathPoints[i];
			if(!bezierSmoothedLocations.IsEmpty())
			{
				currentP.Location = bezierSmoothedLocations[bezierSmoothedLocations.Num() - 1];
			}
			FNavPathPoint nextP = navPathPoints[i + 1];

			//Sample current segment direction
			FVector currentSegmentDir = nextP.Location - currentP.Location;
			currentSegmentDir.Normalize();

			FVector experimentalBias = nextP.Location - currentP.Location;
			// Sample experimental bias from actual plotted interpolated points instead if we already have some.
			if (bezierSmoothedLocations.Num() > 1) {
				experimentalBias = bezierSmoothedLocations[bezierSmoothedLocations.Num() - 1] - bezierSmoothedLocations[bezierSmoothedLocations.Num() - 2];
			}
			
			// We plot this bias point at (previous points direction * half of segment distance + current point location)
			const float currentSegmentDistanceOffset = FVector::Dist(currentP.Location, nextP.Location) * SmoothPathConfigurator.Bias1_DistanceScalar;
			experimentalBias.Normalize();
			experimentalBias *= currentSegmentDistanceOffset;
			experimentalBias += currentP.Location;

			// Second experimental bias. I am sampling the direction vector of the next segment and invert it in order to choose a good location for the second bias.
			// This algorithm ensures that the angles will not be too sharp since it will curve out slightly before curving into the turning point.
			FVector experimentalBias2 = FAISystem::InvalidLocation;
			if(i + 2 < navPathPoints.Num())
			{
				FNavPathPoint nextNextP = navPathPoints[i + 2];
				FVector nextSegmentDir = nextNextP.Location - nextP.Location;
				nextSegmentDir.Normalize();

				// Direction bias at next point. We just sample a hypothetical point with a non cubic bezier equation so we get an approximation of how sharp is the next turn at the end of the curve.
				FVector experimentalPointOnCurve = GetBezierPoint(0.8, currentP.Location, experimentalBias, nextP.Location); 
				FVector directionBias = nextP.Location - experimentalPointOnCurve;
				directionBias.Normalize();

				// Calculate the angle between directionBias and nextSegmentDir 
				float angle = GetAngleBetweenUnitVectors(directionBias, nextSegmentDir, EAngleUnits::Degrees);
				
				// Debug angles
				if(SmoothPathConfigurator.bEnableExtraDebugInfo && DebugStringsComponent)
				{
					DebugStringsComponent->DrawDebugStringAtLocation(FString::SanitizeFloat(angle), FColor::White, 1.f, nextP.Location + FVector(0,0, 50));
				}

				// Determine the second bias position offset based on the angle. Sharper angles usually need a larger offset 
				float distanceOffset = UKismetMathLibrary::MapRangeClamped(angle, 0.f, 110.f, SmoothPathConfigurator.Bias2_MinDistanceOffset, SmoothPathConfigurator.Bias2_MaxDistanceOffset);
				experimentalBias2 = nextSegmentDir;
				experimentalBias2 *= -1;
				experimentalBias2 *= distanceOffset;
				experimentalBias2 += nextP.Location;

				DrawDebugLine(GetWorld(), nextP.Location, experimentalBias2, FColor::Yellow, true, -1.f, 0, 4.f);
				DrawDebugPoint(GetWorld(), experimentalBias2, 22.f, FColor::Yellow, true, -1.f, 0);
			}

			//Skipping in case the angle is too small?
			// if(angle < 5.f)
			// {
			// 	if(i + 2 < navPathPoints.Num())
			// 	{
			// 		FNavPathPoint nextPoint = navPathPoints[i + 2];
			// 		nextP = nextPoint;
			// 		i++;
			// 	}
			// }
			
			// Adjust the first bias in case it's outside of navmesh
			FVector testLocBias1;
			if(!CheckIfSegmentIsFullyOnNavmesh(currentP.Location, experimentalBias, testLocBias1))
			{
				experimentalBias = testLocBias1;
				DrawDebugLine(GetWorld(), currentP.Location, experimentalBias, FColor::Black, true, -1.f, 0, 3.f);
				DrawDebugPoint(GetWorld(), experimentalBias, 25.f, FColor::Black, true, -1.f, 0);
			}
			else
			{
				DrawDebugLine(GetWorld(), currentP.Location, experimentalBias, FColor::Green, true, -1.f, 0, 3.f);
			}
			
			FNavLocation projectedFirstBiasPoint;
			if (!NavSystem->ProjectPointToNavigation(experimentalBias, projectedFirstBiasPoint, FVector(10.f, 10.f, 10.f)))
			{
				FVector closestSafePointToFirstBias;
				FVector locationAlongCurrentSegment = currentP.Location + currentSegmentDir * currentSegmentDistanceOffset;
				RecastNavMesh->GetClosestPointOnPoly(RecastNavMesh->FindNearestPoly(locationAlongCurrentSegment, FVector(10.f,10.f,10.f)), experimentalBias, closestSafePointToFirstBias);
				experimentalBias = closestSafePointToFirstBias;
				DrawDebugPoint(GetWorld(), closestSafePointToFirstBias, 22.f, FColor::Orange, true, -1.f, 0);
			}

			// Adjust the second bias in case it's outside of navmesh
			FNavLocation projectedSecondBiasPoint;
			if(FAISystem::IsValidLocation(experimentalBias2) && !NavSystem->ProjectPointToNavigation(experimentalBias2, projectedSecondBiasPoint, FVector(10.f, 10.f, 10.f)))
			{
				FVector closestSafePointToSecondBias;
				RecastNavMesh->GetClosestPointOnPoly(RecastNavMesh->FindNearestPoly(nextP.Location, FVector(10.f,10.f,10.f)), experimentalBias2, closestSafePointToSecondBias);
				experimentalBias2 = closestSafePointToSecondBias;
				DrawDebugPoint(GetWorld(), closestSafePointToSecondBias, 22.f, FColor::Black, true, -1.f, 0);
			}

			DrawDebugLine(GetWorld(), currentP.Location, experimentalBias, FColor::Red, true, -1.f, 0);
			DrawDebugPoint(GetWorld(), experimentalBias, 16.f, FColor::Red, true, -1.f, 0);

			// Apply a little offset to next point. It behaves well with bezier curves where there can be some inconsistencies at key points depending on the bias of the next bezier curve segment.
			nextP.Location += currentSegmentDir * SmoothPathConfigurator.NextPointOffset;
			
			// Adjust the next point offset in case it's outside of navmesh
			FNavLocation projectedNextPointLocWithOffset;
			if(!NavSystem->ProjectPointToNavigation(nextP.Location, projectedNextPointLocWithOffset, FVector(10.f, 10.f, 10.f)))
			{
				FVector closestSafePointToNextOffsetLoc;
				RecastNavMesh->GetClosestPointOnPoly(RecastNavMesh->FindNearestPoly(nextP.Location, FVector(10.f,10.f,10.f)), nextP.Location, closestSafePointToNextOffsetLoc);
				nextP.Location = closestSafePointToNextOffsetLoc;
			}

			DrawDebugPoint(GetWorld(), nextP.Location, 22.f, FColor::Green, true, -1.f, 0);

			// Using bezier and cubic bezier curve equations, generate intermediate interpolated location points
			for (float t = 0.0; t <= 1.0; t += 0.1f) {
				FVector pointOnCurve = FAISystem::IsValidLocation(experimentalBias2) ?  GetCubicBezierPoint(t, currentP.Location, experimentalBias, experimentalBias2, nextP.Location) : GetBezierPoint(t, currentP.Location, experimentalBias, nextP.Location);
				bezierSmoothedLocations.Emplace(pointOnCurve);
			}
		}

		// Add the very last location to the final array
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

void AATestingNavigatingActor::GetClosestPointOnNearbyPolys(ARecastNavMesh* recastNavMesh, NavNodeRef originalPoly, const FVector& testPt, FVector& pointOnPoly)
{
	if(recastNavMesh)
	{
		TArray<NavNodeRef> polyNeighbors;
		recastNavMesh->GetPolyNeighbors(originalPoly, polyNeighbors);
		recastNavMesh->GetClosestPointOnPoly(originalPoly, testPt, pointOnPoly);
		
		float smallestDistSq = FVector::DistSquared(testPt, pointOnPoly);
		FVector closestPointOnNeighbor;
		for(const NavNodeRef& neighbor : polyNeighbors)
		{
			recastNavMesh->GetClosestPointOnPoly(neighbor, testPt, closestPointOnNeighbor);
			const float distanceSqr = FVector::DistSquared(testPt, closestPointOnNeighbor);
			if(distanceSqr < smallestDistSq)
			{
				smallestDistSq = distanceSqr;
				pointOnPoly = closestPointOnNeighbor;
			}
		}
	}
}

bool AATestingNavigatingActor::CheckIfSegmentIsFullyOnNavmesh(const FVector& segmentStart, const FVector& segmentEnd, FVector& hitLocation) const
{
	ensure(RecastNavMesh);
	return !RecastNavMesh->Raycast(segmentStart, segmentEnd, hitLocation, NavigationData->GetDefaultQueryFilter());
}

