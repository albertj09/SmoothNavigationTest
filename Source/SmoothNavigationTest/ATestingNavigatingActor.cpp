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

		TArray<FNavPathPoint>& navPathPoints = navPath->GetPathPoints();

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

			// First experimental bias 
			FVector experimentalBias = nextP.Location - currentP.Location;
			CalculateFirstBiasPoint(experimentalBias, currentP, nextP, navPathPoints, bezierSmoothedLocations);

			// Second experimental bias. I am sampling the direction vector of the next segment and invert it in order to choose a decent location for the second bias.
			// This algorithm ensures that the angles will not be too sharp since it will curve out slightly before curving into the turning point.
			FVector experimentalBias2 = FAISystem::InvalidLocation;
			if(i + 2 < navPathPoints.Num())
			{
				FNavPathPoint nextNextP = navPathPoints[i + 2];
				FVector nextSegmentDir = nextNextP.Location - nextP.Location;
				nextSegmentDir.Normalize();
				
				FVector currentPToNextNextPDir = nextNextP.Location - currentP.Location;
				currentPToNextNextPDir.Normalize();
				currentPToNextNextPDir *= -1;

				// Tiny offset due to potential precision inaccuracies from nav raycast
				constexpr float tinyOffset = 10.f;
				
				// Attempt to skip nav points in case the angle is too small and the resulting segment from current to skip location is fully on navmesh (EXPERIMENTAL)
				float angle = GetAngleBetweenUnitVectors(currentSegmentDir, nextSegmentDir, EAngleUnits::Degrees);
				if(SmoothPathConfigurator.bNavPointSkipping && angle <= SmoothPathConfigurator.MinAngleSkipThreshold && IsSegmentIsFullyOnNavmesh(currentP.Location, nextNextP.Location + currentPToNextNextPDir * tinyOffset))
				{
					nextP = nextNextP;

					// Recalculate first bias
					CalculateFirstBiasPoint(experimentalBias, currentP, nextP, navPathPoints, bezierSmoothedLocations);

					// Recalculate current direction
					currentSegmentDir = nextP.Location - currentP.Location;
					currentSegmentDir.Normalize();

					// Need to increment the iterator once
					++i;

					// Recalculate next segment dir
					if(i + 2 < navPathPoints.Num())
					{
						nextNextP = navPathPoints[i + 2];
						nextSegmentDir = nextNextP.Location - nextP.Location;
						nextSegmentDir.Normalize();

						// Recalculate current direction
						angle = GetAngleBetweenUnitVectors(currentSegmentDir, nextSegmentDir, EAngleUnits::Degrees);
					}
				}
				
				// Debug angles
				if(SmoothPathConfigurator.bEnableExtraDebugInfo && DebugStringsComponent)
				{
					DebugStringsComponent->DrawDebugStringAtLocation(FString::SanitizeFloat(angle), FColor::White, 1.5f, nextP.Location + FVector(0,0, 50));
				}

				// Determine the second bias position offset based on the angle. Sharper angles usually need a larger offset 
				float distanceOffset = UKismetMathLibrary::MapRangeClamped(angle, 0.f, 110.f, SmoothPathConfigurator.Bias2_MinDistanceOffset, SmoothPathConfigurator.Bias2_MaxDistanceOffset);
				experimentalBias2 = nextSegmentDir;
				experimentalBias2 *= -1;
				experimentalBias2 *= distanceOffset;
				experimentalBias2 += nextP.Location;
			}
			
			// Adjust the first bias in case it's outside of navmesh
			FVector testLocBias1;
			if(!IsSegmentIsFullyOnNavmesh(currentP.Location, experimentalBias, testLocBias1))
			{
				experimentalBias = testLocBias1;
			}
			
			// Adjust the second bias in case it's outside of navmesh
			FVector testLocBias2;
			if(FAISystem::IsValidLocation(experimentalBias2) && !IsSegmentIsFullyOnNavmesh(nextP.Location, experimentalBias2, testLocBias2))
			{
				experimentalBias2 = testLocBias2;
			}
			
			// Apply a little offset to next point. It behaves well with bezier curves where there can be some inconsistencies at key points depending on the bias of the next bezier curve segment.
			// Tiny additional offset because if nav point is perfectly at the angle of navbounds, the nav raycast can fail due to precision
			constexpr float miniOffset = 10.f;
			FVector nextPointLocWithOffset = nextP.Location + currentSegmentDir * (SmoothPathConfigurator.NextPointOffset + miniOffset);
			
			// Adjust the next point offset in case it's outside of navmesh
			FVector testNextPointLocWithOffset;
			if(!IsSegmentIsFullyOnNavmesh(nextP.Location + currentSegmentDir * 5.f, nextPointLocWithOffset, testNextPointLocWithOffset))
			{
				nextP.Location = testNextPointLocWithOffset;
			}
			else
			{
				// Otherwise the vector segment is on the navmesh
				nextP.Location = nextPointLocWithOffset;
			}

			// More debugging
			if(SmoothPathConfigurator.bEnableExtraDebugInfo)
			{
				// Next location
				DrawDebugPoint(GetWorld(), nextP.Location, 22.f, FColor::Green, true, -1.f, 0);

				// Bias 1
				DrawDebugLine(GetWorld(), currentP.Location, experimentalBias, FColor::Red, true, -1.f, 0);
				DrawDebugPoint(GetWorld(), experimentalBias, 22.f, FColor::Red, true, -1.f, 0);

				// Bias 2
				if(FAISystem::IsValidLocation(experimentalBias2))
				{
					DrawDebugLine(GetWorld(), nextP.Location, experimentalBias2, FColor::Yellow, true, -1.f, 0, 4.f);
					DrawDebugPoint(GetWorld(), experimentalBias2, 22.f, FColor::Yellow, true, -1.f, 0);
				}
			}
			
			// Using bezier and cubic bezier curve equations (depending on the access to the data that we have), generate intermediate interpolated location points
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

void AATestingNavigatingActor::GetClosestPointOnNearbyPolys(NavNodeRef originalPoly, const FVector& testPt, FVector& pointOnPoly) const
{
	ensure(RecastNavMesh);
	
	TArray<NavNodeRef> polyNeighbors;
	RecastNavMesh->GetPolyNeighbors(originalPoly, polyNeighbors);
	RecastNavMesh->GetClosestPointOnPoly(originalPoly, testPt, pointOnPoly);
		
	float smallestDistSq = FVector::DistSquared(testPt, pointOnPoly);
	FVector closestPointOnNeighbor;
	for(const NavNodeRef& neighbor : polyNeighbors)
	{
		RecastNavMesh->GetClosestPointOnPoly(neighbor, testPt, closestPointOnNeighbor);
		const float distanceSqr = FVector::DistSquared(testPt, closestPointOnNeighbor);
		if(distanceSqr < smallestDistSq)
		{
			smallestDistSq = distanceSqr;
			pointOnPoly = closestPointOnNeighbor;
		}
	}
}

bool AATestingNavigatingActor::IsSegmentIsFullyOnNavmesh(const FVector& segmentStart, const FVector& segmentEnd, FVector& hitLocation) const
{
	ensure(RecastNavMesh);
	ensure(NavigationData);
	return !RecastNavMesh->Raycast(segmentStart, segmentEnd, hitLocation, NavigationData->GetDefaultQueryFilter());
}

bool AATestingNavigatingActor::IsSegmentIsFullyOnNavmesh(const FVector& segmentStart, const FVector& segmentEnd) const
{
	ensure(RecastNavMesh);
	ensure(NavigationData);
	FVector dummyHitLoc;
	return !RecastNavMesh->Raycast(segmentStart, segmentEnd, dummyHitLoc, NavigationData->GetDefaultQueryFilter());
}

void AATestingNavigatingActor::CalculateFirstBiasPoint(FVector& bias, const FNavPathPoint& currentPoint,const FNavPathPoint& nextPoint, const TArray<FNavPathPoint>& originalPathPoints, const TArray<FVector>& smoothPathPoints) const
{
	bias = nextPoint.Location - currentPoint.Location;
	
	// Sample experimental bias from actual plotted interpolated points instead if we already have some.
	if (smoothPathPoints.Num() > 1) {
		bias = smoothPathPoints[smoothPathPoints.Num() - 1] - smoothPathPoints[smoothPathPoints.Num() - 2];
	}
			
	// We plot this bias point at (previous points direction * distance offset + current point location)
	const float currentSegmentDistanceOffset = FVector::Dist(currentPoint.Location, nextPoint.Location) * SmoothPathConfigurator.Bias1_DistanceScalar;
	bias.Normalize();
	bias *= currentSegmentDistanceOffset;
	bias += currentPoint.Location;
}

