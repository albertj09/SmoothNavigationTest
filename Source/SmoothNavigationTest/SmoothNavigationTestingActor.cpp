#include "SmoothNavigationTestingActor.h"
#include "NavigationSystem.h"
#include "NavigationData.h"

#if WITH_EDITOR
void ASmoothNavigationTestingActor::PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent)
{
	Super::PostEditChangeProperty(PropertyChangedEvent);

	if (PropertyChangedEvent.Property != nullptr && PropertyChangedEvent.Property->GetFName() == GET_MEMBER_NAME_CHECKED(ASmoothNavigationTestingActor, bRecalculateSmoothPath))
	{
		if (bRecalculateSmoothPath)
		{
			// Get the optimal navigation path from the engine
			if (UNavigationSystemV1* navSys = FNavigationSystem::GetCurrent<UNavigationSystemV1>(GetWorld()))
			{
				// Find the path
				FPathFindingResult pathFindingResult = navSys->FindPathSync(BuildPathFindingQuery(OtherActor));

				if (pathFindingResult.IsSuccessful())
				{
					SmoothPath(pathFindingResult.Path);
				}
			}

			// Toggle the bool back to false
			bRecalculateSmoothPath = false;
		}
	}
}
#endif

TArray<FVector> ASmoothNavigationTestingActor::SmoothPath(FNavPathSharedPtr Path)
{
	TArray<FNavPathPoint> points = Path.Get()->GetPathPoints();
	GEngine->AddOnScreenDebugMessage(-1, 5.0f, FColor::Cyan, TEXT("SmoothPathRecalculation..."));
	return TArray<FVector>();
}