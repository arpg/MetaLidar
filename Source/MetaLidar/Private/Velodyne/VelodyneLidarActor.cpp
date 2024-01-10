// Fill out your copyright notice in the Description page of Project Settings.

#include "Velodyne/VelodyneLidarActor.h"

// Sets default values
AVelodyneLidarActor::AVelodyneLidarActor()
{
  // Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
  PrimaryActorTick.bCanEverTick = false;

  LidarComponent = CreateDefaultSubobject<UVelodyneBaseComponent>(TEXT("VelodyneComponent"));
  this->AddOwnedComponent(LidarComponent);
  PublishLidarData = false;
  Frequency = 1/15.0f;
}

// Called when the game starts or when spawned
void AVelodyneLidarActor::BeginPlay()
{
  UE_LOG(LogTemp, Warning, TEXT("AVelodyneLidarActor BeginPlay called"));
  Super::BeginPlay();

  FTimespan ThreadSleepTime = FTimespan::FromMilliseconds(1000);
  FString UniqueThreadName = "LidarThread";

  // ROS Topic

  PointCloudTopic = NewObject<UTopic>(UTopic::StaticClass());
  rosinst = Cast<UROSIntegrationGameInstance>(GetGameInstance());
  PointCloudTopic->Init(rosinst->ROSIntegrationCore, TEXT("/unreal/points"), TEXT("sensor_msgs/PointCloud2"));
  PointCloudTopic->Advertise();

  LidarThread = new LidarThreadProcess(ThreadSleepTime, *UniqueThreadName, this);

  if (LidarThread)
  {
    LidarThread->Init();
    LidarThread->LidarThreadInit();
    UE_LOG(LogTemp, Warning, TEXT("Lidar thread initialized!"));
  }

  GetWorld()->GetTimerManager().SetTimer(TimerHandle_PublishLidar, this, &AVelodyneLidarActor::SetPublishLidar, Frequency, true);

}

void AVelodyneLidarActor::EndPlay(EEndPlayReason::Type Reason)
{
  if (LidarThread)
  {
    LidarThread->LidarThreadShutdown();
    LidarThread->Stop();
  }

  //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  //! Wait here until WorkerThread is verified as having stopped!
  //!
  //! This is a safety feature, that will delay PIE EndPlay or
  //! closing of the game while complex calcs occurring on the
  //! thread have a chance to finish
  //!
  while (!LidarThread->ThreadHasStopped())
  {
    FPlatformProcess::Sleep(0.1);
  }
  //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

  // Do this last
  delete LidarThread;

  Super::EndPlay(Reason);
}

void AVelodyneLidarActor::SetPublishLidar()
{
  PublishLidarData = true;
}

// ! On Thread (not game thread)
// Never stop until finished calculating!
// This would be a verrrrry large hitch if done on game thread!
void AVelodyneLidarActor::LidarThreadTick()
{
  UWorld *World = GetWorld();
  float CurrentGameTime = 0.0f; // Declare outside to have broader scope

  // if (World != nullptr)
  // {
  //   CurrentGameTime = World->GetTimeSeconds();
  //   // UE_LOG(LogTemp, Warning, TEXT("GetWorld() returned not null"));
  // }
  // else
  // {
  //   // UE_LOG(LogTemp, Warning, TEXT("GetWorld() returned nullptr"));
  //   return;
  // }

  //float TimeSinceLastOperation = CurrentGameTime - LastOperationTime;

  //! Make sure to come all the way out of all function routines with this same check
  //! so as to ensure thread exits as quickly as possible, allowing game thread to finish
  //! See EndPlay for more information.
  if (LidarThread && LidarThread->IsThreadPaused())
  {
    return;
  }

  if (PublishLidarData)
  {

    // Generate raycasting data
    LidarComponent->GetScanData();
    LidarComponent->AccumulateMessageData();
    UE_LOG(LogTemp, Log, TEXT("Done Accumlating Data"));

    // Swap buffers in a thread-safe manner

    // Use a mutex or other synchronization mechanism here
    FScopeLock Lock(&SwapMutex);
    LidarComponent->SwapBuffers();

    // Start a new thread for publishing
    AsyncTask(ENamedThreads::AnyBackgroundThreadNormalTask, [this]()
              {
        TSharedPtr<ROSMessages::sensor_msgs::PointCloud2> PointCloudMessage(new ROSMessages::sensor_msgs::PointCloud2());
        LidarComponent->GeneratePointCloud2Msg(PointCloudMessage);
        this->PointCloudTopic->Publish(PointCloudMessage); });

    // LastOperationTime = CurrentGameTime;
    PublishLidarData = false;
  }
  else
  {
    FPlatformProcess::SleepNoStats(0.005f); // Sleep for 10ms
  }
}
