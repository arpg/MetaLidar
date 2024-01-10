// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Velodyne/VelodyneBaseComponent.h"
#include "LidarBaseActor.h"
#include "ROSIntegration/Classes/RI/Topic.h"
#include "ROSIntegration/Classes/ROSIntegrationGameInstance.h"
#include "ROSIntegration/Public/sensor_msgs/PointCloud2.h"
#include "HAL/ThreadingBase.h" // Include this for FMutex
#include "VelodyneLidarActor.generated.h"

/**
 *
 */
UCLASS()
class METALIDAR_API AVelodyneLidarActor : public ALidarBaseActor
{
  GENERATED_BODY()

public:
  // Sets default values for this actor's properties
  AVelodyneLidarActor();

  UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MetaLidar")
  class UVelodyneBaseComponent *LidarComponent;
  UPROPERTY();
  UTopic *PointCloudTopic;
  UROSIntegrationGameInstance *rosinst;

protected:
  // Called when the game starts or when spawned
  virtual void BeginPlay() override;

  // Called when the game end
  virtual void EndPlay(EEndPlayReason::Type Reason) override;

  void SetPublishLidar();

  // Publish lidar data (Triggered by Timer)
  bool PublishLidarData;
  float Frequency;
  FCriticalSection SwapMutex;
  FTimerHandle TimerHandle_PublishLidar;

public:
  /**
   * Set UDP communication parameters for scan data
   */
  // virtual void ConfigureUDPScan() override;

  /**
   * Main routine
   * calculate raytracing and generate LiDAR packet data
   */
  virtual void LidarThreadTick() override;
};
