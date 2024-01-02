// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "Kismet/KismetMathLibrary.h"
#include "Physics/PhysicsInterfaceCore.h"
#include "PhysicalMaterials/PhysicalMaterial.h"
#include "ROSIntegration/Classes/RI/Topic.h"
#include "ROSIntegration/Classes/ROSIntegrationGameInstance.h"
#include "ROSIntegration/Public/sensor_msgs/PointCloud2.h"
#include "ROSIntegration/Public/ROSTime.h"
#include <fstream>
#include "VelodyneBaseComponent.generated.h"

class UPhysicalMaterial;
class PacketGenerationTask;

USTRUCT()
struct FOusterLidar
{
  GENERATED_BODY()

public:
  float MinRange;
  float MaxRange;
  uint8 VerticalResolution;
  uint16 HorizontalResolution;
  float FieldOfView;
  TArray<float> AzimuthAngleArray;
  TArray<float> ElevationAngleArray;
  TArray<FHitResult> RecordedHits;
};

UCLASS(ClassGroup = (Custom), meta = (BlueprintSpawnableComponent))
class METALIDAR_API UVelodyneBaseComponent : public UActorComponent
{
  GENERATED_BODY()

public:
  // Sets default values for this component's properties
  UVelodyneBaseComponent();

  void SaveRecordedHitsToCSV(const FString &Filename);

  // UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Velodyne")
  // TEnumAsByte<EModelName> SensorModel;


  FOusterLidar Sensor;

  void GenerateElevationAngleArray();
  void GenerateAzimuthAngleArray();

  /**
   * Get scanning data using trace.
   */
  void GetScanData();

  /**
   * Calculate intensity based on physics material reflectivity
   *
   * @param Surface Name of physical material
   * @param Distance Range from sensor origin
   *
   * @return Intensity Object's intensity calculated from surface reflectivity and distance
   */
  uint8 GetIntensity(const FString Surface, const float Distance) const;

  // Publishes a ROS PointCloud2 message
  void GeneratePointCloud2Msg(TSharedPtr<ROSMessages::sensor_msgs::PointCloud2> OutMsgPtr);

  void AccumulateMessageData();

  void SwapBuffers();

  /**
   * Get current location of Actor.
   */
  FVector GetActorLocation();

  /**
   * Get current rotation of Actor.
   */
  FRotator GetActorRotation();

  /**
   * Get current time of game.
   */
  uint32 GetTimestampMicroseconds();

protected:
  // Called when the game starts
  virtual void BeginPlay() override;

  // Called when the game end
  virtual void EndPlay(EEndPlayReason::Type Reason) override;

  TArray<uint8> Buffer1;
  TArray<uint8> Buffer2;
  TArray<uint8> *CurrentBuffer;
  TArray<uint8> *PublishBuffer;

public:
  // Called every frame
  virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction *ThisTickFunction) override;
  int PointStep;  // Point Step in ros message

private:
  bool SupportMultithread = true;

  void ConfigureVelodyneSensor();
};
