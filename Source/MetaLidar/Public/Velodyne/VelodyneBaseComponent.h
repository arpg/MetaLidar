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

#define FIRING_CYCLE 55296 // nanosecond
#define PACKET_HEADER_SIZE 42
#define DATA_PACKET_PAYLOAD 1206
#define POSITION_PACKET_PAYLOAD 512

UENUM(BlueprintType)
enum EModelName
{
  VLP16 UMETA(DisplayName = "VLP-16"),
  PUCK_LITE UMETA(DisplayName = "Puck-Lite"),
  PUCK_HIRES UMETA(DisplayName = "Puck-HiRes"),
  VLP_32C UMETA(DisplayName = "VLP-32C")
  // VELARRAY   UMETA(DisplayName = "Velarray"),
  // VLS_128    UMETA(DisplayName = "VLS-128")
  // HDL_32     UMETA(DisplayName = "HDL-32"),
};

UENUM(BlueprintType)
enum EFrequency
{
  SR05 UMETA(DisplayName = "5 Hz"),
  SR10 UMETA(DisplayName = "10 Hz"),
  SR15 UMETA(DisplayName = "15 Hz"),
  SR20 UMETA(DisplayName = "20 Hz")
};

UENUM(BlueprintType)
enum ELaserReturnMode
{
  Strongest UMETA(DisplayName = "Strongest"),
  // LastReturn UMETA(DisplayName = "Last Return"),
  // DualReturn UMETA(DisplayName = "Dual Return")
};

USTRUCT()
struct FVelodyneLidar
{
  GENERATED_BODY()

public:
  uint8 NumberLaserEmitter;
  uint8 NumberDataBlock;
  uint8 NumberDataChannel;
  uint8 SamplingRate;
  uint8 ReturnMode;
  uint8 ModelNumber;
  float MinRange;
  float MaxRange;
  float AzimuthResolution;
  TArray<float> ElevationAngle;
  TArray<float> AzimuthOffset;
  TArray<float> AzimuthAngle;
  TArray<uint8> DataPacket;
  TArray<uint8> PositionPacket;
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

  UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Velodyne")
  TEnumAsByte<EModelName> SensorModel;

  UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Velodyne")
  TEnumAsByte<EFrequency> SamplingRate;

  UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Velodyne")
  TEnumAsByte<ELaserReturnMode> ReturnMode;

  UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Velodyne")
  FString SensorIP;

  UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Velodyne")
  FString DestinationIP;

  UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Velodyne")
  int32 ScanPort;

  UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Velodyne")
  int32 PositionPort;

  FVelodyneLidar Sensor;

  /**
   * Get scanning data using trace.
   */
  void GetScanData();

  /**
   * Generate velodyne packet data using raycast results.
   *
   * @param TimeStamp current time of game in microseconds
   */
  void GenerateDataPacket(uint32 TimeStamp);

  /**
   * Generate position packet data based on GNSS measurement.
   * !! Not implemented yet !!
   *
   * @param TimeStamp current time of game in microseconds
   */
  void GeneratePositionPacket(uint32 TimeStamp);

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
  void GeneratePointCloud2Msg(TSharedPtr<ROSMessages::sensor_msgs::PointCloud2> OutMsgPtr, TArray<uint8> &data);

  void SaveDataToHexFile(const TArray<uint8> &data, const FString &filename);

  void AccumulateMessageData(int32 BlockNum, int PointStep);
  
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

  /**
   * Convert decimal to hexadecimal.
   */
  FString DecToHex(int DecimalNumber);

  /**
   * Convert ASCII to HEX.
   */
  void ASCIItoHEX(FString Ascii, uint8 Hex[]);

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

private:
  bool SupportMultithread = true;

  void ConfigureVelodyneSensor();
};
