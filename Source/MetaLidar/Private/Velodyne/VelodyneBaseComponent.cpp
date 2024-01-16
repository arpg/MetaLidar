// Fill out your copyright notice in the Description page of Project Settings.

#include "Velodyne/VelodyneBaseComponent.h"

// Sets default values for this component's properties
UVelodyneBaseComponent::UVelodyneBaseComponent()
{
  // Set this component to be initialized when the game starts, and to be ticked every frame.  You can turn these features
  // off to improve performance if you don't need them.
  PrimaryComponentTick.bCanEverTick = false;

  // Check Platform supports multithreading
  SupportMultithread = FPlatformProcess::SupportsMultithreading();
  Sensor.FieldOfView = 33.2;
  Sensor.HorizontalResolution = 1024;
  Sensor.VerticalResolution = 64;
  Sensor.MinRange = 0.8f * 100;  // Convert from meters to centimeters
  Sensor.MaxRange = 50.0f * 100; // Convert from meters to centimeters

  FrameName = TEXT("horiz/ouster");
  PointStep = 13; // 4 bytes each for X, Y, Z, and 1 byte for intensity

  UE_LOG(LogTemp, Display, TEXT("AVelodyneLidarActor Constructor called"));
}

// Called when the game starts
void UVelodyneBaseComponent::BeginPlay()
{
  Super::BeginPlay();

  ConfigureVelodyneSensor();
}

// Called every frame
void UVelodyneBaseComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction *ThisTickFunction)
{
  Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
}

void UVelodyneBaseComponent::EndPlay(EEndPlayReason::Type Reason)
{
  Super::EndPlay(Reason);
}

void UVelodyneBaseComponent::ConfigureVelodyneSensor()
{
  // Generate Angles for raycasting
  GenerateAzimuthAngleArray();
  GenerateElevationAngleArray();

  // Initalize Double Buffer for Ros
  int SizeOfOneHit = sizeof(float) * 3 + sizeof(uint8); // For x,y,x (floats) intensity(unit8)
  int MaxNumberOfHits = Sensor.AzimuthAngleArray.Num() * Sensor.ElevationAngleArray.Num();
  int TotalBufferSize = MaxNumberOfHits * PointStep;

  // UE_LOG(LogTemp, Log, TEXT("Size Of One Azimuth Array: %d, Size of Elevation Array %d"), Sensor.AzimuthAngleArray.Num(), Sensor.ElevationAngleArray.Num());

  // UE_LOG(LogTemp, Log, TEXT("Size Of One Hit: %d bytes"), SizeOfOneHit);
  // UE_LOG(LogTemp, Log, TEXT("Max Number Of Hits: %d"), MaxNumberOfHits);
  // UE_LOG(LogTemp, Log, TEXT("Total Buffer Size: %d bytes"), TotalBufferSize);

  // Initialize Dual Buffers for Publshing Message on a seperate thread
  Buffer1.SetNumUninitialized(TotalBufferSize);
  Buffer2.SetNumUninitialized(TotalBufferSize);

  CurrentBuffer = &Buffer1;
  PublishBuffer = &Buffer2;

  // UE_LOG(LogTemp, Log, TEXT("Buffer1 Capacity: %d"), Buffer1.Max());
  // UE_LOG(LogTemp, Log, TEXT("Buffer2 Capacity: %d"), Buffer2.Max());

  // Log the size of one hit and total buffer size
  // UE_LOG(LogTemp, Log, TEXT("Size Of One Hit: %d bytes"), SizeOfOneHit);
  // UE_LOG(LogTemp, Log, TEXT("Total Buffer Size: %d bytes"), TotalBufferSize);
}

/**
 * Generates an array of elevation angles based on the sensor's field of view and vertical resolution.
 */
void UVelodyneBaseComponent::GenerateElevationAngleArray()
{
  float StartAngle = -Sensor.FieldOfView / 2.0f;
  float EndAngle = Sensor.FieldOfView / 2.0f;
  Sensor.ElevationAngleArray.SetNumUninitialized(Sensor.VerticalResolution);

  float VerticalIncrement = Sensor.FieldOfView / Sensor.VerticalResolution;

  UE_LOG(LogTemp, Log, TEXT("Start Angle: %f"), StartAngle);
  UE_LOG(LogTemp, Log, TEXT("End Angle: %f"), EndAngle);

  float angle = StartAngle;
  for (int i = 0; i < Sensor.VerticalResolution; ++i)
  {
    Sensor.ElevationAngleArray[i] = angle;
    angle += VerticalIncrement;
    UE_LOG(LogTemp, Log, TEXT("Elevation Angle: %f"), angle);
  }
}

/**
 * Generates an array of azimuth angles for the Velodyne sensor.
 * The azimuth angles are evenly distributed around a full circle (360 degrees)
 * based on the specified horizontal resolution of the sensor.
 */
void UVelodyneBaseComponent::GenerateAzimuthAngleArray()
{

  Sensor.AzimuthAngleArray.SetNumUninitialized(Sensor.HorizontalResolution);
  // Increments 360
  float HorizontalIncrement = 360.0f / Sensor.HorizontalResolution;

  for (int i = 0; i < Sensor.HorizontalResolution; ++i)
  {
    float angle = i * HorizontalIncrement;
    Sensor.AzimuthAngleArray[i] = angle;
  }
}

// The VLP-16 measures reflectivity of an object independent of laser power and distances involved. Reflectivity values
// returned are based on laser calibration against NIST-calibrated reflectivity reference targets at the factory.
//
// For each laser measurement, a reflectivity byte is returned in addition to distance. Reflectivity byte values are segmented
// into two ranges, allowing software to distinguish diffuse reflectors (e.g. tree trunks, clothing) in the low range from
// retroreflectors (e.g. road signs, license plates) in the high range.
//
// A retroreflector reflects light back to its source with a minimum of scattering. The VLP-16 provides its own light, with negligible
// separation between transmitting laser and receiving detector, so retroreflecting surfaces pop with reflected IR light
// compared to diffuse reflectors that tend to scatter reflected energy.
//
//   - Diffuse reflectors report values from 0 to 100 for reflectivities from 0% to 100%.
//   - Retroreflectors report values from 101 to 255, where 255 represents an idealreflection.
//
// Note: When a laser pulse doesn't result in a measurement, such as when a laser is shot skyward, both distance and
//        reflectivity values will be 0. The key is a distance of 0, because 0 is a valid reflectivity value (i.e. one step above noise).
uint8 UVelodyneBaseComponent::GetIntensity(const FString Surface, const float Distance) const
{
  uint8 MaxReflectivity = 0;
  uint8 MinReflectivity = 0;

  if (Surface.Contains(TEXT("PM_Reflectivity_"), ESearchCase::CaseSensitive))
  {
    // https://docs.unrealengine.com/5.0/en-US/API/Runtime/Core/Containers/FString/RightChop/1/
    MaxReflectivity = (uint8)FCString::Atoi(*Surface.RightChop(16));
    if (MaxReflectivity > 100)
    {
      MinReflectivity = 101;
    }
  }
  else
  { // Default PhysicalMaterial value, in case of the PhysicalMaterial is not applied
    MaxReflectivity = 20;
  }

  return (uint8)((MinReflectivity - MaxReflectivity) / (Sensor.MaxRange - Sensor.MinRange) * Distance + MaxReflectivity);
}

void UVelodyneBaseComponent::GetScanData()
{
  // complex collisions: true
  FCollisionQueryParams TraceParams = FCollisionQueryParams(TEXT("LaserTrace"), true, GetOwner());
  TraceParams.bReturnPhysicalMaterial = true;
  TraceParams.bTraceComplex = true;

  // Get owner's location and rotation
  FVector LidarPosition = this->GetActorLocation();
  FRotator LidarRotation = this->GetActorRotation();
  // UE_LOG(LogTemp, Log, TEXT("Lidar Position: %s, Lidar Rotation: %s"), *LidarPosition.ToString(), *LidarRotation.ToString());

  // Initialize RecordedHits with total number of hits
  int TotalHits = Sensor.ElevationAngleArray.Num() * Sensor.AzimuthAngleArray.Num();
  Sensor.RecordedHits.Init(FHitResult(ForceInit), TotalHits);

  // Get the transform and log it
  FTransform OriginalTransform = GetOwner()->GetTransform();
  OriginalTransform.SetScale3D(FVector(1.0f, 1.0f, 1.0f));

  // Modify the rotation here: Flip roll and yaw by 180 degrees
  FRotator NewRotation = FRotator(0.0f, LidarRotation.Yaw - 180.0f, LidarRotation.Roll - 180.0f); 
  OriginalTransform.SetRotation(FQuat(NewRotation));

  FTransform LidarInverseTransform = OriginalTransform.Inverse();
  UE_LOG(LogTemp, Log, TEXT("Inverted Transform: %s"), *LidarInverseTransform.ToString());

  // UE_LOG(LogTemp, Log, TEXT("Min Range %f, Max Range %f"), Sensor.MinRange, Sensor.MaxRange);

  // Calculate batch size for 'ParallelFor' based on workable thread
  const int ThreadNum = FPlatformMisc::NumberOfWorkerThreadsToSpawn();
  const int DivideEnd = FMath::FloorToInt((float)(TotalHits / ThreadNum));

  ParallelFor(
      ThreadNum,
      [&](int32 PFIndex)
      {
        int StartAt = PFIndex * DivideEnd;
        if (StartAt >= Sensor.RecordedHits.Num())
        {
          return;
        }

        int EndAt = StartAt + DivideEnd;
        if (PFIndex == (ThreadNum - 1))
        {
          EndAt = Sensor.RecordedHits.Num();
        }

        for (int32 Index = StartAt; Index < EndAt; ++Index)
        {

          int HAngleIndex = Index / Sensor.ElevationAngleArray.Num();
          int VAngleIndex = Index % Sensor.ElevationAngleArray.Num();

          float HAngle = Sensor.AzimuthAngleArray[HAngleIndex];
          float VAngle = Sensor.ElevationAngleArray[VAngleIndex];

          FRotator LaserRotation(0.f, 0.f, 0.f);

          LaserRotation.Add(VAngle, HAngle, 0.f);
          FRotator Rotation = UKismetMathLibrary::ComposeRotators(LaserRotation, LidarRotation);

          FVector BeginPoint = LidarPosition + Sensor.MinRange * UKismetMathLibrary::GetForwardVector(Rotation);
          FVector EndPoint = LidarPosition + Sensor.MaxRange * UKismetMathLibrary::GetForwardVector(Rotation);

          FHitResult HitLocation;
          // GetWorld()->LineTraceSingleByChannel(
          //     HitLocation, BeginPoint, EndPoint, ECC_Visibility, TraceParams, FCollisionResponseParams::DefaultResponseParam);
          // FVector InverseHitLocation = LidarInverseTransform.TransformPosition(HitLocation.Location);

          GetWorld()->LineTraceSingleByChannel(
              Sensor.RecordedHits[Index], BeginPoint, EndPoint, ECC_Visibility, TraceParams, FCollisionResponseParams::DefaultResponseParam);

          Sensor.RecordedHits[Index].ImpactPoint = LidarInverseTransform.TransformPosition(Sensor.RecordedHits[Index].ImpactPoint);
          // Sensor.RecordedHits[Index] = HitLocation;
          // Sensor.RecordedHits[Index].ImpactPoint = InverseHitLocation;

          // UE_LOG(LogTemp, Log, TEXT("Hit at index %d: Original Hit Location: X=%f, Y=%f, Z=%f | Transformed Hit Location: X=%f, Y=%f, Z=%f"),
          //        Index, HitLocation.Location.X, HitLocation.Location.Y, HitLocation.Location.Z, InverseHitLocation.X, InverseHitLocation.Y, InverseHitLocation.Z);

          // UE_LOG(LogTemp, Log, TEXT("Hit at index %d: Location in Vector: X=%f, Y=%f, Z=%f" ),
          //        Index, Sensor.RecordedHits[Index].Location.X, Sensor.RecordedHits[Index].Location.Y,Sensor.RecordedHits[Index].Location.Z);

          // FVector HitLocation = Sensor.RecordedHits[Index].Location;

          // UE_LOG(LogTemp, Log, TEXT("Hit at index %d: Start X=%f, Y=%f, Z=%f | End X=%f, Y=%f, Z=%f | Hit X=%f, Y=%f, Z=%f"),
          //        Index, BeginPoint.X, BeginPoint.Y, BeginPoint.Z, EndPoint.X, EndPoint.Y, EndPoint.Z, HitLocation.X, HitLocation.Y, HitLocation.Z);
        }
      },
      !SupportMultithread);
}

FVector UVelodyneBaseComponent::GetActorLocation()
{
  return GetOwner()->GetActorLocation();
}

FRotator UVelodyneBaseComponent::GetActorRotation()
{
  return GetOwner()->GetActorRotation();
}

uint32 UVelodyneBaseComponent::GetTimestampMicroseconds()
{
  return (uint32)(fmod(GetWorld()->GetTimeSeconds(), 3600.f) * 1000000); // sec -> microsec
}

/**
 * Accumulates message data from the sensor's recorded hits for ros.
 * Converts the hit points from centimeters to meters and reorders the coordinates.
 * Copies the converted hit points and intensity data to the current buffer.
 * If the index is out of bounds, the function breaks out of the loop to prevent out-of-bounds access.
 */
void UVelodyneBaseComponent::AccumulateMessageData()
{
  UE_LOG(LogTemp, Log, TEXT("AccumulateMessageData, RecordedHits: %d, PointStep: %d"), Sensor.RecordedHits.Num(), PointStep);
  for (int32 i = 0; i < Sensor.RecordedHits.Num(); i++)
  {
    FVector hitPointCM = Sensor.RecordedHits[i].ImpactPoint; // Assuming ImpactPoint is the XYZ coordinate
    FVector hitPoint;
    hitPoint.X = hitPointCM.X * 0.01f;
    hitPoint.Y = hitPointCM.Y * -0.01f;
    hitPoint.Z = hitPointCM.Z * 0.01f;

    // Convert double to float before copying
    float X = static_cast<float>(hitPoint.X);
    float Y = static_cast<float>(hitPoint.Y);
    float Z = static_cast<float>(hitPoint.Z);

    // UE_LOG(LogTemp, Display, TEXT("Hit at index %d: Location X=%f, Y=%f, Z=%f"), i, X, Y, Z);

    auto PhysMat = Sensor.RecordedHits[i].PhysMaterial;
    uint8 intensity = 0x00;
    if (PhysMat != nullptr)
    {
      intensity = GetIntensity(*PhysMat->GetName(), (Sensor.RecordedHits[i].Distance * 2) / 10);
    }
    int32 idx = i * PointStep;

    if (idx + PointStep <= CurrentBuffer->Max())
    {

      FMemory::Memcpy(CurrentBuffer->GetData() + idx, &X, sizeof(float));
      FMemory::Memcpy(CurrentBuffer->GetData() + idx + 4, &Y, sizeof(float));
      FMemory::Memcpy(CurrentBuffer->GetData() + idx + 8, &Z, sizeof(float));

      // Copy intensity data
      (*CurrentBuffer)[idx + 12] = intensity;
    }
    else
    {
      UE_LOG(LogTemp, Error, TEXT("Index out of bounds for hit %d, idx: %d, Buffer Max Size: %d"), i, idx, CurrentBuffer->Max());
      break; // Break out of the loop to prevent out-of-bounds access
    }
  }
}

/**
 * Generates a PointCloud2 message from the recorded hits of the Velodyne sensor.
 *
 * @param OutMsgPtr A shared pointer to the PointCloud2 message to be generated.
 */
void UVelodyneBaseComponent::GeneratePointCloud2Msg(TSharedPtr<ROSMessages::sensor_msgs::PointCloud2> OutMsgPtr)
{
  UE_LOG(LogTemp, Log, TEXT("GeneratePointCloud2Msg - Function Start"));

  if (!OutMsgPtr.IsValid())
  {
    UE_LOG(LogTemp, Error, TEXT("GeneratePointCloud2Msg - OutMsgPtr is invalid"));
    return; // Early exit if pointer is not valid
  }

  // Initialize PointCloud2 Message
  OutMsgPtr->header.frame_id = FrameName;
  OutMsgPtr->header.time = FROSTime::Now(); // Use your method to get the current time

  UE_LOG(LogTemp, Log, TEXT("Initialized PointCloud2 Message with frame_id=%s"), *OutMsgPtr->header.frame_id);

  // Define the point fields for XYZ and intensity
  TArray<ROSMessages::sensor_msgs::PointCloud2::PointField> fields;
  fields.Init(ROSMessages::sensor_msgs::PointCloud2::PointField(), 4);

  fields[0].name = "x";
  fields[0].offset = 0;
  fields[0].datatype = ROSMessages::sensor_msgs::PointCloud2::PointField::FLOAT32;
  fields[0].count = 1;

  fields[1].name = "y";
  fields[1].offset = 4;
  fields[1].datatype = ROSMessages::sensor_msgs::PointCloud2::PointField::FLOAT32;
  fields[1].count = 1;

  fields[2].name = "z";
  fields[2].offset = 8;
  fields[2].datatype = ROSMessages::sensor_msgs::PointCloud2::PointField::FLOAT32;
  fields[2].count = 1;

  fields[3].name = "intensity";
  fields[3].offset = 12;
  fields[3].datatype = ROSMessages::sensor_msgs::PointCloud2::PointField::UINT8;
  fields[3].count = 1;

  OutMsgPtr->fields = fields;

  UE_LOG(LogTemp, Log, TEXT("Initialized PointCloud2 fields"));

  // Set message properties
  OutMsgPtr->is_bigendian = false;
  OutMsgPtr->point_step = PointStep; // 4 bytes each for X, Y, Z, and 1 byte for intensity
  OutMsgPtr->row_step = OutMsgPtr->point_step * Sensor.RecordedHits.Num();
  OutMsgPtr->height = 1;
  OutMsgPtr->width = Sensor.RecordedHits.Num();
  OutMsgPtr->is_dense = true; // Assuming no invalid (NaN) points

  UE_LOG(LogTemp, Log, TEXT("Set message properties: is_bigendian=%s, point_step=%d, row_step=%d, height=%d, width=%d, is_dense=%s"),
         OutMsgPtr->is_bigendian ? TEXT("True") : TEXT("False"),
         OutMsgPtr->point_step,
         OutMsgPtr->row_step,
         OutMsgPtr->height,
         OutMsgPtr->width,
         OutMsgPtr->is_dense ? TEXT("True") : TEXT("False"));

  OutMsgPtr->data_ptr = PublishBuffer->GetData();
}

/**
 * Swaps the current buffer with the publish buffer.
 * This function is used to update the buffer that is being published.
 */
void UVelodyneBaseComponent::SwapBuffers()
{
  TArray<uint8> *Temp = CurrentBuffer;
  CurrentBuffer = PublishBuffer;
  PublishBuffer = Temp;
}
