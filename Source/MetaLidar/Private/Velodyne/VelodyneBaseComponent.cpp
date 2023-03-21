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

  // Set-up initial values
  SensorModel          = EModelName::eVLP16;
  SamplingRate         = EFrequency::eSR10;
  ReturnMode           = ELaserReturnMode::eStrongest;
  SensorIP             = FString(TEXT("192.168.1.201"));
  DestinationIP        = FString(TEXT("192.168.1.100"));
  ScanPort             = 2368;
  PositionPort         = 8308;
  EnablePositionSensor = false;
}


// Called when the game starts
void UVelodyneBaseComponent::BeginPlay()
{
  Super::BeginPlay();

  ConfigureVelodyneSensor();
}


// Called every frame
void UVelodyneBaseComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
  Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
}

void UVelodyneBaseComponent::EndPlay(EEndPlayReason::Type Reason)
{
  Super::EndPlay(Reason);
}

void UVelodyneBaseComponent::ConfigureVelodyneSensor()
{
  switch (SensorModel.GetValue()) {
  case 0:{
    float Elevation[] = { 15.f, -1.f, 13.f, -3.f, 11.f, -5.f, 9.f, -7.f, 7.f, -9.f, 5.f, -11.f, 3.f, -13.f, 1.f, -15.f };
    Sensor.ElevationAngle.Append(Elevation, UE_ARRAY_COUNT(Elevation));
    Sensor.NumberLaserEmitter = 32;
    Sensor.NumberDataBlock = 12;
    Sensor.NumberDataChannel = 32;
    Sensor.ModelNumber = 33;
    Sensor.MinRange = 20.f;
    Sensor.MaxRange = 10000.f;
    break;
    }
  case 1:{
    float Elevation[] = { -15.f, 1.f, -13.f, 3.f, -11.f, 5.f, -9.f, 7.f, -7.f, 9.f, -5.f, 11.f, -3.f, 13.f, -1.f, 15.f };
    Sensor.ElevationAngle.Append(Elevation, UE_ARRAY_COUNT(Elevation));
    Sensor.NumberLaserEmitter = 16;
    Sensor.NumberDataBlock = 12;
    Sensor.NumberDataChannel = 32;
    Sensor.ModelNumber = 34;
    Sensor.MinRange = 20.f;
    Sensor.MaxRange = 10000.f;
    break;
    }
  case 2:{
    float Elevation[] = { -15.f, 1.f, -13.f, 3.f, -11.f, 5.f, -9.f, 7.f, -7.f, 9.f, -5.f, 11.f, -3.f, 13.f, -1.f, 15.f };
    Sensor.ElevationAngle.Append(Elevation, UE_ARRAY_COUNT(Elevation));
    Sensor.NumberLaserEmitter = 16;
    Sensor.NumberDataBlock = 12;
    Sensor.NumberDataChannel = 32;
    Sensor.ModelNumber = 34;
    Sensor.MinRange = 20.f;
    Sensor.MaxRange = 10000.f;
    break;
    }
  case 3:{
    float Elevation[] = { -10.f, 0.67f, -8.67f, 2.f, -7.33f, 3.33f, -6.f, 4.67f, -4.67f, 6.f, -3.33f, 7.33f, -2.f, 8.67f, -0.67f, 10.f };
    Sensor.ElevationAngle.Append(Elevation, UE_ARRAY_COUNT(Elevation));
    Sensor.NumberLaserEmitter = 16;
    Sensor.NumberDataBlock = 12;
    Sensor.NumberDataChannel = 32;
    Sensor.ModelNumber = 36;
    Sensor.MinRange = 20.f;
    Sensor.MaxRange = 10000.f;
    break;
    }
  case 4:{
    float Elevation[] = { -25.f, -1.f, -1.667f, -15.639f, -11.31f, 0.f, -0.667f, -8.843f, -7.254f, 0.333f, -0.333f, -6.148f, -5.333f, 1.333f, 0.667f, -4.f,
    -4.667f, 1.667f, 1.f, -3.667f, -3.333f, 3.333f, 2.333f, -2.667f, -3.f, 7.f, 4.667f, -2.333f, -2.f, 15.f, 10.333f, -1.333f};
    float AzimuthOffset[] = { 1.4f, -4.2f, 1.4f, -1.4f, 1.4f, -1.4f, 4.2f, -1.4f, 1.4f, -4.2f, 1.4f, -1.4f, 4.2f, -1.4f, 4.2f, -1.4f,
    1.4f, -4.2f, 1.4f, -4.2f, 4.2f, -1.4f, 1.4f, -1.4f, 1.4f, -1.4f, 1.4f, -4.2f, 4.2f, -1.4f, 1.4f, -1.4f };
    Sensor.ElevationAngle.Append(Elevation, UE_ARRAY_COUNT(Elevation));
    Sensor.AzimuthOffset.Append(AzimuthOffset, UE_ARRAY_COUNT(AzimuthOffset));
    Sensor.NumberLaserEmitter = 32;
    Sensor.NumberDataBlock = 12;
    Sensor.NumberDataChannel = 32;
    Sensor.ModelNumber = 40;
    Sensor.MinRange = 50.f;
    Sensor.MaxRange = 10000.f;
    break;
    }
  case 5:{
    float Elevation[] = { 15.f, -1.f, 13.f, -3.f, 11.f, -5.f, 9.f, -7.f, 7.f, -9.f, 5.f, -11.f, 3.f, -13.f, 1.f, -15.f };
    Sensor.ElevationAngle.Append(Elevation, UE_ARRAY_COUNT(Elevation));
    Sensor.NumberLaserEmitter = 32;
    Sensor.NumberDataBlock = 12;
    Sensor.NumberDataChannel = 32;
    Sensor.ModelNumber = 49;
    Sensor.MinRange = 20.f;
    Sensor.MaxRange = 8000.f;
    break;
    }
  case 6:{
    float Elevation[] = { 15.f, -1.f, 13.f, -3.f, 11.f, -5.f, 9.f, -7.f, 7.f, -9.f, 5.f, -11.f, 3.f, -13.f, 1.f, -15.f };
    Sensor.ElevationAngle.Append(Elevation, UE_ARRAY_COUNT(Elevation));
    Sensor.NumberLaserEmitter = 128;
    Sensor.NumberDataBlock = 12;
    Sensor.NumberDataChannel = 32;
    Sensor.ModelNumber = 161;
    Sensor.MinRange = 20.f;
    Sensor.MaxRange = 10000.f;
    break;
    }
  }

  switch (SamplingRate.GetValue()) {
  case 0:
    Sensor.SamplingRate = 5;
    break;
  case 1:
    Sensor.SamplingRate = 10;
    break;
  case 2:
    Sensor.SamplingRate = 15;
    break;
  case 3:
    Sensor.SamplingRate = 20;
    break;
  default:
    Sensor.SamplingRate = 10;
    break;
  }

  switch (ReturnMode.GetValue()) {
  case 0:
    Sensor.ReturnMode = 55;
    break;
  case 1:
    Sensor.ReturnMode = 56;
    break;
  case 2:
    Sensor.ReturnMode = 57;
    break;
  }

  Sensor.AzimuthResolution = (Sensor.SamplingRate * 0.02);

  // Initialize raycast vector and azimuth vector
  Sensor.AzimuthAngle.Init(90.f, Sensor.NumberDataBlock * Sensor.NumberDataChannel);

  // Initialize packet vector
  Sensor.DataPacket.AddUninitialized(DATA_PACKET_PAYLOAD);
  Sensor.PositionPacket.AddUninitialized(POSITION_PACKET_PAYLOAD);
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

  if (Surface.Compare(TEXT("PM_RetroReflector")) == 0) {
    MaxReflectivity = 200;
    MinReflectivity = 101;
  }
  else if (Surface.Contains(TEXT("PM_Diffuse"), ESearchCase::CaseSensitive)) {
    MaxReflectivity = (uint8)FCString::Atoi(*Surface.RightChop(10));
  }
  else { // DefaultPhysicsMaterial
    MaxReflectivity = 20;
  }

  return (uint8)((MinReflectivity-MaxReflectivity) / (Sensor.MaxRange - Sensor.MinRange) * Distance + MaxReflectivity);
}

void UVelodyneBaseComponent::GetScanData()
{
  // complex collisions: true
  FCollisionQueryParams TraceParams = FCollisionQueryParams(TEXT("LaserTrace"), true, GetOwner());
  TraceParams.bReturnPhysicalMaterial = true;
  TraceParams.bTraceComplex = true;
  //TraceParams.bReturnFaceIndex = true;

  // Get owner's location and rotation
  FVector LidarPosition = this->GetActorLocation();
  FRotator LidarRotation = this->GetActorRotation();

  // Initialize array for raycast result
  Sensor.RecordedHits.Init(FHitResult(ForceInit), Sensor.NumberDataBlock * Sensor.NumberDataChannel);
  Sensor.AzimuthAngle.Init(Sensor.AzimuthAngle[Sensor.AzimuthAngle.Num() - 1] + Sensor.AzimuthResolution , Sensor.NumberDataBlock * Sensor.NumberDataChannel);

  // Calculate batch size for 'ParallelFor' based on workable thread
  const int ThreadNum = FPlatformMisc::NumberOfWorkerThreadsToSpawn();
  const int DivideEnd = FMath::FloorToInt((float)(Sensor.RecordedHits.Num() / ThreadNum));

  ParallelFor(
    //Sensor.RecordedHits.Num(),
    ThreadNum,
    [&](int32 PFIndex)
    {
      int StartAt = PFIndex * DivideEnd;
      if (StartAt >= Sensor.RecordedHits.Num()) {
        return;
      }

      int EndAt = StartAt + DivideEnd;
      if (PFIndex == (ThreadNum - 1)) {
        EndAt = Sensor.RecordedHits.Num();
      }

      for (int32 Index = StartAt; Index < EndAt; ++Index) {
        const float HAngle = (float)((int32)(Index / Sensor.NumberLaserEmitter) * Sensor.AzimuthResolution);
        const float VAngle = (float)Sensor.ElevationAngle[Index % Sensor.NumberLaserEmitter];

        Sensor.AzimuthAngle[Index] += HAngle;
        Sensor.AzimuthAngle[Index] = (Sensor.AzimuthAngle[Index] > 360.0 ? Sensor.AzimuthAngle[Index] - 360.f : Sensor.AzimuthAngle[Index]);

        FRotator LaserRotation(0.f, 0.f, 0.f);
        switch(Sensor.ModelNumber) {
        case 40: // VLP-32C
          LaserRotation.Add(VAngle, Sensor.AzimuthAngle[Index] + Sensor.AzimuthOffset[Index % Sensor.NumberLaserEmitter], 0.f);
          break;
        default:
          LaserRotation.Add(VAngle, Sensor.AzimuthAngle[Index], 0.f);
          break;
        }
        FRotator Rotation = UKismetMathLibrary::ComposeRotators(LaserRotation, LidarRotation);

        FVector BeginPoint = LidarPosition + Sensor.MinRange * UKismetMathLibrary::GetForwardVector(Rotation);
        FVector EndPoint = LidarPosition + Sensor.MaxRange * UKismetMathLibrary::GetForwardVector(Rotation);

        GetWorld()->LineTraceSingleByChannel(
            Sensor.RecordedHits[Index], BeginPoint, EndPoint, ECC_Visibility, TraceParams, FCollisionResponseParams::DefaultResponseParam);
      }
    },
    !SupportMultithread
  );
}

void UVelodyneBaseComponent::GenerateDataPacket(uint32 TimeStamp)
{
  // Packet should be encoded based on Sensor Model & Scanning Mode
  uint8 DataFlag[2];
  uint8 AzimuthData[2];
  uint8 DistanceData[2];
  uint8 IntensityData[1];
  uint8 TailData[6];

  int32 PacketIndex = 0;
  for(int32 Index = 0; Index < Sensor.RecordedHits.Num(); Index++){

    if(Index % (Sensor.NumberDataChannel) == 0){
      // Add data flag
      DataFlag[0] = 0xFF;
      DataFlag[1] = 0xEE;
      FMemory::Memcpy(Sensor.DataPacket.GetData() + PacketIndex, DataFlag, UE_ARRAY_COUNT(DataFlag));
      PacketIndex += UE_ARRAY_COUNT(DataFlag);

      // Azimuth data
      uint16 Azimuth = (uint16)(Sensor.AzimuthAngle[Index] * 100);
      AzimuthData[0] = Azimuth & 0x00FF;
      AzimuthData[1] = (Azimuth & 0xFF00) >> 8;

      FMemory::Memcpy(Sensor.DataPacket.GetData() + PacketIndex, AzimuthData, UE_ARRAY_COUNT(AzimuthData));
      PacketIndex += UE_ARRAY_COUNT(AzimuthData);
    }

    // Range data : converting from cm to mm
    // We should add minimum range to make distance from sensor origin
    uint16 Distance = 0;
    if (Sensor.RecordedHits[Index].bBlockingHit) {
      Distance = ((Sensor.RecordedHits[Index].Distance + Sensor.MinRange) * 10) / 2; // 2mm resolution
      //DrawDebugLine(GetWorld(), Sensor.RecordedHits[Index].TraceStart, Sensor.RecordedHits[Index].TraceEnd, FColor::Green, false, 0.1f, 0, 0.5);
    }
    DistanceData[0] = Distance & 0x00FF;
    DistanceData[1] = (Distance & 0xFF00) >> 8;
    FMemory::Memcpy(Sensor.DataPacket.GetData() + PacketIndex, DistanceData, UE_ARRAY_COUNT(DistanceData));
    PacketIndex += UE_ARRAY_COUNT(DistanceData);

    // Intensity data
    auto PhysMat = Sensor.RecordedHits[Index].PhysMaterial;
    if (PhysMat != nullptr) {
      IntensityData[0] = GetIntensity(*PhysMat->GetName(), (Distance * 2) / 10);
    }
    else {
      IntensityData[0] = 0x00;
    }
    FMemory::Memcpy(Sensor.DataPacket.GetData() + PacketIndex, IntensityData, UE_ARRAY_COUNT(IntensityData));
    PacketIndex += UE_ARRAY_COUNT(IntensityData);
  }

  // Add timestamp (0 ~ 35999) and factory bytes
  TailData[0] = TimeStamp & 0x000000FF;
  TailData[1] = (TimeStamp & 0x0000FF00) >> 8;
  TailData[2] = (TimeStamp & 0x00FF0000) >> 16;
  TailData[3] = (TimeStamp & 0xFF000000) >> 24;
  TailData[4] = Sensor.ReturnMode;
  TailData[5] = Sensor.ModelNumber;
  FMemory::Memcpy(Sensor.DataPacket.GetData() + DATA_PACKET_PAYLOAD - 6, TailData, UE_ARRAY_COUNT(TailData));
  PacketIndex += UE_ARRAY_COUNT(TailData);
}

FVector UVelodyneBaseComponent::GetActorLocation()
{
  return GetOwner()->GetActorLocation();
}

FRotator UVelodyneBaseComponent::GetActorRotation()
{
  return GetOwner()->GetActorRotation();
}